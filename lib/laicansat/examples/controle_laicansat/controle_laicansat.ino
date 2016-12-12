/********************************************************
 *Marina Andrade 
 *Controle PID para plataforma Laicansat III
 *03/10/16
 ********************************************************/

#include <SD.h>
#include <SD_t3.h>
#include <SPI.h>
#include <UBX_Parser.h>



HardwareSerial gpsSerial(Serial1);

byte gps_set_sucess = 0 ;
File dataFile;
const int chipSelect = 20; 

double erro = 0;
double lastSample = 0;
long lastProcess;
double pid;
double sinal_de_controle;

double 
	 kP = -0.335721987396887,
	 kI = -0.000360400339747789,
	 kD = 1.30119803967543,
         N  = 0.257196885578689;

double 
	 P = 0,
	 I = 0,
	 D = 0;
double HEADING = 0.0,
      LON     = 0.0,
      LAT     = 0.0;


const int led = 30;




//-----------------------------------------------------------------------
class MyParser : public UBX_Parser {

   void handle_NAV_POSLLH(unsigned long iTOW, 
            long lon, 
            long lat, 
            long height, 
            long hMSL, 
            unsigned long hAcc, 
            unsigned long vAcc) {

        Serial.print("lat=");
        Serial.print(lat/1e7,7);
        LAT = lat;
        Serial.print(" deg lon=");
        Serial.print(lon/1e7,7);
        Serial.print(" deg");
        LON = lon;
       
    }  


       virtual void handle_NAV_VELNED(unsigned long iTOW,
            long velN,
            long velE,
            long velD,
            unsigned long speed,
            unsigned long gSpeed,
            long heading,
            unsigned long sAcc,
            unsigned long cAcc)
    {
        
        
       // Serial.print("cm/s gSpeed=");
        //Serial.print(gSpeed);
        Serial.print("  heading= ");
        Serial.print(heading/1e5, 5);
        Serial.println(" deg");
       HEADING = heading;
     }

   
    
    
    /* Un-comment this to report IDs of messages available but not yet handled.
    void reportUnhandled(char msgid) {
        Serial.print("Got message ID ");
        Serial.println(msgid&0xFF, HEX);
    }*/
    
};
MyParser parser;



//---------------------------------------------------------------------------------------------------------------------


void setup()
{
    
  SPI.setMOSI(7);
  SPI.setMISO(8);
  SPI.setSCK(14);
  
  Serial1.begin(9600);
  pinMode(led,OUTPUT);
  Serial.begin(9600);
  delay(1000);
  
  Serial.print("Initializing SD card...");

if (!SD.begin(chipSelect)) {
Serial.println("Falha na Inicializacao");
//while(1);
}


dataFile = SD.open("cntrl.txt", FILE_WRITE);
   if (!dataFile) {
     Serial.println("error opening cntrl.txt");
    // while (1) ;
   }
   


  
  digitalWrite(led, HIGH);
  




 // baud rate 9600
Serial.println("Setting CFG_PRT:");
  uint8_t setCFG_PRT[] = { 
                                                                                              
   0xB5, 0x62,  0x06,0x00,   0x14,0x00,       0x01, 0x00, 0x00, 0x00, 0xC0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x07, 0x00, 0x03, 0x00,
   0x00, 0x00, 0x00, 0x00,
  };
  while(!gps_set_sucess)
  {
    sendUBX(setCFG_PRT ,26);
    gps_set_sucess=getUBX_ACK(setCFG_PRT);
  }
  gps_set_sucess=0; 

 
  //-----------------------------------------------------------------------------
    //dados atualizados em 1Hz
Serial.println("Setting uBlox set CFG-RATE ");
  uint8_t setCFG_RATE[] = { 
    
    0xB5,0x62,  0x06,0x08,  0x06,0x00,     0xE8,0x03,    0x01,0x00,0x01,0x00, 
  
  };
  while(!gps_set_sucess)
  {
    sendUBX(setCFG_RATE, 12);
    gps_set_sucess=getUBX_ACK(setCFG_RATE);
  }
  gps_set_sucess=0;
  
  //-----------------------------------------------------------------------------
  
    //abilita mensagens UBX RXM-RAWX na UART-1
Serial.println("Setting uBlox set RXM_RAWX ");
  uint8_t setRXM_RAWX[] = { 
    
    0xB5,0x62,0x06,0x01,0x08,0x00,    0x02, 0x15,    0x00,0x00,0x00,0x00,0x00,0x00, 
  
  };
  while(!gps_set_sucess)
  {
    sendUBX(setRXM_RAWX, 14);
    gps_set_sucess=getUBX_ACK(setRXM_RAWX);
  }
  gps_set_sucess=0;
  
  //-----------------------------------------------------------------------------
    
     //turn on UBX RXM-SFRBX messages on UART-1 interface with same freq
Serial.println("Setting uBlox set RXM_SFRBX ");
  uint8_t setRXM_SFRBX[] = { 
    
    0xB5,0x62,0x06,0x01,0x08,0x00,    0x02, 0x13,   0x00,0x00,0x00,0x00,0x00,0x00, 
  
  };
  while(!gps_set_sucess)
  {
    sendUBX(setRXM_SFRBX, 14);
    gps_set_sucess=getUBX_ACK(setRXM_SFRBX);
  }
  gps_set_sucess=0; 
  
 //-----------------------------------------------------------------------------
      //turn on UBX TIM TIM2 messages on UART-1 interface with same freq
Serial.println("Setting uBlox set TIM_TIM2 ");
  uint8_t setTIM_TIM2 [] = { 
    
    0xB5,0x62,0x06,0x01,0x08,0x00,    0x0D, 0x03,   0x00,0x01,0x00,0x00,0x00,0x00, 
  
  };
  while(!gps_set_sucess)
  {
    sendUBX(setTIM_TIM2 , 14);
    gps_set_sucess=getUBX_ACK(setTIM_TIM2);
  }
  gps_set_sucess=0; 
  
 //-----------------------------------------------------------------------------

       //GNSS system settings
Serial.println("Setting GPS 8-16 channels CFG_GNSS1");
  uint8_t setCFG_GNSS1[] = { 
    
    0xB5, 0x62, 0x06, 0x3E, 0x0C, 0x00,    0x00, 0x20, 0x20,    0x01,0x00,   0x08, 0x10,   0x00, 0x01, 0x00, 0x01, 0x01,
  
  };
  while(!gps_set_sucess)
  {
    sendUBX(setCFG_GNSS1 , 18);
    gps_set_sucess=getUBX_ACK(setCFG_GNSS1);
  }
  gps_set_sucess=0;  
  
//-----------------------------------------------------------------------------

       //GNSS system settings
Serial.println("Setting SBAS 1-3 channels off CFG_GNSS2");
  uint8_t setCFG_GNSS2[] = { 
    
    0xB5, 0x62,   0x06, 0x3E,0x0C, 0x00,    0x00,0x20,0x20,      0x01,0x01,     0x01,0x03,    0x00, 0x00, 0x00, 0x01, 0x01,
  
  };
  while(!gps_set_sucess)
  {
    sendUBX(setCFG_GNSS2 , 18);
    gps_set_sucess=getUBX_ACK(setCFG_GNSS2);
  }
  gps_set_sucess=0;  
  
//-----------------------------------------------------------------------------

   //GNSS system settings
Serial.println("Setting Galileo 0 channels off CFG_GNSS3");
  uint8_t setCFG_GNSS3[] = { 
    
    0xB5, 0x62,   0x06, 0x3E,0x0C, 0x00,    0x00,0x20,0x20,  0x01,0x02,  0x00, 0x00,   0x00, 0x00, 0x00, 0x00, 0x01,
  
  };
  while(!gps_set_sucess)
  {
    sendUBX(setCFG_GNSS3 , 18);
    gps_set_sucess=getUBX_ACK(setCFG_GNSS3);
  }
  gps_set_sucess=0; 
 
//-----------------------------------------------------------------------------

   
Serial.println("Setting BeiDou 0 channels off CFG_GNSS4");
  uint8_t setCFG_GNSS4[] = { 
    
    0xB5, 0x62,   0x06, 0x3E,0x0C, 0x00,    0x00,0x20,0x20,   0x01, 0x03,      0x00, 0x00,    0x00, 0x00, 0x00, 0x01, 0x01,
  
  };
  while(!gps_set_sucess)
  {
    sendUBX(setCFG_GNSS4 , 18);
    gps_set_sucess=getUBX_ACK(setCFG_GNSS4);
  }
  gps_set_sucess=0;  
   
//-----------------------------------------------------------------------------

      
Serial.println("Setting IMES 0-8 channels off CFG_GNSS5");
  uint8_t setCFG_GNSS5[] = { 
    
    0xB5, 0x62,   0x06, 0x3E,0x0C, 0x00,    0x00, 0x20, 0x20,   0x01, 0x04,   0x00, 0x08,    0x00, 0x00, 0x00, 0x01, 0x01,
  
  };
  while(!gps_set_sucess)
  {
    sendUBX(setCFG_GNSS5 , 18);
    gps_set_sucess=getUBX_ACK(setCFG_GNSS5);
  }
  gps_set_sucess=0;  

//-----------------------------------------------------------------------------

      
Serial.println("Setting off QZSS 0-3 channels CFG_GNSS6");
  uint8_t setCFG_GNSS6[] = { 
    
    0xB5, 0x62,   0x06, 0x3E,0x0C, 0x00,    0x00, 0x20, 0x20,   0x01, 0x05,   0x00, 0x03,     0x00, 0x00, 0x00, 0x01, 0x01,
  
  };
  while(!gps_set_sucess)
  {
    sendUBX(setCFG_GNSS6 , 18);
    gps_set_sucess=getUBX_ACK(setCFG_GNSS6);
  }
  gps_set_sucess=0;  
    
//-----------------------------------------------------------------------------

      
Serial.println("Setting GLONASS 8-14 channels CFG_GNSS7");
  uint8_t setCFG_GNSS7[] = { 
    
    0xB5, 0x62,   0x06, 0x3E, 0x0C, 0x00,    0x00, 0x20, 0x20,   0x01, 0x06,   0x08, 0x0E,    0x00, 0x01, 0x00, 0x01, 0x01,
  
  };
  while(!gps_set_sucess)
  {
    sendUBX(setCFG_GNSS7 , 18);
    gps_set_sucess=getUBX_ACK(setCFG_GNSS7);
  }
  gps_set_sucess=0;  
   
//-------------------------------------------------------------------
// Set NAV5 dynamic model airborne airborne <1g
  Serial.println("Setting uBlox nav5 dynamic model: ");
  uint8_t setNav5[] = {
    /*header*/0xB5, 0x62,/*class & id*/ 0x06, 0x24, /*length*/0x24, 0x00,
    /*payload*/ 
    /*bitmask*/0xFF, 0xFF,
    /*dynamic model*/ 0x06,
    /*fixMode*/ 0x03,
    /*Fixed altitude*/ 0x00, 0x00, 0x00, 0x00,
    /*Fixed altitude variance*/ 0x10, 0x27, 0x00, 0x00,
    /*Minimum Elevation for a GNSS satellite to be used in NAV*/0x05, 
    /*Reserved*/0x00,
    /*Position DOP Mask to use*/ 0xFA, 0x00,
    /*Time DOP Mask to use*/ 0xFA, 0x00, 
    /*Position Accuracy Mask*/0x64, 0x00, 
    /*Time Accuracy Mask*/0x2C, 0x01,
    /*Static hold threshold*/ 0x00,
    /*DGPS timeout.*/ 0x00,
    /*Number of satellites required to have*/ 0x00,
    /*C/N0 threshold for deciding whether to attempt a fix*/ 0x00,
    /*Reserved*/ 0x00, 0x00,
    /*Static hold distance threshold*/ 0x00, 0x00, 
    /*UTC standard to be used:*/0x00,
    /*Reserved*/ 0x00,0x00, 0x00,0x00, 0x00};
  while(!gps_set_sucess)
  {
    sendUBX(setNav5, 42);
    gps_set_sucess=getUBX_ACK(setNav5);
  }
  gps_set_sucess=0;
  

  
  
//-------------------------------------------------------------------------------------
      
Serial.println("Turn off NMEA GGA");
  uint8_t setCFG_MSG1[] = { 
    
    0xB5, 0x62,   0x06, 0x01,0x08, 0x00,    0xF0, 0x00,  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  };
  while(!gps_set_sucess)
  {
    sendUBX(setCFG_MSG1 , 14);
    gps_set_sucess=getUBX_ACK(setCFG_MSG1);
  }
  gps_set_sucess=0;  
  

//-------------------------------------------------------------------------------------
     
Serial.println("Turn off NMEA GLL");
  uint8_t setCFG_MSG2[] = { 
    
    0xB5, 0x62,   0x06, 0x01,0x08, 0x00,    0xF0, 0x01,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  };
  while(!gps_set_sucess)
  {
    sendUBX(setCFG_MSG2 , 14);
    gps_set_sucess=getUBX_ACK(setCFG_MSG2);
  }
  gps_set_sucess=0; 
  
  
//-------------------------------------------------------------------------------------
      
Serial.println("Turn off NMEA GSA");
  uint8_t setCFG_MSG3[] = { 
    
    0xB5, 0x62,   0x06, 0x01,0x08, 0x00,    0xF0, 0x02,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  };
  while(!gps_set_sucess)
  {
    sendUBX(setCFG_MSG3 , 14);
    gps_set_sucess=getUBX_ACK(setCFG_MSG3);
  }
  gps_set_sucess=0; 
     
 //-------------------------------------------------------------------------------------
      
Serial.println("Turn off NMEA GSV");
  uint8_t setCFG_MSG4[] = { 
    
    0xB5, 0x62,   0x06, 0x01,0x08,0x00,    0xF0, 0x03,     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  };
  while(!gps_set_sucess)
  {
    sendUBX(setCFG_MSG4 , 14);
    gps_set_sucess=getUBX_ACK(setCFG_MSG4);
  }
  gps_set_sucess=0; 
     
 //-------------------------------------------------------------------------------------
      
Serial.println("Turn off NMEA RMC");
  uint8_t setCFG_MSG5[] = { 
    
    0xB5, 0x62,   0x06, 0x01,0x08, 0x00,    0xF0, 0x04,     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  };
  while(!gps_set_sucess)
  {
    sendUBX(setCFG_MSG5 , 14);
    gps_set_sucess=getUBX_ACK(setCFG_MSG5);
  }
  gps_set_sucess=0; 
  
 //-------------------------------------------------------------------------------------
      
Serial.println("Turn off NMEA VTG");
  uint8_t setCFG_MSG6[] = { 
    
    0xB5, 0x62,   0x06, 0x01,0x08, 0x00,    0xF0,0x05,    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  };
  while(!gps_set_sucess)
  {
    sendUBX(setCFG_MSG6 , 14);
    gps_set_sucess=getUBX_ACK(setCFG_MSG6);
  }
  gps_set_sucess=0;
  
  
//-------------------------------------------------------------------------------------
      
Serial.println("Turn off NMEA ZDA");
  uint8_t setCFG_MSG7[] = { 
    
    0xB5, 0x62,   0x06, 0x01,0x08, 0x00,    0xF0, 0x08,    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  };
  while(!gps_set_sucess)
  {
    sendUBX(setCFG_MSG7 , 14);
    gps_set_sucess=getUBX_ACK(setCFG_MSG7);
  }
  gps_set_sucess=0;
  
  //-------------------------------------------------------------------------------------
      
Serial.println("setting NAV-STATUS on");
  uint8_t setNAV_STATUS[] = { 
    
    0xB5, 0x62,   0x06,0x01,0x08,0x00,    0x01, 0x03,      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  };
  while(!gps_set_sucess)
  {
    sendUBX(setNAV_STATUS , 14);
    gps_set_sucess=getUBX_ACK(setNAV_STATUS);
  }
  gps_set_sucess=0;
  
  
   //-------------------------------------------------------------------------------------
      
Serial.println("setting NAV-SOL on");
  uint8_t setNAV_SOL[] = { 
    
    0xB5, 0x62,   0x06,0x01,0x08,0x00,    0x01, 0x06,       0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 
  };
  while(!gps_set_sucess)
  {
    sendUBX(setNAV_SOL , 14);
    gps_set_sucess=getUBX_ACK(setNAV_SOL);
  }
  gps_set_sucess=0;
  
   //-------------------------------------------------------------------------------------
      
Serial.println("setting NAV-VELNED on");
  uint8_t setNAV_VELNED[] = { 
    
    0xB5, 0x62,   0x06,0x01,0x08,0x00,    0x01, 0x12,       0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 
  };
  while(!gps_set_sucess)
  {
    sendUBX(setNAV_VELNED, 14);
    gps_set_sucess=getUBX_ACK(setNAV_VELNED);
  }
  gps_set_sucess=0;
  
   //-------------------------------------------------------------------------------------
      
Serial.println("setting NAV-CLOCK off");
  uint8_t setNAV_CLOCK[] = { 
    
    0xB5, 0x62,   0x06,0x01, 0x08,0x00,    0x01, 0x22,      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  };
  while(!gps_set_sucess)
  {
    sendUBX(setNAV_CLOCK, 14);
    gps_set_sucess=getUBX_ACK(setNAV_CLOCK);
  }
  gps_set_sucess=0;
  
   //-------------------------------------------------------------------------------------
      
Serial.println("setting NAV-SVINFO off");
  uint8_t setNAV_SVINFO[] = { 
    
    0xB5,0x62,  0x06,0x01,  0x08,0x00,  0x01,0x30,  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  };
  while(!gps_set_sucess)
  {
    sendUBX(setNAV_SVINFO, 14);
    gps_set_sucess=getUBX_ACK(setNAV_SVINFO);
  }
  gps_set_sucess=0;

Serial.println("setting NAV-POLLSH on");
  uint8_t setNAV_POLLSH[] = { 
    
    0xB5,0x62,  0x06,0x01,  0x08,0x00  ,0x01, 0x02,   0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 
  };
  while(!gps_set_sucess)
  {
    sendUBX(setNAV_POLLSH, 14);
    gps_set_sucess=getUBX_ACK(setNAV_POLLSH);
  }
  gps_set_sucess=0;
     //-------------------------------------------------------------------------------------
      
Serial.println("setting NAV-DOP on");
  uint8_t setNAV_DOP[] = { 
    
    0xB5, 0x62,   0x06,0x01, 0x08,0x00,    0x01, 0x04,      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  };
  while(!gps_set_sucess)
  {
    sendUBX(setNAV_DOP, 14);
    gps_set_sucess=getUBX_ACK(setNAV_DOP);
  }
  gps_set_sucess=0;
  
   //-------------------------------------------------------------------------------------
     
   
} 






void loop()
{

           int start = millis();
           while (millis() - start < 1000)
              {
                    if(Serial1.available() > 0){
                           
                      parser.parse(Serial1.read());
                            
                    }
              }
              LAT/=10000000;
              LON/=10000000;
              HEADING/=100000;
              Serial.print("LAT = ");
              Serial.print(LAT,8);
              Serial.print(" LON = ");
              Serial.print(LON,8);
              Serial.print(" HEADING = ");
              Serial.println(HEADING,8);
           double setPoint = reference(LON,LAT);
           
              Serial.print("SetPoint = ");
              Serial.println(setPoint);
           sinal_de_controle = calculo_pid(setPoint,HEADING);
           
           
           
           Serial.print("Sinal de controle = ");
           Serial.println(sinal_de_controle,4);
           dataFile.println(sinal_de_controle,8);//ESCREVE OS DADOS
           dataFile.flush();//SALVA OS DADOS
  
    
   
 
  
  
}

double reference(double Xi, double Yi) {
  
  double Xf=0; // coordenadas finais são escolhidas previamente 
  double Yf=0;
  
  double y = Yi - Yf;
  double x = Xi- Xf;
  
  
  double ref = atan2(y, x); // arc tangent of y/x
 
  Serial.print(" Xi (LON) = ");
  Serial.print(Xi,8);
  Serial.print(" Yi (LAT) = ");
  Serial.println(Yi,8);
 
  return ref;
}

double calculo_pid(double REF, double Sample){

                double erro = REF - Sample;

                Serial.print(" REF (setPoint) = ");
                Serial.print(REF,8);
                Serial.print(" Sample (HEADING) = ");
                Serial.println(Sample,8);
                 
		double deltaTime = (millis() - lastProcess) / 1000.0;
		lastProcess = millis();
		
		//P
		P = erro * kP;
		
		//I
		I = I + (erro * kI) * deltaTime;
		
		//D
                double filtro = filtro + (erro*N) * deltaTime;
                D = (N*kD)/(1+filtro);
                
		//D = (lastSample - H)* kD/ deltaTime;
		//lastSample = H;
		
		// Soma tudo
		pid = P + I + D;
                
                Serial.print("P = ");
                Serial.print(P,7);
                Serial.print("  I = ");
                Serial.print(I,7);
                Serial.print("  D = ");
                Serial.print(D,7);
                Serial.print("  pid = ");
                Serial.println(pid);
                


  return pid;//+lksajd;


}












void sendCheckSum(uint8_t *MSG,uint8_t len ){
  uint8_t ckA = 0, ckB = 0;
  for (int i=0;i<len;i++){
    ckA = ckA + MSG[i];
    ckB = ckB + ckA;
  }
  
  gpsSerial.write(ckA);
  Serial.print(ckA, HEX);
  Serial.print(" ");
  
  gpsSerial.write(ckB);
  Serial.print(ckB, HEX);
  Serial.print(" ");
}

// Send a byte array of UBX protocol to the GPS
void sendUBX(uint8_t *MSG, uint8_t len) {
  for(int i=0; i<len; i++) {
    gpsSerial.write(MSG[i]);
    Serial.print(MSG[i], HEX);
    Serial.print(" ");
  }
  sendCheckSum(MSG+2/*remove header*/, len-2/*remove header*/);
  Serial.println();
}
 
 
// Calculate expected UBX ACK packet and parse UBX response from GPS
boolean getUBX_ACK(uint8_t *MSG) {
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();
  Serial.print(" * Reading ACK response: ");
 
  // Construct the expected ACK packet    
  ackPacket[0] = 0xB5;	// header
  ackPacket[1] = 0x62;	// header
  ackPacket[2] = 0x05;	// class
  ackPacket[3] = 0x01;	// id
  ackPacket[4] = 0x02;	// length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2];	// ACK class
  ackPacket[7] = MSG[3];	// ACK id
  ackPacket[8] = 0;		// CK_A
  ackPacket[9] = 0;		// CK_B
 
  // Calculate the checksums
  for (uint8_t i=2; i<8; i++) {
    ackPacket[8] = ackPacket[8] + ackPacket[i];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }
 
  while (1) {
 
    // Test for success
    if (ackByteID > 9) {
      // All packets in order!
      Serial.println(" (SUCCESS!)");
      return true;
    }
 
    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) { 
      Serial.println(" (FAILED!)");
      return false;
    }
 
    // Make sure data is available to read
    if (gpsSerial.available()) {
      b = gpsSerial.read();
 
      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID]) { 
        ackByteID++;
        Serial.print(b, HEX);
      } 
      else {
        ackByteID = 0;	// Reset and look again, invalid order
      }
 
    }
  }
}
