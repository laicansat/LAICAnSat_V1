 
#include <laicansat.h>
#include <SD.h>
#include <SD_t3.h>
#include <SPI.h>
#include <UBX_Parser.h>

int FIX = 0;

class MyParser : public UBX_Parser {

    /*void handle_NAV_POSLLH(unsigned long iTOW, 
            long lon, 
            long lat, 
            long height, 
            long hMSL, 
            unsigned long hAcc, 
            unsigned long vAcc) {

        //Serial.print("NAV-POSLLH: iTOW=");
        //Serial.print(iTOW);
        //Serial.print("ms lat=");
        //Serial.print(lat/1e7,7);
        //Serial.print("deg lon=");
        //Serial.print(lon/1e7,7);
        //Serial.print("deg height=");
        //Serial.print(height);
        //Serial.print("mm hMSL=");
        //Serial.print(hMSL);
        //Serial.print("mm hAcc=");
        //Serial.print(hAcc);
        //Serial.print("mm vAcc=");
        //Serial.print(vAcc);
        //Serial.println("mm");
    }  

    void handle_NAV_DOP(unsigned long iTOW, 
            unsigned short gDOP,
            unsigned short pDOP,
            unsigned short tDOP,
            unsigned short vDOP,
            unsigned short hDOP,
            unsigned short nDOP,
            unsigned short eDOP) {

        Serial.print("NAV-DOP: iTOW=");
        Serial.print(iTOW);
        Serial.print("ms gDOP=");
        Serial.print(gDOP/100., 2);
        Serial.print(" pDOP=");
        Serial.print(gDOP/100., 2);
        Serial.print(" tDOP=");
        Serial.print(tDOP/100., 2);
        Serial.print(" vDOP=");
        Serial.print(vDOP/100., 2);
        Serial.print(" hDOP=");
        Serial.print(hDOP/100., 2);
        Serial.print(" nDOP=");
        Serial.print(nDOP/100., 2);
        Serial.print(" eDOP=");
        Serial.println(eDOP/100., 2);
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
        Serial.print("NAV-VELNED: iTOW=");
        Serial.print(iTOW);
        Serial.print("ms velN=");
        Serial.print(velN);
        Serial.print("cm/s");
        Serial.print("ms velE=");
        Serial.print(velE);
        Serial.print("cm/s");
        Serial.print("ms velD=");
        Serial.print(velD);
        Serial.print("cm/s speed=");
        Serial.print(speed);
        Serial.print("cm/s gSpeed=");
        Serial.print(gSpeed);
        Serial.print("cm/s heading=");
        Serial.print(heading/1e5, 5);
        Serial.print("deg sAcc=");
        Serial.print(sAcc);
        Serial.print("cm/s cAcc=");
        Serial.print(cAcc/1e5, 5);
        Serial.println("deg");
     }*/

    
    void handle_NAV_SOL(unsigned long iTOW,
            long fTOW, 
            short week,
            char gpsFix) {

        //Serial.print("NAV-SOL: iTOW=");
       //Serial.print(iTOW);
        //Serial.print(" fTOW");
        //Serial.print(fTOW);
        //Serial.print(" week= ");
        //Serial.print(week);
        //Serial.print(" gpsFix = ");
        //Serial.println(gpsFix,HEX);
        FIX = int(gpsFix);
       
       
        
    }
};


const int chipSelect = 20; 
File dataFileGNSS;
File dataFile;
byte gps_set_sucess = 0 ;
const int led = 30;
double arrayAccel[3] = {0,0,0};
double angularSpeeds[3] = {0,0,0};


HardwareSerial gpsSerial(Serial1);



  
 
void setup()
{
  
  SPI.setMOSI(7);
  SPI.setMISO(8);
  SPI.setSCK(14);  
  Serial.begin(9600);
  gpsSerial.begin(9600);
  delay(2000);
  
  Serial.print("Initializing SD card...");
  pinMode(20, OUTPUT);
  pinMode(led, OUTPUT);
 
  if (!SD.begin(chipSelect)) {
     Serial.println("Card failed, or not present");
     while (1) ;
   }
  Serial.println("card initialized.");
   
 dataFileGNSS = SD.open("gnss11.txt", FILE_WRITE);
           if (! dataFileGNSS) {
             Serial.println("error opening datalog.txt");
             while (1) ;
           }
//CRIA E ABRE ARQUIVO DE DADOS
  dataFile = SD.open("datalog.txt", FILE_WRITE);
   if (!dataFile) {
     Serial.println("error opening datalog.txt");
     while (1) ;
   }
           
 
  
  delay(1000);
  Serial.println(" GPS Initialising....");
  
  
  
   
  //INICIALIZAÇÃO DE TODOS OS SENSORES
  laicansat.bar->begin();
  laicansat.thermo->begin(BMP180_THERMOMODE);
  laicansat.thermo->begin(MS5611_THERMOMODE);
  laicansat.thermo->begin(SHT15_THERMOMODE);
  laicansat.accel->begin();  
  laicansat.gyro->begin();
  laicansat.hygro->begin();
  
 // ESSE COMANDO SETA O MÓDULO GNSS
 
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
    
    0xB5,0x62, 0x06,0x01,  0x08,0x00,    0x02, 0x15,    0x00,0x01,0x00,0x00,0x00,0x00, 
  
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
    
    0xB5,0x62,0x06,0x01,0x08,0x00,    0x02, 0x13,   0x00,0x01,0x00,0x00,0x00,0x00, 
  
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
    
    0xB5, 0x62,   0x06, 0x01, 0x08, 0x00,    0xF0, 0x00,  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
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
    
    0xB5, 0x62,   0x06,0x01,0x08,0x00,    0x01, 0x03,      0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 
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
    
    0xB5, 0x62,0x06,0x01,0x08,0x00,0x01,0x30,             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  };
  while(!gps_set_sucess)
  {
    sendUBX(setNAV_SVINFO, 14);
    gps_set_sucess=getUBX_ACK(setNAV_SVINFO);
  }
  gps_set_sucess=0;

Serial.println("setting NAV-POLLSH on");
  uint8_t setNAV_POLLSH[] = { 
    
    0xB5, 0x62,0x06,0x01,0x08,0x00,0x01, 0x02,            0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 
  };
  while(!gps_set_sucess)
  {
    sendUBX(setNAV_POLLSH, 14);
    gps_set_sucess=getUBX_ACK(setNAV_POLLSH);
  }
  gps_set_sucess=0;
     
   
   digitalWrite(led, HIGH); 
} 

MyParser parser; 
 
//LOOP PRINCIPAL 
void loop()
{
  
  String dataStringGNSS = "";
  char inByte;
           
        
           int start = millis();
           while (millis() - start < 1000)
              {
                    if(gpsSerial.available() > 0){
                            inByte = gpsSerial.read();
                            parser.parse(inByte);
                            dataStringGNSS  = String(inByte);
                            //Serial.print(dataStringGNSS);
                            dataFileGNSS.print(dataStringGNSS);
                            
                           
                           /* if (FIX==2 || FIX==3){
                            digitalWrite(led, HIGH);
                            
                            }*/
                    }
                    
              }
              
             
              
              
              
              
              dataFileGNSS.flush();
              
              
              String dataSensors = "";
  
  
  double temperatureBMP180 = laicansat.thermo->getTemperatureBMP180();
  double temperatureMS5611 = laicansat.thermo->getTemperatureMS5611();
  double temperatureSHT15 = laicansat.thermo->getTemperatureSHT15();
  double pressureBMP180 = laicansat.bar->getPressure();
  double humiditySHT15 = laicansat.hygro->getHumidity();
  laicansat.accel->getAcceleration(arrayAccel);
  laicansat.gyro->getAngularSpeed(angularSpeeds);
   
  //SALVA TODAS AS VARIÁVEIS EM UMA ÚNCIA STRING 
    int count = 0;
    dataSensors += String (humiditySHT15);
    dataSensors += ",";
    dataSensors += String (temperatureBMP180);
    dataSensors += ",";
    dataSensors += String (temperatureMS5611);
    dataSensors += ",";
    dataSensors += String (temperatureSHT15);
    dataSensors += ",";
    dataSensors += String (pressureBMP180);
    dataSensors += ",";
  
      for (count = 0; count < 3; count++) {
    
          dataSensors += String(arrayAccel[count]);
          if (count < 3) 
           dataSensors += ","; 
         }
       for (count = 0; count < 3; count++) {
    
      dataSensors += String(angularSpeeds[count]);
      if (count < 2) 
      dataSensors += ","; 
     }
     
   pinMode(chipSelect, OUTPUT);
   if (!SD.begin(chipSelect)) {
     digitalWrite(led, LOW); 
     while (1) ;
       }
    //Serial.println("card initialized.");
    dataFile.println(dataSensors);//ESCREVE OS DADOS
    dataFile.flush();//SALVA OS DADOS
    //Serial.println(dataSensors);//IMPRIME OS DADOS NA TELA
       
      
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
