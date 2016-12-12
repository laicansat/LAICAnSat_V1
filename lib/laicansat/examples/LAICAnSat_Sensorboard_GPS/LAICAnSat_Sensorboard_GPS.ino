/*Marina Andrade 
16/10/2015

Laicansat1.1 

As variaveis ground_speed e gorund_course precisam ser divididas por 10 
no processamento de dados.

*/








#include <laicansat.h>

#include <SD.h>
#include <SD_t3.h>

#include <SPI.h>

#include <GPS_UBLOX.h>

const int chipSelect = 20;
const int led = 13; //MUDAR PARA 30
double arrayAccel[3] = {0,0,0};
double angularSpeeds[3] = {0,0,0};
File dataFile;

void setup() {
  SPI.setMOSI(7);
  SPI.setMISO(8);
  SPI.setSCK(14);
  pinMode(led, OUTPUT);
  Serial.begin(9600);
  GPS.Init();
  delay(3000);
  Serial.print("Initializing SD card...");
  pinMode(SS, OUTPUT);
  pinMode(led, OUTPUT);
  
  
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    while (1) ;
  }
  Serial.println("card initialized.");
  

  dataFile = SD.open("datalog.txt", FILE_WRITE);
  if (! dataFile) {
    Serial.println("error opening datalog.txt");
    while (1) ;
  }
  dataFile.println(",Time,Fix,Latitude,Longitude,Altitude/1000,GroundSpeed/10,GroundCourse/10000,UmidadeSHT15,TemperaturaBMP180,TemperaturaMS5611,TemperaturaSHT15,PressaoBMP180,AcelaracaoX,AceleracaoY,AceleracaoZ,VeloAnguX,VeloAnguY,VeloAngZ");
  laicansat.bar->begin(BMP180_BAROMODE);
  laicansat.thermo->begin(BMP180_THERMOMODE);
  laicansat.thermo->begin(MS5611_THERMOMODE);
  laicansat.thermo->begin(SHT15_THERMOMODE);
  laicansat.accel->begin();  
  laicansat.gyro->begin();
  laicansat.hygro->begin();
  digitalWrite(led, HIGH);
  delay(1000);

}

void loop() {
  
  String dataString = "";
  GPS.Read();
  if (GPS.NewData){
    /*Serial.print(GPS.Time);
    Serial.print(",");
    Serial.print((int)GPS.Fix);
    Serial.print(",");
    Serial.print(GPS.Lattitude);
    Serial.print(",");
    Serial.print(GPS.Longitude);
    Serial.print(",");
    Serial.print(GPS.Altitude/1000.0);
    Serial.print(",");
    Serial.print(GPS.Ground_Speed/100.0);
    Serial.print(",");
    Serial.println(GPS.Ground_Course/100000.0);*/
   
    double Ground_Speed = GPS.Ground_Speed/10.0; 
    double Ground_Course = GPS.Ground_Course/10000.0;
    
    dataString += String(GPS.Time);
    dataString += ","; 
    dataString += String((int)GPS.Fix);
    dataString += ",";
    dataString += String(GPS.Lattitude);
    dataString += ","; 
    dataString += String(GPS.Longitude);
    dataString += ","; 
    dataString += String(GPS.Altitude/1000.0);
    dataString += ","; 
    dataString += String(Ground_Speed);
    dataString += ","; 
    dataString += String(Ground_Course);
    dataString += ","; 
    
   
  double temperatureBMP180 = laicansat.thermo->getTemperatureBMP180();
  double temperatureMS5611 = laicansat.thermo->getTemperatureMS5611();
  double temperatureSHT15 = laicansat.thermo->getTemperatureSHT15();
  double pressureBMP180 = laicansat.bar->getPressure();
  double humiditySHT15 = laicansat.hygro->getHumidity();
  laicansat.accel->getAcceleration(arrayAccel);
  laicansat.gyro->getAngularSpeed(angularSpeeds);
   
    int count = 0;
    dataString += String (humiditySHT15);
    dataString += ",";
    dataString += String (temperatureBMP180);
    dataString += ",";
    dataString += String (temperatureMS5611);
    dataString += ",";
    dataString += String (temperatureSHT15);
    dataString += ",";
    dataString += String (pressureBMP180);
    dataString += ",";
  
      for (count = 0; count < 3; count++) {
    
          dataString += String(arrayAccel[count]);
          if (count < 3) 
           dataString += ","; 
         }
       for (count = 0; count < 3; count++) {
    
      dataString += String(angularSpeeds[count]);
      if (count < 2) 
      dataString += ","; 
    
      }
    
    
    dataFile.println(dataString);

  // print to the serial port too:
    Serial.println(dataString);
  
    dataFile.flush();
  
   
  }
  
    GPS.NewData = 0;
    
 
}
