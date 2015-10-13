



/*
 CS = 20
 MOSI(DOUT) = 7
 MISO(DIN) = 8
 SCK = 14 
 
 BAUD RATE 38400
 
 */

#include <laicansat.h>
#include <SPI.h>
#include <SD.h>

const int chipSelect = 20;
const int led = 13; //MUDAR PARA 30
double arrayAccel[3] = {0,0,0};
double angularSpeeds[3] = {0,0,0};
double gpsData[7] = {0,0,0,0,0,0,0};

File dataFile;

void setup()
{
 
  SPI.setMOSI(7);
  SPI.setMISO(8);
  SPI.setSCK(14);
  Serial.begin(9600);
  delay(3500);
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
  dataFile.println("TemperaturaBMP180,PressaoBMP180,AcelaracaoX,AceleracaoY,AceleracaoZ,VeloAnguX,VeloAnguY,VeloAngZ,Time,Fix,Latitude,Longitude,Altitude/1000,GroundSpeed/100,GroundCourse/100000");
  laicansat.gps->beginGPS();
  laicansat.bar->begin(BMP180_BAROMODE);
  laicansat.thermo->begin(BMP180_THERMOMODE);
  //laicansat.thermo->begin(MS5611_THERMOMODE);
  laicansat.accel->begin();  
  laicansat.gyro->begin();
  delay(100);
  
}

void loop()
{
  laicansat.gps->getData(gpsData);
  double temperatureBMP180 = laicansat.thermo->getTemperature();
  //double temperatureMS5611 = laicansat.thermo->getTemperature();
  double pressureBMP180 = laicansat.bar->getPressure();
  laicansat.accel->getAcceleration(arrayAccel);
  laicansat.gyro->getAngularSpeed(angularSpeeds);
  //String dataString = "";
  //dataString = arrangeData(temperatureBMP180,temperatureMS5611,pressureBMP180,arrayAccel,angularSpeeds, gpsData)

  int count = 0;
  String dataString = "";


  dataString += String (temperatureBMP180);
  dataString += ",";
  /*dataString += String (temperatureMS5611);
  dataString += ",";*/
  dataString += String (pressureBMP180);
  dataString += ",";
  
  for (count = 0; count < 3; count++) {
    
    dataString += String(arrayAccel[count]);
    if (count < 2) 
      dataString += ","; 
    
  }
  for (count = 0; count < 3; count++) {
    
    dataString += String(angularSpeeds[count]);
    if (count < 2) 
      dataString += ","; 
    
  }
  for (count = 0; count < 7; count++) {
    
    dataString += String(gpsData[count]);
    if (count < 6) 
      dataString += ","; 
    
  }
  
  dataFile.println(dataString);

  // print to the serial port too:
  Serial.println(dataString);
  
  // The following line will 'save' the file to the SD card after every
  // line of data - this will use more power and slow down how much data
  // you can read but it's safer! 
  // If you want to speed up the system, remove the call to flush() and it
  // will save the file only every 512 bytes - every time a sector on the 
  // SD card is filled with data.
  dataFile.flush();
  
  // Take 1 measurement every 500 milliseconds
  digitalWrite(led, HIGH);
  delay(100);
  digitalWrite(led, LOW);
  delay(100);
}

/*String arrangeData (double temperatureBMP180, //double temperatureMS5611, double pressureBMP180, double arrayAccel, double angularSpeeds, double gpsData)
{

  int count = 0;
  String dataString = "";


  dataString += String (temperatureBMP180);
  dataString += ",";
  //dataString += String (temperatureMS5611);
  dataString += ",";
  dataString += String (pressureBMP180);
  dataString += ",";
  
  for (count = 0; count < 3; count++) {
    
    dataString += String(arrayAccel[count]);
    if (count < 2) 
      dataString += ","; 
    
  }
  for (count = 0; count < 3; count++) {
    
    dataString += String(angularSpeeds[count]);
    if (count < 2) 
      dataString += ","; 
    
  }
  for (count = 0; count < 7; count++) {
    
    dataString += String(gpsData[count]);
    if (count < 6) 
      dataString += ","; 
    
  }


return dataString
}*/







