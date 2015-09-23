#include <laicansat.h>



double gpsData[7] = {0,0,0,0,0,0,0};

void setup() {
  Serial.begin(9600);//GPS Ublox Baud rate
  
  laicansat.gps->beginGPS();
  delay(2000);
}

void loop() {
 
  laicansat.gps->getData(gpsData);
  
  Serial.println("DADOS GPS");
  Serial.print(gpsData[0]);
  Serial.println(", ");
  Serial.print(gpsData[1]);
  Serial.println(", ");
  Serial.print(gpsData[2]);
  Serial.println(", ");
  Serial.print(gpsData[3]);
  Serial.println(", ");
  Serial.print(gpsData[4]);
  Serial.println(", ");
  Serial.print(gpsData[5]);
  Serial.println(", ");
  Serial.println(gpsData[6]);
  delay(1000);
  
 }
