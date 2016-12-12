//LIGAR GPS NO SERIAL1

#include <laicansat.h>
#include <SPI.h>
#include <SD.h>




double gpsData[7] = {0,0,0,0,0,0,0};
const int led = LED_BUILTIN;
void setup() {
  Serial.begin(96000);//GPS Ublox Baud rate
  pinMode(led, OUTPUT);
  laicansat.gps->beginGPS();
  delay(1000);
}

void loop() {
 
  
  laicansat.gps->getData(gpsData);
  
  //Serial.println("DADOS GPS");
  Serial.print(gpsData[0]);
  Serial.print(", ");
  Serial.print(gpsData[1]);
  Serial.print(", ");
  Serial.print(gpsData[2]);
  Serial.print(", ");
  Serial.print(gpsData[3]);
  Serial.print(", ");
  Serial.print(gpsData[4]);
  Serial.print(", ");
  Serial.print(gpsData[5]);
  Serial.print(", ");
  Serial.println(gpsData[6]);
  if(gpsData[1]>0){
  digitalWrite(led, HIGH);
  delay(50);
  digitalWrite(led, LOW);
  //delay(100);
  }
  
  delay(100);
  
 }
