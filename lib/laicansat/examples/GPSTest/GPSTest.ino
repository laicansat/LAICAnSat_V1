//LIGAR GPS NO SERIAL1

#include <laicansat.h>
#include <SPI.h>
#include <SD.h>




double gpsData[7] = {0,0,0,0,0,0,0};
const int led = LED_BUILTIN;
void setup() {
  Serial.begin(38400);//GPS Ublox Baud rate
  pinMode(led, OUTPUT);
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
  digitalWrite(led, HIGH);
  delay(500);
  digitalWrite(led, LOW);
  delay(500);
  
  
 }
