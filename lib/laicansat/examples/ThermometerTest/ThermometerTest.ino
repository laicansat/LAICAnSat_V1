#include<laicansat.h>
#include <SPI.h>
#include <SD.h>

/**
 BMP180_THERMOMODE  =   1
 DS18B20_THERMOMODE =   2
 MS5611_THERMOMODE  =   3
 SHT15_THERMOMODE   =   4
 BMP085_THERMOMODE  =   5
 MEAN_THERMOMODE    =   6
**/
const int led = LED_BUILTIN;
void setup()
{
  Serial.begin(9600);
  
  
  laicansat.thermo->begin(MS5611_THERMOMODE); 
  
  pinMode(led, OUTPUT);
  delay(100);
}

void loop()
{
  double temperature = laicansat.thermo->getTemperature();
  
  
  Serial.println(temperature);
  digitalWrite(led, HIGH);
  delay(100);
  digitalWrite(led, LOW);
  delay(100);
}
