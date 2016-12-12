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
  
  delay(3000);
  laicansat.thermo->begin(MS5611_THERMOMODE); 
  laicansat.thermo->begin(BMP180_THERMOMODE);
  laicansat.thermo->begin(SHT15_THERMOMODE);
  
  pinMode(led, OUTPUT);
  delay(100);
}

void loop()
{
  double temperatureBMP180 = laicansat.thermo->getTemperatureBMP180();
  double temperatureMS5611 = laicansat.thermo->getTemperatureMS5611();
  double temperatureSHT15  = laicansat.thermo->getTemperatureSHT15();
  
  Serial.println(temperatureBMP180);
  Serial.println(temperatureMS5611);
  Serial.println(temperatureSHT15);
  
  
  digitalWrite(led, HIGH);
  delay(500);
  digitalWrite(led, LOW);
  delay(500);
  
}
