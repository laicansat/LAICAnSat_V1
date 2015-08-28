#include<laicansat.h>

/**
 BMP180_THERMOMODE  =   1
 DS18B20_THERMOMODE =   2
 MS5611_THERMOMODE  =   3
 SHT15_THERMOMODE   =   4
 BMP085_THERMOMODE  =   5
 MEAN_THERMOMODE    =   6
**/

void setup()
{
  Serial.begin(9600);
  
  Serial.println("Teste");
  laicansat.thermo->begin(BMP180_THERMOMODE); 
  laicansat.bar->begin(BMP180_BAROMODE);
  Serial.println("Initiated!");
  delay(100);
}

void loop()
{
  double temperature = laicansat.thermo->getTemperature();
  double pressure = laicansat.bar->getPressure();
  
  Serial.println(temperature);
  Serial.println(pressure);
  delay(100);
}
