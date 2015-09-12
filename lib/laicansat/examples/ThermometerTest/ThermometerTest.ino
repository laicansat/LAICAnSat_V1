#include<laicansat.h>

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
  
  Serial.println("Teste");
  laicansat.thermo->begin(BMP085_THERMOMODE); 
  laicansat.bar->begin(BMP085_BAROMODE);
  Serial.println("Initiated!");
   pinMode(led, OUTPUT);
  delay(100);
}

void loop()
{
  double temperature = laicansat.thermo->getTemperature();
  double pressure = laicansat.bar->getPressure();
  
  Serial.println(temperature);
  Serial.println(pressure);
   digitalWrite(led, HIGH);
  delay(100);
  digitalWrite(led, LOW);
  delay(100);
}
