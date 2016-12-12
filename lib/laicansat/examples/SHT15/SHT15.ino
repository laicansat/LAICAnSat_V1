#include <SPI.h>

#include <SD.h>


//AMBOS SENSORES DO SHT15 FUNCIONANDO SIMULTANEAMENTE

#include<laicansat.h>

const int led = LED_BUILTIN;
void setup()
{
  Serial.begin(9600);
  
  
  laicansat.thermo->begin(SHT15_THERMOMODE); 
  laicansat.hygro->begin(); 
 
  
  pinMode(led, OUTPUT);
  delay(100);
}

void loop()
{
  double temperature = laicansat.thermo->getTemperature();
  double humidity = laicansat.hygro->getHumidity();
  
  Serial.print("Humidity: ");
  Serial.println(humidity);
  Serial.print("Temperature: ");
  Serial.println(temperature);
  digitalWrite(led, HIGH);
  delay(100);
  digitalWrite(led, LOW);
  delay(100);
}
