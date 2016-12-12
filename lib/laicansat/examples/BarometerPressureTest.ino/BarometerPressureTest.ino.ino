

#include <laicansat.h>

void setup()
{
  Serial.begin(9600);
  
  laicansat.bar->begin(BMP180_BAROMODE); 
  delay(100);
}

void loop()
{
  double pressure = laicansat.bar->getPressure();
 
  Serial.println(pressure);
  delay(100);
}                                                                         
