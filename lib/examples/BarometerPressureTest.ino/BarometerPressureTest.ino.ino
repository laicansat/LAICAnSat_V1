#include <laicansat.h>

void setup()
{
  Serial.begin(9600);
  
  laicansat.bmp.begin(); 
  delay(100);
}

void loop()
{
  double pressure = laicansat.bmp.getPressure();
  
  Serial.writeln(pressure);
  delay(100);
}
