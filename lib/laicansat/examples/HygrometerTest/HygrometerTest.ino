#include<laicansat.h>

void setup()
{
  Serial.begin(9600);
  
  laicansat.hygro->begin(); 
  delay(100);
}

void loop()
{
  double humidity = laicansat.hygro->getHumidity();
  
  Serial.println(humidity);
  delay(100);
}
