#include <laicansat.h>

double angularSpeeds[3] = {0,0,0};

void setup()
{
  Serial.begin(9600);
  
  laicansat.gyro->begin(); 
  delay(100);
}

void loop()
{
  laicansat.gyro->getAngularSpeed(angularSpeeds);
  
  Serial.print("Angular speeds: ");
  Serial.print(angularSpeeds[0]);
  Serial.print(", ");
  Serial.print(angularSpeeds[1]);
  Serial.print(", ");
  Serial.println(angularSpeeds[2]);
  delay(100);
}
