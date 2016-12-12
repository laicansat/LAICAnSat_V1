#include <laicansat.h>

double accelerations[3] = {0,0,0};

void setup()
{
  Serial.begin(9600);
  
  laicansat.accel->begin(); 
  delay(100);
}

void loop()
{
  laicansat.accel->getAcceleration(accelerations);
  
  Serial.print("Accelerations: ");
  Serial.print(accelerations[0]);
  Serial.print(", ");
  Serial.print(accelerations[1]);
  Serial.print(", ");
  Serial.println(accelerations[2]);
  delay(100);
}
