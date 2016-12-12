#include<laicansat.h>

int led = 30;

void setup()
{
  Serial.begin(9600);
  pinMode(led, OUTPUT);
  
  laicansat.hygro->begin();
 digitalWrite(led, HIGH); 
  delay(100);
}

void loop()
{
  double humidity = laicansat.hygro->getHumidity();
  
  Serial.println(humidity);
  digitalWrite(led, HIGH);
  delay(1000);
  digitalWrite(led, LOW);
}
