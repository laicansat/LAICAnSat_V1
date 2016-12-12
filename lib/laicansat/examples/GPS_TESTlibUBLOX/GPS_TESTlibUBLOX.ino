#include <GPS_UBLOX.h>
const int led = 30; // TROCAR PARA 30
void setup() {
   pinMode(led, OUTPUT);
   digitalWrite(led, HIGH);
  Serial.begin(9600);
  GPS.Init();
  delay(3000);

}

void loop() {
  GPS.Read();
  if (GPS.NewData){
    Serial.print(GPS.Time);
    Serial.print(",");
    Serial.print((int)GPS.Fix);
    Serial.print(",");
    Serial.print(GPS.Lattitude);
    Serial.print(",");
    Serial.print(GPS.Longitude);
    Serial.print(",");
    Serial.print(GPS.Altitude/1000.0);
    Serial.print(",");
    Serial.print(GPS.Ground_Speed/100.0);
    Serial.print(",");
    Serial.print(GPS.Ground_Course/100000.0);
   
  }
    Serial.println(GPS.NewData);
    GPS.NewData = 0;
    

}
