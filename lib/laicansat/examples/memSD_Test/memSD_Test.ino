#include <SPI.h>
#include <SD.h>
#include <laicansat.h>

void setup() {
  
  Serial.begin(9600);
  laicansat.memSD->inicializeSDcard();
  delay(2000);

}

void loop() {
  // put your main code here, to run repeatedly:

}
