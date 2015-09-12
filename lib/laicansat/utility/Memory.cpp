#include Memory.h





void MemoryClass::inicializeSDcard(){


  Serial.begin(9600);
  Serial.print("Initializing SD card...");

  pinMode(53, OUTPUT);
 
  	if (!SD.begin(_chipSelect)) {
    Serial.println("Card failed, or not present");
    
    return;
  	}
  Serial.println("card initialized.");


}