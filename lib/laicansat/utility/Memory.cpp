

#include Memory.h



void MemoryClass::inicializeSDcard(){


  Serial.begin(9600);
  Serial.print("Initializing SD card...");

  pinMode(62, OUTPUT);
 
  	if (!SD.begin(_chipSelect)) {
    Serial.println("Card failed, or not present");
    
    return;
  	}
  Serial.println("card initialized.");


}

void MemoryClass::writeSDcard(Data){
 

  File dataFile = SD.open("LAICAnSat_datalog.csv", FILE_WRITE);
 
  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(Data);
    
    // print to the serial port too:
    Serial.println(dataString);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening LAICAnSat_datalog.csv");
  }



}

void MemoryClass::closeSDcard(){

	dataFile.close();

}