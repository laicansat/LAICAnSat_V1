

#include "Memory.h"


//CS laicansat = 62
void MemoryClass::inicializeSDcard(){

  pinMode(20, OUTPUT);
 #error marina
  this->card = new SDClass();

  Serial.print("Initializing SD card...");

  
    if (!this->card->begin(_chipSelect)) {
    Serial.println("Card failed, or not present");
    
    return;
  	}
  Serial.println("card initialized.");


}

void MemoryClass::writeSDcard(double *Arraydata){
 

  File dataFile = this->card->open("LAICAnSat_datalog.txt", FILE_WRITE);
 
  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(Arraydata);
    
    // print to the serial port too:
    Serial.println(Arraydata);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening LAICAnSat_datalog.txt");
  }



}

void MemoryClass::closeSDcard(){

	dataFile.close();
  //call this when a card is removed. It will allow you to insert and initialise a new card.
  this->card->end();

}
