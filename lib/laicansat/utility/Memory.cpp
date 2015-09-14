

#include Memory.h



void MemoryClass::inicializeSDcard(){

  pinMode(62, OUTPUT);

  this->card = new SD();

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

}