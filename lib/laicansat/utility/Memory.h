#ifndef _MEMORY_H
#define _MEMORY_H

#include <SD.h>

#include "../laicansat.h"

//CS laicansat = 62
class MemoryClass{

public:
	SDClass *card;
	double *Arraydata;
	void inicializeSDcard(); 
	void writeSDcard(double *Arraydata);
	void closeSDcard();
private:
	const int _chipSelect = 20;
};


#endif
