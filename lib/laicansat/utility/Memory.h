#ifndef _MEMORY_H
#define _MEMORY_H

#include "SD.h"

#include "../laicansat.h"

class MemoryClass{

public:
	SDClass *card;
	double *Arraydata;
	void inicializeSDcard(); 
	void writeSDcard(double *Arraydata);
	void closeSDcard();
private:
	const int _chipSelect = 62;
};


#endif
