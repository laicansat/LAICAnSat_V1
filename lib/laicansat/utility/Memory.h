#ifndef _MEMORY_H
#define _MEMORY_H


#include "SD.h"

#include "../laicansat.h"

class MemoryClass{

public:
	double Data[10];
	void inicializeSDcard(); 
	void writeSDcard(double Data[10]);
	void closeSDcard();
private:
	const int _chipSelect = 62;
};


#endif
