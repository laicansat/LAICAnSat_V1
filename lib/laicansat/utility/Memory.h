#ifndef _MEMORY_H
#define _MEMORY_H


#include "SD.h"

#include "../laicansat.h"

class MemoryClass{

public:
	void inicializeSDcard(); 
	void writeSDcard();
private:
	const int _chipSelect = 62;
};


#endif
