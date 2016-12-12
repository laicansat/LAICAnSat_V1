#ifndef _BAROMETER_H
#define _BAROMETER_H

#include "BMP180.h"


#include "../laicansat.h"


class BarometerClass
{
public:
	SFE_BMP180 *bmp180;

	int pressureBaseline;

	void begin();
	
	double getPressure();
	
	
	
};

#endif