#ifndef _BAROMETER_H
#define _BAROMETER_H

#include "BMP180.h"

#include "../laicansat.h"

class BarometerClass
{
public:
	SFE_BMP180 *bmp;

	int pressureBaseline;

	void begin();
	double getPressure();
	double getAltitude();
	double getTemperature(char mode);
};

#endif