#ifndef _BAROMETER_H
#define _BAROMETER_H

#include "BMP180.h"

#include "../laicansat.h"

class BarometerClass
{
public:
	BMP180* bmp;

	int pressureBaseline;

	void begin();
	double getPressure();
	double getAltitude();
	double getTemperature();
	void show();
}

#endif