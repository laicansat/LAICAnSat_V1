#ifndef _BAROMETER_H
#define _BAROMETER_H

#include "BMP180.h"

#include "../laicansat.h"

enum BarometerMode {BMP180_BAROMODE, BMP085_BAROMODE, MS5611_BAROMODE};

class BarometerClass
{
public:
	SFE_BMP180 *bmp;

	int pressureBaseline;

	void begin();
	double getPressure();
	double getAltitude();
};

#endif