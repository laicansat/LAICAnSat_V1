#ifndef _BAROMETER_H
#define _BAROMETER_H

#include "BMP180.h"
#include "Adafruit_BMP085.h"

#include "../laicansat.h"

enum BarometerMode {BMP180_BAROMODE, BMP085_BAROMODE, MS5611_BAROMODE};

class BarometerClass
{
public:
	SFE_BMP180 *bmp180;
	Adafruit_BMP085 *bmp085;

	char baro_mode = BMP180_BAROMODE;
	int pressureBaseline;

	void begin(char mode);
	void beginBMP180();
	void beginBMP085();
	double getPressure();
	double getPressureBMP180();
	double getPressureBMP085();
	double getAltitude();
};

#endif