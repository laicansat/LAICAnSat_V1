#ifndef _GYROMETER_H
#define _GYROMETER_H

#include "Adafruit_L3GD20.h"

#include "../laicansat.h"

class GyrometerClass
{
public:
	Adafruit_L3GD20* l3g;

	void begin();
	void getAngularSpeed(double *arrayAngSpeed);
};

#endif