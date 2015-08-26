#ifndef _ACCELEROMETER_H
#define _ACCELEROMETER_H

#include "Adafruit_LSM303.h"

#include "../laicansat.h"

class AccelerometerClass
{
public:
	Adafruit_LSM303* lsm;

	void begin();
	void getAcceleration(double *arrayAccel); // in milligravity
};

#endif