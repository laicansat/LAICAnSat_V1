#ifndef _GYROMETER_H
#define _GYROMETER_H

#include "L3G.h"

#include "../laicansat.h"

class GyrometerClass
{
public:
	L3G* gyro;

	int pressureBaseline;

	void begin();
	double getGyro();
	void show();
}

#endif