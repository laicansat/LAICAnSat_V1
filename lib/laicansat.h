#ifndef _LAICANSAT_H
#define _LAICANSAT_H

class BarometerClass;

#include <WProgram.h>

#include <utility/BarometerClass.h>

class LaicansatClass
{
public:
	LaicansatClass();
	~LaicansatClass();

	BarometerClass* bmp;
	GyrometerClass* gyro;
}

extern LaicansatClass laicansat;

#endif
