#ifndef _LAICANSAT_H
#define _LAICANSAT_H

class BarometerClass;
class GyrometerClass;
class AccelerometerClass;
class HygrometerClass;

#include <WProgram.h>

#include <utility/Barometer.h>
#include <utility/Gyrometer.h>
#include <utility/Accelerometer.h>
#include <utility/Hygrometer.h>

class LaicansatClass
{
public:
	LaicansatClass();
	~LaicansatClass();

	BarometerClass* bar;
	GyrometerClass* gyro;
	AccelerometerClass* accel;
	HygrometerClass* hygro;
};

extern LaicansatClass laicansat;

#endif
