#ifndef _LAICANSAT_H
#define _LAICANSAT_H

class BarometerClass;
class GyrometerClass;
class AccelerometerClass;
class HygrometerClass;
class ThermometerClass;
class MemoryClass;
class GPSClass;

#include <WProgram.h>

#include <utility/Barometer.h>
#include <utility/Gyrometer.h>
#include <utility/Accelerometer.h>
#include <utility/Hygrometer.h>
#include <utility/Thermometer.h>
#include <utility/Memory.h>
#include <utility/GPS.h>

class LaicansatClass
{
public:
	LaicansatClass();
	~LaicansatClass();

	BarometerClass* bar;
	GyrometerClass* gyro;
	AccelerometerClass* accel;
	HygrometerClass* hygro;
	ThermometerClass* thermo;
	MemoryClass* memSD;
	GPSClass* gps;
};

extern LaicansatClass laicansat;

#endif
