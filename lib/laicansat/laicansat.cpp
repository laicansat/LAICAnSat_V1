#include "laicansat.h"

LaicansatClass laicansat;

LaicansatClass::LaicansatClass()
{
	this->bar = new BarometerClass;
	this->gyro = new GyrometerClass;
	this->accel = new AccelerometerClass;
}

LaicansatClass::~LaicansatClass()
{
	delete this->bar;
	delete this->gyro;
	delete this->accel;
}