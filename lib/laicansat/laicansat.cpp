#include "laicansat.h"

LaicansatClass laicansat;

LaicansatClass::LaicansatClass()
{
	this->bar = new BarometerClass;
	this->gyro = new GyrometerClass;
	this->accel = new AccelerometerClass;
	this->hygro = new HygrometerClass;
}

LaicansatClass::~LaicansatClass()
{
	delete this->bar;
	delete this->gyro;
	delete this->accel;
	delete this->hygro;
}