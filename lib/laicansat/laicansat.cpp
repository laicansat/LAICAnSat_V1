#include "laicansat.h"

LaicansatClass laicansat;

LaicansatClass::LaicansatClass()
{
	this->bar = new BarometerClass;
	this->gyro = new GyrometerClass;
	this->accel = new AccelerometerClass;
	this->hygro = new HygrometerClass;
	this->thermo = new ThermometerClass;
	//this->memSD = new MemoryClass;
	this->gps = new GPSClass;
}

LaicansatClass::~LaicansatClass()
{
	delete this->bar;
	delete this->gyro;
	delete this->accel;
	delete this->hygro;
	delete this->thermo;
	delete this->memSD;
	delete this->gps;
}