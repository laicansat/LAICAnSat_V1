#include "laicansat.h"

LaicansatClass laicansat;

LaicansatClass::LaicansatClass()
{
	this->bmp = new LaicansatClass;
	this->gyro = new GyrometerClass;
}

LaicansatClass::~LaicansatClass()
{
	delete this->bmp;
	delete this->gyro;
}