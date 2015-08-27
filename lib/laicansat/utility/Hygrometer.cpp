#include "Hygrometer.h"

void HygrometerClass::begin()
{
	this->sht = new SHT1x(SDA_PIN,SCL_PIN);
}

double HygrometerClass::getHumidity()
{
	return (double) this->sht->readHumidity();
}