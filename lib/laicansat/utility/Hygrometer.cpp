
#include "Hygrometer.h"

void HygrometerClass::begin()
{
	this->sht = new SHT1x(SDA_PIN_SHT15,SCL_PIN_SHT15);
}

double HygrometerClass::getHumidity()
{
	return (double) this->sht->readHumidity();
}