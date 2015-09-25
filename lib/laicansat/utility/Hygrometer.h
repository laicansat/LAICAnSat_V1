#ifndef _HYGROMETER_H
#define _HYGROMETER_H

#include "SHT1x.h"

#include "../laicansat.h"

#define SDA_PIN 17
#define SCL_PIN 16

class HygrometerClass
{
public:
	SHT1x *sht;

	void begin();
	double getHumidity();
};

#endif