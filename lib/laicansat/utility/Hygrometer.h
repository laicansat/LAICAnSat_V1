#ifndef _HYGROMETER_H
#define _HYGROMETER_H

#include "SHT1x.h"

#include "../laicansat.h"

#define SDA_PIN_SHT15 15
#define SCL_PIN_SHT15 14

class HygrometerClass
{
public:
	SHT1x *sht;

	void begin();
	double getHumidity();
};

#endif