#ifndef _THERMOMETER_H
#define _THERMOMETER_H

#include "BMP180.h"
#include "MS5611.h"
#include "SHT1x.h"


#include "../laicansat.h"

#define SDA_PIN 17
#define SCL_PIN 16
#define SDA_PIN_SHT15 15
#define SCL_PIN_SHT15 14

#define BMP180_THERMOMODE 1
#define MS5611_THERMOMODE 3
#define SHT15_THERMOMODE 4

#define MEAN_THERMOMODE 6

class ThermometerClass
{
public:
	SFE_BMP180 *bmp180;						// Functional
	MS5611 *ms5611;							// Functional
	SHT1x *sht15;							// Functional
	

	char thermo_mode = BMP180_THERMOMODE;

	void begin(char mode);
	void beginBMP180();
	void beginMS5611();
	void beginSHT15();
	

	double getTemperature();
	double getTemperatureBMP180();
	double getTemperatureMS5611();
	double getTemperatureSHT15();
	
};

#endif