#ifndef _THERMOMETER_H
#define _THERMOMETER_H

#include "BMP180.h"
#include "MS5611.h"
#include "SHT1x.h"
#include "Adafruit_BMP085.h"
#include "DallasTemperature.h"

#include "../laicansat.h"

#define SDA_PIN 17
#define SCL_PIN 16

#define BMP180_THERMOMODE 1
#define DS18B20_THERMOMODE 2
#define MS5611_THERMOMODE 3
#define SHT15_THERMOMODE 4
#define BMP085_THERMOMODE 5
#define MEAN_THERMOMODE 6

class ThermometerClass
{
public:
	SFE_BMP180 *bmp180;						// Functional
	DallasTemperature *ds18b20;				// Non-functional
	MS5611 *ms5611;							// Functional
	SHT1x *sht15;							// Functional
	Adafruit_BMP085 *bmp085;				// Functional

	char thermo_mode = BMP180_THERMOMODE;

	void begin(char mode);
	void beginBMP180();
	void beginDS18B20();
	void beginMS5611();
	void beginSHT15();
	void beginBMP085();

	double getTemperature();
	double getTemperatureBMP180();
	double getTemperatureDS18B20();
	double getTemperatureMS5611();
	double getTemperatureSHT15();
	double getTemperatureBMP085();
};

#endif