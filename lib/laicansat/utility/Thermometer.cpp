/*
Resources:
- BMP180 GitHub: https://github.com/sparkfun/BMP180_Breakout
- BMP085 GitHub: https://github.com/adafruit/Adafruit-BMP085-Library
- One Wire Digital Temperature. DS18B20 + Arduino: http://bildr.org/2011/07/ds18b20-arduino/
- DS18B20 GitHub: https://github.com/milesburton/Arduino-Temperature-Control-Library

*/

#include "Thermometer.h"
#include "OneWire.h"

void ThermometerClass::begin(char mode)
{
  this->thermo_mode = mode;

  switch(this->thermo_mode)
  {
    case BMP180_THERMOMODE:
      this->beginBMP180();
    break;

    case DS18B20_THERMOMODE:
      this->beginDS18B20();
    break;

    case MS5611_THERMOMODE:
      this->beginMS5611();
    break;

    case SHT15_THERMOMODE:
      this->beginSHT15();
    break;

    case BMP085_THERMOMODE:
      this->beginBMP085();
    break;

    case MEAN_THERMOMODE:
      this->beginBMP180();
      this->beginDS18B20();
      this->beginMS5611();
      this->beginSHT15();
      this->beginBMP085();
    break;
  }
}

void ThermometerClass::beginBMP180()
{
  this->bmp180 = new SFE_BMP180();

  while(!this->bmp180->begin())
  {
    Serial.println("BMP180 could not start. Trying again.");
    delay(500);
  }

  Serial.println("BMP180 initiation successful!");
}

void ThermometerClass::beginDS18B20()
{
  OneWire oneWire(SCL_PIN);
  this->ds18b20 = new DallasTemperature(&oneWire);
  this->ds18b20->begin();
}

void ThermometerClass::beginMS5611()
{
  this->ms5611 = new MS5611();

  // Ultra high resolution: MS5611_ULTRA_HIGH_RES
  // (default) High resolution: MS5611_HIGH_RES
  // Standard: MS5611_STANDARD
  // Low power: MS5611_LOW_POWER
  // Ultra low power: MS5611_ULTRA_LOW_POWER
  while(!this->ms5611->begin(MS5611_ULTRA_HIGH_RES))
  {
    Serial.println("MS5611 could not start. Trying again.");
    delay(500);
  }

  Serial.println("MS5611 initiation successful!");
}

void ThermometerClass::beginSHT15()
{
  this->sht15 = new SHT1x(SDA_PIN, SCL_PIN);
}

void ThermometerClass::beginBMP085()
{
  this->bmp085 = new Adafruit_BMP085();

  while(!this->bmp085->begin())
  {
    Serial.println("BMP085 could not start. Trying again.");
    delay(500);
  }

  Serial.println("BMP085 initiation successful!");
}

double ThermometerClass::getTemperature()
{
  switch(this->thermo_mode)
  {
    case BMP180_THERMOMODE:
      return getTemperatureBMP180();
    break;

    case DS18B20_THERMOMODE:
      return getTemperatureDS18B20();
    break;

    case MS5611_THERMOMODE:
      return getTemperatureMS5611();
    break;

    case SHT15_THERMOMODE:
      return getTemperatureSHT15();
    break;

    case BMP085_THERMOMODE:
      return getTemperatureBMP085();
    break;

    case MEAN_THERMOMODE:

    break;
  }

  return -1;
}

double ThermometerClass::getTemperatureBMP180()
{
  char status;
  double T;

  status = this->bmp180->startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:
    delay(status);
    
    status = this->bmp180->getTemperature(T);

    return T;
  }
  return -1;
}

double ThermometerClass::getTemperatureDS18B20()
{
  this->ds18b20->requestTemperatures(); // Send the command to get temperatures
  return (double) this->ds18b20->getTempCByIndex(0);
}

double ThermometerClass::getTemperatureMS5611()
{
  return this->ms5611->readTemperature();
}

double ThermometerClass::getTemperatureSHT15()
{
  return (double) this->sht15->readTemperatureC();
}

double ThermometerClass::getTemperatureBMP085()
{
  return (double) this->bmp085->readTemperature();
}