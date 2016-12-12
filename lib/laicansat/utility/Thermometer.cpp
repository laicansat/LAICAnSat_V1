/*
Resources:
- BMP180 GitHub: https://github.com/sparkfun/BMP180_Breakout
- BMP085 GitHub: https://github.com/adafruit/Adafruit-BMP085-Library

*/

#include "Thermometer.h"


void ThermometerClass::begin(char mode)
{
  this->thermo_mode = mode;

  switch(this->thermo_mode)
  {
    case BMP180_THERMOMODE:
      this->beginBMP180();
    break;

    case MS5611_THERMOMODE:
      this->beginMS5611();
    break;

    case SHT15_THERMOMODE:
      this->beginSHT15();
    break;

    

    case MEAN_THERMOMODE:
      this->beginBMP180();
      this->beginMS5611();
      this->beginSHT15();
      
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
  this->sht15 = new SHT1x(SDA_PIN_SHT15, SCL_PIN_SHT15);
}



double ThermometerClass::getTemperature()
{
  switch(this->thermo_mode)
  {
    case BMP180_THERMOMODE:
      return getTemperatureBMP180();
    break;

    case MS5611_THERMOMODE:
      return getTemperatureMS5611();
    break;

    case SHT15_THERMOMODE:
      return getTemperatureSHT15();
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



double ThermometerClass::getTemperatureMS5611()
{
  return this->ms5611->readTemperature();
}

double ThermometerClass::getTemperatureSHT15()
{
  return (double) this->sht15->readTemperatureC();
}

