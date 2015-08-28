#include "Barometer.h"

void BarometerClass::begin(char mode)
{
  this->baro_mode = mode;

  switch(this->baro_mode)
  {
    case BMP180_BAROMODE:
      this->beginBMP180();
    break;

    case BMP085_BAROMODE:
      this->beginBMP085();
    break;
  }
}

void BarometerClass::beginBMP180()
{
  this->bmp180 = new SFE_BMP180();

  while(!this->bmp180->begin())
  {
    Serial.println("BMP180 could not start. Trying again.");
    delay(500);
  }

  Serial.println("BMP180 initiation successful!");

  // Get the baseline pressure:
  this->pressureBaseline = getPressure();
}

void BarometerClass::beginBMP085()
{
  this->bmp085 = new Adafruit_BMP085();

  while(!this->bmp085->begin())
  {
    Serial.println("BMP085 could not start. Trying again.");
    delay(500);
  }

  Serial.println("BMP085 initiation successful!");
}

double BarometerClass::getPressure()
{
  switch(this->baro_mode)
  {
    case BMP180_BAROMODE:
      this->getPressureBMP180();
    break;

    case BMP085_BAROMODE:
      this->getPressureBMP085();
    break;
  }
}

double BarometerClass::getPressureBMP180()
{
	char status;
	double P, dummyT;

	status = this->bmp180->startPressure(3);
  if (status != 0)
  {
    // Wait for the measurement to complete:
    delay(status);

    // Retrieve the completed pressure measurement:
    // Note that the measurement is stored in the variable P.
    // Use '&P' to provide the address of P.
    // Note also that the function requires the previous temperature measurement (T).
    // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
    // Function returns 1 if successful, 0 if failure.
    status = this->bmp180->getPressure(P,dummyT);

    return P;
  }
  return -1;
}

double BarometerClass::getPressureBMP085()
{
  return (double) this->bmp085->readPressure();
}