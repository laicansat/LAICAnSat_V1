#include "Barometer.h"

void BarometerClass::begin()
{
	this->bmp = new SFE_BMP180();

	while(!this->bmp->begin())
	{
    	Serial.println("BMP180 could not start. Trying again.");
    	delay(500);
    }

    Serial.println("BMP180 initiation successful!");
  	 
  	// Get the baseline pressure:
  
  	this->pressureBaseline = getPressure();
}

double BarometerClass::getPressure()
{
	char status;
	double P, dummyT;

	status = this->bmp->startPressure(3);
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
    status = this->bmp->getPressure(P,dummyT);

    return P;
  }
  return -1;
}