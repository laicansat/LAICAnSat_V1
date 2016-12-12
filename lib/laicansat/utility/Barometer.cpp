#include "Barometer.h"

void BarometerClass::begin()
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

double BarometerClass::getPressure()
{

     char status;
  double P, dummyT;

  status = this->bmp180->startPressure(3);
  if (status != 0)
  {
    // Wait for the measurement to complete:
    delay(status);

  
    status = this->bmp180->getPressure(P,dummyT);

    return P;
  }
  return -1;
   
}


