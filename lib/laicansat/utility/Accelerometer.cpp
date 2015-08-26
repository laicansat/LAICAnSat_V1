#include "Accelerometer.h"

void AccelerometerClass::begin()
{
  this->lsm = new Adafruit_LSM303();

  while (!this->lsm->begin())
  {
    Serial.println("LSM303 could not start. Trying again.");
    delay(500);
  }

  Serial.println("LSM303 initiation successful!");
}

void AccelerometerClass::getAcceleration(double *arrayAccel)
{
  this->lsm->read();

  arrayAccel[0] = (double)(int)lsm->accelData.x;
  arrayAccel[1] = (double)(int)lsm->accelData.y;
  arrayAccel[2] = (double)(int)lsm->accelData.z;
}