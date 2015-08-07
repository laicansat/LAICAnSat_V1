#include "Gyrometer.h"

GyrometerClass::GyrometerClass()
{

}

GyrometerClass::~GyrometerClass()
{

}

void GyrometerClass::begin()
{
  this->gyro = new L3G;

    while (!thihs->gyro.init())
    {
      Serial.println("L3G could not start. Trying again.");
      delay(500);
    }

    Serial.println("L3G initiation successful!")
}

double* GyrometerClass::getAngularSpeed()
{
  this->gyro.read();
  double arrayAngSpeed[3];

  this->arrayAngSpeed[0] = (double)this->gyro.g.x;
  this->arrayAngSpeed[1] = (double)this->gyro.g.y;
  this->arrayAngSpeed[2] = (double)this->gyro.g.z;

  return arrayAngSpeed;
}