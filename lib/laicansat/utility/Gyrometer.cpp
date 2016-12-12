#include "Gyrometer.h"

void GyrometerClass::begin()
{
  this->l3g = new Adafruit_L3GD20;

  while (!this->l3g->begin(l3g->L3DS20_RANGE_250DPS))
  {
    Serial.println("L3GD20 could not start. Trying again.");
    delay(500);
  }

  Serial.println("L3GD20 initiation successful!");
}

void GyrometerClass::getAngularSpeed(double *arrayAngSpeed)
{
  this->l3g->read();

  arrayAngSpeed[0] = (double)this->l3g->data.x;
  arrayAngSpeed[1] = (double)this->l3g->data.y;
  arrayAngSpeed[2] = (double)this->l3g->data.z;
}