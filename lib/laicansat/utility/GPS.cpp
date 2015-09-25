#include "GPS.h"

const int led = 13; // TROCAR PARA 30

void GPSClass::beginGPS(){


	pinMode(led, OUTPUT);
	this->gps_ublox = new GPS_UBLOX_Class();
	/*delay(4000);
	gps_ublox->Fix = 10;
	Serial.println(gps_ublox->Fix);
	delay(3000);*/
	
	Serial.println("Initializing GPS...");

	this->gps_ublox->Init();

	/*delay(4000);
	Serial.println(gps_ublox->Fix);
	delay(3000);*/
 
  	Serial.println("GPS initiation successful!");
  	//delay(3000);

}


void GPSClass::getData(double *gpsData){



	this->gps_ublox->Read();
	if (gps_ublox->NewData){

		if (gps_ublox->Fix > 0){
			digitalWrite(led, HIGH);
  			delay(1000);
  			digitalWrite(led, LOW);
  			delay(1000);
  			digitalWrite(led, HIGH);
  			delay(1000);
  			digitalWrite(led, LOW);
  			delay(1000);
  			digitalWrite(led, HIGH);
  			delay(1000);
  			digitalWrite(led, LOW);
  			delay(1000);
  		}
		gpsData[0] = (double) gps_ublox->Time;
		gpsData[1] = (double) (int) gps_ublox->Fix;
		gpsData[2] = (double) (int) gps_ublox->Lattitude;
		gpsData[3] = (double) (int) gps_ublox->Longitude;
		gpsData[4] = (double)  gps_ublox->Altitude/1000.0;
		gpsData[5] = (double)  gps_ublox->Ground_Speed/100.0;
		gpsData[6] = (double)  gps_ublox->Ground_Course/100000.00;

	}
	gps_ublox->NewData = 0;

}