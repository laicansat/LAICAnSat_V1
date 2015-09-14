#include GPS.h


void GPSClass::beginGPS(){

	this->gps_ublox = new GPS_UBLOX_Class();
	Serial.print("Initializing GPS...");

	while (!this->gps_ublox->Init())
  {
    Serial.println("GPS could not start. Trying again.");
    delay(500);
  }

  Serial.println("GPS initiation successful!");

}


void GPSClass::getData(double *gpsData){



	this->gps_ublox->Read();
		if (gps_ublox->NewData){

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