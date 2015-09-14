#ifndef _GPS_H
#define _GPS_H


#include "GPS_UBLOX.h"

#include "../laicansat.h"

class GPSClass{

public:
	GPS *gps_ublox;
	
	
	void beginGPS(); 
	void getData(double *gpsData);
	
private:

	
};


#endif