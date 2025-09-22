#ifndef GPS_H
#define GPS_H

#include "TinyGPSPlus.h"
#include <Arduino.h>

struct GPSCoord {
	double lat = 0.0, lng = 0.0;
	bool valid = false;

	GPSCoord() = default;
	GPSCoord(double lat, double lng, bool valid = true) : lat(lat), lng(lng), valid(valid) {}

	inline GPSCoord &operator=(const GPSCoord &other) {
		lat = other.lat;
		lng = other.lng;
		valid = other.valid;
		return *this;
	}
};

class GPSData {
private:
	static GPSCoord baseCoord;
	GPSCoord coord;

public:
	GPSData() = default;
	GPSData(GPSCoord coord) : coord(coord) {}

	GPSCoord getCoord() const { return coord; }
	static void setBaseCoord(GPSCoord base);
	double distanceToBase();
	double courseToBase();

	GPSData &operator=(const GPSData &other);

	static GPSData obtainGPSData(TinyGPSPlus &gps);
};

#endif // GPS_H
