#ifndef GPS_H
#define GPS_H

#include <Arduino.h>
#include "TinyGPSPlus.h"

struct GPSCoord {
	double lat, lng, height;
	bool valid = true;
	GPSCoord() : lat(0.0), lng(0.0), height(0.0), valid(false) {}
	GPSCoord(double lat, double lng) : lat(lat), lng(lng), height(0.0) {}
	GPSCoord(double lat, double lng, double height) : lat(lat), lng(lng), height(height) {}

	inline GPSCoord &operator=(const GPSCoord &other) {
		lat = other.lat;
		lng = other.lng;
		height = other.height;
		return *this;
	}
};

class GPSData {
private:
	static GPSCoord baseCoord;
	GPSCoord coord;
public:
	GPSData(GPSCoord coord) : coord(coord) {}
	GPSData() : coord(GPSCoord()) {}

	GPSCoord getCoord() const { return coord; }
	static void setBaseCoord(GPSCoord base, double height);
	double distanceToBase();
	double courseToBase();

	GPSData &operator=(const GPSData &other) {
		this->coord = other.coord;
		return *this;
	}

	static GPSData obtainGPSData(TinyGPSPlus gps);
};

#endif // GPS_H
