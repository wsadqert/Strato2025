#include "gps.h"

GPSCoord GPSData::baseCoord;

void GPSData::setBaseCoord(GPSCoord base) {
	baseCoord = GPSCoord(base.lat, base.lng);
}

// https://chatgpt.com/share/68c2bde3-65c4-8013-91cf-6b9322921e12
double GPSData::distanceToBase() {
	return TinyGPSPlus::distanceBetween(coord.lat, coord.lng, baseCoord.lat, baseCoord.lng);
}

double GPSData::courseToBase() {
	return TinyGPSPlus::courseTo(coord.lat, coord.lng, baseCoord.lat, baseCoord.lng);
}

GPSData GPSData::obtainGPSData(TinyGPSPlus &gps) {
	// Obtain GPS data from the TinyGPSPlus object
	if (gps.location.isUpdated()) {
		GPSCoord newCoord(gps.location.lat(), gps.location.lng(), gps.location.isValid());
		return GPSData(newCoord);
	}
	return GPSData();
}

GPSData &GPSData::operator=(const GPSData &other) {
	this->coord = other.coord;
	return *this;
}
