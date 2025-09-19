#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>
#include "Adafruit_Sensor.h"
#include "gps.h"

#include "xyz_data.h"

#define DS_AMOUNT 3

struct telemetry {
	float temp_ds[DS_AMOUNT];
	xyz_data<float> accel, gyro, mag;
	float temp_mpu, temp_baro;
	float pressure;
	float altitude_baro;
	GPSData gpsData;
	bool is_heater_motor_on;
	bool is_heater_accum_on;
};

#endif // TELEMETRY_H
