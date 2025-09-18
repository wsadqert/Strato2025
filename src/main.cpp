// Scanning...
// I2C device found at address 0x0D  !
// I2C device found at address 0x68  !
// I2C device found at address 0x77  !
// done

#include <Arduino.h>
#include <Wire.h>

#include "Adafruit_Sensor.h"

#include "A4988.h"			  // motors
#include "Adafruit_MPU6050.h" // accel + gyro
#include "DFRobot_QMC5883.h"  // mag
#include "GyverDS18Array.h"	  // ds
#include "MS5611.h"			  // baro
#include "TinyGPSPlus.h"	  // gps

#include "gps.h"
#include "logger.h"
#include "telemetry.h"
#include "utils.h"

#define DISABLE_LORA 1

// ---------- SERIAL ----------
#define USB_UART Serial
#define USB_BAUDRATE 115200
#define USB_PARITY SERIAL_8E1

#define LORA_UART Serial1
#define LORA_BAUDRATE 9600
#define LORA_PARITY SERIAL_8N1

#define SD_UART Serial2
#define SD_BAUDRATE 9600	 /// TODO: set real baudrate
#define SD_PARITY SERIAL_8N1 // TODO: set real parity

#define GPS_UART Serial3
#define GPS_BAUDRATE 9600
#define GPS_PARITY SERIAL_8N1

Logger logger;
TinyGPSPlus gps;

// ------------ DS ------------
#define DS_PIN 2
#define DS_AMOUNT 3

// TODO: set real DS addresses
const uint8_t ds18b20_addresses[][8] = {
	{0x28, 0xFF, 0x1C, 0xA2, 0x64, 0x16, 0x03, 0x5C},
	{0x28, 0xFF, 0x4B, 0xC1, 0x64, 0x16, 0x03, 0xA2},
	{0x28, 0xFF, 0x7D, 0xB3, 0x64, 0x16, 0x03, 0xF1}};

GyverDS18Array ds(DS_PIN);

// ---------- MOTORS ----------
#define MOTOR_STEPS 200

// TODO: set real motor pins
#define DIR_ALT_PIN 8
#define STEP_ALT_PIN 9
#define MS1_ALT_PIN 10
#define MS2_ALT_PIN 11
#define MS3_ALT_PIN 12

#define DIR_AZ_PIN 8
#define STEP_AZ_PIN 9
#define MS1_AZ_PIN 10
#define MS2_AZ_PIN 11
#define MS3_AZ_PIN 12

A4988 motor_altitude(MOTOR_STEPS, DIR_ALT_PIN, STEP_ALT_PIN, MS1_ALT_PIN, MS2_ALT_PIN, MS3_ALT_PIN);
A4988 motor_azimuth(MOTOR_STEPS, DIR_AZ_PIN, STEP_AZ_PIN, MS1_AZ_PIN, MS2_AZ_PIN, MS3_AZ_PIN);

// GY-86
#define QMC5883_I2C_ADDRESS 0x0D
// 12° 2' E in radians,  Moscow
// don't change!
const float declinationAngle = (12.0 + (2.0 / 60.0)) / (180 / PI);

Adafruit_MPU6050 mpu;
DFRobot_QMC5883 mag(&Wire, QMC5883_I2C_ADDRESS);
MS5611 baro;

void setup() {
	// COMMUNICATION
	Wire.begin();

	logger.begin(Logger::SerialStream::USB, USB_UART, USB_BAUDRATE, USB_PARITY);
	logger.begin(Logger::SerialStream::SD, SD_UART, SD_BAUDRATE, SD_PARITY);
	logger.begin(Logger::SerialStream::Radio, LORA_UART, LORA_BAUDRATE, LORA_PARITY);

	logger.setLogLevel(Logger::SerialStream::USB, Logger::DEBUG);
	logger.setLogLevel(Logger::SerialStream::SD, Logger::DEBUG);
	logger.setLogLevel(Logger::SerialStream::Radio, Logger::INFO);

#if DISABLE_LORA
	logger.setState(Logger::SerialStream::Radio, Logger::Disabled);
#endif

	logger.waitAllSerials();

	GPS_UART.begin(GPS_BAUDRATE);

	// TODO: set real base coord
	GPSData::setBaseCoord(GPSCoord(55.753215, 37.622504), 0); // Moscow Kremlin
	delay(100);

	logger.info(F("Logger started"));

	// DS
	uint64_t ds18b20_addr64[DS_AMOUNT];
	bytesToUint64(ds18b20_addresses, ds18b20_addr64, DS_AMOUNT);
	ds.setAddress(ds18b20_addr64, DS_AMOUNT);
	ds.requestTemp();
	logger.debug(F("ds18b20 init ok!"));

	// MOTORS
	motor_altitude.begin();
	motor_azimuth.begin();
	logger.debug(F("motors init ok!"));

	// GY-86
	if (mpu.begin())
		logger.debug(F("mpu6050 (accel/gyro) init ok"));
	else
		logger.error(F("mpu6050 (accel/gyro) init error"));

	mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
	mpu.setGyroRange(MPU6050_RANGE_500_DEG);
	mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
	logger.debug(F("mpu6050 settings done"));

	if (mag.begin())
		logger.debug(F("qmc5883 (mag) init ok"));
	else
		logger.error(F("qmc5883 (mag) init error"));

	mag.setDeclinationAngle(declinationAngle);

	if (baro.begin())
		logger.debug(F("ms5611 (baro) init ok"));
	else
		logger.error(F("ms5611 (baro) init error"));

	logger.info(F("Setup done"));
}

void loop() {
	telemetry t;

	// GPS
	while (GPS_UART.available())
		if (gps.encode(GPS_UART.read()))
			t.gpsData = GPSData::obtainGPSData(gps);

	// DS
	if (ds.ready()) {
		for (int i = 0; i < DS_AMOUNT; i++)
			if (ds.readTemp(i))
				t.temp_ds[i] = ds.getTemp();

		ds.requestTemp();
	}

	// MOTORS
	// motor_altitude.rotate(90);

	// MPU
	sensors_event_t accel, gyro, temp_mpu;
	mpu.getEvent(&accel, &gyro, &temp_mpu);

	sVector_t mag_data = mag.readRaw();

	t.accel = xyz_data<float>(accel.acceleration.x, accel.acceleration.y, accel.acceleration.z);
	t.gyro = xyz_data<float>(gyro.gyro.x, gyro.gyro.y, gyro.gyro.z);
	t.mag = xyz_data<float>(mag_data.XAxis, mag_data.YAxis, mag_data.ZAxis);

	// BARO
	baro.read();
	t.temp_baro = baro.getTemperature();
	t.pressure = baro.getPressurePascal();
	t.altitude_baro = baro.getAltitude(101325); // using standard pressure at sea level

	// LOG
	logger.logTelemetry(t);

	delay(1000);
}
