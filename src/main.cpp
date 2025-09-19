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
#include "motor.h"
#include "telemetry.h"
#include "utils.h"
#include <TempControl.h>

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

#if DISABLE_LORA
#define COMMAND_UART USB_UART
#else
#define COMMAND_UART LORA_UART
#endif

Logger logger;
TinyGPSPlus gps;

// ------------ DS ------------
#define DS_PIN 10
#define DS_AMOUNT 3

// TODO: set real DS addresses

#define DS_INTERNAL_IDX 0
#define DS_MOTOR_IDX 1
#define DS_ACCUM_IDX 2

// const uint8_t ds18b20_addresses[DS_AMOUNT][8] = {
// 	{0x28, 0x61, 0x64, 0x34, 0x2B, 0xCD, 0xA8, 0xA7}, // Internal - in baro
// 	{0x28, 0x0D, 0x6C, 0x55, 0x00, 0x00, 0x00, 0x3D}, // Internal - in
// 	{0x28, 0xFF, 0x4B, 0xC1, 0x64, 0x16, 0x03, 0xA2}, // example

// 	// {0x28, 0xFF, 0x4B, 0xC1, 0x64, 0x16, 0x03, 0xA2}, // Motor
// 	// {0x28, 0xFF, 0x1C, 0xA2, 0x64, 0x16, 0x03, 0x5C}, // Accumulator
// 	// {0x28, 0xFF, 0x7D, 0xB3, 0x64, 0x16, 0x03, 0xF1}  // Internal
// };

uint64_t ds18b20_addresses_64[DS_AMOUNT] = {
	0xA7A8CD2B34646128,
	0x3D000000556C0D28,
	0xA2031664C14BFF28
};

GyverDS18Array ds(DS_PIN);

// ---------- MOTORS ----------
#define MOTOR_STEPS 200

#define DIR_ALT_PIN 2
#define STEP_ALT_PIN 3
#define MS1_ALT_PIN 4
#define MS2_ALT_PIN 5
#define MS3_ALT_PIN 6

#define DIR_AZ_PIN A2
#define STEP_AZ_PIN A3
#define MS1_AZ_PIN A4
#define MS2_AZ_PIN A5
#define MS3_AZ_PIN A6

Motor motor_altitude(MOTOR_STEPS, DIR_ALT_PIN, STEP_ALT_PIN, MS1_ALT_PIN, MS2_ALT_PIN, MS3_ALT_PIN);
Motor motor_azimuth(MOTOR_STEPS, DIR_AZ_PIN, STEP_AZ_PIN, MS1_AZ_PIN, MS2_AZ_PIN, MS3_AZ_PIN);

// ------- TEMP CONTROL --------
#define ACCUM_HEATER_PIN 11
#define MOTOR_HEATER_ALT_PIN 42
#define MOTOR_HEATER_AZ_PIN 43

float targetMotorTemp = 5.0; // degrees C
float targetAccumTemp = 5.0; // degrees C
GPIOTool heater_accum(ACCUM_HEATER_PIN);

// ---------- RELAY -----------
#define RELAY_PIN A2

GPIOTool relay(RELAY_PIN);

// warning: this will block everything for 5 seconds
void expandAntenna() {
	relay.pulse(5000);
	logger.info(F("Antenna expanded"));
}

// ---------- GY-86 -----------
#define QMC5883_I2C_ADDRESS 0x0D
// 12° 2' E in radians,  Moscow
// don't change!

Adafruit_MPU6050 mpu;
DFRobot_QMC5883 mag(&Wire, QMC5883_I2C_ADDRESS);
MS5611 baro;
float seaLevelPressure = 101325; // Pa

// ----------- GPS ------------
const float declinationAngle = (12.0 + (2.0 / 60.0)) / (180 / PI);

// TODO: set real base coord
GPSCoord baseCoord(55.753215, 37.622504, 0.0);

telemetry t;

void setup() {
	// PINS
	pinMode(DS_PIN, INPUT);
	// relay pin is set in GPIOTool
	// motors pins are set in A4988 lib

	// COMMUNICATION
	Wire.begin();

	logger.begin(Logger::SerialStream::USB, USB_UART, USB_BAUDRATE, USB_PARITY);
	logger.begin(Logger::SerialStream::SD, SD_UART, SD_BAUDRATE, SD_PARITY);
	logger.begin(Logger::SerialStream::LoRa, LORA_UART, LORA_BAUDRATE, LORA_PARITY);

	logger.setLogLevel(Logger::SerialStream::USB, Logger::ECHO);
	logger.setLogLevel(Logger::SerialStream::SD, Logger::ECHO);
	logger.setLogLevel(Logger::SerialStream::LoRa, Logger::INFO);

#if DISABLE_LORA
	logger.setState(Logger::SerialStream::LoRa, Logger::Disabled);
#endif

	logger.waitAllSerials();
	GPS_UART.begin(GPS_BAUDRATE);
	delay(100);

	GPSData::setBaseCoord(baseCoord); // Moscow Kremlin
	logger.info(F("Logger started"));

	// DS
	// uint64_t ds18b20_addr64[DS_AMOUNT];
	// bytesToUint64(ds18b20_addresses, ds18b20_addr64, DS_AMOUNT);
	ds.setAddress(ds18b20_addresses_64, DS_AMOUNT);
	ds.requestTemp();
	logger.debug(F("ds18b20 init ok!"));

	TempControl::setLogger(logger);

	// MOTORS
	motor_altitude.begin();
	motor_azimuth.begin();
	Motor::setDS(&ds, DS_MOTOR_IDX);
	motor_altitude.setHeaterPin(MOTOR_HEATER_ALT_PIN);
	motor_azimuth.setHeaterPin(MOTOR_HEATER_AZ_PIN);

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

	// if (mag.begin())
	// 	logger.debug(F("qmc5883 (mag) init ok"));
	// else
	// 	logger.error(F("qmc5883 (mag) init error"));

	mag.setDeclinationAngle(declinationAngle);

	if (baro.begin())
		logger.debug(F("ms5611 (baro) init ok"));
	else
		logger.error(F("ms5611 (baro) init error"));

	logger.info(F("Setup done"));
}

void loop() {
	// GPS
	while (GPS_UART.available())
		if (gps.encode(GPS_UART.read()))
			t.gpsData = GPSData::obtainGPSData(gps);

	if (!t.gpsData.getCoord().valid)
		logger.error(F("No GPS data"));

	// DS
	if (ds.ready()) {
		for (int i = 0; i < DS_AMOUNT; i++)
			if (ds.readTemp(i))
				t.temp_ds[i] = ds.getTemp();
			else
				logger.error(String() + F("DS read error: ") + String(i));

		ds.requestTemp();
	}

	// MOTORS

	// motor_altitude.rotate(90);
	// motor_azimutsh.rotate(45);

	// TEMP CONTROL
	TempControl::apply(heater_accum, t.temp_ds[DS_ACCUM_IDX], targetAccumTemp, "Accumulator");
	TempControl::apply(motor_altitude.heater, t.temp_ds[DS_MOTOR_IDX], targetMotorTemp, "Motor Altitude");
	TempControl::apply(motor_azimuth.heater, t.temp_ds[DS_MOTOR_IDX], targetMotorTemp, "Motor Azimuth");

	t.is_heater_motor_on = motor_altitude.heater.isOn(); // equivalent to `motor_azimuth.heater.isOn()` bcz of same state
	t.is_heater_accum_on = heater_accum.isOn();

	// MPU
	sensors_event_t accel, gyro, temp_mpu;
	mpu.getEvent(&accel, &gyro, &temp_mpu);

	// sVector_t mag_data = mag.readRaw();
	sVector_t mag_data;

	t.accel = xyz_data<float>(accel.acceleration.x, accel.acceleration.y, accel.acceleration.z);
	t.gyro = xyz_data<float>(gyro.gyro.x, gyro.gyro.y, gyro.gyro.z);
	t.mag = xyz_data<float>(mag_data.XAxis, mag_data.YAxis, mag_data.ZAxis);

	// BARO
	baro.read();
	t.temp_baro = baro.getTemperature();
	t.pressure = baro.getPressurePascal();
	t.altitude_baro = baro.getAltitude(seaLevelPressure); // using standard pressure at sea level

	// LOG
	logger.logTelemetry(t);

	delay(1000);
}

auto processCommand = [&](const String &cmd) {
	String command = cmd;

	if (command.length() == 0) return;

	logger.echo("Received command: " + command);

	if (command == "EXPAND_ANTENNA") {
		expandAntenna();
	} else if (command == "MOTOR_ALT_90") {
		motor_altitude.rotate(90);
	} else if (command == "MOTOR_AZ_45") {
		motor_azimuth.rotate(45);
	} else if (command == "GET_TEMP") {
		logger.echo("Motor temp: " + String(t.temp_ds[DS_MOTOR_IDX]));
		logger.echo("Accum temp: " + String(t.temp_ds[DS_ACCUM_IDX]));
	} else if (command == "echo") {
		logger.echo("echo");
	} else {
		logger.warn("Unknown command: " + command);
	}
};

// Attach to main loop via serialEvent
void serialEvent() {
	static String cmdBuffer;

	while (COMMAND_UART.available()) {
		char c = COMMAND_UART.read();
		
		// assuming lines ended with \r\n (crlf) or \n (lf)
		
		if (c == '\r') {}  // ignore
		else if (c == '\n') {
			cmdBuffer.trim();
			processCommand(cmdBuffer);
			cmdBuffer = "";
		} else
			cmdBuffer += c;
	}
};
