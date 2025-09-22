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

#include "ButtonArray.h"
#include "TempControl.h"
#include "gps.h"
#include "logger.h"
#include "motor.h"
#include "telemetry.h"
#include "utils.h"

#define DISABLE_LORA 0
#define DISABLE_GY86 1

// ---------- SERIAL ----------
#define USB_UART Serial
#define USB_BAUDRATE 115200
#define USB_PARITY SERIAL_8E1

#define LORA_UART Serial1
#define LORA_BAUDRATE 115200
#define LORA_PARITY SERIAL_8E1

#define SD_UART Serial2
#define SD_BAUDRATE 9600
#define SD_PARITY SERIAL_8N1

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

#define DS_INTERNAL_IDX 0
#define DS_ACCUM_IDX 1
#define DS_MOTOR_IDX 2

uint64_t ds18b20_addresses_64[DS_AMOUNT] = {
	// 0xA7A8CD2B34646128,  // internal - in baro
	0x7000000095837E28, // internal - orange
	0x5F00000097E30628, // Accum
	0xC5511DD434646128, // Motor
};

GyverDS18Array ds(DS_PIN);

// ---------- MOTORS ----------
#define MOTOR_STEPS 200

#define DIR_ALT_PIN 2
#define STEP_ALT_PIN 3
#define MS1_ALT_PIN 4
#define MS2_ALT_PIN 5
#define MS3_ALT_PIN 6

#define DIR_AZ_PIN A5
#define STEP_AZ_PIN A6
#define MS1_AZ_PIN A7
#define MS2_AZ_PIN A8
#define MS3_AZ_PIN A9

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
#define RELAY_PIN A10

GPIOTool relay(RELAY_PIN, true); // true = inverted

// warning: this will block everything for 5 seconds
void expandAntenna() {
	relay.pulse(4000);
	logger.info(F("Antenna expanded"));
}

// ---------- BUTTONS ---------
// const uint8_t buttonPins[BUTTONS_AMOUNT] = {44, 45, 46, 47, 48, 49};
const uint8_t buttonPins[BUTTONS_AMOUNT] = {7, A7, A8, A9, 44, 49};

#define BUTTONS_COLS_AMOUNT 3
#define BUTTONS_ROWS_AMOUNT 3

const uint8_t buttonColsPins[BUTTONS_COLS_AMOUNT] = {50, 51, 52};
const uint8_t buttonRowsPins[BUTTONS_ROWS_AMOUNT] = {A2, A3, A4};

void readButtons(bool buttonstate[][BUTTONS_COLS_AMOUNT]) {
	for (uint8_t i = 0; i < BUTTONS_ROWS_AMOUNT; i++) {
		pinMode(buttonRowsPins[i], INPUT);
		digitalWrite(buttonRowsPins[i], HIGH); // Enable the internal 20K pullup resistors.
	}

	for (uint8_t c = 0; c < BUTTONS_COLS_AMOUNT; c++) {
		pinMode(buttonColsPins[c], OUTPUT);	  // Prepare column pin for sending a pulse.
		digitalWrite(buttonColsPins[c], LOW); // Pulse the pin colPins[c] low.
		for (uint8_t r = 0; r < BUTTONS_ROWS_AMOUNT; r++) {
			if (!digitalRead(buttonRowsPins[r])) // A key is PRESSED on row r, column c
				buttonstate[r][c] = 1;			 // Store PRESSED keys.
			else
				buttonstate[r][c] = 0; // Store OPEN keys.
		}
		digitalWrite(buttonColsPins[c], HIGH); // End the pulse
		pinMode(buttonColsPins[c], INPUT);	   // Prevent shorts between columns when multiple keys are pressed.
	}
}

// ButtonArray<BUTTONS_AMOUNT> buttons(buttonPins);

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
GPSCoord baseCoord(55.753215, 37.622504);

telemetry t;
// Antenna expansion status flag
bool isAntennaExpanded = false;

void setup() {
	// COMMUNICATION
	Wire.begin();

	logger.begin(Logger::SerialStream::USB, USB_UART, USB_BAUDRATE, USB_PARITY);
	logger.begin(Logger::SerialStream::SD, SD_UART, SD_BAUDRATE, SD_PARITY);
	logger.begin(Logger::SerialStream::LoRa, LORA_UART, LORA_BAUDRATE, LORA_PARITY);

	logger.setLogLevel(Logger::SerialStream::USB, Logger::ECHO);
	logger.setLogLevel(Logger::SerialStream::SD, Logger::ECHO);
	logger.setLogLevel(Logger::SerialStream::LoRa, Logger::ECHO);

#if DISABLE_LORA
	logger.setState(Logger::SerialStream::LoRa, Logger::Disabled);
#endif

	logger.waitAllSerials();
	GPS_UART.begin(GPS_BAUDRATE);
	delay(1000); // wait for sd logger init

	GPSData::setBaseCoord(baseCoord); // Moscow Kremlin
	logger.info(F("Logger started"));

	// PINS
	pinMode(DS_PIN, INPUT);
	// relay pin is set in GPIOTool
	// motors pins are set in A4988 lib
	// buttons.begin(); // <-- button pins set here

	logger.debug(F("buttons init ok!"));

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

#if !DISABLE_GY86
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

#endif // !DISABLE_GY86

	logger.info(F("Setup done"));

	/*
	example output:
	[12345] [INFO]: Logger started
	[12346] [DEBUG]: buttons init ok!
	[12347] [DEBUG]: ds18b20 init ok!
	[12348] [DEBUG]: motors init ok!
	[12349] [DEBUG]: mpu6050 (accel/gyro) init ok
	[12350] [DEBUG]: mpu6050 settings done
	[12351] [DEBUG]: qmc5883 (mag) init ok
	[12352] [DEBUG]: ms5611 (baro) init ok
	[12353] [INFO]: Setup done
	*/
}

void loop() {
	// BUTTONS
	// buttons.getStates(t.button_states);

	bool buttonstate[BUTTONS_ROWS_AMOUNT][BUTTONS_COLS_AMOUNT];
	readButtons(buttonstate);
	// print button states for debug
	// copy to telemetry struct
	for (uint8_t r = 0; r < BUTTONS_ROWS_AMOUNT; r++)
		for (uint8_t c = 0; c < BUTTONS_COLS_AMOUNT; c++)
			t.button_states[r * BUTTONS_COLS_AMOUNT + c] = buttonstate[r][c];

	// GPS
	while (GPS_UART.available())
		if (gps.encode(GPS_UART.read()))
			t.gpsData = GPSData::obtainGPSData(gps);

	if (!t.gpsData.getCoord().valid)
		logger.error(F("No GPS data"));
	else {
		t.gpsData.distanceToBase();
	}

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
	motor_azimuth.rotate(45);

	// TEMP CONTROL
	TempControl::apply(heater_accum, t.temp_ds[DS_ACCUM_IDX], targetAccumTemp, F("Accumulator"));
	TempControl::apply(motor_altitude.heater, t.temp_ds[DS_MOTOR_IDX], targetMotorTemp, F("Motor Altitude"));
	TempControl::apply(motor_azimuth.heater, t.temp_ds[DS_MOTOR_IDX], targetMotorTemp, F("Motor Azimuth"));

	t.is_heater_motor_on = motor_altitude.heater.isOn(); // equivalent to `motor_azimuth.heater.isOn()` bcz of same state
	t.is_heater_accum_on = heater_accum.isOn();

// MPU
#if !DISABLE_GY86
	sensors_event_t accel, gyro, temp_mpu;
	mpu.getEvent(&accel, &gyro, &temp_mpu);

	// sVector_t mag_data;
	sVector_t mag_data = mag.readRaw();

	t.accel = xyz_data<float>(accel.acceleration.x, accel.acceleration.y, accel.acceleration.z);
	t.gyro = xyz_data<float>(gyro.gyro.x, gyro.gyro.y, gyro.gyro.z);
	t.mag = xyz_data<float>(mag_data.XAxis, mag_data.YAxis, mag_data.ZAxis);
	t.temp_mpu = temp_mpu.temperature;

	// BARO
	baro.read();
	t.temp_baro = baro.getTemperature();
	t.pressure = baro.getPressurePascal();
	t.height_baro = baro.getAltitude(seaLevelPressure); // using standard pressure at sea level
#endif													// !DISABLE_GY86

	// AIMING
	if (t.height_baro < 0) t.height_baro = 0.1; // to avoid NaN

	float elevationAngle = atan2(t.gpsData.distanceToBase(), t.height_baro) * 180.0 / PI;
	logger.debug(String() + F("Elevation angle: ") + String(elevationAngle, 2) + F("°"));

	motor_altitude.rotateTo(constrain(elevationAngle, 0.0, 45.0));

	// pinMode(A7, OUTPUT);
	// digitalWrite(A7, HIGH);

	t.motor1_angle = motor_altitude.getCurrentAngle();
	t.motor2_angle = motor_azimuth.getCurrentAngle();

	if (millis() / 1000 >= 20 * 60 && !isAntennaExpanded) {
		expandAntenna();
		isAntennaExpanded = true;
	}

	// LOG
	logger.logTelemetry(t);

	delay(1000);
}

void processCommand(const String &command) {
	if (command.length() == 0) return;
	logger.echo(String() + F("Received command: ") + command);

	if (command == "EXPAND_ANTENNA") {
		expandAntenna();
	} else if (command == "echo") {
		logger.echo(F("echo"));
	} else {
		logger.warn(String() + F("Unknown command: ") + command);
	}
}

// Attach to main loop via serialEvent
void serialEvent1() {
	static String cmdBuffer;

	while (COMMAND_UART.available()) {
		char c = COMMAND_UART.read();

		// assuming lines ended with \r\n (crlf) or \n (lf)

		if (c == '\r') {
		} // ignore
		else if (c == '\n') {
			cmdBuffer.trim();
			processCommand(cmdBuffer);
			cmdBuffer = "";
		} else
			cmdBuffer += c;
	}
};
