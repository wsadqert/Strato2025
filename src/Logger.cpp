#include "Logger.h"

#include "gps.h"
#include "telemetry.h"

void Logger::waitAllSerials() {
	for (uint8_t i = 0; i < N; ++i)
		if (_serials[i] && _states[i] == Enabled)
			while (!(*_serials[i]));
}

void Logger::begin(SerialStream stream, HardwareSerial &serial, uint32_t baudrate, byte parity) {
	_serials[stream] = &serial;
	_serials[stream]->begin(baudrate, parity);
	_loglevels[stream] = DEBUG;
	_states[stream] = Enabled;
}

void Logger::setLogLevel(SerialStream stream, LoggingLevel loglevel) {
	_loglevels[stream] = loglevel;
}

void Logger::setState(SerialStream stream, State state) {
	_states[stream] = state;
}

void Logger::setState(SerialStream stream, bool enabled) {
	_states[stream] = enabled ? Enabled : Disabled;
}

void Logger::echo(String text) {
	write(text, ECHO);
}

void Logger::debug(String text) {
	write(text, DEBUG);
}

void Logger::info(String text) {
	write(text, INFO);
}

void Logger::warn(String text) {
	write(text, WARNING);
}

void Logger::error(String text) {
	write(text, ERROR);
}

void Logger::logTelemetry(const telemetry &t) {
	/*
	example output:
	[12345] [INFO]: GPS: lat=0.000000, lon=0.000000, valid=0
	[12345] [INFO]: DS18B20: temp_internal=0.00, temp_accum=0.00, temp_motor=0.00
	[12345] [INFO]: MPU6050: ax=0.00, ay=0.00, az=0.00, gx=0.00, gy=0.00, gz=0.00, temp=0.00
	[12345] [INFO]: HMC5883: mx=0.0000, my=0.0000, mz=0.0000
	[12345] [INFO]: MS5611: pressure=0.00, temp=0.00, altitude=0.00
	[12345] [INFO]: Heaters: motor_on=0, accum_on=0
	[12345] [INFO]: Flags: antenna_expanded=0
	[12345] [INFO]: Buttons: 000000
	*/

	GPSCoord coord = t.gpsData.getCoord();
	info(String(F("GPS: lat=")) + String(coord.lat, 6) + F(", lon=") + String(coord.lng, 6) + F(", valid=") + String(coord.valid));
	info(String(F("DS18B20: temp_internal=")) + String(t.temp_ds[0], 2) + F(", temp_accum=") + String(t.temp_ds[1], 2) + F(", temp_motor=") + String(t.temp_ds[2], 2));
	info(String(F("MPU6050: ax=")) + String(t.accel.x, 2) + F(", ay=") + String(t.accel.y, 2) + F(", az=") + String(t.accel.z, 2) + F(", gx=") + String(t.gyro.x, 2) + F(", gy=") + String(t.gyro.y, 2) + F(", gz=") + String(t.gyro.z, 2) + F(", temp=") + String(t.temp_mpu, 2));
	info(String(F("HMC5883: mx=")) + String(t.mag.x, 4) + F(", my=") + String(t.mag.y, 4) + F(", mz=") + String(t.mag.z, 4));
	info(String(F("MS5611: pressure=")) + String(t.pressure, 2) + F(", temp=") + String(t.temp_baro, 2) + F(", altitude=") + String(t.height_baro, 2));
	info(String(F("Motors: motor1_angle=")) + String(t.motor1_angle, 2) + F(", motor2_angle=") + String(t.motor2_angle, 2));
	info(String(F("Heaters: motor_on=")) + String(t.is_heater_motor_on) + F(", accum_on=") + String(t.is_heater_accum_on));
	info(String(F("Flags: antenna_expanded=")) + String(t.is_antenna_expanded));
	
	String buttonStatesStr = "Buttons: ";
	for (size_t i = 0; i < BUTTONS_AMOUNT; ++i)
		buttonStatesStr += String(t.button_states[i]);
	info(buttonStatesStr);
}

void Logger::write(String text, LoggingLevel loglevel) {
	for (uint8_t streamNum = 0; streamNum < N; ++streamNum)
		write(text, loglevel, (SerialStream)streamNum);
}

void Logger::write(String text, LoggingLevel loglevel, SerialStream stream) {
	if (_states[stream] == Enabled && loglevel >= _loglevels[stream])
		_serials[stream]->println(Logger::formatEntry(text, loglevel));
}

String Logger::formatEntry(String text, LoggingLevel loglevel) {
	// example output:
	// [<timestamp>] [<level>]: <message>
	return "[" + String(millis()) + "] [" + Logger::logLevelToString(loglevel) + "]: " + text;
}

String Logger::logLevelToString(LoggingLevel loglevel) {
	switch (loglevel) {
		case ECHO: return F("ECHO");
		case DEBUG: return F("DEBUG");
		case INFO: return F("INFO");
		case WARNING: return F("WARNING");
		case ERROR: return F("ERROR");
		default: return F("UNKNOWN");
	}
}
