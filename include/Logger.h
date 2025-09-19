#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>
#include "telemetry.h"

class Logger {
public:
	static constexpr size_t N = 3;
public:
	enum LoggingLevel {
		ECHO = 0,
		DEBUG,
		INFO,
		WARNING,
		ERROR
	};
	enum SerialStream {
		LoRa,
		SD,
		USB
	};
    enum State {
        Disabled = 0,
        Enabled = 1
    };

	Logger() {};
	void begin(SerialStream stream, HardwareSerial& serial, uint32_t baudrate, byte parity);
	void setLogLevel(SerialStream stream, LoggingLevel loglevel);
	void setState(SerialStream stream, State state);
	void setState(SerialStream stream, bool enabled);
	void echo(String text);
	void debug(String text);
	void info(String text);
	void warn(String text);
	void error(String text);
	void logTelemetry(const telemetry& t);
	void write(String text, LoggingLevel loglevel);
	void write(String text, LoggingLevel loglevel, SerialStream stream);
	void waitAllSerials();

private:
	HardwareSerial* _serials[N];
	LoggingLevel _loglevels[N];
    State _states[N];
	static String logLevelToString(LoggingLevel loglevel);
};

#endif // LOGGER_H
