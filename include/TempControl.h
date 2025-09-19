#ifndef TEMP_CONTROL_H
#define TEMP_CONTROL_H

#include <Arduino.h>
#include "GPIOTool.h"
#include "Logger.h"

class TempControl {
private:
	static Logger _logger;

public:
	TempControl() {};

	static void setLogger(Logger logger);
	static void apply(GPIOTool &heater, float currentTemp, float minTemp, String controlName);
};

#endif // TEMP_CONTROL_H
