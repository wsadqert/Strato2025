#include "TempControl.h"

Logger TempControl::_logger;

void TempControl::setLogger(Logger logger) {
	_logger = logger;
}

void TempControl::apply(GPIOTool &heater, float currentTemp, float minTemp, String controlName) {
	static bool prevState = false; // false = OFF, true = ON

	bool newState = currentTemp < minTemp;
	if (newState != prevState) {
		if (newState) {
			heater.on();
			_logger.warn(controlName + F(" turned ON"));
		} else {
			heater.off();
			_logger.warn(controlName + F(" turned OFF"));
		}
		prevState = newState;
	}
}
