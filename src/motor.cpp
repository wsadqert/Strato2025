#include "motor.h"
#include "GyverDS18Array.h"

// Define static members
GyverDS18Array* Motor::_ds18b20 = nullptr;
uint8_t Motor::_ds18b20_idx = 0;

void Motor::setHeaterPin(uint16_t pin) {
	heater = GPIOTool(pin);
}

float Motor::getTemp() {
	if (_ds18b20->readTemp(_ds18b20_idx))
		return _ds18b20->getTemp();
	else
		return NAN;
}

void Motor::setDS(GyverDS18Array* ds18b20, uint8_t idx) {
	_ds18b20 = ds18b20;
	_ds18b20_idx = idx;
}
