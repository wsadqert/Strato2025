#include "motor.h"
#include "GyverDS18Array.h"

// Define static members
GyverDS18Array *Motor::_ds18b20 = nullptr;
uint8_t Motor::_ds18b20_idx = 0;

void Motor::rotate(double angle) {
	currentAngle += angle;
	currentAngle = fmod(currentAngle, 360.0); // keep it within 0-360
	A4988::rotate(angle);
}

void Motor::rotateTo(double angle) {
	double delta = angle - currentAngle;

	// Normalize delta to the range [-180, 180]
	if (delta > 180.0)
		delta -= 360.0;
	else if (delta < -180.0)
		delta += 360.0;
	
	rotate(delta);
}

float Motor::getCurrentAngle() {
	return currentAngle;
}

void Motor::setHeaterPin(uint16_t pin) {
	heater = GPIOTool(pin);
}

float Motor::getTemp() {
	if (_ds18b20->readTemp(_ds18b20_idx))
		return _ds18b20->getTemp();
	else
		return NAN;
}

void Motor::setDS(GyverDS18Array *ds18b20, uint8_t idx) {
	_ds18b20 = ds18b20;
	_ds18b20_idx = idx;
}
