#include <Arduino.h>

#include "GPIOTool.h"

GPIOTool::GPIOTool(uint16_t pin, bool inverted) : _pin(pin), _inverted(inverted) {
	if (_pin == 0) return; // no pin set
	pinMode(_pin, OUTPUT);
	digitalWrite(_pin, _inverted ? HIGH : LOW);
}

void GPIOTool::on() {
	digitalWrite(_pin, _inverted ? LOW : HIGH);
	state = true;
}

void GPIOTool::off() {
	digitalWrite(_pin, _inverted ? HIGH : LOW);
	state = false;
}

void GPIOTool::pulse(uint32_t delayMs) {
	on();
	delay(delayMs);
	off();
}

bool GPIOTool::isOn() const {
	return state;
}
