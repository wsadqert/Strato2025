#include <Arduino.h>
#include "GPIOTool.h"

GPIOTool::GPIOTool(uint16_t _pin) : pin(_pin) {
	if (pin == 0) return; // no pin set
	
	pinMode(pin, OUTPUT);
	digitalWrite(pin, LOW);
}

void GPIOTool::on() {
	digitalWrite(pin, HIGH);
	state = true;
}

void GPIOTool::off() {
	digitalWrite(pin, LOW);
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
