#ifndef GPIOTOOL_H
#define GPIOTOOL_H

#include <Arduino.h>

class GPIOTool {
public:
	GPIOTool(uint16_t pin = 0);
	void on();
	void off();
	void pulse(uint32_t delayMs);
	bool isOn() const;
private:
	uint16_t pin;
	bool state = false;
};

#endif // GPIOTOOL_H
