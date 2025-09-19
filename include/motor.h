#ifndef MOTOR_H
#define MOTOR_H

#include "A4988.h"
#include "GyverDS18Array.h"
#include "GPIOTool.h"
#include <Arduino.h>

class Motor : public A4988 {
private:
	static GyverDS18Array* _ds18b20;
	static uint8_t _ds18b20_idx;

public:
	Motor(short steps, short dir_pin, short step_pin, short ms1_pin, short ms2_pin, short ms3_pin)
		: A4988(steps, dir_pin, step_pin, ms1_pin, ms2_pin, ms3_pin) {};

	GPIOTool heater;

	void setHeaterPin(uint16_t pin);
	static void setDS(GyverDS18Array* ds18b20, uint8_t idx);
	static float getTemp();
};

#endif // MOTOR_H
