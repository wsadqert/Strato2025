#ifndef BUTTONARRAY_H
#define BUTTONARRAY_H

#include <Arduino.h>

template <uint16_t N>
class ButtonArray {
public:
    ButtonArray(const uint8_t (&pins)[N]) {
		for (uint16_t i = 0; i < N; ++i)
			buttonPins[i] = pins[i];
	}
    void begin() {
		// set pinmodes
		for (uint16_t i = 0; i < N; ++i)
			pinMode(buttonPins[i], INPUT_PULLUP);
	}
    void getStates(bool (&states)[N]) {
		for (uint16_t i = 0; i < N; ++i)
			states[i] = (digitalRead(buttonPins[i]) == HIGH); // pressed = LOW
	}
private:
    uint8_t buttonPins[N];
};

#endif // BUTTONARRAY_H
