#include <Arduino.h>

void setup() {
	// Initialize both serial ports: 115200 baud, 8 data bits, no parity, 1 stop bit
	Serial.begin(115200, SERIAL_8N1);
	Serial2.begin(115200, SERIAL_8N1);
}

void loop() {
	// Forward from Serial to Serial2
	while (Serial.available() > 0) {
		int byteIn = Serial.read();
		Serial2.write(byteIn);
	}

	// Forward from Serial2 to Serial
	while (Serial2.available() > 0) {
		int byteIn = Serial2.read();
		Serial.write(byteIn);
	}
}
