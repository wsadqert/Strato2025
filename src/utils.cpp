#include <Arduino.h>

void bytesToUint64(const uint8_t src[][8], uint64_t* dst, uint8_t amount) {
	for (uint8_t i = 0; i < amount; i++) {
		dst[i] = 0;
		for (uint8_t j = 0; j < 8; j++) {
			dst[i] |= ((uint64_t)src[i][j] << (8 * (7 - j)));
		}
	}
}

uint64_t singleBytesToUint64(const uint8_t src[8]) {
	uint64_t dst = 0;
	for (uint8_t j = 0; j < 8; j++)
		dst |= ((uint64_t)src[j] << (8 * (7 - j)));
	
	return dst;
}
