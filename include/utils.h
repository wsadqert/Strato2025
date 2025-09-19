#ifndef TWEAKS_H
#define TWEAKS_H

#include <Arduino.h>

void bytesToUint64(const uint8_t src[][8], uint64_t* dst, uint8_t amount);
uint64_t singleBytesToUint64(const uint8_t src[8]);

#endif // TWEAKS_H
