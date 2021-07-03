#pragma once

#include <stdint.h>


uint16_t crc16(uint8_t byte, uint16_t crc);
uint16_t crc16_block(const uint8_t *data, unsigned len, uint16_t crc);
