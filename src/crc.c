#include "crc.h"


uint16_t crc16(uint8_t byte, uint16_t crc) {
  for (int i = 0; i < 8; i++) {
    crc ^= byte & 1;
    crc = (crc & 1) ? (crc >> 1) ^ 0x8408 : (crc >> 1);
    byte = byte >> 1;
  }

  return crc;
}


uint16_t crc16_block(const uint8_t *data, unsigned len, uint16_t crc) {
  for (unsigned i = 0; i < len; i++)
    crc = crc16(data[i], crc);

  return crc;
}
