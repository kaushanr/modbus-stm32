#ifndef ADL3000_CRC_H_
#define ADL3000_CRC_H_

#include <adl3000_modbus.h>
#include "stdint.h"

uint16_t crc16_1(uint8_t *buffer, uint16_t buffer_length);
uint16_t crc16_2(unsigned char *buffer, uint16_t start_pos, uint16_t num_elements);
uint16_t crc16_3(unsigned char crc_lo, unsigned char crc_hi);

#endif

