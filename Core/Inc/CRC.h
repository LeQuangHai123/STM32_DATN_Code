#ifndef CRC_H_
#define CRC_H_
#include"stdint.h"

static uint16_t crc16_table[256];
void build_crc16_table(void);
uint16_t CRC16(uint8_t* input, int size);
uint16_t CRC16_Modbus(uint8_t* input, int size);
#endif


