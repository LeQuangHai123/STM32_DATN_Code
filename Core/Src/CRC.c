#include"CRC.h"

void build_crc16_table()
{
	for (uint16_t i = 0; i < 256; i++) {
		uint16_t ch = i;
		uint16_t crc = 0;
		for (uint8_t j = 0; j < 8; j++) {
			uint8_t b = (ch ^ crc) & 0x01;
			crc >>= 1;
			if (b)
				crc = crc ^ 0xA001;
			ch >>= 1;
		}
		crc16_table[i] = crc;
	}
}

uint16_t CRC16(uint8_t* input, int size)
{
	uint16_t crc = 0xFFFF;
	for (uint8_t i = 0; i < size; i++) {
		uint8_t ch = input[i];
		uint16_t t = (ch ^ crc) & 0xFF;
		crc = (crc >> 8) ^ crc16_table[t];
	}
	return ~crc;
}


uint16_t CRC16_Modbus(uint8_t* input, int size)
{
	uint16_t crc = 0xFFFF;
	for (uint8_t i = 0; i < size; i++) {
		uint8_t ch = input[i];
		uint16_t t = (ch ^ crc) & 0xFF;
		crc = (crc >> 8) ^ crc16_table[t];
	}
	return crc;
}
