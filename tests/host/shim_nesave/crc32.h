#ifndef HOST_CRC32_H
#define HOST_CRC32_H
#include <stdint.h>
uint32_t crc32_update(uint32_t crc, const unsigned char data);
#endif
