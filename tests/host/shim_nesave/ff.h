#ifndef HOST_FF_H
#define HOST_FF_H
#include <stdint.h>
typedef unsigned int UINT;
typedef uint8_t BYTE;
typedef enum { FR_OK = 0, FR_DISK_ERR, FR_NO_FILE = 4, FR_INVALID_NAME = 6 } FRESULT;
#define FA_READ           0x01
#define FA_WRITE          0x02
#define FA_CREATE_ALWAYS  0x08
#endif
