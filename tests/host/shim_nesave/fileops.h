/* host stand-in for src/fileops.h: the file calls nesave.c uses; the fake card lives in nesave_cli.c */
#ifndef HOST_FILEOPS_H
#define HOST_FILEOPS_H
#include <stdint.h>
#include <stddef.h>
#include "ff.h"
extern BYTE    file_buf[512];
extern FRESULT file_res;
void file_open(uint8_t *filename, BYTE flags);
void file_close(void);
UINT file_read(void);
UINT file_write(size_t len);
#endif
