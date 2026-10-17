/* host stand-in for src/memory.h: only the names nesave.c takes from it (the PSRAM itself is reached
   through the fake FPGA of fpga_spi.h). */
#ifndef HOST_MEMORY_H
#define HOST_MEMORY_H
#include <stdint.h>
#include <stddef.h>
#define SRAM_SCRATCHPAD        (0xFFFF00L)
#define SRAM_RELIABILITY_SCORE (0x100)
int srm_path_for_load(char *buf, size_t buflen, uint8_t *filename);
int srm_path_for_save(char *buf, size_t buflen, uint8_t *filename);
#endif
