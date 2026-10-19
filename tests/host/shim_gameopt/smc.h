/* Host shim for src/smc.h, used ONLY by run_gameopt.sh: what gameopt.c reads of a game's
 * identification (its feature word, the chips that pick options) and the two calls it
 * makes, recorded by gameopt_cli.c. */
#ifndef HOST_GAMEOPT_SMC_H
#define HOST_GAMEOPT_SMC_H

#include <stdint.h>

typedef struct {
  uint8_t  has_msu1;
  uint8_t  has_cx4;
  uint8_t  has_gsu;
  uint16_t fpga_features;
} snes_romprops_t;

void smc_set_file_span(uint32_t rom_size);
void smc_id(snes_romprops_t *props, uint32_t file_offset);

#endif
