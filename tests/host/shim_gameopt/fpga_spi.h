/* Host shim for src/fpga_spi.h, used ONLY by run_gameopt.sh: the two feature
 * bits the in-game menu changes live and the calls that carry them to the
 * FPGA, recorded by gameopt_cli.c.  The FEAT_* values are copied out of the REAL
 * header by run_gameopt.sh into build/gameopt_feat.h, so there is no second copy to drift. */
#ifndef HOST_GAMEOPT_FPGA_SPI_H
#define HOST_GAMEOPT_FPGA_SPI_H

#include <stdint.h>

#include "gameopt_feat.h"   /* FEAT_2100 / FEAT_2100_LIMIT, generated from the real header */

extern uint16_t current_features;
void fpga_set_dac_boost(uint8_t boost);
void fpga_set_features(uint16_t feat);

#endif
