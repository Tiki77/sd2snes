/* sd2snes - SD card based universal cartridge for the SNES

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; version 2 of the License only.

   nesave.h: battery-backed PRG-RAM of the NES core (load + autosave)
*/

#ifndef NESAVE_H
#define NESAVE_H

#include "config.h"
#include <stdint.h>

/* The core's PRG-RAM lives in PSRAM at this address (lockstep with nes.h
   NES_PSRAM_CARTRAM_ADDR and the CART-RAM map in nes_wrap.v) and is 8 KB wide:
   every save is a prefix of this window. */
#define NESAVE_WINDOW_ADDR   (0x3C0000L)
#define NESAVE_WINDOW_SIZE   (8192UL)

/* Autosave pacing.  The core owns the PSRAM bus while it renders, so the scan is
   chunked and rate limited: one NESAVE_CHUNK read per NESAVE_PERIOD_MS, i.e. a
   full pass of the window every two periods.  A change is written once it has
   been stable for NESAVE_STABLE_PASSES passes (~1.2 s after the last write by
   the game). */
#define NESAVE_CHUNK         (4096UL)
#define NESAVE_PERIOD_MS     (200)
#define NESAVE_STABLE_PASSES (3)

#ifdef CONFIG_MK2
/* No NES core on the Mk.II: everything compiles away. */
#define nesave_attach(f, b, n, h)  ((void)0)
#define nesave_detach()            ((void)0)
#define nesave_tick(f)             ((void)0)
#define nesave_flush_inreset(f)    ((void)0)
#else
/* Bytes of PRG-RAM a cart persists: 0 = no battery.  iNES 1.0 does not say, so a
   battery means the whole 8 KB window; NES 2.0 byte 10 high nibble (PRG-NVRAM
   shift) narrows it, capped at the window. */
uint32_t nesave_size(uint8_t has_battery, uint8_t is_nes20, uint8_t flags10);

/* Game load (after the core's RAM was cleared): pull <stem>.srm into the window
   (bounded to the persisted size, a longer file is cut) and seed the autosave
   baseline.  No battery disarms everything. */
void nesave_attach(uint8_t *filename, uint8_t has_battery, uint8_t is_nes20, uint8_t flags10);

/* Start of EVERY load_rom (SNES, menu, other consoles included): forget the previous
   game's battery.  load_saveram only runs when the caller asks for SRAM, so without
   this a stale window would be scanned and written under the next game's name. */
void nesave_detach(void);

/* Autosave step for the in-game loops (snes_main_loop AND msu1_loop's
   msu_savecheck: two game loops).  Self rate-limited; bounded work per call. */
void nesave_tick(uint8_t *filename);

/* Reset to menu / MSU short reset: the SNES is already held in reset, so read raw
   and write anything not yet on the card.  Checks the FPGA is alive itself. */
void nesave_flush_inreset(uint8_t *filename);
#endif

#endif
