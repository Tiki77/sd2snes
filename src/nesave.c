/* sd2snes - SD card based universal cartridge for the SNES

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; version 2 of the License only.

   nesave.c: battery-backed PRG-RAM of the NES core

   The core keeps its PRG-RAM in PSRAM (NESAVE_WINDOW_ADDR) and writes it directly,
   so the MCU sees the live contents with a plain read.  Load puts <stem>.srm back
   there; autosave is a chunked CRC scan with a debounce, the same shape as the
   Sufami Turbo Slot B battery (sufami.c) and for the same reason: the SaveRAM scan
   in snes.c watches SRAM_SAVE_ADDR and never sees this window.

   The scan is called from BOTH game loops (snes_main_loop and msu1_loop's
   msu_savecheck) and from the two reset paths (prepare_reset,
   save_during_msu_shortreset): a subsystem wired into only one of them is dead in
   the other.  All of it is bounded: one chunk read per call, one file write per
   save, and every wait on the FPGA is bounded (a game loop must never wedge here).
*/

#include "config.h"

#ifndef CONFIG_MK2

#include "uart.h"
#include "cfg.h"
#include "crc32.h"
#include "ff.h"
#include "fpga.h"
#include "fpga_spi.h"
#include "fileops.h"
#include "led.h"
#include "memory.h"
#include "snes.h"
#include "timer.h"
#include "nesave.h"

extern cfg_t CFG;

/* Every wait on the FPGA in this file is bounded (FPGA_WAIT_RDY_TO*, the precedent is
   nes_fpga_err in nes.c): the unbounded accessors of memory.c (calc_sram_crc,
   sram_reliable, save_srm, sram_writeblock) spin forever on a core that stopped driving
   MCU_RDY, and this code runs from the main loop of a running game.  sv_hw_err is the
   latch; a timeout abandons the operation in progress and never writes a file. */
static uint8_t  sv_hw_err;        /* an MCU_RDY wait timed out in the current operation */
static uint8_t  sv_bail;          /* the reset line was seen during a scan */
static uint8_t  sv_dead;          /* the FPGA stopped answering: autosave off until the next load */
static uint32_t sv_size;          /* persisted bytes; 0 = nothing to do */
static uint32_t sv_crc, sv_crc_old, sv_offset;
static uint8_t  sv_diff;          /* dirty since the last write */
static uint8_t  sv_same;          /* passes the dirty image stayed unchanged */
static uint8_t  sv_failed;
static uint8_t  sv_didnotsave;
static tick_t   sv_next;

uint32_t nesave_size(uint8_t has_battery, uint8_t is_nes20, uint8_t flags10) {
  uint32_t n = NESAVE_WINDOW_SIZE;
  if(!has_battery) return 0;
  if(is_nes20 && (flags10 >> 4)) {
    uint8_t shift = flags10 >> 4;
    n = (shift >= 8) ? NESAVE_WINDOW_SIZE : (64UL << shift);
    if(n > NESAVE_WINDOW_SIZE) n = NESAVE_WINDOW_SIZE;
  }
  return n;
}

static void sv_reset_state(void) {
  sv_crc = sv_crc_old = sv_offset = 0;
  sv_diff = sv_same = sv_failed = sv_didnotsave = 0;
  sv_hw_err = sv_bail = sv_dead = 0;
  sv_next = getticks();
}

/* ---- bounded PSRAM access (the same wire protocol as memory.c) ---- */

static void sv_setaddr(uint32_t a) {
  FPGA_SELECT();
  FPGA_WAIT_RDY_TO(sv_hw_err);              /* prior operations must clear out */
  if(!sv_hw_err) {
    FPGA_TX_BYTE(FPGA_CMD_SETADDR | FPGA_TGT_MEM);
    FPGA_TX_BYTE((a >> 16) & 0xff);
    FPGA_TX_BYTE((a >> 8) & 0xff);
    FPGA_TX_BYTE(a & 0xff);
  }
  FPGA_DESELECT();
}

/* CRC of [addr, addr+size) chained from `crc`.  `watch_reset` = bail the moment the
   console is seen in reset (calc_sram_crc's rule: the read is not stable then); the
   caller holding the SNES in reset passes 0. */
static uint32_t sv_scan(uint32_t addr, uint32_t size, uint32_t crc, uint8_t watch_reset) {
  uint32_t i;
  uint8_t data;
  sv_setaddr(addr);
  if(sv_hw_err) return crc;
  FPGA_SELECT();
  FPGA_TX_BYTE(FPGA_CMD_READMEM | FPGA_MEM_AUTOINC);
  for(i = 0; i < size; i++) {
    FPGA_WAIT_RDY_TO(sv_hw_err);
    if(sv_hw_err) break;
    data = FPGA_RX_BYTE();
    if(watch_reset && get_snes_reset()) { sv_bail = 1; break; }
    crc = crc32_update(crc, data);
  }
  FPGA_DESELECT();
  return crc;
}

/* bounded twin of sram_reliable(): the sentinel must read back SRAM_RELIABILITY_SCORE times */
static uint8_t sv_reliable(void) {
  uint16_t score = 0, i;
  uint32_t val;
  uint8_t  b;
  for(i = 0; i < SRAM_RELIABILITY_SCORE; i++) {
    sv_setaddr(SRAM_SCRATCHPAD);
    if(sv_hw_err) return 0;
    FPGA_SELECT();
    FPGA_TX_BYTE(0x88);                     /* READ */
    val = 0;
    for(b = 0; b < 4; b++) {
      FPGA_WAIT_RDY_TO(sv_hw_err);
      if(sv_hw_err) break;
      val |= (uint32_t)FPGA_RX_BYTE() << (8 * b);
    }
    FPGA_DESELECT();
    if(sv_hw_err) return 0;
    if(val == 0x12345678) score++;
  }
  rdyled(score == SRAM_RELIABILITY_SCORE);
  return score == SRAM_RELIABILITY_SCORE;
}

/* ---- .srm -> window, never writing past `max`.  A missing file is the normal first run. ---- */

static void sv_put(const uint8_t *src, uint32_t addr, uint16_t n) {
  sv_setaddr(addr);
  if(sv_hw_err) return;
  FPGA_SELECT();
  FPGA_TX_BYTE(0x98);                       /* WRITE */
  while(n--) {
    FPGA_TX_BYTE(*src++);
    FPGA_WAIT_RDY_TO(sv_hw_err);
    if(sv_hw_err) break;
  }
  FPGA_DESELECT();
}

static void sv_load(uint8_t *filename, uint32_t max) {
  char path[256];
  uint32_t done = 0;
  UINT got;
  if(srm_path_for_load(path, sizeof(path), filename) < 0) return;
  file_open((uint8_t*)path, FA_READ);
  if(file_res) {
    file_res = 0;                 /* FR_NO_FILE etc: window stays zeroed */
    return;
  }
  while(done < max) {
    got = file_read();
    if(file_res || !got) break;
    if(got > max - done) got = (UINT)(max - done);
    sv_put(file_buf, NESAVE_WINDOW_ADDR + done, (uint16_t)got);
    if(sv_hw_err) break;
    done += got;
  }
  file_close();
  file_res = 0;
}

void nesave_detach(void) {
  sv_size = 0;
  sv_reset_state();
}

void nesave_attach(uint8_t *filename, uint8_t has_battery, uint8_t is_nes20, uint8_t flags10) {
  sv_size = nesave_size(has_battery, is_nes20, flags10);
  sv_reset_state();
  if(!sv_size) return;
  sv_load(filename, sv_size);
  /* the baseline is what the window holds NOW, so an untouched save is never rewritten */
  if(!sv_hw_err) sv_crc_old = sv_scan(NESAVE_WINDOW_ADDR, sv_size, 0, 0);
  if(sv_hw_err) {
    /* the window state is unknown: arm nothing, so nothing can be written over a good save */
    printf("NES battery: FPGA not answering, save disarmed\n");
    sv_size = 0;
  }
}

/* window -> <stem>.srm.  Every FPGA wait is bounded; each 512-byte piece is re-addressed so the
   FPGA is deselected while the card is written (the Xeno Crisis branch of save_sram does the same). */
static void __attribute__((noinline)) sv_save_file(uint8_t *filename) {
  char path[256];
  uint32_t off = 0;
  uint16_t j, n;
  sv_hw_err = 0;
  /* probe BEFORE the file is truncated: a core that is not answering must not cost the old save */
  sv_setaddr(NESAVE_WINDOW_ADDR);
  if(sv_hw_err) return;
  if(srm_path_for_save(path, sizeof(path), filename) < 0) { file_res = FR_INVALID_NAME; return; }
  FPGA_DESELECT();
  file_open((uint8_t*)path, FA_CREATE_ALWAYS | FA_WRITE);
  if(file_res) return;
  while(off < sv_size) {
    n = (sv_size - off > 512) ? 512 : (uint16_t)(sv_size - off);
    sv_setaddr(NESAVE_WINDOW_ADDR + off);
    if(!sv_hw_err) {
      FPGA_SELECT();
      FPGA_TX_BYTE(0x88);                   /* READ */
      for(j = 0; j < n; j++) {
        FPGA_WAIT_RDY_TO(sv_hw_err);
        if(sv_hw_err) break;
        file_buf[j] = FPGA_RX_BYTE();
      }
      FPGA_DESELECT();
    }
    if(sv_hw_err) break;
    file_write(n);
    if(file_res) break;
    off += n;
  }
  file_close();
}

static void sv_write(uint8_t *filename) {
  writeled(1);
  file_res = FR_OK;
  sv_save_file(filename);
  sv_failed = (file_res || sv_hw_err) ? 1 : 0;
  if(sv_failed) {
    printf("NES battery save FAILED (%u)\n", (unsigned)file_res);
    sv_same = 0;                  /* retry after another stable stretch, not every pass */
    sv_didnotsave = 25;
  } else {
    printf("NES battery saved: %lu bytes\n", (unsigned long)sv_size);
    sv_diff = 0;
    sv_same = 0;
    sv_didnotsave = 0;
  }
  writeled(0);
}

void nesave_tick(uint8_t *filename) {
  uint32_t n;
  tick_t now;

  if(!sv_size || sv_dead) return;
  if(!CFG.enable_autosave) {
    /* keep sv_crc_old/sv_diff: the reset flush compares against them */
    sv_offset = 0; sv_crc = 0; sv_same = 0; sv_didnotsave = 0;
    return;
  }
  now = getticks();
  if((int32_t)(now - sv_next) < 0) return;
  sv_next = now + MS_TO_TICKS(NESAVE_PERIOD_MS);

  n = sv_size - sv_offset;
  if(n > NESAVE_CHUNK) n = NESAVE_CHUNK;
  sv_hw_err = 0;
  sv_bail = 0;
  sv_crc = sv_scan(NESAVE_WINDOW_ADDR + sv_offset, n, sv_crc, 1);
  sv_offset += n;

  if(!sv_hw_err && !sv_bail) {
    if(!sv_reliable() && !sv_hw_err) sv_bail = 1;   /* a pass that could not be read cleanly proves nothing */
  }
  if(sv_hw_err) {
    /* MCU_RDY stopped answering: give up on the autosave rather than stall the game loop
       for a whole timeout every pass.  The reset flush still tries (it checks the FPGA first). */
    printf("NES battery: FPGA not answering, autosave off\n");
    sv_dead = 1;
    sv_offset = 0;
    sv_crc = 0;
    return;
  }
  if(sv_bail) {
    sv_offset = 0;
    sv_crc = 0;
    return;
  }
  if(sv_offset < sv_size) return;

  if(sv_failed) sv_didnotsave++;
  if(sv_crc != sv_crc_old) {
    sv_diff = 1;
    sv_same = 0;
    sv_didnotsave++;
  } else if(sv_diff) {
    sv_same++;
  }
  if(sv_diff && sv_same >= NESAVE_STABLE_PASSES) sv_write(filename);
  /* contents that never settle (or a card that keeps failing): write anyway, rarely */
  if(sv_didnotsave > 50) sv_write(filename);
  sv_offset = 0;
  sv_crc_old = sv_crc;
  sv_crc = 0;
}

void nesave_flush_inreset(uint8_t *filename) {
  uint32_t crc;
  if(!sv_size) return;
  if(fpga_test() != FPGA_TEST_TOKEN) return;
  /* the SNES is already held in reset: the read is stable, so no reset watch */
  sv_hw_err = 0;
  crc = sv_scan(NESAVE_WINDOW_ADDR, sv_size, 0, 0);
  if(sv_hw_err) return;
  if(crc != sv_crc_old || sv_diff) {
    sv_write(filename);
    if(!sv_failed) sv_crc_old = crc;
  }
}

#endif /* !CONFIG_MK2 */
