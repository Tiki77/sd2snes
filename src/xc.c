/* Xeno Crisis on the sd2snes: building the soft CPU's image and serving its core.  See xc.h.

   PSRAM layout of a Xeno Crisis load (the same as sttng's src/xc_soc/xc_build_image.py):
     0x000000-0x01FFFF  SNES kernel ROM (load_stream)
     0x020000-0xCFFFFF  RP2040 flash 0x020000-0xCFFFFF      (flash offset = PSRAM address)
     0xD00000-0xD1FFFF  RP2040 flash 0x000000-0x01FFFF
     0xD20000-0xD23FFF  replacement bootrom                 (soft CPU 0x00000000)
     0xD24000-0xD27FFF  firmware additions                  (RP2040 flash 0xF00000)
   It covers the PSRAM the menu, the in-game menu, the cover/game-info staging and the cheat
   records use, so init() must not stage any of those for this game (none of them work on
   the xc cores anyway: no hooks, no cheat engine, no savestates).

   After the copy, the firmware functions listed in xc_soc.bin's patch table are redirected
   in the PSRAM copy; the Mk.II applies a second table ("MK2P") as well: its core has no SIO
   divider and needs the faster code paths.  Without a .srm the save area starts from the
   dump's (RP2040 flash 0xFF8000-0xFFFFFF), so saves made on the cartridge carry over. */
#include <string.h>
#include "config.h"
#include "uart.h"
#include "ff.h"
#include "diskio.h"
#include "fileops.h"
#include "spi.h"
#include "fpga_spi.h"
#include "memory.h"
#include "smc.h"
#include "msu1.h"
#include "xc.h"

#define XC_FLASH_FILE   "/sd2snes/xenocrisis_rp2040.bin"
#define XC_SOC_FILE     "/sd2snes/xc_soc.bin"
#define XC_FLASH_SIZE   0x1000000u
#define XC_SOC_MAGIC    0x434F5358u    /* "XSOC" */
#define XC_FW_MAGIC     0x5843584Du    /* firmware additions header (xc_fw_header.c) */
#define XC_MK2P_MAGIC   0x50324B4Du    /* "MK2P": mk2 patch table after the main one */
#define XC_LAUNCH_CORE1 0x10059060u    /* multicore_launch_core1(): identifies the v1.00 build */

extern snes_romprops_t romprops;

static uint32_t psram_of(uint32_t flash_addr) {
  uint32_t a = flash_addr - 0x10000000u;
  return a < 0x20000u ? 0xD00000u + a : a;
}

/* --- the two files ---------------------------------------------------------------------- */

/* the patch tables of xc_soc.bin (open): offset of the main entries and their count, plus
   (Mk.II) the MK2P table's.  0 when a header is wrong, i.e. xc_soc.bin from another build. */
static uint8_t soc_tables(uint32_t *n_main, uint32_t *t_mk2, uint32_t *n_mk2) {
  uint32_t h[4];
  if(file_readblock(h, 0, 8) != 8 || h[0] != XC_SOC_MAGIC || h[1] != 1) return 0;
  if(file_readblock(h, 0x4200u, 16) != 16 || h[0] != XC_FW_MAGIC || h[1] < 2 || h[3] > 256) return 0;
  *n_main = h[3];
  *t_mk2 = 0x4210u + h[3] * 12;
  *n_mk2 = 0;
#ifdef CONFIG_MK2
  if(file_readblock(h, *t_mk2, 8) != 8 || h[0] != XC_MK2P_MAGIC || h[1] > 64) return 0;
  *n_mk2 = h[1];
#endif
  return 1;
}

const char *xc_check_files(void) {
  uint16_t sig = 0;
  uint32_t n_main, t_mk2, n_mk2;
  uint8_t ok;
  file_open((const uint8_t*)XC_FLASH_FILE, FA_READ);
  ok = !file_res && file_handle.fsize == XC_FLASH_SIZE
       && file_readblock(&sig, psram_of(XC_LAUNCH_CORE1), 2) == 2 && sig == 0x4905;
  file_close();
  if(!ok) {
    file_res = FR_OK;
    return "xenocrisis_rp2040.bin";
  }
  file_open((const uint8_t*)XC_SOC_FILE, FA_READ);
  ok = !file_res && soc_tables(&n_main, &t_mk2, &n_mk2);
  file_close();
  file_res = FR_OK;
  return ok ? NULL : "xc_soc.bin";
}

/* len bytes (a multiple of 512) from file offset off straight into the PSRAM at dst, by the
   SD DMA like load_stream */
static uint8_t offload(const char *fn, uint32_t off, uint32_t len, uint32_t dst) {
  set_mcu_addr(dst);
  file_open((const uint8_t*)fn, FA_READ);
  if(file_res) return 0;
  ff_sd_offload = 1;
  f_lseek(&file_handle, off);
  while(len && !file_res) {
    ff_sd_offload = 1;
    sd_offload_tgt = 0;
    UINT n = file_read();
    if(!n) break;
    len -= n < len ? n : len;
  }
  file_close();
  return !file_res && !len;
}

/* one patch table entry at offset off of xc_soc.bin (open): redirect a firmware function */
static void patch_entry(uint32_t off) {
  uint32_t e[3];                                   /* flash address, target, kind */
  if(file_readblock(e, off, 12) != 12) return;
  uint32_t addr = e[0];
  if(e[2] == 0) {
    /* push {r0}; ldr r0, [pc, #k]; mov ip, r0; pop {r0}; bx ip; nop; .word target */
    uint32_t lit = (addr + 13) & ~3u;
    uint16_t code[6] = { 0xB401, 0x4800 | (uint16_t)((lit - ((addr + 6) & ~3u)) / 4), 0x4684,
                         0xBC01, 0x4760, 0xBF00 };
    sram_writeblock(code, psram_of(addr), 12);
    sram_writeblock(&e[1], psram_of(lit), 4);
  } else if(e[2] == 1) {
    sram_writeblock(&e[1], psram_of(addr + 12), 4);  /* existing veneer: new target */
  } else if(e[2] == 3) {
    sram_writeblock(&e[1], psram_of(addr), 4);       /* code patch (mk2 table): the word */
  } else {
    uint16_t code[2] = { 0x2000, 0x4770 };           /* movs r0, #0; bx lr */
    sram_writeblock(code, psram_of(addr), 4);
  }
}

uint8_t xc_load_image(void) {
  uint32_t n_main = 0, t_mk2 = 0, n_mk2 = 0;
  if(!offload(XC_FLASH_FILE, 0x020000u, 0xCE0000u, 0x020000u)
     || !offload(XC_FLASH_FILE, 0x000000u, 0x020000u, 0xD00000u)
     || !offload(XC_SOC_FILE, 0x200u, 0x8000u, 0xD20000u)) {
    printf("XC: read error %d\n", file_res);
    return 0;
  }
  file_open((const uint8_t*)XC_SOC_FILE, FA_READ);
  if(!file_res && soc_tables(&n_main, &t_mk2, &n_mk2)) {
    for(uint32_t i = 0; i < n_main; i++) patch_entry(0x4210u + i * 12);
    for(uint32_t i = 0; i < n_mk2; i++) patch_entry(t_mk2 + 8 + i * 12);
  }
  file_close();
  printf("XC: image ok, %lu+%lu patches\n", n_main, n_mk2);
  return !file_res;
}

void xc_load_dump_save(void) {
  file_open((const uint8_t*)XC_FLASH_FILE, FA_READ);
  for(uint32_t off = 0; off < 0x8000u && !file_res; off += 512) {
    if(file_readblock(file_buf, 0xFF8000u + off, 512) != 512) break;
    sram_writeblock(file_buf, SRAM_SAVE_ADDR + off, 512);
  }
  file_close();
  printf("XC: save from the dump\n");
}

/* --- the running core ------------------------------------------------------------------- */

/* one byte to the FPGA with the gap spi.v needs after it (the block functions are unsafe
   with these commands, see sttng's xc_audio.c) */
static void xc_tx(uint8_t b) {
  FPGA_TX_BYTE(b);
  FPGA_TX_SYNC();
}

void xc_run(uint8_t msu_music) {
  FPGA_SELECT();
  xc_tx(FPGA_CMD_XC_RUN);
  xc_tx(msu_music ? 3 : 1);
  xc_tx(0x00); /* flop */
  FPGA_DESELECT();
}

void xc_poll(void) {
#ifndef CONFIG_MK2
  /* Mk.III without an MSU-1 pack: the mixer posts each 20 ms music packet to the decode
     mailbox.  "Nothing decoded" (ret 0, final range 0) makes it end the track the way the
     RP2040 firmware treats corrupted music data, and the sound effects keep playing.  The
     Mk.II core is MSU-1 only and has no mailbox. */
  uint8_t st;
  FPGA_SELECT();
  xc_tx(FPGA_CMD_XCA_STATUS);
  FPGA_RX_BYTE(); /* null read */
  st = FPGA_RX_BYTE();
  FPGA_DESELECT();
  if(st & XCA_ST_RESET) {
    FPGA_SELECT();
    xc_tx(FPGA_CMD_XCA_ACKRESET);
    FPGA_DESELECT();
  } else if(st & XCA_ST_JOB) {
    FPGA_SELECT();
    xc_tx(FPGA_CMD_XCA_DONE);
    for(int i = 0; i < 9; i++) xc_tx(0x00);  /* ret, final range, flop */
    FPGA_DESELECT();
  }
#endif
}

void xc_service(void) {
  if(romprops.has_msu1) msu1_audio_service();
  else xc_poll();
}

#ifdef XC_DEBUG
void xc_report(void) {
  uint8_t st;
  uint32_t w[2] = { 0, 0 };
  FPGA_SELECT();
  xc_tx(FPGA_CMD_XC_STATUS);
  FPGA_RX_BYTE(); /* null read */
  st = FPGA_RX_BYTE();
  for(int i = 0; i < 8; i++) w[i >> 2] |= (uint32_t)FPGA_RX_BYTE() << (8 * (i & 3));
  FPGA_DESELECT();
  if(!(st & 1)) return;
  /* halted: a core fault or a firmware panic.  The code and address are what a bug report
     needs; nothing else in the field can see them on the Mk.II (no UART). */
  int n = snprintf((char*)file_buf, sizeof(file_buf), "soft CPU halted: code %08lx at %08lx\r\n",
                   (unsigned long)w[0], (unsigned long)w[1]);
  printf("XC: %s", (char*)file_buf);
  file_open((const uint8_t*)"/sd2snes/xc_debug.txt", FA_WRITE | FA_CREATE_ALWAYS);
  if(!file_res && n > 0) file_write(n);
  file_close();
  file_res = FR_OK;
}
#endif
