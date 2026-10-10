/* Xeno Crisis (Bitmap Bureau, 2019) on the sd2snes.

   The cartridge pairs a 128 KB SNES ROM with an RP2040 that runs the game and streams 65816
   code to the SNES through a window at $3000.  The xc core (verilog/sd2snes_xc, one tree
   for the Mk.II and the Mk.III) replaces the RP2040 with a soft Cortex-M0 that runs the
   cartridge's own firmware out of the PSRAM.  Core design: sttng (github.com/sttng/sd2snes-xenocrisis).

   Files on the card, next to the firmware:
     /sd2snes/fpga_xc.bit (Mk.II) or fpga_xc.bi3   the core
     /sd2snes/xc_soc.bin               replacement RP2040 bootrom + firmware additions, built
                                       with this firmware (src/xc_soc); must match it
     /sd2snes/xenocrisis_rp2040.bin    the cartridge's RP2040 flash dump (16 MB, v1.00),
                                       supplied by the user like a chip BIOS

   Music: from an MSU-1 pack next to the ROM (<rom>.msu + <rom>-1..26.pcm), on every board.
   Without a pack the game runs with its sound effects only: this firmware has no Opus
   decoder (no room on any of the three MCUs), so on the Mk.III the music packets the
   mixer posts are answered as "not decoded".  A pack can be built from the dump itself. */
#ifndef XC_H
#define XC_H
#include <stdint.h>

/* FPGA commands of the xc cores (mcu_cmd.v) */
#define FPGA_CMD_XCA_STATUS    (0xc0)  /* read: status, packet length (2 bytes LE); mk3 only */
#define   XCA_ST_JOB           0x01    /* a music packet is waiting */
#define   XCA_ST_RESET         0x02    /* the mixer asked for a decoder reset */
#define FPGA_CMD_XCA_DONE      (0xc3)  /* write: ret (4 bytes LE), final range (4 bytes LE) */
#define FPGA_CMD_XCA_ACKRESET  (0xc4)  /* clear the reset request */
#define FPGA_CMD_XC_STATUS     (0xc5)  /* read: (null), {running, halted}, halt code, halt address */
#define FPGA_CMD_XC_RUN        (0xc6)  /* write: bit 0 release the soft CPU, bit 1 MSU-1 music */

/* load_check_prereqs, before the ACK: NULL when both files are on the card and usable,
   else the basename to show in the "file missing" popup */
const char *xc_check_files(void);
/* load_stage_bios: the RP2040 flash dump and xc_soc.bin into the PSRAM, firmware patches
   applied.  0 on a read error (the popup names the dump). */
uint8_t xc_load_image(void);
/* load_saveram, no .srm yet: start from the dump's save area (the cartridge's own saves) */
void xc_load_dump_save(void);
/* init(): release the soft CPU once the image and the save are in place */
void xc_run(uint8_t msu_music);
/* game loops: answer the mixer's music packets (Mk.III without a pack), cheap when idle */
void xc_poll(void);
/* from inside long MCU jobs (SaveRAM CRC, save), FPGA deselected: keep the music going */
void xc_service(void);
/* the game is left: report a soft CPU halt in /sd2snes/xc_debug.txt (built with XC_DEBUG=1) */
#ifdef XC_DEBUG
void xc_report(void);
#else
static inline void xc_report(void) {}
#endif

#endif
