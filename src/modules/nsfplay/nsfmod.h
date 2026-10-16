#ifndef NSFMOD_H
#define NSFMOD_H

#include <stddef.h>
#include <stdint.h>
#include "nsf.h"
#include "module.h"

/* --- The NSF player module (nsfplay.bin, module.h): a .nsf from the browser ---
 *
 * The 2A03 emulator (nsf.c, nsfapu.c) and its glue (nsfmod.c) are the card module: none of
 * it is in the firmware.  The core renders 44.1 kHz stereo into the menu music engine's PSRAM
 * ring (music_play_gen), played by the FPGA SFX fetcher; the tune's memory (NES RAM, $6000
 * RAM, ROM banks) sits above the ring, at SRAM_NSF_BK_ADDR.
 *
 * Status block: same contract as PCMPLAY_BLK (pcmplay.h).  The module is the sole writer and
 * republishes it once per menu-loop pass; the menu zeroes the magic before CMD_MOD_OPEN as a
 * capability sentinel.  Mirrored as NSF_* in snes/memmap.i65. */
#define NSFPLAY_MAGIC0          ('N')
#define NSFPLAY_MAGIC1          ('F')
#define NSFPLAY_VERSION         (1)

#define NSFPLAY_STATE_NONE      (0)
#define NSFPLAY_STATE_PLAYING   (1)
#define NSFPLAY_STATE_PAUSED    (2)
#define NSFPLAY_STATE_ERR_OPEN  (3)  /* the file would not open or read */
#define NSFPLAY_STATE_ERR_NSF   (4)  /* not an NSF this player can run */
#define NSFPLAY_STATE_ERR_CORE  (5)  /* nsfplay.bin missing, damaged or of another version */
#define NSFPLAY_STATE_ERR_TUNE  (6)  /* the tune crashed the CPU (or INIT/PLAY never returned) */

#define NSFPLAY_FLAG_PAL        (0x01)
#define NSFPLAY_FLAG_EXPANSION  (0x02)  /* the tune wants expansion audio, which is not played */

/* Control codes in MCU_PARAM's low byte (SNES_CMD_NSF_CTL) */
#define NSFPLAY_CTL_STOP        (0)
#define NSFPLAY_CTL_PAUSE       (1)
#define NSFPLAY_CTL_RESUME      (2)
#define NSFPLAY_CTL_SONG        (3)  /* MCU_PARAM+1 = song, 1-based */

#define NSFPLAY_STR_LEN         (32)

typedef struct __attribute__((__packed__)) _nsfplay_blk {
  uint8_t  magic[2];   /* +0  'N','F' */
  uint8_t  version;    /* +2  NSFPLAY_VERSION */
  uint8_t  state;      /* +3  NSFPLAY_STATE_* */
  uint8_t  song;       /* +4  playing song, 1-based */
  uint8_t  songs;      /* +5  songs in the file */
  uint16_t pos_sec;    /* +6  seconds into the song */
  uint8_t  flags;      /* +8  NSFPLAY_FLAG_* */
  uint8_t  load;       /* +9  CPU the emulator took over the last second, in %: what rendering in
                              real time costs this tune (0 in host builds) */
  uint8_t  pad[6];     /* +10 reserved */
  char     title[NSFPLAY_STR_LEN];      /* +16 NUL-terminated, ASCII (anything else is '?') */
  char     artist[NSFPLAY_STR_LEN];     /* +48 */
  char     copyright[NSFPLAY_STR_LEN];  /* +80 */
  uint8_t  pad2[16];   /* +112 reserved to 128 */
} nsfplay_blk_t;

_Static_assert(offsetof(nsfplay_blk_t, state)     ==   3, "NSF_STATE (+3) in snes/memmap.i65");
_Static_assert(offsetof(nsfplay_blk_t, song)      ==   4, "NSF_SONG (+4) in snes/memmap.i65");
_Static_assert(offsetof(nsfplay_blk_t, pos_sec)   ==   6, "NSF_POS (+6) in snes/memmap.i65");
_Static_assert(offsetof(nsfplay_blk_t, title)     ==  16, "NSF_TITLE (+16) in snes/memmap.i65");
_Static_assert(offsetof(nsfplay_blk_t, artist)    ==  48, "NSF_ARTIST (+48) in snes/memmap.i65");
_Static_assert(offsetof(nsfplay_blk_t, copyright) ==  80, "NSF_COPYRIGHT (+80) in snes/memmap.i65");
_Static_assert(sizeof(nsfplay_blk_t)              == 128, "NSF_SIZE in snes/memmap.i65");

/* The module's whole state, in the scratch FRAME the firmware lends it. */
typedef struct {
  const module_api_t *api;
  uint8_t  state, song, flags, load;
  uint16_t last_pos;
  uint32_t cyc, frames;     /* render cost meter: DWT cycles, and the frames they made */
  nsf_t    n;
} nsfmod_t;

int  nsfmod_start(void *st, const module_api_t *api, const char *path);
void nsfmod_ctl(void *st, uint32_t param);
void nsfmod_idle(void *st);
void nsfmod_stop(void *st);

#endif
