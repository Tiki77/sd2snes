#ifndef NSF_H
#define NSF_H

/* NSF (NES Sound Format) player: a 2A03 (6502 CPU + APU) emulated on the MCU,
 * rendering 44.1 kHz s16 stereo for the menu's sfxdma path (like the QOA clip).
 *
 * Memory model.  The tune never lives in MCU RAM: everything the 6502 can
 * address is kept in a BACKING STORE (PSRAM on the device, an array on the
 * host) behind a small cache.  The cache maps each 128-byte NES page to a
 * virtual page (NSF_VPAGES of them) whose 16-byte sectors come from a shared
 * pool, read one sector at a time: a backing read costs ~1.5 us a byte on the
 * device, and a tune's loop touches a few bytes of many pages.  Backing
 * layout:
 *
 *   NSF_BK_RAM   0x0000  2 KB   NES RAM $0000-$07FF
 *   NSF_BK_SRAM  0x0800  8 KB   $6000-$7FFF
 *   NSF_BK_ROM   0x2800  ...    the tune's 4 KB banks, bank b at NSF_BK_ROM + b*4096
 *
 * The caller fills the ROM banks once (nsf_layout() says where the file data
 * goes and how much zero padding is in front of it and after it) and the core
 * owns the RAM part.  A non-banked tune is played as a banked one with banks
 * 0..7, so there is a single mapping path.
 *
 * Free of firmware dependencies (no config.h, no ff.h) so the host test
 * compiles it straight from src/.  Keep it that way. */

#include <stdint.h>

#define NSF_HDR_SIZE   0x80
#define NSF_BK_RAM     0x0000
#define NSF_BK_SRAM    0x0800
#define NSF_BK_ROM     0x2800
#define NSF_RATE       44100

#define NSF_LINE_SHIFT 7        /* a NES page: 128 bytes */
#define NSF_LINE       (1 << NSF_LINE_SHIFT)
#define NSF_SECT_SHIFT 4        /* a pool sector: 16 bytes */
#define NSF_SECT       (1 << NSF_SECT_SHIFT)
#define NSF_SPP        (NSF_LINE / NSF_SECT)
#define NSF_VPAGES     31       /* virtual pages: an owner byte (page * 8 + sector) stays below 0xff */
#define NSF_PINNED     4        /* virtual pages 0-3 hold $0000-$01FF (zero page + stack) for good */
#define NSF_MINSECT    (NSF_PINNED * NSF_SPP + 16)

/* Reads (write == 0) or writes len bytes of the backing store at off. */
typedef void (*nsf_bk_fn)(void *ctx, uint32_t off, uint8_t *buf, uint16_t len, uint8_t write);

/* What the header says, validated. */
typedef struct {
  uint16_t load, init, play;
  uint16_t speed_us;            /* PLAY period, from the NTSC or PAL field */
  uint8_t  songs, start;        /* start is 1-based */
  uint8_t  banks[8];            /* initial $5FF8-$5FFF (synthesized 0..7 when not banked) */
  uint8_t  pal;                 /* 1 = play with PAL timing */
  uint8_t  chip;                /* expansion audio bits (not emulated) */
  uint16_t nbanks;              /* 4 KB banks in the ROM area */
  uint32_t pad;                 /* zero bytes in front of the file data in the ROM area */
  uint32_t data_len;            /* file bytes (after the header) that go into the ROM area */
} nsf_info_t;

/* --- APU ----------------------------------------------------------------- */

typedef struct {
  uint8_t  reg0, sweep;         /* $4000 DDLC VVVV, $4001 EPPP NSSS */
  uint16_t period;              /* 11-bit timer */
  uint8_t  len, seq;
  uint8_t  env_start, env_div, env_vol;
  uint8_t  sw_div, sw_reload;
  uint8_t  amp;                 /* volume when the duty bit is 1, 0 when muted */
  uint8_t  level;
  uint16_t count;               /* CPU cycles to the next sequencer step */
  uint32_t acc;                 /* level x cycles in the current sample */
} nsf_pulse_t;

typedef struct {
  uint8_t  reg;                 /* $4008 CRRR RRRR */
  uint16_t period;
  uint8_t  len, lin, lin_reload, seq, level;
  uint16_t count;
  uint32_t acc;
} nsf_tri_t;

typedef struct {
  uint8_t  reg0, mode, rate;    /* $400C --LC VVVV, $400E M---PPPP */
  uint8_t  len, env_start, env_div, env_vol, amp, level;
  uint16_t lfsr, count;
  uint32_t acc;
} nsf_noise_t;

typedef struct {
  uint8_t  reg0, addr_reg, len_reg;
  uint8_t  level, buffer, full, shift, bits, silence, irq;
  uint16_t addr, left, count;
  uint32_t acc;
} nsf_dmc_t;

typedef struct {
  nsf_pulse_t p[2];
  nsf_tri_t   t;
  nsf_noise_t n;
  nsf_dmc_t   d;
  uint8_t  enabled;             /* $4015 bits 0-3 */
  uint8_t  fc_mode, fc_inhibit, fc_irq, fc_step;
  uint32_t fc_irq_end;          /* the IRQ flag is set again on every cycle up to this one */
  uint8_t  pal;
  uint16_t fc_count;            /* CPU cycles to the next frame-sequencer event */
  uint32_t time;                /* CPU cycle the APU has been run to */
  void    *ctx;                 /* the nsf_t, for DMC sample fetches */
} nsf_apu_t;

/* --- player -------------------------------------------------------------- */

#define NSF_OK        0
#define NSF_ERR_JAM   1         /* KIL opcode */
#define NSF_ERR_HANG  2         /* INIT or PLAY never returned */

typedef struct nsf_s nsf_t;
struct nsf_s {
  /* 6502 */
  uint8_t  a, x, y, s, p;
  uint16_t pc;
  uint32_t cyc;                 /* CPU cycles, wraps */
  /* player */
  nsf_info_t info;
  uint8_t  bank[8];
  uint8_t  busy;                /* 0 idle, 1 in INIT, 2 in PLAY */
  uint8_t  err;
  uint8_t  rendering;           /* inside nsf_render: APU syncs stop at lim */
  uint32_t lim;                 /* end of the sample being rendered */
  uint8_t  ofs;                 /* cycle of the current instruction's bus access */
  uint32_t call_start;
  uint32_t apu_at;              /* cycle of the last APU register write */
  uint32_t next_play;           /* cycle of the next PLAY call (integer part) */
  uint32_t play_frac, period_int, period_frac;   /* PLAY period, 24.8 fixed */
  uint32_t srem, cpu_hz;        /* sample clock remainder */
  int32_t  hp_x, hp_y;          /* DC blocker */
  /* backing store + cache */
  nsf_bk_fn bk;
  void    *bk_ctx;
  uint8_t  (*io_read)(nsf_t *n, uint16_t a);   /* host tests only: $2000-$5FFF reads; NULL = open bus 0 */
  uint8_t  *vslot;              /* [NSF_VPAGES][NSF_SPP] pool sector of each sector, 0xFF = none */
  uint8_t  *sown;               /* [nsect] owner of each pool sector (vpage * 8 + sector), 0xFF = free */
  uint8_t  (*pool)[NSF_SECT];   /* [nsect] sectors; 0-31 are $0000-$01FF */
  uint8_t  nsect, vvictim, pvictim;
  /* nsf_start fills map..vtag and vused..vdirty as two runs: keep them in order */
  uint8_t  map[512];            /* NES 128-byte page -> virtual page, 0xFF = not mapped */
  uint16_t vtag[NSF_VPAGES];    /* backing page number (offset >> 7), 0xFFFF = free */
  uint8_t  vused[NSF_VPAGES];   /* touched since the last PLAY call started */
  uint8_t  vdirty[NSF_VPAGES];  /* bit s: sector s written since it was read */
  uint32_t misses, writebacks;  /* statistics */
  nsf_apu_t apu;
};

/* Validates the 128-byte header.  Returns 0 if it is not a tune this player
 * can run (bad magic, no songs, load below $8000, file too big for the bank
 * registers).  file_size is the whole file, header included. */
uint8_t nsf_parse(nsf_info_t *info, const uint8_t hdr[NSF_HDR_SIZE], uint32_t file_size);

/* Total size of the ROM area (padding + file data + zero tail up to a bank
 * boundary): the caller writes info->pad zeros at NSF_BK_ROM, the file data
 * (file_size - NSF_HDR_SIZE bytes from offset NSF_HDR_SIZE) right after, and
 * zeros up to NSF_BK_ROM + nsf_rom_size().  Only info->data_len bytes of
 * the file are used (a flat tune that runs past $FFFF is cut there). */
static inline uint32_t nsf_rom_size(const nsf_info_t *info) { return (uint32_t)info->nbanks << 12; }

/* Binds the backing store (its ROM area must already be filled) and the
 * cache, which takes size bytes at mem: NSF_VPAGES * 8 of sector table, then
 * 17 bytes per pool sector (up to 255; more sectors, fewer backing reads).
 * Returns 0 if that is fewer than NSF_MINSECT sectors. */
uint8_t nsf_open(nsf_t *n, const nsf_info_t *info, nsf_bk_fn bk, void *bk_ctx,
                 uint8_t *mem, uint32_t size);

/* Clears RAM, resets the APU and starts INIT for song (1-based).  Rendering
 * runs INIT and then PLAY at the header's rate. */
void nsf_start(nsf_t *n, uint8_t song);

/* Renders frames stereo s16 frames (L = R) into out.  Always fills the whole
 * buffer; after an error (n->err) the rest is silence. */
void nsf_render(nsf_t *n, int16_t *out, unsigned frames);

/* For the host tests: memory access through the cache, one instruction, and
 * an APU register write/read at the current cycle. */
uint8_t  nsf_peek(nsf_t *n, uint16_t a);
void     nsf_poke(nsf_t *n, uint16_t a, uint8_t v);
unsigned nsf_step(nsf_t *n);
void     nsf_apu_reset(nsf_apu_t *apu, uint8_t pal);
void     nsf_apu_run(nsf_apu_t *apu, uint32_t until);
void     nsf_apu_write(nsf_apu_t *apu, uint16_t a, uint8_t v);
uint8_t  nsf_apu_status(nsf_apu_t *apu);
uint8_t  nsf_dmc_fetch(void *ctx, uint16_t a);

#endif
