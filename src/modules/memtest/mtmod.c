/* sd2snes+ -- RAM connection test, run from the menu (the card module memtest.bin).
 *
 * Port of test_memconn() from the standalone diagnostic firmware (src/tests/tests.c).
 * Same walks, same conclusions, but the findings are published as the binary block in
 * memtest.h instead of printed over a serial console the Mk.II does not have.
 *
 * WHAT THE WIRING WALK PROVES: it writes one word at a handful of addresses (the base,
 * plus one address per address line) and checks that every data line toggles
 * independently, that every address line reaches a distinct cell, and that no two address
 * lines are shorted together.  The failures users hit most (a cracked solder joint, a
 * bridged pin, a dead chip) all show up here, in ~10 seconds.
 *
 * WHAT IT DOES NOT PROVE, and why the cell sweep below exists: a single weak cell in the
 * middle of the array passes every walk above.  That is not a hypothetical gap -- the
 * official diagnostic has the same one.  Its test_mem() sweeps all 16 MB but has NO
 * CALLER, and the SNES-side memtest: in snes/tests/tests.a65 is commented out, so the only
 * cells it ever verifies are the first 1 MiB of 16, incidentally, inside test_sddma().
 * The cell sweep closes that: one write pass and one verify pass over both arrays, ~50
 * seconds on top of the walk, run after it on every test.
 *
 * WHY IT NEEDS THE fpga_test CORE: the runtime cores leave RAM1 (U511, the 4 Mbit SRAM)
 * completely undriven -- RAM_DATA/RAM_ADDR have no assignment in sd2snes_base/main.v --
 * so half the test is unreachable without reconfiguring.  fpga_test is also the core the
 * OFFICIAL diagnostic firmware uses, which is what makes a result from here comparable
 * with one from the official image: same core, same walks, same verdict.
 *
 * AS A MODULE: everything board-specific comes through the api table (the PSRAM chips and
 * their stride, fpga_test, the two 16 MB SPI bursts, the LED), so the image is the same on
 * every board.  No globals; the api pointer travels as a parameter.
 */

#include "module.h"
#include "memtest.h"
#include "memmap.h"

/* Scratch word used to load the FPGA's internal data register WITHOUT writing the test
   location -- that is what makes the byte-select check below meaningful.  Any address
   outside the walked set works; this is the one the diagnostic firmware uses. */
#define MT_DUMMY_ADDR   (0x56789aL)

/* Highest address-line offset walked on RAM0.  16 MB PSRAM -> A23. */
#define MT_RAM0_TOP     (0x800000L)
/* ...and on RAM1: up to A17, which is the diagnostic firmware's own bound and the one
   that matches what the MCU can actually reach on this array (A0..A17 => 0x40000 bytes).
   Do NOT "fix" this to A18 by reasoning from the "4 Mbit" on the schematic: the sweep
   below derives its extent from this constant, and everything past 0x40000 measures as
   garbage on known-good hardware. */
#define MT_RAM1_TOP     (0x20000L)

/* Full extent of each array, for the cell sweep.  RAM0 is 16 MB on both boards (Mk.II
   reaches it with one 128 Mbit chip, Mk.III with two 64 Mbit ones interleaved).
   MT_RAM1_SIZE IS DERIVED FROM MT_RAM1_TOP, NOT FROM "4 Mbit": sweeping the 0x80000 that
   "4 Mbit = 512 KB" suggests reads past the end (measured on a healthy Mk.III: every byte
   below 0x40000 verifies clean, everything above comes back unrelated to the pattern, with
   a different error count on every run -- the signature of reading nothing). */
#define MT_RAM0_SIZE    (0x1000000UL)
#define MT_RAM1_SIZE    (MT_RAM1_TOP * 2)

/* Distinct faults on ONE chip past which its own CE#/OE#/WE# becomes the better
   explanation than that many independent broken lines, and the same across all chips for
   the FPGA side.  Both thresholds are the diagnostic firmware's. */
#define MT_CTRL_THRESHOLD      (20)
#define MT_FPGACTRL_THRESHOLD  (40)

/* The sweep writes its pattern over ALL of RAM0, and the top page of that is the block the
   menu and the firmware talk through: SRAM_MENU_CFG_ADDR, every in-game gate, SAVEINFO,
   MANUAL_META, EXPORT_RESULT, and this test's own result block.  The reload only rebuilds
   the MENU IMAGE, and nobody clears this page outside first boot, so a pattern byte in
   SRAM_EXPORT_RESULT_ADDR would come back as a "ROM export failed" modal.  Zero the page
   and let each subsystem's boot path refill what it owns.  Done BEFORE the result is
   published, since MEMTEST_BLK lives in this page too. */
#define MT_STATE_PAGE_ADDR   (0xFF0000L)
#define MT_STATE_PAGE_SIZE   (0x800L)

typedef struct {
  memtest_finding_t find[MEMTEST_MAX_FINDINGS];
  uint8_t  nfind;
  uint8_t  truncated;
  /* Distinct faults already recorded, per chip.  These double as the DEDUP for the
     repeated passes (the walks run mt_iterations times to catch intermittents, so the same
     broken line is seen hundreds of times) AND as the fault COUNT the CE#/OE#/WE#
     heuristics threshold on.  Bitmasks, not the diagnostic firmware's byte arrays: those
     are 1.5 KB of stack, and this MCU has ~3 KB of stack+heap headroom in total. */
  uint32_t addr_seen[MEMTEST_NUM_CHIPS];   /* bit N = address line A<N> already reported */
  uint16_t data_seen[MEMTEST_NUM_CHIPS];   /* bit N = data line D<N> already reported */
  uint8_t  bytesel_seen[MEMTEST_NUM_CHIPS];
  uint8_t  ctrl_seen[MEMTEST_NUM_CHIPS];
  uint8_t  fpgactrl_seen;
} mt_ctx_t;

typedef struct {
  const module_api_t *api;
} mtmod_t;

static void mt_zero(void *p, unsigned n) {
  uint8_t *b = p;
  while(n--) *b++ = 0;
}

/* Append one finding, unless the exact same (chip, kind, line1, line2) is already in the
   list.  Shorts dedup THROUGH THIS LIST rather than through a bitmask of their own: a
   short pair needs 24x24 bits per chip, and the list is capped at 12 entries anyway. */
static void mt_add(mt_ctx_t *ctx, uint8_t chip, uint8_t kind, uint8_t l1, uint8_t l2) {
  uint8_t i;
  for(i = 0; i < ctx->nfind; i++) {
    if(ctx->find[i].chip == chip && ctx->find[i].kind == kind
       && ctx->find[i].line1 == l1 && ctx->find[i].line2 == l2) return;
  }
  if(ctx->nfind >= MEMTEST_MAX_FINDINGS) { ctx->truncated = 1; return; }
  ctx->find[ctx->nfind].chip  = chip;
  ctx->find[ctx->nfind].kind  = kind;
  ctx->find[ctx->nfind].line1 = l1;
  ctx->find[ctx->nfind].line2 = l2;
  ctx->nfind++;
}

static void mt_data_fault(mt_ctx_t *ctx, uint8_t chip, uint8_t line) {
  if(ctx->data_seen[chip] & (1u << line)) return;
  ctx->data_seen[chip] |= (1u << line);
  mt_add(ctx, chip, MEMTEST_KIND_DATA, line, 0);
}

static void mt_addr_fault(mt_ctx_t *ctx, uint8_t chip, uint8_t line) {
  if(ctx->addr_seen[chip] & (1ul << line)) return;
  ctx->addr_seen[chip] |= (1ul << line);
  mt_add(ctx, chip, MEMTEST_KIND_ADDR, line, 0);
}

static void mt_bytesel_fault(mt_ctx_t *ctx, uint8_t chip) {
  if(ctx->bytesel_seen[chip]) return;
  ctx->bytesel_seen[chip] = 1;
  mt_add(ctx, chip, MEMTEST_KIND_BYTESEL, 0, 0);
}

/* popcount without a table: the counts are tiny and this runs a handful of times. */
static uint8_t mt_bits(uint32_t v) {
  uint8_t n = 0;
  while(v) { n += (v & 1); v >>= 1; }
  return n;
}

static uint8_t mt_fault_count(const mt_ctx_t *ctx, uint8_t chip) {
  return mt_bits(ctx->addr_seen[chip]) + mt_bits(ctx->data_seen[chip])
       + ctx->bytesel_seen[chip];
}

/* "Too many independent lines broken" is almost never that -- it is one control signal
   (CE#/OE#/WE#) taking the whole chip out.  Say so, so the user checks the right pin. */
static void mt_check_ctrl(mt_ctx_t *ctx, uint8_t chip) {
  if(ctx->ctrl_seen[chip]) return;
  if(mt_fault_count(ctx, chip) <= MT_CTRL_THRESHOLD) return;
  ctx->ctrl_seen[chip] = 1;
  mt_add(ctx, chip, MEMTEST_KIND_CTRL, 0, 0);
}

static void mt_check_fpgactrl(mt_ctx_t *ctx) {
  uint8_t chip, sum = 0;
  if(ctx->fpgactrl_seen) return;
  for(chip = 0; chip < MEMTEST_NUM_CHIPS; chip++) sum += mt_fault_count(ctx, chip);
  if(sum <= MT_FPGACTRL_THRESHOLD) return;
  ctx->fpgactrl_seen = 1;
  mt_add(ctx, MEMTEST_CHIP_RAM0_1, MEMTEST_KIND_FPGACTRL, 0, 0);
}

/* ---- RAM0: the PSRAM (U501, plus U502 on Mk.III).  16 bits wide, so the byte enables
   (LB#/UB#) are part of the wiring and get their own check. ------------------------- */
static void mt_ram0(const module_api_t *api, mt_ctx_t *ctx) {
  uint8_t chip;
  uint32_t second = api->ram0_second;

  api->mt_select(0);
  for(chip = 0; chip < api->ram0_chips; chip++) {
    uint32_t addr = api->ram0_base[chip];
    int iter;

    for(iter = 0; iter < api->mt_iterations; iter++) {
      uint32_t offset, shortoffset, data16;
      uint8_t line, shortline, shortcount;
      int localerror, byte, i;

      /* Zero every cell the walks below read back -- INCLUDING the other chip's, because
         a short between the two chips' address lines would otherwise read as this chip's
         own leftover data. */
      for(i = 0; i < api->ram0_chips; i++) {
        api->sram_writeshort(0x0000, api->ram0_base[i]);
        for(offset = second; offset <= MT_RAM0_TOP; offset <<= 1)
          api->sram_writeshort(0x0000, offset + api->ram0_base[i]);
      }

      /* --- byte selects: write ONE byte of a word and prove the other byte survived.
         The dummy write in between is what makes this a real test: it loads the FPGA's
         data register with a different value, so a byte enable stuck active writes 0x55
         or 0xaa over the neighbour instead of the 0x22 we asked for. */
      localerror = 0;
      for(byte = 0; byte < 2; byte++) {
        api->sram_writeshort(0x7777, addr);
        api->sram_writeshort(0x55aa, MT_DUMMY_ADDR);
        api->sram_writebyte(0x22, addr ^ byte);
        if(api->sram_readbyte((addr + 1) ^ byte) != 0x77) localerror = 1;
      }
      if(localerror) mt_bytesel_fault(ctx, chip);

      /* --- data lines: each bit must be settable AND clearable on its own. */
      api->sram_writeshort(0x0000, addr);
      line = 0;
      for(data16 = 1; data16 <= 0x8000; data16 <<= 1) {
        localerror = 0;
        api->sram_writeshort((uint16_t)data16, addr);
        if(!(api->sram_readshort(addr) & data16)) localerror = 1;
        api->sram_writeshort((uint16_t)~data16, addr);
        if(api->sram_readshort(addr) & data16) localerror = 1;
        /* ^8 maps the walk index back to the chip's own D numbering: the halves of the
           16-bit word are swapped on the way to the pins (the diagnostic firmware prints
           the same corrected number, so the two reports name the same pin). */
        if(localerror) mt_data_fault(ctx, chip, line ^ 8);
        line++;
      }

      /* --- address lines: set exactly one line and prove (a) the cell it selects is
         reachable, (b) the base cell did NOT change (line stuck high / decoded twice),
         and (c) no OTHER single-line address also changed (two lines shorted). */
      line = 0;
      for(offset = second; offset <= MT_RAM0_TOP; offset <<= 1) {
        for(shortoffset = second; shortoffset <= MT_RAM0_TOP; shortoffset <<= 1)
          api->sram_writeshort(0, addr + shortoffset);
        api->sram_writebyte(0x00, addr);
        api->sram_writebyte(0xff, addr + offset);
        if(!api->sram_readbyte(addr + offset) || api->sram_readbyte(addr))
          mt_addr_fault(ctx, chip, line);

        shortcount = 0;
        shortline = 0;
        for(shortoffset = second; shortoffset <= MT_RAM0_TOP; shortoffset <<= 1) {
          if(api->sram_readbyte(addr + shortoffset)) shortcount++;
          if(shortcount > 1) {
            shortcount = 1;   /* one report per extra hit, not one per remaining line */
            mt_add(ctx, chip, MEMTEST_KIND_SHORT, line, shortline);
          }
          shortline++;
        }
        line++;
      }
    }
    mt_check_ctrl(ctx, chip);
  }
}

/* ---- RAM1: the 4 Mbit SRAM (U511).  8 bits wide, so no byte enables. -------------- */
static void mt_ram1(const module_api_t *api, mt_ctx_t *ctx) {
  int iter;

  api->mt_select(1);
  for(iter = 0; iter < api->mt_iterations; iter++) {
    uint32_t offset, shortoffset;
    uint8_t data, line, shortline;
    int localerror;

    for(offset = 1; offset <= MT_RAM1_TOP; offset <<= 1) api->sram_writebyte(0x00, offset);

    api->sram_writebyte(0x00, 0);
    line = 0;
    for(data = 1; data; data <<= 1) {
      localerror = 0;
      api->sram_writebyte(data, 0);
      if(!(api->sram_readbyte(0) & data)) localerror = 1;
      api->sram_writebyte(~data, 0);
      if(api->sram_readbyte(0) & data) localerror = 1;
      if(localerror) mt_data_fault(ctx, MEMTEST_CHIP_RAM1, line);
      line++;
    }

    line = 0;
    for(offset = 1; offset <= MT_RAM1_TOP; offset <<= 1) {
      for(shortoffset = 1; shortoffset <= MT_RAM1_TOP; shortoffset <<= 1)
        api->sram_writebyte(0, shortoffset);
      api->sram_writebyte(0x00, 0);
      api->sram_writebyte(0xff, offset);
      if(!api->sram_readbyte(offset) || api->sram_readbyte(0))
        mt_addr_fault(ctx, MEMTEST_CHIP_RAM1, line);

      shortline = 0;
      for(shortoffset = 1; shortoffset <= MT_RAM1_TOP; shortoffset <<= 1) {
        /* Unlike RAM0 there is no base offset here, so the line under test reads back
           its OWN 0xff -- skip it or every line would report a short with itself. */
        if(line != shortline && api->sram_readbyte(shortoffset))
          mt_add(ctx, MEMTEST_CHIP_RAM1, MEMTEST_KIND_SHORT, line, shortline);
        shortline++;
      }
      line++;
    }
  }
  mt_check_ctrl(ctx, MEMTEST_CHIP_RAM1);
}

/* ---- RAM0 and RAM1: the cell sweep ------------------------------------------------ */

static void mt_cell_led(const module_api_t *api, memtest_cell_t *cc) {
  uint32_t now = api->ticks();
  if(now - cc->led_at < MT_LED_TICKS) return;
  cc->led_at = now;
  cc->led ^= 1;
  api->led(cc->led);
}

/* RAM1 is swept ONE BYTE AT A TIME, not in a burst like RAM0, which is what the diagnostic
   firmware's test_mem does and is worth keeping.  The burst version of this reported
   thousands of scattered bad bytes on a board whose wiring walk passes clean and whose
   RAM0 sweep is spotless -- and since no runtime core even drives RAM1, there is no
   independent evidence to check that against.  Per-byte accesses are the same path
   test_memconn uses on this array and the same one the official image trusts, so a fault
   reported here is a fault two different methods agree on.  512 KB at ~4 us/byte is ~4 s. */
static void mt_cell_ram1(const module_api_t *api, memtest_cell_t *cc) {
  uint32_t i;
  uint8_t data;

  for(i = 0; i < MT_RAM1_SIZE; i++) {
    if(!(i & MT_LED_MASK)) mt_cell_led(api, cc);
    api->sram_writebyte(MT_PATTERN(i), i);
  }
  for(i = 0; i < MT_RAM1_SIZE; i++) {
    if(!(i & MT_LED_MASK)) mt_cell_led(api, cc);
    data = api->sram_readbyte(i);
    if(data == MT_PATTERN(i)) continue;
    if(!cc->bad) {
      cc->addr = i;
      cc->got  = data;
      cc->info |= MEMTEST_CELL_RAM1;
    }
    if(cc->bad < MEMTEST_CELL_BAD_MAX) cc->bad++;
  }
}

static void mt_cells(const module_api_t *api, memtest_cell_t *cc) {
  api->mt_select(0);
  api->mt_pass(cc, MT_RAM0_SIZE, 0);
  if(!(cc->info & MEMTEST_CELL_TIMEOUT)) api->mt_pass(cc, MT_RAM0_SIZE, 1);

  /* RAM1 only if RAM0 finished: a timeout means the ready line is gone, and hammering a
     second array through the same dead handshake just costs another wait bound. */
  if(!(cc->info & MEMTEST_CELL_TIMEOUT)) {
    api->mt_select(1);
    mt_cell_ram1(api, cc);
  }

  /* Back to RAM0 for the cleanup above -- the RAM1 sweep left the window there. */
  api->mt_select(0);
  api->sram_memset(MT_STATE_PAGE_ADDR, MT_STATE_PAGE_SIZE, 0);

  /* The nav-SFX PCM bodies live at SRAM_MENU_SFX_ADDR (0xCC0000..0xCFFFFF), so the sweep
     just overwrote all four with the address pattern -- and the slot table that says they
     are loaded is MCU .bss, which a menu reload does not touch.  Without this the next
     blip streams the pattern into the DAC: a loud tone that stays until a power cycle. */
  api->sfx_forget();

  api->led(0);
  cc->info |= MEMTEST_CELL_RAN;
}

/* ---- the module ------------------------------------------------------------------- */

static void mt_blk_init(const module_api_t *api, memtest_blk_t *blk, uint8_t state) {
  mt_zero(blk, sizeof(*blk));
  blk->magic[0] = MEMTEST_MAGIC0;
  blk->magic[1] = MEMTEST_MAGIC1;
  blk->version  = MEMTEST_VERSION;
  blk->state    = state;
  blk->chips    = api->ram0_chips;
}

/* Run the whole thing: fpga_test up, walk both RAMs, sweep every cell (unless the walk
   already failed), publish the block.  Returns 0 when fpga_test is not on the card (nothing
   was touched: the menu reports it in place) and 1 otherwise: the SNES is halted, the FPGA
   is on the test core and the PSRAM scribbled over, and the caller cold-boots the menu. */
int mtmod_start(void *st, const module_api_t *api, const char *path);
void mtmod_ctl(void *st, uint32_t param);
void mtmod_idle(void *st);
void mtmod_stop(void *st);

int mtmod_start(void *st, const module_api_t *api, const char *path) {
  mt_ctx_t ctx;
  memtest_cell_t cc;
  memtest_blk_t blk;
  uint8_t i;
  int core;
  (void)path;

  ((mtmod_t *)st)->api = api;
  mt_zero(&ctx, sizeof(ctx));
  mt_zero(&cc, sizeof(cc));
  core = api->mt_core();
  if(core <= 0) {
    mt_blk_init(api, &blk, core ? MEMTEST_STATE_COREFAIL : MEMTEST_STATE_NOCORE);
    api->sram_write(&blk, SRAM_MEMTEST_ADDR, sizeof(blk));
    return core ? 1 : 0;
  }

  mt_ram0(api, &ctx);
  mt_ram1(api, &ctx);
  mt_check_fpgactrl(&ctx);

  /* The sweep only says something over wiring that already checked out: one open address
     line aliases half the array onto the other half, so every cell in it reads back wrong
     and the real answer disappears under millions of "bad cells". */
  if(ctx.nfind) cc.info = MEMTEST_CELL_RAN | MEMTEST_CELL_SKIPPED;
  else mt_cells(api, &cc);

  /* The block lives in PSRAM, and both mt_ram1 and mt_cells leave the MCU window on RAM1. */
  api->mt_select(0);

  mt_blk_init(api, &blk, MEMTEST_STATE_DONE);
  blk.nfind = ctx.nfind;
  if(ctx.nfind || cc.bad) blk.flags |= MEMTEST_FLAG_FAIL;
  if(ctx.truncated) blk.flags |= MEMTEST_FLAG_TRUNCATED;
  for(i = 0; i < ctx.nfind; i++) blk.findings[i] = ctx.find[i];
  blk.cell_bad[0]  = cc.bad & 0xff;
  blk.cell_bad[1]  = (cc.bad >> 8) & 0xff;
  blk.cell_bad[2]  = (cc.bad >> 16) & 0xff;
  blk.cell_got     = cc.got;
  blk.cell_addr[0] = cc.addr & 0xff;
  blk.cell_addr[1] = (cc.addr >> 8) & 0xff;
  blk.cell_addr[2] = (cc.addr >> 16) & 0xff;
  blk.cell_info    = cc.info;
  api->sram_write(&blk, SRAM_MEMTEST_ADDR, sizeof(blk));
  return 1;
}

/* One command, all of it inside start: nothing to control or poll, nothing held. */
void mtmod_ctl(void *st, uint32_t param) { (void)st; (void)param; }
void mtmod_idle(void *st) { (void)st; }
void mtmod_stop(void *st) { (void)st; }

MODULE_DEFINE(mtmod, mtmod_t)
