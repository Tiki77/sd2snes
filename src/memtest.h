#ifndef _MEMTEST_H
#define _MEMTEST_H

#include <stddef.h>
#include <stdint.h>

/* --- RAM connection test, published as a binary block the MENU renders ------------
 *
 * This is the RAM-wiring half of the standalone diagnostic firmware (src/tests/tests.c,
 * test_memconn) brought into the normal firmware so a user can triage "is my cartridge
 * broken or is it the fork?" without a serial cable and without flashing a separate
 * diagnostic image.  It walks the data and address lines of the PSRAM chips (RAM0,
 * U501/U502) and of the 4 Mbit SRAM (RAM1, U511) looking for opens, stuck lines and
 * shorts between address lines.
 *
 * WHY A BINARY BLOCK: the diagnostic firmware reports through LOGPRINT, which does not
 * exist on Mk.II (printf is a no-op there, see config-mk2) and which would cost a full
 * localized string table in MCU flash on top.  So the firmware publishes FINDINGS and the
 * menu formats them from its own language pool -- the same split sysinfo.h documents.
 *
 * THE BYTE OFFSETS ARE THE INTERFACE.  They are mirrored as MT_* in snes/memmap.i65; the
 * _Static_asserts below pin them so a field inserted in the middle breaks the build here
 * instead of silently shifting the menu's reads.
 *
 * THE TEST NEEDS THE fpga_test CORE.  Only that bitstream wires the MCU to RAM1 at all
 * (the runtime cores leave RAM_DATA/RAM_ADDR undriven) and it is the same core the
 * official diagnostic uses, so a result here is comparable with one from the official
 * diagnostic firmware.  Reconfiguring the FPGA means the SNES must be held in reset for
 * the duration and the menu cold-booted afterwards, exactly like the patched-ROM export.
 *
 * ONE TEST, TWO HALVES.  The walk above (~10s) always runs first, and then a sweep of
 * every cell -- one write pass and one verify pass over all of RAM0 and RAM1, another
 * ~20s.  The sweep is skipped (MEMTEST_CELL_SKIPPED) when the walk already found a fault,
 * so a broken board does not pay for it.
 *
 * WHY THE CELL SWEEP EXISTS AT ALL.  The official diagnostic never sweeps either: its
 * test_mem() has no caller and the SNES-side memtest: in snes/tests/tests.a65 is commented
 * out, so the only cells it ever verifies are the first 1 MiB of 16, as a side effect of
 * test_sddma().  A cell that fails above 1 MiB passes its whole suite, which is exactly the
 * "large ROMs break, small ones are fine" report the sweep is meant to settle.
 */
#define MEMTEST_MAGIC0               ('M')
#define MEMTEST_MAGIC1               ('T')
#define MEMTEST_VERSION              (2)

/* Block state.  Persistent in PSRAM across the menu reload -- that reload is what carries
   the result back to the user, since the SNES is in reset while the test runs. */
#define MEMTEST_STATE_NONE           (0)  /* nothing to report (also the cold-boot value the firmware writes once) */
#define MEMTEST_STATE_DONE           (1)  /* a test ran; findings[] is valid */
#define MEMTEST_STATE_NOCORE         (2)  /* refused: /sd2snes/fpga_test.<ext> is not on the card. Answered WITHOUT halting the SNES, so the menu reports it in place (no reload) */
#define MEMTEST_STATE_RUNNING        (3)  /* sentinel the MENU writes before the command; the firmware always overwrites it, so one surviving the ACK means this firmware has no memtest handler */
#define MEMTEST_STATE_SEEN           (4)  /* MENU-ONLY: a DONE result already auto-shown after the reload. The firmware must never write it -- listed here so the value stays reserved (lockstep with MT_ST_SEEN in snes/memmap.i65) */
#define MEMTEST_STATE_NOMODULE       (6)  /* the test is the card module memtest.bin and it is missing, damaged or of another version. Answered in place like NOCORE (nothing was halted) */
#define MEMTEST_STATE_COREFAIL       (5)  /* fpga_test was on the card but did not configure (truncated/corrupt, or it vanished between the f_stat and the load). Reported like a result, i.e. after the reload: by then the SNES is already halted, so there is no in-place path left. Shown with the same message as NOCORE -- the user's next step is the same file either way */

#define MEMTEST_FLAG_FAIL            (0x01)  /* at least one fault was found */
#define MEMTEST_FLAG_TRUNCATED       (0x02)  /* more faults exist than findings[] can hold */

/* cell_info -- the cell sweep's own verdict, kept apart from flags because "the wiring is
   fine but a cell is bad" and "the wiring is bad" are different answers with different
   next steps for the user. */
#define MEMTEST_CELL_RAN             (0x01)  /* the sweep actually ran; without this the cell fields are meaningless and the menu leaves its line blank */
#define MEMTEST_CELL_RAM1            (0x02)  /* the FIRST bad byte was in RAM1 (U511), so cell_addr is a RAM1 offset; clear means RAM0 */
#define MEMTEST_CELL_TIMEOUT         (0x04)  /* the sweep gave up on FPGA_WAIT_RDY: the counts below are partial and say nothing about the untested remainder */
#define MEMTEST_CELL_SKIPPED         (0x08)  /* the wiring walk already failed, so sweeping was pointless -- a broken address line makes every cell "bad" */

/* cell_bad saturates instead of wrapping: 24 bits cannot hold 16777216, and past a few
   thousand the exact count stops meaning anything anyway. */
#define MEMTEST_CELL_BAD_MAX         (0xFFFFFFUL)

/* findings[].chip */
#define MEMTEST_CHIP_RAM0_1          (0)  /* U501 */
#define MEMTEST_CHIP_RAM0_2          (1)  /* U502 (Mk.III only; Mk.II has a single PSRAM) */
#define MEMTEST_CHIP_RAM1            (2)  /* U511 */
#define MEMTEST_NUM_CHIPS            (3)

/* findings[].kind -- what the menu prints, and what the user should check on the board */
#define MEMTEST_KIND_DATA            (0)  /* data line D<line1> open/stuck */
#define MEMTEST_KIND_ADDR            (1)  /* address line A<line1> open/stuck */
#define MEMTEST_KIND_SHORT           (2)  /* address lines A<line1> and A<line2> look shorted */
#define MEMTEST_KIND_BYTESEL         (3)  /* byte-enable fault: check LB# / UB# (RAM0 only) */
#define MEMTEST_KIND_CTRL            (4)  /* so many faults on one chip that its CE#/OE#/WE# is suspect */
#define MEMTEST_KIND_FPGACTRL        (5)  /* so many faults overall that the FPGA's CE#/OE#/WE# is suspect */

#define MEMTEST_MAX_FINDINGS         (12)

typedef struct __attribute__((__packed__)) _memtest_finding {
  uint8_t chip;   /* +0 MEMTEST_CHIP_* */
  uint8_t kind;   /* +1 MEMTEST_KIND_* */
  uint8_t line1;  /* +2 line number (data/address), or the first of a shorted pair */
  uint8_t line2;  /* +3 second line of a shorted pair; 0 otherwise */
} memtest_finding_t;

typedef struct __attribute__((__packed__)) _memtest_blk {
  uint8_t  magic[2];   /* +0 ($00) 'M','T' */
  uint8_t  version;    /* +2 ($02) MEMTEST_VERSION */
  uint8_t  state;      /* +3 ($03) MEMTEST_STATE_* */
  uint8_t  flags;      /* +4 ($04) MEMTEST_FLAG_* */
  uint8_t  chips;      /* +5 ($05) RAM0 chips actually tested (1 on Mk.II, 2 on Mk.III) */
  uint8_t  nfind;      /* +6 ($06) valid entries in findings[] */
  uint8_t  pad;        /* +7 ($07) */
  memtest_finding_t findings[MEMTEST_MAX_FINDINGS]; /* +8 ($08) 4 bytes each */
  /* --- cell sweep.  These fill the 8 bytes the reservation had left over, which is also
     all the room there will ever be: SRAM_PCMPLAY_ADDR starts at $FF07C0, immediately
     after. 24-bit fields are byte arrays rather than a uint32 so the
     menu can read them a byte at a time without caring about the struct's padding. */
  uint8_t  cell_bad[3];   /* +56 ($38) bad bytes found, little-endian, saturating at MEMTEST_CELL_BAD_MAX */
  uint8_t  cell_got;      /* +59 ($3B) the byte actually read at cell_addr (the expected one is derivable from the address) */
  uint8_t  cell_addr[3];  /* +60 ($3C) address of the FIRST bad byte, little-endian */
  uint8_t  cell_info;     /* +63 ($3F) MEMTEST_CELL_* */
} memtest_blk_t;       /* 64 bytes = the whole reservation at SRAM_MEMTEST_ADDR */

_Static_assert(offsetof(memtest_blk_t, state) == 3, "memtest_blk_t.state must stay at MT_STATE (+3)");
_Static_assert(offsetof(memtest_blk_t, flags) == 4, "memtest_blk_t.flags must stay at MT_FLAGS (+4)");
_Static_assert(offsetof(memtest_blk_t, nfind) == 6, "memtest_blk_t.nfind must stay at MT_NFIND (+6)");
_Static_assert(offsetof(memtest_blk_t, findings) == 8, "memtest_blk_t.findings must stay at MT_FINDINGS (+8)");
_Static_assert(offsetof(memtest_blk_t, cell_bad) == 56, "memtest_blk_t.cell_bad must stay at MT_CELL_BAD (+56)");
_Static_assert(offsetof(memtest_blk_t, cell_got) == 59, "memtest_blk_t.cell_got must stay at MT_CELL_GOT (+59)");
_Static_assert(offsetof(memtest_blk_t, cell_addr) == 60, "memtest_blk_t.cell_addr must stay at MT_CELL_ADDR (+60)");
_Static_assert(offsetof(memtest_blk_t, cell_info) == 63, "memtest_blk_t.cell_info must stay at MT_CELL_INFO (+63)");
_Static_assert(sizeof(memtest_finding_t) == 4, "memtest_finding_t must stay 4 bytes (MT_FIND_SIZE in snes/memmap.i65)");
_Static_assert(sizeof(memtest_blk_t) == 64, "memtest_blk_t must stay 64 bytes (MT_SIZE in snes/memmap.i65) -- it fills the reservation, PCMPLAY starts right after");

/* The test itself is the card module memtest.bin (src/modules/memtest/); the firmware keeps
   only what is board-specific or has to be fast: */

/* Bad-cell sweep state, shared with the module (module_api_t.mt_pass). */
typedef struct memtest_cell {
  uint32_t bad;     /* bad bytes seen, saturating at MEMTEST_CELL_BAD_MAX */
  uint32_t addr;    /* address of the first one */
  uint8_t  got;     /* what was actually read there */
  uint8_t  info;    /* MEMTEST_CELL_* */
  uint8_t  led;     /* current LED state, so the flip does not have to read it back */
  uint32_t led_at;  /* tick of the last flip */
} memtest_cell_t;

/* The diagnostic firmware's pattern (test_mem, src/tests/tests.c).  Every byte differs
   from its neighbours AND from the same offset one 64 KB page up, so an address line that
   aliases reads back as mismatched DATA rather than as a plausible value.  Kept
   byte-identical to that one on purpose: it is what makes a result from here comparable
   with a result from the official diagnostic image. */
#define MT_PATTERN(a)   ((uint8_t)((a) + ((a) >> 8) + ((a) >> 16)))
/* Ticks between LED flips (10 ms each), and how often a sweep looks at the clock.  The
   screen is black and the SNES is in reset for about a minute, so with no LED the console
   is indistinguishable from a hung one. */
#define MT_LED_TICKS    (25)
#define MT_LED_MASK     (0xFFFFUL)

/* Put fpga_test up for the module: 0 = not on the card (nothing touched: the menu reports
   it in place), 1 = configured, -1 = it was there but did not configure.  On 1 and -1 the
   SNES has been halted (assert_reset) and the PSRAM is the test's: the caller cold-boots
   the menu afterwards. */
int memtest_core(void);

/* One pass of the pattern over len bytes of the selected array from address 0: written
   (verify = 0) or read back and counted into cc (verify = 1).  SPI bursts with a bounded
   ready wait: 16 M unbounded waits would wedge the MCU if the core stopped answering. */
void memtest_pass(memtest_cell_t *cc, uint32_t len, uint8_t verify);

/* Park a state in the block (MEMTEST_STATE_NONE at cold boot: PSRAM keeps whatever the
   last power-on left; MEMTEST_STATE_NOCORE when the module itself is missing). */
void memtest_state(uint8_t state);

#endif
