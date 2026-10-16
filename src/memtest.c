/* sd2snes+ -- RAM connection test: the firmware's part.  The test (the wiring walks, the
 * fault analysis, the result block) is the card module memtest.bin, src/modules/memtest/,
 * which has the whole story.  What stays here is board-specific or has to be fast: putting
 * fpga_test up, and the two 16 MB SPI bursts of the cell sweep. */

#include "config.h"
#include "uart.h"
#include "ff.h"
#include "fpga.h"
#include "fpga_spi.h"
#include "led.h"
#include "memmap.h"
#include "memory.h"
#include "memtest.h"
#include "snes.h"
#include "timer.h"
#include "dirindex.h"

int memtest_core(void) {
  FILINFO fno;
  fno.lfname = NULL;
  /* asked BEFORE the SNES is halted, so a missing core is refused over the live screen */
  if(f_stat((TCHAR *)FPGA_MEMTEST, &fno) != FR_OK) return 0;
  /* the test reconfigures the FPGA out from under the running menu: halt the SNES first,
     it is executing the menu out of PSRAM */
  assert_reset();
  dirindex_drop();                    /* the test writes all of PSRAM */
  fpga_pgm((uint8_t *)FPGA_MEMTEST);
  /* fpga_pgm sets fpga_config ONLY on success and returns quietly when it cannot open the
     file, so this is the one check that separates "the test core is up" from "the BASE
     core is still up" -- under base, fpga_select_mem means something else entirely (0xee is
     SET213F on every runtime core) and the test would report confident nonsense. */
  return fpga_config == FPGA_MEMTEST ? 1 : -1;
}

/* Straight writeled(), never toggle_write_led(): that one passes ~state, and in PWM mode
   -- which is how the menu runs, see led_pwm() in main() -- ~1 is still non-zero. */
static void mt_cell_led(memtest_cell_t *cc) {
  tick_t now = getticks();
  if(now - cc->led_at < MT_LED_TICKS) return;
  cc->led_at = now;
  cc->led ^= 1;
  writeled(cc->led);
}

void memtest_pass(memtest_cell_t *cc, uint32_t len, uint8_t verify) {
  uint32_t i;
  uint8_t timeout = 0, data;

  set_mcu_addr(0);
  FPGA_SELECT();
  FPGA_TX_BYTE((verify ? FPGA_CMD_READMEM : FPGA_CMD_WRITEMEM) | FPGA_MEM_AUTOINC);
  for(i = 0; i < len; i++) {
    if(!(i & MT_LED_MASK)) mt_cell_led(cc);
    if(!verify) {
      FPGA_TX_BYTE(MT_PATTERN(i));
      FPGA_WAIT_RDY_TO_INLINE(timeout);
      if(timeout) break;
      continue;
    }
    FPGA_WAIT_RDY_TO_INLINE(timeout);
    if(timeout) break;
    data = FPGA_RX_BYTE();
    if(data == MT_PATTERN(i)) continue;
    /* only the FIRST bad byte is located: one address is what a user can act on */
    if(!cc->bad) {
      cc->addr = i;
      cc->got  = data;
    }
    if(cc->bad < MEMTEST_CELL_BAD_MAX) cc->bad++;
  }
  FPGA_DESELECT();
  if(timeout) cc->info |= MEMTEST_CELL_TIMEOUT;
}

void memtest_state(uint8_t state) {
  uint8_t b[4] = { MEMTEST_MAGIC0, MEMTEST_MAGIC1, MEMTEST_VERSION, state };
  sram_writeblock(b, SRAM_MEMTEST_ADDR, sizeof(b));
}
