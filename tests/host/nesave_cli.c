/* Conformance test for the NES battery (src/nesave.c), compiled against the REAL source over a flat
   fake PSRAM and a fake SD card.  What it pins down has no hardware fallback and fails silently:
   a save that is cut at the persisted size, never written when nothing changed, written once the
   game has stopped touching the window, flushed on reset even when the scan had not caught up,
   never carried over to the next game, and -- the part that must not regress -- never waiting on
   an FPGA that stopped answering: the PSRAM is reached through a fake FPGA whose MCU_RDY line can
   hang, and the unbounded wait ends the process. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "memory.h"
#include "fileops.h"
#include "cfg.h"
#include "snes.h"
#include "timer.h"
#include "led.h"
#include "nesave.h"
#include "fpga.h"
#include "fpga_spi.h"

#define WIN NESAVE_WINDOW_ADDR
static uint8_t *psram;
cfg_t CFG = { 1 };
FRESULT file_res;
BYTE file_buf[512];

/* ---- clock / knobs ---- */
static tick_t now;
tick_t getticks(void) { return now; }
void writeled(unsigned int on) { (void)on; }
void rdyled(unsigned int on) { (void)on; }
static int garble;                /* PSRAM reads come back wrong (the sentinel too): sram_reliable fails */
static int reset_line;            /* get_snes_reset() */
static int fpga_alive = 1;
uint8_t fpga_test(void) { return fpga_alive ? FPGA_TEST_TOKEN : 0; }
uint8_t get_snes_reset(void) { return (uint8_t)reset_line; }
static int fail_saves;            /* next N card writes fail */

/* ---- the fake FPGA: set address (0x00 + 3 bytes), read stream (0x88), write stream (0x98) ---- */
static int  f_state, f_naddr;     /* f_state: 0 command, 1 address bytes, 2 read, 3 write */
static uint32_t f_addr;
static long hang_after = -1;      /* >0: the Nth MCU_RDY wait from now on hangs the line */
static int  hung;                 /* MCU_RDY stuck low */
static unsigned long waits;       /* every MCU_RDY wait (bounded or not) */
static unsigned long timeouts;
void fake_select(void) { f_state = 0; }
void fake_deselect(void) { f_state = 0; }
void fake_tx(uint8_t b) {
  switch(f_state) {
  case 0:
    if(b == (FPGA_CMD_SETADDR | FPGA_TGT_MEM)) { f_state = 1; f_naddr = 0; f_addr = 0; }
    else if(b == (FPGA_CMD_READMEM | FPGA_MEM_AUTOINC)) f_state = 2;
    else if(b == 0x98) f_state = 3;
    else { printf("FAIL: fake FPGA got unknown command %02x\n", b); exit(98); }
    break;
  case 1: f_addr = (f_addr << 8) | b; if(++f_naddr == 3) f_state = 0; break;
  case 3: psram[f_addr++] = b; break;
  default: break;
  }
}
uint8_t fake_rx(void) {
  uint8_t v = psram[f_addr++];
  return garble ? (uint8_t)(v ^ 0x55) : v;
}
int fake_wait(int bounded) {
  waits++;
  if(hang_after > 0 && --hang_after == 0) hung = 1;
  if(!hung) return 0;
  if(!bounded) { printf("FAIL: UNBOUNDED wait on a hung FPGA (the MCU would wedge)\n"); exit(99); }
  timeouts++;
  return 1;
}
uint32_t crc32_update(uint32_t crc, const unsigned char b) {
  crc ^= b;
  for(int i = 0; i < 8; i++) crc = (crc >> 1) ^ (0xEDB88320u & -(crc & 1));
  return crc;
}

/* ---- fake card ---- */
typedef struct { char path[64]; uint8_t *data; uint32_t len; int used; unsigned writes; } fakefile_t;
static fakefile_t card[8];
static unsigned total_writes, attempts;
static uint32_t last_write_len;
static int hang_on_write_open;    /* hang the FPGA line as soon as the next save file is opened */

static void srm_name(char *out, const char *game) {
  const char *leaf = strrchr(game, '/'); leaf = leaf ? leaf + 1 : game;
  snprintf(out, 64, "/saves/nes/%.2s/%s", leaf, leaf);
  char *dot = strrchr(out, '.'); if(dot) *dot = 0;
  strcat(out, ".srm");
}
static fakefile_t *card_find(const char *path, int create) {
  for(int i = 0; i < 8; i++) if(card[i].used && !strcmp(card[i].path, path)) return &card[i];
  if(!create) return NULL;
  for(int i = 0; i < 8; i++) if(!card[i].used) { card[i].used = 1; snprintf(card[i].path, 64, "%s", path); return &card[i]; }
  abort();
}
int srm_path_for_load(char *buf, size_t buflen, uint8_t *filename) { (void)buflen; srm_name(buf, (const char*)filename); return 0; }
int srm_path_for_save(char *buf, size_t buflen, uint8_t *filename) { (void)buflen; srm_name(buf, (const char*)filename); return 0; }
static fakefile_t *cur; static uint32_t cur_pos; static int cur_writing, cur_bad;
void file_open(uint8_t *filename, BYTE flags) {
  cur_pos = 0; cur_writing = !!(flags & FA_WRITE); cur_bad = 0;
  if(cur_writing) {
    attempts++;
    cur = card_find((const char*)filename, 1);
    free(cur->data); cur->data = NULL; cur->len = 0;            /* FA_CREATE_ALWAYS truncates */
    if(hang_on_write_open) { hang_on_write_open = 0; hang_after = 40; }
    file_res = FR_OK;
    return;
  }
  cur = card_find((const char*)filename, 0);
  file_res = cur ? FR_OK : FR_NO_FILE;
}
UINT file_write(size_t n) {
  if(!cur || !cur_writing) { file_res = FR_DISK_ERR; return 0; }
  if(fail_saves) { fail_saves--; file_res = FR_DISK_ERR; cur_bad = 1; return 0; }
  cur->data = realloc(cur->data, cur->len + n);
  memcpy(cur->data + cur->len, file_buf, n); cur->len += (uint32_t)n;
  return (UINT)n;
}
void file_close(void) {
  if(cur && cur_writing && !cur_bad && !file_res && !hung) {   /* a hang cut the file short: not a save */ cur->writes++; total_writes++; last_write_len = cur->len; }
  cur = NULL; cur_writing = 0;
}
UINT file_read(void) {
  if(!cur) return 0;
  uint32_t n = cur->len - cur_pos; if(n > 512) n = 512;
  memcpy(file_buf, cur->data + cur_pos, n); cur_pos += n; return n;
}

/* ---- checks ---- */
static int fails;
#define CHECK(cond, ...) do { if(!(cond)) { \
    printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); fails++; } } while(0)

static uint8_t *game = (uint8_t*)"/games/Zelda.nes";
static void reset_world(void) {
  memset(psram, 0, 0x1000000);
  psram[SRAM_SCRATCHPAD + 0] = 0x78; psram[SRAM_SCRATCHPAD + 1] = 0x56;       /* the sram_reliable() sentinel */
  psram[SRAM_SCRATCHPAD + 2] = 0x34; psram[SRAM_SCRATCHPAD + 3] = 0x12;
  garble = 0; hang_after = -1; hung = 0; hang_on_write_open = 0; waits = 0; timeouts = 0; f_state = 0;
  for(int i = 0; i < 8; i++) { free(card[i].data); memset(&card[i], 0, sizeof(card[i])); }
  total_writes = 0; attempts = 0; last_write_len = 0; reset_line = 0; fail_saves = 0; CFG.enable_autosave = 1;
  file_res = FR_OK; now = 1000;
  nesave_detach();
}
/* one in-game loop pass: snes_main_loop runs every 250 ms */
static void ticks(int n) { while(n--) { now += 25; nesave_tick(game); } }
static void put_card(const char *path, const uint8_t *d, uint32_t len) {
  fakefile_t *f = card_find(path, 1); f->data = malloc(len); memcpy(f->data, d, len); f->len = len;
}

int main(void) {
  psram = calloc(1, 0x1000000);

  /* sizes */
  CHECK(nesave_size(0, 0, 0) == 0,            "no battery => 0");
  CHECK(nesave_size(0, 1, 0x70) == 0,         "no battery wins over NVRAM nibble");
  CHECK(nesave_size(1, 0, 0) == 8192,         "iNES 1.0 battery => whole window");
  CHECK(nesave_size(1, 0, 0x50) == 8192,      "iNES 1.0 ignores byte 10");
  CHECK(nesave_size(1, 1, 0x00) == 8192,      "NES 2.0 with no NVRAM size => window");
  CHECK(nesave_size(1, 1, 0x50) == 2048,      "NES 2.0 NVRAM shift 5 => 2048");
  CHECK(nesave_size(1, 1, 0x10) == 128,       "NES 2.0 NVRAM shift 1 => 128");
  CHECK(nesave_size(1, 1, 0x70) == 8192,      "NES 2.0 NVRAM 8 KB");
  CHECK(nesave_size(1, 1, 0xF0) == 8192,      "NES 2.0 oversize NVRAM capped at the window");

  /* 1. no battery: window changes never reach the card */
  reset_world();
  nesave_attach(game, 0, 0, 0);
  psram[WIN + 5] = 0x42;
  ticks(40);
  CHECK(total_writes == 0, "no battery wrote %u times", total_writes);

  /* 2. battery, no .srm yet: untouched RAM is never written */
  reset_world();
  nesave_attach(game, 1, 0, 0);
  ticks(60);
  CHECK(total_writes == 0, "untouched window wrote %u times", total_writes);

  /* 3. the game writes: exactly one save, the whole window, after it settles; then silence */
  psram[WIN + 0x10] = 0xDE; psram[WIN + 0x1FFF] = 0xAD;
  ticks(1);
  CHECK(total_writes == 0, "wrote before the image settled");
  ticks(20);
  CHECK(total_writes == 1, "settled change wrote %u times (want 1)", total_writes);
  CHECK(last_write_len == 8192, "save length %u (want 8192)", last_write_len);
  { char p[64]; srm_name(p, (const char*)game); fakefile_t *f = card_find(p, 0);
    CHECK(f && f->len == 8192 && f->data[0x10] == 0xDE && f->data[0x1FFF] == 0xAD, "saved image does not match the window"); }
  ticks(60);
  CHECK(total_writes == 1, "rewrote an unchanged save (%u)", total_writes);

  /* 4. load: file longer than the window is cut at 8192, nothing past the window is touched */
  reset_world();
  { static uint8_t big[20000]; for(int i = 0; i < 20000; i++) big[i] = (uint8_t)(i * 7 + 1);
    put_card("/saves/nes/Ze/Zelda.srm", big, sizeof(big));
    memset(psram + WIN + 8192, 0x5A, 4096);
    nesave_attach(game, 1, 0, 0);
    CHECK(!memcmp(psram + WIN, big, 8192), "first 8192 bytes not loaded");
    CHECK(psram[WIN + 8192] == 0x5A && psram[WIN + 8192 + 4095] == 0x5A, "load ran past the window");
    ticks(40);
    CHECK(total_writes == 0, "freshly loaded save was rewritten (%u)", total_writes); }

  /* 5. load: short file leaves the rest zeroed */
  reset_world();
  { uint8_t sm[700]; memset(sm, 0x77, sizeof(sm)); put_card("/saves/nes/Ze/Zelda.srm", sm, sizeof(sm));
    nesave_attach(game, 1, 0, 0);
    CHECK(psram[WIN + 699] == 0x77 && psram[WIN + 700] == 0, "short file mis-loaded"); }

  /* 6. NES 2.0 NVRAM of 2 KB: load and save are cut at 2048 */
  reset_world();
  { static uint8_t big[8192]; memset(big, 0x33, sizeof(big)); put_card("/saves/nes/Ze/Zelda.srm", big, sizeof(big));
    nesave_attach(game, 1, 1, 0x50);
    CHECK(psram[WIN + 2047] == 0x33 && psram[WIN + 2048] == 0, "2 KB NVRAM loaded %s", psram[WIN + 2048] ? "too much" : "too little");
    psram[WIN + 100] = 1; ticks(20);
    CHECK(total_writes == 1 && last_write_len == 2048, "2 KB NVRAM wrote %u x %u bytes", total_writes, last_write_len); }

  /* 7. a change in the LAST chunk is seen (the scan covers the whole window) */
  reset_world();
  nesave_attach(game, 1, 0, 0);
  psram[WIN + 8191] = 9; ticks(20);
  CHECK(total_writes == 1, "last-byte change wrote %u", total_writes);

  /* 8. contents that never settle still reach the card, rarely */
  reset_world();
  nesave_attach(game, 1, 0, 0);
  for(int i = 0; i < 400; i++) { psram[WIN + 3] = (uint8_t)(i + 1); ticks(1); }
  CHECK(total_writes >= 1 && total_writes <= 6, "ever-changing RAM wrote %u times", total_writes);

  /* 9. a failing card: retried, then recovers, and dirty state is not lost */
  reset_world();
  nesave_attach(game, 1, 0, 0);
  fail_saves = 2; psram[WIN + 1] = 1; ticks(40);
  CHECK(total_writes == 1, "after 2 failed saves the 3rd should land (writes=%u)", total_writes);
  CHECK(fail_saves == 0, "retries did not happen");

  /* 9b. a dead card is retried after another stable stretch, not on every pass */
  reset_world();
  nesave_attach(game, 1, 0, 0);
  fail_saves = 1000; psram[WIN + 1] = 1; ticks(8);     /* 4 passes: settle after 3, first attempt on the 3rd */
  CHECK(attempts == 1, "first attempt expected, got %u", attempts);
  ticks(4);                                            /* 2 more passes: still inside the back-off */
  CHECK(attempts == 1, "retried inside the back-off (%u attempts)", attempts);
  ticks(40);
  CHECK(attempts >= 2 && attempts <= 8, "retry cadence off (%u attempts in 26 passes)", attempts);

  /* 10. unreliable PSRAM: a pass that cannot be read cleanly saves nothing */
  reset_world();
  nesave_attach(game, 1, 0, 0);
  garble = 1; psram[WIN + 1] = 1; ticks(40);
  CHECK(total_writes == 0, "wrote while the PSRAM was unreliable");
  garble = 0; ticks(20);
  CHECK(total_writes == 1, "did not save once reads were reliable again (%u)", total_writes);

  /* 11. reset line seen mid-scan (calc_sram_crc bails): nothing written */
  reset_world();
  nesave_attach(game, 1, 0, 0);
  reset_line = 1; psram[WIN + 1] = 1; ticks(40);
  CHECK(total_writes == 0, "wrote with the console in reset (%u)", total_writes);

  /* 12. autosave off: the scan is silent, the reset flush still saves */
  reset_world();
  nesave_attach(game, 1, 0, 0);
  CFG.enable_autosave = 0; psram[WIN + 1] = 1; ticks(40);
  CHECK(total_writes == 0, "autosave off wrote");
  nesave_flush_inreset(game);
  CHECK(total_writes == 1, "reset flush skipped a dirty window (%u)", total_writes);
  nesave_flush_inreset(game);
  CHECK(total_writes == 1, "reset flush rewrote a clean window (%u)", total_writes);

  /* 12b. dead FPGA: the reset flush does not touch it */
  reset_world();
  nesave_attach(game, 1, 0, 0);
  psram[WIN + 1] = 1; fpga_alive = 0; nesave_flush_inreset(game); fpga_alive = 1;
  CHECK(total_writes == 0, "flush wrote with a dead FPGA");

  /* 13. reset flush catches a change the scan had SEEN but not yet settled */
  reset_world();
  nesave_attach(game, 1, 0, 0);
  psram[WIN + 9] = 9; ticks(2);            /* pass complete: dirty, not stable */
  CHECK(total_writes == 0, "settled too early");
  nesave_flush_inreset(game);
  CHECK(total_writes == 1, "reset flush lost a pending change (%u)", total_writes);

  /* 14. rate limit: many calls inside one period do one chunk of work */
  reset_world();
  nesave_attach(game, 1, 0, 0);
  psram[WIN + 1] = 1;
  for(int i = 0; i < 50; i++) nesave_tick(game);     /* clock frozen */
  now += 25; for(int i = 0; i < 50; i++) nesave_tick(game);
  /* two chunks make a pass; the frozen-clock burst must not have completed a stable image */
  CHECK(total_writes == 0, "rate limit not honoured");

  /* 15. detach: the next game's loads must not inherit this battery */
  reset_world();
  nesave_attach(game, 1, 0, 0);
  nesave_detach();
  psram[WIN + 1] = 1; ticks(40); nesave_flush_inreset(game);
  CHECK(total_writes == 0, "wrote after detach (%u)", total_writes);

  /* 16. re-attach to another game starts a clean baseline and its own file name */
  reset_world();
  nesave_attach(game, 1, 0, 0);
  psram[WIN + 1] = 1; ticks(20);
  uint8_t *game2 = (uint8_t*)"/games/Dragon Warrior.nes";
  nesave_attach(game2, 1, 0, 0);
  memset(psram + WIN, 0, 8192); psram[WIN + 2] = 2;
  { unsigned before = total_writes; for(int i = 0; i < 20; i++) { now += 25; nesave_tick(game2); }
    CHECK(total_writes == before + 1 && card_find("/saves/nes/Dr/Dragon Warrior.srm", 0), "second game's save not written under its own name"); }

  /* 17. a pass whose chunk could not be read cleanly is thrown away, not kept: with a loaded save in the
         window, one garbled chunk must never end up as a "change" that gets written */
  reset_world();
  { static uint8_t good[8192]; for(int i = 0; i < 8192; i++) good[i] = (uint8_t)(i * 3 + 1);
    put_card("/saves/nes/Ze/Zelda.srm", good, sizeof(good));
    nesave_attach(game, 1, 0, 0);
    CHECK(!memcmp(psram + WIN, good, 8192), "save not loaded");
    garble = 1; ticks(1); garble = 0;
    ticks(80);
    CHECK(total_writes == 0, "a half-read pass became a save (%u writes)", total_writes); }

  /* 18. a dead card with RAM that never settles: the retry cadence comes from the back-off counter
         (about one attempt per 13 passes), not from the stable-image rule */
  reset_world();
  nesave_attach(game, 1, 0, 0);
  fail_saves = 100000;
  for(int i = 0; i < 400; i++) { psram[WIN + 3] = (uint8_t)(i + 1); ticks(1); }
  CHECK(attempts >= 10 && attempts <= 13, "dead card cadence off: %u attempts in 200 passes (want 10..13)", attempts);

  /* 19. the FPGA stops answering in the middle of an autosave pass: the tick returns (the unbounded wait
         would end the process), writes nothing, gives the autosave up and never touches the line again */
  reset_world();
  nesave_attach(game, 1, 0, 0);
  psram[WIN + 1] = 1;
  hang_after = 3000;                              /* inside the first 4 KB chunk */
  ticks(10);
  CHECK(hung, "hang point not reached");
  CHECK(total_writes == 0, "wrote with a hung FPGA (%u)", total_writes);
  { unsigned long w = waits; ticks(30);
    CHECK(waits == w, "the autosave kept waiting on a dead FPGA (%lu waits)", waits - w); }
  hung = 0; hang_after = -1;                      /* the line comes back: the reset flush still saves */
  nesave_flush_inreset(game);
  CHECK(total_writes == 1, "reset flush after a hang wrote %u", total_writes);

  /* 19b. ... and in the reliability probe that follows the chunk (wait #4099: the first sentinel byte) */
  reset_world();
  nesave_attach(game, 1, 0, 0);
  psram[WIN + 1] = 1;
  hang_after = 1 + 4096 + 1 + 1;
  ticks(1);
  CHECK(hung, "reliability probe hang point not reached");
  { unsigned long w = waits; ticks(10); CHECK(waits == w && total_writes == 0, "probe hang: still waiting / wrote"); }

  /* 20. the FPGA hangs while the .srm is being loaded: nothing is armed, so nothing can be written over
         the good save with a window that never got it */
  reset_world();
  { static uint8_t good[8192]; memset(good, 0x5C, sizeof(good)); put_card("/saves/nes/Ze/Zelda.srm", good, sizeof(good)); }
  hang_after = 100;
  nesave_attach(game, 1, 0, 0);
  CHECK(hung, "load hang point not reached");
  hung = 0; hang_after = -1;
  psram[WIN + 1] = 1; ticks(60); nesave_flush_inreset(game);
  CHECK(total_writes == 0 && attempts == 0, "armed after a failed attach (%u writes, %u attempts)", total_writes, attempts);

  /* 21. the FPGA hangs while the file is being written: the write fails cleanly (no counted save), the
         autosave goes dead, and the reset flush completes the save once the line is back */
  reset_world();
  nesave_attach(game, 1, 0, 0);
  psram[WIN + 1] = 1; hang_on_write_open = 1;
  ticks(30);
  CHECK(attempts == 1 && total_writes == 0, "mid-write hang: %u attempts, %u saves", attempts, total_writes);
  hung = 0; hang_after = -1;
  nesave_flush_inreset(game);
  CHECK(total_writes == 1, "flush after a mid-write hang wrote %u", total_writes);
  { fakefile_t *f = card_find("/saves/nes/Ze/Zelda.srm", 0);
    CHECK(f && f->len == 8192 && f->data[1] == 1, "the completed save is not the whole window"); }

  /* 22. the line is gone right when the write starts: the old .srm survives, because the file is only
         truncated after a probe of the FPGA (reset flush: the scan is 1 + 8192 waits, the probe is #8194) */
  reset_world();
  { static uint8_t old[8192]; for(int i = 0; i < 8192; i++) old[i] = (uint8_t)(i ^ 0x3C);
    put_card("/saves/nes/Ze/Zelda.srm", old, sizeof(old));
    nesave_attach(game, 1, 0, 0);
    psram[WIN + 1] ^= 0xFF;
    hang_after = 1 + 8192 + 1;
    nesave_flush_inreset(game);
    CHECK(hung, "probe hang point not reached");
    fakefile_t *f = card_find("/saves/nes/Ze/Zelda.srm", 0);
    CHECK(attempts == 0, "the save file was opened (truncated) although the FPGA was already gone");
    CHECK(f && f->len == 8192 && !memcmp(f->data, old, 8192), "the old save was damaged"); }

  /* 23. a hung line at the reset flush: returns, writes nothing */
  reset_world();
  nesave_attach(game, 1, 0, 0);
  psram[WIN + 1] = 1; hung = 1;
  nesave_flush_inreset(game);
  CHECK(total_writes == 0 && timeouts > 0, "flush on a hung line: %u writes, %lu timeouts", total_writes, timeouts);

  if(fails) { printf("nesave: %d FAILED\n", fails); return 1; }
  printf("nesave: all checks passed\n");
  return 0;
}
