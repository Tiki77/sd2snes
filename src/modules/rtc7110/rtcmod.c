/* SPC7110 / RTC-4513 battery backup, the card module rtc7110.bin.  See ../../spc7110rtc.h
 * for what the battery is for and the sidecar it lives in.
 *
 * A SESSION module (../../module.h): load_rom opens it for an SPC7110 game once the core
 * is configured and before the SNES runs, so start() restores the clock; both game loops
 * call idle(), which keeps the sidecar in step; the next load_rom drops it.  stop() does
 * nothing on purpose: by then the FPGA may already hold another core, and reading the
 * clock from it would write a wrong sidecar.
 *
 * start() answers 1 when the core carries the virtual battery, 0 when it does not (a core
 * from before the battery: the clock is programmed from the console's, as the firmware
 * does itself when this file is missing).  load_saveram plants the factory check's
 * signature on a 0.
 *
 * No globals and no library: the calendar is ../../spc7110rtc.c (pure int32 arithmetic),
 * the clocks and the card come through the api table.
 */

#include "module.h"
#include "spc7110rtc.h"

#define SECS_PER_DAY   (86400)
#define RTC_STATE_LEN  (1 + SPC7110_RTC_TIMELEN)  /* == FPGA_SPC7110_RTC_LEN (fpga_spi.h) */
#define RTC_SAVE_DIR   "/sd2snes/saves/"          /* == SAVE_BASEDIR (memory.h) */
#define RTC_VERSION    (1)

/* how far the two clocks may disagree before the sidecar is rewritten.  The cartridge
   clock and the console clock are different crystals, so their seconds drift apart and
   the difference jitters by one either way; without a dead band that alone would write
   the card every few seconds. */
#define RTC_SLACK      (2)

typedef struct {
  uint8_t magic[4];                     /* "S7RT" */
  uint8_t version;
  uint8_t reserved[3];
  uint8_t flags;                        /* spc7110_rtc_state_t, flattened so */
  uint8_t time[SPC7110_RTC_TIMELEN];    /* the layout is fixed by this file  */
  uint8_t wall[SPC7110_RTC_TIMELEN];    /* console clock when this was saved */
  uint8_t pad;
} rtc_file_t;

_Static_assert(sizeof(rtc_file_t) == 24, "spc7110 rtc sidecar layout");

typedef struct {
  const module_api_t *api;
  /* last state written out, so the game loop only touches the card on a change */
  spc7110_instant_t last_delta;
  uint8_t last_flags;
  uint8_t known;
  /* whether the configured core answered $e6 with the marker, i.e. whether there is a
     battery on the other end at all */
  uint8_t have_core;
  char path[256];                       /* the sidecar; "" when the name did not fit */
} rtcmod_t;

int  rtcmod_start(void *st, const module_api_t *api, const char *path);
void rtcmod_ctl(void *st, uint32_t param);
void rtcmod_idle(void *st);
void rtcmod_stop(void *st);

static const uint8_t rtc_magic[4] = { 'S', '7', 'R', 'T' };

static void copy7(uint8_t *d, const uint8_t *s) {
  int i;
  for (i = 0; i < SPC7110_RTC_TIMELEN; i++) d[i] = s[i];
}

/* The console clock as seven packed BCD bytes, most significant first.  get_bcdtime()
   packs the weekday in the top byte and century..second below it; taken apart in two
   32-bit halves because a variable 64-bit shift would need libgcc. */
static void wall_now(const module_api_t *api, uint8_t *b) {
  uint64_t t = api->bcdtime();
  uint32_t hi = (uint32_t)(t >> 32), lo = (uint32_t)t;
  b[0] = (uint8_t)(hi >> 16); b[1] = (uint8_t)(hi >> 8);  b[2] = (uint8_t)hi;
  b[3] = (uint8_t)(lo >> 24); b[4] = (uint8_t)(lo >> 16); b[5] = (uint8_t)(lo >> 8);
  b[6] = (uint8_t)lo;
}

/* the inverse, with the weekday, in the form set_fpga_time() takes */
static uint64_t bcd64(const uint8_t *b, uint8_t dow) {
  uint32_t hi = ((uint32_t)(dow & 7) << 24) | ((uint32_t)b[0] << 16)
              | ((uint32_t)b[1] << 8) | b[2];
  uint32_t lo = ((uint32_t)b[3] << 24) | ((uint32_t)b[4] << 16)
              | ((uint32_t)b[5] << 8) | b[6];
  return ((uint64_t)hi << 32) | lo;
}

/* The SPI helpers speak eight plain bytes; marshal rather than rely on how the struct
   happens to be laid out.  Returns -1 when what answered is not a core with the battery
   in it - see SPC7110_RTC_MAGIC. */
static int rtc_get(const module_api_t *api, spc7110_rtc_state_t *st) {
  uint8_t b[RTC_STATE_LEN];
  int i, same = 1;
  api->s7rtc_get(b);
  /* Eight identical bytes are a stale latch, not an answer: a core without the handover
     never touches its SPI read register and hands back the previous transfer eight times.
     The marker alone does not catch that (0xa5, left by the $f0 liveness poll, passes the
     mask), but a real answer can never be uniform -- the status byte carries the marker in
     its top two bits and no valid BCD month can. */
  for (i = 1; i < RTC_STATE_LEN; i++) if (b[i] != b[0]) same = 0;
  if (same) return -1;
  if ((b[0] & SPC7110_RTC_MAGIC_MASK) != SPC7110_RTC_MAGIC) return -1;
  st->flags = (uint8_t)(b[0] & ~SPC7110_RTC_MAGIC_MASK);
  copy7(st->time, b + 1);
  return 0;
}

/* `st` is the instant the cartridge has to report from now on; `prog` is what rtc.v is
   handed, which is one second less when the clock is running because programming it
   makes it run a pass straight away. */
static void rtc_program(const module_api_t *api, const spc7110_rtc_state_t *st,
                        const uint8_t *prog) {
  uint8_t b[RTC_STATE_LEN];
  api->fpga_time(bcd64(prog, st->flags & SPC7110_RTC_DOW_MASK));
  b[0] = st->flags;
  copy7(b + 1, st->time);
  api->s7rtc_set(b);
}

/* no usable backup: the console's own clock, as without the battery */
static void rtc_wallclock(const module_api_t *api) {
  api->fpga_time(api->bcdtime());
}

static int rtc_file_ok(const rtc_file_t *f, UINT got) {
  int i;
  /* a truncated file is a broken file: fall back rather than read the tail of whatever
     the buffer happened to hold */
  if (got != sizeof(*f) || f->version != RTC_VERSION) return 0;
  for (i = 0; i < 4; i++) if (f->magic[i] != rtc_magic[i]) return 0;
  return 1;
}

int rtcmod_start(void *vst, const module_api_t *api, const char *path) {
  rtcmod_t *m = vst;
  rtc_file_t f;
  spc7110_rtc_state_t st;
  spc7110_instant_t stored, saved, now, delta, cur, prog;
  uint8_t wall[SPC7110_RTC_TIMELEN];
  uint8_t progtime[SPC7110_RTC_TIMELEN];
  UINT got = 0;

  m->api = api;
  m->known = 0;
  m->path[0] = 0;
  /* Ask the core whether it has the handover before either direction is used: a board
     running an fpga_spc7110 from before the battery would otherwise have its stale latch
     written to the card as a calendar. */
  m->have_core = (rtc_get(api, &st) == 0);
  if (!m->have_core) { rtc_wallclock(api); return 0; }
  wall_now(api, wall);

  /* READ path: name only, never create the directory here */
  if (api->path_asset(m->path, sizeof(m->path), RTC_SAVE_DIR, path, ".rtc") < 0) {
    m->path[0] = 0;
    rtc_wallclock(api);
    return 1;
  }
  if (api->f_open(api->file, m->path, FA_READ) != FR_OK) { rtc_wallclock(api); return 1; }
  if (api->f_read(api->file, &f, sizeof(f), &got) != FR_OK) got = 0;
  api->f_close(api->file);
  if (!rtc_file_ok(&f, got)) { rtc_wallclock(api); return 1; }

  st.flags = f.flags;
  copy7(st.time, f.time);

  /* both are needed on either branch: `now` seeds the change detector even for a
     stopped clock, because that is what the save path will be comparing */
  if (spc7110_rtc_unpack(st.time, &stored) || spc7110_rtc_unpack(wall, &now)) {
    rtc_wallclock(api);
    return 1;
  }

  if (st.flags & SPC7110_RTC_STOPPED) {
    /* stopped clocks do not age.  The core serves this out of its freeze register, so
       the second it comes back on is the second it was left on. */
    cur = stored;
  } else {
    if (spc7110_rtc_unpack(f.wall, &saved)) { rtc_wallclock(api); return 1; }
    spc7110_rtc_diff(&now, &saved, &delta);
    /* console clock moved backwards (it was corrected, or never set): the cartridge
       clock cannot run backwards, so it simply does not advance */
    if (delta.days < 0) { delta.days = 0; delta.secs = 0; }
    spc7110_rtc_add(&stored, &delta, &cur);

    /* the weekday is a counter, not a function of the date: advance it by the number of
       midnights that went by, and by nothing else */
    st.flags = (uint8_t)((st.flags & ~SPC7110_RTC_DOW_MASK)
             | (uint8_t)(((uint32_t)(st.flags & SPC7110_RTC_DOW_MASK)
                          + (uint32_t)spc7110_rtc_midnights(&stored, &cur)) % 7));

    spc7110_rtc_pack(&cur, st.time);
  }

  /* rtc.v runs one per-second pass the moment it is programmed, so what it publishes is
     the value it was handed plus one second.  That second comes off here, and only here:
     `cur` stays the instant the cartridge shows.  A stopped clock is served out of the
     freeze register instead, so it needs no correction. */
  prog = cur;
  if (!(st.flags & SPC7110_RTC_STOPPED)) {
    prog.secs -= 1;
    if (prog.secs < 0) { prog.secs += SECS_PER_DAY; prog.days -= 1; }
  }
  spc7110_rtc_pack(&prog, progtime);
  rtc_program(api, &st, progtime);

  /* Seed the change detector with EXACTLY what the save path will compare against, so
     the first pass of the game loop finds nothing to do: the frozen instant while
     stopped, the offset between the two clocks while running. */
  m->last_flags = (uint8_t)(st.flags & ~SPC7110_RTC_DOW_MASK);
  if (st.flags & SPC7110_RTC_STOPPED) m->last_delta = cur;
  else                                spc7110_rtc_diff(&cur, &now, &m->last_delta);
  m->known = 1;
  return 1;
}

/* Once per game-loop pass: reads the core's state over SPI and writes the sidecar only
   when it actually changed, so a free running clock costs no card writes at all. */
void rtcmod_idle(void *vst) {
  rtcmod_t *m = vst;
  const module_api_t *api = m->api;
  rtc_file_t f;
  spc7110_rtc_state_t st;
  spc7110_instant_t cart, now, delta;
  uint8_t wall[SPC7110_RTC_TIMELEN];
  uint8_t flags_stable;
  UINT bw = 0;
  int i;

  if (!m->have_core || !m->path[0]) return;
  if (rtc_get(api, &st)) return;
  wall_now(api, wall);
  if (spc7110_rtc_unpack(st.time, &cart) || spc7110_rtc_unpack(wall, &now)) return;

  /* What has to stay the same for the backup to still describe reality: the control
     flags, and either the frozen time (stopped) or the offset between the two clocks
     (running).  A clock that simply runs keeps both constant, so the card is never
     touched. */
  flags_stable = (uint8_t)(st.flags & ~SPC7110_RTC_DOW_MASK);
  if (st.flags & SPC7110_RTC_STOPPED) delta = cart;
  else                                spc7110_rtc_diff(&cart, &now, &delta);

  if (m->known
   && flags_stable == m->last_flags
   && delta.days == m->last_delta.days
   && (delta.secs - m->last_delta.secs) <= RTC_SLACK
   && (m->last_delta.secs - delta.secs) <= RTC_SLACK)
    return;

  api->path_asset_mkdir(m->path);       /* only once the name exists */

  for (i = 0; i < 4; i++) f.magic[i] = rtc_magic[i];
  f.version = RTC_VERSION;
  f.reserved[0] = f.reserved[1] = f.reserved[2] = 0;
  f.flags = st.flags;
  copy7(f.time, st.time);
  copy7(f.wall, wall);
  f.pad = 0;

  if (api->f_open(api->file, m->path, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) return;
  api->led(1);
  api->f_write(api->file, &f, sizeof(f), &bw);
  api->led(0);
  api->f_close(api->file);

  m->last_flags = flags_stable;
  m->last_delta = delta;
  m->known = 1;
}

void rtcmod_ctl(void *st, uint32_t param) {
  (void)st; (void)param;
}

void rtcmod_stop(void *st) {
  (void)st;                             /* never the FPGA: the core may be another one */
}

MODULE_DEFINE(rtcmod, rtcmod_t)
