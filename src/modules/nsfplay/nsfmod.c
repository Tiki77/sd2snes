/* The NSF player module (see nsfmod.h and ../module.h): loads the tune into the backing
   store, hands the core to the menu music engine as a generator and publishes the status
   block the player screen (snes/nsfplay.a65) reads.  Built into nsfplay.bin with the core;
   no globals -- everything lives in the nsfmod_t the firmware gives it (the scratch FRAME). */

#include "memmap.h"
#include "scratch.h"
#include "nsfmod.h"

#define NSF_BYTES_PER_SEC ((uint32_t)44100 * 4)

_Static_assert(sizeof(nsfmod_t) <= SCRATCH_FRAME_BYTES, "the NSF module's state must fit the scratch FRAME");

#ifndef MODULE_HOST_LINKED
/* Cortex-M3/M4 cycle counter: fixed addresses, so no relocation in the image */
#define DEMCR    (*(volatile uint32_t *)0xe000edfcUL)
#define DWT_CTRL (*(volatile uint32_t *)0xe0001000UL)
#define DWT_CYC  (*(volatile uint32_t *)0xe0001004UL)
#endif

static void nsf_bk(void *ctx, uint32_t off, uint8_t *buf, uint16_t len, uint8_t write)
{
  const module_api_t *api = ((nsfmod_t *)ctx)->api;
  if(write) api->sram_write(buf, SRAM_NSF_BK_ADDR + off, len);
  else api->sram_read(buf, SRAM_NSF_BK_ADDR + off, len);
}

static unsigned nsf_gen(void *ctx, int16_t *pcm, unsigned frames)
{
  nsfmod_t *m = ctx;
  if(m->n.err) {
    for(unsigned i = 0; i < frames * 2; i++) pcm[i] = 0;
    return frames;
  }
#ifdef MODULE_HOST_LINKED
  nsf_render(&m->n, pcm, frames);
#else
  uint32_t t0 = DWT_CYC;
  nsf_render(&m->n, pcm, frames);
  m->cyc += DWT_CYC - t0;
  m->frames += frames;
#endif
  /* The end of the song (nsfmod.h): every 4th frame is plenty to tell sound from silence,
     and costs a quarter of the time on a CPU that renders with little to spare.  L = R. */
  uint32_t q = m->quiet;
  for(unsigned i = 0; i < frames; i += 4) {
    int16_t s = pcm[2 * i];
    if(s > NSFPLAY_QUIET_LEVEL || s < -NSFPLAY_QUIET_LEVEL) {
      q = 0;
      m->end_at = 0;                          /* sound again: that was a pause, not the end */
    } else if((q += 4) == (uint32_t)44100 / 4 * 4 * NSFPLAY_QUIET_SEC) {
      m->end_at = m->gen + (i + 4) * 4;
    }
  }
  m->quiet = q;
  m->gen += frames * 4;
  return frames;
}

/* Header + tune into the backing store; the three strings into the block.
   Returns an NSFPLAY_STATE_* (PLAYING = loaded). */
static uint8_t nsf_load(nsfmod_t *m, const char *path)
{
  const module_api_t *api = m->api;
  FIL *f = api->file;
  nsf_info_t *info = &m->n.info;
  uint8_t hdr[NSF_HDR_SIZE];
  char s[NSFPLAY_STR_LEN * 3];
  UINT br = 0;
  FRESULT res;
  uint32_t size, at;
  unsigned i;
  if(api->f_open(f, (const TCHAR *)path, FA_READ) != FR_OK) return NSFPLAY_STATE_ERR_OPEN;
  size = f_size(f);
  res = api->f_read(f, hdr, sizeof(hdr), &br);
  if(res != FR_OK || br != sizeof(hdr) || !nsf_parse(info, hdr, size)) {
    api->f_close(f);
    return res != FR_OK ? NSFPLAY_STATE_ERR_OPEN : NSFPLAY_STATE_ERR_NSF;
  }
  at = SRAM_NSF_BK_ADDR + NSF_BK_ROM;
  api->sram_memset(at, info->pad, 0);
  at += info->pad;
  if(!api->psram_stream(f, at, info->data_len, 0)) {
    api->f_close(f);
    return NSFPLAY_STATE_ERR_OPEN;
  }
  api->f_close(f);
  api->sram_memset(at + info->data_len, nsf_rom_size(info) - info->pad - info->data_len, 0);
  m->flags = (info->pal ? NSFPLAY_FLAG_PAL : 0) | (info->chip ? NSFPLAY_FLAG_EXPANSION : 0);
  for(i = 0; i < sizeof(s); i++) {        /* made printable, each string cut at 31 */
    char c = hdr[0x0e + i];
    s[i] = ((i % NSFPLAY_STR_LEN) == NSFPLAY_STR_LEN - 1) ? 0 : (c && (c < 0x20 || c > 0x7e)) ? '?' : c;
  }
  api->sram_write(s, SRAM_NSFPLAY_ADDR + 16, sizeof(s));
  return NSFPLAY_STATE_PLAYING;
}

static void nsf_publish(nsfmod_t *m, int force)
{
  uint8_t b[16];
  uint16_t pos = 0;
  unsigned i;
  if(m->state == NSFPLAY_STATE_PLAYING || m->state == NSFPLAY_STATE_PAUSED)
    pos = (uint16_t)(m->api->music_played() / NSF_BYTES_PER_SEC);   /* restarts with every song */
  if(!force && pos == m->last_pos) return;
  m->last_pos = pos;
  /* cycles per frame against the cycles a frame of real time has: the ring is filled ahead
     of the DAC, so wall time would only say how idle the menu loop was */
  if(m->frames) {
    m->load = (uint8_t)(m->cyc / m->frames * 100 / (m->api->cpu_hz / 44100));
    m->cyc = m->frames = 0;
  }
  for(i = 0; i < sizeof(b); i++) b[i] = 0;
  b[0] = NSFPLAY_MAGIC0;
  b[1] = NSFPLAY_MAGIC1;
  b[2] = NSFPLAY_VERSION;
  b[3] = m->state;
  b[4] = m->song;
  b[5] = m->n.info.songs;
  b[6] = pos;
  b[7] = pos >> 8;
  b[8] = m->flags;
  b[9] = m->load;
  m->api->sram_write(b, SRAM_NSFPLAY_ADDR, sizeof(b));
}

static void nsf_play(nsfmod_t *m, uint8_t song)
{
  if(!song || song > m->n.info.songs) song = 1;
  m->song = song;
  m->gen = m->quiet = m->end_at = 0;
  m->flags &= ~NSFPLAY_FLAG_END;
  nsf_start(&m->n, song);
  m->api->music_play_gen(nsf_gen, m, SRAM_NSF_BK_ADDR - SRAM_ROM_ADDR);
  m->api->music_lock(1);              /* the FMV stop paths leave the DAC to us */
  m->state = NSFPLAY_STATE_PLAYING;
}

int nsfmod_start(void *st, const module_api_t *api, const char *path)
{
  nsfmod_t *m = st;
  uint32_t spare;
  uint8_t *mem = api->spare(&spare);
  uint8_t *b = st;
  for(unsigned i = 0; i < sizeof(*m); i++) b[i] = 0;
  m->api = api;
  m->last_pos = 0xffff;
#ifndef MODULE_HOST_LINKED
  DEMCR |= 1UL << 24;                 /* TRCENA: the render-cost meter (nsf_gen) */
  DWT_CTRL |= 1;
#endif
  api->music_stop();
  api->sram_memset(SRAM_NSFPLAY_ADDR + 16, NSFPLAY_STR_LEN * 3, 0);
  m->state = nsf_load(m, path);
  if(m->state == NSFPLAY_STATE_PLAYING && !nsf_open(&m->n, &m->n.info, nsf_bk, m, mem, spare))
    m->state = NSFPLAY_STATE_ERR_CORE;   /* too little spare RAM for the cache */
  if(m->state == NSFPLAY_STATE_PLAYING) nsf_play(m, m->n.info.start);
  nsf_publish(m, 1);
  return 0;
}

void nsfmod_ctl(void *st, uint32_t param)
{
  nsfmod_t *m = st;
  uint8_t ctl = param, arg = param >> 8;
  switch(ctl) {
    case NSFPLAY_CTL_PAUSE:
    case NSFPLAY_CTL_RESUME:
      if(m->state != NSFPLAY_STATE_PLAYING && m->state != NSFPLAY_STATE_PAUSED) return;
      m->api->music_pause(ctl == NSFPLAY_CTL_PAUSE);
      m->state = (ctl == NSFPLAY_CTL_PAUSE) ? NSFPLAY_STATE_PAUSED : NSFPLAY_STATE_PLAYING;
      break;
    case NSFPLAY_CTL_SONG:
      if(m->state == NSFPLAY_STATE_PLAYING || m->state == NSFPLAY_STATE_PAUSED
         || m->state == NSFPLAY_STATE_ERR_TUNE)
        nsf_play(m, arg);
      break;
    default:                          /* STOP: the firmware unloads us on the next command */
      m->api->music_lock(0);
      m->api->music_stop();
      m->state = NSFPLAY_STATE_NONE;
      break;
  }
  nsf_publish(m, 1);
}

void nsfmod_idle(void *st)
{
  nsfmod_t *m = st;
  if(m->state == NSFPLAY_STATE_PLAYING && m->n.err) {
    m->api->music_lock(0);
    m->api->music_stop();
    m->state = NSFPLAY_STATE_ERR_TUNE;
    nsf_publish(m, 1);
    return;
  }
  /* The song is over (see NSFPLAY_QUIET_SEC) once the DAC played up to there.  Checked on
     every pass, so Repeat turned on at the file's end still starts the last song again. */
  int force = 0;
  if(m->state == NSFPLAY_STATE_PLAYING && m->end_at && m->api->music_played() >= m->end_at) {
    uint8_t repeat = 0;
    unsigned song = m->song;
    m->api->sram_read(&repeat, SRAM_CFG_PLAYER_REPEAT_ADDR, 1);
    if(!repeat) song++;
    force = !(m->flags & NSFPLAY_FLAG_END);
    if(song <= m->n.info.songs) { nsf_play(m, song); force = 1; }
    else m->flags |= NSFPLAY_FLAG_END;
  }
  nsf_publish(m, force);
}

void nsfmod_stop(void *st)
{
  nsfmod_ctl(st, NSFPLAY_CTL_STOP);
}

MODULE_DEFINE(nsfmod, nsfmod_t)
