/* Menu PCM player (the card module pcmplay.bin) -- see src/pcmplay.h for what this is and
   why the block looks the way it does.  Everything audio-related is done by the firmware's
   menu music engine (msu1.c, through the module api); this module only tracks which track
   is up, whether it is paused, and publishes the status block the menu screen
   (snes/pcmplay.a65) renders.  The menu still sends SNES_CMD_PLAY_PCM / SNES_CMD_PCM_CTL;
   the firmware hands them to the module.  No globals: the state is the pcmmod_t below. */

#include "module.h"
#include "memmap.h"
#include "pcmplay.h"

/* 44100 stereo 16-bit frames = 176400 bytes per second.  Kept as one constant so the
   two places that convert bytes<->seconds cannot drift apart. */
#define PCM_BYTES_PER_SEC ((uint32_t)44100 * 4)
#define PCM_HDR_BYTES     8          /* "MSU1" + the loop point, before the samples */

typedef struct {
  const module_api_t *api;
  uint8_t  state;
  uint8_t  last_valid;
  uint16_t total_sec;
  uint32_t body_bytes;              /* track length without the 8-byte header */
  pcmplay_blk_t last;               /* last block published (see pcm_write_blk) */
} pcmmod_t;

int  pcmmod_start(void *st, const module_api_t *api, const char *path);
void pcmmod_ctl(void *st, uint32_t param);
void pcmmod_idle(void *st);
void pcmmod_stop(void *st);

/* The players' Repeat (CFG.player_repeat, the menu's X key): read from the live CFG byte, so
   a toggle while the track plays counts at once.  On, the engine loops the track at the
   header's loop point -- what an MSU-1 track is authored for, gapless, the way the game plays
   it -- instead of stopping at EOF for the menu to move on.  One byte of SPI per pass. */
static uint8_t pcm_repeat(pcmmod_t *m) {
  uint8_t r = 0;
  m->api->sram_read(&r, SRAM_CFG_PLAYER_REPEAT_ADDR, 1);
  m->api->music_noloop(!r);
  return r;
}

static void pcm_write_blk(pcmmod_t *m) {
  const module_api_t *api = m->api;
  pcmplay_blk_t blk;
  uint8_t *b = (uint8_t *)&blk, *l = (uint8_t *)&m->last;
  uint32_t pos = 0;
  unsigned i;

  for(i = 0; i < sizeof(blk); i++) b[i] = 0;
  blk.magic[0] = PCMPLAY_MAGIC0;
  blk.magic[1] = PCMPLAY_MAGIC1;
  blk.version  = PCMPLAY_VERSION;
  blk.state    = m->state;

  if(m->state == PCMPLAY_STATE_PLAYING || m->state == PCMPLAY_STATE_PAUSED
     || m->state == PCMPLAY_STATE_ENDED) {
    blk.total_sec = m->total_sec;
    pos = api->music_tell();
    pos = (pos > PCM_HDR_BYTES) ? (pos - PCM_HDR_BYTES) : 0;
    if(pos > m->body_bytes) pos = m->body_bytes;   /* the read head runs one DAC buffer ahead */
    blk.pos_sec = (uint16_t)(pos / PCM_BYTES_PER_SEC);
    /* 0..255 fraction, pre-divided here because the 65816 has no divide.  num * 255 must
       not wrap 32 bits, so scale BOTH sides down until the product fits: a real track is
       tens of MB, far past the 16.8 MB where the bare product overflows. */
    if(m->body_bytes) {
      uint32_t num = pos, den = m->body_bytes;
      while(den > (uint32_t)0x00ffffff) { den >>= 8; num >>= 8; }
      blk.progress = (uint8_t)((num * (uint32_t)255) / den);
    }
  }

  /* Every write here is FPGA SPI traffic interleaved with the DAC pump, which has to land
     a 1 KB refill every ~5.8 ms.  Writing only on a real change drops a burst per menu-loop
     pass (~50 Hz) to ~1 Hz. */
  if(m->last_valid) {
    for(i = 0; i < sizeof(blk) && b[i] == l[i]; i++);
    if(i == sizeof(blk)) return;
  }
  for(i = 0; i < sizeof(blk); i++) l[i] = b[i];
  m->last_valid = 1;
  api->sram_write(&blk, SRAM_PCMPLAY_ADDR, sizeof(blk));
}

/* Open and start a .pcm.  Fail-safe: a missing/!"MSU1" file publishes an error state and
   plays nothing -- it never hangs and never leaves the DAC half-armed. */
int pcmmod_start(void *st, const module_api_t *api, const char *path) {
  pcmmod_t *m = st;
  int res = api->music_play(path);   /* 0xA0 playing / 0x01 open-fail / 0x02 bad magic */

  m->api = api;
  m->total_sec  = 0;
  m->body_bytes = 0;
  if(res == 0xA0) {
    uint32_t size = api->music_size();
    api->music_lock(1);        /* the FMV stop paths must leave our track alone */
    pcm_repeat(m);             /* play it once (the menu then moves to the next track), or loop it */
    m->body_bytes = (size > PCM_HDR_BYTES) ? (size - PCM_HDR_BYTES) : 0;
    m->total_sec  = (uint16_t)(m->body_bytes / PCM_BYTES_PER_SEC);
    m->state      = PCMPLAY_STATE_PLAYING;
  } else {
    /* music_play already closed the handle and left the DAC alone on both error paths */
    m->state = (res == 0x02) ? PCMPLAY_STATE_ERR_MAGIC : PCMPLAY_STATE_ERR_OPEN;
  }
  /* ALWAYS write: the menu zeroed the magic before this command (its capability sentinel),
     and a track of the same length as the last one would publish an identical block. */
  m->last_valid = 0;
  pcm_write_blk(m);
  return 0;
}

static void pcm_seek(pcmmod_t *m, uint32_t pos) {
  const module_api_t *api = m->api;
  if(m->state != PCMPLAY_STATE_PLAYING && m->state != PCMPLAY_STATE_PAUSED
     && m->state != PCMPLAY_STATE_ENDED) return;
  if(api->music_seek(pos)) {
    if(m->state == PCMPLAY_STATE_ENDED) m->state = PCMPLAY_STATE_PLAYING;
  } else if(!api->music_active() && !api->music_ended()) {
    /* the refill read failed and stopped the clip: say so, as pcmmod_idle would */
    api->music_lock(0);
    api->music_stop();
    m->state = PCMPLAY_STATE_ERR_OPEN;
  }
  pcm_write_blk(m);
}

void pcmmod_ctl(void *st, uint32_t param) {
  pcmmod_t *m = st;
  switch(param & 0xff) {
    case PCMPLAY_CTL_PAUSE:
    case PCMPLAY_CTL_RESUME:
      /* Freeze/unfreeze the DAC read pointer only: the file stays open at its position,
         which is what makes resuming instant. */
      if(m->state != PCMPLAY_STATE_PLAYING && m->state != PCMPLAY_STATE_PAUSED) return;
      m->api->music_pause((param & 0xff) == PCMPLAY_CTL_PAUSE);
      m->state = ((param & 0xff) == PCMPLAY_CTL_PAUSE) ? PCMPLAY_STATE_PAUSED : PCMPLAY_STATE_PLAYING;
      pcm_write_blk(m);
      return;
    case PCMPLAY_CTL_SEEK: {
      /* clamp BEFORE multiplying: 65535 s * 176400 B/s does not fit 32 bits */
      uint16_t sec = (param >> 8) & 0xffff;
      if(sec > m->total_sec) sec = m->total_sec;
      pcm_seek(m, (uint32_t)sec * PCM_BYTES_PER_SEC);
      return;
    }
    case PCMPLAY_CTL_SEEKFRAC:
      /* body >> 8 first for the same reason; the 255 bytes it drops are a fraction of a ms */
      pcm_seek(m, (m->body_bytes >> 8) * ((param >> 8) & 0xff));
      return;
    default:
      pcmmod_stop(st);
      return;
  }
}

void pcmmod_idle(void *st) {
  pcmmod_t *m = st;
  if(m->state == PCMPLAY_STATE_NONE) return;
  if((m->state == PCMPLAY_STATE_PLAYING || m->state == PCMPLAY_STATE_PAUSED)
     && pcm_repeat(m) && m->api->music_ended()) {
    pcm_seek(m, 0);                      /* Repeat went on after the EOF read: from the top */
    return;
  }
  /* The clip dying under us -- an unreadable file makes the pump give up -- must not
     leave the screen showing a track that is no longer playing.  Reported as the READ
     error it is, rather than as plain "nothing playing". */
  if(m->state == PCMPLAY_STATE_PLAYING && m->api->music_ended()) {
    m->state = PCMPLAY_STATE_ENDED;      /* played out: the menu decides what comes next */
  } else if(m->state == PCMPLAY_STATE_PLAYING && !m->api->music_active()) {
    m->api->music_lock(0);
    m->api->music_stop();                /* already stopped; this only drops the handle */
    m->state = PCMPLAY_STATE_ERR_OPEN;
  }
  pcm_write_blk(m);
}

void pcmmod_stop(void *st) {
  pcmmod_t *m = st;
  m->api->music_lock(0);
  m->api->music_stop();
  m->state      = PCMPLAY_STATE_NONE;
  m->total_sec  = 0;
  m->body_bytes = 0;
  pcm_write_blk(m);
}

MODULE_DEFINE(pcmmod, pcmmod_t)
