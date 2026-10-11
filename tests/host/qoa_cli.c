/* Conformance test for the streaming QOA decoder (src/qoa.c).
 *
 * The fixtures in qoa/ were encoded by the reference encoder (qoa.h, phoboslab/qoa)
 * from a deterministic integer test signal; the expected CRC32s are of the
 * reference decoder's output (mono duplicated to stereo, as the firmware plays it).
 * Every way of slicing the work -- one slice per call, odd request sizes, a whole
 * file at once -- has to give the same bytes, and broken input has to stop cleanly
 * without reading or writing out of bounds (ASan/UBSan own that part). */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "qoa.h"

typedef struct { const uint8_t *p; unsigned n, pos; } mem_t;

static unsigned mem_read(void *ctx, uint8_t *buf, unsigned len) {
  mem_t *m = ctx;
  unsigned k = m->n - m->pos < len ? m->n - m->pos : len;
  memcpy(buf, m->p + m->pos, k);
  m->pos += k;
  return k;
}

static uint32_t crc32_upd(uint32_t crc, const uint8_t *p, size_t n) {
  crc = ~crc;
  while(n--) {
    int k;
    crc ^= *p++;
    for(k = 0; k < 8; k++) crc = (crc >> 1) ^ (0xEDB88320u & -(crc & 1));
  }
  return ~crc;
}

static uint8_t *slurp(const char *path, unsigned *n) {
  FILE *f = fopen(path, "rb");
  uint8_t *b;
  long sz;
  if(!f) { perror(path); exit(2); }
  fseek(f, 0, SEEK_END); sz = ftell(f); fseek(f, 0, SEEK_SET);
  b = malloc((size_t)sz);
  if(fread(b, 1, (size_t)sz, f) != (size_t)sz) exit(2);
  fclose(f);
  *n = (unsigned)sz;
  return b;
}

/* Decode a whole buffer asking for `step` frames per call (step 0 = a varying pattern).
   Returns the frames produced, the CRC through *crc and the error flag through *err. */
static unsigned decode(const uint8_t *file, unsigned n, unsigned step, uint32_t rate,
                       uint32_t *crc, int *err) {
  static const unsigned pattern[] = { 20, 41, 5120, 39, 60, 1000, 20, 100000 };
  qoa_dec_t d;
  mem_t m = { file, n, QOA_HEADER_SIZE };
  unsigned total = 0, k = 0, cap = 100000;
  int16_t *out = malloc(cap * 4);
  *crc = 0;
  *err = 0;
  if(n < QOA_HEADER_SIZE || !qoa_dec_open(&d, file, rate)) { free(out); *err = 2; return 0; }
  for(;;) {
    unsigned want = step ? step : pattern[k++ % (sizeof pattern / sizeof pattern[0])];
    /* exact-size allocation per call: ASan catches a write past what was asked for */
    int16_t *o = malloc((size_t)want * 4);
    unsigned got = qoa_dec_run(&d, o, want, mem_read, &m);
    if(got > want) { printf("FAIL: wrote %u frames into %u\n", got, want); exit(1); }
    *crc = crc32_upd(*crc, (const uint8_t *)o, (size_t)got * 4);
    free(o);
    total += got;
    if(!got) break;
  }
  free(out);
  *err = d.err;
  return total;
}

static int fails = 0;

static void check_file(const char *path, uint32_t want_crc, unsigned want_frames) {
  static const unsigned steps[] = { 0, 20, 21, 40, 5120, 5121, 100000 };
  unsigned n, i;
  uint8_t *f = slurp(path, &n);
  for(i = 0; i < sizeof steps / sizeof steps[0]; i++) {
    uint32_t crc;
    int err;
    unsigned got = decode(f, n, steps[i], 44100, &crc, &err);
    if(got != want_frames || crc != want_crc || err) {
      printf("FAIL %s step %u: %u frames crc %08x err %d (want %u / %08x)\n",
             path, steps[i], got, crc, err, want_frames, want_crc);
      fails++;
    }
  }
  /* a samplerate the DAC does not play is refused at the first frame header */
  {
    uint32_t crc;
    int err;
    unsigned got = decode(f, n, 0, 48000, &crc, &err);
    if(got || !err) { printf("FAIL %s: 48 kHz request accepted\n", path); fails++; }
  }
  /* every truncation stops early, flagged, never past the data */
  for(i = 0; i < n; i += 37) {
    uint32_t crc;
    int err;
    uint8_t *t = malloc(i ? i : 1);
    unsigned got;
    memcpy(t, f, i);
    got = decode(t, i, 0, 44100, &crc, &err);
    if(got >= want_frames || !err) { printf("FAIL %s cut at %u: %u frames err %d\n", path, i, got, err); fails++; }
    free(t);
  }
  /* corrupted bytes anywhere: bounded, no sanitizer report, never more frames than the header */
  {
    uint32_t seed = 1;
    uint8_t *t = malloc(n);
    for(i = 0; i < 3000; i++) {
      uint32_t crc;
      int err, j;
      unsigned got;
      memcpy(t, f, n);
      for(j = 0; j < 4; j++) {
        seed = seed * 1103515245u + 12345u;
        t[(seed >> 8) % n] ^= (uint8_t)(1u << ((seed >> 4) & 7));
      }
      got = decode(t, n, 0, 44100, &crc, &err);
      if(got > want_frames && err == 0 && !memcmp(t, f, 8)) {
        printf("FAIL %s fuzz %u: %u frames\n", path, i, got); fails++;
      }
    }
    free(t);
  }
  free(f);
}

int main(void) {
  static const uint8_t bad_magic[8] = { 'q', 'o', 'a', 'x', 0, 0, 0, 1 };
  static const uint8_t no_len[8]    = { 'q', 'o', 'a', 'f', 0, 0, 0, 0 };
  qoa_dec_t d;
  check_file("qoa/v1_stereo.qoa", 0xaf6737f1u, 16597);
  check_file("qoa/v2_mono.qoa",   0xea9f03e0u, 11000);
  if(qoa_dec_open(&d, bad_magic, 44100)) { printf("FAIL: bad magic accepted\n"); fails++; }
  if(qoa_dec_open(&d, no_len, 44100))    { printf("FAIL: streaming (length 0) accepted\n"); fails++; }
  if(fails) { printf("qoa: %d failure(s)\n", fails); return 1; }
  printf("qoa: OK\n");
  return 0;
}
