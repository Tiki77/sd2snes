#ifndef QOA_H
#define QOA_H

/* Streaming QOA ("Quite OK Audio", https://qoaformat.org) decoder for the game
 * info screen's clip soundtrack (msu1.c menu_music_*).  Bit-exact with the
 * reference qoa.h (phoboslab/qoa, MIT) decoder; tests/host/run_qoa.sh checks it
 * against files encoded and decoded by the reference.
 *
 * Free of firmware dependencies (no config.h, no ff.h) so the host test compiles
 * it straight from src/.  Keep it that way. */

#include <stdint.h>

#define QOA_SLICE_LEN   20      /* samples per channel in one 8-byte slice */
#define QOA_FRAME_LEN   5120    /* samples per channel in a full frame (256 slices) */
#define QOA_HEADER_SIZE 8       /* "qoaf" + u32 BE samples per channel */

/* Fills buf with exactly len bytes; returns how many it got (short = EOF/error). */
typedef unsigned (*qoa_read_fn)(void *ctx, uint8_t *buf, unsigned len);

typedef struct {
  int32_t history[4];
  int32_t weights[4];
} qoa_lms_t;

typedef struct {
  uint32_t  left;         /* samples per channel still to decode in the file */
  uint16_t  frame_left;   /* ... and in the current frame (0 = next is a frame header) */
  uint8_t   channels;     /* 0 until the first frame header, then 1 or 2 */
  uint8_t   err;          /* nonzero: the stream is broken, nothing more will decode */
  uint32_t  rate;         /* the only samplerate accepted */
  qoa_lms_t lms[2];
} qoa_dec_t;

/* Validates the 8-byte file header; returns the samples per channel, 0 if it is
 * not a usable QOA file.  Channels and rate are checked per frame by qoa_dec_run. */
uint32_t qoa_dec_open(qoa_dec_t *d, const uint8_t hdr[QOA_HEADER_SIZE], uint32_t rate);

/* Decodes up to `frames` sample frames (whole slices, so a multiple of
 * QOA_SLICE_LEN is used up to the last one) as interleaved STEREO s16 into out
 * (a mono file is duplicated to both channels).  Returns the frames written;
 * fewer than asked means the end of the file or an error (d->err). */
unsigned qoa_dec_run(qoa_dec_t *d, int16_t *out, unsigned frames, qoa_read_fn rd, void *ctx);

#endif
