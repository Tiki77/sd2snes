/* sd2snes - streaming QOA decoder.  See qoa.h.
 *
 * Follows the reference decoder (qoa.h, phoboslab/qoa, MIT) step for step; the
 * only change is the shape: it pulls one slice at a time through a read callback
 * instead of decoding a whole frame from memory, so it needs a few dozen bytes of
 * RAM.  The arithmetic is done in uint32_t and cast back, which is what the
 * reference's int arithmetic compiles to, without the overflow being undefined. */

#include "qoa.h"

/* dequant_tab[s][q] = round_ties_away_from_zero(scalefactor_tab[s] * {0.75, -0.75,
 * 2.5, -2.5, 4.5, -4.5, 7, -7}[q]), scalefactor_tab[s] = round(pow(s + 1, 2.75)). */
static const int16_t qoa_dequant_tab[16][8] = {
  {   1,    -1,    3,    -3,    5,    -5,     7,     -7},
  {   5,    -5,   18,   -18,   32,   -32,    49,    -49},
  {  16,   -16,   53,   -53,   95,   -95,   147,   -147},
  {  34,   -34,  113,  -113,  203,  -203,   315,   -315},
  {  63,   -63,  210,  -210,  378,  -378,   588,   -588},
  { 104,  -104,  345,  -345,  621,  -621,   966,   -966},
  { 158,  -158,  528,  -528,  950,  -950,  1477,  -1477},
  { 228,  -228,  760,  -760, 1368, -1368,  2128,  -2128},
  { 316,  -316, 1053, -1053, 1895, -1895,  2947,  -2947},
  { 422,  -422, 1405, -1405, 2529, -2529,  3934,  -3934},
  { 548,  -548, 1828, -1828, 3290, -3290,  5117,  -5117},
  { 696,  -696, 2320, -2320, 4176, -4176,  6496,  -6496},
  { 868,  -868, 2893, -2893, 5207, -5207,  8099,  -8099},
  {1064, -1064, 3548, -3548, 6386, -6386,  9933,  -9933},
  {1286, -1286, 4288, -4288, 7718, -7718, 12005, -12005},
  {1536, -1536, 5120, -5120, 9216, -9216, 14336, -14336},
};

static uint32_t be32(const uint8_t *b) {
  return (uint32_t)b[0] << 24 | (uint32_t)b[1] << 16 | (uint32_t)b[2] << 8 | b[3];
}

uint32_t qoa_dec_open(qoa_dec_t *d, const uint8_t hdr[QOA_HEADER_SIZE], uint32_t rate) {
  d->left = 0;
  d->frame_left = 0;
  d->channels = 0;
  d->err = 1;
  d->rate = rate;
  if(hdr[0] != 'q' || hdr[1] != 'o' || hdr[2] != 'a' || hdr[3] != 'f') return 0;
  d->left = be32(hdr + 4);
  if(!d->left) return 0;           /* 0 = the streaming variant, length unknown */
  d->err = 0;
  return d->left;
}

/* Frame header (channels, rate, samples, size) + the LMS state of each channel. */
static int qoa_frame_begin(qoa_dec_t *d, qoa_read_fn rd, void *ctx) {
  uint8_t b[16];
  unsigned ch, fsamples, fsize, slices, c, i;
  if(rd(ctx, b, 8) != 8) return 0;
  ch       = b[0];
  fsamples = (unsigned)b[4] << 8 | b[5];
  fsize    = (unsigned)b[6] << 8 | b[7];
  slices   = (fsamples + QOA_SLICE_LEN - 1) / QOA_SLICE_LEN;
  if(ch < 1 || ch > 2 || (d->channels && ch != d->channels)) return 0;
  if(((uint32_t)b[1] << 16 | (uint32_t)b[2] << 8 | b[3]) != d->rate) return 0;
  /* Every frame is full but the last, which holds exactly what is left; anything else
     means a broken file, and stopping there beats playing it misaligned. */
  if(!fsamples || fsamples > QOA_FRAME_LEN || fsamples > d->left
     || (fsamples < QOA_FRAME_LEN && fsamples != d->left)) return 0;
  if(fsize < 8 + 16 * ch + slices * 8 * ch) return 0;
  d->channels = (uint8_t)ch;
  for(c = 0; c < ch; c++) {
    if(rd(ctx, b, 16) != 16) return 0;
    for(i = 0; i < 4; i++) {
      d->lms[c].history[i] = (int16_t)((uint16_t)b[2 * i] << 8 | b[2 * i + 1]);
      d->lms[c].weights[i] = (int16_t)((uint16_t)b[8 + 2 * i] << 8 | b[8 + 2 * i + 1]);
    }
  }
  d->frame_left = (uint16_t)fsamples;
  return 1;
}

/* One slice of one channel: n samples written every other int16_t of o. */
static void qoa_slice(qoa_lms_t *l, const uint8_t *b, int16_t *o, unsigned n) {
  uint64_t s = (uint64_t)be32(b) << 32 | be32(b + 4);
  const int16_t *dq = qoa_dequant_tab[s >> 60];
  s <<= 4;
  while(n--) {
    int32_t p = (int32_t)((uint32_t)l->weights[0] * (uint32_t)l->history[0]
                        + (uint32_t)l->weights[1] * (uint32_t)l->history[1]
                        + (uint32_t)l->weights[2] * (uint32_t)l->history[2]
                        + (uint32_t)l->weights[3] * (uint32_t)l->history[3]) >> 13;
    int32_t r = dq[s >> 61];
    int32_t v = (int32_t)((uint32_t)p + (uint32_t)r);
    int32_t delta = r >> 4;
    int i;
    if(v < -32768) v = -32768; else if(v > 32767) v = 32767;
    *o = (int16_t)v;
    o += 2;
    s <<= 3;
    for(i = 0; i < 4; i++)
      l->weights[i] = (int32_t)((uint32_t)l->weights[i]
                                + (uint32_t)(l->history[i] < 0 ? -delta : delta));
    l->history[0] = l->history[1];
    l->history[1] = l->history[2];
    l->history[2] = l->history[3];
    l->history[3] = v;
  }
}

unsigned qoa_dec_run(qoa_dec_t *d, int16_t *out, unsigned frames, qoa_read_fn rd, void *ctx) {
  unsigned done = 0;
  uint8_t b[16];
  while(!d->err && d->left && frames - done >= (d->left < QOA_SLICE_LEN ? d->left : QOA_SLICE_LEN)) {
    unsigned n, c;
    if(!d->frame_left && !qoa_frame_begin(d, rd, ctx)) { d->err = 1; break; }
    n = d->frame_left < QOA_SLICE_LEN ? d->frame_left : QOA_SLICE_LEN;
    if(rd(ctx, b, 8u * d->channels) != 8u * d->channels) { d->err = 1; break; }
    for(c = 0; c < d->channels; c++)
      qoa_slice(&d->lms[c], b + 8 * c, out + 2 * done + c, n);
    if(d->channels == 1)
      for(c = 0; c < n; c++) out[2 * (done + c) + 1] = out[2 * (done + c)];
    d->frame_left -= n;
    d->left -= n;
    done += n;
  }
  return done;
}
