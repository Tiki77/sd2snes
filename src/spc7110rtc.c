/* SPC7110 / RTC-4513 battery backup: the calendar arithmetic.  See spc7110rtc.h for what
   this is for.  Pure code, no hardware and no files: the card module rtc7110.bin
   (modules/rtc7110/) links it, and so does the host self test. */

#include <stdint.h>

#include "spc7110rtc.h"

#define SECS_PER_DAY (86400)

/* ------------------------------------------------------------------------ */
/* calendar                                                                  */
/*                                                                           */
/* Days are counted from an epoch of 0000-03-01 with the year starting in     */
/* March, so February - the only month whose length moves - is always the     */
/* last one and no leap year special case is needed anywhere else.  Both      */
/* directions are exact for every year the RTC-4513 can express (0000-9999),  */
/* including 2000 and 2400 (leap) against 1900 and 2100 (not).                */
/* ------------------------------------------------------------------------ */

/* Gregorian day-count conversion below follows Howard Hinnant's public-domain
   "chrono-Compatible Low-Level Date Algorithms" (era/yoe/doy/doe form). */
static int32_t days_from_civil(int32_t y, int32_t m, int32_t d) {
  int32_t era;
  uint32_t yoe, doy, doe;

  y -= (m <= 2);
  era = (y >= 0 ? y : y - 399) / 400;
  yoe = (uint32_t)(y - era * 400);                                /* 0..399 */
  doy = (153u * (uint32_t)(m + (m > 2 ? -3 : 9)) + 2u) / 5u + (uint32_t)d - 1u;
  doe = yoe * 365u + yoe / 4u - yoe / 100u + doy;                 /* 0..146096 */
  return era * 146097 + (int32_t)doe;
}

static void civil_from_days(int32_t z, int32_t *py, int32_t *pm, int32_t *pd) {
  int32_t era, y;
  uint32_t doe, yoe, doy, mp, d, m;

  era = (z >= 0 ? z : z - 146096) / 146097;
  doe = (uint32_t)(z - era * 146097);                             /* 0..146096 */
  yoe = (doe - doe / 1460u + doe / 36524u - doe / 146096u) / 365u;
  y   = (int32_t)yoe + era * 400;
  doy = doe - (365u * yoe + yoe / 4u - yoe / 100u);
  mp  = (5u * doy + 2u) / 153u;
  d   = doy - (153u * mp + 2u) / 5u + 1u;
  m   = mp + (mp < 10u ? 3u : (uint32_t)-9);
  *py = y + (m <= 2);
  *pm = (int32_t)m;
  *pd = (int32_t)d;
}

static int bcd2bin(uint8_t b, int32_t *v) {
  if ((b & 0x0f) > 9 || (b >> 4) > 9) return -1;
  *v = (int32_t)(b >> 4) * 10 + (int32_t)(b & 0x0f);
  return 0;
}

static uint8_t bin2bcd(int32_t v) {
  return (uint8_t)((((v / 10) % 10) << 4) | (v % 10));
}

int spc7110_rtc_unpack(const uint8_t *time, spc7110_instant_t *out) {
  int32_t cent, year, mon, day, hour, min, sec;

  if (bcd2bin(time[0], &cent) || bcd2bin(time[1], &year)
   || bcd2bin(time[2], &mon)  || bcd2bin(time[3], &day)
   || bcd2bin(time[4], &hour) || bcd2bin(time[5], &min)
   || bcd2bin(time[6], &sec)) return -1;
  /* the day is only range checked, not checked against the month: a corrupt
     sidecar normalises (30 February becomes 2 March) instead of being thrown
     away, which is the friendlier failure for a clock */
  if (mon < 1 || mon > 12 || day < 1 || day > 31
   || hour > 23 || min > 59 || sec > 59) return -1;

  out->days = days_from_civil(cent * 100 + year, mon, day);
  out->secs = hour * 3600 + min * 60 + sec;
  return 0;
}

void spc7110_rtc_pack(const spc7110_instant_t *in, uint8_t *time) {
  int32_t y, m, d, s = in->secs;

  civil_from_days(in->days, &y, &m, &d);
  y %= 10000;
  if (y < 0) y += 10000;
  time[0] = bin2bcd(y / 100);
  time[1] = bin2bcd(y % 100);
  time[2] = bin2bcd(m);
  time[3] = bin2bcd(d);
  time[4] = bin2bcd(s / 3600);
  time[5] = bin2bcd((s / 60) % 60);
  time[6] = bin2bcd(s % 60);
}

static void normalise(spc7110_instant_t *t) {
  while (t->secs >= SECS_PER_DAY) { t->secs -= SECS_PER_DAY; t->days += 1; }
  while (t->secs < 0)             { t->secs += SECS_PER_DAY; t->days -= 1; }
}

void spc7110_rtc_diff(const spc7110_instant_t *a, const spc7110_instant_t *b,
                      spc7110_instant_t *d) {
  d->days = a->days - b->days;
  d->secs = a->secs - b->secs;
  normalise(d);
}

void spc7110_rtc_add(const spc7110_instant_t *a, const spc7110_instant_t *d,
                     spc7110_instant_t *r) {
  r->days = a->days + d->days;
  r->secs = a->secs + d->secs;
  normalise(r);
}

int32_t spc7110_rtc_midnights(const spc7110_instant_t *from,
                              const spc7110_instant_t *to) {
  return to->days - from->days;
}
