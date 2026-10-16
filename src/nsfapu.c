/* 2A03 APU for the NSF player (nsf.h).  Event driven: every channel keeps the
 * CPU cycles left to its next timer step, and nsf_apu_run() advances them in
 * spans, adding level x cycles to the channel's accumulator so nsf_render()
 * can box-filter each output sample.  Register writes and frame-sequencer
 * clocks happen at span boundaries.  Written from the nesdev wiki APU pages. */

#include "nsf.h"

static const uint8_t len_tab[32] = {
  10, 254, 20, 2, 40, 4, 80, 6, 160, 8, 60, 10, 14, 12, 26, 14,
  12, 16, 24, 18, 48, 20, 96, 22, 192, 24, 72, 26, 16, 28, 32, 30
};
/* duty sequences, bit i = output at sequencer step i (the step counts down) */
static const uint8_t duty_tab[4] = { 0x02, 0x06, 0x1e, 0xf9 };

static const uint16_t noise_ntsc[16] = {
  4, 8, 16, 32, 64, 96, 128, 160, 202, 254, 380, 508, 762, 1016, 2034, 4068
};
static const uint16_t noise_pal[16] = {
  4, 8, 14, 30, 60, 88, 118, 148, 188, 236, 354, 472, 708, 944, 1890, 3778
};
static const uint16_t dmc_ntsc[16] = {
  428, 380, 340, 320, 286, 254, 226, 214, 190, 160, 142, 128, 106, 84, 72, 54
};
static const uint16_t dmc_pal[16] = {
  398, 354, 316, 298, 276, 236, 210, 198, 176, 148, 132, 118, 98, 78, 66, 50
};
/* Frame sequencer: CPU cycles from one event to the next, starting at the
 * $4017 write.  4-step: quarter frames at 7457/14913/22371/29829 and the
 * sequence repeats every 29830; 5-step: 7457/14913/22371/(29829 = nothing)/
 * 37281, repeating every 37282.  PAL: 8313/16627/24939/33253(/41565). */
#ifndef FC_DELAY
#define FC_DELAY 3
#endif
static const uint16_t fc4_ntsc[4] = { 7457, 7456, 7458, 7458 };
static const uint16_t fc5_ntsc[5] = { 7457, 7456, 7458, 7458, 7452 };
static const uint16_t fc4_pal[4]  = { 8313, 8314, 8312, 8314 };
static const uint16_t fc5_pal[5]  = { 8313, 8314, 8312, 8314, 8312 };

/* ---- pulse -------------------------------------------------------------- */

static uint16_t pulse_target(const nsf_pulse_t *p, uint8_t ones)
{
  uint16_t d = p->period >> (p->sweep & 7);
  if(p->sweep & 0x08)
    return p->period - d - ones;
  return p->period + d;
}

static void pulse_amp(nsf_pulse_t *p, uint8_t ones)
{
  uint8_t vol = (p->reg0 & 0x10) ? (p->reg0 & 15) : p->env_vol;
  if(!p->len || p->period < 8 || (!(p->sweep & 0x08) && pulse_target(p, ones) > 0x7ff))
    vol = 0;
  p->amp = vol;
  p->level = ((duty_tab[p->reg0 >> 6] >> p->seq) & 1) ? vol : 0;
}

static void pulse_run(nsf_pulse_t *p, uint32_t n)
{
  uint32_t per = ((uint32_t)p->period + 1) << 1;
  if(!p->amp) {                       /* silent: only the phase moves */
    if(n < p->count) { p->count -= n; return; }
    n -= p->count;
    p->seq = (p->seq - 1 - n / per) & 7;
    p->count = per - n % per;
    return;
  }
  uint8_t duty = duty_tab[p->reg0 >> 6];
  uint32_t acc = p->acc;
  while(n >= p->count) {
    acc += (uint32_t)p->level * p->count;
    n -= p->count;
    p->count = per;
    p->seq = (p->seq - 1) & 7;
    p->level = ((duty >> p->seq) & 1) ? p->amp : 0;
  }
  acc += (uint32_t)p->level * n;
  p->count -= n;
  p->acc = acc;
}

static void env_clock(uint8_t reg0, uint8_t *start, uint8_t *div, uint8_t *vol)
{
  if(*start) {
    *start = 0; *vol = 15; *div = reg0 & 15;
  } else if(*div) {
    (*div)--;
  } else {
    *div = reg0 & 15;
    if(*vol) (*vol)--;
    else if(reg0 & 0x20) *vol = 15;
  }
}

static void pulse_half(nsf_pulse_t *p, uint8_t ones)
{
  if(p->len && !(p->reg0 & 0x20)) p->len--;
  uint16_t t = pulse_target(p, ones);
  if(!p->sw_div && (p->sweep & 0x80) && (p->sweep & 7)
     && p->period >= 8 && t <= 0x7ff)
    p->period = t;
  if(!p->sw_div || p->sw_reload) {
    p->sw_div = (p->sweep >> 4) & 7;
    p->sw_reload = 0;
  } else {
    p->sw_div--;
  }
}

/* ---- triangle ----------------------------------------------------------- */

static uint8_t tri_level(uint8_t seq)
{
  return seq < 16 ? 15 - seq : seq - 16;
}

static void tri_run(nsf_tri_t *t, uint32_t n)
{
  /* periods 0 and 1 are ultrasonic: hold the level instead of aliasing */
  if(!t->len || !t->lin || t->period < 2) {
    t->acc += (uint32_t)t->level * n;
    return;
  }
  uint32_t per = (uint32_t)t->period + 1;
  uint32_t acc = t->acc;
  while(n >= t->count) {
    acc += (uint32_t)t->level * t->count;
    n -= t->count;
    t->count = per;
    t->seq = (t->seq + 1) & 31;
    t->level = tri_level(t->seq);
  }
  acc += (uint32_t)t->level * n;
  t->count -= n;
  t->acc = acc;
}

/* ---- noise -------------------------------------------------------------- */

static void noise_amp(nsf_noise_t *ns)
{
  uint8_t vol = (ns->reg0 & 0x10) ? (ns->reg0 & 15) : ns->env_vol;
  ns->amp = ns->len ? vol : 0;
  ns->level = (ns->lfsr & 1) ? 0 : ns->amp;
}

static void noise_run(nsf_noise_t *ns, uint32_t n, const uint16_t *tab)
{
  if(!ns->amp) {                      /* silent: the LFSR phase is not audible */
    if(n < ns->count) ns->count -= n;
    else ns->count = tab[ns->rate] - (n - ns->count) % tab[ns->rate];
    return;
  }
  uint32_t per = tab[ns->rate];
  uint8_t tap = ns->mode ? 6 : 1;
  uint32_t acc = ns->acc;
  uint16_t lfsr = ns->lfsr;
  while(n >= ns->count) {
    acc += (uint32_t)ns->level * ns->count;
    n -= ns->count;
    ns->count = per;
    lfsr = (lfsr >> 1) | ((uint16_t)((lfsr ^ (lfsr >> tap)) & 1) << 14);
    ns->level = (lfsr & 1) ? 0 : ns->amp;
  }
  acc += (uint32_t)ns->level * n;
  ns->count -= n;
  ns->acc = acc;
  ns->lfsr = lfsr;
}

/* ---- DMC ---------------------------------------------------------------- */

static void dmc_restart(nsf_dmc_t *d)
{
  d->addr = 0xc000 | ((uint16_t)d->addr_reg << 6);
  d->left = ((uint16_t)d->len_reg << 4) + 1;
}

static void dmc_fill(nsf_apu_t *apu)
{
  nsf_dmc_t *d = &apu->d;
  if(d->full || !d->left) return;
  d->buffer = nsf_dmc_fetch(apu->ctx, d->addr);
  d->full = 1;
  d->addr = (d->addr == 0xffff) ? 0x8000 : d->addr + 1;
  if(!--d->left) {
    if(d->reg0 & 0x40) dmc_restart(d);
    else if(d->reg0 & 0x80) d->irq = 1;
  }
}

static void dmc_run(nsf_apu_t *apu, uint32_t n, const uint16_t *tab)
{
  nsf_dmc_t *d = &apu->d;
  uint32_t per = tab[d->reg0 & 15];
  while(n >= d->count) {
    d->acc += (uint32_t)d->level * d->count;
    n -= d->count;
    d->count = per;
    if(!d->silence) {
      if(d->shift & 1) { if(d->level <= 125) d->level += 2; }
      else if(d->level >= 2) d->level -= 2;
    }
    d->shift >>= 1;
    if(!--d->bits) {
      d->bits = 8;
      if(d->full) {
        d->silence = 0;
        d->shift = d->buffer;
        d->full = 0;
        dmc_fill(apu);
      } else {
        d->silence = 1;
      }
    }
  }
  d->acc += (uint32_t)d->level * n;
  d->count -= n;
}

/* ---- frame sequencer ---------------------------------------------------- */

static void clock_quarter(nsf_apu_t *apu)
{
  env_clock(apu->p[0].reg0, &apu->p[0].env_start, &apu->p[0].env_div, &apu->p[0].env_vol);
  env_clock(apu->p[1].reg0, &apu->p[1].env_start, &apu->p[1].env_div, &apu->p[1].env_vol);
  env_clock(apu->n.reg0, &apu->n.env_start, &apu->n.env_div, &apu->n.env_vol);
  nsf_tri_t *t = &apu->t;
  if(t->lin_reload) t->lin = t->reg & 0x7f;
  else if(t->lin) t->lin--;
  if(!(t->reg & 0x80)) t->lin_reload = 0;
}

static void clock_half(nsf_apu_t *apu)
{
  pulse_half(&apu->p[0], 1);
  pulse_half(&apu->p[1], 0);
  if(apu->t.len && !(apu->t.reg & 0x80)) apu->t.len--;
  if(apu->n.len && !(apu->n.reg0 & 0x20)) apu->n.len--;
}

static void refresh(nsf_apu_t *apu)
{
  pulse_amp(&apu->p[0], 1);
  pulse_amp(&apu->p[1], 0);
  noise_amp(&apu->n);
}

static const uint16_t *fc_tab(const nsf_apu_t *apu)
{
  if(apu->pal) return apu->fc_mode ? fc5_pal : fc4_pal;
  return apu->fc_mode ? fc5_ntsc : fc4_ntsc;
}

static void fc_event(nsf_apu_t *apu)
{
  uint8_t s = apu->fc_step;
  if(!apu->fc_mode) {
    clock_quarter(apu);
    if(s & 1) clock_half(apu);
    if(s == 3 && !apu->fc_inhibit) {
      apu->fc_irq = 1;
      apu->fc_irq_end = apu->time + 1;
    }
    s = (s + 1) & 3;
  } else {
    if(s != 3) clock_quarter(apu);
    if(s == 1 || s == 4) clock_half(apu);
    s = (s == 4) ? 0 : s + 1;
  }
  apu->fc_step = s;
  /* the tables hold the deltas from the $4017 write; after a wrap the first
     event is one cycle further away (the sequence period is one cycle longer
     than its last event) */
  apu->fc_count = fc_tab(apu)[s] + (s == 0);
  refresh(apu);
}

/* ---- interface ---------------------------------------------------------- */

void nsf_apu_reset(nsf_apu_t *apu, uint8_t pal)
{
  void *ctx = apu->ctx;
  uint32_t time = apu->time;
  volatile uint8_t *b = (volatile uint8_t *)apu;   /* volatile: no memset call */
  for(unsigned i = 0; i < sizeof(*apu); i++) b[i] = 0;
  apu->ctx = ctx;
  apu->time = time;
  apu->pal = pal;
  apu->n.lfsr = 1;
  apu->p[0].count = apu->p[1].count = 2;
  apu->t.count = 1;
  apu->n.count = 4;
  apu->d.count = pal ? dmc_pal[0] : dmc_ntsc[0];
  apu->d.bits = 8;
  apu->d.silence = 1;
  apu->fc_count = fc_tab(apu)[0];
}

void nsf_apu_run(nsf_apu_t *apu, uint32_t until)
{
  int32_t n = (int32_t)(until - apu->time);
  if(n <= 0) return;
  const uint16_t *ntab = apu->pal ? noise_pal : noise_ntsc;
  const uint16_t *dtab = apu->pal ? dmc_pal : dmc_ntsc;
  while(n > 0) {
    uint32_t span = (uint32_t)n < apu->fc_count ? (uint32_t)n : apu->fc_count;
    pulse_run(&apu->p[0], span);
    pulse_run(&apu->p[1], span);
    tri_run(&apu->t, span);
    noise_run(&apu->n, span, ntab);
    dmc_run(apu, span, dtab);
    apu->fc_count -= span;
    apu->time += span;
    n -= span;
    if(!apu->fc_count) fc_event(apu);
  }
}

void nsf_apu_write(nsf_apu_t *apu, uint16_t a, uint8_t v)
{
  switch(a) {
    case 0x4000: case 0x4004:
      apu->p[(a >> 2) & 1].reg0 = v; break;
    case 0x4001: case 0x4005: {
      nsf_pulse_t *p = &apu->p[(a >> 2) & 1];
      p->sweep = v; p->sw_reload = 1; break;
    }
    case 0x4002: case 0x4006: {
      nsf_pulse_t *p = &apu->p[(a >> 2) & 1];
      p->period = (p->period & 0x700) | v; break;
    }
    case 0x4003: case 0x4007: {
      uint8_t ch = (a >> 2) & 1;
      nsf_pulse_t *p = &apu->p[ch];
      p->period = (p->period & 0xff) | ((uint16_t)(v & 7) << 8);
      if(apu->enabled & (1 << ch)) p->len = len_tab[v >> 3];
      p->seq = 0;
      p->env_start = 1;
      break;
    }
    case 0x4008: apu->t.reg = v; break;
    case 0x400a: apu->t.period = (apu->t.period & 0x700) | v; break;
    case 0x400b:
      apu->t.period = (apu->t.period & 0xff) | ((uint16_t)(v & 7) << 8);
      if(apu->enabled & 4) apu->t.len = len_tab[v >> 3];
      apu->t.lin_reload = 1;
      break;
    case 0x400c: apu->n.reg0 = v; break;
    case 0x400e: apu->n.mode = v & 0x80; apu->n.rate = v & 15; break;
    case 0x400f:
      if(apu->enabled & 8) apu->n.len = len_tab[v >> 3];
      apu->n.env_start = 1;
      break;
    /* the DMC registers change nothing refresh() looks at: no refresh (a tune
       that plays samples writes $4011 thousands of times a second) */
    case 0x4010:
      apu->d.reg0 = v;
      if(!(v & 0x80)) apu->d.irq = 0;
      return;
    case 0x4011: apu->d.level = v & 0x7f; return;
    case 0x4012: apu->d.addr_reg = v; return;
    case 0x4013: apu->d.len_reg = v; return;
    case 0x4015:
      apu->enabled = v & 15;
      if(!(v & 1)) apu->p[0].len = 0;
      if(!(v & 2)) apu->p[1].len = 0;
      if(!(v & 4)) apu->t.len = 0;
      if(!(v & 8)) apu->n.len = 0;
      apu->d.irq = 0;
      if(!(v & 0x10)) apu->d.left = 0;
      else if(!apu->d.left) { dmc_restart(&apu->d); dmc_fill(apu); }
      break;
    case 0x4017:
      apu->fc_mode = v >> 7;
      apu->fc_inhibit = (v >> 6) & 1;
      if(apu->fc_inhibit) apu->fc_irq = 0;
      apu->fc_step = 0;
      /* the sequencer restarts 3 or 4 cycles after the write, depending on
         whether it lands on an APU cycle (the immediate clocks of mode 1 are
         not delayed here) */
      apu->fc_count = fc_tab(apu)[0] + FC_DELAY + (apu->time & 1);
      if(apu->fc_mode) { clock_quarter(apu); clock_half(apu); }
      break;
    default:
      return;
  }
  refresh(apu);
}

uint8_t nsf_apu_status(nsf_apu_t *apu)
{
  /* the IRQ flag goes up one cycle before the 4th step's clock (29828,
     29829 and 29830 after the restart) */
  if(!apu->fc_mode && !apu->fc_inhibit && apu->fc_step == 3 && apu->fc_count <= 1)
    apu->fc_irq = 1;
  uint8_t v = (apu->p[0].len ? 1 : 0) | (apu->p[1].len ? 2 : 0)
            | (apu->t.len ? 4 : 0) | (apu->n.len ? 8 : 0)
            | (apu->d.left ? 0x10 : 0) | (apu->fc_irq ? 0x40 : 0)
            | (apu->d.irq ? 0x80 : 0);
  /* still inside the 3 cycles that set it: it comes right back */
  apu->fc_irq = (apu->fc_irq && (int32_t)(apu->fc_irq_end - apu->time) > 0);
  return v;
}
