/* NSF player core: header, backing-store cache, 6502 and the render loop.
 * See nsf.h for the memory model.  The CPU is a plain 2A03 (no decimal mode)
 * with the stable undocumented opcodes; cycle counts follow the standard
 * tables, without dummy reads/writes or the DMC DMA stall. */

#include <stddef.h>
#include "nsf.h"

_Static_assert(offsetof(nsf_t, vtag) > offsetof(nsf_t, map)
               && offsetof(nsf_t, vdirty) == offsetof(nsf_t, vused) + NSF_VPAGES,
               "nsf_start fills these as runs");

#define NSF_RET      0x3ff0     /* RTS lands here when INIT/PLAY returns */
#define CPU_NTSC     1789773
#define CPU_PAL      1662607

/* p flags */
#define F_C 0x01
#define F_Z 0x02
#define F_I 0x04
#define F_V 0x40
#define F_N 0x80

enum { M_IMP, M_ACC, M_IMM, M_ZP, M_ZPX, M_ZPY, M_ABS, M_ABX, M_ABY, M_IZX, M_IZY, M_REL, M_IND };

/* operand reads: I_LDA..I_LXA, read-modify-write: I_ASL..I_ISB (the shifts first, in the
   SH_* order: shared code below), stores: I_STA..I_TAS */
enum {
  I_LDA, I_LDX, I_LDY, I_LAX, I_ORA, I_AND, I_EOR, I_ADC, I_SBC, I_CMP, I_CPX, I_CPY,
  I_BIT, I_NOP, I_LAS, I_ANC, I_ALR, I_ARR, I_AXS, I_XAA, I_LXA, I_ASL, I_ROL, I_LSR,
  I_ROR, I_SLO, I_RLA, I_SRE, I_RRA, I_INC, I_DEC, I_DCP, I_ISB, I_STA, I_STX, I_STY,
  I_SAX, I_SHA, I_SHX, I_SHY, I_TAS, I_BRK, I_JAM, I_PHP, I_BPL, I_CLC, I_JSR, I_PLP,
  I_BMI, I_SEC, I_RTI, I_PHA, I_JMP, I_BVC, I_CLI, I_RTS, I_PLA, I_BVS, I_SEI, I_DEY,
  I_TXA, I_BCC, I_TYA, I_TXS, I_TAY, I_TAX, I_BCS, I_CLV, I_TSX, I_INY, I_DEX, I_BNE,
  I_CLD, I_INX, I_BEQ, I_SED,
};
static const uint8_t op_mn[256] = {
  I_BRK, I_ORA, I_JAM, I_SLO, I_NOP, I_ORA, I_ASL, I_SLO, I_PHP, I_ORA, I_ASL, I_ANC, I_NOP, I_ORA, I_ASL, I_SLO,
  I_BPL, I_ORA, I_JAM, I_SLO, I_NOP, I_ORA, I_ASL, I_SLO, I_CLC, I_ORA, I_NOP, I_SLO, I_NOP, I_ORA, I_ASL, I_SLO,
  I_JSR, I_AND, I_JAM, I_RLA, I_BIT, I_AND, I_ROL, I_RLA, I_PLP, I_AND, I_ROL, I_ANC, I_BIT, I_AND, I_ROL, I_RLA,
  I_BMI, I_AND, I_JAM, I_RLA, I_NOP, I_AND, I_ROL, I_RLA, I_SEC, I_AND, I_NOP, I_RLA, I_NOP, I_AND, I_ROL, I_RLA,
  I_RTI, I_EOR, I_JAM, I_SRE, I_NOP, I_EOR, I_LSR, I_SRE, I_PHA, I_EOR, I_LSR, I_ALR, I_JMP, I_EOR, I_LSR, I_SRE,
  I_BVC, I_EOR, I_JAM, I_SRE, I_NOP, I_EOR, I_LSR, I_SRE, I_CLI, I_EOR, I_NOP, I_SRE, I_NOP, I_EOR, I_LSR, I_SRE,
  I_RTS, I_ADC, I_JAM, I_RRA, I_NOP, I_ADC, I_ROR, I_RRA, I_PLA, I_ADC, I_ROR, I_ARR, I_JMP, I_ADC, I_ROR, I_RRA,
  I_BVS, I_ADC, I_JAM, I_RRA, I_NOP, I_ADC, I_ROR, I_RRA, I_SEI, I_ADC, I_NOP, I_RRA, I_NOP, I_ADC, I_ROR, I_RRA,
  I_NOP, I_STA, I_NOP, I_SAX, I_STY, I_STA, I_STX, I_SAX, I_DEY, I_NOP, I_TXA, I_XAA, I_STY, I_STA, I_STX, I_SAX,
  I_BCC, I_STA, I_JAM, I_SHA, I_STY, I_STA, I_STX, I_SAX, I_TYA, I_STA, I_TXS, I_TAS, I_SHY, I_STA, I_SHX, I_SHA,
  I_LDY, I_LDA, I_LDX, I_LAX, I_LDY, I_LDA, I_LDX, I_LAX, I_TAY, I_LDA, I_TAX, I_LXA, I_LDY, I_LDA, I_LDX, I_LAX,
  I_BCS, I_LDA, I_JAM, I_LAX, I_LDY, I_LDA, I_LDX, I_LAX, I_CLV, I_LDA, I_TSX, I_LAS, I_LDY, I_LDA, I_LDX, I_LAX,
  I_CPY, I_CMP, I_NOP, I_DCP, I_CPY, I_CMP, I_DEC, I_DCP, I_INY, I_CMP, I_DEX, I_AXS, I_CPY, I_CMP, I_DEC, I_DCP,
  I_BNE, I_CMP, I_JAM, I_DCP, I_NOP, I_CMP, I_DEC, I_DCP, I_CLD, I_CMP, I_NOP, I_DCP, I_NOP, I_CMP, I_DEC, I_DCP,
  I_CPX, I_SBC, I_NOP, I_ISB, I_CPX, I_SBC, I_INC, I_ISB, I_INX, I_SBC, I_NOP, I_SBC, I_CPX, I_SBC, I_INC, I_ISB,
  I_BEQ, I_SBC, I_JAM, I_ISB, I_NOP, I_SBC, I_INC, I_ISB, I_SED, I_SBC, I_NOP, I_ISB, I_NOP, I_SBC, I_INC, I_ISB,
};
/* high nibble = addressing mode (M_*), low nibble = base cycles */
static const uint8_t op_mc[256] = {
  0x07, 0x96, 0x02, 0x98, 0x33, 0x33, 0x35, 0x35, 0x03, 0x22, 0x12, 0x22, 0x64, 0x64, 0x66, 0x66,
  0xb2, 0xa5, 0x02, 0xa8, 0x44, 0x44, 0x46, 0x46, 0x02, 0x84, 0x02, 0x87, 0x74, 0x74, 0x77, 0x77,
  0x66, 0x96, 0x02, 0x98, 0x33, 0x33, 0x35, 0x35, 0x04, 0x22, 0x12, 0x22, 0x64, 0x64, 0x66, 0x66,
  0xb2, 0xa5, 0x02, 0xa8, 0x44, 0x44, 0x46, 0x46, 0x02, 0x84, 0x02, 0x87, 0x74, 0x74, 0x77, 0x77,
  0x06, 0x96, 0x02, 0x98, 0x33, 0x33, 0x35, 0x35, 0x03, 0x22, 0x12, 0x22, 0x63, 0x64, 0x66, 0x66,
  0xb2, 0xa5, 0x02, 0xa8, 0x44, 0x44, 0x46, 0x46, 0x02, 0x84, 0x02, 0x87, 0x74, 0x74, 0x77, 0x77,
  0x06, 0x96, 0x02, 0x98, 0x33, 0x33, 0x35, 0x35, 0x04, 0x22, 0x12, 0x22, 0xc5, 0x64, 0x66, 0x66,
  0xb2, 0xa5, 0x02, 0xa8, 0x44, 0x44, 0x46, 0x46, 0x02, 0x84, 0x02, 0x87, 0x74, 0x74, 0x77, 0x77,
  0x22, 0x96, 0x22, 0x96, 0x33, 0x33, 0x33, 0x33, 0x02, 0x22, 0x02, 0x22, 0x64, 0x64, 0x64, 0x64,
  0xb2, 0xa6, 0x02, 0xa6, 0x44, 0x44, 0x54, 0x54, 0x02, 0x85, 0x02, 0x85, 0x75, 0x75, 0x85, 0x85,
  0x22, 0x96, 0x22, 0x96, 0x33, 0x33, 0x33, 0x33, 0x02, 0x22, 0x02, 0x22, 0x64, 0x64, 0x64, 0x64,
  0xb2, 0xa5, 0x02, 0xa5, 0x44, 0x44, 0x54, 0x54, 0x02, 0x84, 0x02, 0x84, 0x74, 0x74, 0x84, 0x84,
  0x22, 0x96, 0x22, 0x98, 0x33, 0x33, 0x35, 0x35, 0x02, 0x22, 0x02, 0x22, 0x64, 0x64, 0x66, 0x66,
  0xb2, 0xa5, 0x02, 0xa8, 0x44, 0x44, 0x46, 0x46, 0x02, 0x84, 0x02, 0x87, 0x74, 0x74, 0x77, 0x77,
  0x22, 0x96, 0x22, 0x98, 0x33, 0x33, 0x35, 0x35, 0x02, 0x22, 0x02, 0x22, 0x64, 0x64, 0x66, 0x66,
  0xb2, 0xa5, 0x02, 0xa8, 0x44, 0x44, 0x46, 0x46, 0x02, 0x84, 0x02, 0x87, 0x74, 0x74, 0x77, 0x77,
};

/* Nonlinear DAC mix (nesdev "APU Mixer", the formula approximation), scaled
   so the loudest possible output is ~28000; n16 is the channel sum x 16:
     pulse = 28000 * 95.52  * n / (8128 + 100 n),   n = pulse1 + pulse2
     tnd   = 28000 * 163.67 * n / (24329 + 100 n),  n = 3*tri + 2*noise + dmc
   both divided through by 4 so the products stay in 32 bits. */
static uint32_t mix_pulse(uint32_t n16) { return 668640 * n16 / (32512 + 25 * n16); }
static uint32_t mix_tnd(uint32_t n16)   { return 1145690 * n16 / (97316 + 25 * n16); }

/* ---- header ------------------------------------------------------------- */

static uint16_t le16(const uint8_t *p) { return p[0] | (p[1] << 8); }

uint8_t nsf_parse(nsf_info_t *info, const uint8_t hdr[NSF_HDR_SIZE], uint32_t file_size)
{
  static const uint8_t magic[5] = { 'N', 'E', 'S', 'M', 0x1a };
  uint8_t banked = 0;
  uint32_t len;
  for(unsigned i = 0; i < 5; i++)
    if(hdr[i] != magic[i]) return 0;
  if(file_size <= NSF_HDR_SIZE || !hdr[6]) return 0;
  len = file_size - NSF_HDR_SIZE;
  info->songs = hdr[6];
  info->start = (hdr[7] && hdr[7] <= hdr[6]) ? hdr[7] : 1;
  info->load = le16(hdr + 0x08);
  info->init = le16(hdr + 0x0a);
  info->play = le16(hdr + 0x0c);
  info->pal = (hdr[0x7a] & 3) == 1;
  info->speed_us = le16(hdr + (info->pal ? 0x78 : 0x6e));
  if(!info->speed_us) info->speed_us = info->pal ? 19997 : 16639;
  info->chip = hdr[0x7b];
  for(unsigned i = 0; i < 8; i++) {
    info->banks[i] = hdr[0x70 + i];
    banked |= hdr[0x70 + i];
  }
  if(info->load < 0x8000) return 0;
  if(banked) {
    info->pad = info->load & 0xfff;
    if(info->pad + len > 256UL * 4096) return 0;
    info->nbanks = (info->pad + len + 4095) >> 12;
  } else {
    /* a flat tune is a banked one with banks 0..7 and the data at load-$8000 */
    info->pad = info->load - 0x8000;
    info->nbanks = 8;
    for(unsigned i = 0; i < 8; i++) info->banks[i] = i;
  }
  info->data_len = len;
  if(info->pad + len > (uint32_t)info->nbanks << 12)
    info->data_len = ((uint32_t)info->nbanks << 12) - info->pad;   /* past $FFFF */
  return 1;
}

/* ---- memory ------------------------------------------------------------- */

static uint32_t bk_addr(const nsf_t *n, uint16_t a)
{
  if(a < 0x2000) return NSF_BK_RAM + (a & 0x7ff);
  if(a < 0x8000) return NSF_BK_SRAM + (a - 0x6000);
  uint16_t b = n->bank[(a >> 12) & 7];
  if(b >= n->info.nbanks) b %= n->info.nbanks;
  return NSF_BK_ROM + ((uint32_t)b << 12) + (a & 0xfff);
}

/* The cache (see nsf.h): a NES page maps to a virtual page (map[]), which
   holds up to 8 sectors of 16 bytes from the pool (vslot).  A miss reads one
   sector: the PSRAM costs ~1.5 us a byte from the MCU, so a tune whose loop
   spans many pages but few bytes of each (the usual) fits in a pool that its
   128-byte lines never did. */
#define VSLOT(n, v) ((n)->vslot + ((v) << 3))

/* Unmaps every NES page that points at virtual page v. */
static void vpage_unmap(nsf_t *n, uint8_t v)
{
  for(unsigned i = 0; i < 512; i++)
    if(n->map[i] == v) n->map[i] = 0xff;
}

static void sect_writeback(nsf_t *n, uint8_t v, uint8_t s, uint8_t p)
{
  if(n->vdirty[v] & (1 << s)) {
    n->bk(n->bk_ctx, ((uint32_t)n->vtag[v] << NSF_LINE_SHIFT) + (s << NSF_SECT_SHIFT),
          n->pool[p], NSF_SECT, 1);
    n->vdirty[v] &= ~(1 << s);
    n->writebacks++;
  }
}

/* The virtual page of a's backing page, found by tag or given a victim's slot
   (the next one, going round, that the current PLAY call has not touched:
   see nsf_age), and mapped for a's NES page. */
static uint8_t vpage_get(nsf_t *n, uint16_t a)
{
  uint16_t bl = bk_addr(n, a) >> NSF_LINE_SHIFT;
  unsigned v, k;
  for(v = 0; v < NSF_VPAGES; v++)
    if(n->vtag[v] == bl) break;
  if(v == NSF_VPAGES) {
    k = NSF_VPAGES - NSF_PINNED;
    v = n->vvictim;
    while(k-- && n->vused[v])
      if(++v >= NSF_VPAGES) v = NSF_PINNED;
    n->vvictim = (v + 1 >= NSF_VPAGES) ? NSF_PINNED : v + 1;
    vpage_unmap(n, v);
    for(k = 0; k < NSF_SPP; k++) {
      uint8_t p = VSLOT(n, v)[k];
      if(p != 0xff) {
        sect_writeback(n, v, k, p);
        n->sown[p] = 0xff;
        VSLOT(n, v)[k] = 0xff;
      }
    }
    n->vtag[v] = bl;
  }
  n->map[a >> NSF_LINE_SHIFT] = v;
  n->vused[v] = 1;
  return v;
}

/* The pool sector holding a (RAM, $6000 RAM or ROM only), read on a miss.
   Victim: the next free sector, or the next whose page the current PLAY call
   has not touched; if every one was touched, the next one. */
static uint8_t *sect_get(nsf_t *n, uint16_t a)
{
  uint8_t v = n->map[a >> NSF_LINE_SHIFT], s = (a >> NSF_SECT_SHIFT) & (NSF_SPP - 1), p;
  if(v == 0xff) v = vpage_get(n, a);
  p = VSLOT(n, v)[s];
  if(p == 0xff) {
    unsigned k = n->nsect - NSF_PINNED * NSF_SPP;
    p = n->pvictim;
    while(k-- && n->sown[p] != 0xff && n->vused[n->sown[p] >> 3])
      if(++p >= n->nsect) p = NSF_PINNED * NSF_SPP;
    n->pvictim = (p + 1 >= n->nsect) ? NSF_PINNED * NSF_SPP : p + 1;
    if(n->sown[p] != 0xff) {
      uint8_t ov = n->sown[p] >> 3, os = n->sown[p] & 7;
      sect_writeback(n, ov, os, p);
      VSLOT(n, ov)[os] = 0xff;
    }
    n->bk(n->bk_ctx, ((uint32_t)n->vtag[v] << NSF_LINE_SHIFT) + (s << NSF_SECT_SHIFT),
          n->pool[p], NSF_SECT, 0);
    n->misses++;
    n->sown[p] = (v << 3) | s;
    VSLOT(n, v)[s] = p;
  }
  return n->pool[p];
}

/* Start of a PLAY call: forget which pages were used and unmap them, so the
   first access of this call goes through vpage_get (a tag hit, no backing
   read) and marks the page used again.  Pages and sectors a whole call goes
   without are the ones evicted first: a cheap LRU for a loop that runs once
   per frame. */
static void nsf_age(nsf_t *n)
{
  for(unsigned i = 0; i < 512; i++)
    if(n->map[i] != 0xff && n->map[i] >= NSF_PINNED) n->map[i] = 0xff;
  for(unsigned v = NSF_PINNED; v < NSF_VPAGES; v++) n->vused[v] = 0;
}

/* The cycle an access of the current instruction happens on (its last one). */
static uint32_t apu_now(nsf_t *n)
{
  uint32_t t = n->cyc + n->ofs;
  if(n->rendering && (int32_t)(t - n->lim) > 0) t = n->lim;
  return t;
}

static void apu_sync(nsf_t *n)
{
  nsf_apu_run(&n->apu, apu_now(n));
}

static __attribute__((noinline)) uint8_t rd_slow(nsf_t *n, uint16_t a)
{
  if(a >= 0x2000 && a < 0x6000) {
    if(a == 0x4015) {
      apu_sync(n);
      return nsf_apu_status(&n->apu);
    }
    if(n->io_read) return n->io_read(n, a);
    return a >> 8;                  /* open bus */
  }
  return sect_get(n, a)[a & (NSF_SECT - 1)];
}

/* -Os does not inline a plain "inline" here: every access became a call. */
static inline __attribute__((always_inline)) uint8_t rd(nsf_t *n, uint16_t a)
{
  uint8_t v = n->map[a >> NSF_LINE_SHIFT];
  if(v != 0xff) {
    uint8_t p = VSLOT(n, v)[(a >> NSF_SECT_SHIFT) & (NSF_SPP - 1)];
    if(p != 0xff) return n->pool[p][a & (NSF_SECT - 1)];
  }
  return rd_slow(n, a);
}

/* $0000-$01FF (zero page and stack): the pinned pages 0-3, whose sectors are
   the first 32 of the pool, in order, loaded by nsf_start and never evicted. */
#define ZP(n) ((uint8_t *)(n)->pool)

static void bank_set(nsf_t *n, uint8_t slot, uint8_t v)
{
  uint16_t page = 0x100 + ((uint16_t)slot << 5);   /* 32 pages of 128 bytes */
  n->bank[slot] = v;
  for(unsigned i = 0; i < 32; i++) n->map[page + i] = 0xff;
}

static void wr_slow(nsf_t *n, uint16_t a, uint8_t v)
{
  if(a < 0x2000 || (a >= 0x6000 && a < 0x8000)) {
    sect_get(n, a)[a & (NSF_SECT - 1)] = v;
    n->vdirty[n->map[a >> NSF_LINE_SHIFT]] |= 1 << ((a >> NSF_SECT_SHIFT) & (NSF_SPP - 1));
  } else if(a >= 0x4000 && a <= 0x4017) {
    nsf_dmc_t *d = &n->apu.d;
    int32_t dt;
    if(a == 0x4011 && d->silence && !d->full && !d->left) {
      /* $4011 with the DMC idle, whose level then changes nowhere else: the
         other channels need not run up to this cycle.  The span from
         apu.time on runs later at the new level, so the old one is owed its
         share now.  A tune that plays samples writes this thousands of times
         a second, and a full sync cost ~570 ARM cycles. */
      dt = (int32_t)(apu_now(n) - n->apu.time);
      if(dt > 0) d->acc += ((int32_t)d->level - (v & 0x7f)) * dt;
      d->level = v & 0x7f;
    } else {
      apu_sync(n);
      nsf_apu_write(&n->apu, a, v);
    }
    n->apu_at = n->cyc;
  } else if(a >= 0x5ff8 && a <= 0x5fff) {
    bank_set(n, a & 7, v);
  }
  /* ROM and everything else: ignored */
}

static inline void wr(nsf_t *n, uint16_t a, uint8_t v)
{
  uint8_t m = n->map[a >> NSF_LINE_SHIFT], s = (a >> NSF_SECT_SHIFT) & (NSF_SPP - 1), p;
  if(m != 0xff) {
    if(a >= 0x8000) return;         /* ROM: ignored (only RAM pages are mapped below) */
    p = VSLOT(n, m)[s];
    if(p != 0xff) {
      n->pool[p][a & (NSF_SECT - 1)] = v;
      n->vdirty[m] |= 1 << s;
      return;
    }
  }
  wr_slow(n, a, v);
}

/* ---- 6502 --------------------------------------------------------------- */

/* Out-of-line memory access for the instruction bodies: inlining rd()/wr()
   into every case tripled the size of the CPU.  The opcode and its operand
   bytes are fetched inline, once, and zero page and stack go straight to the
   pinned lines (ZP). */
static __attribute__((noinline)) uint8_t rd_op(nsf_t *n, uint16_t a) { return rd(n, a); }
static __attribute__((noinline)) void wr_op(nsf_t *n, uint16_t a, uint8_t v) { wr(n, a, v); }

uint8_t nsf_peek(nsf_t *n, uint16_t a) { return rd_op(n, a); }
void nsf_poke(nsf_t *n, uint16_t a, uint8_t v) { wr_op(n, a, v); }
uint8_t nsf_dmc_fetch(void *ctx, uint16_t a) { return rd_op((nsf_t *)ctx, a); }

/* call() (player) pushes the return address before the CPU runs */
static void push(nsf_t *n, uint8_t v)
{
  ZP(n)[0x100 | n->s] = v;
  n->s--;
}

/* Shift/rotate kinds; the result in bits 0-7, the carry out in bit 8. */
enum { SH_ASL, SH_ROL, SH_LSR, SH_ROR };

static unsigned shift(unsigned v, unsigned kind, unsigned c)
{
  if(kind <= SH_ROL) return ((v << 1) | (kind == SH_ROL ? c : 0)) & 0x1ff;
  return ((v >> 1) | (kind == SH_ROR ? c << 7 : 0)) | ((v & 1) << 8);
}

/* The CPU runs in cpu_run with its registers in locals: before, every
   instruction was a call that loaded and stored them in the nsf_t.  N and Z
   are kept as the values they come from (fn: bit 7 is N; fz: zero means Z),
   so the usual instruction sets both with two moves; p holds the others. */
#define SETNZ(v)  (fn = fz = (uint8_t)(v))
#define PFLAGS()  ((p & ~(F_N | F_Z)) | (fn & F_N) | (fz ? 0 : F_Z))
#define PUSH(v)   (zp[0x100 | s--] = (v))
#define PULL()    (zp[0x100 | ++s])
#define RD(ad)    rd_op(n, (ad))
#define WR(ad, v) wr_op(n, (ad), (v))
#define SHIFT(val, k) do { unsigned t_ = shift((val), (k), p & F_C); \
                           p = (p & ~F_C) | (t_ >> 8); SETNZ(v = t_); } while(0)

/* Runs instructions until the cycle count reaches end, an INIT or PLAY call
   returns to NSF_RET, the CPU jams or, with single, after one instruction.
   n->cyc (the instruction's first cycle) and n->ofs are kept current for the
   out-of-line accesses: the APU syncs to n->cyc + n->ofs. */
static void cpu_run(nsf_t *n, uint32_t end, uint8_t single)
{
  uint8_t a = n->a, x = n->x, y = n->y, s = n->s, p = n->p, fn = p, fz = !(p & F_Z);
  uint16_t pc = n->pc;
  uint32_t cyc = n->cyc;
  uint8_t *zp = ZP(n);
  do {
    uint16_t ea = 0, base = 0;
    /* the instruction's bytes: one lookup when all three can sit in the sector
       that holds the opcode (ip), one access each otherwise */
    const uint8_t *ip = 0;
    uint8_t m = n->map[pc >> NSF_LINE_SHIFT];
    n->cyc = cyc;
    if(m != 0xff && (pc & (NSF_SECT - 1)) < NSF_SECT - 2) {
      uint8_t q = VSLOT(n, m)[(pc >> NSF_SECT_SHIFT) & (NSF_SPP - 1)];
      if(q != 0xff) ip = &n->pool[q][pc & (NSF_SECT - 1)];
    }
    uint8_t op = ip ? ip[0] : RD(pc);
    uint8_t mc = op_mc[op], mode = mc >> 4, mn = op_mn[op], cross = 0, v = 0, z = 0;
    unsigned c = mc & 15;

    pc++;
    /* z = the first operand byte, base = both (3-byte modes) */
    if(mode >= M_IMM) {
      z = ip ? ip[1] : RD(pc);
      pc++;
      if(mode == M_ABS || mode == M_ABX || mode == M_ABY || mode == M_IND) {
        base = z | ((ip ? ip[2] : RD(pc)) << 8);
        pc++;
      }
    }
    /* DEX or DEY and a BNE back to it, while rendering: the whole countdown at
       once (the loop touches nothing but the register; nestest steps it).  A
       tune that feeds $4011 from a timed loop runs it thousands of times a
       second.  NSF_NO_IDLE_SKIP: the host test's reference build. */
#ifndef NSF_NO_IDLE_SKIP
    if((op == 0xca || op == 0x88) && n->rendering
       && (ip ? ip[1] : RD(pc)) == 0xd0 && (ip ? ip[2] : RD(pc + 1)) == 0xfd) {
      uint8_t *r = (op == 0xca) ? &x : &y;
      unsigned k = *r ? *r : 256, taken = 3 + (((pc + 2) ^ (pc - 1)) > 0xff);
      *r = 0;
      fn = fz = 0;
      pc += 2;
      cyc += k * (2 + taken) - (taken - 2);   /* the last BNE falls through: 2 cycles */
      goto next;
    }
#endif

    n->ofs = c - 1;
    switch(mode) {
      case M_IMM: ea = pc - 1; break;
      case M_ZP:  ea = z; break;
      case M_ZPX: ea = (uint8_t)(z + x); break;
      case M_ZPY: ea = (uint8_t)(z + y); break;
      case M_ABS: ea = base; break;
      case M_ABX:
      case M_ABY:
        ea = base + (mode == M_ABX ? x : y);
        cross = (ea ^ base) > 0xff;
        break;
      case M_IZX:
        z += x;
        ea = zp[z] | (zp[(uint8_t)(z + 1)] << 8);
        break;
      case M_IZY:
        base = zp[z] | (zp[(uint8_t)(z + 1)] << 8);
        ea = base + y;
        cross = (ea ^ base) > 0xff;
        break;
      case M_REL:
        ea = pc + (int8_t)z;
        break;
      case M_IND:
        ea = RD(base) | (RD((base & 0xff00) | ((base + 1) & 0xff)) << 8);
        break;
    }

    /* operand in v: read ops (+1 cycle on a page cross) and read-modify-write */
    if(mn <= I_ISB && mode != M_IMP) {
      if(mode == M_ACC) v = a;
      else if(mode == M_IMM) v = z;
      else if(mode <= M_ZPY) v = zp[ea];
      else v = RD(ea);
      if(mn <= I_LXA) c += cross;
    }

    switch(mn) {
      /* reads */
      case I_LDA: SETNZ(a = v); break;
      case I_LDX: SETNZ(x = v); break;
      case I_LDY: SETNZ(y = v); break;
      case I_LAX: SETNZ(a = x = v); break;
      case I_ORA: ora: SETNZ(a |= v); break;
      case I_AND: and: SETNZ(a &= v); break;
      case I_EOR: eor: SETNZ(a ^= v); break;
      case I_SBC: v = ~v;           /* fall through */
      case I_ADC: z = v;
      adc: {                        /* A + z + C; z, so v survives for RRA and ISB */
        unsigned r_ = a + z + (p & F_C);
        p = (p & ~(F_C | F_V)) | (r_ > 0xff ? F_C : 0) | ((~(a ^ z) & (a ^ r_) & 0x80) ? F_V : 0);
        SETNZ(a = r_);
        break;
      }
      case I_CMP: z = v;
      cmpa: base = a; goto cmp;
      case I_CPX: z = v; base = x; goto cmp;
      case I_CPY: z = v; base = y;
      cmp:                          /* register (base) - z */
        p = (p & ~F_C) | ((uint8_t)base >= z ? F_C : 0);
        SETNZ(base - z);
        break;
      case I_BIT:
        p = (p & ~F_V) | (v & F_V);
        fn = v;
        fz = a & v;
        break;
      case I_NOP: break;
      case I_LAS: SETNZ(a = x = s = v & s); break;
      case I_ANC:
        SETNZ(a &= v);
        p = (p & ~F_C) | (a >> 7);
        break;
      case I_ALR: SHIFT(a & v, SH_LSR); a = v; break;
      case I_ARR:
        SHIFT(a & v, SH_ROR);
        a = v;
        p = (p & ~(F_C | F_V)) | ((a >> 6) & 1) | ((((a >> 6) ^ (a >> 5)) & 1) ? F_V : 0);
        break;
      case I_AXS:
        z = a & x;
        p = (p & ~F_C) | (z >= v ? F_C : 0);
        SETNZ(x = z - v);
        break;
      case I_XAA: SETNZ(a = (a | 0xee) & x & v); break;
      case I_LXA: SETNZ(a = x = (a | 0xee) & v); break;
      /* read-modify-write: the new value goes back after the switch */
      case I_ASL: case I_ROL: case I_LSR: case I_ROR:
      case I_SLO: case I_RLA: case I_SRE: case I_RRA:
        SHIFT(v, (mn - I_ASL) & 3);
        if(mn == I_SLO) goto ora;
        if(mn == I_RLA) goto and;
        if(mn == I_SRE) goto eor;
        if(mn == I_RRA) { z = v; goto adc; }
        break;
      case I_INC: SETNZ(++v); break;
      case I_DEC: SETNZ(--v); break;
      case I_DCP: z = --v; goto cmpa;
      case I_ISB: z = ~++v; goto adc;
      /* stores */
      case I_STA: v = a; break;
      case I_STX: v = x; break;
      case I_STY: v = y; break;
      case I_SAX: v = a & x; break;
      case I_SHA: v = a & x & ((ea >> 8) + 1); break;
      case I_SHX: v = x & ((ea >> 8) + 1); break;
      case I_SHY: v = y & ((ea >> 8) + 1); break;
      case I_TAS: s = a & x; v = s & ((ea >> 8) + 1); break;
      /* the rest */
      case I_INX: SETNZ(++x); break;
      case I_INY: SETNZ(++y); break;
      case I_DEX: SETNZ(--x); break;
      case I_DEY: SETNZ(--y); break;
      case I_TAX: SETNZ(x = a); break;
      case I_TAY: SETNZ(y = a); break;
      case I_TXA: SETNZ(a = x); break;
      case I_TYA: SETNZ(a = y); break;
      case I_TSX: SETNZ(x = s); break;
      case I_TXS: s = x; break;
      case I_CLC: p &= ~F_C; break;
      case I_SEC: p |= F_C; break;
      case I_CLI: p &= ~F_I; break;
      case I_SEI: p |= F_I; break;
      case I_CLV: p &= ~F_V; break;
      case I_CLD: p &= ~0x08; break;
      case I_SED: p |= 0x08; break;
      case I_PHA: PUSH(a); break;
      case I_PHP: PUSH(PFLAGS() | 0x30); break;
      case I_PLA: SETNZ(a = PULL()); break;
      case I_PLP:
        p = (PULL() & ~0x10) | 0x20;
        fn = p;
        fz = !(p & F_Z);
        break;
      /* xxy10000: flag xx (N V C Z) equal to y */
      case I_BPL: case I_BMI: case I_BVC: case I_BVS:
      case I_BCC: case I_BCS: case I_BNE: case I_BEQ: {
        static const uint8_t bflag[4] = { F_N, F_V, F_C, F_Z };
        if(!(PFLAGS() & bflag[op >> 6]) == !(op & 0x20)) {
          c += 1 + ((ea ^ pc) > 0xff);
          pc = ea;
        }
        break;
      }
      case I_JMP: pc = ea; break;
      case I_JSR:
        PUSH((pc - 1) >> 8); PUSH(pc - 1);
        pc = ea;
        break;
      case I_RTS:
        v = PULL();
        pc = (v | (PULL() << 8)) + 1;
        break;
      case I_RTI:
        p = (PULL() & ~0x10) | 0x20;
        fn = p;
        fz = !(p & F_Z);
        v = PULL();
        pc = v | (PULL() << 8);
        break;
      case I_BRK:
        pc++;
        PUSH(pc >> 8); PUSH(pc); PUSH(PFLAGS() | 0x30);
        p |= F_I;
        pc = RD(0xfffe) | (RD(0xffff) << 8);
        break;
      case I_JAM:
        pc--;
        n->err = NSF_ERR_JAM;
        break;
    }

    if(mn >= I_ASL && mn <= I_TAS) {
      if(mode == M_ACC) a = v;
      else if(mode <= M_ZPY) zp[ea] = v;
      else WR(ea, v);
    }
    cyc += c;
    if(mn == I_JAM) break;
#ifndef NSF_NO_IDLE_SKIP
next:
#endif
    if(pc == NSF_RET || single) break;
  } while((int32_t)(end - cyc) > 0);
  n->a = a; n->x = x; n->y = y; n->s = s;
  n->p = PFLAGS();
  n->pc = pc;
  n->cyc = cyc;
}

unsigned nsf_step(nsf_t *n)
{
  uint32_t c0 = n->cyc;
  unsigned c;
  cpu_run(n, c0 + 1, 1);
  c = n->cyc - c0;
  n->cyc = c0;                      /* the caller adds the cycles */
  return c;
}

/* ---- player ------------------------------------------------------------- */

uint8_t nsf_open(nsf_t *n, const nsf_info_t *info, nsf_bk_fn bk, void *bk_ctx,
                 uint8_t *mem, uint32_t size)
{
  uint32_t nsect = size < NSF_VPAGES * NSF_SPP ? 0 : (size - NSF_VPAGES * NSF_SPP) / (NSF_SECT + 1);
  /* field by field: a struct copy would call memcpy, which the stand-alone
     build does not have */
  const uint8_t *src = (const uint8_t *)info;
  volatile uint8_t *dst = (volatile uint8_t *)&n->info;
  for(unsigned i = 0; i < sizeof(*info); i++) dst[i] = src[i];
  if(nsect < NSF_MINSECT) return 0;
  if(nsect > 255) nsect = 255;      /* a sector number is a byte, 0xff = none */
  n->vslot = mem;
  n->sown = mem + NSF_VPAGES * NSF_SPP;
  n->pool = (uint8_t (*)[NSF_SECT])(n->sown + nsect);
  n->nsect = nsect;
  n->bk = bk;
  n->bk_ctx = bk_ctx;
  n->io_read = 0;
  n->cpu_hz = info->pal ? CPU_PAL : CPU_NTSC;
  /* PLAY period in 24.8 cycles: speed_us * cpu_hz / 1e6, with cpu_hz/100 so
     the product stays in 32 bits */
  uint32_t q = (uint32_t)info->speed_us * (n->cpu_hz / 100);
  n->period_int = q / 10000;
  n->period_frac = (q % 10000) * 256 / 10000;
  /* The standard header values are a rounded video frame; the game ran on
     the real NMI (NTSC 29780.5 cycles = 60.0988 Hz, PAL 33247.5 = 50.007 Hz),
     and 16666 us would play 0.16% slow. */
  if(info->pal ? (info->speed_us >= 19997 && info->speed_us <= 20000)
               : (info->speed_us >= 16639 && info->speed_us <= 16667)) {
    n->period_int = info->pal ? 33247 : 29780;
    n->period_frac = 128;
  }
  n->apu.ctx = n;
  n->cyc = 0;
  n->apu.time = 0;
  n->rendering = 0;
  return 1;
}

static void call(nsf_t *n, uint16_t addr, uint8_t kind)
{
  n->s = 0xff;
  push(n, (NSF_RET - 1) >> 8);
  push(n, (NSF_RET - 1) & 0xff);
  n->pc = addr;
  n->busy = kind;
  n->call_start = n->apu_at = n->cyc;
}

void nsf_start(nsf_t *n, uint8_t song)
{
  unsigned i;
  /* cache: drop everything, zero the RAM part of the backing store; $0000-$01FF
     (pages 0-3 = sectors 0-31) stays resident */
  /* map + vtag (adjacent in nsf_t), vslot + sown (adjacent in the caller's
     memory): all 0xff; vused + vdirty: 0 */
  for(i = offsetof(nsf_t, map); i < offsetof(nsf_t, vtag) + sizeof(n->vtag); i++) ((uint8_t *)n)[i] = 0xff;
  for(i = offsetof(nsf_t, vused); i < offsetof(nsf_t, vdirty) + sizeof(n->vdirty); i++) ((uint8_t *)n)[i] = 0;
  for(i = 0; i < NSF_VPAGES * NSF_SPP + (unsigned)n->nsect; i++) n->vslot[i] = 0xff;
  for(i = 0; i < NSF_PINNED * NSF_LINE; i++) ZP(n)[i] = 0;
  for(i = NSF_BK_RAM; i < NSF_BK_ROM; i += NSF_LINE)
    n->bk(n->bk_ctx, i, ZP(n), NSF_LINE, 1);
  for(i = 0; i < NSF_PINNED * NSF_SPP; i++) n->vslot[i] = n->sown[i] = i;
  for(i = 0; i < NSF_PINNED; i++) {
    n->vtag[i] = (NSF_BK_RAM >> NSF_LINE_SHIFT) + i;
    n->map[i] = i;
  }
  n->vvictim = NSF_PINNED;
  n->pvictim = NSF_PINNED * NSF_SPP;
  n->misses = n->writebacks = 0;
  for(i = 0; i < 8; i++) n->bank[i] = n->info.banks[i];
  /* $FFFA-$FFFF of a flat tune that does not fill them (bank 7 is then zero
     tail): vectors to NSF_RET, as a hardware player points them into its own
     code.  A BRK then ends the call, and a tune that reads them as data (a $FFFF
     "no stream" pointer) gets a non-zero byte as on a cartridge: with zeros,
     Rockman no Constancy's song 3 took it for a command and ran into its data. */
  if(n->info.nbanks >= 8 && n->info.pad + n->info.data_len <= 0x7ffa) {
    uint16_t vec[3] = { NSF_RET, NSF_RET, NSF_RET };   /* little-endian, as the 6502 */
    n->bk(n->bk_ctx, NSF_BK_ROM + 0x7ffa, (uint8_t *)vec, 6, 1);
  }

  n->apu.time = n->cyc;
  nsf_apu_reset(&n->apu, n->info.pal);
  for(i = 0x4000; i < 0x4014; i++) nsf_apu_write(&n->apu, i, 0);
  nsf_apu_write(&n->apu, 0x4015, 0);
  nsf_apu_write(&n->apu, 0x4015, 0x0f);
  nsf_apu_write(&n->apu, 0x4017, 0x40);

  n->err = NSF_OK;
  n->hp_x = n->hp_y = 0;
  n->srem = 0;
  n->a = song ? song - 1 : 0;
  n->x = n->info.pal;
  n->y = 0;
  n->p = 0x24;
  call(n, n->info.init, 1);
}

static int16_t mix(nsf_t *n, uint32_t c)
{
  nsf_apu_t *apu = &n->apu;
  uint32_t pi = ((apu->p[0].acc + apu->p[1].acc) << 4) / c;
  uint32_t ti = ((3 * apu->t.acc + 2 * apu->n.acc + apu->d.acc) << 4) / c;
  int32_t x, y;
  apu->p[0].acc = apu->p[1].acc = apu->t.acc = apu->n.acc = apu->d.acc = 0;
  /* x2: typical tunes then peak around -3 dBFS; the rare full-scale
     moment clips below */
  x = (mix_pulse(pi) + mix_tnd(ti)) << 1;
  /* DC blocker, ~37 Hz one-pole high-pass: y += x - x' - y * (1 - 32595/32768),
     written so y * coefficient never leaves 32 bits */
  y = x - n->hp_x + n->hp_y - ((n->hp_y * 173) >> 15);
  n->hp_x = x;
  n->hp_y = y;
  if(y > 32767) y = 32767;
  if(y < -32768) y = -32768;
  return y;
}

void nsf_render(nsf_t *n, int16_t *out, unsigned frames)
{
  /* a call that runs longer than this without writing the APU is a hang: INIT
     gets 4 s, PLAY 1 s.  A call that keeps writing it is a tune that plays from
     inside INIT or PLAY (a sample fed to $4011 in a timed loop). */
  uint32_t init_max = n->cpu_hz * 4, play_max = n->cpu_hz;
  n->rendering = 1;
  while(frames--) {
    uint32_t c, end;
    n->srem += n->cpu_hz;
    c = n->srem / NSF_RATE;
    n->srem -= c * NSF_RATE;
    end = n->apu.time + c;
    n->lim = end;
    while(!n->err && (int32_t)(end - n->cyc) > 0) {
      if(!n->busy) {
        if((int32_t)(n->next_play - end) >= 0) { n->cyc = end; break; }
        if((int32_t)(n->next_play - n->cyc) > 0) n->cyc = n->next_play;
        n->play_frac += n->period_frac;
        n->next_play += n->period_int + (n->play_frac >> 8);
        n->play_frac &= 0xff;
        if((int32_t)(n->next_play - n->cyc) < 0) n->next_play = n->cyc;
        nsf_age(n);
        call(n, n->info.play, 2);
      }
      cpu_run(n, end, 0);
      if(n->pc == NSF_RET) {
        if(n->busy == 1) { n->next_play = n->cyc; n->play_frac = 0; }
        n->busy = 0;
      }
    }
    if(n->busy) {                   /* checked once a sample, not per instruction */
      uint32_t max = n->busy == 1 ? init_max : play_max;
      if(n->cyc - n->call_start > max && n->cyc - n->apu_at > max) n->err = NSF_ERR_HANG;
    }
    nsf_apu_run(&n->apu, end);
    int16_t s = mix(n, c);
    *out++ = s;
    *out++ = s;
  }
  n->rendering = 0;
}
