/* nes_perf_cli.c -- executes the REAL telemetry routines of the NES renderer
 * (snes/nes/nes_perf.a65, the bytes of misc/nes_snes.bin) in the 65816
 * interpreter, driven by a script on stdin.  The script is written by
 * run_nes_perf.py, which holds the MODEL (what each routine must compute) and
 * compares.
 *
 * Commands (one per line, '#' starts a comment):
 *   boot                     m_reset_memory + load the ROM + nes_boot_init
 *   init                     call nes_pf_init
 *   v N                      scanline $213D reports (9 bits)
 *   p8 SYM[+off] VAL         poke a byte
 *   p16 SYM[+off] VAL        poke a word
 *   g8 SYM[+off]             print "SYM[+off]=VAL"
 *   g16 SYM[+off]
 *   call SYM [a=N] [x=N] [y=N]
 *                            JSR-style call with A/X/Y = N (16 bits).  Prints
 *                            "ret a=.. x=.. y=.. p=.. instr=.."
 *   cyc SYM [a=N] [x=N]      the same call under the master-cycle clock; prints
 *                            "cyc SYM mc=.. cycles=.. instr=.."  (cycles = mc/8:
 *                            the slow-ROM figure the renderer profile uses)
 *   stepcall SYM MAX         start SYM suspended after MAX instructions
 *   inject SYM [a= x= y=]    run SYM to completion ON TOP of the suspended
 *                            call, then restore every register but S (what an
 *                            NMI does on silicon)
 *   resume MAX               continue the suspended call (prints "done" when it
 *                            returned)
 *   rep                      print the 36 report bytes + the published byte
 *   acc                      print the 24 accumulators
 *   sys FRAMES [ev=F:kind]...  INTEGRATION: the REAL poll loop and the REAL NMI
 *                            handler run under the interpreter clock against a
 *                            bridge model (control block, mailbox, V counter);
 *                            see run_sys().  Prints the telemetry the renderer
 *                            kept ("act") and what the ground truth of the run
 *                            says it must have kept ("exp").
 *
 * Addresses come from misc/nes_snes.map; a pair .bin/.map out of sync is
 * refused by the loader below (the map carries the CRC32 of the .bin).
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdarg.h>
#include "m65816.h"

static void die(const char *f, ...) {
  va_list ap; va_start(ap, f);
  fprintf(stderr, "nes_perf_cli: "); vfprintf(stderr, f, ap);
  fprintf(stderr, "\n"); va_end(ap); exit(1);
}

#define MAXSYM 8192
static struct { char name[64]; uint32_t addr; } syms[MAXSYM];
static int nsym;
static uint32_t map_bytes, map_crc;

static void load_map(const char *p) {
  FILE *f = fopen(p, "r"); char line[256];
  if (!f) die("cannot open map %s", p);
  while (fgets(line, sizeof line, f)) {
    unsigned a; char n[64];
    if (line[0] == '#') {
      if (sscanf(line, "# rom_bytes %u", &a) == 1) map_bytes = a;
      else if (sscanf(line, "# rom_crc32 %x", &a) == 1) map_crc = a;
      continue;
    }
    if (sscanf(line, "%x %63s", &a, n) != 2) continue;
    if (nsym >= MAXSYM) die("map has more than %d symbols", MAXSYM);
    syms[nsym].addr = a; snprintf(syms[nsym].name, sizeof syms[nsym].name, "%s", n); nsym++;
  }
  fclose(f);
}

static uint32_t crc32_of(const uint8_t *d, size_t n) {
  uint32_t c = 0xffffffffu; size_t i; int k;
  for (i = 0; i < n; i++) { c ^= d[i]; for (k = 0; k < 8; k++) c = (c >> 1) ^ (0xedb88320u & (0u - (c & 1))); }
  return ~c;
}

/* "name" or "name+N" (N decimal or 0x..) */
static uint32_t sym(const char *spec) {
  char nm[64]; const char *plus = strchr(spec, '+'); long off = 0; int i;
  size_t l = plus ? (size_t)(plus - spec) : strlen(spec);
  if (l >= sizeof nm) die("symbol too long: %s", spec);
  memcpy(nm, spec, l); nm[l] = 0;
  if (plus) off = strtol(plus + 1, NULL, 0);
  for (i = 0; i < nsym; i++) if (!strcmp(syms[i].name, nm)) return syms[i].addr + (uint32_t)off;
  die("symbol missing from the map: %s", nm);
  return 0;
}

static uint8_t *rom; static size_t romsz;

static void set_regs(char **tok, int n) {
  int i;
  m_cpu.p &= (uint8_t)~(M_M | M_X);      /* native, 16-bit A/X/Y */
  m_cpu.dbr = 0; m_cpu.d = 0;
  for (i = 0; i < n; i++) {
    unsigned v;
    if (sscanf(tok[i], "a=%i", &v) == 1) m_cpu.a = (uint16_t)v;
    else if (sscanf(tok[i], "x=%i", &v) == 1) m_cpu.x = (uint16_t)v;
    else if (sscanf(tok[i], "y=%i", &v) == 1) m_cpu.y = (uint16_t)v;
  }
}

static void print_ret(void) {
  printf("ret a=%u x=%u y=%u p=%u instr=%llu\n", m_cpu.a, m_cpu.x, m_cpu.y, m_cpu.p,
         (unsigned long long)m_cpu.instrs);
}


/* ===========================================================================
 * INTEGRATION RUN -- nothing below calls a telemetry routine.  The driver does
 * exactly what the device does: the main loop (`jsr nes_dev_poll_frame` over
 * and over, nes_snes.a65) and the NMI the clock raises at V=225 enter the
 * renderer through ITS call sites, so a probe that is missing, misplaced or fed
 * the wrong register shows up as a counter that disagrees with the ground
 * truth.  The ground truth is observed from the interpreter, never from the
 * renderer's own counters: every V the code latches ($2137 read watch), every
 * interrupt entry and RTI (int hook), the pending/busy bytes at entry, and the
 * entry of each decode probe (pc hook).  The model that turns that ground truth
 * into the accumulators is the spec of utils/nes_perf_decode.py.
 *
 * The bridge: a frame of NOP opcodes (CMD_NOP = $00, 1 B each, closed by FRAME_DONE,
 * so the length is the decode cost) published at line 60 of each frame; the mailbox is served
 * from two 8 KB buffers selected by NES_BUF_SEL.  Events (`ev=F:kind`), at the
 * start of line 224 of frame F (counted from the first closed window):
 *   reent   nes_nmi_busy is raised, so the NMI takes the reentry path
 *   late    the CPU stalls 35 lines: the NMI enters at V=259 (V[8] = 1)
 *   disp    the CPU stalls 45 lines: the NMI enters at V=7 (display active)
 * =========================================================================== */
#define NACC 24
enum { A_DECN, A_H0, A_H1, A_H2, A_H3, A_H4, A_H5, A_SKIP, A_LATE, A_LDISP, A_APPLY, A_E0, A_E1, A_E2,
       A_EMAX, A_NDUR, A_NEND, A_NOVR, A_AGE1, A_AGE2, A_DMAX, A_DSUM, A_DNMI, A_DRN };
#define MAXEV 16
static struct {
  uint32_t a_n0, a_dnmi, a_drain_ret, a_ack, a_pad, a_vis, a_begin, a_mid, a_end, a_samp, a_samp_end, a_guard, a_guard_end, a_nend, a_nend_end, s_busy, s_pend, s_last, s_drain, s_ent, s_nmil, s_wseq,
           s_acc, s_dreent;
  uint8_t  mbx[2][8192];
  int      started, frame, nframes, nev;
  struct { int f, kind, fired; } ev[MAXEV];
  /* model */
  uint32_t macc[NACC]; uint16_t ment, mnmil;
  unsigned nmis, reents, lates, applies, polls_dec, dec_full, errors;
  /* the NMI in flight */
  struct { int active, busy, pend, ng, ne, g, e, nother, sawack, sawpad, sawvis; } cur;
  /* the decode in flight */
  struct { int phase, drained; uint32_t pc; struct { int ent, line, seen; } lat[3]; uint16_t n0, n1, old_last, seq_end; } dec;
  unsigned dec_seen[3];
} G;

static uint16_t g16(uint32_t a) { return m_peek16(a); }
static void gerr(const char *f, ...) {
  va_list ap; va_start(ap, f);
  if (G.errors < 12) { printf("err "); vprintf(f, ap); printf("\n"); }
  va_end(ap); G.errors++;
}
static void gadd(int i, uint32_t v) { G.macc[i] = (G.macc[i] + v) > 0xFFFF ? 0xFFFF : G.macc[i] + v; }
static void gmax(int i, uint32_t v) { if (v >= G.macc[i]) G.macc[i] = v; }
static int mod262(int x) { x %= 262; return x < 0 ? x + 262 : x; }

static void g_latch_cb(uint32_t a24) {
  (void)a24;
  if (m_int_depth() > 0 && G.cur.active) {
    /* WHICH read it is comes from the PC: the late-guard of the apply, the
       latch of nes_pf_nmi_end, or any other (the apply reads V elsewhere too) */
    if (G.dec.pc >= G.a_guard && G.dec.pc < G.a_guard_end) { G.cur.g = m_clock_line(); G.cur.ng++; }
    else if (G.dec.pc >= G.a_nend && G.dec.pc < G.a_nend_end) { G.cur.e = m_clock_line(); G.cur.ne++; }
    else G.cur.nother++;
  } else if (m_int_depth() == 0 && G.dec.phase > 0 && G.dec.pc >= G.a_samp && G.dec.pc < G.a_samp_end) {
    int k = G.dec.phase - 1;
    G.dec.lat[k].ent = G.ment; G.dec.lat[k].line = m_clock_line(); G.dec.lat[k].seen = 1;
  }
}

static void g_wwatch(uint32_t a24) {            /* the `sta nes_pf_n0` of the decode begin */
  if ((a24 & 0xffffff) == G.a_n0) G.dec.n0 = G.mnmil;
}

static void g_pc_hook(uint32_t pc) {
  G.dec.pc = pc;
  /* Position of the probes, from the order the code runs (the ground truth of the
     latches uses the entry PC of each probe, so on its own it is blind to a probe
     moved inside the same flow): the NMI-end probe only after the apply's ack, the
     pad read and the visual-latch block; the mid-decode probe only after the
     mailbox drain returned. */
  if (G.cur.active) {
    if (pc == G.a_ack) G.cur.sawack = 1;
    else if (pc == G.a_pad) G.cur.sawpad = 1;
    else if (pc == G.a_vis) G.cur.sawvis = 1;
    else if (pc == G.a_nend && !G.cur.busy && !(G.cur.sawack && G.cur.sawpad && G.cur.sawvis))
      gerr("nes_pf_nmi_end ran before the NMI work (ack %d, pad %d, visual block %d)",
           G.cur.sawack, G.cur.sawpad, G.cur.sawvis);
  }
  if (pc == G.a_drain_ret) G.dec.drained = 1;
  if (pc == G.a_begin) {
    memset(&G.dec.lat, 0, sizeof G.dec.lat); G.dec.phase = 1; G.dec.drained = 0;
    G.dec.old_last = g16(sym("nes_dev_last_seq")); G.dec_seen[0]++;
  } else if (pc == G.a_dnmi) { G.dec.n1 = G.mnmil;
  } else if (pc == G.a_mid) {
    if (!G.dec.drained) gerr("nes_pf_dec_mid ran before nes_dev_drain_mailbox returned");
    G.dec.phase = 2; G.dec_seen[1]++; }
  else if (pc == G.a_end)   { G.dec.phase = 3; G.dec_seen[2]++;
    G.dec.seq_end = (uint16_t)(m_nesdev_cb(2) | (m_nesdev_cb(3) << 8)); }
}

static void g_finish_nmi(void) {
  int g = -1, e, ens = 0, le, d;
  if (G.cur.busy) {                      /* the reentry path: only the clock ticks */
    G.ment++; G.reents++;
    if (G.cur.ng || G.cur.ne) gerr("reentry NMI latched V (guard %d, end %d)", G.cur.ng, G.cur.ne);
    return;
  }
  if (G.cur.pend != (G.cur.ng == 1)) { gerr("pending=%d but the late-guard latched %d times", G.cur.pend, G.cur.ng); return; }
  if (G.cur.ng > 1 || G.cur.ne != 1) { gerr("NMI latched V: guard %d, end %d (want <=1, 1)", G.cur.ng, G.cur.ne); return; }
  if (G.cur.ng) g = G.cur.g;
  e = G.cur.e;
  if (g >= 0) {
    ens = mod262(g - 225);
    if (g >= 225 && g <= 232) {
      G.applies++;
      gadd(ens < 2 ? A_E0 : ens < 4 ? A_E1 : A_E2, 1);
    } else {
      G.lates++;
      gadd(A_LATE, 1); if (ens >= 37) gadd(A_LDISP, 1);
    }
    gmax(A_EMAX, ens);
  }
  le = mod262(e - 225); d = mod262(le - ens);
  G.mnmil = (uint16_t)(G.mnmil + d);
  gmax(A_NDUR, d); gmax(A_NEND, le);
  if (le >= 38) gadd(A_NOVR, 1);
  G.ment++; G.nmis++;
}

static void g_int_hook(int kind, int v, int h) {
  (void)h; (void)v;
  if (kind == 0) {
    memset(&G.cur, 0, sizeof G.cur);
    G.cur.active = 1; G.cur.busy = m_peek(G.s_busy) != 0; G.cur.pend = m_peek(sym("nes_fb_pending")) != 0;
  } else if (kind == 2 && G.cur.active) {
    G.cur.active = 0;
    if (G.started) g_finish_nmi();
    else if (g16(G.s_wseq) != 0) {          /* the first window just closed: this is the baseline */
      G.started = 1; G.ment = g16(G.s_ent); G.mnmil = g16(G.s_nmil); G.frame = 0;
    }
  }
}

static void g_publish(int len) {
  uint16_t seq = (uint16_t)(m_nesdev_cb(2) | (m_nesdev_cb(3) << 8)) + 1;
  /* the frame ends with the FRAME_DONE opcode ($F0, one byte of check after it): the
     drain of an old buffer walks the commands up to it, and a window of bare NOPs
     would be walked to the end of the mailbox */
  memset(G.mbx[seq & 1], 0, sizeof G.mbx[0]);
  G.mbx[seq & 1][len - 2] = 0xF0;
  m_nesdev_cb_set(8, (uint8_t)len); m_nesdev_cb_set(9, (uint8_t)(len >> 8));
  m_nesdev_cb_set(2, (uint8_t)seq); m_nesdev_cb_set(3, (uint8_t)(seq >> 8));
}

/* Fires event `kind_id` once, at the first frame >= its F that qualifies: the late and
   display-active entries need a frame PENDING (the late-guard only runs then). */
static int g_ev(int kind_id, int f) {
  int i, pend = m_peek(sym("nes_fb_pending")) != 0;
  for (i = 0; i < G.nev; i++)
    if (G.ev[i].kind == kind_id && !G.ev[i].fired && f >= G.ev[i].f && (kind_id == 1 || pend)) {
      G.ev[i].fired = f + 1; return 1;
    }
  return 0;
}
static int g_ev_active(int kind_id) {          /* fired in the current frame */
  int i; for (i = 0; i < G.nev; i++) if (G.ev[i].kind == kind_id && G.ev[i].fired == G.frame + 1) return 1;
  return 0;
}

/* short frames: the drain of an old buffer walks the commands one by one (a NOP is a command),
   and a frame of thousands of them is walked for longer than the bridge takes to overwrite it */
static void g_line_hook(int v) {
  static const int lens[] = {75, 375, 175, 1000, 25, 625, 1500, 225};
  if (v == 0 && G.started) G.frame++;
  if (!G.started) return;
  if (v == 60 || (v == 61 && G.frame % 9 == 4)) g_publish(lens[(G.frame + (v == 61)) % 8]);
  if (v == 224) {
    if (g_ev(1, G.frame)) m_poke(G.s_busy, 1);
    if (g_ev(2, G.frame)) m_clock_stall(35ull * 1364);
    if (g_ev(3, G.frame)) m_clock_stall(45ull * 1364);
  }
  if (v == 228 && g_ev_active(1)) m_poke(G.s_busy, 0);
}

static void g_decode_done(uint16_t new_last) {
  int b = G.dec.lat[0].seen, m = G.dec.lat[1].seen, e = G.dec.lat[2].seen;
  uint32_t wall, drn, skip, age;
  int tb, tm, te;
  G.polls_dec++;
  if (!(b && m && e && G.dec.phase == 3)) {
    gerr("decode #%u: probes begin=%d mid=%d end=%d seen (phase %d)", G.polls_dec, b, m, e, G.dec.phase);
    G.dec.phase = 0; return;
  }
  G.dec_full++;
  tb = G.dec.lat[0].ent * 262 + mod262(G.dec.lat[0].line - 225);
  tm = G.dec.lat[1].ent * 262 + mod262(G.dec.lat[1].line - 225);
  te = G.dec.lat[2].ent * 262 + mod262(G.dec.lat[2].line - 225);
  wall = (uint32_t)(te - tb) & 0xFFFF;
  drn = (G.dec.lat[1].ent == G.dec.lat[0].ent)
        ? (uint32_t)(mod262(G.dec.lat[1].line - 225) - mod262(G.dec.lat[0].line - 225)) & 0xFFFF
        : (uint32_t)(tm - tb) & 0xFFFF;
  gadd(A_DRN, drn); gadd(A_DSUM, wall); gmax(A_DMAX, wall);
  gadd(A_DNMI, (uint16_t)(G.dec.n1 - G.dec.n0));
  age = (uint16_t)(G.dec.seq_end - new_last);     /* frames published after the drained one */
  if (age >= 1) gadd(A_AGE1, 1);
  if (age >= 2) gadd(A_AGE2, 1);
  gadd(wall < 131 ? A_H0 : wall < 262 ? A_H1 : wall < 393 ? A_H2 : wall < 524 ? A_H3 : wall < 786 ? A_H4 : A_H5, 1);
  skip = (uint16_t)(new_last - G.dec.old_last - 1);
  if (skip && skip < 0x8000) gadd(A_SKIP, skip);
  G.dec.phase = 0;
}

static void run_sys(char **tok, int nt) {
  uint32_t poll = sym("nes_dev_poll_frame");
  int i;
  memset(&G, 0, sizeof G);
  G.nframes = nt > 0 ? atoi(tok[0]) : 100;
  for (i = 1; i < nt && G.nev < MAXEV; i++) {
    int f; char k[16];
    if (sscanf(tok[i], "ev=%d:%15s", &f, k) == 2) {
      G.ev[G.nev].f = f;
      G.ev[G.nev].kind = !strcmp(k, "reent") ? 1 : !strcmp(k, "late") ? 2 : !strcmp(k, "disp") ? 3 : 0;
      if (!G.ev[G.nev].kind) die("sys: unknown event %s", k);
      G.nev++;
    }
  }
  G.a_begin = sym("nes_pf_dec_begin"); G.a_mid = sym("nes_pf_dec_mid"); G.a_end = sym("nes_pf_dec_end");
  G.a_samp = sym("pf_samp"); G.a_samp_end = G.a_begin;
  G.a_n0 = sym("nes_pf_n0"); G.a_dnmi = sym("pfde_dm");
  G.a_guard = sym("nav_chk_late"); G.a_guard_end = sym("nav_ok");
  G.a_ack = sym("nes_dev_ack_frame"); G.a_pad = sym("nes_dev_send_pad"); G.a_vis = sym("nmi_no_vis");
  { uint32_t pf = sym("nes_dev_poll_frame"), dr = sym("nes_dev_drain_mailbox"), a;
    G.a_drain_ret = 0;
    for (a = pf; a < pf + 0x300 && !G.a_drain_ret; a++) {
      const uint8_t *b = rom + (a - 0x8000);
      if (b[0] == 0x20 && b[1] == (dr & 0xff) && b[2] == (dr >> 8)) G.a_drain_ret = a + 3;
    }
    if (!G.a_drain_ret) die("sys: nes_dev_poll_frame does not call nes_dev_drain_mailbox");
  }
  G.a_nend = sym("nes_pf_nmi_end"); G.a_nend_end = sym("nes_pf_stream");
  G.s_busy = sym("nes_nmi_busy"); G.s_ent = sym("nes_pf_ent"); G.s_nmil = sym("nes_pf_nmil");
  G.s_wseq = sym("nes_pf_wseq"); G.s_acc = sym("nes_pf_acc"); G.s_dreent = sym("nes_dbg_reent");
  /* THE REAL BOOT: reset, then nes_snes_start up to the instruction after the
     `jsr nes_boot_init` / `jsr nes_pf_init` pair (which must be adjacent: the probe
     block is zeroed and seeded from the counters the boot just cleared).  Its
     zeroing of the gates (WRAM comes up as $55) and the APU unmute (no S-SMP here:
     the handshake times out and the boot goes on, as on a dead APU) run for real.
     What follows in nes_snes_start (CHR upload from PSRAM) the interpreter cannot
     serve; the driver goes on from there. */
  { uint32_t start = sym("nes_snes_start"), bi = sym("nes_boot_init"), pi = sym("nes_pf_init"), a, stop = 0;
    for (a = start; a < start + 0x400; a++) {
      const uint8_t *b = rom + (a - 0x8000);
      if (b[0] == 0x20 && b[1] == (bi & 0xff) && b[2] == (bi >> 8) &&
          b[3] == 0x20 && b[4] == (pi & 0xff) && b[5] == (pi >> 8)) { stop = a + 6; break; }
    }
    if (!stop) die("sys: nes_snes_start does not call nes_pf_init right after nes_boot_init");
    m_reset_memory(); m_load_rom(rom, romsz); m_dbgreg_sink(0x2bdf); m_vcounter = 240;
    m_nesdev_mode(1); m_nesdev_mailbox(G.mbx[0], G.mbx[1]);
    m_cpu.p = (uint8_t)(M_I | M_M); m_cpu.s = 0x1fff; m_cpu.d = 0; m_cpu.dbr = 0;
    m_instr_budget = 400000000ull;
    m_run_to(start, stop);
  }
  m_set_pad(0);
  m_instr_budget = 400000000ull;
  { static const uint32_t lo[1] = {0x2137}, hi[1] = {0x2137};
    m_set_read_watch(1, lo, hi, g_latch_cb); }
  m_set_int_hook(g_int_hook); m_set_line_hook(g_line_hook); m_set_pc_hook(g_pc_hook); m_set_write_watch(g_wwatch);
  m_cpu.p &= (uint8_t)~(M_M | M_X);
  m_bus_write(0x4200, 0x81);                 /* NMI + auto-joypad, as nes_snes_start */
  m_clock_start(100, 262);
  for (;;) {
    uint16_t before = g16(sym("nes_dev_last_seq")), after;
    m_cpu.p &= (uint8_t)~(M_M | M_X);
    m_call(poll, 0);
    after = g16(sym("nes_dev_last_seq"));
    if (G.started && after != before) g_decode_done(after);
    if (G.started && G.frame >= G.nframes) break;
    if (!G.started && m_clock_mc() > 40ull * 1364 * 262) die("sys: no window closed after 40 frames (is nes_pf_nmi_end called by NMI_HANDLER?) (ent=%u wseq=%u busy=%u depth=%d line=%d pc=%06X)",
                                  g16(G.s_ent), g16(G.s_wseq), m_peek(G.s_busy), m_int_depth(), m_clock_line(), G.dec.pc);
  }
  m_clock_stop();
  { uint32_t a = G.s_acc; int k;
    printf("act"); for (k = 0; k < NACC; k++) printf(" %u", g16(a + 2 * k));
    printf("\nexp"); for (k = 0; k < NACC; k++) printf(" %u", G.macc[k]);
    printf("\nsys ent=%u/%u nmil=%u/%u nmis=%u reents=%u applies=%u lates=%u decodes=%u full=%u probes=%u,%u,%u"
           " reent_ctr=%u errors=%u wseq=%u\n",
           g16(G.s_ent), G.ment, g16(G.s_nmil), G.mnmil, G.nmis, G.reents, G.applies, G.lates, G.polls_dec,
           G.dec_full, G.dec_seen[0], G.dec_seen[1], G.dec_seen[2], g16(G.s_dreent), G.errors, g16(G.s_wseq)); }
  m_set_read_watch(0, NULL, NULL, NULL); m_set_int_hook(NULL); m_set_line_hook(NULL); m_set_pc_hook(NULL); m_set_write_watch(NULL);
}

int main(int argc, char **argv) {
  FILE *f; char line[512]; long fsz; int live = 0;
  if (argc < 3) die("usage: nes_perf_cli nes_snes.bin nes_snes.map < script");
  f = fopen(argv[1], "rb"); if (!f) die("cannot open %s", argv[1]);
  fseek(f, 0, SEEK_END); fsz = ftell(f); fseek(f, 0, SEEK_SET); romsz = (size_t)fsz;
  rom = malloc(romsz); if (fread(rom, 1, romsz, f) != romsz) die("read"); fclose(f);
  load_map(argv[2]);
  if (map_bytes != romsz || map_crc != crc32_of(rom, romsz))
    die("%s does not match %s (size/CRC32): rebuild the pair", argv[1], argv[2]);
  m_reset_memory(); m_load_rom(rom, romsz); m_dbgreg_sink(0x2bdf);

  while (fgets(line, sizeof line, stdin)) {
    char *tok[16]; int nt = 0; char *p;
    if (line[0] == '#' || line[0] == '\n') continue;
    for (p = strtok(line, " \t\r\n"); p && nt < 16; p = strtok(NULL, " \t\r\n")) tok[nt++] = p;
    if (!nt) continue;
    if (!strcmp(tok[0], "boot")) {
      m_reset_memory(); m_load_rom(rom, romsz); m_dbgreg_sink(0x2bdf); m_vcounter = 240;
      m_cpu.p &= (uint8_t)~(M_M | M_X); m_cpu.s = 0x1fff;
      m_call(sym("nes_boot_init"), 0);
    } else if (!strcmp(tok[0], "init")) {
      set_regs(tok + 1, nt - 1); m_call(sym("nes_pf_init"), 0);
    } else if (!strcmp(tok[0], "v") && nt == 2) {
      m_vcounter = (uint16_t)strtol(tok[1], NULL, 0);
    } else if (!strcmp(tok[0], "p8") && nt == 3) {
      m_poke(sym(tok[1]), (uint8_t)strtol(tok[2], NULL, 0));
    } else if (!strcmp(tok[0], "p16") && nt == 3) {
      m_poke16(sym(tok[1]), (uint16_t)strtol(tok[2], NULL, 0));
    } else if (!strcmp(tok[0], "g8") && nt == 2) {
      printf("%s=%u\n", tok[1], m_peek(sym(tok[1])));
    } else if (!strcmp(tok[0], "g16") && nt == 2) {
      printf("%s=%u\n", tok[1], m_peek16(sym(tok[1])));
    } else if (!strcmp(tok[0], "call") && nt >= 2) {
      set_regs(tok + 2, nt - 2); m_call(sym(tok[1]), 0); print_ret();
    } else if (!strcmp(tok[0], "cyc") && nt >= 2) {
      uint64_t t0, t1, i0, i1;
      set_regs(tok + 2, nt - 2);
      m_clock_start(100, 262);
      t0 = m_clock_mc(); i0 = m_cpu.instrs;
      m_call(sym(tok[1]), 0);
      t1 = m_clock_mc(); i1 = m_cpu.instrs;
      m_clock_stop();
      printf("cyc %s mc=%llu cycles=%.1f instr=%llu\n", tok[1], (unsigned long long)(t1 - t0),
             (double)(t1 - t0) / 8.0, (unsigned long long)(i1 - i0));
    } else if (!strcmp(tok[0], "stepcall") && nt == 3) {
      set_regs(tok + 3, nt - 3);
      live = m_call_stepped(sym(tok[1]), 0, strtoull(tok[2], NULL, 0));
      printf("stepcall %s\n", live ? "suspended" : "returned");
    } else if (!strcmp(tok[0], "inject") && nt >= 2) {
      m_cpu_t save = m_cpu; uint16_t s = m_cpu.s;
      set_regs(tok + 2, nt - 2); m_call(sym(tok[1]), 0);
      m_cpu = save; m_cpu.s = s;
    } else if (!strcmp(tok[0], "resume") && nt == 2) {
      /* always two lines: the status and a "ret" record (zeros unless done) */
      if (!live) printf("resume none\nret a=0 x=0 y=0 p=0 instr=0\n");
      else if (m_step_resume(strtoull(tok[1], NULL, 0))) printf("resume suspended\nret a=0 x=0 y=0 p=0 instr=0\n");
      else { live = 0; printf("resume done\n"); print_ret(); }
    } else if (!strcmp(tok[0], "rep")) {
      int i; uint32_t b = sym("nes_pf_repw");
      printf("rep");
      for (i = 0; i < 36; i++) printf(" %u", m_peek16(b + 2 * i));
      printf("\npub %u\n", m_peek(sym("nes_pf_pub")));
    } else if (!strcmp(tok[0], "sys")) {
      run_sys(tok + 1, nt - 1);
    } else if (!strcmp(tok[0], "acc")) {
      int i; uint32_t b = sym("nes_pf_acc");
      printf("acc");
      for (i = 0; i < 24; i++) printf(" %u", m_peek16(b + 2 * i));
      printf("\n");
    } else die("unknown command: %s", tok[0]);
  }
  return 0;
}
