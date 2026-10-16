/* Host driver for the NSF player core (src/nsf.c, src/nsfapu.c).
 *
 *   nsf_cli nestest <nestest.nes> <nestest.log>
 *       Runs the CPU from $C000 (automation mode) and compares PC/A/X/Y/P/SP
 *       and the cycle count with the log before every instruction.
 *   nsf_cli apu <test.nes>
 *       Runs one of blargg's apu_test ROMs (NROM, result at $6000) on the core's
 *       CPU + APU, with a minimal $2002 so the shell's vblank waits finish.
 *   nsf_cli render <file.nsf> <song> <seconds> <out.wav|->
 *       Renders a song and prints the backing-store traffic.
 *   nsf_cli info <file.nsf>
 *   nsf_cli tone
 *       Builds tiny NSFs that hold one channel on a fixed period and checks
 *       the output frequency against the APU's period formula.
 *   nsf_cli loops
 *       Builds a tune whose PLAY never returns and feeds $4011 from DEX/BNE
 *       and DEY/BNE delay loops (a sample player): it has to play, not stop
 *       as hung, and its output checksum is printed for run_nsf.sh to compare
 *       with the build that steps the loops (-DNSF_NO_IDLE_SKIP).  Then a PLAY
 *       stuck in a JMP to itself, which has to stop as hung.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "nsf.h"

static uint8_t *bk_mem;
static uint32_t bk_size, bk_reads, bk_writes;

static void bk(void *ctx, uint32_t off, uint8_t *buf, uint16_t len, uint8_t write)
{
  (void)ctx;
  if(off + len > bk_size) {
    fprintf(stderr, "backing access out of range: %06x+%u (size %06x)\n", off, len, bk_size);
    exit(3);
  }
  if(write) { memcpy(bk_mem + off, buf, len); bk_writes++; }
  else { memcpy(buf, bk_mem + off, len); bk_reads++; }
}

/* cache memory handed to the core: NSF_MEM bytes in the environment, default
   2768 (what the AHB SRAM tail gives on the LPC175x boards) */
static uint8_t cli_mem[8192];
static uint32_t cli_memsize(void)
{
  const char *e = getenv("NSF_MEM");
  uint32_t v = e ? (uint32_t)atoi(e) : 2768;
  return v > sizeof(cli_mem) ? sizeof(cli_mem) : v;
}
static void open_or_die(nsf_t *n, const nsf_info_t *info)
{
  if(!nsf_open(n, info, bk, 0, cli_mem, cli_memsize())) {
    fprintf(stderr, "nsf_open refused %u bytes\n", (unsigned)cli_memsize());
    exit(2);
  }
}
static uint8_t *slurp(const char *path, uint32_t *size)
{
  FILE *f = fopen(path, "rb");
  if(!f) { perror(path); exit(2); }
  fseek(f, 0, SEEK_END);
  long n = ftell(f);
  fseek(f, 0, SEEK_SET);
  uint8_t *d = malloc(n ? n : 1);
  if(fread(d, 1, n, f) != (size_t)n) { perror(path); exit(2); }
  fclose(f);
  *size = n;
  return d;
}

/* A NES ROM image (PRG only) as the core's ROM area, banks 0..7 = $8000-$FFFF. */
static void load_nrom(nsf_t *n, const char *path)
{
  uint32_t size;
  uint8_t *d = slurp(path, &size);
  if(size < 16 || memcmp(d, "NES\x1a", 4)) { fprintf(stderr, "%s: not iNES\n", path); exit(2); }
  uint32_t prg = d[4] * 16384;
  nsf_info_t info;
  memset(&info, 0, sizeof info);
  info.nbanks = prg >> 12;
  for(int i = 0; i < 8; i++) info.banks[i] = i % info.nbanks;
  info.speed_us = 16639;
  bk_size = NSF_BK_ROM + prg;
  bk_mem = calloc(1, bk_size);
  memcpy(bk_mem + NSF_BK_ROM, d + 16, prg);
  open_or_die(n, &info);
  nsf_start(n, 1);
  n->busy = 0;
  free(d);
}

static int cmd_nestest(const char *rom, const char *logpath)
{
  static nsf_t n;
  load_nrom(&n, rom);
  n.pc = 0xc000; n.s = 0xfd; n.p = 0x24; n.a = n.x = n.y = 0;
  n.cyc = 7;
  FILE *f = fopen(logpath, "r");
  if(!f) { perror(logpath); return 2; }
  char line[256];
  unsigned ln = 0;
  while(fgets(line, sizeof line, f)) {
    unsigned pc, a, x, y, p, sp, cyc;
    ln++;
    char *q;
    if(sscanf(line, "%4x", &pc) != 1) continue;
    q = strstr(line, "A:");
    if(!q || sscanf(q, "A:%x X:%x Y:%x P:%x SP:%x", &a, &x, &y, &p, &sp) != 5) continue;
    q = strstr(line, "CYC:");
    if(!q || sscanf(q, "CYC:%u", &cyc) != 1) continue;
    if(n.pc != pc || n.a != a || n.x != x || n.y != y || n.p != p || n.s != sp || n.cyc != cyc) {
      printf("nestest: mismatch at log line %u\n  log : %s", ln, line);
      printf("  core: PC:%04X A:%02X X:%02X Y:%02X P:%02X SP:%02X CYC:%u\n",
             n.pc, n.a, n.x, n.y, n.p, n.s, n.cyc);
      return 1;
    }
    n.cyc += nsf_step(&n);
    if(n.err) { printf("nestest: JAM at %04X (line %u)\n", n.pc, ln); return 1; }
  }
  fclose(f);
  printf("nestest: %u lines OK, $02=%02X $03=%02X\n", ln, nsf_peek(&n, 2), nsf_peek(&n, 3));
  return (nsf_peek(&n, 2) || nsf_peek(&n, 3)) ? 1 : 0;
}

/* $2002: vblank flag every 29781 cycles, cleared by the read. */
static uint32_t vbl_last;
static uint8_t io_read(nsf_t *n, uint16_t a)
{
  if((a & 0xe007) == 0x2002) {
    if(n->cyc - vbl_last >= 29781) {
      vbl_last += 29781 * ((n->cyc - vbl_last) / 29781);
      return 0x80;
    }
    return 0;
  }
  return a >> 8;
}

static int cmd_apu(const char *rom)
{
  static nsf_t n;
  load_nrom(&n, rom);
  n.io_read = io_read;
  n.pc = nsf_peek(&n, 0xfffc) | (nsf_peek(&n, 0xfffd) << 8);
  n.s = 0xfd; n.p = 0x24; n.cyc = 0;
  vbl_last = 0;
  int started = 0;
  uint64_t total = 0;
  while(total < 1789773ull * 60) {
    unsigned c = nsf_step(&n);
    n.cyc += c; total += c;
    if(n.err) { printf("%s: JAM at %04X\n", rom, n.pc); return 1; }
    if((total & 0x3ff) < c) {
      uint8_t st = nsf_peek(&n, 0x6000);
      int sig = nsf_peek(&n, 0x6001) == 0xde && nsf_peek(&n, 0x6002) == 0xb0 && nsf_peek(&n, 0x6003) == 0x61;
      if(sig && st == 0x80) started = 1;
      if(sig && started && st < 0x80) {
        char txt[512];
        unsigned i;
        for(i = 0; i < sizeof txt - 1; i++) {
          txt[i] = nsf_peek(&n, 0x6004 + i);
          if(!txt[i]) break;
        }
        txt[i] = 0;
        const char *name = strrchr(rom, '/');
        printf("%s: result %u\n%s", name ? name + 1 : rom, st, txt);
        return st ? 1 : 0;
      }
    }
  }
  printf("%s: timeout, PC=%04X\n", rom, n.pc);
  return 1;
}

static void wav_header(FILE *f, uint32_t frames)
{
  uint32_t data = frames * 4;
  uint8_t h[44];
  memcpy(h, "RIFF....WAVEfmt \x10\0\0\0\x01\0\x02\0....\0\0\0\0\x04\0\x10\0data....", 44);
  uint32_t v;
  v = 36 + data; memcpy(h + 4, &v, 4);
  v = NSF_RATE; memcpy(h + 24, &v, 4);
  v = NSF_RATE * 4; memcpy(h + 28, &v, 4);
  memcpy(h + 40, &data, 4);
  fwrite(h, 1, 44, f);
}

static void load_nsf(nsf_t *n, const char *path)
{
  uint32_t size;
  uint8_t *d = slurp(path, &size);
  nsf_info_t info;
  if(size < NSF_HDR_SIZE || !nsf_parse(&info, d, size)) {
    fprintf(stderr, "%s: not a playable NSF\n", path);
    exit(2);
  }
  bk_size = NSF_BK_ROM + nsf_rom_size(&info);
  bk_mem = calloc(1, bk_size);
  memcpy(bk_mem + NSF_BK_ROM + info.pad, d + NSF_HDR_SIZE, info.data_len);
  open_or_die(n, &info);
  free(d);
}

static int cmd_info(const char *path)
{
  static nsf_t n;
  load_nsf(&n, path);
  nsf_info_t *i = &n.info;
  printf("songs %u start %u load %04X init %04X play %04X speed %u us %s chip %02X\n",
         i->songs, i->start, i->load, i->init, i->play, i->speed_us, i->pal ? "PAL" : "NTSC", i->chip);
  printf("banks %02X %02X %02X %02X %02X %02X %02X %02X  nbanks %u pad %u data %u\n",
         i->banks[0], i->banks[1], i->banks[2], i->banks[3], i->banks[4], i->banks[5],
         i->banks[6], i->banks[7], i->nbanks, i->pad, i->data_len);
  return 0;
}

static int cmd_render(const char *path, int song, double secs, const char *out)
{
  static nsf_t n;
  load_nsf(&n, path);
  uint32_t frames = (uint32_t)(secs * NSF_RATE);
  int16_t buf[2 * 1024];
  FILE *f = strcmp(out, "-") ? fopen(out, "wb") : NULL;
  if(strcmp(out, "-") && !f) { perror(out); return 2; }
  if(f) wav_header(f, frames);
  nsf_start(&n, song);
  bk_reads = bk_writes = 0;
  clock_t t0 = clock();
  uint64_t steps_cyc = 0;
  uint32_t c0 = n.cyc;
  for(uint32_t done = 0; done < frames; ) {
    unsigned k = frames - done > 1024 ? 1024 : frames - done;
    nsf_render(&n, buf, k);
    if(f) fwrite(buf, 4, k, f);
    done += k;
  }
  double wall = (double)(clock() - t0) / CLOCKS_PER_SEC;
  steps_cyc = n.cyc - c0;
  if(f) fclose(f);
  printf("song %d: %.1f s, err %u, sector misses %u (%.1f/s), writebacks %u, wall %.3f s (%.1fx realtime) cyc %llu\n",
         song, secs, n.err, n.misses, n.misses / secs, n.writebacks, wall, secs / wall,
         (unsigned long long)steps_cyc);
  return n.err ? 1 : 0;
}

/* A one-song NSF built here: INIT ($8000) is the given register writes, PLAY
   ($8100) is a bare RTS.  Returns the zero crossings per second of the output
   between 0.25 s and 1.25 s. */
static double tone(const uint8_t (*w)[2], unsigned nw)
{
  static nsf_t n;
  uint8_t img[NSF_HDR_SIZE + 0x200];
  memset(img, 0, sizeof img);
  memcpy(img, "NESM\x1a\x01\x01\x01\x00\x80\x00\x80\x00\x81", 14);
  img[0x6e] = 16666 & 0xff; img[0x6f] = 16666 >> 8;
  uint8_t *c = img + NSF_HDR_SIZE;
  for(unsigned i = 0; i < nw; i++) {
    *c++ = 0xa9; *c++ = w[i][1];                        /* LDA #v */
    *c++ = 0x8d; *c++ = w[i][0]; *c++ = 0x40;           /* STA $40xx */
  }
  *c = 0x60;                                            /* RTS */
  img[NSF_HDR_SIZE + 0x100] = 0x60;
  nsf_info_t info;
  if(!nsf_parse(&info, img, sizeof img)) return -1;
  bk_size = NSF_BK_ROM + nsf_rom_size(&info);
  free(bk_mem);
  bk_mem = calloc(1, bk_size);
  memcpy(bk_mem + NSF_BK_ROM + info.pad, img + NSF_HDR_SIZE, info.data_len);
  open_or_die(&n, &info);
  nsf_start(&n, 1);
  static int16_t buf[2 * NSF_RATE * 5 / 4];
  nsf_render(&n, buf, NSF_RATE * 5 / 4);
  unsigned zc = 0;
  for(unsigned i = NSF_RATE / 4 + 1; i < NSF_RATE * 5 / 4; i++)
    zc += (buf[2 * i] >= 0) != (buf[2 * i - 2] >= 0);
  return n.err ? -1 : zc;
}

static int cmd_tone(void)
{
  /* pulse 1: 50% duty, constant volume 15, negate sweep (no high-period
     mute), period $0FD -> 1789773 / (16 * 254) = 440.40 Hz */
  static const uint8_t pulse[][2] = { {0x00, 0xbf}, {0x01, 0x08}, {0x02, 0xfd}, {0x03, 0x08} };
  /* triangle: control + linear 127, period $07E -> 1789773 / (32 * 127) = 440.40 Hz */
  static const uint8_t tri[][2] = { {0x08, 0xff}, {0x0a, 0x7e}, {0x0b, 0x08} };
  /* pulse 2, 12.5% duty, period $1AB -> 1789773 / (16 * 428) = 261.36 Hz */
  static const uint8_t pulse2[][2] = { {0x04, 0x3f}, {0x05, 0x08}, {0x06, 0xab}, {0x07, 0x09} };
  struct { const char *name; const uint8_t (*w)[2]; unsigned nw; double hz; } t[] = {
    { "pulse1 50%", pulse, 4, 1789773.0 / (16 * 254) },
    { "triangle", tri, 3, 1789773.0 / (32 * 127) },
    { "pulse2 12.5%", pulse2, 4, 1789773.0 / (16 * 428) },
  };
  int fail = 0;
  for(unsigned i = 0; i < sizeof t / sizeof t[0]; i++) {
    double hz = tone(t[i].w, t[i].nw) / 2.0;
    int ok = hz > 0 && hz > t[i].hz * 0.995 && hz < t[i].hz * 1.005;
    printf("tone %-13s %7.2f Hz (want %7.2f) %s\n", t[i].name, hz, t[i].hz, ok ? "ok" : "FAIL");
    fail |= !ok;
  }
  return fail;
}

/* One song, PLAY ($8100) = the code given; INIT a bare RTS.  Renders secs
   seconds; returns the error, the output's checksum in *sum. */
static unsigned play_code(const uint8_t *code, unsigned len, double secs, uint32_t *sum)
{
  static nsf_t n;
  uint8_t img[NSF_HDR_SIZE + 0x300];
  memset(img, 0, sizeof img);
  memcpy(img, "NESM\x1a\x01\x01\x01\x00\x80\x00\x80\x00\x81", 14);
  img[0x6e] = 16666 & 0xff; img[0x6f] = 16666 >> 8;
  img[NSF_HDR_SIZE] = 0x60;                             /* INIT: RTS */
  memcpy(img + NSF_HDR_SIZE + 0x100, code, len);
  for(unsigned i = 0; i < 256; i++)                     /* $8200: a sample, 7-bit */
    img[NSF_HDR_SIZE + 0x200 + i] = (uint8_t)((i * 37) ^ (i >> 2)) & 0x7f;
  nsf_info_t info;
  if(!nsf_parse(&info, img, sizeof img)) return 99;
  bk_size = NSF_BK_ROM + nsf_rom_size(&info);
  free(bk_mem);
  bk_mem = calloc(1, bk_size);
  memcpy(bk_mem + NSF_BK_ROM + info.pad, img + NSF_HDR_SIZE, info.data_len);
  open_or_die(&n, &info);
  nsf_start(&n, 1);
  int16_t buf[2 * 1024];
  uint32_t frames = (uint32_t)(secs * NSF_RATE), h = 2166136261u;
  for(uint32_t done = 0; done < frames; done += 1024) {
    nsf_render(&n, buf, 1024);
    for(unsigned i = 0; i < 2 * 1024; i++) h = (h ^ (uint16_t)buf[i]) * 16777619u;
  }
  *sum = h;
  return n.err;
}

static int cmd_loops(void)
{
  static const uint8_t sample[] = {
    0xa0, 0x00,             /* 8100 LDY #0 */
    0xb9, 0x00, 0x82,       /* 8102 LDA $8200,Y */
    0x8d, 0x11, 0x40,       /* 8105 STA $4011 */
    0xa2, 0x17,             /* 8108 LDX #$17 */
    0xca,                   /* 810a DEX */
    0xd0, 0xfd,             /* 810b BNE $810a */
    0x84, 0x00,             /* 810d STY $00 */
    0xa0, 0x05,             /* 810f LDY #5 */
    0x88,                   /* 8111 DEY */
    0xd0, 0xfd,             /* 8112 BNE $8111 */
    0xa4, 0x00,             /* 8114 LDY $00 */
    0xc8,                   /* 8116 INY */
    0x4c, 0x02, 0x81,       /* 8117 JMP $8102 */
  };
  static const uint8_t hang[] = { 0x4c, 0x00, 0x81 };   /* 8100 JMP $8100 */
  uint32_t sum, hsum;
  unsigned err = play_code(sample, sizeof sample, 3.0, &sum);
  unsigned herr = play_code(hang, sizeof hang, 3.0, &hsum);
  printf("loops: sample err %u sum %08x, hang err %u\n", err, sum, herr);
  return (err != NSF_OK || herr != NSF_ERR_HANG) ? 1 : 0;
}

int main(int argc, char **argv)
{
  if(argc == 2 && !strcmp(argv[1], "loops")) return cmd_loops();
  if(argc == 4 && !strcmp(argv[1], "nestest")) return cmd_nestest(argv[2], argv[3]);
  if(argc == 3 && !strcmp(argv[1], "apu")) return cmd_apu(argv[2]);
  if(argc == 3 && !strcmp(argv[1], "info")) return cmd_info(argv[2]);
  if(argc == 2 && !strcmp(argv[1], "tone")) return cmd_tone();
  if(argc == 6 && !strcmp(argv[1], "render")) return cmd_render(argv[2], atoi(argv[3]), atof(argv[4]), argv[5]);
  fprintf(stderr, "usage: see the comment at the top of nsf_cli.c\n");
  return 2;
}
