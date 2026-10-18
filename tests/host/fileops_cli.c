/* Conformance test for the file manager engine (src/modules/fileops/fopmod.c), the REAL
   module over the REAL FatFs on a RAM disk.  See run_fileops.sh. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "ff.h"
#include "diskio.h"
#include "module.h"
#include "memmap.h"
#include "fileman.h"
#include "scratch.h"

extern const module_fns_t fopmod_fns;
int path_asset(char *buf, int buflen, const char *root, const char *src, const char *ext);
FRESULT path_asset_mkdir(char *path);

/* ---- what ff.c expects from the rest of the firmware ---- */
int sd_offload, ff_sd_offload, sd_offload_tgt, newcard, sd_offload_partial;
uint16_t sd_offload_partial_start, sd_offload_partial_end;
volatile enum diskstates disk_state;
DWORD get_fattime(void) { return ((DWORD)(2026 - 1980) << 25) | ((DWORD)10 << 21) | ((DWORD)17 << 16); }
int dirindex_find(const char *path, uint32_t *clust, uint32_t *size) { (void)path; (void)clust; (void)size; return -1; }

/* ---- the RAM disk ---- */
static uint8_t *disk, *pristine;
static size_t disk_size;
DSTATUS disk_initialize(BYTE d) { (void)d; return 0; }
DSTATUS disk_status(BYTE d) { (void)d; return 0; }
DRESULT disk_read(BYTE d, BYTE *b, DWORD s, UINT c) {
  (void)d;
  if(((size_t)s + c) * 512 > disk_size) return RES_PARERR;
  memcpy(b, disk + (size_t)s * 512, (size_t)c * 512);
  return RES_OK;
}
static uint64_t disk_wr;          /* sectors written, for the bytes_done check in run() */
DRESULT disk_write(BYTE d, const BYTE *b, DWORD s, UINT c) {
  (void)d;
  disk_wr += c;
  if(((size_t)s + c) * 512 > disk_size) return RES_PARERR;
  memcpy(disk + (size_t)s * 512, b, (size_t)c * 512);
  return RES_OK;
}
DRESULT disk_ioctl(BYTE d, BYTE cmd, void *buf) {
  (void)d;
  if(cmd == GET_SECTOR_SIZE) *(WORD *)buf = 512;
  else if(cmd == GET_SECTOR_COUNT) *(DWORD *)buf = (DWORD)(disk_size / 512);
  else if(cmd == GET_BLOCK_SIZE) *(DWORD *)buf = 1;
  return RES_OK;
}

/* ---- the fake module api ---- */
#define PS (0x1000000UL)
static uint8_t *psram;
static void ps_chk(uint32_t a, uint32_t n) {
  if(a > PS || n > PS - a) { fprintf(stderr, "PSRAM access out of range: %06x + %u\n", a, n); abort(); }
}
/* the module writes only its own regions: FM_STAT, and FM_VIEW up to the end of FM_WORK (the
   viewer lines, the walk stack, the copy staging, the window table) */
static void ps_wchk(uint32_t a, uint32_t n) {
  ps_chk(a, n);
  if((a >= SRAM_FM_STAT_ADDR && a + n <= SRAM_FM_STAT_ADDR + 256)
     || (a >= SRAM_FM_VIEW_ADDR && a + n <= SRAM_FM_WORK_END)) return;
  fprintf(stderr, "module write outside its PSRAM: %06x + %u\n", a, n);
  abort();
}
static uint16_t ps_read(void *buf, uint32_t a, uint16_t n) { ps_chk(a, n); memcpy(buf, psram + a, n); return n; }
static uint16_t ps_write(void *buf, uint32_t a, uint16_t n) { ps_wchk(a, n); memcpy(psram + a, buf, n); return n; }
static void ps_memset(uint32_t a, uint32_t n, uint8_t v) { ps_wchk(a, n); memset(psram + a, v, n); }
static uint8_t ps_rb(uint32_t a) { ps_chk(a, 1); return psram[a]; }
static void ps_wb(uint8_t v, uint32_t a) { ps_wchk(a, 1); psram[a] = v; }
static uint16_t ps_rs(uint32_t a) { ps_chk(a, 2); return psram[a] | (psram[a + 1] << 8); }
static void ps_ws(uint16_t v, uint32_t a) { ps_wchk(a, 2); psram[a] = v & 0xff; psram[a + 1] = v >> 8; }

static FIL fh;
static uint32_t tk;
static uint32_t fake_ticks(void) { return tk++ >> 2; }
static size_t spare_size = 3072;
static uint8_t *spare;
static uint8_t *spare_fn(uint32_t *size) { *size = (uint32_t)spare_size; return spare; }
static int refreshes;
static void lists_refresh(void) { refreshes++; }
static int hurries;
static void hurry(void) { hurries++; }
/* the menu font, reduced to what the checks can tell apart: Latin-1 as itself, a few
   stand-ins, '?' for the rest */
static int font_cp(uint32_t cp, uint8_t *out) {
  if(cp == 0x2026) { out[0] = out[1] = out[2] = '.'; return 3; }
  if(cp == 0x20ac) { out[0] = 'E'; return 1; }
  if(cp < 0x100) { out[0] = (uint8_t)cp; return 1; }
  out[0] = '?';
  return 1;
}

static module_api_t api = {
  .sram_read = ps_read, .sram_write = ps_write, .sram_memset = ps_memset,
  .sram_readbyte = ps_rb, .sram_writebyte = ps_wb, .sram_readshort = ps_rs, .sram_writeshort = ps_ws,
  .f_open = f_open, .f_read = f_read, .f_close = f_close, .file = &fh,
  .ticks = fake_ticks, .spare = spare_fn,
  .path_asset = path_asset, .path_asset_mkdir = path_asset_mkdir, .f_write = f_write,
  .f_lseek = f_lseek, .f_unlink = f_unlink, .f_rename = f_rename, .f_mkdir = f_mkdir,
  .f_opendir = f_opendir, .f_readdir = f_readdir, .f_closedir = f_closedir, .f_stat = f_stat,
  .f_getfree = f_getfree, .font_cp = font_cp, .lists_refresh = lists_refresh, .hurry = hurry,
};

/* ---- checks ---- */
static int fails, case_fails;
#define CHECK(c, ...) do { if(!(c)) { fails++; case_fails++; \
  printf("  FAIL line %d: ", __LINE__); printf(__VA_ARGS__); printf("\n"); } } while(0)

/* ---- the card, through FatFs ---- */
static FATFS fs;
static void fresh(void) {
  memcpy(disk, pristine, disk_size);
  memset(psram, 0, PS);
  if(f_mount(&fs, "/", 1) != FR_OK) { fprintf(stderr, "mount failed\n"); exit(1); }
  refreshes = 0;
}

static uint8_t pat(uint32_t i, int seed) { return (uint8_t)(i * 31 + seed * 7 + (i >> 8) + (i >> 16)); }

static void mkd(const char *p) {
  FRESULT r = f_mkdir(p);
  if(r != FR_OK) { fprintf(stderr, "setup: mkdir %s = %d\n", p, r); exit(1); }
}
/* every folder on the way */
static void mkdp(const char *p) {
  char t[300];
  size_t i;
  for(i = 1; p[i]; i++) if(p[i] == '/') { memcpy(t, p, i); t[i] = 0; f_mkdir(t); }
  f_mkdir(p);
}
static void mkbytes(const char *p, const void *d, uint32_t n) {
  FIL f;
  UINT bw;
  char t[300];
  const char *s = strrchr(p, '/');
  if(s && s != p) { memcpy(t, p, s - p); t[s - p] = 0; mkdp(t); }
  if(f_open(&f, p, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) { fprintf(stderr, "setup: create %s\n", p); exit(1); }
  if(n && (f_write(&f, d, n, &bw) != FR_OK || bw != n)) { fprintf(stderr, "setup: write %s\n", p); exit(1); }
  f_close(&f);
}
static void mkf(const char *p, uint32_t n, int seed) {
  uint8_t *d = malloc(n ? n : 1);
  for(uint32_t i = 0; i < n; i++) d[i] = pat(i, seed);
  mkbytes(p, d, n);
  free(d);
}
static long readall(const char *p, uint8_t *d, uint32_t max) {
  FIL f;
  UINT br;
  uint32_t got = 0;
  if(f_open(&f, p, FA_READ) != FR_OK) return -1;
  while(got < max) {
    uint32_t n = max - got > 512 ? 512 : max - got;
    if(f_read(&f, d + got, n, &br) != FR_OK) { f_close(&f); return -1; }
    got += br;
    if(br < n) break;
  }
  f_close(&f);
  return got;
}
/* p holds exactly n bytes of pattern seed */
static int chkf(const char *p, uint32_t n, int seed) {
  uint8_t *d = malloc(n + 1);
  long got = readall(p, d, n + 1);
  int ok = got == (long)n;
  for(uint32_t i = 0; ok && i < n; i++) if(d[i] != pat(i, seed)) ok = 0;
  free(d);
  return ok;
}
static int ex(const char *p) {
  FILINFO f = { 0 };
  return f_stat(p, &f) == FR_OK;
}
static int isdirp(const char *p) {
  FILINFO f = { 0 };
  return f_stat(p, &f) == FR_OK && (f.fattrib & AM_DIR);
}
static DWORD clust(const char *p) {
  FILINFO f = { 0 };
  return f_stat(p, &f) == FR_OK ? f.fclust : 0;
}
static DWORD nfree(void) {
  DWORD n;
  FATFS *f;
  f_getfree("/", &n, &f);
  return n;
}
/* entries below p, recursively */
static int tree_count(const char *p) {
  DIR d;
  FILINFO f;
  char lfn[256], sub[300];
  int n = 0;
  if(f_opendir(&d, p) != FR_OK) return -1;
  for(;;) {
    f.lfname = lfn; f.lfsize = sizeof(lfn);
    if(f_readdir(&d, &f) != FR_OK || !f.fname[0]) break;
    const char *nm = lfn[0] ? lfn : f.fname;
    if(!strcmp(nm, ".") || !strcmp(nm, "..")) continue;
    n++;
    if(f.fattrib & AM_DIR) {
      snprintf(sub, sizeof(sub), "%s/%s", strcmp(p, "/") ? p : "", nm);
      n += tree_count(sub);
    }
  }
  return n;
}
/* the same names (exact case), kinds and bytes below a and b */
static int tree_eq(const char *a, const char *b) {
  DIR d;
  FILINFO f;
  char lfn[256], sa[300], sb[300];
  int n = 0;
  if(f_opendir(&d, a) != FR_OK) return 0;
  for(;;) {
    f.lfname = lfn; f.lfsize = sizeof(lfn);
    if(f_readdir(&d, &f) != FR_OK || !f.fname[0]) break;
    const char *nm = lfn[0] ? lfn : f.fname;
    if(!strcmp(nm, ".") || !strcmp(nm, "..")) continue;
    snprintf(sa, sizeof(sa), "%s/%s", a, nm);
    snprintf(sb, sizeof(sb), "%s/%s", b, nm);
    /* the exact name: find it in b by listing */
    {
      DIR e; FILINFO g; char l2[256]; int found = 0;
      if(f_opendir(&e, b) != FR_OK) return 0;
      for(;;) {
        g.lfname = l2; g.lfsize = sizeof(l2);
        if(f_readdir(&e, &g) != FR_OK || !g.fname[0]) break;
        if(!strcmp(l2[0] ? l2 : g.fname, nm) && ((g.fattrib ^ f.fattrib) & AM_DIR) == 0) { found = 1; break; }
      }
      if(!found) { printf("    missing %s\n", sb); return 0; }
    }
    if(f.fattrib & AM_DIR) {
      if(!tree_eq(sa, sb)) return 0;
    } else {
      uint8_t *x = malloc(f.fsize + 1), *y = malloc(f.fsize + 2);
      long gx = readall(sa, x, f.fsize + 1), gy = readall(sb, y, f.fsize + 2);
      int ok = gx == (long)f.fsize && gy == gx && !memcmp(x, y, f.fsize);
      free(x); free(y);
      if(!ok) { printf("    differs %s\n", sb); return 0; }
    }
    n++;
  }
  return tree_count(a) == tree_count(b);
}
static int has_leaf(const char *dir, const char *exact) {
  DIR d; FILINFO f; char lfn[256];
  if(f_opendir(&d, dir) != FR_OK) return 0;
  for(;;) {
    f.lfname = lfn; f.lfsize = sizeof(lfn);
    if(f_readdir(&d, &f) != FR_OK || !f.fname[0]) return 0;
    if(!strcmp(lfn[0] ? lfn : f.fname, exact)) return 1;
  }
}

/* ---- the clipboard, laid out as fileman.c writes it ---- */
static void clip(uint8_t mode, const char *src, const char *const *names) {
  uint32_t off = SRAM_FM_CLIP_ADDR + FM_CLIP_HDR;
  uint16_t n = 0;
  memset(psram + SRAM_FM_CLIP_ADDR, 0, FM_CLIP_HDR);
  psram[SRAM_FM_CLIP_ADDR] = mode;
  strcpy((char *)psram + SRAM_FM_CLIP_ADDR + 4, src);
  for(; *names; names++, n++) {
    size_t l = strlen(*names);
    psram[off++] = (*names)[l - 1] == '/' ? FM_ENT_DIR : FM_ENT_FILE;   /* folders end in '/' */
    memcpy(psram + off, *names, l + 1);
    off += l + 1;
  }
  psram[SRAM_FM_CLIP_ADDR + 2] = n & 0xff;
  psram[SRAM_FM_CLIP_ADDR + 3] = n >> 8;
}
#define NAMES(...) ((const char *const[]){ __VA_ARGS__, NULL })

/* ---- running the module like the menu would ---- */
typedef struct {
  uint8_t ans[16], all[16];
  int n;                /* answers given */
  uint8_t kind[16];     /* what was asked */
  char name[16][FM_NAME_LEN];
  uint32_t cancel_at;   /* send FM_CTL_CANCEL once bytes_done reaches this (0 = never) */
  int cancel_ask;       /* answer the first question with FM_CTL_CANCEL */
  int cancelled;
} script_t;

static fm_stat_t st;
static void *mst;
static void rd_stat(void) { memcpy(&st, psram + SRAM_FM_STAT_ADDR, sizeof(st)); }

static void end_run(void) {
  if(!mst) return;
  fopmod_fns.stop(mst);
  free(mst); free(spare);
  mst = NULL;
}

static void run(uint8_t op, const char *path, const char *name, script_t *sc) {
  long it;
  uint64_t w0 = disk_wr;
  int ahead = 0;
  end_run();
  memset(psram + SRAM_FM_STAT_ADDR, 0, 256);
  psram[SRAM_FM_STAT_ADDR + 4] = op;
  memset(psram + SRAM_FM_NAME_ADDR, 0, 256);
  if(name) strcpy((char *)psram + SRAM_FM_NAME_ADDR, name);
  mst = malloc(SCRATCH_FRAME_BYTES);
  spare = malloc(spare_size);
  memset(mst, 0x55, SCRATCH_FRAME_BYTES);
  memset(spare, 0xa5, spare_size);
  fopmod_fns.start(mst, &api, path);
  for(it = 0; ; it++) {
    rd_stat();
    if(st.magic[0] != FM_MAGIC0 || st.magic[1] != FM_MAGIC1 || st.version != FM_VERSION) { CHECK(0, "no FM_STAT"); return; }
    /* bytes count as done when they are WRITTEN: never more than what reached the card
       (the last sector of a file may still sit in FatFs' buffer) */
    if(!ahead && st.bytes_done > (disk_wr - w0) * 512 + 511) {
      ahead = 1;
      CHECK(0, "bytes_done %u ahead of the %llu bytes written", st.bytes_done, (unsigned long long)(disk_wr - w0) * 512);
    }
    if(st.state == FM_ST_DONE || st.state == FM_ST_ERROR || st.state == FM_ST_VIEW) return;
    if(it > 4000000) { CHECK(0, "the module never finished (state %d)", st.state); return; }
    if(st.state == FM_ST_ASK && sc && sc->cancel_ask) {
      fopmod_fns.ctl(mst, FM_CTL_CANCEL);
    } else if(st.state == FM_ST_ASK) {
      if(!sc || sc->n >= 16 || !sc->ans[sc->n]) { CHECK(0, "unexpected question (kind %d)", st.ask); fopmod_fns.ctl(mst, FM_CTL_CANCEL); }
      else {
        sc->kind[sc->n] = st.ask;
        strcpy(sc->name[sc->n], (char *)st.cur_name);
        fopmod_fns.ctl(mst, FM_CTL_ANSWER | (sc->ans[sc->n] << 8) | (sc->all[sc->n] << 16));
        sc->n++;
      }
    }
    if(sc && sc->cancel_at && !sc->cancelled && st.bytes_done >= sc->cancel_at) {
      sc->cancelled = 1;
      fopmod_fns.ctl(mst, FM_CTL_CANCEL);
    }
    fopmod_fns.idle(mst);
  }
}

static void paste(uint8_t mode, const char *src, const char *const *names, const char *dst, script_t *sc) {
  clip(mode, src, names);
  run(FM_OP_PASTE, dst, NULL, sc);
}

/* ================================== the cases ================================== */

static void t_copy_single(void) {
  fresh();
  mkd("/src"); mkd("/dst");
  mkf("/src/a.bin", 5000, 1);
  paste(FM_CLIP_COPY, "/src/", NAMES("a.bin"), "/dst/", NULL);
  CHECK(st.state == FM_ST_DONE, "state %d err %d", st.state, st.err);
  CHECK(st.n_done == 1 && st.n_failed == 0 && st.n_skipped == 0, "counts %d/%d/%d", st.n_done, st.n_skipped, st.n_failed);
  CHECK(st.bytes_total == 5000 && st.bytes_done == 5000, "bytes %u/%u", st.bytes_done, st.bytes_total);
  CHECK(st.items_total == 1 && st.items_done == 1 && st.progress == 255, "items %d/%d", st.items_done, st.items_total);
  CHECK(chkf("/dst/a.bin", 5000, 1), "copy differs");
  CHECK(chkf("/src/a.bin", 5000, 1), "source changed");
  CHECK(refreshes == 0, "a copy touched the lists");
}

static void t_copy_tree(void) {
  char lvl[300], f[300], longn[120];
  static const uint32_t sizes[] = { 0, 1, 511, 512, 513, 1500, 4096, 9000 };
  int depth, i;
  fresh();
  mkd("/dst");
  memset(longn, 0, sizeof(longn));
  for(i = 0; i < 110; i++) longn[i] = 'a' + i % 26;
  memcpy(longn, "Long Folder ", 12);
  snprintf(lvl, sizeof(lvl), "/src/T");
  mkdp(lvl);
  for(depth = 0; depth < 8; depth++) {
    for(i = 0; i < 8; i++) {
      snprintf(f, sizeof(f), "%s/file %d of level %d.dat", lvl, i, depth);
      mkf(f, sizes[i] + depth * 37, depth * 10 + i);
    }
    if(depth == 2) {
      snprintf(f, sizeof(f), "%s/big.bin", lvl);
      mkf(f, 70000, 99);
      snprintf(f, sizeof(f), "%s/many", lvl);
      mkd(f);
      for(i = 0; i < 70; i++) { snprintf(f, sizeof(f), "%s/many/m%02d.txt", lvl, i); mkf(f, i * 13, i); }
    }
    snprintf(lvl + strlen(lvl), sizeof(lvl) - strlen(lvl), "/%s", depth == 0 ? longn : "d");
    mkd(lvl);
  }
  /* a long leaf at the bottom: the destination path is the same length, it still fits */
  snprintf(f, sizeof(f), "%s/%.*s.end", lvl, 90, longn);
  mkf(f, 300, 3);
  int n = tree_count("/src/T");
  paste(FM_CLIP_COPY, "/src", NAMES("T/"), "/dst", NULL);
  CHECK(st.state == FM_ST_DONE && st.n_failed == 0, "state %d err %d failed %d", st.state, st.err, st.n_failed);
  CHECK(st.items_total == n + 1 && st.items_done == n + 1 && st.n_done == n + 1, "items %d/%d, tree %d", st.items_done, st.items_total, n + 1);
  CHECK(st.bytes_done == st.bytes_total, "bytes %u/%u", st.bytes_done, st.bytes_total);
  CHECK(tree_eq("/src/T", "/dst/T"), "the copied tree differs");
}

static void t_copy_same_folder(void) {
  fresh();
  mkd("/src");
  mkf("/src/a.bin", 700, 1);
  mkf("/src/D/x.txt", 30, 2);
  mkf("/src/noext", 10, 3);
  mkf("/src/.cfg", 10, 4);
  paste(FM_CLIP_COPY, "/src", NAMES("a.bin", "D/", "noext", ".cfg"), "/src", NULL);
  CHECK(st.state == FM_ST_DONE && st.n_done == 5 && st.n_failed == 0, "state %d done %d failed %d", st.state, st.n_done, st.n_failed);
  CHECK(chkf("/src/a (2).bin", 700, 1) && chkf("/src/D (2)/x.txt", 30, 2), "keep-both copies");
  CHECK(chkf("/src/noext (2)", 10, 3) && chkf("/src/.cfg (2)", 10, 4), "names without an extension");
  paste(FM_CLIP_COPY, "/src", NAMES("a.bin", "D/"), "/src", NULL);
  CHECK(st.state == FM_ST_DONE && chkf("/src/a (3).bin", 700, 1) && chkf("/src/D (3)/x.txt", 30, 2), "the next free number");
  /* a cut into its own folder does nothing */
  int n = tree_count("/src");
  paste(FM_CLIP_CUT, "/src", NAMES("a.bin", "D/"), "/src", NULL);
  CHECK(st.state == FM_ST_DONE && st.n_done == 2 && tree_count("/src") == n && chkf("/src/a.bin", 700, 1), "same-folder cut");
}

static void t_cut_is_rename(void) {
  fresh();
  mkd("/dst");
  mkf("/src/big.bin", 300000, 5);
  mkf("/src/F/in.bin", 2000, 6);
  DWORD cb = clust("/src/big.bin"), cf = clust("/src/F"), fr = nfree();
  paste(FM_CLIP_CUT, "/src", NAMES("big.bin", "F/"), "/dst", NULL);
  CHECK(st.state == FM_ST_DONE && st.n_done == 2, "state %d done %d err %d", st.state, st.n_done, st.err);
  CHECK(!ex("/src/big.bin") && !ex("/src/F"), "the sources are still there");
  CHECK(clust("/dst/big.bin") == cb && clust("/dst/F") == cf, "moved by copying (clusters %u/%u)", clust("/dst/big.bin"), cb);
  CHECK(chkf("/dst/big.bin", 300000, 5) && chkf("/dst/F/in.bin", 2000, 6), "contents");
  CHECK(nfree() == fr && st.bytes_total == 0, "a cut used space");
}

static void t_into_self(void) {
  fresh();
  mkf("/src/D/sub/x.bin", 100, 1);
  mkd("/src/Dx");
  int n = tree_count("/");
  DWORD fr = nfree();
  paste(FM_CLIP_COPY, "/src", NAMES("D/"), "/src/D", NULL);
  CHECK(st.state == FM_ST_ERROR && st.err == FM_ERR_INTO_SELF, "into itself: state %d err %d", st.state, st.err);
  paste(FM_CLIP_COPY, "/src", NAMES("D/"), "/SRC/d/SUB", NULL);
  CHECK(st.state == FM_ST_ERROR && st.err == FM_ERR_INTO_SELF, "below itself: state %d err %d", st.state, st.err);
  paste(FM_CLIP_CUT, "/src", NAMES("D/"), "/src/D/sub/", NULL);
  CHECK(st.state == FM_ST_ERROR && st.err == FM_ERR_INTO_SELF, "cut below itself: state %d err %d", st.state, st.err);
  CHECK(tree_count("/") == n && nfree() == fr, "something was touched");
  paste(FM_CLIP_COPY, "/src", NAMES("D/"), "/src/Dx", NULL);
  CHECK(st.state == FM_ST_DONE && chkf("/src/Dx/D/sub/x.bin", 100, 1), "a sibling with a longer name is not 'inside'");
}

static void conflict_setup(void) {
  fresh();
  mkf("/src/a.bin", 1000, 1); mkf("/src/b.bin", 1000, 2); mkf("/src/c.bin", 1000, 3);
  mkf("/dst/a.bin", 500, 9); mkf("/dst/b.bin", 500, 8); mkf("/dst/c.bin", 500, 7);
}

static void t_conflict_file(void) {
  script_t sc;
  conflict_setup();
  memset(&sc, 0, sizeof(sc)); sc.ans[0] = FM_ANS_REPLACE;
  paste(FM_CLIP_COPY, "/src", NAMES("a.bin"), "/dst", &sc);
  CHECK(st.state == FM_ST_DONE && sc.n == 1 && sc.kind[0] == FM_ASK_FILE && !strcmp(sc.name[0], "a.bin"), "asked %d kind %d '%s'", sc.n, sc.kind[0], sc.name[0]);
  CHECK(chkf("/dst/a.bin", 1000, 1) && st.n_done == 1, "replace");

  conflict_setup();
  memset(&sc, 0, sizeof(sc)); sc.ans[0] = FM_ANS_SKIP;
  paste(FM_CLIP_COPY, "/src", NAMES("a.bin"), "/dst", &sc);
  CHECK(st.state == FM_ST_DONE && st.n_skipped == 1 && st.n_done == 0 && chkf("/dst/a.bin", 500, 9), "skip");

  conflict_setup();
  memset(&sc, 0, sizeof(sc)); sc.ans[0] = FM_ANS_KEEPBOTH;
  paste(FM_CLIP_COPY, "/src", NAMES("a.bin"), "/dst", &sc);
  CHECK(st.state == FM_ST_DONE && chkf("/dst/a (2).bin", 1000, 1) && chkf("/dst/a.bin", 500, 9), "keep both");

  /* the same answer for all: one question for three conflicts */
  conflict_setup();
  memset(&sc, 0, sizeof(sc)); sc.ans[0] = FM_ANS_REPLACE; sc.all[0] = 1;
  paste(FM_CLIP_COPY, "/src", NAMES("a.bin", "b.bin", "c.bin"), "/dst", &sc);
  CHECK(st.state == FM_ST_DONE && sc.n == 1 && st.n_done == 3, "asked %d done %d", sc.n, st.n_done);
  CHECK(chkf("/dst/a.bin", 1000, 1) && chkf("/dst/b.bin", 1000, 2) && chkf("/dst/c.bin", 1000, 3), "replace all");

  /* ...only for the same kind of conflict: a folder still asks */
  conflict_setup();
  mkf("/src/M/new.txt", 10, 4); mkd("/dst/M");
  memset(&sc, 0, sizeof(sc)); sc.ans[0] = FM_ANS_SKIP; sc.all[0] = 1; sc.ans[1] = FM_ANS_REPLACE;
  paste(FM_CLIP_COPY, "/src", NAMES("a.bin", "M/", "b.bin"), "/dst", &sc);
  CHECK(st.state == FM_ST_DONE && sc.n == 2 && sc.kind[0] == FM_ASK_FILE && sc.kind[1] == FM_ASK_DIR, "asked %d (%d %d)", sc.n, sc.kind[0], sc.kind[1]);
  CHECK(chkf("/dst/a.bin", 500, 9) && chkf("/dst/b.bin", 500, 8) && chkf("/dst/M/new.txt", 10, 4), "skip all + merge");

  /* a cut that replaces: the old file goes, the moved one keeps its clusters */
  conflict_setup();
  DWORD ca = clust("/src/a.bin");
  memset(&sc, 0, sizeof(sc)); sc.ans[0] = FM_ANS_REPLACE;
  paste(FM_CLIP_CUT, "/src", NAMES("a.bin"), "/dst", &sc);
  CHECK(st.state == FM_ST_DONE && !ex("/src/a.bin") && clust("/dst/a.bin") == ca && chkf("/dst/a.bin", 1000, 1), "cut replace");

  /* cancel while asked */
  conflict_setup();
  memset(&sc, 0, sizeof(sc)); sc.cancel_ask = 1;
  paste(FM_CLIP_COPY, "/src", NAMES("a.bin", "b.bin"), "/dst", &sc);
  CHECK(st.state == FM_ST_ERROR && st.err == FM_ERR_CANCELLED && chkf("/dst/a.bin", 500, 9) && chkf("/dst/b.bin", 500, 8),
        "cancel at a question: %d %d", st.state, st.err);
}

static void merge_setup(void) {
  fresh();
  mkf("/src/M/x", 100, 1); mkf("/src/M/y", 100, 2); mkf("/src/M/sub/z", 100, 3);
  mkf("/dst/M/x", 50, 9); mkf("/dst/M/w", 50, 8);
}

static void t_merge(void) {
  script_t sc;
  merge_setup();
  memset(&sc, 0, sizeof(sc)); sc.ans[0] = FM_ANS_REPLACE; sc.ans[1] = FM_ANS_SKIP;
  paste(FM_CLIP_COPY, "/src", NAMES("M/"), "/dst", &sc);
  CHECK(st.state == FM_ST_DONE && sc.n == 2 && sc.kind[0] == FM_ASK_DIR && sc.kind[1] == FM_ASK_FILE, "copy merge asked %d", sc.n);
  CHECK(chkf("/dst/M/x", 50, 9) && chkf("/dst/M/y", 100, 2) && chkf("/dst/M/w", 50, 8) && chkf("/dst/M/sub/z", 100, 3), "copy merge result");

  merge_setup();
  mkf("/dst/M/sub/q", 10, 5);
  memset(&sc, 0, sizeof(sc)); sc.ans[0] = FM_ANS_REPLACE; sc.ans[1] = FM_ANS_REPLACE; sc.ans[2] = FM_ANS_REPLACE;
  paste(FM_CLIP_CUT, "/src", NAMES("M/"), "/dst", &sc);
  CHECK(st.state == FM_ST_DONE && st.n_failed == 0, "cut merge state %d failed %d", st.state, st.n_failed);
  CHECK(chkf("/dst/M/x", 100, 1) && chkf("/dst/M/y", 100, 2) && chkf("/dst/M/w", 50, 8)
        && chkf("/dst/M/sub/z", 100, 3) && chkf("/dst/M/sub/q", 10, 5), "cut merge result");
  CHECK(!ex("/src/M"), "the emptied source folder is still there");

  merge_setup();
  memset(&sc, 0, sizeof(sc)); sc.ans[0] = FM_ANS_REPLACE; sc.ans[1] = FM_ANS_SKIP;
  paste(FM_CLIP_CUT, "/src", NAMES("M/"), "/dst", &sc);
  CHECK(st.state == FM_ST_DONE && chkf("/src/M/x", 100, 1) && chkf("/dst/M/x", 50, 9) && !ex("/src/M/y") && chkf("/dst/M/sub/z", 100, 3),
        "cut merge with a skip keeps the source folder and what was skipped");
}

static void t_mixed(void) {
  script_t sc;
  fresh();
  mkf("/src/f", 100, 1); mkf("/dst/f/inside", 10, 2);
  mkf("/src/g/in", 10, 3); mkf("/dst/g", 20, 4);
  memset(&sc, 0, sizeof(sc)); sc.ans[0] = FM_ANS_REPLACE; sc.ans[1] = FM_ANS_REPLACE;
  paste(FM_CLIP_COPY, "/src", NAMES("f", "g/"), "/dst", &sc);
  CHECK(st.state == FM_ST_DONE && sc.n == 2 && sc.kind[0] == FM_ASK_MIXED && sc.kind[1] == FM_ASK_MIXED, "asked %d", sc.n);
  CHECK(st.n_skipped == 2 && isdirp("/dst/f") && chkf("/dst/f/inside", 10, 2) && chkf("/dst/g", 20, 4), "replace over the other kind must skip");
  memset(&sc, 0, sizeof(sc)); sc.ans[0] = FM_ANS_KEEPBOTH; sc.ans[1] = FM_ANS_KEEPBOTH;
  paste(FM_CLIP_COPY, "/src", NAMES("f", "g/"), "/dst", &sc);
  CHECK(st.state == FM_ST_DONE && chkf("/dst/f (2)", 100, 1) && chkf("/dst/g (2)/in", 10, 3), "keep both over the other kind");
  memset(&sc, 0, sizeof(sc)); sc.ans[0] = FM_ANS_REPLACE;
  paste(FM_CLIP_CUT, "/src", NAMES("f"), "/dst", &sc);
  CHECK(st.state == FM_ST_DONE && st.n_skipped == 1 && chkf("/src/f", 100, 1) && isdirp("/dst/f"), "cut mixed replace skips");
}

static void t_nospace(void) {
  char p[64];
  int i;
  fresh();
  for(i = 0; i < 60; i++) { snprintf(p, sizeof(p), "/src/s%02d.txt", i); mkf(p, 10, i); }
  mkf("/src/big.bin", 100000, 7);
  mkd("/dst");
  DWORD f0 = nfree();
  CHECK(fs.csize == 2, "the image should have 1 KB clusters");
  mkf("/filler", (f0 - 50) * 1024, 0);
  DWORD fr = nfree();
  CHECK(fr >= 48 && fr <= 50, "filler left %u clusters", fr);
  /* 60 files of 10 bytes: 600 bytes, 60 clusters */
  static const char *sixty[61];
  static char names[60][16];
  for(i = 0; i < 60; i++) { snprintf(names[i], 16, "s%02d.txt", i); sixty[i] = names[i]; }
  sixty[60] = NULL;
  int n = tree_count("/dst");
  clip(FM_CLIP_COPY, "/src", sixty);
  run(FM_OP_PASTE, "/dst", NULL, NULL);
  CHECK(st.state == FM_ST_ERROR && st.err == FM_ERR_NOSPACE, "60 one-cluster files into %u clusters: state %d err %d", fr, st.state, st.err);
  CHECK(tree_count("/dst") == n && nfree() == fr, "a refused copy wrote something");
  paste(FM_CLIP_COPY, "/src", NAMES("big.bin"), "/dst", NULL);
  CHECK(st.state == FM_ST_ERROR && st.err == FM_ERR_NOSPACE && !ex("/dst/big.bin"), "100 KB into 50 KB: state %d err %d", st.state, st.err);
  sixty[40] = NULL;
  clip(FM_CLIP_COPY, "/src", sixty);
  run(FM_OP_PASTE, "/dst", NULL, NULL);
  CHECK(st.state == FM_ST_DONE && st.n_done == 40 && chkf("/dst/s39.txt", 10, 39), "40 files fit: state %d err %d done %d", st.state, st.err, st.n_done);
}

static void t_cancel(void) {
  script_t sc;
  fresh();
  mkf("/src/a.small", 1000, 1);
  mkf("/src/z.large", 8u << 20, 2);
  mkd("/dst");
  DWORD fr = nfree();
  memset(&sc, 0, sizeof(sc)); sc.cancel_at = 2u << 20;
  paste(FM_CLIP_COPY, "/src", NAMES("a.small", "z.large"), "/dst", &sc);
  CHECK(st.state == FM_ST_ERROR && st.err == FM_ERR_CANCELLED, "state %d err %d", st.state, st.err);
  CHECK(st.bytes_done > (2u << 20) && st.bytes_done < (8u << 20), "cancelled at %u", st.bytes_done);
  CHECK(chkf("/dst/a.small", 1000, 1) && st.n_done == 1, "the finished file is kept");
  CHECK(!ex("/dst/z.large") && nfree() == fr - 1, "the half-copied file is left (free %u, was %u)", nfree(), fr);
}

/* Files around the staging's run length: the module fills up to 64 KB in whole buffers
   (bufsz = what the spare leaves after the five path buffers, in sectors, at most 4 KB),
   then writes it all out. */
static void t_copy_staging(void) {
  uint32_t bufsz = (uint32_t)((spare_size - 5 * 256) & ~511UL), run, sizes[8], i;
  char p[64];
  script_t sc;
  if(bufsz > 4096) bufsz = 4096;
  run = (0x10000 / bufsz) * bufsz;
  sizes[0] = 200000; sizes[1] = 2 * 0x10000; sizes[2] = 0x10000 + 1; sizes[3] = run;
  sizes[4] = 3 * run; sizes[5] = run + 1; sizes[6] = run - 1; sizes[7] = 5 * bufsz;
  fresh();
  mkd("/dst");
  for(i = 0; i < 8; i++) { snprintf(p, sizeof(p), "/src/f%u.bin", i); mkf(p, sizes[i], 40 + i); }
  paste(FM_CLIP_COPY, "/src", NAMES("f0.bin", "f1.bin", "f2.bin", "f3.bin", "f4.bin", "f5.bin", "f6.bin", "f7.bin"), "/dst", NULL);
  CHECK(st.state == FM_ST_DONE && st.n_done == 8 && st.n_failed == 0, "state %d done %d err %d", st.state, st.n_done, st.err);
  CHECK(st.bytes_done == st.bytes_total, "bytes %u/%u", st.bytes_done, st.bytes_total);
  for(i = 0; i < 8; i++) {
    snprintf(p, sizeof(p), "/dst/f%u.bin", i);
    CHECK(chkf(p, sizes[i], 40 + i), "%u bytes (run %u, buffer %u) differ", sizes[i], run, bufsz);
  }
  /* cancelled while the second run is being written out */
  fresh();
  mkf("/src/z.bin", 4 * run, 9);
  mkd("/dst");
  DWORD fr = nfree();
  memset(&sc, 0, sizeof(sc)); sc.cancel_at = run + bufsz;
  paste(FM_CLIP_COPY, "/src", NAMES("z.bin"), "/dst", &sc);
  CHECK(st.state == FM_ST_ERROR && st.err == FM_ERR_CANCELLED, "state %d err %d", st.state, st.err);
  CHECK(st.bytes_done > run && st.bytes_done < 2 * run, "cancelled at %u, not inside the second run's drain", st.bytes_done);
  CHECK(!ex("/dst/z.bin") && nfree() == fr, "the half-written file is left (free %u, was %u)", nfree(), fr);
  /* a sidecar larger than a run goes through the same staging (copy_whole) */
  fresh();
  mkf("/games/Big.sfc", 100, 1);
  mkf("/sd2snes/info/BI/Big.fmv", 3 * run + 7, 2);
  paste(FM_CLIP_COPY, "/games", NAMES("Big.sfc"), "/games", NULL);
  CHECK(st.state == FM_ST_DONE && chkf("/sd2snes/info/BI/Big (2).fmv", 3 * run + 7, 2) && st.bytes_done == 100 + 3 * run + 7,
        "a big sidecar copy: state %d bytes %u", st.state, st.bytes_done);
}

static void t_rename(void) {
  static const char *const bad[] = { "", "a/b", "a\\b", "a:b", "a*b", "a?b", "a\"b", "a<b", "a>b", "a|b",
                                     "...", "  ", ". .", "\x01x", "x\x7f", NULL };
  int i;
  fresh();
  mkf("/r/a.txt", 100, 1);
  mkf("/r/D/x", 10, 2);
  run(FM_OP_RENAME, "/r/a.txt", "b.txt", NULL);
  CHECK(st.state == FM_ST_DONE && !ex("/r/a.txt") && chkf("/r/b.txt", 100, 1), "file: state %d err %d", st.state, st.err);
  CHECK(!strcmp((char *)st.cur_name, "b.txt"), "cur_name '%s'", st.cur_name);
  run(FM_OP_RENAME, "/r/D/", "E", NULL);
  CHECK(st.state == FM_ST_DONE && !ex("/r/D") && chkf("/r/E/x", 10, 2), "folder: state %d err %d", st.state, st.err);
  mkf("/r/c.txt", 50, 3);
  run(FM_OP_RENAME, "/r/c.txt", "b.txt", NULL);
  CHECK(st.state == FM_ST_ERROR && st.err == FM_ERR_EXISTS && chkf("/r/c.txt", 50, 3), "taken: state %d err %d", st.state, st.err);
  run(FM_OP_RENAME, "/r/c.txt", "B.TXT", NULL);
  CHECK(st.state == FM_ST_ERROR && st.err == FM_ERR_EXISTS && chkf("/r/b.txt", 100, 1), "taken, other case: state %d err %d", st.state, st.err);
  for(i = 0; bad[i]; i++) {
    run(FM_OP_RENAME, "/r/c.txt", bad[i], NULL);
    CHECK(st.state == FM_ST_ERROR && st.err == FM_ERR_BADNAME, "bad name #%d: state %d err %d", i, st.state, st.err);
  }
  CHECK(chkf("/r/c.txt", 50, 3) && tree_count("/r") == 4, "a refused rename touched the card");
  /* only the case changes */
  mkf("/r/game.txt", 70, 4);
  run(FM_OP_RENAME, "/r/game.txt", "GAME.txt", NULL);
  CHECK(st.state == FM_ST_DONE && has_leaf("/r", "GAME.txt") && !has_leaf("/r", "game.txt") && chkf("/r/GAME.txt", 70, 4),
        "case-only rename: state %d err %d", st.state, st.err);
  CHECK(tree_count("/r") == 5, "a temporary name was left behind");
  run(FM_OP_RENAME, "/r/b.txt", "b.txt", NULL);
  CHECK(st.state == FM_ST_DONE && chkf("/r/b.txt", 100, 1), "the same name");
  run(FM_OP_RENAME, "/r/b.txt", "caf\xe9 \x80.txt", NULL);
  CHECK(st.state == FM_ST_DONE && chkf("/r/caf\xe9 \x80.txt", 100, 1), "CP1252 name: state %d err %d", st.state, st.err);
  CHECK(!strcmp((char *)st.cur_name, "caf\xe9 E.txt"), "cur_name in font bytes '%s'", st.cur_name);
  /* the new path would not fit */
  char deep[300], nm[80];
  memset(nm, 'n', 70); nm[70] = 0;
  snprintf(deep, sizeof(deep), "/r/%.*s", 200, "dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd");
  mkd(deep);
  strcat(deep, "/f");
  mkf(deep, 1, 1);
  run(FM_OP_RENAME, deep, nm, NULL);
  CHECK(st.state == FM_ST_ERROR && st.err == FM_ERR_PATH, "too long: state %d err %d", st.state, st.err);
  run(FM_OP_RENAME, "/r/none.txt", "x.txt", NULL);
  CHECK(st.state == FM_ST_ERROR && st.err == FM_ERR_NOTFOUND, "missing: state %d err %d", st.state, st.err);
}

static void t_mkdir(void) {
  fresh();
  mkd("/r");
  run(FM_OP_MKDIR, "/r/", "New", NULL);
  CHECK(st.state == FM_ST_DONE && isdirp("/r/New"), "mkdir: state %d err %d", st.state, st.err);
  run(FM_OP_MKDIR, "/r", "new", NULL);
  CHECK(st.state == FM_ST_ERROR && st.err == FM_ERR_EXISTS, "taken: state %d err %d", st.state, st.err);
  run(FM_OP_MKDIR, "/r", "a:b", NULL);
  CHECK(st.state == FM_ST_ERROR && st.err == FM_ERR_BADNAME, "bad: state %d err %d", st.state, st.err);
  run(FM_OP_MKDIR, "/", "Top", NULL);
  CHECK(st.state == FM_ST_DONE && isdirp("/Top"), "at the root: state %d err %d", st.state, st.err);
}

/* a game with its whole family on the card */
static void game_setup(void) {
  fresh();
  mkf("/games/Super Game.sfc", 4096, 1);
  mkf("/sd2snes/saves/SU/Super Game.srm", 8192, 2);
  mkf("/sd2snes/saves/SU/Super Game.slot", 2, 3);
  mkf("/sd2snes/saves/SU/Super Game.03.srm", 100, 4);
  mkf("/sd2snes/states/SU/Super Game01.state", 3000, 5);
  mkf("/sd2snes/states/SU/Super Game12.state", 3000, 6);
  mkf("/sd2snes/info/SU/Super Game.yml", 300, 7);
  mkf("/sd2snes/info/SU/Super Game.gcv", 900, 8);
  mkf("/sd2snes/info/SU/Super Game.01.man", 500, 9);
  mkf("/sd2snes/cheats/SU/Super Game.yml", 200, 10);
  mkf("/sd2snes/patches/SU/Super Game.yml", 50, 11);
  /* not this game's */
  mkf("/sd2snes/saves/SU/Super Game Two.srm", 10, 12);
  mkf("/sd2snes/saves/SU/Super Gamex.srm", 10, 13);
  mkf("/sd2snes/states/SU/Super Game1.state", 10, 14);
  mkf("/sd2snes/info/SU/Super Game.toolong", 10, 15);
  /* next to it */
  mkf("/games/Super Game.cov", 2048, 16);
  mkf("/games/Super Game.msu", 0, 0);
  mkf("/games/Super Game-1.pcm", 5000, 17);
  mkf("/games/Super Game-12.pcm", 5000, 18);
  mkf("/games/Super Game-x.pcm", 10, 19);
  mkf("/games/Super Game Two.cov", 10, 20);
}

static void t_rom_rename(void) {
  game_setup();
  mkf("/sd2snes/saves/ME/Mega Game.slot", 2, 77);
  run(FM_OP_RENAME, "/games/Super Game.sfc", "Mega Game.sfc", NULL);
  CHECK(st.state == FM_ST_DONE, "state %d err %d", st.state, st.err);
  CHECK((st.flags & FM_FL_ROM_ASSETS) && (st.flags & FM_FL_ASSET_SKIP), "flags %02x", st.flags);
  CHECK(chkf("/games/Mega Game.sfc", 4096, 1), "the game");
  CHECK(chkf("/sd2snes/saves/ME/Mega Game.srm", 8192, 2) && !ex("/sd2snes/saves/SU/Super Game.srm"), ".srm");
  CHECK(chkf("/sd2snes/saves/ME/Mega Game.03.srm", 100, 4), ".NN.srm");
  CHECK(chkf("/sd2snes/saves/ME/Mega Game.slot", 2, 77) && chkf("/sd2snes/saves/SU/Super Game.slot", 2, 3), "a taken target is left alone");
  CHECK(chkf("/sd2snes/states/ME/Mega Game01.state", 3000, 5) && chkf("/sd2snes/states/ME/Mega Game12.state", 3000, 6), "states");
  CHECK(chkf("/sd2snes/info/ME/Mega Game.yml", 300, 7) && chkf("/sd2snes/info/ME/Mega Game.gcv", 900, 8)
        && chkf("/sd2snes/info/ME/Mega Game.01.man", 500, 9), "info");
  CHECK(chkf("/sd2snes/cheats/ME/Mega Game.yml", 200, 10) && chkf("/sd2snes/patches/ME/Mega Game.yml", 50, 11), "cheats, patches");
  CHECK(chkf("/sd2snes/saves/SU/Super Game Two.srm", 10, 12) && chkf("/sd2snes/saves/SU/Super Gamex.srm", 10, 13)
        && chkf("/sd2snes/states/SU/Super Game1.state", 10, 14) && chkf("/sd2snes/info/SU/Super Game.toolong", 10, 15),
        "another game's files moved");
  CHECK(chkf("/games/Mega Game.cov", 2048, 16) && ex("/games/Mega Game.msu") && chkf("/games/Mega Game-1.pcm", 5000, 17)
        && chkf("/games/Mega Game-12.pcm", 5000, 18), "siblings");
  CHECK(chkf("/games/Super Game-x.pcm", 10, 19) && chkf("/games/Super Game Two.cov", 10, 20), "not siblings");
  /* another console: the namespace changes with the extension */
  mkf("/games/Pk.sfc", 100, 1);
  mkf("/sd2snes/saves/PK/Pk.srm", 100, 2);
  run(FM_OP_RENAME, "/games/Pk.sfc", "Pk.gb", NULL);
  CHECK(st.state == FM_ST_DONE && chkf("/sd2snes/saves/sgb/PK/Pk.srm", 100, 2) && !ex("/sd2snes/saves/PK/Pk.srm"), "into sgb/");
  /* the stem stays: nothing to do under /sd2snes */
  mkf("/games/Same.sfc", 100, 1);
  mkf("/sd2snes/saves/SA/Same.srm", 100, 2);
  run(FM_OP_RENAME, "/games/Same.sfc", "Same.smc", NULL);
  CHECK(st.state == FM_ST_DONE && chkf("/sd2snes/saves/SA/Same.srm", 100, 2) && !(st.flags & FM_FL_ROM_ASSETS), "same stem");
  /* not a game */
  mkf("/games/notes.txt", 10, 1);
  mkf("/sd2snes/info/NO/notes.yml", 10, 2);
  run(FM_OP_RENAME, "/games/notes.txt", "nates.txt", NULL);
  CHECK(st.state == FM_ST_DONE && chkf("/sd2snes/info/NO/notes.yml", 10, 2), "a non-game file has no family");
}

static void t_game_paste(void) {
  script_t sc;
  game_setup();
  mkd("/other");
  paste(FM_CLIP_CUT, "/games", NAMES("Super Game.sfc"), "/other", NULL);
  CHECK(st.state == FM_ST_DONE && st.n_failed == 0, "state %d failed %d", st.state, st.n_failed);
  CHECK(chkf("/other/Super Game.cov", 2048, 16) && ex("/other/Super Game.msu") && chkf("/other/Super Game-1.pcm", 5000, 17)
        && chkf("/other/Super Game-12.pcm", 5000, 18) && !ex("/games/Super Game.cov"), "cut takes the siblings");
  CHECK(chkf("/sd2snes/saves/SU/Super Game.srm", 8192, 2), "the same leaf needs nothing under /sd2snes");

  game_setup();
  mkd("/other");
  paste(FM_CLIP_CUT, "/games", NAMES("Super Game.sfc", "Super Game.msu"), "/other", NULL);
  CHECK(st.state == FM_ST_DONE && st.n_failed == 0 && st.n_done == 2 && ex("/other/Super Game.msu"),
        "a sibling also on the clipboard: done %d failed %d err %d", st.n_done, st.n_failed, st.err);

  game_setup();
  mkd("/other");
  paste(FM_CLIP_COPY, "/games", NAMES("Super Game.sfc"), "/other", NULL);
  CHECK(st.state == FM_ST_DONE && chkf("/other/Super Game.cov", 2048, 16) && chkf("/games/Super Game.cov", 2048, 16), "copy takes a copy of the cover");
  CHECK(!ex("/other/Super Game.msu") && !ex("/other/Super Game-1.pcm"), "copy took the MSU-1 pack");

  game_setup();
  paste(FM_CLIP_COPY, "/games", NAMES("Super Game.sfc"), "/games", NULL);
  CHECK(st.state == FM_ST_DONE && chkf("/games/Super Game (2).sfc", 4096, 1) && chkf("/games/Super Game (2).cov", 2048, 16), "keep both");
  CHECK(chkf("/sd2snes/info/SU/Super Game (2).yml", 300, 7) && chkf("/sd2snes/cheats/SU/Super Game (2).yml", 200, 10)
        && chkf("/sd2snes/info/SU/Super Game.yml", 300, 7), "info and cheats copied");
  CHECK(!ex("/sd2snes/saves/SU/Super Game (2).srm") && !ex("/sd2snes/states/SU/Super Game (2)01.state")
        && !ex("/sd2snes/patches/SU/Super Game (2).yml"), "a copy shares saves / states / patches");

  fresh();
  mkf("/src/Game.sfc", 100, 1);
  mkf("/sd2snes/saves/GA/Game.srm", 100, 2);
  mkf("/dst/Game.sfc", 100, 3);
  memset(&sc, 0, sizeof(sc)); sc.ans[0] = FM_ANS_KEEPBOTH;
  paste(FM_CLIP_CUT, "/src", NAMES("Game.sfc"), "/dst", &sc);
  CHECK(st.state == FM_ST_DONE && chkf("/dst/Game (2).sfc", 100, 1) && chkf("/sd2snes/saves/GA/Game (2).srm", 100, 2)
        && !ex("/sd2snes/saves/GA/Game.srm"), "cut kept as both moves the saves");
}

static long list_read(const char *p, char *out) {
  long n = readall(p, (uint8_t *)out, 4096);
  if(n >= 0) out[n] = 0;
  return n;
}
static int list_is(const char *p, const char *want, size_t wl) {
  char got[4100];
  long n = list_read(p, got);
  if(n != (long)wl || memcmp(got, want, wl)) {
    printf("    %s: ", p);
    for(long i = 0; i < n; i++) putchar(got[i] ? (got[i] == '\t' ? '>' : got[i]) : '|');
    printf("\n");
    return 0;
  }
  return 1;
}
#define LIST(s) s, sizeof(s) - 1

static void lists_setup(void) {
  static const char last[] = "/games/Super Game.sfc\0/games/Other.sfc\tpatch.ips\0/games/sub/Deep.sfc\0/games/sub/Deep2.sfc\0/games/subway.sfc\0";
  static const char fav[] = "/games/Super Game.sfc\tSG fix.bps\0/elsewhere/X.sfc\0";
  fresh();
  mkf("/games/Super Game.sfc", 10, 1);
  mkf("/games/Other.sfc", 10, 2);
  mkf("/games/patch.ips", 10, 3);
  mkf("/games/SG fix.bps", 10, 4);
  mkf("/games/sub/Deep.sfc", 10, 5);
  mkf("/games/sub/Deep2.sfc", 10, 6);
  mkf("/games/subway.sfc", 10, 7);
  mkbytes("/sd2snes/lastgame.cfg", last, sizeof(last) - 1);
  mkbytes("/sd2snes/favorites.cfg", fav, sizeof(fav) - 1);
}

static void t_lists(void) {
  lists_setup();
  run(FM_OP_RENAME, "/games/Super Game.sfc", "Mega Game.sfc", NULL);
  CHECK(st.state == FM_ST_DONE && (st.flags & FM_FL_LISTS) && refreshes == 1, "rename: flags %02x refreshes %d", st.flags, refreshes);
  CHECK(list_is("/sd2snes/lastgame.cfg", LIST("/games/Mega Game.sfc\0/games/Other.sfc\tpatch.ips\0/games/sub/Deep.sfc\0/games/sub/Deep2.sfc\0/games/subway.sfc\0")), "recents");
  CHECK(list_is("/sd2snes/favorites.cfg", LIST("/games/Mega Game.sfc\tSG fix.bps\0/elsewhere/X.sfc\0")), "favorites");
  CHECK(!ex("/sd2snes/lastgame.cfg.tmp") && !ex("/sd2snes/favorites.cfg.tmp"), "a temp file was left");

  run(FM_OP_RENAME, "/games/patch.ips", "fix.ips", NULL);
  CHECK(list_is("/sd2snes/lastgame.cfg", LIST("/games/Mega Game.sfc\0/games/Other.sfc\tfix.ips\0/games/sub/Deep.sfc\0/games/sub/Deep2.sfc\0/games/subway.sfc\0")), "a renamed patch");

  run(FM_OP_RENAME, "/games/sub/", "sub2", NULL);
  CHECK(st.state == FM_ST_DONE, "folder rename");
  CHECK(list_is("/sd2snes/lastgame.cfg", LIST("/games/Mega Game.sfc\0/games/Other.sfc\tfix.ips\0/games/sub2/Deep.sfc\0/games/sub2/Deep2.sfc\0/games/subway.sfc\0")), "a renamed folder");

  mkd("/x");
  refreshes = 0;
  paste(FM_CLIP_CUT, "/games", NAMES("Other.sfc"), "/x", NULL);
  CHECK(st.state == FM_ST_DONE && refreshes == 1, "cut: refreshes %d", refreshes);
  CHECK(list_is("/sd2snes/lastgame.cfg", LIST("/games/Mega Game.sfc\0/x/Other.sfc\tfix.ips\0/games/sub2/Deep.sfc\0/games/sub2/Deep2.sfc\0/games/subway.sfc\0")), "a cut game");

  refreshes = 0;
  paste(FM_CLIP_COPY, "/games", NAMES("Mega Game.sfc", "sub2/"), "/x", NULL);
  CHECK(st.state == FM_ST_DONE && refreshes == 0 && !(st.flags & FM_FL_LISTS), "a copy touched the lists");
  CHECK(list_is("/sd2snes/lastgame.cfg", LIST("/games/Mega Game.sfc\0/x/Other.sfc\tfix.ips\0/games/sub2/Deep.sfc\0/games/sub2/Deep2.sfc\0/games/subway.sfc\0")), "after a copy");

  run(FM_OP_RENAME, "/games/subway.sfc", "subway2.sfc", NULL);
  run(FM_OP_RENAME, "/games/subway2.sfc", "subway.sfc", NULL);
  refreshes = 0;
  run(FM_OP_RENAME, "/x/Mega Game.sfc", "unrelated.sfc", NULL);
  CHECK(st.state == FM_ST_DONE && refreshes == 0 && !(st.flags & FM_FL_LISTS), "an unlisted file rewrote the lists");
}

/* ---- the viewer ---- */
static const char *vline(int i) { return (const char *)psram + SRAM_FM_VIEW_ADDR + i * FM_VIEW_STRIDE; }
static void view(const char *p) { run(FM_OP_VIEW, p, NULL, NULL); }
static void view_win(int k) { fopmod_fns.ctl(mst, FM_CTL_VIEW_WIN | (k << 8)); rd_stat(); }
static int lines_are(const char *const *want) {
  int i;
  for(i = 0; want[i]; i++) {
    if(i >= st.view_lines || strcmp(vline(i), want[i])) {
      printf("    line %d: '%s' want '%s'\n", i, i < st.view_lines ? vline(i) : "(none)", want[i]);
      return 0;
    }
  }
  if(i != st.view_lines) { printf("    %d lines, want %d\n", st.view_lines, i); return 0; }
  return 1;
}

static void t_view_text(void) {
  static const char u8[] = "\xEF\xBB\xBFh\xC3\xA9llo \xE2\x82\xAC\n\xE2\x80\xA6x";
  static const char w1252[] = "h\xe9llo \x80\n\x85x";
  fresh();
  mkbytes("/t/u.txt", u8, sizeof(u8) - 1);
  mkbytes("/t/w.txt", w1252, sizeof(w1252) - 1);
  mkbytes("/t/brk.txt", LIST("a\r\nb\nc\rd\r\n\r\ne"));
  mkbytes("/t/tab.txt", LIST("\tx\ty\tzz\t|"));
  mkbytes("/t/ctl.txt", LIST("a\0b\x01" "c\x7f" "d\x1b"));
  mkbytes("/t/empty.txt", "", 0);
  view("/t/u.txt");
  CHECK(st.state == FM_ST_VIEW && st.view_last == 1 && st.view_win == 0 && st.view_pct == 0, "state %d", st.state);
  CHECK(lines_are(NAMES("h\xe9llo E", "...x")), "UTF-8");
  CHECK(!strcmp((char *)st.cur_name, "u.txt"), "cur_name '%s'", st.cur_name);
  view("/t/w.txt");
  CHECK(lines_are(NAMES("h\xe9llo E", "...x")), "CP1252");
  view("/t/brk.txt");
  CHECK(lines_are(NAMES("a", "b", "c", "d", "", "e")), "CR, LF, CRLF");
  view("/t/tab.txt");
  CHECK(lines_are(NAMES("    x   y   zz  |")), "tabs");
  view("/t/ctl.txt");
  CHECK(lines_are(NAMES("a?b?c?d?")), "control bytes");
  view("/t/empty.txt");
  CHECK(st.state == FM_ST_VIEW && st.view_lines == 0 && st.view_last == 1, "empty file");
  view("/t/none.txt");
  CHECK(st.state == FM_ST_ERROR && st.err == FM_ERR_NOTFOUND, "missing file: %d %d", st.state, st.err);
}

static void t_view_wrap(void) {
  char buf[400], want[100];
  int i;
  fresh();
  memset(buf, 'x', 150);
  mkbytes("/t/x.txt", buf, 150);
  view("/t/x.txt");
  CHECK(st.view_lines == 3 && strlen(vline(0)) == FM_VIEW_COLS && strlen(vline(1)) == FM_VIEW_COLS
        && strlen(vline(2)) == 150 - 2 * FM_VIEW_COLS, "hard wrap");
  /* each record says where its line starts in the file, for the viewer's border */
  CHECK(psram[SRAM_FM_VIEW_ADDR + FM_VIEW_PCT] == 0
        && psram[SRAM_FM_VIEW_ADDR + FM_VIEW_STRIDE + FM_VIEW_PCT] == FM_VIEW_COLS * 100 / 150
        && psram[SRAM_FM_VIEW_ADDR + 2 * FM_VIEW_STRIDE + FM_VIEW_PCT] == 2 * FM_VIEW_COLS * 100 / 150,
        "line positions %d %d %d", psram[SRAM_FM_VIEW_ADDR + FM_VIEW_PCT],
        psram[SRAM_FM_VIEW_ADDR + FM_VIEW_STRIDE + FM_VIEW_PCT],
        psram[SRAM_FM_VIEW_ADDR + 2 * FM_VIEW_STRIDE + FM_VIEW_PCT]);
  buf[0] = 0;
  for(i = 0; i < 20; i++) strcat(buf, "word567 ");
  mkbytes("/t/w.txt", buf, strlen(buf));
  view("/t/w.txt");
  memcpy(want, buf, 55); want[55] = 0;
  CHECK(!strcmp(vline(0), want), "wrap at a space: '%s'", vline(0));
  CHECK(!strncmp(vline(1), "word567 ", 8), "the next line starts with the word: '%s'", vline(1));
}

/* page through every window; the lines must come back in order, each once */
static void t_view_windows(void) {
  char *buf = malloc(200000), want[128];
  int i, k, line = 0, ok = 1, pct = -1;
  size_t n = 0;
  fresh();
  for(i = 0; i < 12000; i++) n += sprintf(buf + n, "line%05d\n", i);
  mkbytes("/t/many.txt", buf, n);
  view("/t/many.txt");
  CHECK(st.view_lines == FM_VIEW_MAXLINES && !st.view_last, "window 0: %d lines last %d", st.view_lines, st.view_last);
  view_win(3);
  CHECK(st.view_win == 0, "a window whose start is not known was rendered");
  for(k = 0; ; k++) {
    if(k) view_win(k);
    CHECK(st.state == FM_ST_VIEW && st.view_win == k, "window %d: state %d win %d", k, st.state, st.view_win);
    CHECK(st.view_pct > pct || (k == 0 && st.view_pct == 0), "pct %d after %d", st.view_pct, pct);
    pct = st.view_pct;
    for(i = 0; i < st.view_lines; i++, line++) {
      sprintf(want, "line%05d", line);
      if(ok && strcmp(vline(i), want)) { ok = 0; printf("    window %d line %d: '%s' want '%s'\n", k, i, vline(i), want); }
    }
    if(st.view_last || k > 20) break;
  }
  CHECK(ok && line == 12000 && k == 5, "%d lines in %d windows", line, k + 1);
  view_win(1);
  sprintf(want, "line%05d", FM_VIEW_MAXLINES);
  CHECK(st.view_win == 1 && !strcmp(vline(0), want), "back to window 1: '%s'", vline(0));

  /* windows cut by bytes: 100-character lines, two screen lines each */
  n = 0;
  for(i = 0; i < 1000; i++) n += sprintf(buf + n, "%099d\n", i);
  mkbytes("/t/wide.txt", buf, n);
  view("/t/wide.txt");
  line = 0; ok = 1;
  for(k = 0; ; k++) {
    if(k) view_win(k);
    CHECK(st.view_lines % 2 == 0, "window %d ends inside a line (%d)", k, st.view_lines);
    for(i = 0; i + 1 < st.view_lines; i += 2, line++) {
      char got[128];
      snprintf(got, sizeof(got), "%s%s", vline(i), vline(i + 1));
      sprintf(want, "%099d", line);
      if(ok && strcmp(got, want)) { ok = 0; printf("    window %d line %d: '%s'\n", k, i, got); }
    }
    if(st.view_last || k > 20) break;
  }
  CHECK(ok && line == 1000 && k == 3, "%d lines in %d windows", line, k + 1);

  /* only line breaks: the screen lines run out first */
  memset(buf, '\n', 4000);
  mkbytes("/t/nl.txt", buf, 4000);
  view("/t/nl.txt");
  CHECK(st.view_lines == FM_VIEW_MAXLINES && !st.view_last, "window 0: %d", st.view_lines);
  view_win(1);
  CHECK(st.view_lines == 4000 - FM_VIEW_MAXLINES && st.view_last, "window 1: %d last %d", st.view_lines, st.view_last);

  /* one line longer than a window */
  memset(buf, 'y', 40000);
  mkbytes("/t/long.txt", buf, 40000);
  view("/t/long.txt");
  size_t total = 0;
  for(i = 0; i < st.view_lines; i++) total += strlen(vline(i));
  CHECK(!st.view_last && total == FM_VIEW_WINDOW, "window 0 holds %zu", total);
  view_win(1);
  for(i = 0; i < st.view_lines; i++) total += strlen(vline(i));
  CHECK(st.view_last && total == 40000, "both windows hold %zu", total);
  free(buf);
}

/* ================================================================================ */

static const struct { const char *name; void (*fn)(void); } cases[] = {
  { "copy a file", t_copy_single },
  { "copy a deep tree", t_copy_tree },
  { "copy into its own folder", t_copy_same_folder },
  { "cut = rename", t_cut_is_rename },
  { "into itself", t_into_self },
  { "file conflicts", t_conflict_file },
  { "folder merges", t_merge },
  { "file vs folder", t_mixed },
  { "no space", t_nospace },
  { "cancel", t_cancel },
  { "copy through the staging", t_copy_staging },
  { "rename", t_rename },
  { "mkdir", t_mkdir },
  { "game rename", t_rom_rename },
  { "game paste", t_game_paste },
  { "recents / favorites", t_lists },
  { "viewer: text", t_view_text },
  { "viewer: wrap", t_view_wrap },
  { "viewer: windows", t_view_windows },
};

int main(void) {
  const char *img = getenv("FOP_DISK");
  const char *sp = getenv("FOP_SPARE");
  FILE *f;
  unsigned i;
  if(sp) spare_size = (size_t)atol(sp);
  if(!img || !(f = fopen(img, "rb"))) { fprintf(stderr, "FOP_DISK: no image\n"); return 1; }
  fseek(f, 0, SEEK_END);
  disk_size = (size_t)ftell(f);
  fseek(f, 0, SEEK_SET);
  pristine = malloc(disk_size);
  disk = malloc(disk_size);
  psram = malloc(PS);
  if(fread(pristine, 1, disk_size, f) != disk_size) { fprintf(stderr, "short image\n"); return 1; }
  fclose(f);
  for(i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
    case_fails = 0;
    cases[i].fn();
    end_run();
    printf("%-28s %s\n", cases[i].name, case_fails ? "FAIL" : "ok");
  }
  printf("%s (%d failed check%s, spare %zu)\n", fails ? "FAILED" : "all passed", fails, fails == 1 ? "" : "s", spare_size);
  free(pristine); free(disk); free(psram);
  return fails ? 1 : 0;
}
