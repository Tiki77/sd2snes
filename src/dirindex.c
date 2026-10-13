/* sd2snes - index of one directory in PSRAM, so a read-only f_open there costs no scan.

   FAT finds a name by reading the directory from its first entry.  The info bucket of a
   common prefix holds thousands of sidecars (/sd2snes/info/SU: every "Super ..." game, over
   2000 files and ~900 sectors), and the info screen opens five or six of them per game:
   each open read hundreds of sectors and a missing file the whole directory, over a second
   per Up/Down on a Mk.III.  How long depended on where the game's files happened to be
   written in the directory, not on where the game is in the list.

   One pass over the directory records a hash of every file's name with its start cluster and
   size, in an open-addressed table in PSRAM; f_open then answers "found" (and opens by the
   cluster) or "no such file" from the table.  The table is trusted only while
       - it is the directory asked about,
       - the volume is the one it was built on (FatFs mount ID), and
       - nothing has written to the volume since (ff_wgen, bumped by every FatFs operation
         that can change it),
   and dirindex_drop() forgets it before anything else claims its PSRAM (a game load, the
   memory test).  The valid flag lives in MCU RAM, so what PSRAM holds after a power cycle
   is never trusted.  A name is matched by two independent hashes and its length: telling
   two names apart this way fails for ~1 pair in 2^48.  An 8.3 alias (with '~') is never
   looked up here: only long names are indexed. */

#include <string.h>
#include "config.h"
#include "uart.h"
#include "ff.h"
#include "memory.h"
#include "scratch.h"
#include "dirindex.h"

#define DIX_SLOTS     8192u                      /* power of two */
#define DIX_MAX       (DIX_SLOTS / 2)            /* more files than this: not indexed */
#define DIX_HDR       SRAM_DIRINDEX_ADDR
#define DIX_TAB       (SRAM_DIRINDEX_ADDR + 16)

typedef struct {
  uint32_t h1;
  uint32_t clust;
  uint32_t size;
  uint16_t h2;
  uint8_t  len;
  uint8_t  tag;      /* the build that wrote the slot; any other value = empty */
} dix_slot_t;
_Static_assert(sizeof(dix_slot_t) == 16, "dix slot");
_Static_assert(16 + DIX_SLOTS * sizeof(dix_slot_t) <= SRAM_DIRINDEX_SIZE, "dirindex region");

typedef struct {
  char     magic[4];
  uint32_t gen;
  uint16_t count;
  uint8_t  tag;
  uint8_t  pad;
  uint32_t id;
} dix_hdr_t;

static uint8_t   dix_valid;
static uint8_t   dix_cleared;       /* the table was zeroed once this session */
static dix_hdr_t dix_cur;
static char      dix_dir[48];

/* case folding as FAT compares long names (code page 1252) */
static uint8_t dix_fold(uint8_t c) {
  if(c >= 'a' && c <= 'z') return c - 0x20;
  if(c >= 0xe0 && c != 0xf7 && c != 0xff) return c - 0x20;
  if(c == 0x9a || c == 0x9c || c == 0x9e) return c - 0x10;
  if(c == 0xff) return 0x9f;
  return c;
}

static void dix_hash(const char *s, unsigned n, uint32_t *h1, uint16_t *h2) {
  uint32_t a = 2166136261u, b = 5381;
  while(n--) {
    uint8_t c = dix_fold((uint8_t)*s++);
    a = (a ^ c) * 16777619u;
    b = b * 33 + c;
  }
  *h1 = a;
  *h2 = (uint16_t)(b ^ (b >> 16));
}

static int dix_same_dir(const char *a, const char *b, unsigned n) {
  while(n--) if(dix_fold((uint8_t)*a++) != dix_fold((uint8_t)*b++)) return 0;
  return 1;
}

static int dix_current(void) {
  return dix_valid && dix_cur.gen == ff_wgen && dix_cur.id == ff_vol_id();
}

void dirindex_drop(void) {
  dix_valid = 0;
}

int dirindex_covers(const char *dir) {
  unsigned n = strlen(dir);
  return dix_current() && n == strlen(dix_dir) && dix_same_dir(dir, dix_dir, n);
}

/* 1 = inserted, 0 = the table is full */
static int dix_insert(const char *name, unsigned n, uint32_t clust, uint32_t size) {
  dix_slot_t s;
  uint32_t h1, i, k;
  uint16_t h2;
  dix_hash(name, n, &h1, &h2);
  for(k = 0, i = h1; k < DIX_SLOTS; k++, i++) {
    uint32_t a = DIX_TAB + (i & (DIX_SLOTS - 1)) * sizeof(s);
    sram_readblock(&s, a, sizeof(s));
    if(s.tag == dix_cur.tag) continue;
    s.h1 = h1; s.clust = clust; s.size = size; s.h2 = h2; s.len = (uint8_t)n; s.tag = dix_cur.tag;
    sram_writeblock(&s, a, sizeof(s));
    return 1;
  }
  return 0;
}

typedef struct {
  char lfn[256];
} dix_leaf_t;
SCRATCH_FITS(dix_leaf_t, SCRATCH_LEAF_BYTES);

void dirindex_prepare(const char *dir) {
  DIR      d;
  FILINFO  fno;
  unsigned count = 0;
  int      ok = 0;

  if(dirindex_covers(dir)) return;
  dix_valid = 0;
  if(strlen(dir) >= sizeof(dix_dir) || !scratch_leaf_take(SCR_DIRINDEX)) return;
  /* a new tag per build makes every slot of the previous one empty; the table is zeroed only
     the first time and when the tag wraps */
  if(!dix_cleared || ++dix_cur.tag == 0) {
    sram_memset(DIX_TAB, DIX_SLOTS * sizeof(dix_slot_t), 0);
    dix_cur.tag = 1;
    dix_cleared = 1;
  }
  dix_cur.gen = ff_wgen;
  fno.lfname = SCRATCH_LEAF(dix_leaf_t)->lfn;
  fno.lfsize = sizeof(SCRATCH_LEAF(dix_leaf_t)->lfn);
  if(f_opendir(&d, (const TCHAR *)dir) == FR_OK) {
    for(;;) {
      const char *nm;
      unsigned n;
      if(f_readdir(&d, &fno) != FR_OK) break;
      nm = *fno.lfname ? fno.lfname : fno.fname;
      if(!nm[0]) { ok = 1; break; }                    /* end of directory */
      if(fno.fattrib & (AM_DIR | AM_VOL)) continue;    /* f_open of these fails anyway */
      n = strlen(nm);
      if(n > 255 || ++count > DIX_MAX || !dix_insert(nm, n, fno.fclust, fno.fsize)) break;
    }
    f_closedir(&d);
  }
  scratch_leaf_drop();
  if(!ok || dix_cur.gen != ff_wgen) return;
  memcpy(dix_cur.magic, "DIX1", 4);
  dix_cur.count = (uint16_t)count;
  dix_cur.id = ff_vol_id();
  sram_writeblock(&dix_cur, DIX_HDR, sizeof(dix_cur));
  strcpy(dix_dir, dir);
  dix_valid = 1;
  printf("dirindex: %s, %u files\n", dir, count);
}

int dirindex_find(const char *path, uint32_t *clust, uint32_t *size) {
  dix_hdr_t hdr;
  dix_slot_t s;
  const char *leaf;
  unsigned dn, n;
  uint32_t h1, i, k;
  uint16_t h2;

  if(!dix_current()) return -1;
  dn = strlen(dix_dir);
  if(!dix_same_dir(path, dix_dir, dn) || path[dn] != '/') return -1;
  leaf = path + dn + 1;
  n = strlen(leaf);
  if(!n || n > 255 || strchr(leaf, '/') || strchr(leaf, '~')) return -1;
  /* the region is MCU-only, but if anything did write over it the header goes first */
  sram_readblock(&hdr, DIX_HDR, sizeof(hdr));
  if(memcmp(&hdr, &dix_cur, sizeof(hdr))) { dix_valid = 0; return -1; }
  dix_hash(leaf, n, &h1, &h2);
  for(k = 0, i = h1; k < DIX_SLOTS; k++, i++) {
    sram_readblock(&s, DIX_TAB + (i & (DIX_SLOTS - 1)) * sizeof(s), sizeof(s));
    if(s.tag != dix_cur.tag) return 0;
    if(s.h1 == h1 && s.h2 == h2 && s.len == n) {
      *clust = s.clust;
      *size = s.size;
      return 1;
    }
  }
  return 0;
}
