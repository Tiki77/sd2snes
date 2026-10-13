#ifndef DIRINDEX_H
#define DIRINDEX_H

#include <stdint.h>

/* An index of ONE directory's files, kept in PSRAM, that answers a read-only f_open in that
   directory without scanning it (see dirindex.c).  Menu mode only. */

/* Index `dir` (absolute, no trailing '/') unless it is already the indexed one. */
void dirindex_prepare(const char *dir);

/* 1 = `dir` is indexed and the index is current. */
int dirindex_covers(const char *dir);

/* Forget the index (its PSRAM is about to be overwritten). */
void dirindex_drop(void);

/* For f_open: -1 = not covered (scan as usual), 0 = no such file, 1 = found. */
int dirindex_find(const char *path, uint32_t *clust, uint32_t *size);

#endif
