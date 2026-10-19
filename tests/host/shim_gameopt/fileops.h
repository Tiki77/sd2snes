/* Host shim for src/fileops.h, used ONLY by run_gameopt.sh: shim/fileops.h plus the
 * Game Boy name test gameopt.c sorts games with (gameopt_cli.c has it). */
#ifndef HOST_GAMEOPT_FILEOPS_H
#define HOST_GAMEOPT_FILEOPS_H

#include "../shim/fileops.h"

int path_is_gb(const char *name);

#endif
