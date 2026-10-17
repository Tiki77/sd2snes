/* host stand-in for src/cfg.h: only the field nesave.c reads. */
#ifndef HOST_CFG_H
#define HOST_CFG_H
#include <stdint.h>
typedef struct { uint8_t enable_autosave; } cfg_t;
#endif
