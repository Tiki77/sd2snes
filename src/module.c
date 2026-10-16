/* Card modules -- see module.h. */

#include "config.h"
#include "ff.h"
#include "fileops.h"
#include "memory.h"
#include "msu1.h"
#include "crc32.h"
#include "psram_io.h"
#include "scratch.h"
#include "snes.h"
#include "fpga_spi.h"
#include "timer.h"
#include "led.h"
#include "memtest.h"
#include "module.h"

/* sort.c: the READDIR sort cache, idle while a module's screen is open (module_gate) */
extern uint32_t ptrcache[QSORT_MAXELEM];

static uint32_t module_ticks(void)
{
  return getticks();
}

static const uint8_t ram0_bases[] = RAM0_BASES;

static const module_api_t mod_api = {
  sram_readblock, sram_writeblock, sram_memset,
  sram_readbyte, sram_writebyte, sram_readshort, sram_writeshort,
  f_open, f_read, f_close, psram_stream, &file_handle,
  menu_music_play, menu_music_stop, menu_music_pause, menu_music_lock,
  menu_music_noloop, menu_music_seek, menu_music_active, menu_music_ended,
  menu_music_tell, menu_music_size,
  memtest_core, fpga_select_mem, memtest_pass, writeled, module_ticks, menu_sfx_forget,
  CONFIG_CPU_FREQUENCY, ram0_bases, RAM0_NUM_CHIPS, RAM0_SECOND_ADDR, MEMTEST_ITERATIONS
};

static module_fns_t mod;
static uint8_t mod_live;            /* started, not stopped yet */
static uint8_t mod_id;              /* the module in ptrcache, 0 = none (or gone) */

#ifdef MODULE_HOST_LINKED
/* Host builds of the firmware (the Ciclone) cannot run an ARM image: there the modules are
   linked in and nothing is loaded. */
extern const module_fns_t mtmod_fns, pcmmod_fns;
static int module_load(uint8_t id)
{
  static const module_fns_t *const fns[MODULE_COUNT] = { &mtmod_fns, &pcmmod_fns };
  if(!id || id > MODULE_COUNT) return 0;
  mod = *fns[id - 1];
  return 1;
}
#else
static int module_load(uint8_t id)
{
  static const char *const file[MODULE_COUNT] = {
    "/sd2snes/memtest.bin", "/sd2snes/pcmplay.bin"
  };
  uint8_t *img = (uint8_t *)ptrcache;
  const module_hdr_t *h = (const module_hdr_t *)img;
  uint32_t crc = crc32_init(), size, i;
  UINT br = 0;
  if(!id || id > MODULE_COUNT) return 0;
  if(f_open(&file_handle, file[id - 1], FA_READ) != FR_OK) return 0;
  size = f_size(&file_handle);
  if(size < sizeof(*h) || size > sizeof(ptrcache)
     || f_read(&file_handle, img, size, &br) != FR_OK || br != size) {
    f_close(&file_handle);
    return 0;
  }
  f_close(&file_handle);
  for(i = 0; i < size; i++)
    crc = crc32_update(crc, (i >= 12 && i < 16) ? 0 : img[i]);   /* the crc field reads as 0 */
  if(h->magic != MODULE_MAGIC || h->abi != MODULE_ABI || h->size != size
     || h->state_size > sizeof(scratch_frame) || crc32_finalize(crc) != h->crc)
    return 0;
  mod.start = (void *)((uint32_t)img + h->start);
  mod.ctl   = (void *)((uint32_t)img + h->ctl);
  mod.idle  = (void *)((uint32_t)img + h->idle);
  mod.stop  = (void *)((uint32_t)img + h->stop);
  return 1;
}
#endif

int module_open(uint8_t id, const char *path)
{
  if(mod_live) {
    mod.stop(scratch_frame);
    mod_live = 0;
  }
  /* the same module again (the next track, the next file): its image is still in place,
     nothing has used ptrcache since -- every foreign command goes through module_gate */
  if(id != mod_id) {
    mod_id = 0;
    if(!module_load(id)) {
      sram_writebyte(MODULE_ST_MISSING, SRAM_MODULE_STATUS);
      return MODULE_MISSING;
    }
    mod_id = id;
  }
  sram_writebyte(MODULE_ST_OK, SRAM_MODULE_STATUS);
  mod_live = 1;
  return mod.start(scratch_frame, &mod_api, path);
}

void module_ctl(uint32_t param)
{
  if(mod_live) mod.ctl(scratch_frame, param);
}

void module_gate(uint8_t cmd)
{
  if(cmd == SNES_CMD_PLAY_PCM || cmd == SNES_CMD_PCM_CTL) return;
  if(mod_live) mod.stop(scratch_frame);
  mod_live = 0;
  mod_id = 0;                       /* ptrcache and the FRAME are someone else's from here */
}

void module_idle(void)
{
  if(mod_live) mod.idle(scratch_frame);
}
