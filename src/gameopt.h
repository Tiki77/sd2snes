/* sd2snes - SD card based universal cartridge for the SNES

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; version 2 of the License only.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

   gameopt.h: per-game settings -- config.yml options that one game overrides, and the
   battery-save / savestate slot the game starts with

   THE FILE.  /sd2snes/saves/[<ns>/]<BB>/<stem>.yml, keyed like the .srm (the patch of a
   patched game, the ROM otherwise).  Flat "Key: value" lines with the SAME keys and the
   same values as config.yml (true/false, numbers), plus two keys of its own:
       SaveGameSlot: 1..4     the battery-save slot the game starts with
       SaveStateSlot: 1..4    its savestate slot
   A key the file does not have, or the value -1, means "follow the global option" (slot 1
   for the two slot keys); the firmware never writes either, it drops the line.  A line
   whose key it does not know is kept as it is (a newer firmware's, the Web Manager's), and
   a file left with nothing is deleted.  The .slot sidecar of before is still read for a
   slot key the file lacks, and removed the first time the file is written.

   WHICH OPTIONS.  gameopt_list in gameopt.c, one line each, by cfg_t field: that order is
   the option's index here and in GAMEOPT_BLK.  The menus show each option inside the
   Settings submenu its row lives in, with the row's own name, description and values
   (snes/gameopt.a65, snes/igopt.i65), so the list is the only place an option is named.

   WHEN.  load_rom calls gameopt_apply right after opening the file and before anything
   identifies the game: from there to the next menu load CFG holds the game's values (so
   every reader -- smc_id, the cores' feature bits, the hooks, the savestates, the console
   players -- needs no change), and the globals wait in gameopt.c.  cfg_save always
   writes the globals, and the next load (the menu's included) puts them back.

   THE BLOCK.  GAMEOPT_BLK (SRAM_GAMEOPT_ADDR, lockstep with GOB_* in snes/memmap.i65),
   republished at every game load and for every SNES_CMD_GAMEOPT.
*/

#ifndef GAMEOPT_H
#define GAMEOPT_H

#include <stdint.h>

#define GAMEOPT_MAX         40        /* room in GAMEOPT_BLK; gameopt_list may not exceed it */

#define GAMEOPT_V_DEFAULT   0xFF      /* the global option decides (the file's -1 or no key) */
#define GAMEOPT_V_NA        0xFE      /* not on this board: the menus leave the row out */

/* a gameopt_list entry's flags; GAMEOPT_F_LIVE also rides in the block's offset words */
#define GAMEOPT_F_LIVE      0x01      /* the in-game menu may change it: takes effect at once */
#define GAMEOPT_F_MK3       0x02      /* a Mk.III-only feature */
#define GAMEOPT_F_SGB       0x04      /* for a Game Boy game: only the SGB core uses it */
#define GAMEOPT_F_GBC       0x08      /* ...only the GBC core (offset word: these << 11) */

/* GAMEOPT_BLK */
#define GAMEOPT_BLK_SIZE    (8 + 4 * GAMEOPT_MAX)
#define GAMEOPT_H_MAGIC     0         /* 'G','O'; the menu zeroes +0 before asking */
#define GAMEOPT_H_VERSION   2
#define GAMEOPT_H_COUNT     3         /* options in the list */
#define GAMEOPT_H_RESULT    4         /* GAMEOPT_RES_* of the last command */
#define GAMEOPT_VAL         8         /* +8.. the game's value of each option, GAMEOPT_V_* */
#define GAMEOPT_GLOBAL      (8 + GAMEOPT_MAX)       /* .. the global value of each */
#define GAMEOPT_OFF         (8 + 2 * GAMEOPT_MAX)   /* .. word: its cfg_t offset, $8000 = live, $2000 / $4000 = a Game Boy game's SGB / GBC core only */
#define GAMEOPT_VERSION     2
#define GAMEOPT_RES_OK      0
#define GAMEOPT_RES_FAIL    1

/* SNES_CMD_GAMEOPT: MCU_PARAM+9 */
#define GAMEOPT_OP_INFO     0
#define GAMEOPT_OP_SET      1         /* +10 = option, +11 = value (GAMEOPT_V_DEFAULT or one) */

/* load_rom: back to the globals (a menu load), then the game's values over them */
void    gameopt_restore(void);
void    gameopt_apply(const char *src);
/* cfg_save: exchange the game's values in CFG with the globals (twice = as before) */
void    gameopt_swap(void);

/* The game the menus' screen is about (its ROM, not its patch): which options apply. */
void    gameopt_classify(const char *rom);

/* Publish GAMEOPT_BLK for the running game. */
void    gameopt_stage(uint8_t result);

/* SNES_CMD_GAMEOPT for the game whose sidecars are keyed off `src`.  live = the game
   running now (the in-game menu): CFG follows and the value takes effect at once. */
void    gameopt_command(const char *src, uint8_t op, uint8_t idx, uint8_t val, int live);

/* The slots a game starts with (0..3, 0xff = the file has none) and their write. */
void    gameopt_slots(const char *src, uint8_t *srm, uint8_t *ss);
int     gameopt_slots_set(const char *src, uint8_t srm, uint8_t ss);

/* For the host test: the per-game file of `path` parsed / written by the same code. */
#define GAMEOPT_SLOTS       4
typedef struct {
  uint8_t val[GAMEOPT_MAX];           /* the file's value of each option, or GAMEOPT_V_DEFAULT */
  uint8_t srm, ss;                    /* 0..3, 0xff = no key */
  uint8_t legacy;                     /* a .slot of before supplied a slot */
} gameopt_file_t;
int     gameopt_count(void);
void    gameopt_line(gameopt_file_t *f, char *line);

#endif
