# xc_soc – Xeno Crisis soft CPU support

Xeno Crisis ships on a SNES cartridge with an RP2040 coprocessor. The sd2snes core (`sd2snes_xc`: 40.25 MHz on the Mk.III,
22 MHz on the Mk.II) replaces it with a soft Cortex-M0 that runs the game's own RP2040 firmware
(`xenocrisis_rp2040.bin`, Xeno Crisis SNES v1.00) from the PSRAM. That CPU has no RP2040 boot ROM, no second core
and no PIO/DMA; this folder fills those gaps. It builds **`xc_soc.bin`**, which contains no game code or data.

**Use:** copy `xc_soc.bin` to `/sd2snes/` with the firmware; mk2 and mk3 use the same file. **It must come from the
same build as the firmware** (`src/obj-<config>/`); the loader (`src/xc_load.c`) refuses a mismatch and
`xc_debug.txt` says why.

## Contents

| Part | Source | Purpose |
|---|---|---|
| Replacement bootrom (soft CPU `0x0`) | `xc_bootrom*.c/.S/.ld` | The boot ROM functions the pico-sdk looks up: tables, bit operations, memset/memcpy, soft float/double. Unused ones (sqrt, trig, exp/log) stop with a panic code. |
| Mixer (`0x10F00000`) | `xc_mix.c`, `xc_mix_entry.S` | Core 1's work. mk3: a 1 kHz tick and a "decoded" interrupt; the MCU decodes the Opus music, `xc_brr` encodes BRR; with an MSU-1 pack the track starts go to the MSU-1. mk2: see below. |
| Function replacements | `xc_shim.c` | SNES bus layer (→ `$3000` window), `sleep_until`, `puts`/`printf` (→ debug port), `panic`, flash write/erase (→ save area, `.srm`). |
| Patch tables | `xc_fw_header.c` | Main table: firmware functions to redirect. mk2 table ("MK2P", up to 64 entries), applied only by the mk2 firmware. Kinds: 0 veneer to the replacement, 1 new target for a veneer, 2 "return 0", 3 write a word. |
| mk2 only | `xc_div.S`, `xc_mix_voice.S`, `xc_brr_sw.*`, `xc_emit.S` | Software division (no SIO divider; same results as the originals), mixing and BRR encoding in assembly, a faster 65816 code emitter. |
| Register map | `xc_soc.h` | The SoC registers the FPGA cores add. |

`xc_soc.bin`: a 512-byte header (`"XSOC"`, version, part lengths), then the bootrom and the firmware additions, each
padded to 16 KB (loaded to PSRAM `0xD20000` and `0xD24000`).

**Build:** `make` in `src/` builds it with the firmware; on its own `make [OUT=dir]` here (only `arm-none-eabi-gcc`).

**Tests and tools:** `test_mix_math.c`, `test_brr_sw.c` (C, against the firmware's own algorithms),
`test_brr_sw_asm.py`, `test_mix_voice_asm.py` (assembly against C, in Unicorn), `xc_patch_image.py` /
`xc_build_image.py` (apply the patches to a flash dump offline).

## mk2

No tick timer, interrupts or BRR encoder, and a write-through cache: **every store, including every push, is a write
to the 8-bit SRAM chip**, so the hot code avoids stores.

- **Mixer:** the firmware's wait loops call `xc_mix_poll()`, which checks without a store whether a tick is due or
  the BRR ring has room, and then runs the due ticks and mixes 2 blocks (8 and the encoder's fast mode while the
  rings are less than half full, 24 below a quarter; without the 24, busy scenes ran the rings almost empty, heard
  as crackling).
- **Mixing:** the sound effects have equal volume left and right, so one channel is mixed, encoded once and written
  to both rings (stereo path in C otherwise).
- **BRR encoding:** only the two candidate shifts instead of all 11; 99.9% of the game's blocks come out identical
  to the firmware's, the rest with the same error. Fast mode (one shift) adds ~1 dB of noise on ~10% of the blocks.
- **Speed-ups** (same results as the originals): the 65816 code emitter as leaf functions entered without stack
  stores (`xc_emit.S`, code patches in the mk2 table), `memcpy` without stack stores and with word stores for any
  alignment (`xc_bootrom_memcpy.S`, mk3 gets it too), and the game's two tile-flag loops with everything in
  registers and two entries per word store (`xc_tile.S`; identical results in 800 randomized cases). The core keeps both stacks in block RAM
  (`verilog/sd2snes_xc/xc_scratch.v`), so stack stores don't go to the SRAM chip.

RTL simulation of the mk2 core at 22 MHz, 60 s of play:

| | |
|---|---|
| frame message → stream post, p50 / p95 / p99 / max | 7.0 / 9.1 / 10.2 / 27.8 ms (without the speed-ups at 20 MHz: p95 11.1, max 55.9) |
| game ticks later than one frame | 1 of 3,673 |
| mixer | 1,492 blocks/s (as the game needs), 18% of the CPU; ring below half full 5 ms of 60 s, no fast-mode blocks |

What is left in slow frames is mostly the game's own code and waiting for the SNES to read the stream. Tried and
not kept: pausing the mixer while a SNES message waits, and mixing less while a frame is built (neither made frames
shorter).
