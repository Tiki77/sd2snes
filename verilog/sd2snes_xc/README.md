# Xeno Crisis core (`sd2snes_xc`)

A soft Cortex-M0 in the FPGA runs the cartridge's RP2040 firmware. Core design and the replacement bootrom /
firmware additions (`src/xc_soc`): M2M ([sttng](https://github.com/sttng/sd2snes-xenocrisis)).

One source tree for both boards; the Mk.II differences are under `` `ifdef MK2 ``:

| Board | Bitstream | Project | Macros |
|---|---|---|---|
| Mk.III / FXPAK Pro (Cyclone IV EP4CE15) | `fpga_xc.bi3` | `sd2snes_xc.qpf` / `main.qsf`, `main.sdc` | `MK3`, `XC_MSU`, `XC_MK3` |
| Mk.II (Spartan-3 XC3S400) | `fpga_xc.bit` | `sd2snes_xc.xise`, `main.ucf` | `MK2`, `XC_MSU` |

`make mk3` / `make mk2` build them (`VSRC` in the `Makefile` is the Mk.II file list; Quartus takes its list from
`main.qsf`). Board-only sources: `xc_scratch.v` (Mk.II); `rtc.v`, `xc_brr.v`, `xc_tick.v`, `xc_decbox.v`,
`xc_mk3_pll.v` (Mk.III).

`main.qsf` puts the registers that sample the SNES bus (address, strobes, data) in the pins' I/O blocks
(`FAST_INPUT_REGISTER`). Without it their placement changed from fit to fit, and fits that met timing froze the
game on some consoles: in those fits the SNES inputs reached their registers ~1.5 ns later and the data bus
direction / output enable changed later. With the registers in the I/O blocks the same fits ran. The outputs to
the SNES are still unconstrained, so a change to this core needs a test on more than one console.

## On the card

- `/sd2snes/fpga_xc.bi3` and/or `fpga_xc.bit`, and `/sd2snes/xc_soc.bin` (built with the firmware; the firmware
  refuses one from another build).
- `/sd2snes/xenocrisis_rp2040.bin`: the cartridge's RP2040 flash dump (16 MB, v1.00), supplied by the user.
- The ROM (`XENOCRISIS`, 128 KB). Without a `.srm`, the cartridge's saves carry over from the dump.
- Music: an MSU-1 pack next to the ROM (`<rom>.msu` + `<rom>-1..26.pcm`, [`MSU1_PACK.md`](MSU1_PACK.md)). This
  firmware has no Opus decoder on any board, so without a pack the game runs with its sound effects only (on the
  Mk.III the mixer's music packets are answered as "not decoded").
- Firmware built with `XC_DEBUG=1` (`make XC_DEBUG=1`, or `XC_DEBUG=1 ./build.sh`) writes a soft CPU halt to
  `/sd2snes/xc_debug.txt` (code and address) when the game is left; the default build leaves it out.

## Design

Two clock domains: the sd2snes side at CLK2 (85.87 MHz) talks to the SNES, the MCU and the memory chips; the soft
CPU side holds the RP2040 replacement (40.25 MHz on the Mk.III, 22 MHz on the Mk.II). Everything between them goes
through `xc_bridge`.

```
         sd2snes side (CLK2)                                soft CPU side

 SNES  <--> $3000 window (xc_window) --+
 PSRAM <--> ROM state machine ---------+
 SRAM  <--> RAM state machine ---------+-- xc_bridge --> xc_soc: Cortex-M0 (xc_m0), caches,
 MCU   <--> mcu_cmd.v <--> Opus mailbox/MSU-1 regs --+       timer, SIO, NVIC (+ BRR encoder on the Mk.III)
 audio <--  msu.v + xc_dac
```

1. The soft CPU runs the game's RP2040 firmware from the PSRAM (code) and the SRAM chip (RAM).
2. It builds 65816 code for the SNES in its RAM and posts it to the `$3000` window as descriptors.
3. The window's DMA copies the bytes into a 512-byte ring; the SNES reads and runs them from `$3000`.
4. The SNES writes its input and frame messages back through the window's RX FIFO.

Left out for space: cheats and in-game hooks (a long reset returns to the menu, the button combination doesn't),
save states.

### Mk.II differences

- **No mixer hardware** (tick timer, interrupts, BRR encoder): the firmware's wait loops call `xc_mix_poll()`, which
  runs the due 1 kHz ticks and mixes a block or two; sound effects are mixed and BRR-encoded in software.
- **No hardware divider:** software division (`src/xc_soc/xc_div.S`).
- **Stacks in block RAM** (`xc_scratch.v`): the top halves of SCRATCH_X/Y take one cycle and are not in the SRAM chip.
- **Speed-ups** applied by the Mk.II firmware through the "MK2P" patch table (the 65816 code emitter, `memcpy`, the
  tile-flag loops).
- **Smaller soft CPU and SoC:** register file in LUT RAM; ROR (register), REV16, REVSH and most MRS/MSR fault;
  APB writes ignored with fixed reads; 40-bit timer; one 16 KB 2-way write-through cache. Anything removed halts
  with a fault code if reached.
- **Timing:** `main.ucf` declares `SYSTEM_JITTER = 0.48 ns` (the DCM CLKFX jitter, which trce does not count by
  itself); the fit closes with it.

### Memory maps

PSRAM (16 MB, built by the MCU when the game loads):

| Address | Contents |
|---|---|
| `0x000000-0x01FFFF` | SNES ROM (128 KB) |
| `0x020000-0xCFFFFF` | RP2040 flash `0x020000-0xCFFFFF` |
| `0xD00000-0xD1FFFF` | RP2040 flash `0x000000-0x01FFFF` |
| `0xD20000-0xD23FFF` | replacement bootrom (soft CPU address 0) |
| `0xD24000-0xD27FFF` | firmware additions (RP2040 flash `0xF00000`) |

SRAM chip (512 KB): `0x00000-0x07FFF` save area (the `.srm`, 32 KB), `0x08000-0x49FFF` RP2040 RAM,
`0x4A000-0x4AFFF` RP2040 USB RAM, `0x4C000-0x4DFFF` APB register shadow (Mk.III).

### MCU commands

| Command | Function |
|---|---|
| `$C0`–`$C4` | Opus mailbox (Mk.III): status, read packet, write PCM, done, acknowledge reset |
| `$C5` XC_STATUS | running/halted, halt code and address |
| `$C6` XC_RUN | bit 0 release the soft CPU, bit 1 MSU-1 music (Mk.III) |
| `$C7` / `$C8` | performance counter snapshot / read |
