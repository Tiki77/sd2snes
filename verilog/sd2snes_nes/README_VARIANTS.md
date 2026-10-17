# The NES core in three files

The complete NES core (every mapper, both capture protocols) fills the Cyclone IV to 99 % of its
logic elements, so the mapper families are split over three builds of the same sources:

| file on the card | directory | mappers |
|---|---|---|
| `fpga_nes.bi3` (S) | `sd2snes_nes` | NROM / UxROM / CNROM / AxROM / 28, MMC1, the discrete-logic boards (11, 34, 66, 70, 71, 72, 78, 79, 86, 87, 89, 93, 94, 97, 113, 140, 152, 180, 184, 185, 232) |
| `fpga_nes_mmc3.bi3` (B) | `sd2snes_nes_mmc3` | MMC3 (4, MMC6 as its submapper 1), Namco 108 (88, 95, 154, 206), Taito (33, 48, 80, 82) |
| `fpga_nes_k.bi3` (K) | `sd2snes_nes_k` | Sunsoft FME-7 (69), Konami VRC1 / VRC2 / VRC4 (75, 21, 22, 23, 25), Namco 163 / 175 / 340 (19, 210), Jaleco SS 88006 (18), Irem (32, 65) |

`nes_core_for()` in `src/nes.c` sends each mapper number to one of the three files. A card that
has only the first one gets the usual "file missing" popup for a game of another family.

## How the split is made

`MultiMapper` in `mmu.v` selects its mapper with seven one-hot arms (`h_mmc1`, `h_mmc3`, `h_m28`,
`h_m69`, `h_disc`, `h_vrc`, `h_m1k`). Each arm has an `NES_NO_<ARM>` macro that ties it to 0, and
the synthesizer removes the module behind it and the part of the capture protocol only it
used. With no macro defined the source is the complete core, bit for bit.

The macros live in the `main.qsf` of each directory (`VERILOG_MACRO "NES_NO_...=<None>"`). The B and K
directories are variants of `sd2snes_nes` (`VSRC_DIR`, like `sd2snes_bootleg`): their `main.qsf` is
the parent's with the paths prefixed by `../sd2snes_nes/`, the macros of the other families and
their own `SEED`. Simulation testbenches use the same macros through `iverilog -D`.

`tests/test_nes_core_routing.py` keeps the three places that must agree in step: the mapper numbers
of each arm in `mmu.v`, the macros of each `main.qsf`, and the tables in `src/nes.c`. It also checks
that the two variant `main.qsf` have not drifted from the parent.

## Adding a mapper

1. Put it in the arm of its family (or a new arm with its own macro, listed in all three `main.qsf`).
2. Add the number to the matching table in `src/nes.c` (`nes_maps_s`, `nes_maps_b`, `nes_maps_k`).
3. Run `tests/test_nes_core_routing.py`.
4. Re-synthesize the core that carries the family and run the mapper gates with the macros of that
   core defined (`iverilog -DNES_NO_...`), not on the complete source.

## Fitter seeds

Each `main.qsf` pins a `SEED` taken from a sweep of ten seeds (setup and hold of every corner,
`Total negative slack` 0), the one with the widest worst-case margin of setup and hold. A change in the sources of a core (or in the Quartus version) re-rolls
its fit: sweep again before pinning.
