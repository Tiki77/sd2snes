#!/usr/bin/env python3
"""Build the sd2snes image for Xeno Crisis (sd2snes_xc core, Mk.III build).

Not needed any more with the current sd2snes firmware: it builds the same image at load time from the 128 KB
SNES ROM, /sd2snes/xenocrisis_rp2040.bin and /sd2snes/xc_soc.bin (from `make` in this folder, or MesenCE socfw/build.sh); see sd2snes src/xc_load.c.
An image made by this script still loads (any Xeno Crisis file larger than 128 KB is taken as prebuilt).

    xc_build_image.py <kernel.sfc> <xenocrisis_rp2040.bin> <out.sfc> [--srm out.srm]

The sd2snes MCU loads the whole file into the PSRAM, from address 0. The sd2snes_xc core maps it as:

    file / PSRAM         contents                                  seen by
    0x000000-0x01FFFF    SNES kernel ROM (128 KB, LoROM)           SNES
    0x020000-0xCFFFFF    RP2040 flash 0x020000-0xCFFFFF            soft CPU (flash offset = PSRAM address)
    0xD00000-0xD1FFFF    RP2040 flash 0x000000-0x01FFFF            soft CPU
    0xD20000-0xD23FFF    replacement bootrom (xc_bootrom.bin)      soft CPU (0x00000000)
    0xD24000-0xD27FFF    RP2040 flash 0xF00000-0xF03FFF            soft CPU (firmware additions, xc_fw.bin)

The flash image is patched for the SoC first (xc_patch_image.py). The flash save area (0xFF8000-0xFFFFFF)
is not in the file: the core keeps it in the SRAM chip, which the MCU loads from / saves to the .srm file.
--srm writes the dump's save area as a .srm (to keep the saves from the cartridge).

xc_fw.bin and xc_bootrom.bin come from `make` (or MesenCE socfw/build.sh), next to this script.
"""
import os, sys, struct
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import xc_patch_image

SAVE_OFFSET = 0xFF8000
FLASH_END = 0xD00000          # identity-mapped flash ends here (the firmware uses up to 0xCE3000)
BLOB_OFFSET = 0xF00000
LAYOUT_END = 0xD28000

def build(kernel, flash, fw, bootrom):
    if len(kernel) != 0x20000:
        raise ValueError('expected the 128 KB kernel ROM')
    if kernel[0x7FB0:0x7FB6] != b'BMXCRI' or kernel[0x7FD6] != 0x63:
        raise ValueError('not the Xeno Crisis kernel (maker BM, game XCRI, chipset $63)')
    img = bytearray(flash)
    xc_patch_image.patch(img, fw)
    if len(fw) > 0x4000:
        raise ValueError('firmware additions larger than 16 KB')
    if len(bootrom) > 0x4000:
        raise ValueError('bootrom larger than 16 KB')
    if any(b != 0xFF for b in img[0xCE3000:FLASH_END]):
        raise ValueError('flash data above 0xCE3000: unexpected firmware build')
    if any(b != 0xFF for b in img[BLOB_OFFSET + len(fw):SAVE_OFFSET]):
        raise ValueError('unexpected data between the firmware additions and the save area')
    out = bytearray(b'\xFF' * LAYOUT_END)
    out[0x000000:0x020000] = kernel
    out[0x020000:FLASH_END] = img[0x020000:FLASH_END]
    out[0xD00000:0xD20000] = img[0x000000:0x020000]
    out[0xD20000:0xD20000 + len(bootrom)] = bootrom
    out[0xD24000:0xD28000] = img[BLOB_OFFSET:BLOB_OFFSET + 0x4000]
    return out, bytes(img[SAVE_OFFSET:])

if __name__ == '__main__':
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    srm = None
    if '--srm' in sys.argv:
        srm = sys.argv[sys.argv.index('--srm') + 1]
        args.remove(srm)
    if len(args) != 3:
        sys.exit(__doc__)
    here = os.path.dirname(os.path.abspath(__file__))
    kernel = open(args[0], 'rb').read()
    flash = open(args[1], 'rb').read()
    fw = open(os.path.join(here, 'xc_fw.bin'), 'rb').read()
    bootrom = open(os.path.join(here, 'xc_bootrom.bin'), 'rb').read()
    try:
        out, save = build(kernel, flash, fw, bootrom)
    except ValueError as e:
        sys.exit(str(e))
    open(args[2], 'wb').write(out)
    print('%s: %d bytes' % (args[2], len(out)))
    if srm:
        open(srm, 'wb').write(save)
        print('%s: save area of the dump (%d bytes)' % (srm, len(save)))
