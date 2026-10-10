#!/usr/bin/env python3
"""Patch the Xeno Crisis RP2040 flash dump for the sd2snes SoC (what MesenCE does in SoC mode):
  - the firmware additions (xc_fw.bin from `make` here or MesenCE socfw/build.sh: mixer + function replacements) go to 0x10F00000;
  - every function listed in the blob's patch table is redirected (see xc_fw_header.c).
With --split-only, only multicore_launch_core1() is patched (MesenCE Split mode).
With --mk2, the mk2 table is applied as well (sd2snes mk2 core: software division instead of the SIO divider).
The replacement bootrom (xc_bootrom.bin) is separate: it goes to the SoC's boot ROM at 0x00000000.

    xc_patch_image.py [--split-only | --mk2] xenocrisis_rp2040.bin xc_fw.bin patched.bin
"""
import struct, sys

FLASH = 0x10000000
BLOB_ADDR = 0x10F00000
LAUNCH_CORE1 = 0x10059060


def patch(img, blob, split_only=False, mk2=False):
    """Apply the firmware additions to a 16 MB flash image (bytearray) in place; returns the number of patches."""
    if len(img) != 0x1000000:
        raise ValueError('expected a 16 MB flash dump')
    if struct.unpack_from('<H', img, LAUNCH_CORE1 - FLASH)[0] != 0x4905:
        raise ValueError('unexpected firmware build (multicore_launch_core1 not at 0x10059060)')
    magic, version, install, count = struct.unpack_from('<IIII', blob, 0)
    if magic != 0x5843584D or version < 2:
        raise ValueError('not a firmware blob (v2)')
    off = BLOB_ADDR - FLASH
    if any(b != 0xFF for b in img[off:off + len(blob)]):
        raise ValueError('target area in the image is not empty')
    img[off:off + len(blob)] = blob
    patched = 0
    for i in range(count):
        addr, target, kind = struct.unpack_from('<III', blob, 16 + i * 12)
        if split_only and addr != LAUNCH_CORE1:
            continue
        p = addr - FLASH
        if kind == 0:   # push {r0}; ldr r0, [pc, #k]; mov ip, r0; pop {r0}; bx ip; nop; .word target
            lit = (addr + 10 + 3) & ~3
            k = (lit - ((addr + 6) & ~3)) // 4
            struct.pack_into('<6H', img, p, 0xB401, 0x4800 | k, 0x4684, 0xBC01, 0x4760, 0xBF00)
            struct.pack_into('<I', img, lit - FLASH, target)
        elif kind == 1:  # existing veneer: new target word at +12
            struct.pack_into('<I', img, p + 12, target)
        else:            # movs r0, #0; bx lr
            struct.pack_into('<HH', img, p, 0x2000, 0x4770)
        patched += 1
    t = 16 + count * 12            # mk2 table: "MK2P", count, {address, target, kind 0 or 3} per patch
    if mk2 and not split_only:
        if len(blob) < t + 8 or struct.unpack_from('<I', blob, t)[0] != 0x50324B4D:
            raise ValueError('the firmware additions have no mk2 table')
        for i in range(struct.unpack_from('<I', blob, t + 4)[0]):
            addr, target, kind = struct.unpack_from('<III', blob, t + 8 + i * 12)
            if kind == 3:  # code patch: the word itself
                struct.pack_into('<I', img, addr - FLASH, target)
                continue
            lit = (addr + 10 + 3) & ~3
            k = (lit - ((addr + 6) & ~3)) // 4
            struct.pack_into('<6H', img, addr - FLASH, 0xB401, 0x4800 | k, 0x4684, 0xBC01, 0x4760, 0xBF00)
            struct.pack_into('<I', img, lit - FLASH, target)
            patched += 1
    return version, patched


if __name__ == '__main__':
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    split_only = '--split-only' in sys.argv
    mk2 = '--mk2' in sys.argv
    img = bytearray(open(args[0], 'rb').read())
    blob = open(args[1], 'rb').read()
    try:
        version, patched = patch(img, blob, split_only, mk2)
    except ValueError as e:
        sys.exit(str(e))
    open(args[2], 'wb').write(img)
    print('firmware additions v%d (%d bytes) at %08X, %d functions patched' % (version, len(blob), BLOB_ADDR, patched))
