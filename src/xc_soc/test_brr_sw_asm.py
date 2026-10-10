#!/usr/bin/env python3
"""Checks xc_brr_sw.S (the mk2 BRR encoder in assembly) against xc_brr_sw.h (built as a host library), in Unicorn.
   pip install unicorn;  python3 test_brr_sw_asm.py [blocks.bin ...]   (16 x int16 per block, e.g. XC_BRRDUMP)
Without files: 200,000 random blocks. Both layouts (stride 2 and 4 bytes) and both modes (exact, fast) are tested. Also counts instructions."""
import os, sys, struct, random, subprocess, tempfile, ctypes
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE
from unicorn.arm_const import *

here = os.path.dirname(os.path.abspath(__file__))
tmp = tempfile.mkdtemp()
subprocess.check_call(['arm-none-eabi-gcc', '-mcpu=cortex-m0plus', '-c', os.path.join(here, 'xc_brr_sw.S'), '-o', tmp + '/a.o'])
subprocess.check_call(['arm-none-eabi-objcopy', '-O', 'binary', '-j', '.text.xc_brr_encode', tmp + '/a.o', tmp + '/a.bin'])
code = open(tmp + '/a.bin', 'rb').read()
open(tmp + '/c.c', 'w').write('#include "%s"\nuint32_t enc(const int16_t* s, int stride, uint32_t* ab, int fast) { return brr_sw_encode(s, stride, ab, ab + 1, fast); }\n' % os.path.join(here, 'xc_brr_sw.h'))
subprocess.check_call(['cc', '-O2', '-shared', '-fPIC', tmp + '/c.c', '-o', tmp + '/c.so'])
lib = ctypes.CDLL(tmp + '/c.so')
lib.enc.restype = ctypes.c_uint32

CODE, DATA, STACK = 0x10000000, 0x20000000, 0x20010000
mu = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
mu.mem_map(CODE, 0x10000); mu.mem_write(CODE, code)
mu.mem_map(DATA, 0x20000)
count = [0]
mu.hook_add(UC_HOOK_CODE, lambda uc, a, s, u: count.__setitem__(0, count[0] + 1))
RET = CODE + 0x8000
mu.mem_write(RET, b'\x00\xbf\x00\xbf')

def run_asm(samples, stride, fast):
    buf = bytearray(64)
    for i, v in enumerate(samples): struct.pack_into('<h', buf, i * stride, v)
    mu.mem_write(DATA, bytes(buf))
    mu.reg_write(UC_ARM_REG_R0, DATA); mu.reg_write(UC_ARM_REG_R1, stride | fast); mu.reg_write(UC_ARM_REG_R2, DATA + 0x100)
    mu.reg_write(UC_ARM_REG_SP, STACK); mu.reg_write(UC_ARM_REG_LR, RET | 1)
    mu.emu_start(CODE | 1, RET)
    h = mu.reg_read(UC_ARM_REG_R0)
    a, b = struct.unpack('<II', mu.mem_read(DATA + 0x100, 8))
    return h, a, b

def run_c(samples, stride, fast):
    arr = (ctypes.c_int16 * 32)()
    for i, v in enumerate(samples): arr[i * (stride // 2)] = v
    ab = (ctypes.c_uint32 * 2)()
    h = lib.enc(arr, stride // 2, ab, fast)
    return h, ab[0], ab[1]

def blocks():
    files = sys.argv[1:]
    if files:
        for f in files:
            d = open(f, 'rb').read()
            for k in range(len(d) // 32):
                yield list(struct.unpack_from('<16h', d, k * 32))
    else:
        rnd = random.Random(1)
        for k in range(200000):
            amp = 1 << rnd.randrange(16)
            yield [max(-32768, min(32767, rnd.randint(-amp, amp))) for _ in range(16)]

n = bad = nonzero = 0
count[0] = 0
for s in blocks():
    if not any(s): continue          # the mixer never calls the encoder for silence
    stride = 2 if n & 1 else 4
    fast = (n >> 1) & 1
    x, y = run_asm(s, stride, fast), run_c(s, stride, fast)
    n += 1
    if x != y:
        bad += 1
        if bad <= 5: print('mismatch', s, stride, [hex(v) for v in x], [hex(v) for v in y])
print('%d blocks, %d mismatches, %.0f instructions per block' % (n, bad, count[0] / max(n, 1)))
sys.exit(1 if bad else 0)
