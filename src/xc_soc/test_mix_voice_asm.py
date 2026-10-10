#!/usr/bin/env python3
"""Checks xc_mix_voice.S (mono voice mixing for the mk2 mixer, assembly) against the C loop it replaces, in
Unicorn: random voices (lengths, positions, steps incl. negative, volumes, samples), first voice and added voice,
voices that end inside the block.   pip install unicorn;  python3 test_mix_voice_asm.py"""
import os, sys, struct, random, subprocess, tempfile, ctypes
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE
from unicorn.arm_const import *

here = os.path.dirname(os.path.abspath(__file__))
tmp = tempfile.mkdtemp()
subprocess.check_call(['arm-none-eabi-gcc', '-mcpu=cortex-m0plus', '-c', os.path.join(here, 'xc_mix_voice.S'), '-o', tmp + '/a.o'])
subprocess.check_call(['arm-none-eabi-objcopy', '-O', 'binary', '-j', '.text.xc_mix_voice_mono', tmp + '/a.o', tmp + '/a.bin'])
code = open(tmp + '/a.bin', 'rb').read()

# the C reference: the loop of mix_voice_mono() in xc_mix.c, which follows the firmware's (mix_blocks)
open(tmp + '/c.c', 'w').write(r'''
#include <stdint.h>
static inline int32_t div_pow2(int32_t x, int k) { return (x + (int32_t)((uint32_t)(x >> 31) >> (32 - k))) >> k; }
static inline int32_t qmul(int32_t a, int32_t b) { int32_t p = (int32_t)((uint32_t)a * (uint32_t)b); int32_t q = div_pow2(p, 14); int32_t h = div_pow2(q, 1); return h + (q - 2 * h); }
static inline int32_t clamp(int32_t x) { if((uint32_t)x + 0x7FFFu > 0xFFFEu) return (int32_t)((uint32_t)div_pow2(x, 15) * 32760u); return x; }
/* v: 32-byte voice; samples: base of the sample data (host pointer), buf: 8 words */
void ref(uint8_t* v, const int16_t* samples, uint32_t* buf, int add)
{
	int32_t length = *(int32_t*)(v + 0), pos = *(int32_t*)(v + 12), step = *(int32_t*)(v + 8), vol = *(int32_t*)(v + 16);
	int ended = 0;
	for(int k = 0; k < 8; k++) {
		int32_t c[2];
		for(int j = 0; j < 2; j++) {
			c[j] = 0;
			if(!ended) {
				int32_t idx = div_pow2(pos, 15);
				if(length <= idx || pos < (int32_t)0xFFFF8001u) { v[0x1C] = 0; v[0x1D] = 1; ended = 1; }
				else { c[j] = qmul(samples[idx], vol); pos += step; }
			}
		}
		if(add) { uint32_t w = buf[k]; c[0] = clamp(c[0] + (int32_t)(int16_t)w); c[1] = clamp(c[1] + ((int32_t)w >> 16)); }
		else { c[0] = clamp(c[0]); c[1] = clamp(c[1]); }
		buf[k] = ((uint32_t)c[0] & 0xFFFFu) | (uint32_t)c[1] << 16;
		if(ended && add) break;
	}
	*(int32_t*)(v + 12) = pos;
}
''')
subprocess.check_call(['cc', '-O2', '-shared', '-fPIC', tmp + '/c.c', '-o', tmp + '/c.so'])
lib = ctypes.CDLL(tmp + '/c.so')

CODE, DATA, SAMP, STACK = 0x10000000, 0x20000000, 0x20100000, 0x20010000
mu = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
mu.mem_map(CODE, 0x10000); mu.mem_write(CODE, code)
mu.mem_map(DATA, 0x20000); mu.mem_map(SAMP, 0x40000)
count = [0]
mu.hook_add(UC_HOOK_CODE, lambda uc, a, s, u: count.__setitem__(0, count[0] + 1))
RET = CODE + 0x8000
mu.mem_write(RET, b'\x00\xbf\x00\xbf')
rnd = random.Random(2)
NS = 0x10000
samples = [rnd.randint(-32768, 32767) for _ in range(NS)]
mu.mem_write(SAMP, struct.pack('<%dh' % NS, *samples))
csamp = (ctypes.c_int16 * NS)(*samples)

bad = 0
N = int(sys.argv[1]) if len(sys.argv) > 1 else 200000
count[0] = 0
for n in range(N):
    kind = rnd.randrange(6)
    length = rnd.choice([rnd.randint(0, NS), rnd.randint(-5, 40), 0x7FFFFFFF if False else rnd.randint(0, NS)])
    pos = rnd.choice([rnd.randint(0, (NS - 64) * 32768 // 2), rnd.randint(-70000, 70000), (length - rnd.randint(0, 20)) * 32768 if 0 < length < NS else 0])
    step = rnd.choice([0x4AAA, rnd.randint(0, 0x10000), rnd.randint(-0x10000, 0x10000)])
    vol = rnd.choice([0x3333, rnd.randint(0, 0x8000), rnd.randint(-0x10000, 0x10000), 0x7FFF, 0x10000])
    if pos // 32768 + 16 * max(step, 0) // 32768 + 2 >= NS: pos = 0
    v = bytearray(32)
    struct.pack_into('<iIiii', v, 0, length, SAMP, step, pos, vol); v[0x1C] = 1
    add = rnd.randrange(2)
    buf = [rnd.getrandbits(32) if kind else rnd.choice([0x7FFF7FFF, 0x80008000, 0x80018001]) for _ in range(8)]
    # C
    cv = (ctypes.c_uint8 * 32)(*v); cb = (ctypes.c_uint32 * 8)(*buf)
    lib.ref(cv, csamp, cb, add)
    # asm
    mu.mem_write(DATA, bytes(v)); mu.mem_write(DATA + 0x100, struct.pack('<8I', *buf))
    mu.reg_write(UC_ARM_REG_R0, DATA); mu.reg_write(UC_ARM_REG_R1, DATA + 0x100); mu.reg_write(UC_ARM_REG_R2, add)
    mu.reg_write(UC_ARM_REG_SP, STACK); mu.reg_write(UC_ARM_REG_LR, RET | 1)
    mu.emu_start(CODE | 1, RET)
    av = bytes(mu.mem_read(DATA, 32)); ab = list(struct.unpack('<8I', mu.mem_read(DATA + 0x100, 32)))
    if av != bytes(cv) or ab != list(cb):
        bad += 1
        if bad <= 5: print('mismatch', length, pos, step, vol, add, [hex(x) for x in ab], [hex(x) for x in cb], av.hex(), bytes(cv).hex())
print('%d voices, %d mismatches, %.0f instructions per call' % (N, bad, count[0] / N))
sys.exit(1 if bad else 0)
