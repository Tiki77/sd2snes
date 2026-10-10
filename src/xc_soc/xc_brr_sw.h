/* Xeno Crisis on sd2snes mk2: BRR encoding in software (the mk2 core has no xc_brr block).
 *
 * The firmware's encoder (and xc_brr.v) tries all 11 shifts 12..2 on each 16-sample block, about 176 squared errors
 * per block and channel: too slow for the mk2's 20 MHz soft CPU at ~1,500 blocks per second. This one finds the
 * smallest shift at which no sample clips, and scores only that shift and the next smaller one with the firmware's
 * own nibble rule and error sum (the firmware tries 12 down to 2 and keeps a strictly better one). On the game's
 * sound effects (178,720 blocks from 60 s of play) 99.9% of the blocks come out identical; the rest are ties, with
 * the same error. The firmware's error sum can wrap at 32 bits on very loud blocks, which can make it pick a
 * clipping shift; this one does not follow it there.
 * On the mk2 core every store goes through to the SRAM chip, so the encoder keeps its work in registers.
 * Shared by xc_mix.c (Cortex-M0+) and the host test (test_brr_sw.c). */
#ifndef XC_BRR_SW_H
#define XC_BRR_SW_H
#include <stdint.h>

#define BRR_SW_INLINE static inline __attribute__((always_inline))

/* sample i of s[] (int16, every stride-th) >> 1, as the firmware's encoder starts */
#define BRR_SW_R7(s, stride, i) ((int32_t)(s)[(i) * (stride)] >> 1)

/* squared error of one shift (the firmware's nibble choice); stops early (returning a value above limit) once
   the sum exceeds limit */
BRR_SW_INLINE uint32_t brr_sw_err(const int16_t* s, int stride, int shift, uint32_t limit)
{
	int32_t half = (1 << shift) >> 1;
	int sm1 = shift - 1;
	uint32_t err = 0;
	for(int i = 0; i < 16; i++) {
		int32_t x = BRR_SW_R7(s, stride, i);
		/* for x >= 0 the firmware's "p" (rounded, at most 7) wins; for x < 0 its "n" (rounded, at least -8) */
		int32_t q = (2 * x + half) >> shift;
		if(q > 7) q = 7;
		else if(q < -8) q = -8;
		int32_t e = x - (q << sm1);
		err += (uint32_t)(e * e);
		if(err > limit) break;
	}
	return err;
}

/* 16 samples (s[0], s[stride], ...) -> BRR block: returns the header (shift << 4 | loop flag, no END flag); the
   8 nibble bytes go to *a (bytes 1-4 of the block, byte 1 in bits 31:24) and *b (bytes 5-8). fast: s0 only */
BRR_SW_INLINE uint32_t brr_sw_encode(const int16_t* s, int stride, uint32_t* a, uint32_t* b, int fast)
{
	int32_t mx = 0, mn = 0;
	for(int i = 0; i < 16; i++) {
		int32_t x = BRR_SW_R7(s, stride, i);
		if(x > mx) mx = x;
		if(x < mn) mn = x;
	}
	/* smallest shift without clipping: 2x + half < 7.5 * 2^shift and 2x + half >= -8 * 2^shift */
	int s0 = 2;
	while(s0 < 12 && (2 * mx + ((1 << s0) >> 1) >= (15 << (s0 - 1)) || 2 * mn + ((1 << s0) >> 1) < -(8 << s0))) s0++;
	/* s0 - 1 (some samples clip) can be better; s0 + 1 can only tie with s0 (then the firmware keeps s0 + 1),
	   which is rare and sounds the same, so it is not tried */
	int sh = s0;
	if(s0 > 2 && !fast) {
		uint32_t best = brr_sw_err(s, stride, s0, 0xFFFFFFFFu);
		if(best && brr_sw_err(s, stride, s0 - 1, best - 1) < best) sh = s0 - 1;
	}
	/* the nibbles of the chosen shift */
	int32_t half = (1 << sh) >> 1;
	uint32_t n0 = 0, n1 = 0;
	for(int i = 0; i < 16; i++) {
		int32_t q = (2 * BRR_SW_R7(s, stride, i) + half) >> sh;
		if(q > 7) q = 7;
		else if(q < -8) q = -8;
		n0 = n0 << 4 | n1 >> 28;
		n1 = n1 << 4 | (uint32_t)(q & 0x0F);
	}
	*a = n0; *b = n1;
	/* all nibbles 0 (silence, or samples too small for any shift): every larger shift gives the same, and the
	   firmware keeps the first it tried, 12 */
	return (uint32_t)(((n0 | n1) ? sh : 12) << 4 | 0x02);
}
#endif
