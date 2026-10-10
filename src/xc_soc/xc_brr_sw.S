/* Xeno Crisis on sd2snes mk2: the software BRR encoder of xc_brr_sw.h in assembly (Cortex-M0+).
 *
 * The same algorithm and results as brr_sw_encode() in xc_brr_sw.h (the host test checks the C version against
 * the firmware's encoder, test_brr_sw_asm.py checks this one against the C version). Written by hand because on
 * the mk2 core every store goes through to the SRAM chip: the compiled C spills registers to the stack inside
 * its loops (~50 stores per block); this keeps everything in registers (9 stores per call: the saved registers
 * and the two result words).
 *
 *   uint32_t xc_brr_encode(const int16_t* s, uint32_t stride_bytes, uint32_t* ab);
 *   16 samples s[0], s[stride], ... -> returns the header (shift << 4 | 0x02); ab[0] = BRR bytes 1-4 (byte 1 in
 *   bits 31:24), ab[1] = bytes 5-8. stride_bytes | 1: fast mode, s0 only (no error passes; the mixer uses it
 *   when it has fallen behind: about 10% of the game's blocks then come out with ~1 dB more noise).
 *
 * Passes over the 16 samples: min/max (gives s0, the smallest shift without clipping); s0: nibbles, and the error
 * unless a lower bound of the error at s0 - 1 (the clipping of the largest and smallest sample) already reaches
 * the most the error at s0 can be; s0 - 1: error only, skipped when the bound reaches the error at s0, stopped once
 * it does; nibbles again only when s0 - 1 wins.
 * Registers: r0 sample pointer, r1 temp, r2 shift, r3 nibbles 1-4 / limit, r4 error, r5 nibbles 5-8, r6 x,
 * r7 q + 8 (collected as is; the nibble is (q + 8) ^ 8, one XOR with 0x88888888 at the end); r8 half, r9 stride, r10 ab, r11 start of the samples, r12 bound / best
 * error, lr end pointer / "nibbles not all 0". */
	.syntax unified
	.cpu cortex-m0plus
	.thumb

/* one sample at shift r2: r6 = x, r7 = clamped q + 8 (0..15); falls through */
	.macro QUANT
	ldrh r6, [r0]
	sxth r6, r6
	asrs r6, r6, #1
	lsls r7, r6, #1
	add r7, r8
	asrs r7, r7, r2          /* q = (2x + half) >> shift */
	adds r7, r7, #8
	bpl 91f
	movs r7, #0              /* q < -8 */
	b 92f
91:	cmp r7, #15
	ble 92f
	movs r7, #15             /* q > 7 */
92:
	.endm

/* squared error of the sample: r4 += (x - (q << (shift - 1)))^2 (clobbers r7) */
	.macro SQERR
	subs r7, r7, #8
	lsls r7, r7, r2
	asrs r7, r7, #1
	subs r7, r6, r7
	muls r7, r7, r7
	adds r4, r4, r7
	.endm

/* r8 = half = (1 << shift) >> 1, r0 = start, lr = middle (after 8 samples), r4 = 0 */
	.macro PASS_INIT
	movs r1, #1
	lsls r1, r1, r2
	asrs r1, r1, #1
	mov r8, r1
	mov r0, r11
	mov r1, r9
	lsls r1, r1, #3
	add r1, r11
	mov lr, r1
	movs r4, #0
	.endm

/* lr = end (after 16 samples) */
	.macro PASS_SECOND_HALF
	mov r1, r9
	lsls r1, r1, #3
	add lr, r1
	.endm

/* nibbles (q + 8 per sample, samples 0-7 in r3, 8-15 in r5, first sample in the top bits) to ab[] as BRR nibbles
   (q + 8) ^ 8; lr = nibbles not all 0 */
	.macro PASS_STORE
	ldr r1, =0x88888888
	eors r3, r1
	eors r5, r1
	mov r1, r10
	str r3, [r1]
	str r5, [r1, #4]
	orrs r3, r5
	mov lr, r3
	.endm

/* error and nibbles at shift r2 (no limit) */
	.macro PASS_NIB
	PASS_INIT
93:	QUANT
	lsls r3, r3, #4
	orrs r3, r7
	SQERR
	add r0, r9
	cmp r0, lr
	bne 93b
	PASS_SECOND_HALF
95:	QUANT
	lsls r5, r5, #4
	orrs r5, r7
	SQERR
	add r0, r9
	cmp r0, lr
	bne 95b
	PASS_STORE
	.endm

/* nibbles only at shift r2 */
	.macro PASS_NIBONLY
	PASS_INIT
94:	QUANT
	lsls r3, r3, #4
	orrs r3, r7
	add r0, r9
	cmp r0, lr
	bne 94b
	PASS_SECOND_HALF
96:	QUANT
	lsls r5, r5, #4
	orrs r5, r7
	add r0, r9
	cmp r0, lr
	bne 96b
	PASS_STORE
	.endm

	.section .text.xc_brr_encode, "ax"
	.global xc_brr_encode
	.thumb_func
xc_brr_encode:
	push {r4, r5, r6, r7, lr}
	mov r4, r8
	mov r5, r9
	mov r6, r10
	mov r7, r11
	push {r4, r5, r6, r7}
	movs r4, #1
	ands r4, r1              /* bit 0 of the stride: fast mode */
	mov r12, r4
	bics r1, r4
	mov r9, r1
	mov r10, r2
	mov r11, r0
	/* min and max of s[i] (then >> 1: x = s[i] >> 1), two samples per round */
	lsls r5, r1, #4          /* end offset */
	movs r2, #0              /* offset */
	movs r3, #0              /* max */
	movs r4, #0              /* min */
1:	ldrsh r6, [r0, r2]
	cmp r6, r3
	ble 2f
	mov r3, r6
2:	cmp r6, r4
	bge 3f
	mov r4, r6
3:	adds r2, r2, r1
	ldrsh r6, [r0, r2]
	cmp r6, r3
	ble 22f
	mov r3, r6
22:	cmp r6, r4
	bge 23f
	mov r4, r6
23:	adds r2, r2, r1
	cmp r2, r5
	bne 1b
	asrs r3, r3, #1
	asrs r4, r4, #1
	/* s0: the smallest shift (2..12) at which no sample clips (12 if none):
	   2 * (2 max + half) < 15 << shift and 2 min + half >= -(8 << shift); both only get easier with a larger
	   shift, so step down from 12 while the next smaller one does not clip (loud sounds need only a few steps) */
	lsls r5, r3, #1
	lsls r7, r4, #1
	movs r2, #12
4:	cmp r2, #2
	beq 6f
	subs r2, r2, #1
	movs r6, #1
	lsls r6, r6, r2
	asrs r6, r6, #1          /* half */
	adds r1, r5, r6
	lsls r1, r1, #1
	movs r0, #15
	lsls r0, r0, r2
	cmp r1, r0
	bge 5f                   /* clips: s0 is one more */
	adds r1, r7, r6
	movs r0, #8
	lsls r0, r0, r2
	negs r0, r0
	cmp r1, r0
	bge 4b
5:	adds r2, r2, #1
6:	/* at s0 = 2 there is no smaller shift to try; in fast mode s0 - 1 is not tried */
	cmp r2, #2
	beq 99168f
	mov r0, r12
	cmp r0, #0
	beq 99169f
99168:
	b 18f
99169:
	/* lower bound of the error at s0 - 1: the largest and the smallest sample clip there by
	   x - (7 << (s0 - 2)) and (-8 << (s0 - 2)) - x */
	subs r5, r2, #2
	movs r6, #7
	lsls r6, r6, r5
	subs r6, r3, r6
	bgt 13f
	movs r6, #0
13:	muls r6, r6, r6
	movs r7, #8
	lsls r7, r7, r5
	adds r7, r4, r7          /* min + (8 << (s0 - 2)) */
	bpl 15f
	muls r7, r7, r7
	adds r6, r6, r7
15:	mov r12, r6
	/* the error at s0 is at most 16 * (2^(s0 - 2))^2 = 2^(2 s0) (half a step per sample): if the bound reaches
	   that, s0 - 1 cannot win and the error at s0 is not needed (about 45% of the game's blocks) */
	movs r0, #1
	lsls r0, r0, r2
	lsls r0, r0, r2
	cmp r6, r0
	blo 99192f
	b 18f
99192:
	/* s0: error and nibbles */
	PASS_NIB
	/* s0 - 1 wins only when its error is strictly smaller */
	cmp r4, #0
	bne 99197f
	b 20f
99197:
	cmp r12, r4
	blo 9227f
	b 20f                    /* the bound already reaches the error at s0 */
9227:
	mov r12, lr              /* keep "not all 0" */
	subs r3, r4, #1          /* limit: best - 1 */
	subs r2, r2, #1
	PASS_INIT
	PASS_SECOND_HALF         /* lr = end: one loop over all 16 */
16:	QUANT
	SQERR
	cmp r4, r3
	bhi 17f                  /* not better */
	add r0, r9
	cmp r0, lr
	bne 16b
	/* s0 - 1 is better: its nibbles */
	PASS_NIB
	b 20f
17:	adds r2, r2, #1
	mov lr, r12
	b 20f
18:	/* nibbles only, at s0 */
	PASS_NIBONLY
20:	/* header; all nibbles 0: the firmware keeps shift 12 */
	mov r1, lr
	cmp r1, #0
	bne 21f
	movs r2, #12
21:	lsls r0, r2, #4
	adds r0, r0, #2
	pop {r4, r5, r6, r7}
	mov r8, r4
	mov r9, r5
	mov r10, r6
	mov r11, r7
	pop {r4, r5, r6, r7, pc}
	.pool
