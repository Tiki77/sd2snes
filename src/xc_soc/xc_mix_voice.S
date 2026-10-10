/* Xeno Crisis on sd2snes mk2: one sound effect voice through one 16-sample block, mono (the mk2 mixer, xc_mix.c).
 *
 *   void xc_mix_voice_mono(uint32_t v, uint32_t* buf, int add);
 *   v: the voice (firmware struct: +0 length, +4 samples, +8 step, +0x0C position, +0x10 volume, +0x1C playing,
 *   +0x1D ended); buf[k]: samples 2k (bits 15:0) and 2k + 1 (bits 31:16); add = 0: the first voice of the block
 *   (buf is written, not added to).
 *
 * The same results as mix_voice_mono() in C (the firmware's mixing loop: position >> 15 as the sample index, Q15
 * volume with the firmware's rounding, saturation), checked by test_mix_voice_asm.py. Written by hand because on
 * the mk2 core every store goes through to the SRAM chip: the compiled C spills to the stack in the loop; this
 * stores only the saved registers, the 8 result words and the position.
 *
 * Registers: r0 sample 2k+1, r1 buf pointer, r2 r4 temps, r3 position, r5 sample base (0 once the voice ended),
 * r6 volume, r7 sample 2k; r8 length, r9 step, r10 v, r11 0x8000, r12 end of buf, lr add. */
	.syntax unified
	.cpu cortex-m0plus
	.thumb

/* \d = the voice's next sample times the volume (0 once it has ended); advances the position */
	.macro SAMPLE d
	cmp r5, #0
	beq 90\@f
	asrs r4, r3, #31
	lsrs r4, r4, #17
	adds r4, r4, r3
	asrs r4, r4, #15         /* idx = position / 32768, towards 0 */
	cmp r4, r8
	bge 91\@f                /* length <= idx */
	cmp r4, #0
	blt 91\@f                /* position < -32767 */
	lsls r4, r4, #1
	ldrsh \d, [r5, r4]
	muls \d, r6, \d
	asrs r4, \d, #31
	lsrs r4, r4, #18
	adds r4, r4, \d
	asrs r4, r4, #14         /* q = p / 16384, towards 0 */
	lsrs \d, r4, #31
	adds \d, \d, r4
	asrs \d, \d, #1          /* h = q / 2, towards 0 */
	subs \d, r4, \d          /* q / 2 + q % 2 = q - h */
	add r3, r9
	b 92\@f
91\@:	mov r4, r10              /* the voice has ended */
	movs \d, #0
	strb \d, [r4, #0x1C]
	movs \d, #1
	strb \d, [r4, #0x1D]
	movs r5, #0
90\@:	movs \d, #0
92\@:
	.endm

/* \d = the next sample times the volume, advances the position; no end check (the fast path) */
	.macro QSAMPLE d
	asrs r4, r3, #15
	lsls r4, r4, #1
	ldrsh \d, [r5, r4]
	muls \d, r6, \d
	asrs r4, \d, #31
	lsrs r4, r4, #18
	adds r4, r4, \d
	asrs r4, r4, #14
	lsrs \d, r4, #31
	adds \d, \d, r4
	asrs \d, \d, #1
	subs \d, r4, \d
	add r3, r9
	.endm

/* saturation: outside -32767..32767 -> (x / 32768, towards 0) * 32760 (clobbers r2, r4) */
	.macro CLAMP x
	mov r2, r11
	adds r2, r2, \x          /* x + 0x8000 */
	lsrs r4, r2, #16
	bne 93\@f                /* x < -32768 or x > 32767 */
	cmp r2, #0
	bne 94\@f                /* x = -32768 also saturates */
93\@:	asrs r4, \x, #31
	lsrs r4, r4, #17
	adds r4, r4, \x
	asrs r4, r4, #15
	movs \x, #0xFF
	lsls \x, \x, #7
	adds \x, #0x78           /* 32760 */
	muls \x, r4, \x
94\@:
	.endm

	.section .text.xc_mix_voice_mono, "ax"
	.global xc_mix_voice_mono
	.thumb_func
xc_mix_voice_mono:
	push {r4, r5, r6, r7, lr}
	mov r4, r8
	mov r5, r9
	mov r6, r10
	mov r7, r11
	push {r4, r5, r6, r7}
	mov lr, r2
	mov r10, r0
	ldr r4, [r0, #0]
	mov r8, r4               /* length */
	ldr r4, [r0, #8]
	mov r9, r4               /* step */
	ldr r3, [r0, #0x0C]      /* position */
	ldr r5, [r0, #4]         /* samples */
	ldr r6, [r0, #0x10]      /* volume */
	movs r4, #0x80
	lsls r4, r4, #8
	mov r11, r4
	movs r4, #32
	adds r4, r4, r1
	mov r12, r4
	/* fast path, the usual case (one sound playing): first voice of the block, volume 0..0x7FFF (no saturation
	   possible), position >= 0, step 0..2^24 - 1 and the voice does not end within the block: no checks per
	   sample */
	mov r4, lr
	cmp r4, #0
	bne 1f
	lsrs r4, r6, #15
	bne 1f
	cmp r3, #0
	blt 1f
	mov r0, r9
	lsrs r4, r0, #24
	bne 1f                   /* step < 0 or >= 2^24 */
	lsls r4, r0, #4
	subs r4, r4, r0          /* 15 * step */
	adds r4, r4, r3          /* position of the 16th sample */
	bvs 1f
	asrs r4, r4, #15
	cmp r4, r8
	bge 1f
5:	QSAMPLE r7
	QSAMPLE r0
	uxth r7, r7
	lsls r0, r0, #16
	orrs r0, r7
	str r0, [r1]
	adds r1, r1, #4
	cmp r1, r12
	bne 5b
	b 4f
1:	SAMPLE r7
	SAMPLE r0
	mov r4, lr
	cmp r4, #0
	beq 2f
	ldr r4, [r1]
	sxth r2, r4
	adds r7, r7, r2
	asrs r4, r4, #16
	adds r0, r0, r4
2:	CLAMP r7
	CLAMP r0
	uxth r7, r7
	lsls r0, r0, #16
	orrs r0, r7
	str r0, [r1]
	adds r1, r1, #4
	cmp r5, #0
	bne 3f
	mov r4, lr
	cmp r4, #0
	bne 4f                   /* ended, adding: the rest keeps the other voices' samples */
3:	cmp r1, r12
	bne 1b
4:	mov r4, r10
	str r3, [r4, #0x0C]
	pop {r4, r5, r6, r7}
	mov r8, r4
	mov r9, r5
	mov r10, r6
	mov r11, r7
	pop {r4, r5, r6, r7, pc}
