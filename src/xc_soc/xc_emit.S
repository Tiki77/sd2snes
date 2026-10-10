/* mk2: faster versions of the firmware's 65816 code emitter (the "MK2P" table redirects the originals here).
 *
 * The RP2040 firmware builds the 65816 code it streams to the SNES one byte at a time: emit8(b, x) at 0x100674A0
 * appends a byte to a buffer object b ({+0 base, +8 offset, +12 length}), emit16 appends two, and small helpers
 * append an opcode and its operand (LDA #, LDX #, STA dp, STA abs, STX abs, STZ abs). Every call pushes registers,
 * and on the mk2 core every store (pushes included) is an SRAM write through the bridge: a helper call cost 14
 * stores for 2-3 bytes of output. In the RTL simulation of the mk2 core these functions took about a fifth of the
 * frames that came out late. These versions are leaf functions without pushes (one store per output byte plus one
 * for the length).
 *
 * Same results as the originals: the bytes and the new length in the buffer object; r0 and r1 are preserved
 * (the originals preserve them too), r2 and r3 are clobbered (the veneer also uses ip). */
	.syntax unified
	.thumb
	.section .text.xc_emit, "ax"

/* r2 = base + offset + length of the buffer object r0 */
	.macro dst
	ldr r2, [r0, #0]
	ldr r3, [r0, #8]
	adds r2, r2, r3
	ldr r3, [r0, #12]
	adds r2, r2, r3
	.endm

/* length += n */
	.macro grow n
	ldr r3, [r0, #12]
	adds r3, r3, #\n
	str r3, [r0, #12]
	.endm

	.global xc_emit8
	.thumb_func
xc_emit8:                       /* 0x100674A0: byte r1 */
	dst
	strb r1, [r2, #0]
	grow 1
	bx lr

	.global xc_emit16
	.thumb_func
xc_emit16:                      /* 0x100674B2: halfword r1, low byte first */
	dst
	strb r1, [r2, #0]
	lsrs r3, r1, #8
	strb r3, [r2, #1]
	grow 2
	bx lr

/* opcode + 8-bit operand */
	.macro op8 name, opc
	.global \name
	.thumb_func
\name:
	dst
	movs r3, #\opc
	strb r3, [r2, #0]
	strb r1, [r2, #1]
	grow 2
	bx lr
	.endm

/* opcode + 16-bit operand */
	.macro op16 name, opc
	.global \name
	.thumb_func
\name:
	dst
	movs r3, #\opc
	strb r3, [r2, #0]
	strb r1, [r2, #1]
	lsrs r3, r1, #8
	strb r3, [r2, #2]
	grow 3
	bx lr
	.endm

	op8  xc_emit_lda_imm8, 0xA9   /* 0x100674D4 */
	op16 xc_emit_ldx_imm16, 0xA2  /* 0x100674EA */
	op8  xc_emit_sta_dp, 0x85     /* 0x10067500 */
	op16 xc_emit_sta_abs, 0x8D    /* 0x10067516 */
	op16 xc_emit_stx_abs, 0x8E    /* 0x1006752C */
	op16 xc_emit_stz_abs, 0x9C    /* 0x10067542 */
