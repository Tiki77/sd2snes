/* Software replacements for the pico-sdk divider functions (hardware/divider.S in the game firmware), for cores
 * without the RP2040 SIO hardware divider (sd2snes mk2). They are in the mk2 patch table of xc_fw_header.c, which
 * only the mk2 loader applies; the mk3 cores keep the hardware divider and the original functions.
 *
 * Results match the originals register for register where the callers can see them:
 *   32-bit: quotient r0, remainder r1; r2 = SIO base, r3 = 0 on return (as the originals leave them).
 *   64-bit: quotient r0:r1, remainder r2:r3.
 *   Signed: truncating division, remainder with the sign of the dividend; INT_MIN / -1 = INT_MIN, remainder 0.
 *   Division by zero: the same saturated value in r0 (r0:r1) as the originals, passed to the firmware's
 *   __aeabi_idiv0 / __aeabi_ldiv0 (whose result is returned), remainder 0.
 * The nonzero cases use libgcc's unsigned division (__aeabi_uidivmod, __aeabi_uldivmod). */
	.syntax unified
	.cpu cortex-m0plus
	.thumb

	.equ SIO_BASE,   0xD0000000
	.equ FW_IDIV0,   0x1006E650 + 1      /* firmware __aeabi_idiv0 (veneer to RAM) */
	.equ FW_LDIV0,   0x1006E5A0 + 1      /* firmware __aeabi_ldiv0 */

	.section .text.xc_div, "ax"

/* ---- divmod_s32s32 (0x10067EE8): r0 / r1 ---- */
	.global xc_sdiv32
	.thumb_func
xc_sdiv32:
	cmp r1, #0
	beq 8f
	push {r4, r5, lr}
	asrs r4, r0, #31          /* r4 = dividend sign (0 / -1) */
	asrs r5, r1, #31          /* r5 = divisor sign */
	eors r0, r4
	subs r0, r0, r4           /* |dividend| (INT_MIN stays 0x80000000) */
	eors r1, r5
	subs r1, r1, r5           /* |divisor| */
	eors r5, r4               /* quotient sign */
	bl __aeabi_uidivmod       /* r0 = q, r1 = r (unsigned) */
	eors r0, r5
	subs r0, r0, r5
	eors r1, r4
	subs r1, r1, r4
	ldr r2, =SIO_BASE
	movs r3, #0
	pop {r4, r5, pc}
8:	ldr r2, =SIO_BASE          /* x / 0: as the original (0x10067F04) */
	movs r3, #0
	push {r2, lr}
	movs r1, #1
	lsls r1, r1, #31
	asrs r2, r0, #31
	eors r1, r2
	cmp r0, #0
	beq 1f
	mvns r0, r1               /* x < 0: INT_MIN, x > 0: INT_MAX */
1:	ldr r3, =FW_IDIV0
	mov ip, r3
	movs r3, #0               /* r3 = 0 at the handler, as in the original */
	blx ip
	movs r1, #0
	pop {r2, pc}

/* ---- divmod_u32u32 (0x10067F38): r0 / r1 ---- */
	.global xc_udiv32
	.thumb_func
xc_udiv32:
	cmp r1, #0
	beq 8f
	push {r4, lr}
	bl __aeabi_uidivmod
	ldr r2, =SIO_BASE
	movs r3, #0
	pop {r4, pc}
8:	ldr r2, =SIO_BASE          /* x / 0: as the original (0x10067F54) */
	movs r3, #0
	push {r2, lr}
	cmp r0, #0
	beq 1f
	movs r0, #0
	mvns r0, r0               /* 0xFFFFFFFF */
1:	ldr r3, =FW_IDIV0
	mov ip, r3
	movs r3, #0               /* r3 = 0 at the handler, as in the original */
	blx ip
	movs r1, #0
	pop {r2, pc}

/* ---- divmod_u64u64 (0x10067FB4): r0:r1 / r2:r3 ---- */
	.global xc_udiv64
	.thumb_func
xc_udiv64:
	push {r4, lr}
	movs r4, r2
	orrs r4, r3
	beq 8f
	bl __aeabi_uldivmod       /* r0:r1 = q, r2:r3 = r */
	pop {r4, pc}
8:	movs r4, r0               /* x / 0: as the original (0x10068092) */
	orrs r4, r1
	beq 1f
	movs r0, #0
	mvns r0, r0
	movs r1, r0               /* 0xFFFFFFFF'FFFFFFFF */
1:	ldr r4, =FW_LDIV0
	blx r4
	movs r2, #0
	movs r3, #0
	pop {r4, pc}

/* ---- divmod_s64s64 (0x10067F84): r0:r1 / r2:r3 ---- */
	.global xc_sdiv64
	.thumb_func
xc_sdiv64:
	push {r4, r5, r6, lr}
	movs r4, r2
	orrs r4, r3
	beq 8f
	asrs r4, r1, #31          /* r4 = dividend sign */
	asrs r5, r3, #31          /* r5 = divisor sign */
	eors r0, r4               /* |dividend|: (x ^ s) - s over 64 bits */
	eors r1, r4
	subs r0, r0, r4
	sbcs r1, r1, r4
	eors r2, r5
	eors r3, r5
	subs r2, r2, r5
	sbcs r3, r3, r5
	eors r5, r4               /* quotient sign */
	bl __aeabi_uldivmod
	eors r0, r5
	eors r1, r5
	subs r0, r0, r5
	sbcs r1, r1, r5
	eors r2, r4
	eors r3, r4
	subs r2, r2, r4
	sbcs r3, r3, r4
	pop {r4, r5, r6, pc}
8:	movs r4, r0               /* x / 0: as the original (0x1006800E) */
	orrs r4, r1
	beq 1f
	movs r0, #0
	lsrs r1, r1, #31
	lsls r1, r1, #31          /* x < 0: 0x80000000'00000000 */
	bne 1f
	mvns r0, r0
	lsrs r1, r0, #1           /* x > 0: 0x7FFFFFFF'FFFFFFFF */
1:	ldr r4, =FW_LDIV0
	blx r4
	movs r2, #0
	movs r3, #0
	pop {r4, r5, r6, pc}
	.pool
