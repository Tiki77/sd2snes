/* memcpy of the replacement bootrom (ROM table entries 'MC' and 'C4'), by hand.
 *
 * The C version compiled with -Os kept copies of its values on the stack (two extra stack stores per 16 bytes in
 * the aligned loop, one per word in the shifting loop, five pushes on entry). On the mk2 core every store is an
 * SRAM write through the bridge, and in the RTL simulation memcpy was about a tenth of the time of the frames that
 * took longer than 10 ms. This version has no stack stores at all when source and destination share their
 * alignment or the copy is short, and four pushes for the shifting loop.
 *
 *   r0 dst, r1 src, r2 n  ->  r0 dst (r1-r3, ip clobbered)
 *
 * Same behaviour as the C version: word stores when source and destination share their alignment; otherwise (16
 * bytes or more) bytes up to an aligned destination, then aligned words read from the source and shifted together
 * (reading at most up to the end of the word that holds the last source byte), then the rest byte by byte. */
	.syntax unified
	.thumb
	.section .text.rom_memcpy, "ax"

	.global rom_memcpy
	.type rom_memcpy, %function
	.thumb_func
rom_memcpy:
	mov ip, r0
	cmp r2, #16
	blo .Lbytes
	movs r3, r0
	eors r3, r1
	lsls r3, r3, #30
	bne .Lshift

	/* same alignment: bytes up to an aligned destination (n >= 16, at most 3) */
.Lalign_a:
	lsls r3, r0, #30
	beq .Lwords
	ldrb r3, [r1]
	strb r3, [r0]
	adds r0, r0, #1
	adds r1, r1, #1
	subs r2, r2, #1
	b .Lalign_a
.Lwords:
	subs r2, r2, #16
	blo .Lwords4
.Lw16:
	ldmia r1!, {r3}
	stmia r0!, {r3}
	ldmia r1!, {r3}
	stmia r0!, {r3}
	ldmia r1!, {r3}
	stmia r0!, {r3}
	ldmia r1!, {r3}
	stmia r0!, {r3}
	subs r2, r2, #16
	bhs .Lw16
.Lwords4:
	adds r2, r2, #12          /* n - 4 */
	blo .Ltail4
.Lw4:
	ldmia r1!, {r3}
	stmia r0!, {r3}
	subs r2, r2, #4
	bhs .Lw4
.Ltail4:
	adds r2, r2, #4
.Lbytes:
	cmp r2, #0
	beq .Ldone
.Lb:
	ldrb r3, [r1]
	strb r3, [r0]
	adds r0, r0, #1
	adds r1, r1, #1
	subs r2, r2, #1
	bne .Lb
.Ldone:
	mov r0, ip
	bx lr

	/* different alignment, n >= 16 */
.Lshift:
	lsls r3, r0, #30
	beq .Ls_go
	ldrb r3, [r1]
	strb r3, [r0]
	adds r0, r0, #1
	adds r1, r1, #1
	subs r2, r2, #1
	b .Lshift
.Ls_go:                       /* n >= 13, dst aligned, src not */
	push {r4-r7}
	lsls r5, r1, #30
	lsrs r5, r5, #27          /* r5 = 8 * (src & 3): right shift */
	movs r6, #32
	subs r6, r6, r5           /* r6 = left shift */
	lsrs r1, r1, #2
	lsls r1, r1, #2           /* aligned source word address */
	ldmia r1!, {r3}           /* cur */
	subs r2, r2, #4
.Ls_loop:
	ldmia r1!, {r4}           /* nxt */
	lsrs r3, r5
	movs r7, r4
	lsls r7, r6
	orrs r3, r7
	stmia r0!, {r3}
	movs r3, r4
	subs r2, r2, #4
	bhs .Ls_loop
	adds r2, r2, #4           /* 0..3 left */
	lsrs r5, r5, #3
	subs r1, r1, #4
	adds r1, r1, r5           /* back to the real source address */
	pop {r4-r7}
	b .Lbytes
	.size rom_memcpy, . - rom_memcpy
