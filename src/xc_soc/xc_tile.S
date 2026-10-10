/* mk2: the game's tile-flag loops, faster (the "MK2P" table puts a veneer on their outer loop heads).
 *
 * Two places in the firmware (outer loop heads 0x1000BB10 and 0x1000C308) set bit 12 (0x1000) in a rectangle of a
 * 32 x 64 halfword table (0x20016360 and 0x20015360): rows [sp+8] .. r1-1, columns r3 .. r0-1, entry
 * (((col >> 5) << 10) + (col & 31) + row * 32) & 0x7FF. The compiled loop keeps its values on the stack and does one
 * halfword read-modify-write per entry; in the RTL simulation of the mk2 core the two loops were about 6% of the
 * frames that took longer than 10 ms. This version keeps everything in registers and handles two neighbouring
 * entries with one word read-modify-write (every store is an SRAM write on the mk2 core).
 *
 * It leaves exactly the state the original leaves at the loop exit (0x1000BB18 / 0x1000C310): the same table
 * contents, r0, r1, r3, r4 unchanged, r2 = [sp+8] = the row after the last one, r5 = the column after the last
 * one, r6 = the byte offset of the last entry, r7 = the table, ip = (last column & 31) + last row * 32, [sp+12] =
 * the table, [sp+16] = last row * 32 (r5, r6, r7, ip and [sp+12/16] keep their values where the original loops
 * did not run). The veneer at the loop head uses r2, which the loop head loads anyway. */
	.syntax unified
	.thumb
	.section .text.xc_tile, "ax"

	.macro tile_entry name, table, ret
	.global \name
	.thumb_func
\name:
	push {r0, r1, r3, r4, r5}
	ldr r4, =\table
	ldr r5, =\ret
	b tile_mark
	.endm

	tile_entry xc_tile_mark_a, 0x20016360, 0x1000BB19
	tile_entry xc_tile_mark_b, 0x20015360, 0x1000C311

	.equ O, 28                   /* caller's sp = sp + O inside the loops */
	.thumb_func
tile_mark:
	ldr r2, [sp, #20 + 8]        /* row = [sp+8] */
	push {r2, r5}                /* [sp] first row, [sp+4] return address; r0 r1 r3 r4 r5 at sp+8 .. sp+24 */
row_loop:
	ldr r1, [sp, #12]
	cmp r1, r2
	ble exit                     /* while row < r1 (signed, as the original) */
	lsls r5, r2, #5
	str r5, [sp, #O + 16]        /* [sp+16] = row * 32 */
	str r4, [sp, #O + 12]        /* [sp+12] = table */
	ldr r5, [sp, #16]            /* column = r3 */
	ldr r0, [sp, #8]             /* column end = r0 */
col_loop:
	cmp r0, r5
	ble row_end                  /* while column < r0 (signed) */
	movs r3, #31
	ands r3, r5                  /* c = column & 31 */
	ldr r1, [sp, #O + 16]
	adds r1, r1, r3
	mov ip, r1                   /* c + row * 32 */
	asrs r6, r5, #5
	lsls r6, r6, #10
	adds r6, r6, r1
	lsls r6, r6, #21
	lsrs r6, r6, #20             /* byte offset of the entry */
	mov r7, r4
	lsls r1, r6, #30
	bmi 2f                       /* odd entry: alone */
	cmp r3, #31
	beq 2f                       /* last column of a 32-column half: the next one is elsewhere */
	adds r1, r5, #1
	cmp r0, r1
	ble 2f                       /* last column of the row */
	ldr r1, [r4, r6]             /* two entries in one word */
	ldr r3, =0x10001000
	orrs r1, r3
	str r1, [r4, r6]
	adds r5, r5, #2
	adds r6, r6, #2
	mov r1, ip
	adds r1, r1, #1
	mov ip, r1
	b col_loop
2:	ldrh r1, [r4, r6]
	movs r3, #1
	lsls r3, r3, #12
	orrs r1, r3
	strh r1, [r4, r6]
	adds r5, r5, #1
	b col_loop
row_end:
	adds r2, r2, #1
	str r2, [sp, #O + 8]         /* [sp+8] = next row */
	b row_loop
exit:
	ldr r1, [sp, #0]
	cmp r1, r2
	bne 1f
	ldr r5, [sp, #24]            /* no row: r5 as it was */
1:	ldr r1, [sp, #4]
	str r1, [sp, #24]            /* return address where pop {pc} takes it */
	add sp, #8
	pop {r0, r1, r3, r4}
	pop {pc}
	.pool
