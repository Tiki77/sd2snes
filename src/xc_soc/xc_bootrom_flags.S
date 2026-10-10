/* soft-float table entry "fcmp_fast_flags": compare r0, r1 as floats and return with the APSR flags set
 * like CMP a, b (unordered: N=0 Z=0 C=1 V=1), r0 unchanged. */
	.syntax unified
	.cpu cortex-m0plus
	.thumb
	.text
	.global xc_f_cmp_fast_flags
	.thumb_func
xc_f_cmp_fast_flags:
	push {r0, lr}
	bl xc_f_cmp_code       /* -1, 0, 1, or 2 for unordered */
	cmp r0, #2
	beq 3f
	cmp r0, #0
	bmi 1f
	beq 2f
	movs r0, #1            /* a > b: 1 - 0 -> N=0 Z=0 C=1 V=0 */
	movs r1, #0
	b 4f
1:	movs r0, #0            /* a < b: 0 - 1 -> N=1 Z=0 C=0 V=0 */
	movs r1, #1
	b 4f
2:	movs r0, #0            /* equal: Z=1 C=1 */
	movs r1, #0
	b 4f
3:	movs r0, #1            /* unordered: 0x80000000 - 1 -> N=0 Z=0 C=1 V=1 */
	lsls r0, r0, #31
	movs r1, #1
4:	cmp r0, r1
	pop {r0, pc}
