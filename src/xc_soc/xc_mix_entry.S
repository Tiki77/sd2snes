/* Interrupt wrappers: run the mixer on its own stack (top of SCRATCH_X), not on core 0's.
 * The core has no nesting, so one stack is enough. Hardware stacking already saved r0-r3, r12, lr, pc, xPSR. */
	.syntax unified
	.cpu cortex-m0plus
	.thumb

	.section .text.xc_irq, "ax"
	.global xc_irq_tick
	.global xc_irq_decoded
	.thumb_func
xc_irq_tick:
	ldr r1, =xc_mix_tick
	b 1f
	.thumb_func
xc_irq_decoded:
	ldr r1, =xc_mix_decoded
1:	mov r0, sp
	ldr r2, =__xc_stack_top
	mov sp, r2
	push {r0, lr}            /* old SP and EXC_RETURN */
	blx r1
	pop {r0, r1}
	mov sp, r0
	bx r1
	.pool

/* mk2: called by core 0's wait loops (xc_shim.c), all the time. Returns at once, without any store (on the mk2
 * core every store goes through to the SRAM chip), unless xc_mix_install_mk2() has run and either a mixer tick is
 * due or the left BRR ring has room for a block (and core 0 holds no lock); then runs xc_mix_poll_body() on the mixer's stack (never nested:
 * core 0 only). Uses only r0-r3 and r12, like any called function. */
	.equ BRR_LEFT_FREE, 0x2001B4EC          /* Brr + 0x6C8 (xc_mix.c) */
	.equ TIMERAWL, 0x40054028
	.equ LOCKS, 0x20022C67                  /* Locks (xc_mix.c): lock 0, lock 1 */
	.section .text.xc_mix_poll, "ax"
	.global xc_mix_poll
	.thumb_func
xc_mix_poll:
	ldr r0, =xc_mix_coop
	ldr r1, [r0]
	ldr r2, =0x434F4F50
	cmp r1, r2
	bne 2f
	ldr r1, [r0, #4]         /* next tick */
	ldr r2, =TIMERAWL
	ldr r2, [r2]
	subs r2, r2, r1
	bpl 1f                   /* due */
	ldr r2, =BRR_LEFT_FREE
	ldr r2, [r2]
	cmp r2, #8
	bls 2f                   /* rings full */
	ldr r2, =LOCKS
	ldrb r1, [r2]
	ldrb r2, [r2, #1]
	orrs r1, r2
	bne 2f                   /* core 0 holds a lock: no mixing now */
1:	mov r0, sp
	ldr r2, =__xc_stack_top
	mov sp, r2
	push {r0, lr}
	bl xc_mix_poll_body
	pop {r0, r1}
	mov sp, r0
	bx r1
2:	bx lr
	.pool
