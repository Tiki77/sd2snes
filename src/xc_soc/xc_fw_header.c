/* Header at 0x10F00000: tells MesenCE and xc_patch_image.py where to put what.
 *   word 0 magic "MXCX", word 1 version, word 2 xc_mix_install (unused since v2, kept for v1 tools),
 *   word 3 number of patches, then {firmware address, target, kind} per patch:
 *     kind 0: 16-byte veneer at the function entry: push {r0}; ldr r0, =target; mov ip, r0; pop {r0}; bx ip
 *     kind 1: the function is already such a veneer (flash functions): replace its target word at +12
 *     kind 2: "movs r0, #0; bx lr" (4 bytes)
 *     kind 3 (mk2 table only): write the 32-bit "target" word itself at the address (code patches)
 *   then the mk2 table: word "MK2P", number of patches (at most 64), {firmware address, target, kind 0 or 3} per
 *   patch. Only a core without the SIO hardware divider (sd2snes mk2) applies it. It is outside the main table
 *   because the loaders of the mk3 firmware and older MesenCE treat an unknown kind as kind 2. */
#include <stdint.h>

extern void xc_mix_install(void (*entry)(void)), xc_mix_install_mk2(void (*entry)(void));
extern int xc_bus_init(), xc_bus_flush(), xc_bus_wait_tx(), xc_bus_send(), xc_bus_recv(), xc_bus_push();
extern void xc_sleep_until(), xc_panic();
extern int xc_puts(), xc_printf();
extern void xc_flash_do_cmd(), xc_flash_range_erase(), xc_flash_range_program();
extern void xc_sdiv32(), xc_udiv32(), xc_sdiv64(), xc_udiv64();
extern void xc_emit8(), xc_emit16(), xc_emit_lda_imm8(), xc_emit_ldx_imm16(), xc_emit_sta_dp(), xc_emit_sta_abs(), xc_emit_stx_abs(),
	xc_emit_stz_abs(), xc_tile_mark_a(), xc_tile_mark_b();

#define P(addr, fn, kind) addr, (uint32_t)(fn), kind
#define W(addr, word) addr, (uint32_t)(word), 3

/* mk2: veneer without a stack store at a function entry, "ldr r3, =fn; bx r3" (for functions that clobber r3
   anyway); A4 at a word address (8 bytes), A2 at a halfword address (12 bytes from A - 2; the halfword before
   the function must be dead code, here the rest of the function before it that has a veneer too) */
#define A4(a, fn) W(a, 0x47184B00u), W((a) + 4, fn)                                /* ldr r3, [pc, #0]; bx r3 */
#define R2(a, fn) W(a, 0x47104A00u), W((a) + 4, fn)                                /* ldr r2, [pc, #0]; bx r2 */
#define A2(a, fn) W((a) - 2, 0x4B01BF00u), W((a) + 2, 0xBF004718u), W((a) + 6, fn) /* nop; ldr r3, [pc, #4]; bx r3; nop */
/* mk2: the emitter's one-line wrappers ("push {r4, lr}; ldr r3, =builder; <movs r1, ...>; ldr r0, [r3]; bl emit;
   pop {r4, pc}"; 20 bytes with the builder's address at +16) as a tail call without the push:
   ldr r3, [pc, #12]; <movs r1, ...>; ldr r0, [r3]; ldr r3, [pc, #4]; bx r3; nop; .word fn; (builder's address) */
#define WRAP(a, movs_r1, fn) W(a, 0x4B03u | (uint32_t)(movs_r1) << 16), W((a) + 4, 0x4B016818u), W((a) + 8, 0xBF004718u), \
	W((a) + 12, fn)
#define MOVS_R1_R0 0x0001u
#define MOVS_R1_EA 0x21EAu   /* movs r1, #0xEA (NOP) */

__attribute__((section(".text.xc_header"), used))
const uint32_t xc_fw_header[4 + 3 * 17 + 2 + 3 * 57] = {
	0x5843584Du, 2, (uint32_t)xc_mix_install, 17,
	P(0x10059060u, xc_mix_install, 0),        /* multicore_launch_core1 */
	P(0x10054288u, 0, 2),                     /* set_sys_clock_pll */
	P(0x10056EB6u, 0, 2),                     /* stdio_init_all */
	P(0x10056E3Cu, xc_puts, 0),
	P(0x10056EA0u, xc_printf, 0),
	P(0x100555C0u, xc_panic, 0),
	P(0x10054ECCu, xc_sleep_until, 0),
	P(0x10066D94u, xc_bus_init, 0),
	P(0x100670CCu, xc_bus_flush, 0),
	P(0x10067188u, xc_bus_wait_tx, 0),
	P(0x100671A4u, xc_bus_send, 0),
	P(0x100671F4u, xc_bus_recv, 0),
	P(0x10067274u, xc_bus_push, 0),
	P(0x1006E560u, xc_flash_do_cmd, 1),
	P(0x1006E610u, xc_flash_range_program, 1),
	P(0x1006E690u, xc_flash_range_erase, 1),
	P(0x10067E34u, 0, 2),                     /* core1_init_opus: never reached (no core 1); returns 0 if it were */
	/* mk2 table: the pico-sdk divider functions -> software division (xc_div.S). These are all the entry points
	   into the divider code from outside it; the other readers of the divider (the soft-float/double wrappers
	   around the bootrom calls) only read DIV_CSR, which reads 0 (not dirty) on the mk2 core. */
	0x50324B4Du, 57,                           /* "MK2P" */
	P(0x10067EE8u, xc_sdiv32, 0),             /* divmod_s32s32 (__aeabi_idivmod) */
	P(0x10067F38u, xc_udiv32, 0),             /* divmod_u32u32 (__aeabi_uidivmod) */
	P(0x10067FB4u, xc_udiv64, 0),             /* divmod_u64u64 (__aeabi_uldivmod) */
	P(0x10067F84u, xc_sdiv64, 0),             /* divmod_s64s64 (not called in this build; for completeness) */
	/* no tick interrupt or BRR encoder on the mk2 core: the mixer runs from core 0's wait loops (xc_mix.c) */
	P(0x10059060u, xc_mix_install_mk2, 0),    /* multicore_launch_core1, instead of xc_mix_install */
	/* the 65816 code emitter, without pushes (every store is an SRAM write on the mk2 core; xc_emit.S), entered
	   through veneers without a store; its wrappers (the builder in a global) tail-call it without a push */
	A4(0x100674A0u, xc_emit8),
	A2(0x100674B2u, xc_emit16),
	A4(0x100674D4u, xc_emit_lda_imm8),
	A2(0x100674EAu, xc_emit_ldx_imm16),
	A4(0x10067500u, xc_emit_sta_dp),
	A2(0x10067516u, xc_emit_sta_abs),
	A4(0x1006752Cu, xc_emit_stx_abs),
	A2(0x10067542u, xc_emit_stz_abs),
	WRAP(0x1006763Cu, MOVS_R1_R0, xc_emit8),
	WRAP(0x10067678u, MOVS_R1_EA, xc_emit8),
	WRAP(0x1006768Cu, MOVS_R1_R0, xc_emit_lda_imm8),
	WRAP(0x100676A0u, MOVS_R1_R0, xc_emit_ldx_imm16),
	WRAP(0x100676B4u, MOVS_R1_R0, xc_emit_sta_abs),
	WRAP(0x100676C8u, MOVS_R1_R0, xc_emit_stx_abs),
	WRAP(0x100676DCu, MOVS_R1_R0, xc_emit_stz_abs),
	/* the game's tile-flag loops (xc_tile.S): veneers on their outer loop heads, which load r2 anyway */
	R2(0x1000BB10u, xc_tile_mark_a),
	R2(0x1000C308u, xc_tile_mark_b),
};
