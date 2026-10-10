/* Replacement RP2040 bootrom: header and the 16-bit lookup tables the pico-sdk reads.
 * 16-bit relocations do not set the Thumb bit, so function entries add 1 explicitly. */
	.syntax unified
	.cpu cortex-m0plus
	.thumb

	.section .bootrom_header, "a"
	.word 0x20042000              /* 0x00 initial SP */
	.word xc_rom_hang             /* 0x04 reset (unused) */
	.word 0, 0
	.byte 'M', 'u', 0x01, 3       /* 0x10 magic, 0x13 version 3 (B2) */
	.hword xc_rom_func_table      /* 0x14 */
	.hword xc_rom_data_table      /* 0x16 */
	.hword xc_rom_table_lookup + 1 /* 0x18 */
	.hword 0

	.section .rodata.tables, "a"
	.balign 4
	.global xc_rom_func_table
xc_rom_func_table:
	.macro F a, b, fn
	.byte \a, \b
	.hword \fn + 1
	.endm
	F 'P', '3', rom_popcount32
	F 'R', '3', rom_reverse32
	F 'L', '3', rom_clz32
	F 'T', '3', rom_ctz32
	F 'M', 'S', rom_memset
	F 'S', '4', rom_memset
	F 'M', 'C', rom_memcpy
	F 'C', '4', rom_memcpy
	F 'I', 'F', rom_return0
	F 'E', 'X', rom_return0
	F 'R', 'E', rom_unsupported
	F 'R', 'P', rom_unsupported
	F 'F', 'C', rom_return0
	F 'C', 'X', rom_return0
	F 'U', 'B', rom_unsupported
	F 'D', 'T', rom_unsupported
	F 'D', 'E', rom_unsupported
	F 'W', 'V', rom_unsupported
	.hword 0, 0

	.global xc_rom_data_table
xc_rom_data_table:
	.byte 'S', 'F'
	.hword xc_rom_sf_table
	.byte 'S', 'D'
	.hword xc_rom_sd_table
	.byte 'C', 'R'
	.hword xc_rom_copyright
	.hword 0, 0

	.text
	.thumb_func
	.global xc_rom_hang
xc_rom_hang:
	b xc_rom_hang
