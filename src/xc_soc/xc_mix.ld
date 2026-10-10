/* Mixer blob: code in free flash (the image is empty from 0x10CE3000 to the save area at 0x10FF8000),
 * data and stack in SCRATCH_X (core 1's stack on the RP2040; unused here). */
MEMORY {
  FLASH (rx)  : ORIGIN = 0x10F00000, LENGTH = 64K
  SCRATCH (rw): ORIGIN = 0x20040000, LENGTH = 4K
}
ENTRY(xc_mix_install)
SECTIONS {
  .text : {
    KEEP(*(.text.xc_header))
    *(.text*) *(.rodata*)
    . = ALIGN(4);
  } > FLASH
  .bss (NOLOAD) : {
    __bss_start__ = .;
    KEEP(*(.bss.xc_dbg))   /* first: at 0x20040000, where the MCU reads it (xc_audio.c, MSU-1 log) */
    *(.bss*) *(COMMON)
    . = ALIGN(4);
    __bss_end__ = .;
  } > SCRATCH
  .data : { *(.data*) } > SCRATCH
  ASSERT(SIZEOF(.data) == 0, "the mixer must not have initialized data (nothing copies it)")
  __xc_stack_top = ORIGIN(SCRATCH) + LENGTH(SCRATCH);
  /DISCARD/ : { *(.ARM.exidx*) *(.comment) }
}
