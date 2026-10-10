MEMORY { ROM (rx) : ORIGIN = 0x00000000, LENGTH = 16K }
SECTIONS {
  .text : { KEEP(*(.bootrom_header)) *(.text*) *(.rodata*) . = ALIGN(4); } > ROM
  .data : { *(.data*) } > ROM
  .bss : { *(.bss*) *(COMMON) } > ROM
  ASSERT(SIZEOF(.data) == 0 && SIZEOF(.bss) == 0, "the bootrom must not have writable data")
  /DISCARD/ : { *(.ARM.exidx*) *(.comment) }
}
