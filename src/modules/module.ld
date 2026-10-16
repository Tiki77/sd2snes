/* A card module (../module.h): one read-only blob linked at 0, header first.  Code is
   built -fpic (PC-relative), so the image runs wherever the firmware loads it; the only
   absolute values are the header's offsets, which are relative to 0. */
SECTIONS
{
  . = 0;
  .text :
  {
    KEEP(*(.modhdr))
    *(.text .text.*)
    *(.rodata .rodata.*)
    . = ALIGN(4);
    __mod_end = .;
  }
  .data : { *(.data .data.*) *(.bss .bss.*) *(COMMON) }
  .got : { *(.got .got.*) }
  /DISCARD/ : { *(.ARM.exidx*) *(.ARM.attributes) *(.comment) }
}
ASSERT(SIZEOF(.data) == 0, "module: writable data is not allowed, state lives in the FRAME the firmware lends")
ASSERT(SIZEOF(.got) == 0, "module: GOT access, the image would not be position independent")
