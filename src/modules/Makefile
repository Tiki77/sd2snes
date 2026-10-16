# The card modules (see ../module.h): menu features whose code is a file on the card.
# Built with every firmware so the set on the card always matches it; one image per module
# serves every board (Cortex-M3 code, which the STM32F4 also runs).
#   make [OUT=dir]   ->  $(OUT)/<module>.bin for each module below

OUT ?= .
B := $(OUT)/modules_build
CROSS ?= arm-none-eabi-
CC := $(CROSS)gcc
OBJCOPY := $(CROSS)objcopy
READELF := $(CROSS)readelf
# -fpic + hidden visibility: every reference is PC-relative.  -fno-builtin and no
# loop-to-memset: the images link without any library.
CFLAGS := -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -Os -fpic -fvisibility=hidden \
          -ffreestanding -fno-builtin -fno-tree-loop-distribute-patterns \
          -ffunction-sections -fdata-sections -Wall -Wextra -Werror -I..

MODULES := memtest pcmplay
memtest_SRC := memtest/mtmod.c
memtest_INC :=
pcmplay_SRC := pcmplay/pcmmod.c
pcmplay_INC :=

all: $(MODULES:%=$(OUT)/%.bin)

# $(1) = module name
define MODULE_RULES
$(B)/$(1)/%.o: $(1)/%.c ../module.h | $(B)/$(1)
	$$(CC) $$(CFLAGS) $$($(1)_INC) -c $$< -o $$@
$(B)/$(1)/%.o: ../%.c ../module.h | $(B)/$(1)
	$$(CC) $$(CFLAGS) $$($(1)_INC) -c $$< -o $$@
$(B)/$(1):
	mkdir -p $$@
# No -lgcc on purpose: a helper call (64-bit division, memcpy...) is a link error.
# The header's 4-byte words are the only absolute values allowed in the image.
$(B)/$(1)/$(1).elf: $$(addprefix $(B)/$(1)/,$$(notdir $$($(1)_SRC:.c=.o))) module.ld
	$$(CC) -mcpu=cortex-m3 -mthumb -nostdlib -T module.ld -Wl,--gc-sections -Wl,--emit-relocs \
	  -Wl,-Map=$(B)/$(1)/$(1).map $$(filter %.o,$$^) -o $$@
	@bad=$$$$($$(READELF) -r $$@ | awk '/R_ARM_ABS32/ { if (strtonum("0x" $$$$1) >= 32) print }'); \
	  if [ -n "$$$$bad" ]; then echo "$(1): absolute relocation outside the header:"; echo "$$$$bad"; exit 1; fi
$(OUT)/$(1).bin: $(B)/$(1)/$(1).elf
	$$(OBJCOPY) -O binary -j .text $$< $(B)/$(1)/$(1).raw
	python3 -c 'import sys, zlib, struct; d = bytearray(open(sys.argv[1], "rb").read()); \
	  struct.pack_into("<I", d, 12, zlib.crc32(bytes(d)) & 0xffffffff); open(sys.argv[2], "wb").write(d)' \
	  $(B)/$(1)/$(1).raw $$@
	@echo "  MODULE $$@ ($$$$(wc -c < $$@) bytes, state $$$$(python3 -c 'import sys,struct; print(struct.unpack_from("<H", open(sys.argv[1],"rb").read(), 6)[0])' $$@) bytes)"
endef
$(foreach m,$(MODULES),$(eval $(call MODULE_RULES,$(m))))

clean:
	rm -rf $(B) $(MODULES:%=$(OUT)/%.bin)

.PHONY: all clean
