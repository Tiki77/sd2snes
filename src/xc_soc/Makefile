# xc_soc.bin: the Xeno Crisis soft CPU support file (sd2snes_xc core, both boards). Built
# with the mk3 and mk2 firmware (src/Makefile) and shipped next to it in /sd2snes/; the firmware loads it with the game (xc_load.c).
#   make [OUT=dir]   ->  $(OUT)/xc_soc.bin (also xc_fw.bin, xc_bootrom.bin, used by the image tools here)
#
# Contents:
#   0x0000 header: "XSOC", version 1, bootrom length, additions length (LE words), 0xFF to 512 bytes
#   0x0200 replacement RP2040 bootrom (xc_bootrom*), 0xFF to 16 KB      -> PSRAM 0xD20000
#   0x4200 firmware additions (mixer xc_mix.c, header, shim), 0xFF to 16 KB -> PSRAM 0xD24000 (RP2040 flash 0xF00000)
# Only needs arm-none-eabi-gcc (Cortex-M0+ code), like the firmware itself.

OUT ?= .
B := $(OUT)/xc_soc_build
CROSS ?= arm-none-eabi-
CC := $(CROSS)gcc
OBJCOPY := $(CROSS)objcopy
CFLAGS := -mcpu=cortex-m0plus -mthumb -mfloat-abi=soft -Os -ffreestanding -fno-builtin -ffunction-sections \
          -fdata-sections -Wall -Wextra -Werror -Wno-unused-parameter
AFLAGS := -mcpu=cortex-m0plus

FW_OBJ := $(B)/xc_fw_header.o $(B)/xc_mix.o $(B)/xc_shim.o $(B)/xc_mix_entry.o $(B)/xc_div.o $(B)/xc_brr_sw.o $(B)/xc_mix_voice.o $(B)/xc_emit.o $(B)/xc_tile.o
ROM_OBJ := $(B)/xc_bootrom_tables.o $(B)/xc_bootrom.o $(B)/xc_bootrom_flags.o $(B)/xc_bootrom_memcpy.o

all: $(OUT)/xc_soc.bin

$(B):
	mkdir -p $@

$(B)/%.o: %.c xc_soc.h | $(B)
	$(CC) $(CFLAGS) $(if $(filter xc_fw_header.c,$<),-Wno-strict-prototypes) -c $< -o $@

$(B)/%.o: %.S | $(B)
	$(CC) $(AFLAGS) -c $< -o $@

# mixer + firmware function replacements, linked at 0x10F00000 (free space in the flash image)
$(B)/xc_fw.elf: $(FW_OBJ) xc_mix.ld
	$(CC) -mcpu=cortex-m0plus -nostdlib -T xc_mix.ld -Wl,--gc-sections -Wl,-Map=$(B)/xc_fw.map $(FW_OBJ) -lgcc -o $@

# replacement RP2040 bootrom, linked at 0x00000000
$(B)/xc_bootrom.elf: $(ROM_OBJ) xc_bootrom.ld
	$(CC) -mcpu=cortex-m0plus -nostdlib -T xc_bootrom.ld -Wl,--gc-sections -Wl,-Map=$(B)/xc_bootrom.map \
	  -Wl,--undefined=xc_rom_func_table -Wl,--undefined=xc_rom_data_table $(ROM_OBJ) -lgcc -o $@

$(OUT)/xc_fw.bin: $(B)/xc_fw.elf
	$(OBJCOPY) -O binary -j .text $< $@

$(OUT)/xc_bootrom.bin: $(B)/xc_bootrom.elf
	$(OBJCOPY) -O binary $< $@

# the header needs the two lengths; each part is padded to 16 KB with 0xFF
$(OUT)/xc_soc.bin: $(OUT)/xc_bootrom.bin $(OUT)/xc_fw.bin xc_soc_hdr.S
	@b=$$(wc -c < $(OUT)/xc_bootrom.bin); f=$$(wc -c < $(OUT)/xc_fw.bin); \
	  if [ $$b -gt 16384 ] || [ $$f -gt 16384 ]; then echo "xc_soc: bootrom ($$b) or additions ($$f) over 16 KB"; exit 1; fi; \
	  $(CC) $(AFLAGS) -c xc_soc_hdr.S -Wa,--defsym,BOOTROM_LEN=$$b -Wa,--defsym,FW_LEN=$$f -o $(B)/xc_soc_hdr.o
	$(OBJCOPY) -O binary -j .data $(B)/xc_soc_hdr.o $(B)/hdr.bin
	$(OBJCOPY) -I binary -O binary --pad-to 16384 --gap-fill 0xff $(OUT)/xc_bootrom.bin $(B)/rom16k.bin
	$(OBJCOPY) -I binary -O binary --pad-to 16384 --gap-fill 0xff $(OUT)/xc_fw.bin $(B)/fw16k.bin
	cat $(B)/hdr.bin $(B)/rom16k.bin $(B)/fw16k.bin > $@
	@echo "  XC_SOC $@ ($$(wc -c < $@) bytes)"

clean:
	rm -rf $(B) $(OUT)/xc_soc.bin $(OUT)/xc_fw.bin $(OUT)/xc_bootrom.bin

.PHONY: all clean
