CORE = nes_mmc5

# Variant of the NES core for MMC5 (mapper 5) cartridges: the sd2snes_nes source with
# NES_MMC5 defined (the VERILOG_MACRO of this main.qsf), plus mmu_mmc5.v and
# nes_m5_capture.v of the parent directory. NES_MMC5 swaps the mapper multiplexer for the
# MMC5 alone (the mapper family of the base core does not fit next to it) and emits the MMC5
# state/event commands of the video bridge in place of the CHR window ones. mk3 only, like the NES core itself.
VSRC_DIR = ../sd2snes_nes

VSRC = address.v dac.v dcm.v main.v mcu_cmd.v msu.v sd_dma.v spi.v \
       nes_wrap.v nes_bridge.v nes_split_capture.v nes_chrsplit_capture.v nes_chrwin_capture.v nes_ppusplit_capture.v nes.v cpu.v MicroCode.v ppu.v apu.v mmu.v mmu_mmc5.v nes_m5_capture.v compat.v
VHSRC =

HEADER = ../sd2snes_nes/config.vh

COMMON_IP = dac_buf msu_databuf snescmd_buf

XIL_IP =
XIL_IPCORE_DIR = ../sd2snes_nes/ip/mk2

INT_IP = pll
INT_IP_DIR = ../sd2snes_nes/ip/mk3

include ../settings.mk
include ../common.mk
