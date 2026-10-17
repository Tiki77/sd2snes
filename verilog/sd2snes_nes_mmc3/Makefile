CORE = nes_mmc3

# fpga_nes_mmc3: MMC3, Namco 108, Taito TC0190/X1-005/X1-017 and MMC6.
# Variant of the NES core: the sd2snes_nes sources with the NES_NO_* macros of this
# main.qsf that remove the mapper families (and the MSU-1 data buffer) the core does not
# serve.  mk3 only, like sd2snes_nes: no .xise / .ucf.  The firmware picks the file by
# mapper in nes_core_for() (src/nes.c).  The renderer (nes_snes.bin) is the same for all of them.
VSRC_DIR = ../sd2snes_nes

VSRC = address.v dac.v dcm.v main.v mcu_cmd.v msu.v sd_dma.v spi.v \
       nes_wrap.v nes_bridge.v nes_split_capture.v nes_chrsplit_capture.v nes_chrwin_capture.v nes_ppusplit_capture.v nes.v cpu.v MicroCode.v ppu.v apu.v mmu.v compat.v
VHSRC =

HEADER = ../sd2snes_nes/config.vh

COMMON_IP = dac_buf msu_databuf snescmd_buf

XIL_IP =
XIL_IPCORE_DIR = ../sd2snes_nes/ip/mk2

INT_IP = pll
INT_IP_DIR = ../sd2snes_nes/ip/mk3

include ../settings.mk
include ../common.mk
