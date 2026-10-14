CORE = dspx

# mk2-only variant of the dsp core: the same source with DSP_COMP defined (this project's
# Verilog Macros), i.e. with the Campus Challenge '92 / PowerFest '94 event board map,
# select latch and round timer. See address.v.
VSRC_DIR = ../sd2snes_dsp

VSRC = address.v cheat.v clk_test.v ctx.v dac.v dcm.v dma.v upd77c25.v main.v mcu_cmd.v msu.v sd_dma.v spi.v
VHSRC =

COMMON_IP = dac_buf msu_databuf snescmd_buf upd77c25_datram upd77c25_datrom upd77c25_pgmrom

HEADER = config.vh

XIL_IP =
XIL_IPCORE_DIR = ../sd2snes_dsp/ip/mk2

INT_IP = pll

include ../settings.mk
include ../common.mk
