CORE = xc

# Xeno Crisis: fpga_xc.bi3 (Mk.III: Opus decode mailbox and MSU-1, chosen at load time) and fpga_xc.bit
# (Mk.II: MSU-1 only). One source tree; the Mk.II differences are under `ifdef MK2 (macros MK2 | XC_MSU in
# sd2snes_xc.xise; main.qsf defines MK3, XC_MSU and XC_MK3). See README.md.
# VSRC is the Mk.II file list (it becomes main.prj); Quartus takes its list from main.qsf.
VSRC = address.v cheat.v dcm.v main.v mcu_cmd.v msu.v sd_dma.v spi.v xc_dac.v \
       xc_top.v xc_soc.v xc_m0.v xc_cache.v xc_scratch.v xc_bridge.v xc_window.v xc_stream.v xc_msubox.v
VHSRC =

HEADER = config.vh

COMMON_IP = dac_buf snescmd_buf

XIL_IP =
XIL_IPCORE_DIR = ip/mk2

INT_IP = msu_databuf
INT_IP_DIR = ip/mk3

include ../settings.mk
include ../common.mk

# the Mk.III-only sources (not part of the Mk.II project)
output_files/main.rbf: rtc.v xc_brr.v xc_tick.v xc_decbox.v xc_mk3_pll.v
