`timescale 1ns / 1ps
// dspx: the mk2-only flavour of the dsp core that keeps the event cart (Campus Challenge '92 /
// PowerFest '94) logic. The variant is selected by the DSP_COMP Verilog macro, which this
// project's .xise sets globally (Verilog Macros property) -- NOT by this header, because
// address.v and main.v derive their gate from the global macros. This file exists to keep
// the HEADER dependency of common.mk pointing at a file that lives with the project, like
// sd2snes_basex.
`ifndef _config_vh
`define _config_vh

`ifdef MK2
  `ifdef DEBUG
    `define MK2_DEBUG
  `endif
`endif

`endif
