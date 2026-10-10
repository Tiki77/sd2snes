`timescale 1ns / 1ps
// gsu3: the FX3-only flavour of the GSU core (pipelined gsu_fx3.v), mk2 and mk3.
// The variant is selected by the GSU3 Verilog macro, which this project's .xise
// (mk2) and main.qsf (mk3) set globally -- NOT by this header.  This file keeps the
// HEADER dependency of common.mk pointing at a file that lives with the project.
`ifndef _config_vh
`define _config_vh

`ifdef MK2
  `ifdef DEBUG
    `define MK2_DEBUG
  `endif
`endif

`endif
