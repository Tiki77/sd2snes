`timescale 1 ns / 1 ns
//////////////////////////////////////////////////////////////////////////////////
// Company: Rehkopf
// Engineer: Rehkopf
//
// Create Date:    01:13:46 05/09/2009
// Design Name:
// Module Name:    address
// Project Name:
// Target Devices:
// Tool versions:
// Description: Address logic for the Xeno Crisis core: LoROM kernel (128 KB), $3000 window
//
// Dependencies:
//
// Revision:
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////
module address(
  input CLK,
  input [15:0] featurebits, // peripheral enable/disable
  input [2:0] MAPPER,       // MCU detected mapper
  input [23:0] SNES_ADDR,   // requested address from SNES
  input [7:0] SNES_PA,      // peripheral address from SNES
  input SNES_ROMSEL,        // ROMSEL from SNES
  output [23:0] ROM_ADDR,   // Address to request from SRAM0
  output ROM_HIT,           // enable SRAM0
  output IS_SAVERAM,        // address/CS mapped as SRAM?
  output IS_ROM,            // address mapped as ROM?
  output IS_WRITABLE,       // address somehow mapped as writable area?
  input [23:0] SAVERAM_MASK,
  input [23:0] ROM_MASK,
  output msu_enable,
  output r213f_enable,
  output r2100_hit,
  output snescmd_enable,
  output nmicmd_enable,
  output return_vector_enable,
  output branch1_enable,
  output branch2_enable,
  output branch3_enable,
  output xc_enable
);

parameter [2:0]
  //FEAT_DSPX = 0,
  //FEAT_ST0010 = 1,
  //FEAT_SRTC = 2,
  FEAT_MSU1 = 3,
  FEAT_213F = 4,
  FEAT_2100 = 6
;

wire [23:0] SRAM_SNES_ADDR;

assign IS_ROM = ~SNES_ROMSEL;

// The cartridge has no save RAM on the SNES side (saves are in the RP2040's flash, which the soft CPU
// keeps in the SRAM chip; the MCU saves it as the .srm).
assign IS_SAVERAM = 1'b0;
assign IS_WRITABLE = 1'b0;

// LoROM: 00-7D/80-FF:8000-FFFF. The kernel is the first 128 KB of the loaded image; the rest of the
// image (the RP2040 flash) must not be visible to the SNES, so the mask is fixed here.
assign SRAM_SNES_ADDR = {2'b00, SNES_ADDR[22:16], SNES_ADDR[14:0]} & ROM_MASK & 24'h01FFFF;

assign ROM_ADDR = SRAM_SNES_ADDR;

assign ROM_HIT = IS_ROM;

assign msu_enable = featurebits[FEAT_MSU1] & (!SNES_ADDR[22] && ((SNES_ADDR[15:0] & 16'hfff8) == 16'h2000));
assign r213f_enable = featurebits[FEAT_213F] & (SNES_PA == 8'h3f);
assign r2100_hit = (SNES_PA == 8'h00);
assign snescmd_enable = ({SNES_ADDR[22], SNES_ADDR[15:9]} == 8'b0_0010101);
assign nmicmd_enable = (SNES_ADDR == 24'h002BF2);
assign return_vector_enable = (SNES_ADDR == 24'h002A6C);
assign branch1_enable = (SNES_ADDR == 24'h002A1F);
assign branch2_enable = (SNES_ADDR == 24'h002A59);
assign branch3_enable = (SNES_ADDR == 24'h002A5E);
// 00-3F/80-BF:3000-3FFF: the RP2040 stream window (xc_window_decode in xc_stream.v)
assign xc_enable = (!SNES_ADDR[22] && (SNES_ADDR[15:12] == 4'h3));

endmodule
