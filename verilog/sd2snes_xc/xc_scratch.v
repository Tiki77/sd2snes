`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// xc_scratch (mk2): 4 KB of block RAM for RP2040 RAM 0x20040800-0x20040FFF and 0x20041800-0x20041FFF, the top
// halves of SCRATCH_X and SCRATCH_Y: the mixer's stack (xc_mix.ld) and core 0's stack (pico-sdk). Accesses
// there take one cycle and never go to the SRAM chip, so pushes and stack stores don't wait for the
// write-through. Word address {a[12], a[10:2]}.
//
// Two RAMB16_S9_S9 (2 KB, two 8-bit ports each) give 4 byte lanes per cycle with their own write enables:
// block 0 holds lanes 0 (port A) and 1 (port B), block 1 lanes 2 and 3; byte address {lane & 1, word}.
// Read: synchronous, raddr one cycle ahead (as the cache); the output reads 0 unless rd_en (block RAM output
// reset), so xc_soc can OR it with the cache's output, which reads 0 when rd_en is set. Write: one word with
// byte enables; while a write is done the read port reads the write address (the controller marks that read
// stale).
//////////////////////////////////////////////////////////////////////////////////
module xc_scratch (
  input clk,
  input [9:0] raddr,                // word address
  input rd_en,                      // the read is for this RAM (else q reads 0)
  output [31:0] q,
  input [3:0] we,                   // byte enables (0 = no write)
  input [9:0] waddr,
  input [31:0] wdata
);

wire w = |we;
wire [9:0] a = w ? waddr : raddr;

`ifdef VERILATOR
reg [7:0] m0 [0:1023], m1 [0:1023], m2 [0:1023], m3 [0:1023];
reg [7:0] q0, q1, q2, q3;
always @(posedge clk) begin
  if(we[0]) m0[a] <= wdata[7:0];
  if(we[1]) m1[a] <= wdata[15:8];
  if(we[2]) m2[a] <= wdata[23:16];
  if(we[3]) m3[a] <= wdata[31:24];
  if(!rd_en) begin q0 <= 8'd0; q1 <= 8'd0; q2 <= 8'd0; q3 <= 8'd0; end
  else begin q0 <= m0[a]; q1 <= m1[a]; q2 <= m2[a]; q3 <= m3[a]; end
end
assign q = {q3, q2, q1, q0};
`else
RAMB16_S9_S9 b0 (
  .CLKA(clk), .ENA(1'b1), .SSRA(~rd_en), .WEA(we[0]), .ADDRA({1'b0, a}), .DIA(wdata[7:0]), .DIPA(1'b0), .DOA(q[7:0]), .DOPA(),
  .CLKB(clk), .ENB(1'b1), .SSRB(~rd_en), .WEB(we[1]), .ADDRB({1'b1, a}), .DIB(wdata[15:8]), .DIPB(1'b0), .DOB(q[15:8]), .DOPB()
);
RAMB16_S9_S9 b1 (
  .CLKA(clk), .ENA(1'b1), .SSRA(~rd_en), .WEA(we[2]), .ADDRA({1'b0, a}), .DIA(wdata[23:16]), .DIPA(1'b0), .DOA(q[23:16]), .DOPA(),
  .CLKB(clk), .ENB(1'b1), .SSRB(~rd_en), .WEB(we[3]), .ADDRB({1'b1, a}), .DIB(wdata[31:24]), .DIPB(1'b0), .DOB(q[31:24]), .DOPB()
);
`endif

endmodule
