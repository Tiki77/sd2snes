`timescale 1ns / 1ps
// xc_tick: mixer tick timer for the soft CPU (XC_TICK_PERIOD at 0x50802000, microseconds, 0 = off).
// Raises a one-cycle irq pulse every period. The clk frequency is CLK_NUM / CLK_DEN MHz (any fraction, e.g. 161/4 =
// 40.25 MHz from the sd2snes PLL); a phase accumulator keeps the microsecond count exact on average.
// CLK_DEN = 1 behaves exactly like a plain divide-by-CLK_NUM counter.
// The event register at +0x08 (statistics in MesenCE) is accepted and ignored.
module xc_tick #(parameter CLK_NUM = 40, parameter CLK_DEN = 1) (
  input clk,
  input rst,
  input sel,
  input we,
  input [3:2] addr,
  input [31:0] wdata,
  output [31:0] rdata,
  output reg irq
);
reg [31:0] period_us;
reg [31:0] us_left;
reg [15:0] div;                  // phase accumulator: + CLK_DEN per clock, one microsecond per CLK_NUM
assign rdata = (addr == 2'd0) ? period_us : 32'd0;
always @(posedge clk) begin
  irq <= 1'b0;
  if(rst) begin
    period_us <= 32'd0; us_left <= 32'd0; div <= 16'd0;
  end else if(sel && we && addr == 2'd0) begin
    period_us <= wdata; us_left <= wdata; div <= 16'd0;
  end else if(period_us != 32'd0) begin
    if(div + CLK_DEN >= CLK_NUM) begin
      div <= div + CLK_DEN - CLK_NUM;
      if(us_left <= 32'd1) begin us_left <= period_us; irq <= 1'b1; end
      else us_left <= us_left - 32'd1;
    end else div <= div + CLK_DEN;
  end
end
endmodule
