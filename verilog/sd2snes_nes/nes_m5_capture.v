`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// nes_m5_capture -- display-start baseline + register-write event log of the MMC5
// (NES_MMC5 variant only; instantiated by that variant's MultiMapper in
// mmu_mmc5.v).  What it hands the video bridge, per frame:
//
//   base[175:0]  the MMC5 state (CMD_M5_STATE payload) at the moment the display
//                starts: it follows the live registers until the first ce with the
//                PPU rendering on a visible line, then freezes.  Writes made while
//                the PPU is not rendering (vblank, forced blank) are therefore part
//                of the baseline, never events.
//   ev[167:0]    up to K=8 register writes after the freeze, each
//                {scanline[7:0], idx[4:0], value[7:0]} (CMD_M5_EVENTS entries, in
//                write order; slot i at [i*21 +: 21]); `cnt` counts them, `ovf` is
//                set by the ninth.  Only scanlines 0..239 are logged; a write with
//                rendering off AFTER the freeze is logged too (the display state
//                it changes is needed once rendering comes back).
//
// One buffer, no frozen copy: the arrays are re-armed on the pre-render line
// (dot 300) of every frame, ~20 scanlines after the bridge accepts the frame at
// scanline ~241, and the bridge emits CMD_M5_STATE / CMD_M5_EVENTS in the first
// bytes of the frame it accepts, so a frame that is serialized always reads the
// arrays of the frame that just ended.  A frame the bridge does not accept is
// simply overwritten by the next one: baseline plus events are absolute per frame.
//
// Every sequential element is gated by ce (NES:core multicycle contract).
//////////////////////////////////////////////////////////////////////////////////
module nes_m5_capture(input clk, input ce, input reset,
                      input [8:0] scanline, input [8:0] cycle, input rendering,
                      input [175:0] state,
                      input ev_we, input [4:0] ev_idx, input [7:0] ev_val,
                      output [175:0] base_o,
                      output [167:0] ev_o,
                      output [3:0] cnt_o,
                      output ovf_o);
  reg [175:0] base;
  reg [20:0]  evr [0:7];
  reg [3:0]   cnt;
  reg         ovf;
  reg         frozen;
  integer i;
  initial begin
    base = 176'd0; cnt = 4'd0; ovf = 1'b0; frozen = 1'b0;
    for (i = 0; i < 8; i = i + 1) evr[i] = 21'd0;
  end
  wire vis = (scanline < 9'd240);
  wire arm = (scanline == 9'h1FF) && (cycle == 9'd300);
  always @(posedge clk) begin
    if (reset) begin frozen <= 1'b0; cnt <= 4'd0; ovf <= 1'b0; end
    else if (ce) begin
      if (arm) begin
        frozen <= 1'b0; cnt <= 4'd0; ovf <= 1'b0;
      end else begin
        if (!frozen) begin
          base <= state;
          if (vis && rendering) frozen <= 1'b1;
        end
        if (ev_we && vis && (frozen || rendering)) begin
          if (cnt < 4'd8) begin
            evr[cnt[2:0]] <= {scanline[7:0], ev_idx, ev_val};
            cnt <= cnt + 4'd1;
          end else ovf <= 1'b1;
        end
      end
    end
  end
  assign base_o = base;
  assign cnt_o = cnt;
  assign ovf_o = ovf;
  genvar g;
  generate for (g = 0; g < 8; g = g + 1) begin : ge
    assign ev_o[g*21 +: 21] = evr[g];
  end endgenerate
endmodule
