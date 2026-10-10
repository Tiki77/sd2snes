`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// xc_brr: BRR encoder for the Xeno Crisis audio mixer (bit-exact with the RP2040 firmware's
// brute-force encoder at 0x20000A00 / XcAudio::EncodeBrrBlock with a zero fallback).
//
// For each 16-sample block it tries shift values 12..2 (filter 0), picks per sample the better of the
// two candidate nibbles (towards +/- rounding), and keeps the shift with the smallest total squared error.
// The firmware's integer quirks are kept on purpose:
//   - the per-shift error sum wraps at 32 bits and is compared as a signed number;
//   - a shift only wins when its (signed) error is strictly smaller than the best so far (first wins ties);
//   - the header is shift << 4 | 0x02 (loop flag), filter 0, no END flag (the mixer sets END itself).
//
// Timing: three-stage pipeline, one sample per clock: 11 shifts x 16 samples = 176 clocks + 3 per block.
//   stage 0: candidate nibbles (variable shift, clamp); stage 1: reconstruction error and squares;
//   stage 2: accumulate and pick the shift. (The squares were in stage 0 before; split for 40 MHz.)
//
// Soft-core register interface (word registers, byte address offsets):
//   0x00-0x3C  SAMPLE[i]  (write) sample i, low 16 bits used
//   0x40       CTRL       (write) bit 0: start encoding the 16 samples
//              STATUS     (read)  bit 0: busy
//   0x44       OUT0       (read)  bytes 0-3 of the block (byte 0 = header) in bits 7:0, 15:8, 23:16, 31:24
//   0x48       OUT1       (read)  bytes 4-7
//   0x4C       OUT2       (read)  byte 8 in bits 7:0
// Samples must not be written while busy (the mixer loads the next block after reading the result).
//////////////////////////////////////////////////////////////////////////////////
module xc_brr (
  input clk,
  input rst,
  input sel,
  input we,
  input [6:2] addr,
  input [31:0] wdata,
  output reg [31:0] rdata,
  output busy
);

reg signed [15:0] s [0:15];      // written by the core

reg running;
reg [3:0] sh;                    // shift of the item entering stage 1 (12..2)
reg [3:0] idx;                   // sample index entering stage 1
reg [71:0] out_block;            // result: byte 0 in bits 7:0

// stage 0 -> stage 1 registers
reg v0;
reg [3:0] sh0, idx0;
reg last0;
reg signed [14:0] r70;
reg [2:0] p0;
reg signed [4:0] n0;

// stage 1 -> stage 2 registers
reg v1;
reg [3:0] sh1, idx1;
reg last1;                       // last sample of a shift
reg [30:0] sqp1, sqn1;
reg [2:0] p1;
reg [3:0] nn1;                   // n & 0xF

// stage 2 state
reg [31:0] err;
reg [31:0] best;                 // compared as signed
reg found;
reg [3:0] best_sh;
reg [63:0] nibs;                 // nibbles of the current shift, sample i in bits 63-4i..60-4i
reg [63:0] best_nibs;
reg finishing;


//------------------------------------------------------------------------------
// Stage 0: candidate nibbles for sample idx at shift sh
//------------------------------------------------------------------------------
wire signed [15:0] cur = s[idx];
wire signed [14:0] r7 = cur[15:1];                        // s >> 1 (arithmetic)
wire [15:0] pu = {r7[14:0], 1'b0};                        // ((u32)r7 << 17) >> 16
wire [11:0] half = 12'd1 << (sh - 4'd1);                  // (1 << sh) >> 1
wire [16:0] psum = {1'b0, pu} + {5'd0, half};
wire [16:0] pshift = psum >> sh;
wire [2:0] p = (pshift > 17'd7) ? 3'd7 : pshift[2:0];
// n: ((((u32)r7 | 0xFFFF8000) << 1) + half) >> sh (arithmetic) = floor((pu - 65536 + half) / 2^sh)
wire signed [17:0] nsum = $signed({2'b00, pu}) - 18'sd65536 + $signed({6'd0, half});
wire signed [17:0] nshift = nsum >>> sh;
wire signed [4:0] n = (nshift < -18'sd8) ? -5'sd8 : (nshift > 18'sd7) ? 5'sd7 : nshift[4:0];
//------------------------------------------------------------------------------
// Stage 1: reconstruction errors and their squares (from the stage 0 registers)
//------------------------------------------------------------------------------
// reconstructions: (int16)((u32)x << sh & ~1) >> 1 == x << (sh - 1) for these ranges
wire signed [16:0] recp = $signed({14'd0, p0}) <<< (sh0 - 4'd1);
wire signed [16:0] recn = $signed({{12{n0[4]}}, n0}) <<< (sh0 - 4'd1);
wire signed [16:0] ep = $signed({{2{r70[14]}}, r70}) - recp;
wire signed [16:0] en = $signed({{2{r70[14]}}, r70}) - recn;
wire signed [33:0] sqp_full = ep * ep;
wire signed [33:0] sqn_full = en * en;

//------------------------------------------------------------------------------
// Stage 2: accumulate, pick the shift
//------------------------------------------------------------------------------
wire take_p = (sqp1 < sqn1);                              // both < 2^31: signed == unsigned compare
wire [31:0] err_next = err + (take_p ? {1'b0, sqp1} : {1'b0, sqn1});
wire [3:0] nib_now = take_p ? {1'b0, p1} : nn1;
wire [63:0] nibs_next = {nibs[59:0], nib_now};   // sample 0 ends up in bits 63:60

assign busy = running | v0 | v1 | finishing;

always @(posedge clk) begin
  if(rst) begin
    running <= 1'b0;
    v0 <= 1'b0;
    v1 <= 1'b0;
    finishing <= 1'b0;
    out_block <= 72'd0;
  end else begin
    // bus writes
    if(sel && we) begin
      if(addr[6] == 1'b0) begin
        if(!busy) s[addr[5:2]] <= wdata[15:0];
      end else if(addr[5:2] == 4'd0 && wdata[0] && !busy) begin
        running <= 1'b1;
        sh <= 4'd12;
        idx <= 4'd0;
        err <= 32'd0;
        nibs <= 64'd0;
        best <= 32'h7FFFFFFF;
        found <= 1'b0;
      end
    end

    // stage 0
    v0 <= running;
    if(running) begin
      sh0 <= sh; idx0 <= idx; last0 <= (idx == 4'd15);
      r70 <= r7; p0 <= p; n0 <= n;
      idx <= idx + 4'd1;
      if(idx == 4'd15) begin
        if(sh == 4'd2) running <= 1'b0;
        sh <= sh - 4'd1;
      end
    end

    // stage 1
    v1 <= v0;
    if(v0) begin
      sh1 <= sh0; idx1 <= idx0; last1 <= last0;
      sqp1 <= sqp_full[30:0]; sqn1 <= sqn_full[30:0];
      p1 <= p0; nn1 <= n0[3:0];
    end

    // stage 2
    finishing <= 1'b0;
    if(v1) begin
      if(last1) begin
        if($signed(err_next) < $signed(best)) begin
          best <= err_next;
          best_sh <= sh1;
          best_nibs <= nibs_next;
          found <= 1'b1;
        end
        err <= 32'd0;
        nibs <= 64'd0;
        if(sh1 == 4'd2) finishing <= 1'b1;
      end else begin
        err <= err_next;
        nibs <= nibs_next;
      end
    end
    if(finishing) begin
      out_block[7:0] <= found ? {best_sh, 4'b0010} : 8'h02;
      out_block[71:8] <= found ? {best_nibs[7:0], best_nibs[15:8], best_nibs[23:16], best_nibs[31:24],
                                  best_nibs[39:32], best_nibs[47:40], best_nibs[55:48], best_nibs[63:56]} : 64'd0;
    end
  end
end

always @* begin
  case(addr)
    5'h10: rdata = {31'd0, busy};
    5'h11: rdata = out_block[31:0];
    5'h12: rdata = out_block[63:32];
    5'h13: rdata = {24'd0, out_block[71:64]};
    default: rdata = 32'd0;
  endcase
end

endmodule
