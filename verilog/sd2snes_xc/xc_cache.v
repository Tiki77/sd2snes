`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// xc_cache: storage of a 2-way set-associative cache with 32-byte lines (the controller is in xc_soc.v).
//
//   SETS  number of sets (64 = 4 KB, 128 = 8 KB)
//   TAGW  tag width (address key bits above the index)
//
// Read port (synchronous, one cycle): rd_key selects the set and word; outputs for both ways the word,
// the tag and the valid/dirty bits. Data is stored as 4 byte-wide RAMs per way (byte writes, M9K).
// Write ports: one data word (with byte enables) and one tag entry per cycle.
// LRU: one bit per set in flip-flops (lru[s] = way to replace next), updated with touch.
//////////////////////////////////////////////////////////////////////////////////
module xc_cache #(
  parameter SETS = 128,
  parameter IDXW = 7,               // log2(SETS)
  parameter TAGW = 14
) (
  input clk,

  // read (registered outputs)
  input [IDXW+2:0] rd_word,         // {set, word in line}
`ifdef MK2
  input rd_zero,                    // data outputs read as 0 (block RAM output reset: the read is for xc_scratch)
`endif
  output [31:0] q0,
  output [31:0] q1,
  output [TAGW+1:0] t0,             // {valid, dirty, tag}
  output [TAGW+1:0] t1,

  // data write
  input dwe,
  input dway,
  input [IDXW+2:0] dword,
  input [3:0] dbe,
  input [31:0] dwdata,

  // tag write
  input twe,
  input tway,
  input [IDXW-1:0] tset,
  input [TAGW+1:0] twdata,

  // LRU
  input [IDXW-1:0] lru_set,
  output lru_way,                   // way to replace in lru_set
  input touch,                      // mark touch_way as most recently used in touch_set
  input [IDXW-1:0] touch_set,
  input touch_way,
  input inv_all_lru                 // reset LRU state
);

localparam WORDS = SETS * 8;

reg [7:0] d00 [0:WORDS-1], d01 [0:WORDS-1], d02 [0:WORDS-1], d03 [0:WORDS-1];
reg [7:0] d10 [0:WORDS-1], d11 [0:WORDS-1], d12 [0:WORDS-1], d13 [0:WORDS-1];
reg [TAGW+1:0] tg0 [0:SETS-1], tg1 [0:SETS-1];
`ifdef MK2
// mk2: LRU bits in distributed RAM (a replacement hint only, so it needs no reset: inv_all_lru is ignored)
(* ram_style = "distributed" *) reg lru [0:SETS-1];
`else
reg [SETS-1:0] lru;
`endif

reg [7:0] q00, q01, q02, q03, q10, q11, q12, q13;
reg [TAGW+1:0] rt0, rt1;
wire [IDXW-1:0] rd_set = rd_word[IDXW+2:3];

always @(posedge clk) begin
  if(dwe & ~dway & dbe[0]) d00[dword] <= dwdata[7:0];
  if(dwe & ~dway & dbe[1]) d01[dword] <= dwdata[15:8];
  if(dwe & ~dway & dbe[2]) d02[dword] <= dwdata[23:16];
  if(dwe & ~dway & dbe[3]) d03[dword] <= dwdata[31:24];
  if(dwe &  dway & dbe[0]) d10[dword] <= dwdata[7:0];
  if(dwe &  dway & dbe[1]) d11[dword] <= dwdata[15:8];
  if(dwe &  dway & dbe[2]) d12[dword] <= dwdata[23:16];
  if(dwe &  dway & dbe[3]) d13[dword] <= dwdata[31:24];
`ifdef MK2
  if(rd_zero) begin
    q00 <= 8'd0; q01 <= 8'd0; q02 <= 8'd0; q03 <= 8'd0; q10 <= 8'd0; q11 <= 8'd0; q12 <= 8'd0; q13 <= 8'd0;
  end else begin
    q00 <= d00[rd_word]; q01 <= d01[rd_word]; q02 <= d02[rd_word]; q03 <= d03[rd_word];
    q10 <= d10[rd_word]; q11 <= d11[rd_word]; q12 <= d12[rd_word]; q13 <= d13[rd_word];
  end
`else
  q00 <= d00[rd_word]; q01 <= d01[rd_word]; q02 <= d02[rd_word]; q03 <= d03[rd_word];
  q10 <= d10[rd_word]; q11 <= d11[rd_word]; q12 <= d12[rd_word]; q13 <= d13[rd_word];
`endif
end

always @(posedge clk) begin
  if(twe & ~tway) tg0[tset] <= twdata;
  if(twe &  tway) tg1[tset] <= twdata;
  rt0 <= tg0[rd_set];
  rt1 <= tg1[rd_set];
end

`ifdef MK2
always @(posedge clk) if(touch) lru[touch_set] <= ~touch_way;
`else
always @(posedge clk) begin
  if(inv_all_lru) lru <= {SETS{1'b0}};
  else if(touch) lru[touch_set] <= ~touch_way;
end
`endif

assign q0 = {q03, q02, q01, q00};
assign q1 = {q13, q12, q11, q10};
assign t0 = rt0;
assign t1 = rt1;
assign lru_way = lru[lru_set];

endmodule
