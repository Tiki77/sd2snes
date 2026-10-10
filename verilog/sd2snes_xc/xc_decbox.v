`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// xc_decbox: Opus decode mailbox between the soft CPU (Xeno Crisis mixer) and the sd2snes MCU.
//
// Soft-core side (word registers, see mixer/xc_soc.h; byte offsets in a 4 KB window):
//   0x000 CTRL   write: bit0 submit job, bit1 ack result, bit2 request decoder reset
//                read:  bit0 job pending, bit1 result ready, bit2 reset pending
//   0x004 LEN    packet length in bytes (<= 1536)
//   0x008 RET    opus_decode() result (from the MCU)
//   0x00C RANGE  decoder final range (from the MCU)
//   0x100-0x6FF  PACKET (write), 384 words
//   0x800-0xF7F  PCM (read), 480 words: one stereo sample (L low, R high) per word
//   Reads take one extra cycle (block RAM): `ready` answers the soft core's bus.
//
// MCU side (driven by mcu_cmd.v from the XCA_* SPI commands, see mcu/xc_audio.h):
//   mcu_status = {reset pending, job pending}, mcu_len
//   mcu_pkt_start / mcu_pkt_rd: read packet bytes from 0 (mcu_pkt_data valid the cycle after mcu_pkt_rd)
//   mcu_pcm_start / mcu_pcm_wr + mcu_pcm_data: write PCM bytes from 0
//   mcu_done + mcu_ret + mcu_range: result ready, job no longer pending, irq pulse to the soft CPU
//   mcu_ack_reset: reset request handled
//////////////////////////////////////////////////////////////////////////////////
module xc_decbox (
  input clk,
  input rst,

  // soft core
  input sel,
  input we,
  input [11:2] addr,
  input [31:0] wdata,
  output reg [31:0] rdata,
  output ready,
  output reg irq,              // one-cycle pulse when a result arrives

  // MCU
  output [1:0] mcu_status,
  output [10:0] mcu_len,
  input mcu_pkt_start,
  input mcu_pkt_rd,
  output reg [7:0] mcu_pkt_data,
  input mcu_pcm_start,
  input mcu_pcm_wr,
  input [7:0] mcu_pcm_data,
  input mcu_done,
  input [31:0] mcu_ret,
  input [31:0] mcu_range,
  input mcu_ack_reset
);

reg pending, result, reset_req;
reg [10:0] len;
reg [31:0] ret, range;

// packet RAM: 4 byte lanes x 384
reg [7:0] pkt0 [0:383], pkt1 [0:383], pkt2 [0:383], pkt3 [0:383];
// PCM RAM: 4 byte lanes x 480
reg [7:0] pcm0 [0:479], pcm1 [0:479], pcm2 [0:479], pcm3 [0:479];

reg [10:0] pkt_ptr;
reg [10:0] pcm_ptr;

assign mcu_status = {reset_req, pending};
assign mcu_len = len;

// soft-core side
wire is_pkt = (addr[11:8] >= 4'h1) && (addr[11:8] < 4'h7);
wire [8:0] pkt_word = addr[10:2] - 9'd64;           // (addr - 0x100) / 4
wire is_pcm = addr[11];
wire [8:0] pcm_word = addr[10:2];                   // (addr - 0x800) / 4
reg rd_pending;
reg [7:0] pcm_q0, pcm_q1, pcm_q2, pcm_q3;
reg [1:0] rsel;                                     // 0: registers, 1: PCM
reg [31:0] reg_q;
assign ready = sel & (we | rd_pending);

always @(posedge clk) begin
  // soft-core writes to the packet RAM
  if(sel && we && is_pkt) begin
    pkt0[pkt_word] <= wdata[7:0];
    pkt1[pkt_word] <= wdata[15:8];
    pkt2[pkt_word] <= wdata[23:16];
    pkt3[pkt_word] <= wdata[31:24];
  end
  // PCM RAM: MCU writes one byte lane, soft core reads all four
  if(mcu_pcm_wr) begin
    case(pcm_ptr[1:0])
      2'd0: pcm0[pcm_ptr[10:2]] <= mcu_pcm_data;
      2'd1: pcm1[pcm_ptr[10:2]] <= mcu_pcm_data;
      2'd2: pcm2[pcm_ptr[10:2]] <= mcu_pcm_data;
      default: pcm3[pcm_ptr[10:2]] <= mcu_pcm_data;
    endcase
  end
  pcm_q0 <= pcm0[pcm_word];
  pcm_q1 <= pcm1[pcm_word];
  pcm_q2 <= pcm2[pcm_word];
  pcm_q3 <= pcm3[pcm_word];
  // MCU reads packet bytes
  case(pkt_ptr[1:0])
    2'd0: mcu_pkt_data <= pkt0[pkt_ptr[10:2]];
    2'd1: mcu_pkt_data <= pkt1[pkt_ptr[10:2]];
    2'd2: mcu_pkt_data <= pkt2[pkt_ptr[10:2]];
    default: mcu_pkt_data <= pkt3[pkt_ptr[10:2]];
  endcase
end

always @(posedge clk) begin
  if(rst) begin
    pending <= 1'b0;
    result <= 1'b0;
    reset_req <= 1'b0;
    len <= 11'd0;
    ret <= 32'd0;
    range <= 32'd0;
    pkt_ptr <= 11'd0;
    pcm_ptr <= 11'd0;
    irq <= 1'b0;
    rd_pending <= 1'b0;
  end else begin
    irq <= 1'b0;
    // soft core: reads complete one cycle after the request
    rd_pending <= sel & ~we & ~rd_pending;
    if(sel && !we && !rd_pending) begin
      rsel <= is_pcm ? 2'd1 : 2'd0;
      case(addr[11:2])
        10'h000: reg_q <= {29'd0, reset_req, result, pending};
        10'h001: reg_q <= {21'd0, len};
        10'h002: reg_q <= ret;
        10'h003: reg_q <= range;
        default: reg_q <= 32'd0;
      endcase
    end
    if(sel && we) begin
      if(addr[11:2] == 10'h000) begin
        if(wdata[2]) begin reset_req <= 1'b1; pending <= 1'b0; result <= 1'b0; end
        if(wdata[1]) result <= 1'b0;
        if(wdata[0] && !pending) begin pending <= 1'b1; result <= 1'b0; end
      end else if(addr[11:2] == 10'h001) begin
        len <= (wdata[10:0] > 11'd1536) ? 11'd1536 : wdata[10:0];
      end
    end
    // MCU
    if(mcu_pkt_start) pkt_ptr <= 11'd0;
    else if(mcu_pkt_rd) pkt_ptr <= pkt_ptr + 11'd1;
    if(mcu_pcm_start) pcm_ptr <= 11'd0;
    else if(mcu_pcm_wr) pcm_ptr <= pcm_ptr + 11'd1;
    if(mcu_ack_reset) reset_req <= 1'b0;
    if(mcu_done && pending) begin
      ret <= mcu_ret;
      range <= mcu_range;
      pending <= 1'b0;
      result <= 1'b1;
      irq <= 1'b1;
    end
  end
end

always @* begin
  rdata = (rsel == 2'd1) ? {pcm_q3, pcm_q2, pcm_q1, pcm_q0} : reg_q;
end

endmodule
