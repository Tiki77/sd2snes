`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// xc_stream: the Xeno Crisis cartridge's $3000 window (RP2040 PIO link) for sd2snes.
//
// On the real cartridge the RP2040 answers every access in $00-$3F/$80-$BF:$3000-$3FFF:
//   - a read returns the next byte of the RP2040 -> SNES stream and consumes it, or $00 when
//     nothing is queued (the kernel polls on that, and the stream also carries 65816 code
//     that the SNES executes directly from the window after JSL $00:3000);
//   - a write sends one byte to the RP2040.
// The address inside the window is ignored; every bus cycle is one byte, so a 16-bit read
// (LDY $3000) consumes two bytes and a 16-bit write (STX $3000) sends two.
//
// Producer side (MCU or soft core, called "host" below):
//   TX (host -> SNES): host_tx_we appends host_tx_data to a ring buffer. Bytes become visible
//     to the SNES only on host_tx_commit, so a batch the host writes slowly can never be seen
//     half-finished (an underrun in the middle of streamed code would execute $00 = BRK).
//     host_tx_flush drops everything queued, committed or not (RP2040 BusFlush); host_rx_flush
//     drops the received bytes.
//   RX (SNES -> host): host_rx_re pops host_rx_data. host_rx_data shows the oldest byte and is
//     valid whenever host_rx_level != 0 (two clocks after a push or pop).
//
// SNES side: plug into main.v like the OBC1 / MSU cores:
//   enable      = window decode (see xc_window below)
//   snes_rd_start / snes_rd_end / snes_wr_end = SNES_RD_start / SNES_RD_end / SNES_WR_end
//   snes_data_in = BUS_DATA, snes_data_out -> SNES data bus mux while enable is set.
// A read decides at snes_rd_start whether a byte is available and holds that value on
// snes_data_out until the read ends; the byte is consumed at snes_rd_end. A byte committed
// while a read is already in progress is therefore delivered by the next read and is never
// lost. A flush during a read cancels that read's consume.
//////////////////////////////////////////////////////////////////////////////////
module xc_stream #(
  parameter TX_BITS = 15,   // TX ring size = 2^TX_BITS bytes (largest recorded batch: 26,332 bytes)
  parameter RX_BITS = 8,    // RX ring size = 2^RX_BITS bytes
  parameter STATS = 1       // statistics counters (0: tied to zero)
) (
  input clk,
  input rst,

  // SNES side
  input enable,
  input snes_rd_start,
  input snes_rd_end,
  input snes_wr_end,
  input [7:0] snes_data_in,
  output [7:0] snes_data_out,

  // host side, TX
  input host_tx_we,
  input [7:0] host_tx_data,
  input host_tx_commit,
  input host_tx_flush,
  output [TX_BITS:0] host_tx_free,      // bytes that can still be written
  output [TX_BITS:0] host_tx_pending,   // committed bytes the SNES has not read yet
  output reg host_tx_overflow,          // sticky: a write was dropped because the ring was full

  // host side, RX
  input host_rx_re,
  input host_rx_flush,                  // drop every byte the SNES has written so far
  output [7:0] host_rx_data,
  output [RX_BITS:0] host_rx_level,
  output reg host_rx_overflow,          // sticky: a SNES write was dropped because the ring was full
  input host_clear_flags,

  // statistics
  output [31:0] stat_tx_read,           // bytes the SNES consumed
  output [31:0] stat_rx_written         // bytes the SNES wrote
);
reg [31:0] stat_tx_read_r, stat_rx_written_r;
assign stat_tx_read = STATS ? stat_tx_read_r : 32'd0;
assign stat_rx_written = STATS ? stat_rx_written_r : 32'd0;

localparam TX_SIZE = (1 << TX_BITS);
localparam RX_SIZE = (1 << RX_BITS);

//------------------------------------------------------------------------------
// TX ring
//------------------------------------------------------------------------------
reg [7:0] tx_mem [0:TX_SIZE-1];
reg [TX_BITS:0] tx_wp = 0;       // host write pointer (uncommitted data up to here)
reg [TX_BITS:0] tx_cp = 0;       // commit pointer
reg [TX_BITS:0] tx_cp_d1 = 0;
reg [TX_BITS:0] tx_vis = 0;      // commit pointer as seen by the SNES side (delayed so tx_q is settled)
reg [TX_BITS:0] tx_rp = 0;       // SNES read pointer
reg [7:0] tx_q = 0;              // tx_mem[tx_rp], registered (block RAM read port)
reg [7:0] rd_value = 0;          // value driven during the current read
reg rd_consume = 0;              // current read takes a byte at snes_rd_end

wire [TX_BITS:0] tx_used = tx_wp - tx_rp;
wire tx_full = tx_used[TX_BITS];
wire tx_avail = (tx_vis != tx_rp);

assign host_tx_free = TX_SIZE[TX_BITS:0] - tx_used;
assign host_tx_pending = tx_cp - tx_rp;
assign snes_data_out = rd_value;

wire tx_pop = enable & snes_rd_end & rd_consume & ~host_tx_flush;

always @(posedge clk) begin
  if(host_tx_we & ~tx_full) tx_mem[tx_wp[TX_BITS-1:0]] <= host_tx_data;
  // registered read; the address is the pointer after this cycle's pop
  tx_q <= tx_mem[tx_pop ? tx_rp[TX_BITS-1:0] + 1'b1 : tx_rp[TX_BITS-1:0]];
end

always @(posedge clk) begin
  if(rst) begin
    tx_wp <= 0;
    tx_cp <= 0;
    tx_cp_d1 <= 0;
    tx_vis <= 0;
    tx_rp <= 0;
    rd_value <= 8'h00;
    rd_consume <= 1'b0;
    host_tx_overflow <= 1'b0;
    stat_tx_read_r <= 0;
  end else begin
    // host side
    if(host_tx_flush) begin
      // drop everything; bytes written in this very cycle are dropped too
      tx_rp <= tx_wp + (host_tx_we & ~tx_full);
      tx_wp <= tx_wp + (host_tx_we & ~tx_full);
      tx_cp <= tx_wp + (host_tx_we & ~tx_full);
      tx_cp_d1 <= tx_wp + (host_tx_we & ~tx_full);
      tx_vis <= tx_wp + (host_tx_we & ~tx_full);
      rd_consume <= 1'b0;
      if(enable & snes_rd_start) rd_value <= 8'h00;
    end else begin
      if(host_tx_we) begin
        if(tx_full) host_tx_overflow <= 1'b1;
        else tx_wp <= tx_wp + 1'b1;
      end
      if(host_tx_commit) tx_cp <= tx_wp + (host_tx_we & ~tx_full);
      // the RAM read port needs two clocks to show a byte written at tx_rp
      tx_cp_d1 <= tx_cp;
      tx_vis <= tx_cp_d1;

      // SNES side
      if(enable & snes_rd_start) begin
        rd_consume <= tx_avail;
        rd_value <= tx_avail ? tx_q : 8'h00;
      end
      if(tx_pop) begin
        tx_rp <= tx_rp + 1'b1;
        rd_consume <= 1'b0;
        stat_tx_read_r <= stat_tx_read_r + 1'b1;
      end
    end
    if(host_clear_flags) host_tx_overflow <= 1'b0;
  end
end

//------------------------------------------------------------------------------
// RX ring
//------------------------------------------------------------------------------
reg [7:0] rx_mem [0:RX_SIZE-1];
reg [RX_BITS:0] rx_wp = 0;
reg [RX_BITS:0] rx_wp_d1 = 0;
reg [RX_BITS:0] rx_vis = 0;
reg [RX_BITS:0] rx_rp = 0;
reg [7:0] rx_q = 0;

wire [RX_BITS:0] rx_used = rx_wp - rx_rp;
wire rx_full = rx_used[RX_BITS];
wire rx_push = enable & snes_wr_end;
wire rx_pop = host_rx_re & (rx_vis != rx_rp);
wire [RX_BITS:0] rx_wp_next = rx_wp + (rx_push & ~rx_full);

assign host_rx_level = rx_vis - rx_rp;
assign host_rx_data = rx_q;

always @(posedge clk) begin
  if(rx_push & ~rx_full) rx_mem[rx_wp[RX_BITS-1:0]] <= snes_data_in;
  rx_q <= rx_mem[rx_pop ? rx_rp[RX_BITS-1:0] + 1'b1 : rx_rp[RX_BITS-1:0]];
end

always @(posedge clk) begin
  if(rst) begin
    rx_wp <= 0;
    rx_wp_d1 <= 0;
    rx_vis <= 0;
    rx_rp <= 0;
    host_rx_overflow <= 1'b0;
    stat_rx_written_r <= 0;
  end else begin
    if(rx_push) begin
      stat_rx_written_r <= stat_rx_written_r + 1'b1;
      if(rx_full) host_rx_overflow <= 1'b1;
      else rx_wp <= rx_wp + 1'b1;
    end
    if(host_rx_flush) begin
      // drop everything, including a byte arriving in this cycle
      rx_rp <= rx_wp_next;
      rx_wp_d1 <= rx_wp_next;
      rx_vis <= rx_wp_next;
    end else begin
      rx_wp_d1 <= rx_wp;
      rx_vis <= rx_wp_d1;
      if(rx_pop) rx_rp <= rx_rp + 1'b1;
    end
    if(host_clear_flags) host_rx_overflow <= 1'b0;
  end
end

endmodule

//------------------------------------------------------------------------------
// Window decode for address.v: banks $00-$3F and $80-$BF, $3000-$3FFF.
//------------------------------------------------------------------------------
module xc_window_decode(
  input [23:0] SNES_ADDR,
  output hit
);
assign hit = (~SNES_ADDR[22]) & (SNES_ADDR[15:12] == 4'h3);
endmodule
