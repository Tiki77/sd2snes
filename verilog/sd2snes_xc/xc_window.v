`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// xc_window: the Xeno Crisis $3000 window with a DMA send side (sd2snes, CLK2 domain).
//
// The soft CPU sends bytes to the SNES by posting descriptors: {RP2040 RAM address, length}, or one
// immediate byte. A DMA reader copies them in order from the SRAM chip (where the RP2040 RAM lives)
// into a 512-byte prefetch ring (xc_stream), which the SNES reads through $3000. Bytes are visible
// to the SNES as soon as they are in the ring. The SRAM bus (~13 MB/s) is far faster than the SNES
// can read (<= 3.6 MB/s), so the ring stays ahead; MesenCE measured no underrun down to 4 MB/s.
//
// The SoC cleans the D-cache for the descriptor's range before it writes TX_LEN (xc_soc.v), so the
// DMA sees what the core wrote.
//
// Registers (word offsets in the 4 KB block at 0x50803000, see src/xc_soc/xc_soc.h):
//   0x00 TX_ADDR    W  RP2040 address of the next descriptor (must be in RAM 0x20000000-0x20041FFF)
//   0x04 TX_LEN     W  length; posts {TX_ADDR, length} (0 is ignored)
//   0x08 TX_PENDING R  bytes posted and not yet read by the SNES (ring + queued descriptors)
//   0x0C CTRL       W  bit 0 flush send side (queue, DMA and ring), bit 1 flush receive side
//   0x10 RX_DATA    R  bit 8 valid, bits 7:0 byte; reading pops it
//   0x14 RX_LEVEL   R  bytes waiting
//   0x18 TX_FREE    R  free descriptor slots (8 deep)
//   0x1C TX_BYTE    W  queue one immediate byte
//   0x20 STATUS     R  bit 0 descriptor overflow (sticky), bit 1 bad DMA address (sticky),
//                      bit 2 RX overflow (sticky); write 1s to clear
// Register accesses: hold sel until ready (ready is high one cycle after sel, for reads and writes).
//
// DMA memory port (GSU-style, see main.v): dma_rrq pulses with dma_addr (SRAM chip byte address);
// dma_rdy goes low on the next cycle and returns high with dma_rdata valid.
//////////////////////////////////////////////////////////////////////////////////
module xc_window #(
  parameter [18:0] RAM_BASE = 19'h08000,  // SRAM chip address of RP2040 RAM 0x20000000
  parameter STATS = 1                      // statistics counters (simulation); 0 on hardware
) (
  input clk,
  input rst,

  // SNES side (see xc_stream.v)
  input enable,
  input snes_rd_start,
  input snes_rd_end,
  input snes_wr_end,
  input [7:0] snes_data_in,
  output [7:0] snes_data_out,

  // soft-CPU register port
  input sel,
  input we,
  input [5:2] addr,
  input [31:0] wdata,
  output reg [31:0] rdata,
  output reg ready,

  // DMA reader on the SRAM chip
  output reg dma_rrq,
  output reg [18:0] dma_addr,
  input dma_rdy,
  input [7:0] dma_rdata,

  // statistics / debug
  output [31:0] stat_tx_read,
  output [31:0] stat_rx_written,
  output [31:0] stat_desc,
  output [31:0] stat_underrun       // SNES reads that found the ring empty while a descriptor was queued
);

`ifdef MK2
localparam DEPTH = 2;
`else
localparam DEPTH = 8;
`endif

//------------------------------------------------------------------------------
// Descriptor queue
//------------------------------------------------------------------------------
reg [18:0] q_addr [0:DEPTH-1];
reg [15:0] q_len  [0:DEPTH-1];     // the largest descriptor the firmware posts is 26,332 bytes
reg        q_imm  [0:DEPTH-1];
reg [7:0]  q_byte [0:DEPTH-1];
reg [2:0] q_head, q_tail;
reg [3:0] q_count;
reg [19:0] q_bytes;              // bytes in queued descriptors not yet in the ring (<= 8 x 64 KB)
reg [31:0] tx_addr_r;
reg st_ovf, st_badaddr, st_rxovf_clr;

wire q_full = (q_count == DEPTH);

//------------------------------------------------------------------------------
// Ring (xc_stream)
//------------------------------------------------------------------------------
reg host_tx_we;
reg [7:0] host_tx_data;
reg host_tx_flush;
wire [9:0] host_tx_free;
wire [9:0] host_tx_pending;
wire host_tx_overflow;
reg host_rx_re;
wire [7:0] host_rx_data;
wire [8:0] host_rx_level;
wire host_rx_overflow;
reg host_rx_flush;

xc_stream #(.TX_BITS(9), .RX_BITS(8), .STATS(STATS)) stream (
  .clk(clk),
  .rst(rst),
  .enable(enable),
  .snes_rd_start(snes_rd_start),
  .snes_rd_end(snes_rd_end),
  .snes_wr_end(snes_wr_end),
  .snes_data_in(snes_data_in),
  .snes_data_out(snes_data_out),
  .host_tx_we(host_tx_we),
  .host_tx_data(host_tx_data),
  .host_tx_commit(host_tx_we),     // bytes are visible as soon as they are in the ring
  .host_tx_flush(host_tx_flush),
  .host_tx_free(host_tx_free),
  .host_tx_pending(host_tx_pending),
  .host_tx_overflow(host_tx_overflow),
  .host_rx_re(host_rx_re),
  .host_rx_flush(host_rx_flush),
  .host_rx_data(host_rx_data),
  .host_rx_level(host_rx_level),
  .host_rx_overflow(host_rx_overflow),
  .host_clear_flags(st_rxovf_clr),
  .stat_tx_read(stat_tx_read),
  .stat_rx_written(stat_rx_written)
);

//------------------------------------------------------------------------------
// DMA
//------------------------------------------------------------------------------
localparam D_IDLE = 2'd0, D_REQ = 2'd1, D_WAIT = 2'd2;
reg [1:0] dstate;
reg dma_drop;                    // flush while a read is in flight: discard its byte
wire [18:0] h_addr = q_addr[q_head];
wire [15:0] h_len = q_len[q_head];
wire h_imm = q_imm[q_head];
// keep 2 bytes of headroom (the ring pointer update is one cycle behind a write), plus one because the
// flag is registered (at most one more byte can be written in the cycle it is stale). Registered so the
// DMA decision does not wait for two subtractions and a compare (clk2 timing).
reg ring_room = 1'b0;
always @(posedge clk) ring_room <= (host_tx_free > 10'd3);

reg [31:0] stat_desc_r, stat_underrun_r;
assign stat_desc = STATS ? stat_desc_r : 32'd0;
assign stat_underrun = STATS ? stat_underrun_r : 32'd0;
`ifndef MK2
always @(posedge clk) begin
  if(rst) begin
    stat_underrun_r <= 0;
    stat_desc_r <= 0;
  end else begin
    if(enable & snes_rd_start & (host_tx_pending == 0) & (q_count != 0)) stat_underrun_r <= stat_underrun_r + 1'b1;
    if(post) stat_desc_r <= stat_desc_r + 1'b1;
  end
end
`endif

//------------------------------------------------------------------------------
// Register port, queue and DMA
//------------------------------------------------------------------------------
// TX_PENDING = ring (committed, unread) + queued bytes + the byte on its way into the ring
wire [31:0] tx_pending_all = {22'd0, host_tx_pending} + {12'd0, q_bytes} + {31'd0, host_tx_we};
// RP2040 RAM address -> SRAM chip address
wire addr_ok = (tx_addr_r[31:19] == 13'h0400) && (tx_addr_r[18:0] < 19'h42000);
wire [18:0] tx_phys = RAM_BASE + tx_addr_r[18:0];

wire reg_cycle = sel & ~ready;
wire post_len  = reg_cycle & we & (addr == 4'h1) & (wdata != 32'd0);
wire post_imm  = reg_cycle & we & (addr == 4'h7);
wire post      = (post_len | post_imm) & ~q_full;
wire flush_now = reg_cycle & we & (addr == 4'h3) & wdata[0];
`ifdef MK2

// statistics (after 'post': XST needs declarations before use)
always @(posedge clk) begin
  if(rst) begin
    stat_underrun_r <= 0;
    stat_desc_r <= 0;
  end else begin
    if(enable & snes_rd_start & (host_tx_pending == 0) & (q_count != 0)) stat_underrun_r <= stat_underrun_r + 1'b1;
    if(post) stat_desc_r <= stat_desc_r + 1'b1;
  end
end
`endif

// a byte leaves the head descriptor this cycle
reg take;
reg take_last;                   // ...and it was the descriptor's last byte
always @* begin
  take = 1'b0;
  take_last = 1'b0;
  if(!flush_now) begin
    if(dstate == D_IDLE && q_count != 0 && ring_room && h_imm) begin
      take = 1'b1;
      take_last = 1'b1;
    end else if(dstate == D_WAIT && dma_rdy && !dma_drop) begin
      take = 1'b1;
      take_last = (h_len == 16'd1);
    end
  end
end

always @(posedge clk) begin
  host_tx_we <= 1'b0;
  host_tx_flush <= 1'b0;
  host_rx_re <= 1'b0;
  host_rx_flush <= 1'b0;
  st_rxovf_clr <= 1'b0;
  dma_rrq <= 1'b0;
  ready <= 1'b0;
  if(rst) begin
    q_head <= 0;
    q_tail <= 0;
    q_count <= 0;
    q_bytes <= 0;
    tx_addr_r <= 0;
    st_ovf <= 1'b0;
    st_badaddr <= 1'b0;
    dstate <= D_IDLE;
    dma_drop <= 1'b0;
    rdata <= 0;
  end else begin
    //---------------- register port ----------------
    if(reg_cycle) begin
      ready <= 1'b1;
      if(we) begin
        case(addr)
          4'h0: tx_addr_r <= wdata;
          4'h1, 4'h7: if(post_len | post_imm) begin
            if(q_full) st_ovf <= 1'b1;
            else if(post_len && (!addr_ok || wdata[31:16] != 16'd0)) st_badaddr <= 1'b1;   // bad address or > 64 KB
          end
          4'h3: begin
            if(wdata[0]) host_tx_flush <= 1'b1;
            if(wdata[1]) host_rx_flush <= 1'b1;
          end
          4'h8: begin
            if(wdata[0]) st_ovf <= 1'b0;
            if(wdata[1]) st_badaddr <= 1'b0;
            if(wdata[2]) st_rxovf_clr <= 1'b1;
          end
          default: ;
        endcase
      end else begin
        case(addr)
          4'h2: rdata <= tx_pending_all;
          4'h4: begin
            rdata <= {23'd0, host_rx_level != 9'd0, host_rx_data};
            host_rx_re <= (host_rx_level != 9'd0);
          end
          4'h5: rdata <= {23'd0, host_rx_level};
          4'h6: rdata <= DEPTH - q_count;
          4'h8: rdata <= {29'd0, host_rx_overflow, st_badaddr, st_ovf};
          default: rdata <= 32'd0;
        endcase
      end
    end

    //---------------- queue ----------------
    if(flush_now) begin
      q_head <= 0;
      q_tail <= 0;
      q_count <= 0;
      q_bytes <= 0;
      if(dstate != D_IDLE) dma_drop <= 1'b1;   // discard the read in flight
    end else begin
      if(post) begin
        q_addr[q_tail] <= tx_phys;
        q_len[q_tail] <= post_imm ? 16'd1 : wdata[15:0];
        q_imm[q_tail] <= post_imm;
        q_byte[q_tail] <= wdata[7:0];
        q_tail <= q_tail + 1'b1;
      end
      if(take_last) q_head <= q_head + 1'b1;
      q_count <= q_count + (post ? 4'd1 : 4'd0) - (take_last ? 4'd1 : 4'd0);
      q_bytes <= q_bytes + (post_len & ~q_full ? {4'd0, wdata[15:0]} : 20'd0)
                         + (post_imm & ~q_full ? 20'd1 : 20'd0)
                         - (take ? 20'd1 : 20'd0);
    end

    //---------------- DMA ----------------
    if(take) begin
      host_tx_we <= 1'b1;
      host_tx_data <= h_imm ? q_byte[q_head] : dma_rdata;
      if(!take_last) begin
        q_addr[q_head] <= h_addr + 1'b1;
        q_len[q_head] <= h_len - 1'b1;
      end
    end
    case(dstate)
      D_IDLE: begin
        if(!flush_now && q_count != 0 && ring_room && !h_imm) begin
          dma_rrq <= 1'b1;
          dma_addr <= h_addr;
          dstate <= D_REQ;
        end
      end
      D_REQ: dstate <= D_WAIT;    // dma_rdy drops on this cycle
      D_WAIT: begin
        if(dma_rdy) begin
          dstate <= D_IDLE;
          if(!flush_now) dma_drop <= 1'b0;
        end
      end
      default: dstate <= D_IDLE;
    endcase
  end
end

endmodule
