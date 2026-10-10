`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// xc_top: everything the Xeno Crisis cartridge needs besides the sd2snes base, as one block for main.v.
//
//   clk_soc domain: xc_soc (xc_m0 core, caches, SoC-local peripherals)
//   clk2 domain:    xc_bridge (executor), xc_window ($3000 window + DMA), xc_decbox (MCU decode mailbox),
//                   SRAM arbiter (DMA first, per byte)
//
// main.v connects (GSU-style request ports, see sd2snes_gsu/main.v):
//   ROM bus (PSRAM): rom_rrq / rom_addr / rom_rdy / rom_rdata (16-bit word reads)
//   RAM bus (SRAM chip): ram_rrq / ram_wrq / ram_addr / ram_len / ram_wdata / ram_bstb / ram_rdy / ram_rdata
//     (bursts of ram_len bytes, see the RAM pipeline in main.v)
//   SNES: the window strobes and data (enable = $00-$3F/$80-$BF:$3000-$3FFF)
//   MCU: the decode mailbox ports (mcu_cmd.v, XCA_* commands) and a run/reset control
//////////////////////////////////////////////////////////////////////////////////
module xc_top #(
  parameter SOC_CLK_NUM = 40,        // soft CPU clock = SOC_CLK_NUM / SOC_CLK_DEN MHz (sd2snes PLL: 161/4 = 40.25 MHz)
  parameter SOC_CLK_DEN = 1,
  parameter STATS = 1,             // window statistics (simulation); 0 on hardware
`ifdef MK2
  parameter DIDX = 8               // the cache (code and data): 2^DIDX sets x 2 ways x 32 B (8 = 16 KB)
`else
  parameter DIDX = 8,              // D-cache: 2^DIDX sets x 2 ways x 32 B (8 = 16 KB)
  parameter IIDX = 8,              // I-cache: 2^IIDX sets x 2 ways x 32 B (8 = 16 KB)
  parameter MSU = 0                // 1: MSU-1 only (xc_msubox instead of the Opus decode mailbox);
                                   // 2: combined core (Mk.III): both, msu_mode picks one at load time
`endif
) (
  input clk2,
  input clk_soc,
  input rst2,                      // CLK2 domain reset (held while the MCU loads, see main.v)
  input soc_run,                   // CLK2 domain: 0 holds the SoC in reset

  // SNES window
  input win_enable,
  input snes_rd_start,
  input snes_rd_end,
  input snes_wr_end,
  input [7:0] snes_data_in,
  output [7:0] snes_data_out,

  // ROM bus (PSRAM)
  output rom_rrq,
  output [23:0] rom_addr,
  input rom_rdy,
  input [15:0] rom_rdata,

  // RAM bus (SRAM chip)
  output reg ram_rrq,
  output reg ram_wrq,
  output reg [18:0] ram_addr,
  output reg [5:0] ram_len,
  output [7:0] ram_wdata,
  input ram_bstb,
  input ram_rdy,
  input [7:0] ram_rdata,

  // MCU decode service (xc_decbox MCU side)
  output [1:0] mcu_status,
  output [10:0] mcu_len,
  input mcu_pkt_start,
  input mcu_pkt_rd,
  output [7:0] mcu_pkt_data,
  input mcu_pcm_start,
  input mcu_pcm_wr,
  input [7:0] mcu_pcm_data,
  input mcu_done,
  input [31:0] mcu_ret,
  input [31:0] mcu_range,
  input mcu_ack_reset,

  // MSU-1 core (MSU = 1): msu.v register writes from the soft CPU, clk2 domain
  output msu_we,
  output [2:0] msu_addr,
  output [7:0] msu_data,
  input [7:0] msu_status,
`ifndef MK2
  input msu_mode,                  // MSU = 2: 1 = music from the MSU-1 (CTRL bit 3 reads 1), 0 = Opus (MCU, $C6 bit 1)
`endif

  // status (clk_soc domain; quasi-static)
  output soc_running,
  output soc_halted,
  output [31:0] soc_halt_code,
  output [31:0] soc_halt_addr,
  output soc_dbg_char_valid,
  output [7:0] soc_dbg_char,
  output [31:0] stat_tx_read,
  output [31:0] stat_rx_written,
  output [31:0] stat_desc,
  output [31:0] stat_underrun,

  // performance counters for the MCU ($C7 snapshot, $C8 read; see "perf" below)
  input perf_snap,                 // clk2 pulse: take a snapshot
  output [255:0] perf_data,        // the snapshot (static after perf_snap)

  // simulation observation
  output mon_req, output mon_ready, output mon_we, output mon_fetch, output [1:0] mon_size,
  output [31:0] mon_addr, output [31:0] mon_wdata, output [31:0] mon_rdata,
  output mon_dma_rd, output [18:0] mon_dma_addr, output [7:0] mon_dma_data
);

//------------------------------------------------------------------------------
// Resets
//------------------------------------------------------------------------------
wire rst2_all = rst2 | ~soc_run;
(* altera_attribute = "-name SYNCHRONIZER_IDENTIFICATION FORCED_IF_ASYNCHRONOUS" *) reg [2:0] rst_soc_sync = 3'b111;
always @(posedge clk_soc) rst_soc_sync <= {rst_soc_sync[1:0], rst2_all};
wire rst_soc = rst_soc_sync[2];

//------------------------------------------------------------------------------
// SoC and bridge
//------------------------------------------------------------------------------
wire op_start, op_done;
wire [2:0] op_kind;
wire [23:0] op_addr;
wire [5:0] op_len;
wire wb_we;
wire [2:0] wb_addr, rb_addr;
wire [31:0] wb_data, rb_q;
reg dec_irq_tog = 1'b0;

`ifdef MK2
xc_soc #(.CLK_NUM(SOC_CLK_NUM), .CLK_DEN(SOC_CLK_DEN), .DIDX(DIDX), .IIDX(5)) soc (
`else
xc_soc #(.CLK_NUM(SOC_CLK_NUM), .CLK_DEN(SOC_CLK_DEN), .DIDX(DIDX), .IIDX(IIDX)) soc (
`endif
  .clk(clk_soc), .rst(rst_soc),
  .op_start(op_start), .op_kind(op_kind), .op_addr(op_addr), .op_len(op_len),
  .wb_we(wb_we), .wb_addr(wb_addr), .wb_data(wb_data), .op_done(op_done), .rb_addr(rb_addr), .rb_q(rb_q),
  .dec_irq_tog(dec_irq_tog),
  .running(soc_running), .halted(soc_halted), .halt_code(soc_halt_code), .halt_addr(soc_halt_addr),
  .dbg_char_valid(soc_dbg_char_valid), .dbg_char(soc_dbg_char),
  .step_mode(1'b0), .step_go(1'b0), .step_done(), .dbg_we(1'b0), .dbg_sel(5'd0), .dbg_wdata(32'd0), .dbg_rsel(5'd0), .dbg_rdata(),
  .mon_req(mon_req), .mon_ready(mon_ready), .mon_we(mon_we), .mon_fetch(mon_fetch), .mon_size(mon_size),
  .mon_addr(mon_addr), .mon_wdata(mon_wdata), .mon_rdata(mon_rdata), .core_fault_code_o()
);

wire br_rrq, br_wrq, br_rdy;
wire [18:0] br_addr;
wire [7:0] br_wdata;
wire [7:0] br_rdata;
wire [5:0] br_len;
wire br_bstb;
wire win_sel, win_we, dec_sel, dec_we;
wire [11:2] reg_addr;
wire [31:0] reg_wdata, win_rdata, dec_rdata;
wire win_ready, dec_ready;

xc_bridge bridge (
  .clk_soc(clk_soc), .rst_soc(rst_soc),
  .op_start(op_start), .op_kind(op_kind), .op_addr(op_addr), .op_len(op_len),
  .wb_we(wb_we), .wb_addr(wb_addr), .wb_data(wb_data), .op_done(op_done), .rb_addr(rb_addr), .rb_q(rb_q),
  .clk2(clk2), .rst2(rst2_all),
  .rom_rrq(rom_rrq), .rom_addr(rom_addr), .rom_rdy(rom_rdy), .rom_rdata(rom_rdata),
  .sram_rrq(br_rrq), .sram_wrq(br_wrq), .sram_addr(br_addr), .sram_wdata(br_wdata), .sram_rdy(br_rdy), .sram_rdata(br_rdata),
  .sram_len(br_len), .sram_bstb(br_bstb),
  .win_sel(win_sel), .win_we(win_we), .reg_addr(reg_addr), .reg_wdata(reg_wdata), .win_rdata(win_rdata), .win_ready(win_ready),
  .dec_sel(dec_sel), .dec_we(dec_we), .dec_rdata(dec_rdata), .dec_ready(dec_ready)
);

//------------------------------------------------------------------------------
// $3000 window
//------------------------------------------------------------------------------
wire dma_rrq, dma_rdy;
wire [18:0] dma_addr;
reg [7:0] dma_rdata;

xc_window #(.STATS(STATS)) window (
  .clk(clk2), .rst(rst2_all),
  .enable(win_enable), .snes_rd_start(snes_rd_start), .snes_rd_end(snes_rd_end), .snes_wr_end(snes_wr_end),
  .snes_data_in(snes_data_in), .snes_data_out(snes_data_out),
  .sel(win_sel), .we(win_we), .addr(reg_addr[5:2]), .wdata(reg_wdata), .rdata(win_rdata), .ready(win_ready),
  .dma_rrq(dma_rrq), .dma_addr(dma_addr), .dma_rdy(dma_rdy), .dma_rdata(dma_rdata),
  .stat_tx_read(stat_tx_read), .stat_rx_written(stat_rx_written), .stat_desc(stat_desc), .stat_underrun(stat_underrun)
);

//------------------------------------------------------------------------------
// Decode mailbox
//------------------------------------------------------------------------------
wire dec_irq;
`ifdef MK2
// mk2: MSU-1 only (no Opus decode mailbox)
`else
generate if(MSU == 2) begin : g_combo
// combined core: the decode mailbox and the MSU-1 registers at the same place. The mailbox does not use 0x010 /
// 0x014, so writes go to both; reads of MSUSTAT (0x014) come from xc_msubox, CTRL bit 3 is msu_mode (the mixer
// reads it once at start, src/xc_soc/xc_mix.c), everything else from the mailbox. Both answer reads one cycle late.
wire [31:0] dec_rdata_d, msu_rdata;
wire msu_ready_unused;
reg rd_msu = 1'b0, rd_ctrl = 1'b0;
always @(posedge clk2) if(dec_sel & ~dec_we) begin
  rd_msu <= reg_addr[11:2] == 10'h005;
  rd_ctrl <= reg_addr[11:2] == 10'h000;
end
assign dec_rdata = rd_msu ? msu_rdata : (dec_rdata_d | {28'd0, rd_ctrl & msu_mode, 3'd0});
xc_decbox decbox (
  .clk(clk2), .rst(rst2_all),
  .sel(dec_sel), .we(dec_we), .addr(reg_addr[11:2]), .wdata(reg_wdata), .rdata(dec_rdata_d), .ready(dec_ready), .irq(dec_irq),
  .mcu_status(mcu_status), .mcu_len(mcu_len),
  .mcu_pkt_start(mcu_pkt_start), .mcu_pkt_rd(mcu_pkt_rd), .mcu_pkt_data(mcu_pkt_data),
  .mcu_pcm_start(mcu_pcm_start), .mcu_pcm_wr(mcu_pcm_wr), .mcu_pcm_data(mcu_pcm_data),
  .mcu_done(mcu_done), .mcu_ret(mcu_ret), .mcu_range(mcu_range), .mcu_ack_reset(mcu_ack_reset)
);
xc_msubox msubox (
  .clk(clk2), .rst(rst2_all),
  .sel(dec_sel), .we(dec_we), .addr(reg_addr[11:2]), .wdata(reg_wdata), .rdata(msu_rdata), .ready(msu_ready_unused),
  .msu_we(msu_we), .msu_addr(msu_addr), .msu_data(msu_data), .msu_status(msu_status)
);
end else if(MSU) begin : g_msu
`endif
xc_msubox msubox (
  .clk(clk2), .rst(rst2_all),
  .sel(dec_sel), .we(dec_we), .addr(reg_addr[11:2]), .wdata(reg_wdata), .rdata(dec_rdata), .ready(dec_ready),
  .msu_we(msu_we), .msu_addr(msu_addr), .msu_data(msu_data), .msu_status(msu_status)
);
assign dec_irq = 1'b0;
assign mcu_status = 2'b00;
assign mcu_len = 11'd0;
assign mcu_pkt_data = 8'h00;
`ifndef MK2
end else begin : g_dec
xc_decbox decbox (
  .clk(clk2), .rst(rst2_all),
  .sel(dec_sel), .we(dec_we), .addr(reg_addr[11:2]), .wdata(reg_wdata), .rdata(dec_rdata), .ready(dec_ready), .irq(dec_irq),
  .mcu_status(mcu_status), .mcu_len(mcu_len),
  .mcu_pkt_start(mcu_pkt_start), .mcu_pkt_rd(mcu_pkt_rd), .mcu_pkt_data(mcu_pkt_data),
  .mcu_pcm_start(mcu_pcm_start), .mcu_pcm_wr(mcu_pcm_wr), .mcu_pcm_data(mcu_pcm_data),
  .mcu_done(mcu_done), .mcu_ret(mcu_ret), .mcu_range(mcu_range), .mcu_ack_reset(mcu_ack_reset)
);
assign msu_we = 1'b0;
assign msu_addr = 3'd0;
assign msu_data = 8'd0;
end endgenerate
`endif
always @(posedge clk2) if(dec_irq) dec_irq_tog <= ~dec_irq_tog;

//------------------------------------------------------------------------------
// SRAM arbiter: the window DMA first (one byte per grant), then the bridge (a burst of up to 32 bytes per
// grant: line fills and write-backs). During a bridge burst the byte strobe, the read data and the write
// data pass straight between main.v and the bridge.
//------------------------------------------------------------------------------
reg pend_a, pend_b, pend_b_we;
reg [5:0] len_b;
reg [18:0] addr_a, addr_b;
reg rdy_a = 1'b1, rdy_b = 1'b1;
reg [1:0] ast;                     // 0 idle, 1 issued (rdy drops), 2 waiting
reg grant_b;
assign dma_rdy = rdy_a;
assign br_rdy = rdy_b;
assign br_bstb = ram_bstb & grant_b & (ast == 2'd2);
assign br_rdata = ram_rdata;
assign ram_wdata = br_wdata;        // only the bridge writes; it holds each byte until the strobe

assign mon_dma_rd = (ast == 2'd2) && ram_rdy && !grant_b;
assign mon_dma_addr = ram_addr;
assign mon_dma_data = ram_rdata;

always @(posedge clk2) begin
  ram_rrq <= 1'b0;
  ram_wrq <= 1'b0;
  if(rst2) begin
    pend_a <= 1'b0; pend_b <= 1'b0; rdy_a <= 1'b1; rdy_b <= 1'b1; ast <= 2'd0;
  end else begin
    if(dma_rrq) begin pend_a <= 1'b1; addr_a <= dma_addr; rdy_a <= 1'b0; end
    if(br_rrq | br_wrq) begin pend_b <= 1'b1; pend_b_we <= br_wrq; addr_b <= br_addr; len_b <= br_len; rdy_b <= 1'b0; end
    case(ast)
      2'd0: begin
        if(pend_a) begin
          ram_rrq <= 1'b1; ram_addr <= addr_a; ram_len <= 6'd1; grant_b <= 1'b0; ast <= 2'd1;
        end else if(pend_b) begin
          ram_rrq <= ~pend_b_we; ram_wrq <= pend_b_we; ram_addr <= addr_b; ram_len <= len_b; grant_b <= 1'b1; ast <= 2'd1;
        end
      end
      2'd1: ast <= 2'd2;
      2'd2: if(ram_rdy) begin
        if(grant_b) begin pend_b <= 1'b0; rdy_b <= 1'b1; end
        else begin pend_a <= 1'b0; rdy_a <= 1'b1; dma_rdata <= ram_rdata; end
        ast <= 2'd0;
      end
      default: ast <= 2'd0;
    endcase
  end
end

//------------------------------------------------------------------------------
// perf: hardware counters, read by the MCU into its log (xcaudio.txt)
//   word 0  SoC cycles
//   word 1  SoC cycles stalled on instruction fetches (I-cache misses: flash / bootrom over the ROM bus)
//   word 2  SoC cycles stalled on data accesses to flash (D-cache misses over the ROM bus)
//   word 3  SoC cycles stalled on data accesses to RAM (D-cache misses and write-backs over the RAM bus)
//   word 4  game ticks: the firmware read the SNES end-of-frame message ($111 from RX_DATA) and then posted
//           its next stream descriptor (the same measure as the RTL co-simulation)
//   word 5  game ticks that took longer than one SNES frame (16.64 ms)
//   word 6  longest game tick since the previous snapshot (SoC cycles)
//   word 7  window: SNES reads that found the prefetch ring empty while a descriptor was queued
// The SoC counters run in clk_soc and are copied on a synchronized toggle; the MCU reads them a few
// microseconds after $C7, when they are static. The bus monitor signals are registered first, so the
// counters stay off the core's timing paths.
//------------------------------------------------------------------------------
`ifdef MK2
assign perf_data = 256'd0;   // mk2: no performance counters
`else
reg perf_tog2 = 1'b0;
reg [31:0] perf_underrun_s;
always @(posedge clk2) if(perf_snap) begin perf_tog2 <= ~perf_tog2; perf_underrun_s <= stat_underrun; end

(* altera_attribute = "-name SYNCHRONIZER_IDENTIFICATION FORCED_IF_ASYNCHRONOUS" *) reg [2:0] perf_tog_s = 3'b000;
reg m_req, m_ready, m_we, m_fetch, m_flash, m_ram, m_msg_rd, m_post;
reg have_msg = 1'b0;
reg [31:0] pc_cyc = 0, pc_fstall = 0, pc_dflash = 0, pc_dram = 0, pc_ticks = 0, pc_late = 0, pc_max = 0, tick_lat = 0;
reg [31:0] ps_cyc, ps_fstall, ps_dflash, ps_dram, ps_ticks, ps_late, ps_max;
localparam [31:0] LATE = SOC_CLK_NUM * 16640 / SOC_CLK_DEN;   // 16.64 ms in SoC cycles
always @(posedge clk_soc) begin
  perf_tog_s <= {perf_tog_s[1:0], perf_tog2};
  m_req <= mon_req; m_ready <= mon_ready; m_we <= mon_we; m_fetch <= mon_fetch;
  m_flash <= (mon_addr[31:28] == 4'h1) || (mon_addr[31:14] == 18'd0);
  m_ram <= (mon_addr[31:28] == 4'h2);
  m_msg_rd <= mon_req & mon_ready & ~mon_we & ~mon_fetch & (mon_addr == 32'h50803010) & (mon_rdata == 32'h111);
  m_post <= mon_req & mon_ready & mon_we & (mon_addr == 32'h50803004) & (mon_wdata != 32'd0);
  pc_cyc <= pc_cyc + 1'b1;
  if(m_req & ~m_ready) begin
    if(m_fetch) pc_fstall <= pc_fstall + 1'b1;
    else if(m_flash) pc_dflash <= pc_dflash + 1'b1;
    else if(m_ram) pc_dram <= pc_dram + 1'b1;
  end
  if(have_msg) tick_lat <= tick_lat + 1'b1;
  if(m_msg_rd && !have_msg) begin have_msg <= 1'b1; tick_lat <= 32'd0; end
  else if(m_post && have_msg) begin
    have_msg <= 1'b0;
    pc_ticks <= pc_ticks + 1'b1;
    if(tick_lat > LATE) pc_late <= pc_late + 1'b1;
    if(tick_lat > pc_max) pc_max <= tick_lat;
  end
  if(perf_tog_s[2] != perf_tog_s[1]) begin
    ps_cyc <= pc_cyc; ps_fstall <= pc_fstall; ps_dflash <= pc_dflash; ps_dram <= pc_dram;
    ps_ticks <= pc_ticks; ps_late <= pc_late; ps_max <= pc_max;
    pc_max <= 32'd0;
  end
  if(rst_soc) have_msg <= 1'b0;
end
assign perf_data = {perf_underrun_s, ps_max, ps_late, ps_ticks, ps_dram, ps_dflash, ps_fstall, ps_cyc};
`endif

endmodule
