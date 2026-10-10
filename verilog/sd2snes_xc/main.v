`timescale 1 ns / 1 ns
//////////////////////////////////////////////////////////////////////////////////
// Company: Rehkopf
// Engineer: Rehkopf
//
// Create Date:    01:13:46 05/09/2009
// Design Name:
// Module Name:    main
// Project Name:
// Target Devices:
// Tool versions:
// Description: Master Control FSM
//              sd2snes_xc: Xeno Crisis (Bitmap Bureau, RP2040 coprocessor) core, derived from sd2snes_gsu.
//              The RP2040 is replaced by xc_top (soft Thumb CPU + $3000 window with DMA + Opus mailbox),
//              see xc_top.v and SD2SNES_CORE.md. mk3 only.
//
// Dependencies: address
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////
`include "config.vh"

module main(
`ifdef MK2
  /* Bus 1: PSRAM, 128Mbit, 16bit, 70ns */
  output [22:0] ROM_ADDR,
  output ROM_CE,
  input MCU_OVR,
  /* debug */
  output p113_out,
`endif
`ifdef MK3
  input SNES_CIC_CLK,
  /* Bus 1: 2x PSRAM, 64Mbit, 16bit, 70ns */
  output [21:0] ROM_ADDR,
  output ROM_1CE,
  output ROM_2CE,
  output ROM_ZZ,
  /* debug */
  output PM6_out,
  output PN6_out,
  input  PT5_in,
`endif

  /* input clock */
  input CLKIN,

  /* SNES signals */
  input [23:0] SNES_ADDR_IN,
  input SNES_READ_IN,
  input SNES_WRITE_IN,
  input SNES_ROMSEL_IN,
  inout [7:0] SNES_DATA,
  input SNES_CPU_CLK_IN,
  input SNES_REFRESH,
  output SNES_IRQ,
  output SNES_DATABUS_OE,
  output SNES_DATABUS_DIR,
  input SNES_SYSCLK,

  input [7:0] SNES_PA_IN,
  input SNES_PARD_IN,
  input SNES_PAWR_IN,

  /* SRAM signals */
  inout [15:0] ROM_DATA,
  output ROM_OE,
  output ROM_WE,
  output ROM_BHE,
  output ROM_BLE,

  /* Bus 2: SRAM, 4Mbit, 8bit, 45ns */
  inout [7:0] RAM_DATA,
  output [18:0] RAM_ADDR,
  output RAM_OE,
  output RAM_WE,

  /* MCU signals */
  input SPI_MOSI,
  inout SPI_MISO,
  input SPI_SS,
  input SPI_SCK,

  output MCU_RDY,

  output DAC_MCLK,
  output DAC_LRCK,
  output DAC_SDOUT,

  /* SD signals */
  input [3:0] SD_DAT,
  inout SD_CMD,
  inout SD_CLK
);

wire CLK2;

wire dspx_dp_enable;

wire [7:0] spi_cmd_data;
wire [7:0] spi_param_data;
wire [7:0] spi_input_data;
wire [31:0] spi_byte_cnt;
wire [2:0] spi_bit_cnt;
wire [23:0] MCU_ADDR;
wire [2:0] MAPPER;
wire [23:0] SAVERAM_MASK;
wire [23:0] ROM_MASK;
wire [7:0] SD_DMA_SRAM_DATA;
wire [1:0] SD_DMA_TGT;
wire [10:0] SD_DMA_PARTIAL_START;
wire [10:0] SD_DMA_PARTIAL_END;

wire [10:0] dac_addr;
wire [2:0] dac_vol_select_out;
wire [8:0] dac_ptr_addr;
//wire [7:0] dac_volume;
wire [7:0] msu_volumerq_out;
wire [7:0] msu_status_out;
wire [31:0] msu_addressrq_out;
wire [15:0] msu_trackrq_out;
wire [13:0] msu_write_addr;
wire [13:0] msu_ptr_addr;
wire [7:0] MSU_SNES_DATA_IN;
wire [7:0] MSU_SNES_DATA_OUT;
wire [5:0] msu_status_reset_bits;
wire [5:0] msu_status_set_bits;

wire [9:0] GSU_PGM_ADDR;
wire [7:0] GSU_PGM_DATA = 8'h00;
wire CLK_SOC;

wire [15:0] featurebits;
wire feat_cmd_unlock = featurebits[5];

wire [23:0] MAPPED_SNES_ADDR;
wire ROM_ADDR0;

//wire [4:0] DBG_srtc_state;
//wire DBG_srtc_we_rising;
//wire [3:0] DBG_srtc_ptr;
//wire [5:0] DBG_srtc_we_sreg;
wire [13:0] DBG_msu_address;
wire DBG_msu_reg_oe_rising;
wire DBG_msu_reg_oe_falling;
wire DBG_msu_reg_we_rising;
wire [2:0] SD_DMA_DBG_clkcnt;
wire [10:0] SD_DMA_DBG_cyclecnt;

wire [15:0] dsp_feat;

wire [8:0] snescmd_addr_mcu;
wire [7:0] snescmd_data_out_mcu;
wire [7:0] snescmd_data_in_mcu;

// config
wire [7:0] reg_group;
wire [7:0] reg_index;
wire [7:0] reg_value;
wire [7:0] reg_invmask;
wire       reg_we;
wire [7:0] reg_read;
// unit level configuration output
wire [7:0] gsu_config_data = 8'h00;

reg [7:0] SNES_PARDr = 8'b11111111;
reg [7:0] SNES_PAWRr = 8'b11111111;
reg [7:0] SNES_READr = 8'b11111111;
reg [7:0] SNES_WRITEr = 8'b11111111;
reg [7:0] SNES_CPU_CLKr = 8'b00000000;
reg [7:0] SNES_ROMSELr = 8'b11111111;
reg [7:0] SNES_PULSEr = 8'b11111111;
reg [23:0] SNES_ADDRr [6:0];
reg [7:0] SNES_PAr [6:0];
reg [7:0] SNES_DATAr [4:0];

reg SNES_DEADr = 1;
reg SNES_reset_strobe = 0;

reg free_strobe = 0;
reg ram_free_strobe = 0;

wire [23:0] SNES_ADDR = (SNES_ADDRr[5] & SNES_ADDRr[4]);
wire [7:0] SNES_PA = (SNES_PAr[5] & SNES_PAr[4]);
wire [7:0] SNES_DATA_IN = (SNES_DATAr[3] & SNES_DATAr[2]);

wire SNES_PULSE_IN = SNES_READ_IN & SNES_WRITE_IN & ~SNES_CPU_CLK_IN;

wire SNES_PULSE_end = (SNES_PULSEr[6:1] == 6'b000011);
wire SNES_PARD_start = (SNES_PARDr[6:1] == 6'b111110);
wire SNES_PARD_end = (SNES_PARDr[6:1] == 6'b000001);
// Sample PAWR data earlier on CPU accesses, later on DMA accesses...
wire SNES_PAWR_start = (SNES_PAWRr[6:1] == (({SNES_ADDR[22], SNES_ADDR[15:0]} == 17'h02100) ? 6'b111000 : 6'b100000));
wire SNES_PAWR_end = (SNES_PAWRr[6:1] == 6'b000001);
wire SNES_RD_start = (SNES_READr[6:1] == 6'b111110);
wire SNES_RD_end = (SNES_READr[6:1] == 6'b000001);
wire SNES_WR_end = (SNES_WRITEr[6:1] == 6'b000001);
wire SNES_cycle_start = (SNES_CPU_CLKr[6:1] == 6'b000001);
wire SNES_cycle_end = (SNES_CPU_CLKr[6:1] == 6'b111110);
wire SNES_WRITE = SNES_WRITEr[2] & SNES_WRITEr[1];
wire SNES_READ = SNES_READr[2] & SNES_READr[1];
wire SNES_READ_late = SNES_READr[5] & SNES_READr[4];
wire SNES_READ_narrow = SNES_READ | SNES_READ_late;
wire SNES_CPU_CLK = SNES_CPU_CLKr[2] & SNES_CPU_CLKr[1];
wire SNES_PARD = SNES_PARDr[2] & SNES_PARDr[1];
wire SNES_PAWR = SNES_PAWRr[2] & SNES_PAWRr[1];

wire SNES_ROMSEL = (SNES_ROMSELr[5] & SNES_ROMSELr[4]);

reg [7:0] BUS_DATA;

always @(posedge CLK2) begin
  if(~SNES_READ) BUS_DATA <= SNES_DATA;
  else if(~SNES_WRITE) BUS_DATA <= SNES_DATA_IN;
end

wire SD_DMA_TO_ROM;
wire free_slot = (SNES_PULSE_end | free_strobe) & ~SD_DMA_TO_ROM;

wire ROM_HIT;

assign DCM_RST=0;

wire IS_SAVERAM;

// Provide full bandwidth if snes is not accessing the bus.
always @(posedge CLK2) begin
  if (SNES_cycle_start) free_strobe <= ~ROM_HIT | IS_SAVERAM;
  else free_strobe <= 1'b0;
end

always @(posedge CLK2) begin
  SNES_PULSEr <= {SNES_PULSEr[6:0], SNES_PULSE_IN};
  SNES_PARDr <= {SNES_PARDr[6:0], SNES_PARD_IN};
  SNES_PAWRr <= {SNES_PAWRr[6:0], SNES_PAWR_IN};
  SNES_READr <= {SNES_READr[6:0], SNES_READ_IN};
  SNES_WRITEr <= {SNES_WRITEr[6:0], SNES_WRITE_IN};
  SNES_CPU_CLKr <= {SNES_CPU_CLKr[6:0], SNES_CPU_CLK_IN};
  SNES_ROMSELr <= {SNES_ROMSELr[6:0], SNES_ROMSEL_IN};
  SNES_ADDRr[6] <= SNES_ADDRr[5];
  SNES_ADDRr[5] <= SNES_ADDRr[4];
  SNES_ADDRr[4] <= SNES_ADDRr[3];
  SNES_ADDRr[3] <= SNES_ADDRr[2];
  SNES_ADDRr[2] <= SNES_ADDRr[1];
  SNES_ADDRr[1] <= SNES_ADDRr[0];
  SNES_ADDRr[0] <= SNES_ADDR_IN;
  SNES_PAr[6] <= SNES_PAr[5];
  SNES_PAr[5] <= SNES_PAr[4];
  SNES_PAr[4] <= SNES_PAr[3];
  SNES_PAr[3] <= SNES_PAr[2];
  SNES_PAr[2] <= SNES_PAr[1];
  SNES_PAr[1] <= SNES_PAr[0];
  SNES_PAr[0] <= SNES_PA_IN;
  SNES_DATAr[4] <= SNES_DATAr[3];
  SNES_DATAr[3] <= SNES_DATAr[2];
  SNES_DATAr[2] <= SNES_DATAr[1];
  SNES_DATAr[1] <= SNES_DATAr[0];
  SNES_DATAr[0] <= SNES_DATA;
end

parameter ST_IDLE            = 11'b00000000001;
parameter ST_MCU_RD_ADDR     = 11'b00000000010;
parameter ST_MCU_RD_END      = 11'b00000000100;
parameter ST_MCU_WR_ADDR     = 11'b00000001000;
parameter ST_MCU_WR_END      = 11'b00000010000;
parameter ST_GSU_ROM_RD_ADDR = 11'b00000100000;
parameter ST_GSU_ROM_RD_END  = 11'b00001000000;
parameter ST_GSU_RAM_RD_ADDR = 11'b00010000000;
parameter ST_GSU_RAM_RD_END  = 11'b00100000000;
parameter ST_GSU_RAM_WR_ADDR = 11'b01000000000;
parameter ST_GSU_RAM_WR_END  = 11'b10000000000;

`ifdef MK2
parameter SNES_DEAD_TIMEOUT = 17'd85714; // 1ms
`endif
`ifdef MK3
parameter SNES_DEAD_TIMEOUT = 17'd85867; // 1ms
`endif

parameter ROM_CYCLE_LEN = 4'd7; // Increased from 6 due to tight timing on some sd2snes.

reg [10:0] STATE;
initial STATE = ST_IDLE;

//assign SRTC_SNES_DATA_IN = BUS_DATA[3:0];
assign MSU_SNES_DATA_IN = BUS_DATA;

sd_dma snes_sd_dma(
  .CLK(CLK2),
  .SD_DAT(SD_DAT),
  .SD_CLK(SD_CLK),
  .SD_DMA_EN(SD_DMA_EN),
  .SD_DMA_STATUS(SD_DMA_STATUS),
  .SD_DMA_SRAM_WE(SD_DMA_SRAM_WE),
  .SD_DMA_SRAM_DATA(SD_DMA_SRAM_DATA),
  .SD_DMA_NEXTADDR(SD_DMA_NEXTADDR),
  .SD_DMA_PARTIAL(SD_DMA_PARTIAL),
  .SD_DMA_PARTIAL_START(SD_DMA_PARTIAL_START),
  .SD_DMA_PARTIAL_END(SD_DMA_PARTIAL_END),
  .SD_DMA_START_MID_BLOCK(SD_DMA_START_MID_BLOCK),
  .SD_DMA_END_MID_BLOCK(SD_DMA_END_MID_BLOCK),
  .DBG_cyclecnt(SD_DMA_DBG_cyclecnt),
  .DBG_clkcnt(SD_DMA_DBG_clkcnt)
);

assign SD_DMA_TO_ROM = (SD_DMA_STATUS && (SD_DMA_TGT == 2'b00));

// MSU-1 and the audio DAC (`define XC_MSU). Mk.III (XC_MK3 as well): fpga_xc.bi3 has both the Opus mailbox and
// the MSU-1, chosen at load time; Mk.II: MSU-1 only. With the MSU-1 the music comes from a pack: the soft CPU
// writes the MSU-1 registers (xc_msubox in xc_top), the SNES does not see them. Its msu.v is audio-only (no
// 16 KB data buffer), xc_dac.v is dac.v with linear interpolation instead of the CIC (fits next to the soft CPU).
wire       xc_msu_we;
wire [2:0] xc_msu_addr;
wire [7:0] xc_msu_data;
wire [7:0] xc_msu_status;
`ifdef XC_MSU
xc_dac snes_dac(
  .clkin(CLK2),
  .sysclk(SNES_SYSCLK),
  .mclk_out(DAC_MCLK),
  .lrck_out(DAC_LRCK),
  .sdout(DAC_SDOUT),
  .sclk_out(),
  .we(SD_DMA_TGT==2'b01 ? SD_DMA_SRAM_WE : 1'b1),
  .pgm_address(dac_addr),
  .pgm_data(SD_DMA_SRAM_DATA),
  .DAC_STATUS(DAC_STATUS),
  .volume(msu_volumerq_out),
  .vol_latch(msu_volume_latch_out),
  .vol_select(dac_vol_select_out),
  .palmode(dac_palmode_out),
  .play(dac_play),
  .reset(dac_reset),
  .dac_address_ext(dac_ptr_addr)
);

msu snes_msu (
  .clkin(CLK2),
  .enable(1'b1),
  .pgm_address(msu_write_addr),
  .pgm_data(SD_DMA_SRAM_DATA),
  .pgm_we(1'b1),
  .reg_addr(xc_msu_addr),
  .reg_data_in(xc_msu_data),
  .reg_data_out(),
  .reg_oe_falling(1'b0),
  .reg_oe_rising(1'b0),
  .reg_we_rising(xc_msu_we),
  .status_out(msu_status_out),
  .reg_status(xc_msu_status),
  .volume_out(msu_volumerq_out),
  .volume_latch_out(msu_volume_latch_out),
  .addr_out(msu_addressrq_out),
  .track_out(msu_trackrq_out),
  .status_reset_bits(msu_status_reset_bits),
  .status_set_bits(msu_status_set_bits),
  .status_reset_we(msu_status_reset_we),
  .msu_address_ext(msu_ptr_addr),
  .msu_address_ext_write(msu_addr_reset),
  .DBG_msu_reg_oe_rising(),
  .DBG_msu_reg_oe_falling(),
  .DBG_msu_reg_we_rising(),
  .DBG_msu_address(),
  .DBG_msu_address_ext_write_rising()
);
assign MSU_SNES_DATA_OUT = 8'h00;
`else
assign DAC_MCLK = 1'b0;
assign DAC_LRCK = 1'b0;
assign DAC_SDOUT = 1'b0;
wire DAC_STATUS = 1'b0;
assign msu_status_out = 8'h00;
assign MSU_SNES_DATA_OUT = 8'h00;
assign msu_volumerq_out = 8'h00;
assign msu_addressrq_out = 32'h0;
assign msu_trackrq_out = 16'h0;
assign xc_msu_status = 8'h00;
`endif

spi snes_spi(
  .clk(CLK2),
  .MOSI(SPI_MOSI),
  .MISO(SPI_MISO),
  .SSEL(SPI_SS),
  .SCK(SPI_SCK),
  .cmd_ready(spi_cmd_ready),
  .param_ready(spi_param_ready),
  .cmd_data(spi_cmd_data),
  .param_data(spi_param_data),
  .endmessage(spi_endmessage),
  .startmessage(spi_startmessage),
  .input_data(spi_input_data),
  .byte_cnt(spi_byte_cnt),
  .bit_cnt(spi_bit_cnt)
);

// Xeno Crisis coprocessor (xc_top): ROM (PSRAM) reads and RAM (SRAM chip) accesses use the ports the
// GSU core had. The SoC is held in reset while the SNES is (SNES_DEADr), like the cartridge's RP2040.
reg  [15:0] XC_ROM_DINr;
wire [23:0] XC_ROM_ADDR;
wire        XC_ROM_RRQ;
wire        XC_ROM_RDY;
reg  [7:0]  XC_RAM_DINr;
wire [18:0] XC_RAM_ADDR;
wire [7:0]  XC_RAM_DOUT;
wire        XC_RAM_RRQ, XC_RAM_WRQ, XC_RAM_RDY;
wire [5:0]  XC_RAM_LEN;              // burst length in bytes (1..32), with XC_RAM_RRQ / XC_RAM_WRQ
wire        XC_RAM_BSTB;             // burst byte strobe (see the RAM pipeline)
wire [7:0]  XC_SNES_DATA_OUT;
wire        xc_enable;

wire [1:0]  xca_status;
wire [10:0] xca_len;
wire        xca_pkt_start, xca_pkt_rd, xca_pcm_start, xca_pcm_wr, xca_done, xca_ack_reset;
wire [7:0]  xca_pkt_data, xca_pcm_data;
wire [31:0] xca_ret, xca_range;
wire        xc_running, xc_halted;
wire        xc_run;
`ifndef MK2
wire        xc_msu_mode;
`endif
wire [31:0] xc_halt_code, xc_halt_addr;
wire        xc_perf_snap;
wire [255:0] xc_perf;

`ifdef MK2
// mk2: soft CPU clock 22 MHz (soc_dcm: 24 MHz x 11 / 12, must match SOC_CLK_NUM / SOC_CLK_DEN); one 2-way cache for
// code and data, 2^DIDX sets x 2 ways x 32 B (8 = 16 KB: 8 block RAMs for the data, one per byte lane and way)
xc_top #(.SOC_CLK_NUM(22), .SOC_CLK_DEN(1), .STATS(0), .DIDX(8)) snes_xc (
`else
// soft CPU clock 40.25 MHz (must match the PLL's clk1); caches: D$ 2^DIDX sets, I$ 2^IIDX sets (x 2 ways x 32 B)
// = 16 KB each. If Quartus misses timing on clk[1], try IIDX 7 (8 KB I$: same M9K count as a 4 KB one).
`ifdef XC_MK3
xc_top #(.SOC_CLK_NUM(161), .SOC_CLK_DEN(4), .STATS(1), .DIDX(8), .IIDX(8), .MSU(2)) snes_xc (
`elsif XC_MSU
xc_top #(.SOC_CLK_NUM(161), .SOC_CLK_DEN(4), .STATS(1), .DIDX(8), .IIDX(8), .MSU(1)) snes_xc (
`else
xc_top #(.SOC_CLK_NUM(161), .SOC_CLK_DEN(4), .STATS(1), .DIDX(8), .IIDX(8), .MSU(0)) snes_xc (
`endif
`endif
  .clk2(CLK2),
  .clk_soc(CLK_SOC),
  .rst2(SNES_DEADr),
  .soc_run(xc_run),                 // MCU: image and save loaded
  .win_enable(xc_enable),
  .snes_rd_start(SNES_RD_start),
  .snes_rd_end(SNES_RD_end),
  .snes_wr_end(SNES_WR_end),
  .snes_data_in(BUS_DATA),
  .snes_data_out(XC_SNES_DATA_OUT),
  .rom_rrq(XC_ROM_RRQ),
  .rom_addr(XC_ROM_ADDR),
  .rom_rdy(XC_ROM_RDY),
  .rom_rdata(XC_ROM_DINr),
  .ram_rrq(XC_RAM_RRQ),
  .ram_wrq(XC_RAM_WRQ),
  .ram_addr(XC_RAM_ADDR),
  .ram_wdata(XC_RAM_DOUT),
  .ram_len(XC_RAM_LEN),
  .ram_bstb(XC_RAM_BSTB),
  .ram_rdy(XC_RAM_RDY),
  .ram_rdata(XC_RAM_DINr),
  .mcu_status(xca_status),
  .mcu_len(xca_len),
  .mcu_pkt_start(xca_pkt_start),
  .mcu_pkt_rd(xca_pkt_rd),
  .mcu_pkt_data(xca_pkt_data),
  .mcu_pcm_start(xca_pcm_start),
  .mcu_pcm_wr(xca_pcm_wr),
  .mcu_pcm_data(xca_pcm_data),
  .mcu_done(xca_done),
  .mcu_ret(xca_ret),
  .mcu_range(xca_range),
  .mcu_ack_reset(xca_ack_reset),
  .msu_we(xc_msu_we),
  .msu_addr(xc_msu_addr),
  .msu_data(xc_msu_data),
  .msu_status(xc_msu_status),
`ifndef MK2
  .msu_mode(xc_msu_mode),
`endif
  .soc_running(xc_running),
  .soc_halted(xc_halted),
  .soc_halt_code(xc_halt_code),
  .soc_halt_addr(xc_halt_addr),
  .soc_dbg_char_valid(),
  .soc_dbg_char(),
  .stat_tx_read(),
  .stat_rx_written(),
  .stat_desc(),
  .stat_underrun(),
  .perf_snap(xc_perf_snap),
  .perf_data(xc_perf),
  .mon_req(), .mon_ready(), .mon_we(), .mon_fetch(), .mon_size(), .mon_addr(), .mon_wdata(), .mon_rdata(),
  .mon_dma_rd(), .mon_dma_addr(), .mon_dma_data()
);

reg [7:0] MCU_DINr;
reg [7:0] MCU_ROM_DINr;
reg [7:0] MCU_RAM_DINr;
wire [7:0] MCU_DOUT;
wire [31:0] cheat_pgm_data;
wire [7:0] cheat_data_out;
wire [2:0] cheat_pgm_idx;

mcu_cmd snes_mcu_cmd(
  .clk(CLK2),
  .snes_sysclk(SNES_SYSCLK),
  .cmd_ready(spi_cmd_ready),
  .param_ready(spi_param_ready),
  .cmd_data(spi_cmd_data),
  .param_data(spi_param_data),
  .mcu_mapper(MAPPER),
  .mcu_write(MCU_WRITE),
  .mcu_data_in(MCU_DINr),
  .mcu_data_out(MCU_DOUT),
  .spi_byte_cnt(spi_byte_cnt),
  .spi_bit_cnt(spi_bit_cnt),
  .spi_data_out(spi_input_data),
  .addr_out(MCU_ADDR),
  .saveram_mask_out(SAVERAM_MASK),
  .rom_mask_out(ROM_MASK),
  .SD_DMA_EN(SD_DMA_EN),
  .SD_DMA_STATUS(SD_DMA_STATUS),
  .SD_DMA_NEXTADDR(SD_DMA_NEXTADDR),
  .SD_DMA_SRAM_DATA(SD_DMA_SRAM_DATA),
  .SD_DMA_SRAM_WE(SD_DMA_SRAM_WE),
  .SD_DMA_TGT(SD_DMA_TGT),
  .SD_DMA_PARTIAL(SD_DMA_PARTIAL),
  .SD_DMA_PARTIAL_START(SD_DMA_PARTIAL_START),
  .SD_DMA_PARTIAL_END(SD_DMA_PARTIAL_END),
  .SD_DMA_START_MID_BLOCK(SD_DMA_START_MID_BLOCK),
  .SD_DMA_END_MID_BLOCK(SD_DMA_END_MID_BLOCK),
  .dac_addr_out(dac_addr),
  .DAC_STATUS(DAC_STATUS),
  .dac_play_out(dac_play),
  .dac_reset_out(dac_reset),
  .dac_vol_select_out(dac_vol_select_out),
  .dac_palmode_out(dac_palmode_out),
  .dac_ptr_out(dac_ptr_addr),
  .msu_addr_out(msu_write_addr),
  .MSU_STATUS(msu_status_out),
  .msu_status_reset_out(msu_status_reset_bits),
  .msu_status_set_out(msu_status_set_bits),
  .msu_status_reset_we(msu_status_reset_we),
  .msu_volumerq(msu_volumerq_out),
  .msu_addressrq(msu_addressrq_out),
  .msu_trackrq(msu_trackrq_out),
  .msu_ptr_out(msu_ptr_addr),
  .msu_reset_out(msu_addr_reset),
  .gsu_addr_out(GSU_PGM_ADDR),
  .gsu_data(GSU_PGM_DATA),
  // config
  .reg_group_out(reg_group),
  .reg_index_out(reg_index),
  .reg_value_out(reg_value),
  .reg_invmask_out(reg_invmask),
  .reg_we_out(reg_we),
  .reg_read_out(reg_read),
  // vv config data in vv
  .gsu_config_data_in(gsu_config_data),
  // ^^ config data in ^^
  .featurebits_out(featurebits),
  .mcu_rrq(MCU_RRQ),
  .mcu_wrq(MCU_WRQ),
  .mcu_rq_rdy(MCU_RDY),
  .region_out(mcu_region),
  .snescmd_addr_out(snescmd_addr_mcu),
  .snescmd_we_out(snescmd_we_mcu),
  .snescmd_data_out(snescmd_data_out_mcu),
  .snescmd_data_in(snescmd_data_in_mcu),
  .cheat_pgm_idx_out(cheat_pgm_idx),
  .cheat_pgm_data_out(cheat_pgm_data),
  .cheat_pgm_we_out(cheat_pgm_we),
  .dsp_feat_out(dsp_feat),
  // Xeno Crisis decode service
  .xca_status(xca_status),
  .xca_len(xca_len),
  .xca_pkt_start(xca_pkt_start),
  .xca_pkt_rd(xca_pkt_rd),
  .xca_pkt_data(xca_pkt_data),
  .xca_pcm_start(xca_pcm_start),
  .xca_pcm_wr(xca_pcm_wr),
  .xca_pcm_data(xca_pcm_data),
  .xca_done(xca_done),
  .xca_ret(xca_ret),
  .xca_range(xca_range),
  .xca_ack_reset(xca_ack_reset),
  .xc_run(xc_run),
`ifndef MK2
  .xc_msu_mode(xc_msu_mode),
`endif
  .xc_status({xc_running, xc_halted}),
  .xc_halt_code(xc_halt_code),
  .xc_halt_addr(xc_halt_addr),
  .xc_perf_snap(xc_perf_snap),
  .xc_perf(xc_perf)
);

address snes_addr(
  .CLK(CLK2),
  .MAPPER(MAPPER),
  .featurebits(featurebits),
  .SNES_ADDR(SNES_ADDR), // requested address from SNES
  .SNES_PA(SNES_PA),
  .SNES_ROMSEL(SNES_ROMSEL),
  .ROM_ADDR(MAPPED_SNES_ADDR),   // Address to request from SRAM (active low)
  .ROM_HIT(ROM_HIT),     // want to access RAM0
  .IS_SAVERAM(IS_SAVERAM),
  .IS_ROM(IS_ROM),
  .IS_WRITABLE(IS_WRITABLE),
  .SAVERAM_MASK(SAVERAM_MASK),
  .ROM_MASK(ROM_MASK),
  //MSU-1
  .msu_enable(msu_enable),
  //Xeno Crisis $3000 window
  .xc_enable(xc_enable),
  .r213f_enable(r213f_enable),
  .r2100_hit(r2100_hit),
  .snescmd_enable(snescmd_enable),
  .nmicmd_enable(nmicmd_enable),
  .return_vector_enable(return_vector_enable),
  .branch1_enable(branch1_enable),
  .branch2_enable(branch2_enable),
  .branch3_enable(branch3_enable)
);

reg pad_latch = 0;
reg [4:0] pad_cnt = 0;

reg snes_ajr = 0;

// The cheat engine and in-game hooks are left out too (the game's SNES code is streamed from the
// cartridge, so ROM patches and NMI hooks do not apply). Define XC_WITH_CHEAT to put them back.
`ifdef XC_WITH_CHEAT
cheat snes_cheat(
  .clk(CLK2),
  .SNES_ADDR(SNES_ADDR),
  .SNES_PA(SNES_PA),
  .SNES_DATA(SNES_DATA),
  .SNES_reset_strobe(SNES_reset_strobe),
  .SNES_wr_strobe(SNES_WR_end),
  .SNES_rd_strobe(SNES_RD_start),
  .snescmd_enable(snescmd_enable),
  .nmicmd_enable(nmicmd_enable),
  .return_vector_enable(return_vector_enable),
  .branch1_enable(branch1_enable),
  .branch2_enable(branch2_enable),
  .branch3_enable(branch3_enable),
  .pad_latch(pad_latch),
  .snes_ajr(snes_ajr),
  .SNES_cycle_start(SNES_cycle_start),
  .pgm_idx(cheat_pgm_idx),
  .pgm_we(cheat_pgm_we),
  .pgm_in(cheat_pgm_data),
  .gsu_vec_enable(1'b0),
  .data_out(cheat_data_out),
  .cheat_hit(cheat_hit),
  .snescmd_unlock(snescmd_unlock)
);
`else
assign cheat_data_out = 8'h00;
wire cheat_hit = 1'b0;
wire snescmd_unlock = 1'b0;
`endif

wire [7:0] snescmd_dout;

parameter ST_R213F_ARMED     = 4'b0001;
parameter ST_R213F_WAITBUS   = 4'b0010;
parameter ST_R213F_OVERRIDE  = 4'b0100;
parameter ST_R213F_HOLD      = 4'b1000;

reg [7:0] r213fr;
reg r213f_forceread;
reg [2:0] r213f_delay;
reg [1:0] r213f_state;
initial r213fr = 8'h55;
initial r213f_forceread = 0;
initial r213f_state = 2'b01;
initial r213f_delay = 3'b000;

reg [7:0] r2100r = 0;
reg r2100_forcewrite = 0;
reg r2100_forcewrite_pre = 0;
wire [3:0] r2100_limit = featurebits[10:7];
wire [3:0] r2100_limited = (SNES_DATA[3:0] > r2100_limit) ? r2100_limit : SNES_DATA[3:0];
wire r2100_patch = featurebits[6];
wire r2100_enable = r2100_hit & (r2100_patch | ~(&r2100_limit));

wire snoop_4200_enable = {SNES_ADDR[22], SNES_ADDR[15:0]} == 17'h04200;
wire r4016_enable = {SNES_ADDR[22], SNES_ADDR[15:0]} == 17'h04016;

always @(posedge CLK2) begin
  r2100_forcewrite <= r2100_forcewrite_pre;
end

always @(posedge CLK2) begin
  if(SNES_WR_end & snoop_4200_enable) begin
    snes_ajr <= SNES_DATA[0];
  end
end

always @(posedge CLK2) begin
  if(SNES_WR_end & r4016_enable) begin
    pad_latch <= 1'b1;
    pad_cnt <= 5'h0;
  end
  if(SNES_RD_start & r4016_enable) begin
    pad_cnt <= pad_cnt + 1;
    if(&pad_cnt[3:0]) begin
      pad_latch <= 1'b0;
    end
  end
end

assign SNES_DATA = (r213f_enable & ~SNES_PARD & ~r213f_forceread) ? r213fr
                   :(r2100_enable & ~SNES_PAWR & r2100_forcewrite) ? r2100r
                   :((~SNES_READ ^ (r213f_forceread & r213f_enable & ~SNES_PARD))
                                & ~(r2100_enable & ~SNES_PAWR & ~r2100_forcewrite & ~IS_ROM & ~IS_WRITABLE))
                                ? ( msu_enable ? MSU_SNES_DATA_OUT
                                  : xc_enable ? XC_SNES_DATA_OUT  // $3000 window
                                  : (cheat_hit & ~feat_cmd_unlock) ? cheat_data_out
                                  : ((snescmd_unlock | feat_cmd_unlock) & snescmd_enable) ? snescmd_dout
                                  : (ROM_HIT & IS_SAVERAM) ? RAM_DATA
                                  : (ROM_ADDR0 ? ROM_DATA[7:0] : ROM_DATA[15:8])
                                  ) : 8'bZ;

reg [3:0] ST_MEM_DELAYr;

// MCU
reg MCU_RD_PENDr = 0;
reg MCU_WR_PENDr = 0;
reg [23:0] ROM_ADDRr;

reg RQ_MCU_RDYr;
initial RQ_MCU_RDYr = 1'b1;

wire MCU_WE_HIT = |(STATE & ST_MCU_WR_ADDR);
wire MCU_WR_HIT = |(STATE & (ST_MCU_WR_ADDR | ST_MCU_WR_END));
wire MCU_RD_HIT = |(STATE & (ST_MCU_RD_ADDR | ST_MCU_RD_END));
wire MCU_HIT = MCU_WR_HIT | MCU_RD_HIT;

// XC ROM (16-bit word reads)
reg GSU_ROM_RD_PENDr; initial GSU_ROM_RD_PENDr = 0;
reg [23:0] GSU_ROM_ADDRr;
wire       GSU_ROM_WORDr = 1'b1;

reg RQ_GSU_ROM_RDYr; initial RQ_GSU_ROM_RDYr = 1;
assign XC_ROM_RDY = RQ_GSU_ROM_RDYr;

wire GSU_ROM_HIT = |(STATE & ST_GSU_ROM_RD_ADDR);

`ifdef MK2
my_dcm snes_dcm(
  .CLKIN(CLKIN),
  .CLKFX(CLK2),
  .LOCKED(DCM_LOCKED),
  .RST(DCM_RST)
);

`ifdef MK2
// Xeno Crisis: the soft CPU's own clock (xc_bridge crosses to CLK2)
soc_dcm xc_soc_dcm(
  .CLKIN(CLKIN),
  .CLKFX(CLK_SOC),
  .LOCKED(),
  .RST(DCM_RST)
);

`endif
assign ROM_ADDR  = (SD_DMA_TO_ROM) ? MCU_ADDR[23:1] : GSU_ROM_HIT ? GSU_ROM_ADDRr[23:1] : MCU_HIT ? ROM_ADDRr[23:1] : MAPPED_SNES_ADDR[23:1];
assign ROM_ADDR0 = (SD_DMA_TO_ROM) ? MCU_ADDR[0]    : GSU_ROM_HIT ? GSU_ROM_ADDRr[0]    : MCU_HIT ? ROM_ADDRr[0]    : MAPPED_SNES_ADDR[0];
assign ROM_CE = 1'b0;

assign p113_out = 1'b0;

snescmd_buf snescmd (
  .clka(CLK2), // input clka
  .wea(SNES_WR_end & ((snescmd_unlock | feat_cmd_unlock) & snescmd_enable)), // input [0 : 0] wea
  .addra(SNES_ADDR[8:0]), // input [8 : 0] addra
  .dina(SNES_DATA), // input [7 : 0] dina
  .douta(snescmd_dout), // output [7 : 0] douta
  .clkb(CLK2), // input clkb
  .web(snescmd_we_mcu), // input [0 : 0] web
  .addrb(snescmd_addr_mcu), // input [8 : 0] addrb
  .dinb(snescmd_data_out_mcu), // input [7 : 0] dinb
  .doutb(snescmd_data_in_mcu) // output [7 : 0] doutb
);
`endif
`ifdef MK3
`ifdef MK2
pll snes_pll(
`else
xc_mk3_pll snes_pll(
`endif
  .inclk0(CLKIN),
  .c0(CLK2),
  .c1(CLK_SOC),       // soft CPU clock, 8 MHz x 161/32 = 40.25 MHz (xc_top SOC_CLK_NUM/DEN)
  .locked(DCM_LOCKED),
  .areset(DCM_RST)
);

wire ROM_ADDR22;
assign ROM_ADDR22 = (SD_DMA_TO_ROM) ? MCU_ADDR[1]    : GSU_ROM_HIT ? GSU_ROM_ADDRr[1]    : MCU_HIT ? ROM_ADDRr[1]    : MAPPED_SNES_ADDR[1];
assign ROM_ADDR   = (SD_DMA_TO_ROM) ? MCU_ADDR[23:2] : GSU_ROM_HIT ? GSU_ROM_ADDRr[23:2] : MCU_HIT ? ROM_ADDRr[23:2] : MAPPED_SNES_ADDR[23:2];
assign ROM_ADDR0  = (SD_DMA_TO_ROM) ? MCU_ADDR[0]    : GSU_ROM_HIT ? GSU_ROM_ADDRr[0]    : MCU_HIT ? ROM_ADDRr[0]    : MAPPED_SNES_ADDR[0];

assign ROM_ZZ = 1'b1;
assign ROM_1CE = ROM_ADDR22;
assign ROM_2CE = ~ROM_ADDR22;

snescmd_buf snescmd (
  .clock(CLK2), // input clka
  .wren_a(SNES_WR_end & ((snescmd_unlock | feat_cmd_unlock) & snescmd_enable)), // input [0 : 0] wea
  .address_a(SNES_ADDR[8:0]), // input [8 : 0] addra
  .data_a(SNES_DATA), // input [7 : 0] dina
  .q_a(snescmd_dout), // output [7 : 0] douta
  .wren_b(snescmd_we_mcu), // input [0 : 0] web
  .address_b(snescmd_addr_mcu), // input [8 : 0] addrb
  .data_b(snescmd_data_out_mcu), // input [7 : 0] dinb
  .q_b(snescmd_data_in_mcu) // output [7 : 0] doutb
);
`endif

// OE always active. Overridden by WE when needed.
assign ROM_OE = 1'b0;

reg[17:0] SNES_DEAD_CNTr;
initial SNES_DEAD_CNTr = 0;

reg ROM_ADDR0_r;
always @(posedge CLK2) ROM_ADDR0_r <= ROM_ADDR0;

// MCU r/w request
always @(posedge CLK2) begin
  if(MCU_RRQ && MCU_ADDR[23:19] != 5'b11100) begin
    MCU_RD_PENDr <= 1'b1;
    RQ_MCU_RDYr <= 1'b0;
    ROM_ADDRr <= MCU_ADDR;
  end else if(MCU_WRQ && MCU_ADDR[23:19] != 5'b11100) begin
    MCU_WR_PENDr <= 1'b1;
    RQ_MCU_RDYr <= 1'b0;
    ROM_ADDRr <= MCU_ADDR;
  end else if(STATE & (ST_MCU_RD_END | ST_MCU_WR_END)) begin
    MCU_RD_PENDr <= 1'b0;
    MCU_WR_PENDr <= 1'b0;
    RQ_MCU_RDYr <= 1'b1;
  end
end

// XC read request
always @(posedge CLK2) begin
  if(XC_ROM_RRQ) begin
    GSU_ROM_RD_PENDr <= 1'b1;
    RQ_GSU_ROM_RDYr <= 1'b0;
    GSU_ROM_ADDRr <= XC_ROM_ADDR;
  end else if(STATE & ST_GSU_ROM_RD_END) begin
    GSU_ROM_RD_PENDr <= 1'b0;
    RQ_GSU_ROM_RDYr <= 1'b1;
  end
end

always @(posedge CLK2) begin
  if(~SNES_CPU_CLKr[1]) SNES_DEAD_CNTr <= SNES_DEAD_CNTr + 1;
  else SNES_DEAD_CNTr <= 17'h0;
end

always @(posedge CLK2) begin
  SNES_reset_strobe <= 1'b0;
  if(SNES_CPU_CLKr[1]) begin
    SNES_DEADr <= 1'b0;
    if(SNES_DEADr) SNES_reset_strobe <= 1'b1;
  end
  else if(SNES_DEAD_CNTr > SNES_DEAD_TIMEOUT) SNES_DEADr <= 1'b1;
end

always @(posedge CLK2) begin
  if(SNES_DEADr & SNES_CPU_CLKr[1]) STATE <= ST_IDLE; // interrupt+restart an ongoing MCU access when the SNES comes alive
  else
  case(STATE)
    ST_IDLE: begin
      STATE <= ST_IDLE;
      if(free_slot | SNES_DEADr) begin
        if (GSU_ROM_RD_PENDr) begin
          STATE <= ST_GSU_ROM_RD_ADDR;
          ST_MEM_DELAYr <= ROM_CYCLE_LEN;
        end
        else if(MCU_RD_PENDr) begin
          STATE <= ST_MCU_RD_ADDR;
          ST_MEM_DELAYr <= ROM_CYCLE_LEN;
        end
        else if(MCU_WR_PENDr) begin
          STATE <= ST_MCU_WR_ADDR;
          ST_MEM_DELAYr <= ROM_CYCLE_LEN;
        end
      end
    end
    ST_MCU_RD_ADDR: begin
      STATE <= ST_MCU_RD_ADDR;
      ST_MEM_DELAYr <= ST_MEM_DELAYr - 1;
      if(ST_MEM_DELAYr == 0) STATE <= ST_MCU_RD_END;
      MCU_ROM_DINr <= (ROM_ADDR0 ? ROM_DATA[7:0] : ROM_DATA[15:8]);
    end
    ST_MCU_WR_ADDR: begin
      STATE <= ST_MCU_WR_ADDR;
      ST_MEM_DELAYr <= ST_MEM_DELAYr - 1;
      if(ST_MEM_DELAYr == 0) STATE <= ST_MCU_WR_END;
    end
    ST_GSU_ROM_RD_ADDR: begin
      STATE <= ST_GSU_ROM_RD_ADDR;
      ST_MEM_DELAYr <= ST_MEM_DELAYr - 1;
      if(ST_MEM_DELAYr == 0) STATE <= ST_GSU_ROM_RD_END;
      XC_ROM_DINr <= (ROM_ADDR0_r ? ROM_DATA[15:0] : {ROM_DATA[7:0],ROM_DATA[15:8]});
    end
    ST_MCU_RD_END, ST_MCU_WR_END, ST_GSU_ROM_RD_END, ST_GSU_RAM_RD_END, ST_GSU_RAM_WR_END: begin
      STATE <= ST_IDLE;
    end
  endcase
end

/***********************
 * R213F read patching *
 ***********************/
always @(posedge CLK2) begin
  case(r213f_state)
    ST_R213F_HOLD: begin
      r213f_state <= ST_R213F_HOLD;
      if(SNES_PULSE_end) begin
        r213f_forceread <= 1'b1;
        r213f_state <= ST_R213F_ARMED;
      end
    end
    ST_R213F_ARMED: begin
      r213f_state <= ST_R213F_ARMED;
      if(SNES_PARD_start & r213f_enable) begin
        r213f_delay <= 3'b001;
        r213f_state <= ST_R213F_WAITBUS;
      end
    end
    ST_R213F_WAITBUS: begin
      r213f_state <= ST_R213F_WAITBUS;
      r213f_delay <= r213f_delay - 1;
      if(r213f_delay == 3'b000) begin
        r213f_state <= ST_R213F_OVERRIDE;
        r213fr <= {SNES_DATA[7:5], mcu_region, SNES_DATA[3:0]};
      end
    end
    ST_R213F_OVERRIDE: begin
      r213f_state <= ST_R213F_HOLD;
      r213f_forceread <= 1'b0;
    end
  endcase
end

/*********************************
 * R2100 patching (experimental) *
 *********************************/
reg [3:0] r2100_bright = 0;
reg [3:0] r2100_bright_orig = 0;

always @(posedge CLK2) begin
  if(SNES_cycle_end) r2100_forcewrite_pre <= 1'b0;
  else if(SNES_PAWR_start & r2100_hit) begin
    if(r2100_patch & SNES_DATA[7]) begin
    // keep previous brightness during forced blanking so there is no DAC step
      r2100_forcewrite_pre <= 1'b1;
      r2100r <= {SNES_DATA[7], 3'b010, r2100_bright}; // 0xAx
    end else if (r2100_patch && SNES_DATA == 8'h00 && r2100r[7]) begin
    // extend forced blanking when game goes from blanking to brightness 0 (Star Fox top of screen)
      r2100_forcewrite_pre <= 1'b1;
      r2100r <= {1'b1, 3'b111, r2100_bright}; // 0xFx
    end else if (r2100_patch && SNES_DATA[3:0] < 4'h8 && r2100_bright_orig > 4'hd) begin
    // substitute big brightness changes with brightness 0 (so it is visible on 1CHIP)
      r2100_forcewrite_pre <= 1'b1;
      r2100r <= {SNES_DATA[7], 3'b011, 4'h0}; // 0x3x / 0xBx(!)
    end else if (r2100_patch | ~(&r2100_limit)) begin
    // save brightness, limit brightness
      r2100_bright <= r2100_limited;
      r2100_bright_orig <= SNES_DATA[3:0];
      if (~(&r2100_limit) && SNES_DATA[3:0] > r2100_limit) begin
        r2100_forcewrite_pre <= 1'b1;
        r2100r <= {SNES_DATA[7], 3'b100, r2100_limited}; // 0x4x / 0xCx
      end
    end
  end
end

reg MCU_WRITE_1;
always @(posedge CLK2) MCU_WRITE_1<= MCU_WRITE;

// odd addresses xxx1
assign ROM_DATA[7:0] = (ROM_ADDR0)
                       ?(SD_DMA_TO_ROM ? (!MCU_WRITE_1 ? MCU_DOUT : 8'bZ)
                                       : (ROM_HIT & ~IS_SAVERAM & ~SNES_WRITE ) ? SNES_DATA
                                       : MCU_WR_HIT ? MCU_DOUT : 8'bZ
                        )
                       :8'bZ;

// even addresses xxx0
assign ROM_DATA[15:8] = (ROM_ADDR0)
                        ? 8'bZ
                        :(SD_DMA_TO_ROM ? (!MCU_WRITE_1 ? MCU_DOUT : 8'bZ)
                                        : (ROM_HIT & ~IS_SAVERAM & ~SNES_WRITE ) ? SNES_DATA
                                        : MCU_WR_HIT ? MCU_DOUT
                                        : 8'bZ
                         );

assign ROM_WE = SD_DMA_TO_ROM
                ?MCU_WRITE
                : (ROM_HIT & IS_WRITABLE & ~IS_SAVERAM & SNES_CPU_CLK ) ? SNES_WRITE
                : MCU_WE_HIT ? 1'b0
                : 1'b1;

assign ROM_BHE =  ROM_ADDR0 && !(!SD_DMA_TO_ROM && GSU_ROM_HIT && GSU_ROM_WORDr);
assign ROM_BLE = !ROM_ADDR0 && !(!SD_DMA_TO_ROM && GSU_ROM_HIT && GSU_ROM_WORDr);

//--------------
// RAM Pipeline
//--------------
parameter ST_RAM_IDLE            = 9'b000000001;
parameter ST_RAM_MCU_RD_ADDR     = 9'b000000010;
parameter ST_RAM_MCU_RD_END      = 9'b000000100;
parameter ST_RAM_MCU_WR_ADDR     = 9'b000001000;
parameter ST_RAM_MCU_WR_END      = 9'b000010000;
parameter ST_RAM_GSU_RD_ADDR     = 9'b000100000;
parameter ST_RAM_GSU_RD_END      = 9'b001000000;
parameter ST_RAM_GSU_WR_ADDR     = 9'b010000000;
parameter ST_RAM_GSU_WR_END      = 9'b100000000;

parameter RAM_CYCLE_LEN = 4'd5;

reg [8:0] RAM_STATE; initial RAM_STATE = ST_RAM_IDLE;
reg [3:0] ST_RAM_DELAYr;

wire ram_free_slot = SNES_cycle_end | ram_free_strobe;

// The SNES never accesses the SRAM chip in this core (no save RAM mapping): full bandwidth.
always @(posedge CLK2) begin
  ram_free_strobe <= 1'b1;
end

// MCU state machine
reg MCU_RAM_RD_PENDr = 0;
reg MCU_RAM_WR_PENDr = 0;
reg [18:0] RAM_ADDRr;

reg RQ_RAM_MCU_RDYr;
initial RQ_RAM_MCU_RDYr = 1'b1;

wire MCU_RAM_WE_HIT = |(RAM_STATE & ST_RAM_MCU_WR_ADDR);
wire MCU_RAM_WR_HIT = |(RAM_STATE & (ST_RAM_MCU_WR_ADDR | ST_RAM_MCU_WR_END));
wire MCU_RAM_RD_HIT = |(RAM_STATE & (ST_RAM_MCU_RD_ADDR | ST_RAM_MCU_RD_END));
wire MCU_RAM_HIT = MCU_RAM_WR_HIT | MCU_RAM_RD_HIT;

// MCU RAM1 r/w request
always @(posedge CLK2) begin
  if(MCU_RRQ && MCU_ADDR[23:19] == 5'b11100) begin
    MCU_RAM_RD_PENDr <= 1'b1;
    RQ_RAM_MCU_RDYr <= 1'b0;
    RAM_ADDRr <= MCU_ADDR;
  end else if(MCU_WRQ && MCU_ADDR[23:19] == 5'b11100) begin
    MCU_RAM_WR_PENDr <= 1'b1;
    RQ_RAM_MCU_RDYr <= 1'b0;
    RAM_ADDRr <= MCU_ADDR;
  end else if(RAM_STATE & (ST_RAM_MCU_RD_END | ST_RAM_MCU_WR_END)) begin
    MCU_RAM_RD_PENDr <= 1'b0;
    MCU_RAM_WR_PENDr <= 1'b0;
    RQ_RAM_MCU_RDYr <= 1'b1;
  end
end

// GSU RAM
reg GSU_RAM_RD_PENDr; initial GSU_RAM_RD_PENDr = 0;
reg GSU_RAM_WR_PENDr; initial GSU_RAM_WR_PENDr = 0;
reg [18:0] GSU_RAM_ADDRr;
reg [7:0]  GSU_RAM_DATAr;

reg RQ_GSU_RAM_RDYr; initial RQ_GSU_RAM_RDYr = 1;
assign XC_RAM_RDY = RQ_GSU_RAM_RDYr;
// Bursts (Xeno Crisis soft CPU line fills / write-backs): XC_RAM_LEN bytes from XC_RAM_ADDR up, one request.
// Each byte is a normal access (ADDR for RAM_CYCLE_LEN + 1 cycles, END, IDLE), about 8 CLK2 cycles instead of
// about 15 for a separate request per byte. XC_RAM_BSTB (one cycle):
//   read:  a byte is in XC_RAM_DINr (bytes in order; with the last one XC_RAM_RDY rises too)
//   write: the byte on XC_RAM_DOUT was latched; present the next one before the next strobe position
//          (the first byte is taken with XC_RAM_WRQ; XC_RAM_RDY rises after the last byte is written)
reg [5:0] GSU_RAM_LEFTr;
reg GSU_RAM_BSTBr = 1'b0;
assign XC_RAM_BSTB = GSU_RAM_BSTBr;

wire GSU_RAM_WE_HIT = |(RAM_STATE & ST_RAM_GSU_WR_ADDR);
wire GSU_RAM_WR_HIT = |(RAM_STATE & (ST_RAM_GSU_WR_ADDR | ST_RAM_GSU_WR_END));
wire GSU_RAM_RD_HIT = |(RAM_STATE & (ST_RAM_GSU_RD_ADDR | ST_RAM_GSU_RD_END));
wire GSU_RAM_HIT    = GSU_RAM_WR_HIT | GSU_RAM_RD_HIT;

// GSU RAM1 r/w request
always @(posedge CLK2) begin
  GSU_RAM_BSTBr <= 1'b0;
  if(XC_RAM_RRQ) begin
    GSU_RAM_RD_PENDr <= 1'b1;
    RQ_GSU_RAM_RDYr <= 1'b0;
    GSU_RAM_ADDRr <= XC_RAM_ADDR;
    GSU_RAM_LEFTr <= (XC_RAM_LEN == 6'd0) ? 6'd0 : XC_RAM_LEN - 6'd1;
  end else if(XC_RAM_WRQ) begin
    GSU_RAM_WR_PENDr <= 1'b1;
    RQ_GSU_RAM_RDYr <= 1'b0;
    GSU_RAM_ADDRr <= XC_RAM_ADDR;
    GSU_RAM_DATAr <= XC_RAM_DOUT;
    GSU_RAM_LEFTr <= (XC_RAM_LEN == 6'd0) ? 6'd0 : XC_RAM_LEN - 6'd1;
    GSU_RAM_BSTBr <= 1'b1;                                   // write byte latched: present the next one
  end else if(RAM_STATE & (ST_RAM_GSU_RD_END | ST_RAM_GSU_WR_END)) begin
    if(RAM_STATE & ST_RAM_GSU_RD_END) GSU_RAM_BSTBr <= 1'b1;   // read byte in XC_RAM_DINr
    if(GSU_RAM_LEFTr != 6'd0) begin
      // burst: next byte (the request stays pending, the state machine goes through IDLE back to ADDR)
      GSU_RAM_ADDRr <= GSU_RAM_ADDRr + 19'd1;
      GSU_RAM_LEFTr <= GSU_RAM_LEFTr - 6'd1;
      if(RAM_STATE & ST_RAM_GSU_WR_END) begin
        GSU_RAM_DATAr <= XC_RAM_DOUT;
        GSU_RAM_BSTBr <= 1'b1;
      end
    end else begin
      GSU_RAM_RD_PENDr <= 1'b0;
      GSU_RAM_WR_PENDr <= 1'b0;
      RQ_GSU_RAM_RDYr <= 1'b1;
    end
  end
end

// RAM state machine
always @(posedge CLK2) begin
  if(SNES_DEADr & SNES_CPU_CLKr[1]) RAM_STATE <= ST_RAM_IDLE; // interrupt+restart an ongoing MCU access when the SNES comes alive
  else
  case(RAM_STATE)
    ST_RAM_IDLE: begin
      if(ram_free_slot | SNES_DEADr) begin
        if (GSU_RAM_RD_PENDr) begin
          RAM_STATE <= ST_RAM_GSU_RD_ADDR;
          ST_RAM_DELAYr <= RAM_CYCLE_LEN;
        end
        else if (GSU_RAM_WR_PENDr) begin
          RAM_STATE <= ST_RAM_GSU_WR_ADDR;
          ST_RAM_DELAYr <= RAM_CYCLE_LEN;
        end
        else if(MCU_RAM_RD_PENDr) begin
          RAM_STATE <= ST_RAM_MCU_RD_ADDR;
          ST_RAM_DELAYr <= RAM_CYCLE_LEN;
        end
        else if(MCU_RAM_WR_PENDr) begin
          RAM_STATE <= ST_RAM_MCU_WR_ADDR;
          ST_RAM_DELAYr <= RAM_CYCLE_LEN;
        end
      end
    end
    ST_RAM_MCU_RD_ADDR: begin
      ST_RAM_DELAYr <= ST_RAM_DELAYr - 1;
      if(ST_RAM_DELAYr == 0) RAM_STATE <= ST_RAM_MCU_RD_END;
      MCU_RAM_DINr <= RAM_DATA;
    end
    ST_RAM_MCU_WR_ADDR: begin
      ST_RAM_DELAYr <= ST_RAM_DELAYr - 1;
      if(ST_RAM_DELAYr == 0) RAM_STATE <= ST_RAM_MCU_WR_END;
    end
    ST_RAM_GSU_RD_ADDR: begin
      ST_RAM_DELAYr <= ST_RAM_DELAYr - 1;
      if(ST_RAM_DELAYr == 0) RAM_STATE <= ST_RAM_GSU_RD_END;
      XC_RAM_DINr <= RAM_DATA;
    end
    ST_RAM_GSU_WR_ADDR: begin
      ST_RAM_DELAYr <= ST_RAM_DELAYr - 1;
      if(ST_RAM_DELAYr == 0) RAM_STATE <= ST_RAM_GSU_WR_END;
    end
    ST_RAM_MCU_RD_END, ST_RAM_MCU_WR_END, ST_RAM_GSU_RD_END, ST_RAM_GSU_WR_END: begin
      RAM_STATE <= ST_RAM_IDLE;
    end
  endcase
end

assign RAM_ADDR = GSU_RAM_HIT ? GSU_RAM_ADDRr[18:0] : MCU_RAM_HIT ? RAM_ADDRr[18:0] : MAPPED_SNES_ADDR[18:0];

assign RAM_DATA[7:0] = ( GSU_RAM_WR_HIT ? GSU_RAM_DATAr[7:0]
                       : (ROM_HIT & IS_SAVERAM & ~SNES_WRITE) ? SNES_DATA
                       : MCU_RAM_WR_HIT ? MCU_DOUT
                       : 8'bZ
                       );

assign RAM_WE = ( GSU_RAM_WE_HIT ? 1'b0
                : (ROM_HIT & IS_SAVERAM & SNES_CPU_CLK) ? SNES_WRITE
                : MCU_RAM_WE_HIT ? 1'b0
                : 1'b1
                );

assign RAM_OE = 1'b0;

always @(posedge CLK2) begin
  // flop data based on source
  if (STATE & ST_MCU_RD_END) begin
    MCU_DINr <= MCU_ROM_DINr;
  end
  else if (RAM_STATE & ST_RAM_MCU_RD_END) begin
    MCU_DINr <= MCU_RAM_DINr;
  end
end

assign MCU_RDY = RQ_MCU_RDYr & RQ_RAM_MCU_RDYr;

//--------------

assign SNES_DATABUS_OE = msu_enable ? 1'b0 :
                         xc_enable & ~(SNES_READ & SNES_WRITE) ? 1'b0 :
                         snescmd_enable & ~(SNES_READ & SNES_WRITE) ? ~(snescmd_unlock | feat_cmd_unlock) :
                         (r213f_enable & !SNES_PARD) ? 1'b0 :
                         (r2100_enable & ~SNES_PAWR) ? 1'b0 :
                         snoop_4200_enable ? SNES_WRITE :
                         ( (IS_ROM & SNES_ROMSEL)
                         | (!IS_ROM & !IS_SAVERAM & !IS_WRITABLE)
                         | (SNES_READ & SNES_WRITE)
                         );

/* data bus direction: 0 = SNES -> FPGA; 1 = FPGA -> SNES
 * data bus is always SNES -> FPGA to avoid fighting except when:
 *  a) the SNES wants to read
 *  b) we want to force a value on the bus
 */
assign SNES_DATABUS_DIR = (~SNES_READ | (~SNES_PARD & (r213f_enable)))
                           ? (1'b1 ^ (r213f_forceread & r213f_enable & ~SNES_PARD)
                                   ^ (r2100_enable & ~SNES_PAWR & ~r2100_forcewrite & ~IS_ROM & ~IS_WRITABLE))
                           : ((~SNES_PAWR & r2100_enable) ? r2100_forcewrite
                           : 1'b0);

assign SNES_IRQ = 1'b0;

endmodule
