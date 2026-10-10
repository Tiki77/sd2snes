`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// xc_dac: MSU-1 audio output for the Xeno Crisis core (fpga_xc.bi3 / fpga_xc.bit, MSU-1 music).
//
// dac.v with the 3-stage CIC interpolator (6 x 64-bit adders, ~2,000 LEs) replaced by linear interpolation
// between consecutive 44.1 kHz samples, so that the MSU-1 fits next to the soft CPU. Same ports, the same
// buffer (dac_buf: 2 KB written by the SD DMA, read as 512 stereo samples), the same MCU interface
// (DAC_STATUS = read address bit 8: the half being played), the same 44.1 kHz timing from the SNES
// clock (NTSC/PAL), volume ramp and I2S output (mclk = clk/8, lrck = mclk/64, 16 bits per channel).
//
// Interpolation: the output steps 16 times per input sample (705.6 kHz, as dac.v's integrator strobes); each
// step s (0..15) outputs prev + (cur - prev) * s / 16. Mk.III: both channels are computed in parallel (DSP
// multipliers) and registered, so the I2S side selects a register by lrck exactly as dac.v selects its
// integrator. Mk.II: the channels are computed one at a time (see ch_lo) with the hard multipliers.
//////////////////////////////////////////////////////////////////////////////////
`include "config.vh"

module xc_dac(
  input clkin,
  input sysclk,
  input we,
  input[10:0] pgm_address,
  input[7:0] pgm_data,
  input[7:0] volume,
  input vol_latch,
  input [2:0] vol_select,
  input [8:0] dac_address_ext,
  input play,
  input reset,
  input palmode,
  output sdout,
  output mclk_out,
  output lrck_out,
  output sclk_out,
  output DAC_STATUS
);

reg [8:0] dac_address_r = 9'd0;
reg [8:0] dac_address_r_sync = 9'd0;
wire [31:0] dac_data;
assign DAC_STATUS = dac_address_r[8];

reg [2:0] sysclk_sreg = 3'd0;
wire sysclk_rising = (sysclk_sreg[2:1] == 2'b01);
always @(posedge clkin) sysclk_sreg <= {sysclk_sreg[1:0], sysclk};

`ifdef MK2
dac_buf snes_dac_buf (
  .clka(clkin),
  .wea(~we),
  .addra(pgm_address),
  .dina(pgm_data),
  .clkb(clkin),
  .addrb(dac_address_r_sync),
  .doutb(dac_data));
`else
dac_buf snes_dac_buf (
  .clock(clkin),
  .wren(~we),
  .wraddress(pgm_address),
  .data(pgm_data),
  .rdaddress(dac_address_r_sync),
  .q(dac_data));
`endif

reg [10:0] cnt = 11'h100;
wire mclk = cnt[2]; // mclk = clk/8
wire lrck = cnt[8]; // lrck = mclk/64
wire sclk = cnt[3]; // sclk = lrck*32

reg [2:0] mclk_sreg = 3'd0;
reg [2:0] lrck_sreg = 3'd0;
reg [1:0] sclk_sreg = 2'd0;
assign mclk_out = ~mclk_sreg[2];
assign lrck_out = lrck_sreg[2];
assign sclk_out = sclk_sreg[1];
wire lrck_rising = ({lrck_sreg[0], lrck} == 2'b01);
wire lrck_falling = ({lrck_sreg[0], lrck} == 2'b10);
wire sclk_falling = ({sclk_sreg[0], sclk} == 2'b10);

reg play_r = 1'b0;
always @(posedge clkin) begin
  cnt <= cnt + 1'b1;
  mclk_sreg <= {mclk_sreg[1:0], mclk};
  lrck_sreg <= {lrck_sreg[1:0], lrck};
  sclk_sreg <= {sclk_sreg[0], sclk};
  play_r <= play;
end

/*
  21477272.727272... /  37500 *  1232 = 44100 * 16
  21281370           / 709379 * 23520 = 44100 * 16
*/
reg [19:0] phaseacc = 0;
wire [14:0] phasemul = (palmode ? 15'd23520 : 15'd1232);
wire [19:0] phasediv = (palmode ? 20'd709379 : 20'd37500);
reg [3:0] subcount = 0;

// consecutive input samples {right, left} (dac_buf word: [31:16], [15:0]); step = subcount of the output
reg [31:0] s_prev = 32'd0, s_cur = 32'd0;
reg [3:0] step = 4'd0;

always @(posedge clkin) begin
  if(reset) begin
    dac_address_r <= dac_address_ext;
    phaseacc <= 0;
    subcount <= 0;
    s_prev <= 32'd0;
    s_cur <= 32'd0;
    step <= 4'd0;
  end else if(sysclk_rising) begin
    if(phaseacc >= phasediv) begin
      phaseacc <= phaseacc - phasediv + phasemul;
      subcount <= subcount + 1'b1;
      step <= subcount;
      if(subcount == 0) begin
        dac_address_r <= dac_address_r + play_r;
        // dac_data holds the sample at the address from before this increment (synced at lrck, read since)
        s_prev <= s_cur;
        s_cur <= play_r ? dac_data : s_cur;
      end
    end else begin
      phaseacc <= phaseacc + phasemul;
    end
  end
end

always @(posedge clkin) begin
  if(lrck_rising) dac_address_r_sync <= dac_address_r;
end

`ifdef MK2
// One channel at a time: the I2S side loads the low half at the falling edge of lrck and the high half at the
// rising edge, so while lrck is high this computes the low channel and while it is low the high channel. The
// pipeline (4 stages) has settled long before the edge (256 clocks per half), and the result is the same as two
// parallel channels, with one subtractor, adder, multiplier pair and saturation instead of two (mk2: size).
wire ch_lo = lrck;
wire [15:0] c_cur = ch_lo ? s_cur[15:0] : s_cur[31:16];
`ifdef XC_DAC_NOLINEAR
// mk2 (size): no interpolation, each input sample is held until the next one (define XC_DAC_NOLINEAR)
reg signed [15:0] i_s;
always @(posedge clkin) begin
  i_s <= c_cur;
end
`else
// linear interpolation: (prev * (16 - step) + cur * step) / 16, which is exactly prev + (cur - prev) * step / 16
// (prev * 16 is a multiple of 16). Both products are registered, so XST puts the registers into the hard
// multipliers (MULT18X18S) and the datapath needs no subtractor and no slice flip-flops for them.
wire [15:0] c_prev = ch_lo ? s_prev[15:0] : s_prev[31:16];
reg signed [21:0] pa, pb;
reg signed [15:0] i_s;
wire signed [22:0] psum = pa + pb;
always @(posedge clkin) begin
  pa <= $signed(c_prev) * $signed({1'b0, 5'd16 - {1'b0, step}});
  pb <= $signed(c_cur) * $signed({1'b0, 1'b0, step});
  i_s <= psum[19:4];   // the weighted sum of two 16-bit samples divided by 16 stays within 16 bits
end
`endif
`else
// linear interpolation, both channels, two register stages
reg signed [16:0] d_hi, d_lo;
reg signed [15:0] p_hi, p_lo;
reg [3:0] step_r;
reg signed [15:0] i_hi, i_lo;
wire signed [21:0] m_hi = d_hi * $signed({1'b0, step_r});
wire signed [21:0] m_lo = d_lo * $signed({1'b0, step_r});
always @(posedge clkin) begin
  d_hi <= $signed({s_cur[31], s_cur[31:16]}) - $signed({s_prev[31], s_prev[31:16]});
  d_lo <= $signed({s_cur[15], s_cur[15:0]}) - $signed({s_prev[15], s_prev[15:0]});
  p_hi <= s_prev[31:16];
  p_lo <= s_prev[15:0];
  step_r <= step;
  i_hi <= p_hi + m_hi[19:4];   // |prev + (cur - prev) * s/16| stays within 16 bits
  i_lo <= p_lo + m_lo[19:4];
end
`endif

// volume (as dac.v)
wire [9:0] vol_orig = volume + volume[7];
wire [9:0] vol_3db = volume + volume[7:1] + volume[7];
wire [9:0] vol_6db = {1'b0, volume, volume[7]} + volume[7];
wire [9:0] vol_9db = {1'b0, volume, 1'b0} + volume + volume[7:6];
wire [9:0] vol_12db = {volume, volume[7:6]};
reg [9:0] vol_scaled;
always @* begin
  case(vol_select)
    3'b000: vol_scaled = vol_orig;
    3'b001: vol_scaled = vol_3db;
    3'b010: vol_scaled = vol_6db;
    3'b011: vol_scaled = vol_9db;
    3'b100: vol_scaled = vol_12db;
    default: vol_scaled = vol_orig;
  endcase
end
reg [10:0] vol_target_reg = 11'd0;
reg [10:0] vol_reg = 11'd0;
always @(posedge clkin) vol_target_reg <= vol_scaled;
`ifdef MK2
// ramp volume only on sample boundaries (one comparator and an equality test)
wire vol_up = vol_reg < vol_target_reg;
always @(posedge clkin) begin
  if(lrck_rising && vol_reg != vol_target_reg) vol_reg <= vol_up ? vol_reg + 1'b1 : vol_reg - 1'b1;
end

// volume and saturation (plain wires: XST builds a function called twice in one block as one circuit)
// The product is registered (in the hard multiplier, MULT18X18S): multiplier plus saturation in one CLK2 cycle
// missed timing by 0.76 ns. One more stage of latency is harmless: the result is only loaded at an lrck edge.
reg signed [26:0] vm;
always @(posedge clkin) vm <= i_s * $signed({1'b0, vol_reg});
wire [15:0] vsat = (vm[26:23] == 4'b0000 || vm[26:23] == 4'b1111) ? vm[23:8] : vm[26] ? 16'h8000 : 16'h7fff;
reg [15:0] v;
always @(posedge clkin) v <= vsat;
`else
// ramp volume only on sample boundaries
always @(posedge clkin) begin
  if(lrck_rising) begin
    if(vol_reg > vol_target_reg) vol_reg <= vol_reg - 1'b1;
    else if(vol_reg < vol_target_reg) vol_reg <= vol_reg + 1'b1;
  end
end

reg signed [15:0] v_hi, v_lo;
wire signed [26:0] vm_hi = i_hi * $signed({1'b0, vol_reg});
wire signed [26:0] vm_lo = i_lo * $signed({1'b0, vol_reg});
function [15:0] sat;
  input signed [26:0] v;
  sat = (v[26:23] == 4'b0000 || v[26:23] == 4'b1111) ? v[23:8] : v[26] ? 16'h8000 : 16'h7fff;
endfunction
always @(posedge clkin) begin
  v_hi <= sat(vm_hi);
  v_lo <= sat(vm_lo);
end
`endif

// I2S (as dac.v: lrck high = dac_buf [31:16])
reg [15:0] smpshift = 16'd0;
reg sdout_reg = 1'b0;
assign sdout = sdout_reg;
always @(posedge clkin) begin
  if(sclk_falling) begin
    sdout_reg <= smpshift[15];
`ifdef MK2
    if(lrck_rising | lrck_falling) smpshift <= v;
`else
    if(lrck_rising | lrck_falling) smpshift <= lrck ? v_hi : v_lo;
`endif
    else smpshift <= {smpshift[14:0], 1'b0};
  end
end

endmodule
