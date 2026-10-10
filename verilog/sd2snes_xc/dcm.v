`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date:    13:06:52 06/28/2009
// Design Name:
// Module Name:    dcm
// Project Name:
// Target Devices:
// Tool versions:
// Description:
//
// Dependencies:
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////
module my_dcm (
  input CLKIN,
  output CLKFX,
  output LOCKED,
  input RST,
  output[7:0] STATUS
);

// DCM: Digital Clock Manager Circuit
//      Spartan-3
// Xilinx HDL Language Template, version 11.1

  DCM #(
    .SIM_MODE("SAFE"),  // Simulation: "SAFE" vs. "FAST", see "Synthesis and Simulation Design Guide" for details
    .CLKDV_DIVIDE(2.0), // Divide by: 1.5,2.0,2.5,3.0,3.5,4.0,4.5,5.0,5.5,6.0,6.5
                        //   7.0,7.5,8.0,9.0,10.0,11.0,12.0,13.0,14.0,15.0 or 16.0
    //.CLKFX_DIVIDE(1),   // Can be any integer from 1 to 32 // 24 MHz
    //.CLKFX_MULTIPLY(4), // Can be any integer from 2 to 32
    //.CLKFX_DIVIDE(5),   // Can be any integer from 1 to 32 // 21.6 MHz
    //.CLKFX_MULTIPLY(18), // Can be any integer from 2 to 32
    .CLKFX_DIVIDE(7),   // Can be any integer from 1 to 32 // 21.43 MHz
    .CLKFX_MULTIPLY(25), // Can be any integer from 2 to 32
    .CLKIN_DIVIDE_BY_2("FALSE"), // TRUE/FALSE to enable CLKIN divide by two feature
    .CLKIN_PERIOD(41.667),  // Specify period of input clock
    .CLKOUT_PHASE_SHIFT("NONE"), // Specify phase shift of NONE, FIXED or VARIABLE
    .CLK_FEEDBACK("NONE"),  // Specify clock feedback of NONE, 1X or 2X
    .DESKEW_ADJUST("SYSTEM_SYNCHRONOUS"), // SOURCE_SYNCHRONOUS, SYSTEM_SYNCHRONOUS or
                                          //   an integer from 0 to 15
    .DFS_FREQUENCY_MODE("LOW"),  // HIGH or LOW frequency mode for frequency synthesis
    .DLL_FREQUENCY_MODE("LOW"),  // HIGH or LOW frequency mode for DLL
    .DUTY_CYCLE_CORRECTION("TRUE"), // Duty cycle correction, TRUE or FALSE
    .FACTORY_JF(16'hFFFF),   // FACTORY JF values
//      .LOC("DCM_X0Y0"),
    .PHASE_SHIFT(0),     // Amount of fixed phase shift from -255 to 255
    .STARTUP_WAIT("TRUE")   // Delay configuration DONE until DCM LOCK, TRUE/FALSE
  ) DCM_inst (
    .CLK0(CLK0),     // 0 degree DCM CLK output
    .CLK180(CLK180), // 180 degree DCM CLK output
    .CLK270(CLK270), // 270 degree DCM CLK output
    .CLK2X(CLK2X),   // 2X DCM CLK output
    .CLK2X180(CLK2X180), // 2X, 180 degree DCM CLK out
    .CLK90(CLK90),   // 90 degree DCM CLK output
    .CLKDV(CLKDV),   // Divided DCM CLK out (CLKDV_DIVIDE)
    .CLKFX(CLKFX),   // DCM CLK synthesis out (M/D)
    .CLKFX180(CLKFX180), // 180 degree CLK synthesis out
    .LOCKED(LOCKED), // DCM LOCK status output
    .PSDONE(PSDONE), // Dynamic phase adjust done output
    .STATUS(STATUS), // 8-bit DCM status bits output
    .CLKFB(CLKFB),   // DCM clock feedback
    .CLKIN(CLKIN),   // Clock input (from IBUFG, BUFG or DCM)
    .PSCLK(PSCLK),   // Dynamic phase adjust clock input
    .PSEN(PSEN),     // Dynamic phase adjust enable input
    .PSINCDEC(PSINCDEC), // Dynamic phase adjust increment/decrement
    .RST(RST)        // DCM asynchronous reset input
  );
endmodule

`ifdef MK2
//////////////////////////////////////////////////////////////////////////////////
// Xeno Crisis mk2: second DCM for the soft CPU clock, CLKIN 24 MHz x 11 / 12 = 22 MHz
// (xc_top SOC_CLK_NUM / SOC_CLK_DEN in main.v must match). The Spartan-3 -4 reaches about 22 MHz with this core
// (ISE: 45 ns worst path). If the soft CPU clock misses timing, go back to x 5 / 6 = 20 MHz (SOC_CLK_NUM 20), or
// try x 9 / 10 = 21.6 MHz (SOC_CLK_NUM 108, SOC_CLK_DEN 5).
//////////////////////////////////////////////////////////////////////////////////
module soc_dcm (
  input CLKIN,
  output CLKFX,
  output LOCKED,
  input RST
);
  DCM #(
    .SIM_MODE("SAFE"),
    .CLKFX_DIVIDE(12),
    .CLKFX_MULTIPLY(11),
    .CLKIN_DIVIDE_BY_2("FALSE"),
    .CLKIN_PERIOD(41.667),
    .CLKOUT_PHASE_SHIFT("NONE"),
    .CLK_FEEDBACK("NONE"),
    .DESKEW_ADJUST("SYSTEM_SYNCHRONOUS"),
    .DFS_FREQUENCY_MODE("LOW"),
    .DLL_FREQUENCY_MODE("LOW"),
    .DUTY_CYCLE_CORRECTION("TRUE"),
    .FACTORY_JF(16'hFFFF),
    .PHASE_SHIFT(0),
    .STARTUP_WAIT("TRUE")
  ) DCM_soc (
    .CLKFX(CLKFX),
    .LOCKED(LOCKED),
    .CLKIN(CLKIN),
    .CLKFB(1'b0),
    .PSCLK(1'b0),
    .PSEN(1'b0),
    .PSINCDEC(1'b0),
    .RST(RST)
  );
endmodule

`endif