`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// xc_bridge: the soft CPU's path from its own clock domain (clk_soc) to the sd2snes side (CLK2).
//
// One operation at a time. The SoC side writes the data words (wb_*), sets the operation fields, pulses
// op_start and waits for op_done; the fields must stay stable until then. Results are read back from a
// small RAM (rb_addr -> rb_q, one cycle). The request and the completion cross the clock domains as
// toggles through two-flop synchronizers; the data sits in two dual-clock block RAMs (8 x 32 bits each)
// that are only read while the other side leaves them alone (set_clock_groups in the SDC).
//
// Operations (op_kind):
//   0 PSRAM_RD  read op_len bytes (even, from an even address) from the PSRAM (sd2snes "ROM" bus)
//   1 SRAM_RD   read op_len bytes from the SRAM chip
//   2 SRAM_WR   write op_len bytes (data words, byte 0 = bits 7:0 of word 0) to the SRAM chip
//   3 WIN_RD / 4 WIN_WR   register access to the $3000 window block (op_addr = byte offset)
//   5 DEC_RD / 6 DEC_WR   register access to the decode mailbox (op_addr = byte offset)
//   op_len is 1..32 bytes; data and results are little-endian: byte k is in word k/4, bits 8*(k%4)+7:8*(k%4).
//
// CLK2 side ports are GSU-style (see sd2snes main.v): *_rrq / *_wrq pulse with the address,
// *_rdy drops on the next cycle and returns high when the access is done (read data valid).
// SRAM operations are one burst of op_len bytes (sram_len): sram_bstb marks each read byte on sram_rdata,
// and for writes each byte taken from sram_wdata (the next one must then be presented); see main.v.
//////////////////////////////////////////////////////////////////////////////////
module xc_bridge (
  // SoC side
  input clk_soc,
  input rst_soc,
  input op_start,
  input [2:0] op_kind,
  input [23:0] op_addr,
  input [5:0] op_len,
  input wb_we,                     // data words for writes
  input [2:0] wb_addr,
  input [31:0] wb_data,
  output reg op_done,
  input [2:0] rb_addr,             // result words (registered read)
  output [31:0] rb_q,

  // CLK2 side
  input clk2,
  input rst2,
  output reg rom_rrq,
  output reg [23:0] rom_addr,
  input rom_rdy,
  input [15:0] rom_rdata,          // {byte at addr+1, byte at addr}
  output reg sram_rrq,
  output reg sram_wrq,
  output reg [18:0] sram_addr,
  output reg [7:0] sram_wdata,
  output reg [5:0] sram_len,
  input sram_bstb,
  input sram_rdy,
  input [7:0] sram_rdata,
  output reg win_sel,
  output reg win_we,
  output reg [11:2] reg_addr,
  output reg [31:0] reg_wdata,
  input [31:0] win_rdata,
  input win_ready,
  output reg dec_sel,
  output reg dec_we,
  input [31:0] dec_rdata,
  input dec_ready
);

//------------------------------------------------------------------------------
// Data RAMs
//------------------------------------------------------------------------------
reg [2:0] w_raddr;
wire [31:0] w_q;
reg r_we;
reg [2:0] r_waddr;
reg [31:0] r_wdata;
xc_dpram8 wram (.wclk(clk_soc), .we(wb_we), .waddr(wb_addr), .wdata(wb_data), .rclk(clk2), .raddr(w_raddr), .q(w_q));
xc_dpram8 rram (.wclk(clk2), .we(r_we), .waddr(r_waddr), .wdata(r_wdata), .rclk(clk_soc), .raddr(rb_addr), .q(rb_q));

//------------------------------------------------------------------------------
// SoC side: request toggle, completion synchronizer
//------------------------------------------------------------------------------
reg req_tog = 1'b0;
(* altera_attribute = "-name SYNCHRONIZER_IDENTIFICATION FORCED_IF_ASYNCHRONOUS" *) reg [2:0] ack_sync = 3'b000;
reg ack_tog = 1'b0;                // CLK2 domain

always @(posedge clk_soc) begin
  op_done <= 1'b0;
  ack_sync <= {ack_sync[1:0], ack_tog};
  if(rst_soc) begin
    req_tog <= 1'b0;
    ack_sync <= 3'b000;
  end else begin
    if(op_start) req_tog <= ~req_tog;
    if(ack_sync[2] != ack_sync[1]) op_done <= 1'b1;
  end
end

//------------------------------------------------------------------------------
// CLK2 side: executor
//------------------------------------------------------------------------------
(* altera_attribute = "-name SYNCHRONIZER_IDENTIFICATION FORCED_IF_ASYNCHRONOUS" *) reg [2:0] req_sync = 3'b000;
reg [2:0] kind;
reg [23:0] addr;
reg [5:0] len;
reg [5:0] pos;                     // byte position
reg [2:0] st;
reg [31:0] acc;                    // result word being assembled
localparam E_IDLE = 3'd0, E_RDW = 3'd1, E_ISSUE = 3'd2, E_WAIT1 = 3'd3, E_WAIT = 3'd4, E_REG = 3'd5, E_DONE = 3'd6, E_REGW = 3'd7;

wire [7:0] wbyte = w_q[{pos[1:0], 3'b000} +: 8];

always @(posedge clk2) begin
  req_sync <= {req_sync[1:0], req_tog};
  rom_rrq <= 1'b0;
  sram_rrq <= 1'b0;
  sram_wrq <= 1'b0;
  r_we <= 1'b0;
  if(rst2) begin
    st <= E_IDLE;
    ack_tog <= 1'b0;
    win_sel <= 1'b0;
    dec_sel <= 1'b0;
    req_sync <= 3'b000;
  end else begin
    case(st)
      E_IDLE: begin
        if(req_sync[2] != req_sync[1]) begin
          // the SoC set the fields and the data words before toggling: they are stable now
          kind <= op_kind;
          addr <= op_addr;
          len <= op_len;
          pos <= 6'd0;
          acc <= 32'd0;
          w_raddr <= 3'd0;
          st <= (op_kind >= 3'd3) ? E_REGW : (op_kind == 3'd2) ? E_RDW : E_ISSUE;
        end
      end
      E_RDW: st <= E_ISSUE;        // data word for this byte is read (w_raddr set before)
      E_ISSUE: begin
        case(kind)
          3'd0: begin rom_rrq <= 1'b1; rom_addr <= addr + pos; end
          3'd1: begin sram_rrq <= 1'b1; sram_addr <= addr[18:0]; sram_len <= len; end
          default: begin sram_wrq <= 1'b1; sram_addr <= addr[18:0]; sram_len <= len; sram_wdata <= wbyte; end
        endcase
        st <= E_WAIT1;
      end
      E_WAIT1: st <= E_WAIT;       // *_rdy drops on this cycle
      E_WAIT: begin
        if(kind == 3'd0 ? rom_rdy : sram_rdy) begin
          // store what arrived into the result word (written every time; the last write has all bytes)
          if(kind == 3'd0) begin
            r_we <= 1'b1; r_waddr <= pos[4:2];
            r_wdata <= pos[1] ? {rom_rdata, acc[15:0]} : {acc[31:16], rom_rdata};
            acc <= pos[1] ? {rom_rdata, acc[15:0]} : {acc[31:16], rom_rdata};
            if(pos[1]) acc <= 32'd0;
            pos <= pos + 6'd2;
            st <= (pos + 6'd2 >= len) ? E_DONE : E_ISSUE;
          end else st <= E_DONE;     // SRAM burst complete (the bytes were handled at their strobes below)
        end
        if(kind != 3'd0 && sram_bstb) begin
          if(kind == 3'd1) begin
            r_we <= 1'b1; r_waddr <= pos[4:2];
            case(pos[1:0])
              2'd0: begin r_wdata <= {acc[31:8], sram_rdata}; acc <= {acc[31:8], sram_rdata}; end
              2'd1: begin r_wdata <= {acc[31:16], sram_rdata, acc[7:0]}; acc <= {acc[31:16], sram_rdata, acc[7:0]}; end
              2'd2: begin r_wdata <= {acc[31:24], sram_rdata, acc[15:0]}; acc <= {acc[31:24], sram_rdata, acc[15:0]}; end
              default: begin r_wdata <= {sram_rdata, acc[23:0]}; acc <= 32'd0; end
            endcase
          end
          pos <= pos + 6'd1;
          w_raddr <= (pos + 6'd1) >> 2;          // write: the word of the next byte
        end
        if(kind == 3'd2) sram_wdata <= wbyte;    // write: follows pos (the word read lags one cycle, well
                                                 // before main.v takes the next byte)
      end
      E_REGW: st <= E_REG;         // register write data (word 0) is read
      E_REG: begin
        reg_addr <= addr[11:2];
        reg_wdata <= w_q;
        if(kind == 3'd3 || kind == 3'd4) begin
          win_sel <= 1'b1; win_we <= (kind == 3'd4);
          if(win_sel && win_ready) begin
            win_sel <= 1'b0;
            r_we <= 1'b1; r_waddr <= 3'd0; r_wdata <= win_rdata;
            st <= E_DONE;
          end
        end else begin
          dec_sel <= 1'b1; dec_we <= (kind == 3'd6);
          if(dec_sel && dec_ready) begin
            dec_sel <= 1'b0;
            r_we <= 1'b1; r_waddr <= 3'd0; r_wdata <= dec_rdata;
            st <= E_DONE;
          end
        end
      end
      E_DONE: begin
        ack_tog <= ~ack_tog;       // the result RAM write (if any) happened on the previous edge
        st <= E_IDLE;
      end
      default: st <= E_IDLE;
    endcase
  end
end

endmodule

//------------------------------------------------------------------------------
// 8 x 32 dual-clock simple dual-port RAM (one M9K)
//------------------------------------------------------------------------------
module xc_dpram8 (
  input wclk,
  input we,
  input [2:0] waddr,
  input [31:0] wdata,
  input rclk,
  input [2:0] raddr,
  output reg [31:0] q
);
`ifdef MK2
(* ram_style = "distributed" *) reg [31:0] m [0:7];   // mk2: distributed RAM (block RAMs are all used)
`else
(* ramstyle = "M9K" *) reg [31:0] m [0:7];
`endif
always @(posedge wclk) if(we) m[waddr] <= wdata;
`ifdef MK2
wire [31:0] qa = m[raddr];
always @(posedge rclk) q <= qa;
`else
always @(posedge rclk) q <= m[raddr];
`endif
endmodule
