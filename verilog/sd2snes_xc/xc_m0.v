`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// xc_m0: small multi-cycle ARMv6-M (Thumb) core for running the Xeno Crisis RP2040 core 0 firmware
// on the sd2snes FPGA: Cyclone IV EP4CE15 (Mk.III) or Spartan-3 XC3S400 (Mk.II). The Mk.II build is cut down for size:
//   - the register file has one write port and no reset, so it maps to distributed (LUT) RAM: SP and LR
//     are written in two start-up cycles (S_RST0/S_RST1), and exception entry writes LR one bus cycle
//     before SP instead of in the same cycle;
//   - instructions the game firmware (and xc_soc.bin) never contain raise `fault` (code 0x01, 32-bit: 0x06):
//     ROR (register), REV16, REVSH, MRS other than IPSR/PRIMASK, MSR other than PRIMASK. With ROR gone
//     the barrel shifter's right shift is 32 bits wide instead of 64.
//   Everything else behaves exactly as on the Mk.III.
//
// Implements the whole ARMv6-M instruction set except SVC/BKPT/UDF (and on the Mk.II the above; they raise `fault`), plus
// exception entry/return for external interrupts (thread mode and handler mode, MSP only, no
// priorities or nesting: the NVIC outside the core decides when to interrupt).
//
// Area-oriented datapath: a register file with two read ports and one write port (plus an SP-only
// write port), one shared ALU (adder, logic, barrel shifter, extend/reverse, multiplier) that also
// computes load/store and branch addresses. Every cycle is described by one combinational
// "micro-op" block; the sequential block only applies what it decided.
//
// Timing (40 MHz on Cyclone IV C8): instructions are decoded only from the fetch buffer register (a fetched word
// is latched and executed the next cycle), the next fetch is issued in the cycle an instruction finishes
// (EARLY_FETCH, off in step mode), MUL takes two cycles (registered product), and nothing that depends on
// bus_ready/bus_rdata feeds the main ALU, whose sum drives bus_next_addr.
//
// Bus: one access at a time, held until bus_ready (which may be combinational for zero wait states).
//   bus_size 0/1/2 = byte/halfword/word. bus_wdata holds the value in its low bits; bus_rdata must
//   return the addressed value in its low bits (on the Mk.II, bits above a byte / halfword load may be anything: the
//   core extends bits 7:0 / 15:0 itself; the bus adapter handles byte lanes). Instruction
//   fetches are word reads with bus_fetch set.
//
// Control:
//   exc_req/exc_num  take exception exc_num (16 + IRQ) at the next boundary (when PRIMASK is clear
//               and the core is in thread mode); the core acks with exc_ack.
//   step_mode=1 makes the core run exactly one instruction (or one exception entry when exc_req is
//               set) per step_go pulse; step_done pulses when it finishes (lockstep testing).
//   dbg_*       read/write registers while stopped (lockstep harness).
//////////////////////////////////////////////////////////////////////////////////
module xc_m0 #(
  parameter DEBUG = 1,           // debug read port and cycle/instruction counters (lockstep harness only)
`ifdef MK2
  parameter EARLY_FETCH = 0      // issue the next fetch in the cycle an instruction finishes (not in step mode)
`else
  parameter EARLY_FETCH = 1      // issue the next fetch in the cycle an instruction finishes (not in step mode)
`endif
) (
  input clk,
  input rst,
  input [31:0] reset_sp,
  input [31:0] reset_pc,

  output reg bus_req,
  output reg bus_we,
  output reg bus_fetch,
  output reg [1:0] bus_size,
  output reg [31:0] bus_addr,
  output reg [31:0] bus_wdata,
  input bus_ready,
  input [31:0] bus_rdata,
  // Next-cycle bus address, for synchronous cache RAMs: address the RAM with
  // (bus_next_req ? bus_next_addr : bus_addr) so a hit can answer with bus_ready in the first cycle.
  output bus_next_req,
  output [31:0] bus_next_addr,

  input [31:0] vtor,
  input exc_req,
  input [5:0] exc_num,
  output reg exc_ack,

  input step_mode,
  input step_go,
  output reg step_done,
  output reg fault,
  output reg [7:0] fault_code,
  output reg sleeping,           // executing WFI/WFE (hint for clock gating; the core treats them as NOP)

  input dbg_we,
  input [4:0] dbg_sel,           // 0-14 R0-R14, 15 PC, 16 NZCV in [31:28], 17 PRIMASK, 18 IPSR, 19 invalidate fetch buffer
  input [31:0] dbg_wdata,
  input [4:0] dbg_rsel,          // same numbering; 20/21 = cycle count low/high, 22/23 = instruction count
  output reg [31:0] dbg_rdata
);

//------------------------------------------------------------------------------
// Architectural state
//------------------------------------------------------------------------------
reg [31:0] rf [0:14];
reg [31:0] pc;
reg flag_n, flag_z, flag_c, flag_v;
reg primask;
reg [5:0] ipsr;

reg [63:0] stat_cycles;
reg [63:0] stat_instr;

localparam S_IDLE   = 4'd0,
           S_FETCH  = 4'd1,
           S_FETCH2 = 4'd2,
           S_MEM    = 4'd3,
           S_STRREG = 4'd4,
           S_MULTI  = 4'd5,
           S_POPPC  = 4'd6,
           S_EXC    = 4'd7,
           S_VEC    = 4'd8,
           S_ERET   = 4'd9,
           S_HALT   = 4'd10,
           S_X32    = 4'd11,     // execute a 32-bit instruction whose second halfword was just fetched
`ifdef MK2
           S_MULW   = 4'd12,     // MUL second cycle: write the registered product
           S_RST0   = 4'd13,     // after reset: SP <= reset_sp
           S_RST1   = 4'd14;     // after reset: LR <= 0xFFFFFFFF
`else
           S_MULW   = 4'd12;     // MUL second cycle: write the registered product
`endif

reg [3:0] state;

// fetch buffer (last fetched word). Instructions are decoded only from this register: the word arriving
// in S_FETCH/S_FETCH2 is latched and executed the next cycle, which keeps the cache hit path
// (tag RAM -> compare -> data) out of the decode/execute path. The early fetch (EARLY_FETCH) hides the
// extra cycle for sequential code and branches.
reg [31:0] ibuf;
reg [29:0] ibuf_addr;
reg ibuf_valid;
reg [15:0] ir1;                 // first halfword of a 32-bit instruction (S_FETCH2 / S_X32)

wire [31:0] pc2 = pc + 32'd2;
// pc + 4 (what the core reads as PC) is kept in a register, so the ALU operand path starts at a flip-flop
// instead of an adder (it was on the critical path: pc -> +4 -> opA -> ALU -> register file). It follows
// pc on sequential steps; after a jump it is recomputed in the next cycle, and no instruction starts
// before that (exec16_now waits for pc4_ok; the fetch after a jump takes longer anyway).
reg [31:0] pc4r;
reg pc4_ok;
wire [31:0] pc4 = pc4r;
wire fetch_now = bus_ready && (state == S_FETCH || state == S_FETCH2);
wire [15:0] op = pc[1] ? ibuf[31:16] : ibuf[15:0];
wire ibuf_hit = ibuf_valid && (ibuf_addr == pc[31:2]);
wire ibuf_hit2 = ibuf_hit && !pc[1];                 // both halfwords of a 32-bit instruction in the buffer
wire [15:0] op2 = pc[1] ? ibuf[15:0] : ibuf[31:16];  // halfword at pc + 2 (S_X32: first half of the new word)

// multi-register / memory / exception operation registers
reg [31:0] m_addr;
reg [1:0] m_size;
reg m_we, m_signed;
reg [3:0] m_rt;
reg [8:0] ml_list;              // bit 8 = LR (PUSH)
reg [3:0] ml_idx;               // register being transferred (14 = LR)
reg ml_load, ml_pop, ml_wb;
reg [2:0] ml_rn;
reg [31:0] ml_final;
reg [2:0] ex_idx;
reg [31:0] ex_sp;
reg [31:0] ex_xpsr;
reg [5:0] ex_num;

reg step_armed;                 // step mode: a step was requested and has not finished yet
wire go = !step_mode || step_go || step_armed;
wire dbg_now = (state == S_IDLE) && dbg_we;
`ifdef MK2
wire take_exc = 1'b0;
`else
wire take_exc = exc_req && (step_mode || (!primask && ipsr == 6'd0));
`endif
wire [31:0] xpsr = {flag_n, flag_z, flag_c, flag_v, 3'b000, 1'b1, 18'd0, ipsr};

function [3:0] first9(input [8:0] l); // lowest set bit, as a register number (bit 8 = LR = 14)
  first9 = l[0] ? 4'd0 : l[1] ? 4'd1 : l[2] ? 4'd2 : l[3] ? 4'd3 : l[4] ? 4'd4 :
           l[5] ? 4'd5 : l[6] ? 4'd6 : l[7] ? 4'd7 : 4'd14;
endfunction

function [3:0] popc8(input [7:0] v);
  popc8 = {3'd0, v[0]} + {3'd0, v[1]} + {3'd0, v[2]} + {3'd0, v[3]} + {3'd0, v[4]} + {3'd0, v[5]} + {3'd0, v[6]} + {3'd0, v[7]};
endfunction

function cond_pass(input [3:0] c, input n, input z, input cf, input v);
  case(c)
    4'h0: cond_pass = z;
    4'h1: cond_pass = ~z;
    4'h2: cond_pass = cf;
    4'h3: cond_pass = ~cf;
    4'h4: cond_pass = n;
    4'h5: cond_pass = ~n;
    4'h6: cond_pass = v;
    4'h7: cond_pass = ~v;
    4'h8: cond_pass = cf & ~z;
    4'h9: cond_pass = ~cf | z;
    4'hA: cond_pass = (n == v);
    4'hB: cond_pass = (n != v);
    4'hC: cond_pass = ~z & (n == v);
    4'hD: cond_pass = z | (n != v);
    default: cond_pass = 1'b1;
  endcase
endfunction

function [31:0] align(input [31:0] a, input [1:0] size);
  align = (size == 2'd2) ? {a[31:2], 2'b00} : (size == 2'd1) ? {a[31:1], 1'b0} : a;
endfunction

//------------------------------------------------------------------------------
// Register read ports (15 reads as PC + 4)
//------------------------------------------------------------------------------
reg [3:0] ra, rb;
wire [31:0] rdA = (ra == 4'd15) ? pc4 : rf[ra];
wire [31:0] rdB = (rb == 4'd15) ? pc4 : rf[rb];

//------------------------------------------------------------------------------
// ALU
//------------------------------------------------------------------------------
localparam A_REG = 2'd0, A_PC4 = 2'd1, A_PC4AL = 2'd2, A_EXSP = 2'd3;
localparam OP_ADD = 3'd0, OP_AND = 3'd1, OP_EOR = 3'd2, OP_ORR = 3'd3, OP_BIC = 3'd4, OP_MOVB = 3'd5, OP_SHIFT = 3'd6, OP_MISC = 3'd7;
// OP_MISC sub-ops: 0 MVN, 1 MUL, 2 SXTH, 3 SXTB, 4 UXTH, 5 UXTB, 6 REV, 7 REV16, 8 REVSH (7 and 8 not on the Mk.II)

reg [1:0] asel;
reg azero;                 // operand A = 0 (RSB #0)
reg bimm_en;
reg [31:0] bimm;
reg binv;
reg [1:0] cin_sel;         // 0: 0, 1: 1, 2: C
reg [2:0] aop;
reg [3:0] misc_op;
reg [1:0] sh_kind;         // LSL, LSR, ASR, ROR
reg sh_val_b;              // shift the B register (immediate forms) instead of A
reg sh_amt_imm;
reg [7:0] sh_imm;

wire [31:0] opA = azero ? 32'd0 : (asel == A_PC4) ? pc4 : (asel == A_PC4AL) ? {pc4[31:2], 2'b00} : (asel == A_EXSP) ? ex_sp : rdA;
wire [31:0] opB0 = bimm_en ? bimm : rdB;
wire [31:0] opB = binv ? ~opB0 : opB0;
wire cin = (cin_sel == 2'd1) | ((cin_sel == 2'd2) & flag_c);
wire [32:0] sum = {1'b0, opA} + {1'b0, opB} + {32'd0, cin};
wire add_v = (~(opA[31] ^ opB[31])) & (opA[31] ^ sum[31]);

// barrel shifter
wire [31:0] sh_in = sh_val_b ? rdB : rdA;
wire [7:0] sh_n = sh_amt_imm ? sh_imm : rdB[7:0];
wire [4:0] sh_n5 = sh_n[4:0];
wire sh_big = |sh_n[7:5];            // amount >= 32
`ifdef MK2
// no ROR (it faults): a 32-bit logical or arithmetic right shift
wire signed [31:0] sh_asr = $signed(sh_in) >>> sh_n5;
wire [31:0] sh_lsr = sh_in >> sh_n5;
wire [31:0] sh_right = (sh_kind == 2'd2) ? sh_asr : sh_lsr;
`else
wire [63:0] sh_right_in = {(sh_kind == 2'd2) ? {32{sh_in[31]}} : (sh_kind == 2'd3) ? sh_in : 32'd0, sh_in};
wire [63:0] sh_right = sh_right_in >> sh_n5;
`endif
wire [32:0] sh_left = {1'b0, sh_in} << sh_n5;
wire sh_last_out = sh_in[sh_n5 - 5'd1];
reg [31:0] sh_res;
reg sh_c;
always @* begin
  sh_res = sh_in;
  sh_c = flag_c;
  if(sh_n != 8'd0) begin
    case(sh_kind)
      2'd0: begin // LSL
        if(!sh_big) begin sh_res = sh_left[31:0]; sh_c = sh_left[32]; end
        else begin sh_res = 32'd0; sh_c = (sh_n == 8'd32) ? sh_in[0] : 1'b0; end
      end
`ifndef MK2
      2'd1: begin // LSR
        if(!sh_big) begin sh_res = sh_right[31:0]; sh_c = sh_last_out; end
        else begin sh_res = 32'd0; sh_c = (sh_n == 8'd32) ? sh_in[31] : 1'b0; end
      end
`endif
      2'd2: begin // ASR
`ifdef MK2
        if(!sh_big) begin sh_res = sh_right; sh_c = sh_last_out; end
`else
        if(!sh_big) begin sh_res = sh_right[31:0]; sh_c = sh_last_out; end
`endif
        else begin sh_res = {32{sh_in[31]}}; sh_c = sh_in[31]; end
      end
`ifdef MK2
      default: begin // LSR (ROR is not decoded)
        if(!sh_big) begin sh_res = sh_right; sh_c = sh_last_out; end
        else begin sh_res = 32'd0; sh_c = (sh_n == 8'd32) ? sh_in[31] : 1'b0; end
`else
      default: begin // ROR
        sh_res = sh_right[31:0];
        sh_c = sh_right[31];
`endif
      end
    endcase
  end
end

// MUL takes two cycles: the product of the first cycle's operands is registered (DSP output register)
reg [31:0] mul_q;
always @(posedge clk) mul_q <= rdA * rdB;

reg [31:0] alu_res;
always @* begin
  case(aop)
    OP_ADD: alu_res = sum[31:0];
    OP_AND: alu_res = opA & opB;
    OP_EOR: alu_res = opA ^ opB;
    OP_ORR: alu_res = opA | opB;
    OP_BIC: alu_res = opA & ~opB;
    OP_MOVB: alu_res = opB;
    OP_SHIFT: alu_res = sh_res;
    default: begin
      case(misc_op)
        4'd0: alu_res = ~rdB;
        4'd1: alu_res = mul_q;
        4'd2: alu_res = {{16{rdB[15]}}, rdB[15:0]};
        4'd3: alu_res = {{24{rdB[7]}}, rdB[7:0]};
        4'd4: alu_res = {16'd0, rdB[15:0]};
        4'd5: alu_res = {24'd0, rdB[7:0]};
`ifdef MK2
        default: alu_res = {rdB[7:0], rdB[15:8], rdB[23:16], rdB[31:24]};
`else
        4'd6: alu_res = {rdB[7:0], rdB[15:8], rdB[23:16], rdB[31:24]};
        4'd7: alu_res = {rdB[23:16], rdB[31:24], rdB[7:0], rdB[15:8]};
        default: alu_res = {{16{rdB[7]}}, rdB[7:0], rdB[15:8]};
`endif
      endcase
    end
  endcase
end

// load data extension
wire [1:0] ld_size = (state == S_MEM) ? m_size : 2'd2;
wire [31:0] ld_data = (ld_size == 2'd0) ? (m_signed ? {{24{bus_rdata[7]}}, bus_rdata[7:0]} : {24'd0, bus_rdata[7:0]}) :
                      (ld_size == 2'd1) ? (m_signed ? {{16{bus_rdata[15]}}, bus_rdata[15:0]} : {16'd0, bus_rdata[15:0]}) : bus_rdata;

//------------------------------------------------------------------------------
// Micro-op decode: everything the current cycle does
//------------------------------------------------------------------------------
localparam W_ALU = 3'd0, W_LOAD = 3'd1, W_DBG = 3'd2, W_PC2L = 3'd3, W_PC4L = 3'd4, W_SYS = 3'd5, W_EXCLR = 3'd6, W_FINAL = 3'd7;
localparam W_BUS = W_LOAD; // word loads outside S_MEM (LDM/POP/exception return) are not extended
localparam PC_2 = 3'd0, PC_4 = 3'd1, PC_ALU = 3'd2, PC_RDB = 3'd3, PC_BUS = 3'd4;
localparam WD_RDB = 2'd0, WD_PC = 2'd1, WD_XPSR = 2'd2;
localparam FL_NONE = 2'd0, FL_ALU = 2'd1, FL_RDB = 2'd2, FL_BUS = 2'd3;

reg w_en; reg [3:0] w_idx; reg [2:0] w_src;
reg sp_en; reg [31:0] sp_val;
reg [1:0] fl_src; reg fl_nz, fl_c, fl_v, fl_c_sh;
reg pc_en; reg [2:0] pc_src;
reg done;                       // instruction finished (counts an instruction)
reg iss; reg iss_we; reg iss_fetch; reg [1:0] iss_size; reg [31:0] iss_addr; reg [1:0] iss_wd;
reg bus_end;                    // drop bus_req (access finished, nothing new issued)
reg [3:0] nstate;
reg set_fault; reg [7:0] fault_val;
reg [7:0] sysm;                 // MRS source
reg primask_en; reg primask_val;
reg ibuf_inval;
reg exec16_now, exec32_now;
reg [15:0] h1;
reg [8:0] ml_rest;
reg mem_ld;                     // single load/store: latch m_*
reg m_we_d, m_sgn_d; reg [1:0] m_size_d; reg [3:0] m_rt_d;
reg multi_start; reg [8:0] ml_list_d; reg ml_load_d, ml_pop_d, ml_wb_d;
reg eret_start;
reg exc_start;
reg ipsr_clear, ipsr_set;
reg is_sleep;
reg mul_start;
reg early;                      // early fetch issued this cycle
reg iss_sum;                    // the issued address is the ALU sum (selected last: keeps the adder near the bus address)
reg [2:0] iss_sum_clr;          // low address bits to clear in the sum (alignment)

// 16-bit instruction fields
wire [2:0] f_rd = op[2:0];
wire [2:0] f_rn = op[5:3];
wire [2:0] f_rm = op[8:6];
wire [2:0] f_r8 = op[10:8];
wire [3:0] f_hrdn = {op[7], op[2:0]};
wire [3:0] f_hrm = op[6:3];
wire [3:0] push_n = popc8(op[7:0]) + {3'd0, op[8]};

always @* begin
  // defaults
  ra = 4'd0; rb = 4'd0;
  asel = A_REG; azero = 1'b0; bimm_en = 1'b0; bimm = 32'd0; binv = 1'b0; cin_sel = 2'd0;
  aop = OP_ADD; misc_op = 4'd0; sh_kind = 2'd0; sh_val_b = 1'b0; sh_amt_imm = 1'b0; sh_imm = 8'd0;
  w_en = 1'b0; w_idx = 4'd0; w_src = W_ALU;
  sp_en = 1'b0; sp_val = 32'd0;
  fl_src = FL_NONE; fl_nz = 1'b0; fl_c = 1'b0; fl_v = 1'b0; fl_c_sh = 1'b0;
  pc_en = 1'b0; pc_src = PC_2;
  done = 1'b0;
  iss = 1'b0; iss_we = 1'b0; iss_fetch = 1'b0; iss_size = 2'd2; iss_addr = 32'd0; iss_wd = WD_RDB;
  bus_end = 1'b0;
  nstate = state;
  set_fault = 1'b0; fault_val = 8'd0;
  sysm = 8'd0;
  primask_en = 1'b0; primask_val = 1'b0;
  ibuf_inval = 1'b0;
  h1 = (state == S_X32) ? ir1 : op;
  ml_rest = ml_list & ~(9'd1 << ((ml_idx == 4'd14) ? 4'd8 : ml_idx));
  mem_ld = 1'b0; m_we_d = 1'b0; m_sgn_d = 1'b0; m_size_d = 2'd2; m_rt_d = 4'd0;
  multi_start = 1'b0; ml_list_d = 9'd0; ml_load_d = 1'b0; ml_pop_d = 1'b0; ml_wb_d = 1'b0;
  eret_start = 1'b0;
  exc_start = 1'b0;
  ipsr_clear = 1'b0; ipsr_set = 1'b0;
  is_sleep = 1'b0;
  mul_start = 1'b0;
  early = 1'b0;
  iss_sum = 1'b0; iss_sum_clr = 3'b000;

  exec16_now = (state == S_IDLE) && go && !dbg_we && !take_exc && ibuf_hit && pc4_ok;
  exec32_now = (exec16_now && op[15:13] == 3'b111 && op[12:11] != 2'b00 && ibuf_hit2) || (state == S_X32); // op[15:11] >= 0x1D

  if(exec32_now) begin
    //---------------------------------------------------------------- 32-bit instructions
    pc_en = 1'b1; pc_src = PC_4; done = 1'b1;
    if(h1[15:11] == 5'b11110 && op2[15:14] == 2'b11 && op2[12]) begin // BL
      asel = A_PC4; bimm_en = 1'b1;
      bimm = {{8{h1[10]}}, ~(op2[13] ^ h1[10]), ~(op2[11] ^ h1[10]), h1[9:0], op2[10:0], 1'b0};
      pc_src = PC_ALU;
      w_en = 1'b1; w_idx = 4'd14; w_src = W_PC4L;
`ifdef MK2
    end else if(h1[15:4] == 12'hF3E && op2[15:12] == 4'h8 && (op2[7:0] == 8'd5 || op2[7:0] == 8'd16)) begin // MRS IPSR / PRIMASK
`else
    end else if(h1[15:4] == 12'hF3E && op2[15:12] == 4'h8) begin // MRS
`endif
      sysm = op2[7:0];
      w_en = (op2[11:8] != 4'd15); w_idx = op2[11:8]; w_src = W_SYS;
`ifdef MK2
    end else if(h1[15:4] == 12'hF38 && op2[15:8] == 8'h88 && op2[7:0] == 8'd16) begin // MSR PRIMASK
`else
    end else if(h1[15:4] == 12'hF38 && op2[15:8] == 8'h88) begin // MSR
`endif
      rb = h1[3:0];
`ifdef MK2
      primask_en = 1'b1; primask_val = rdB[0];
`else
      case(op2[7:0])
        8'd0, 8'd1, 8'd2, 8'd3: fl_src = FL_RDB;
        8'd8, 8'd9: begin sp_en = 1'b1; sp_val = {rdB[31:2], 2'b00}; end
        8'd16: begin primask_en = 1'b1; primask_val = rdB[0]; end
        default: ;
      endcase
`endif
    end else if(h1 == 16'hF3BF && op2[15:8] == 8'h8F) begin // DSB / DMB / ISB
      ibuf_inval = (op2[7:4] == 4'h6);
    end else begin
      pc_en = 1'b0; done = 1'b0; set_fault = 1'b1; fault_val = 8'h06;
    end
  end else if(exec16_now) begin
    //---------------------------------------------------------------- 16-bit instructions
    pc_en = 1'b1; pc_src = PC_2; done = 1'b1;
    case(op[15:11])
      5'h00, 5'h01, 5'h02: begin // shift by immediate
        rb = {1'b0, f_rn}; sh_val_b = 1'b1; sh_amt_imm = 1'b1; sh_kind = op[12:11];
        sh_imm = (op[12:11] != 2'd0 && op[10:6] == 5'd0) ? 8'd32 : {3'd0, op[10:6]};
        aop = OP_SHIFT;
        w_en = 1'b1; w_idx = {1'b0, f_rd};
        fl_src = FL_ALU; fl_nz = 1'b1; fl_c = 1'b1; fl_c_sh = 1'b1;
      end
      5'h03: begin // ADD/SUB register / imm3
        ra = {1'b0, f_rn}; rb = {1'b0, f_rm};
        bimm_en = op[10]; bimm = {29'd0, op[8:6]};
        binv = op[9]; cin_sel = op[9] ? 2'd1 : 2'd0;
        w_en = 1'b1; w_idx = {1'b0, f_rd};
        fl_src = FL_ALU; fl_nz = 1'b1; fl_c = 1'b1; fl_v = 1'b1;
      end
      5'h04: begin // MOV imm
        bimm_en = 1'b1; bimm = {24'd0, op[7:0]}; aop = OP_MOVB;
        w_en = 1'b1; w_idx = {1'b0, f_r8};
        fl_src = FL_ALU; fl_nz = 1'b1;
      end
      5'h05, 5'h06, 5'h07: begin // CMP / ADD / SUB imm8
        ra = {1'b0, f_r8}; bimm_en = 1'b1; bimm = {24'd0, op[7:0]};
        binv = (op[12:11] != 2'd2); cin_sel = (op[12:11] != 2'd2) ? 2'd1 : 2'd0;
        w_en = (op[12:11] != 2'd1); w_idx = {1'b0, f_r8};
        fl_src = FL_ALU; fl_nz = 1'b1; fl_c = 1'b1; fl_v = 1'b1;
      end
      5'h08: begin
        if(!op[10]) begin // data processing
          ra = {1'b0, f_rd}; rb = {1'b0, f_rn};
          w_en = 1'b1; w_idx = {1'b0, f_rd};
          fl_src = FL_ALU; fl_nz = 1'b1;
          case(op[9:6])
            4'h0: aop = OP_AND;
            4'h1: aop = OP_EOR;
            4'h2: begin aop = OP_SHIFT; sh_kind = 2'd0; fl_c = 1'b1; fl_c_sh = 1'b1; end
            4'h3: begin aop = OP_SHIFT; sh_kind = 2'd1; fl_c = 1'b1; fl_c_sh = 1'b1; end
            4'h4: begin aop = OP_SHIFT; sh_kind = 2'd2; fl_c = 1'b1; fl_c_sh = 1'b1; end
            4'h5: begin cin_sel = 2'd2; fl_c = 1'b1; fl_v = 1'b1; end                               // ADC
            4'h6: begin binv = 1'b1; cin_sel = 2'd2; fl_c = 1'b1; fl_v = 1'b1; end                  // SBC
`ifdef MK2
            4'h7: begin w_en = 1'b0; fl_src = FL_NONE; pc_en = 1'b0; done = 1'b0; set_fault = 1'b1; fault_val = 8'h01; end // ROR: not in the firmware
`else
            4'h7: begin aop = OP_SHIFT; sh_kind = 2'd3; fl_c = 1'b1; fl_c_sh = 1'b1; end
`endif
            4'h8: begin aop = OP_AND; w_en = 1'b0; end                                           // TST
            4'h9: begin azero = 1'b1; binv = 1'b1; cin_sel = 2'd1; fl_c = 1'b1; fl_v = 1'b1; end    // RSB #0
            4'hA: begin binv = 1'b1; cin_sel = 2'd1; w_en = 1'b0; fl_c = 1'b1; fl_v = 1'b1; end    // CMP
            4'hB: begin w_en = 1'b0; fl_c = 1'b1; fl_v = 1'b1; end                                 // CMN
            4'hC: aop = OP_ORR;
            4'hD: begin w_en = 1'b0; fl_src = FL_NONE; pc_en = 1'b0; done = 1'b0; mul_start = 1'b1; end // MUL (2 cycles)
            4'hE: aop = OP_BIC;
            default: begin aop = OP_MISC; misc_op = 4'd0; end                                    // MVN
          endcase
        end else begin // special data processing / branch exchange
          ra = f_hrdn; rb = f_hrm;
          case(op[9:8])
            2'd0: begin // ADD (high)
              if(f_hrdn == 4'd15) pc_src = PC_ALU;
              else begin w_en = 1'b1; w_idx = f_hrdn; end
            end
            2'd1: begin // CMP (high)
              binv = 1'b1; cin_sel = 2'd1;
              fl_src = FL_ALU; fl_nz = 1'b1; fl_c = 1'b1; fl_v = 1'b1;
            end
            2'd2: begin // MOV (high)
              aop = OP_MOVB;
              if(f_hrdn == 4'd15) pc_src = PC_ALU;
              else begin w_en = 1'b1; w_idx = f_hrdn; end
            end
            default: begin // BX / BLX
              ra = 4'd13;
              if(op[7]) begin w_en = 1'b1; w_idx = 4'd14; w_src = W_PC2L; end
`ifdef MK2
              if(!rdB[0]) begin
`else
              if(ipsr != 6'd0 && rdB[31:28] == 4'hF) begin
                pc_en = 1'b0; done = 1'b0; eret_start = 1'b1;
              end else if(!rdB[0]) begin
`endif
                pc_en = 1'b0; done = 1'b0; w_en = 1'b0; set_fault = 1'b1; fault_val = 8'h02;
              end else pc_src = PC_RDB;
            end
          endcase
        end
      end
      5'h09: begin // LDR literal
        asel = A_PC4AL; bimm_en = 1'b1; bimm = {22'd0, op[7:0], 2'b00};
        mem_ld = 1'b1; m_size_d = 2'd2; m_rt_d = {1'b0, f_r8};
      end
      5'h0A, 5'h0B: begin // load/store register offset
        ra = {1'b0, f_rn}; rb = {1'b0, f_rm};
        mem_ld = 1'b1; m_rt_d = {1'b0, f_rd};
        case(op[11:9])
          3'd0: begin m_we_d = 1'b1; m_size_d = 2'd2; end
          3'd1: begin m_we_d = 1'b1; m_size_d = 2'd1; end
          3'd2: begin m_we_d = 1'b1; m_size_d = 2'd0; end
          3'd3: begin m_size_d = 2'd0; m_sgn_d = 1'b1; end
          3'd4: m_size_d = 2'd2;
          3'd5: m_size_d = 2'd1;
          3'd6: m_size_d = 2'd0;
          default: begin m_size_d = 2'd1; m_sgn_d = 1'b1; end
        endcase
      end
      5'h0C, 5'h0D, 5'h0E, 5'h0F, 5'h10, 5'h11: begin // load/store immediate offset
        ra = {1'b0, f_rn}; rb = {1'b0, f_rd}; bimm_en = 1'b1;
        mem_ld = 1'b1; m_we_d = ~op[11]; m_rt_d = {1'b0, f_rd};
        case(op[15:12])
          4'h6: begin m_size_d = 2'd2; bimm = {25'd0, op[10:6], 2'b00}; end
          4'h7: begin m_size_d = 2'd0; bimm = {27'd0, op[10:6]}; end
          default: begin m_size_d = 2'd1; bimm = {26'd0, op[10:6], 1'b0}; end
        endcase
      end
      5'h12, 5'h13: begin // SP-relative
        ra = 4'd13; rb = {1'b0, f_r8}; bimm_en = 1'b1; bimm = {22'd0, op[7:0], 2'b00};
        mem_ld = 1'b1; m_we_d = ~op[11]; m_size_d = 2'd2; m_rt_d = {1'b0, f_r8};
      end
      5'h14: begin // ADR
        asel = A_PC4AL; bimm_en = 1'b1; bimm = {22'd0, op[7:0], 2'b00};
        w_en = 1'b1; w_idx = {1'b0, f_r8};
      end
      5'h15: begin // ADD Rd, SP, imm
        ra = 4'd13; bimm_en = 1'b1; bimm = {22'd0, op[7:0], 2'b00};
        w_en = 1'b1; w_idx = {1'b0, f_r8};
      end
      5'h16, 5'h17: begin // miscellaneous
        casez(op[11:8])
          4'b0000: begin // ADD/SUB SP, imm7
            ra = 4'd13; bimm_en = 1'b1; bimm = {23'd0, op[6:0], 2'b00};
            binv = op[7]; cin_sel = op[7] ? 2'd1 : 2'd0;
            w_en = 1'b1; w_idx = 4'd13;
          end
          4'b0010: begin // SXTH/SXTB/UXTH/UXTB
            rb = {1'b0, f_rn}; aop = OP_MISC; misc_op = 4'd2 + {2'd0, op[7:6]};
            w_en = 1'b1; w_idx = {1'b0, f_rd};
          end
          4'b010?: begin // PUSH
            ra = 4'd13; bimm_en = 1'b1; bimm = {26'd0, push_n, 2'b00}; binv = 1'b1; cin_sel = 2'd1;
            pc_en = 1'b0; done = 1'b0;
            if(op[8:0] == 9'd0) begin set_fault = 1'b1; fault_val = 8'h01; end
            else begin
              w_en = 1'b1; w_idx = 4'd13;
              rb = first9(op[8:0]);
              iss = 1'b1; iss_we = 1'b1; iss_sum = 1'b1;
              multi_start = 1'b1; ml_list_d = op[8:0];
            end
          end
          4'b0110: begin
            if(op[7:5] == 3'b011 && op[3:0] == 4'b0010) begin primask_en = 1'b1; primask_val = op[4]; end // CPS
            else begin pc_en = 1'b0; done = 1'b0; set_fault = 1'b1; fault_val = 8'h01; end
          end
          4'b1010: begin // REV/REV16/REVSH
            rb = {1'b0, f_rn}; aop = OP_MISC;
`ifdef MK2
            misc_op = 4'd6;
            if(op[7:6] != 2'd0) begin pc_en = 1'b0; done = 1'b0; set_fault = 1'b1; fault_val = 8'h01; end // REV16/REVSH: not in the firmware
`else
            misc_op = (op[7:6] == 2'd0) ? 4'd6 : (op[7:6] == 2'd1) ? 4'd7 : 4'd8;
            if(op[7:6] == 2'd2) begin pc_en = 1'b0; done = 1'b0; set_fault = 1'b1; fault_val = 8'h01; end
`endif
            else begin w_en = 1'b1; w_idx = {1'b0, f_rd}; end
          end
          4'b110?: begin // POP
            ra = 4'd13; bimm_en = 1'b1; bimm = {26'd0, push_n, 2'b00};
            pc_en = 1'b0; done = 1'b0;
            if(op[8:0] == 9'd0) begin set_fault = 1'b1; fault_val = 8'h01; end
            else begin
              w_en = 1'b1; w_idx = 4'd13;
              iss = 1'b1; iss_addr = rdA;
              if(op[7:0] != 8'd0) begin multi_start = 1'b1; ml_list_d = {1'b0, op[7:0]}; ml_load_d = 1'b1; ml_pop_d = op[8]; end
              else nstate = S_POPPC;
            end
          end
          4'b1110: begin pc_en = 1'b0; done = 1'b0; set_fault = 1'b1; fault_val = 8'h03; end // BKPT
          4'b1111: is_sleep = (op[7:4] == 4'd2 || op[7:4] == 4'd3); // hints
          default: begin pc_en = 1'b0; done = 1'b0; set_fault = 1'b1; fault_val = 8'h01; end
        endcase
      end
      5'h18, 5'h19: begin // STMIA / LDMIA
        ra = {1'b0, f_r8}; bimm_en = 1'b1; bimm = {26'd0, popc8(op[7:0]), 2'b00};
        pc_en = 1'b0; done = 1'b0;
        if(op[7:0] == 8'd0) begin set_fault = 1'b1; fault_val = 8'h01; end
        else begin
          iss = 1'b1; iss_we = ~op[11]; iss_addr = rdA;
          rb = first9({1'b0, op[7:0]});
          multi_start = 1'b1; ml_list_d = {1'b0, op[7:0]}; ml_load_d = op[11];
          if(op[11]) begin
            // LDM: base write-back now (the transfer addresses come from the bus address)
            w_en = ~op[f_r8]; w_idx = {1'b0, f_r8};
          end else ml_wb_d = 1'b1; // STM: write-back after the stores
        end
      end
      5'h1A, 5'h1B: begin // Bcc / UDF / SVC
        asel = A_PC4; bimm_en = 1'b1; bimm = {{23{op[7]}}, op[7:0], 1'b0};
        if(op[11:8] == 4'hE || op[11:8] == 4'hF) begin
          pc_en = 1'b0; done = 1'b0; set_fault = 1'b1; fault_val = (op[11:8] == 4'hE) ? 8'h04 : 8'h05;
        end else if(cond_pass(op[11:8], flag_n, flag_z, flag_c, flag_v)) pc_src = PC_ALU;
      end
      5'h1C: begin // B
        asel = A_PC4; bimm_en = 1'b1; bimm = {{20{op[10]}}, op[10:0], 1'b0};
        pc_src = PC_ALU;
      end
      default: begin // 32-bit instruction whose second halfword is not in the buffer
        pc_en = 1'b0; done = 1'b0;
        iss = 1'b1; iss_fetch = 1'b1; iss_addr = {pc2[31:2], 2'b00};
        nstate = S_FETCH2;
      end
    endcase
    if(mem_ld) begin
      // single load/store: address from the ALU; a register-offset store reads its data next cycle
      pc_en = 1'b0; done = 1'b0;
      if(m_we_d && op[15:12] == 4'h5) nstate = S_STRREG;
      else begin
        iss = 1'b1; iss_we = m_we_d; iss_size = m_size_d;
        iss_sum = 1'b1; iss_sum_clr = (m_size_d == 2'd2) ? 3'b011 : (m_size_d == 2'd1) ? 3'b001 : 3'b000;
        nstate = S_MEM;
      end
    end
  end else begin
    //---------------------------------------------------------------- other cycles
    case(state)
`ifdef MK2
      S_RST0: begin sp_en = 1'b1; sp_val = reset_sp; nstate = S_RST1; end
      S_RST1: begin w_en = 1'b1; w_idx = 4'd14; w_src = W_EXCLR; nstate = S_IDLE; end
`endif
      S_IDLE: begin
        if(!dbg_we && go) begin
          if(take_exc) begin
            // exception entry: frame at (SP - 32) & ~7, first word (R0) written now
            ra = 4'd13; rb = 4'd0; bimm_en = 1'b1; bimm = 32'h20; binv = 1'b1; cin_sel = 2'd1;
            exc_start = 1'b1;
            iss = 1'b1; iss_we = 1'b1; iss_sum = 1'b1; iss_sum_clr = 3'b111;
            nstate = S_EXC;
          end else begin
            iss = 1'b1; iss_fetch = 1'b1; iss_addr = {pc[31:2], 2'b00};
            nstate = S_FETCH;
          end
        end
      end
      S_FETCH: if(bus_ready) nstate = S_IDLE;   // word latched into the buffer, executed next cycle
      S_FETCH2: if(bus_ready) nstate = S_X32;
      S_MULW: begin
        ra = {1'b0, ir1[2:0]}; aop = OP_MISC; misc_op = 4'd1;
        w_en = 1'b1; w_idx = {1'b0, ir1[2:0]};
        fl_src = FL_ALU; fl_nz = 1'b1;
        pc_en = 1'b1; pc_src = PC_2; done = 1'b1;
      end
      S_MEM: begin
        if(bus_ready) begin
          bus_end = 1'b1;
          if(!m_we) begin w_en = 1'b1; w_idx = m_rt; w_src = W_LOAD; end
          else if(m_addr[31:2] == ibuf_addr) ibuf_inval = 1'b1;
          pc_en = 1'b1; pc_src = PC_2; done = 1'b1;
        end
      end
      S_STRREG: begin // register-offset store: the address was computed last cycle, read the data now
        rb = m_rt;
        iss = 1'b1; iss_we = 1'b1; iss_size = m_size; iss_addr = align(m_addr, m_size);
        nstate = S_MEM;
      end
      S_MULTI: begin
        rb = first9(ml_rest);   // next register to store (selected independently of bus_ready: keeps the cache hit out of the register-read path)
        if(bus_ready) begin
          if(ml_load) begin w_en = 1'b1; w_idx = ml_idx; w_src = W_BUS; end
          else if(bus_addr[31:2] == ibuf_addr) ibuf_inval = 1'b1;
          if(ml_rest != 9'd0) begin
            iss = 1'b1; iss_we = ~ml_load; iss_addr = bus_addr + 32'd4;
          end else if(ml_pop) begin
            iss = 1'b1; iss_addr = bus_addr + 32'd4;
            nstate = S_POPPC;
          end else begin
            bus_end = 1'b1;
            if(ml_wb) begin w_en = 1'b1; w_idx = {1'b0, ml_rn}; w_src = W_FINAL; end
            pc_en = 1'b1; pc_src = PC_2; done = 1'b1;
          end
        end
      end
      S_POPPC: begin
        ra = 4'd13;
        if(bus_ready) begin
`ifdef MK2
          if(!bus_rdata[0]) begin set_fault = 1'b1; fault_val = 8'h02; end
`else
          if(ipsr != 6'd0 && bus_rdata[31:28] == 4'hF) eret_start = 1'b1;
          else if(!bus_rdata[0]) begin set_fault = 1'b1; fault_val = 8'h02; end
`endif
          else begin bus_end = 1'b1; pc_en = 1'b1; pc_src = PC_BUS; done = 1'b1; end
        end
      end
      S_EXC: begin
        case(ex_idx)            // register for the next stacked word (independent of bus_ready)
          3'd3: rb = 4'd12;
          3'd4: rb = 4'd14;
          3'd5: iss_wd = WD_PC;
          3'd6: iss_wd = WD_XPSR;
          default: rb = {1'b0, ex_idx + 3'd1};
        endcase
        if(bus_ready) begin
          if(bus_addr[31:2] == ibuf_addr) ibuf_inval = 1'b1;
`ifdef MK2
          // LR <= EXC_RETURN once LR is stacked (word 5 was issued at ex_idx 4), SP <= frame at the end:
          // the register file has a single write port
          if(ex_idx == 3'd5) begin w_en = 1'b1; w_idx = 4'd14; w_src = W_EXCLR; end
`endif
          if(ex_idx == 3'd7) begin
            sp_en = 1'b1; sp_val = ex_sp;
`ifndef MK2
            w_en = 1'b1; w_idx = 4'd14; w_src = W_EXCLR;
`endif
            ipsr_set = 1'b1;
            iss = 1'b1; iss_addr = {vtor[31:8], 8'd0} + {24'd0, ex_num, 2'b00};
            nstate = S_VEC;
          end else begin
            iss = 1'b1; iss_we = 1'b1; iss_addr = bus_addr + 32'd4;
          end
        end
      end
      S_VEC: begin
        if(bus_ready) begin
          bus_end = 1'b1;
          pc_en = 1'b1; pc_src = PC_BUS;
          nstate = S_IDLE;
        end
      end
      S_ERET: begin
        if(bus_ready) begin
          case(ex_idx)
            3'd0, 3'd1, 3'd2, 3'd3: begin w_en = 1'b1; w_idx = {1'b0, ex_idx}; w_src = W_BUS; end
            3'd4: begin w_en = 1'b1; w_idx = 4'd12; w_src = W_BUS; end
            3'd5: begin w_en = 1'b1; w_idx = 4'd14; w_src = W_BUS; end
            3'd6: begin pc_en = 1'b1; pc_src = PC_BUS; end
            default: begin
              fl_src = FL_BUS; ipsr_clear = 1'b1;
              // own adder: keeps bus data (cache hit path) out of the main ALU, whose sum drives the bus address
              sp_en = 1'b1; sp_val = ex_sp + (bus_rdata[9] ? 32'h24 : 32'h20);
            end
          endcase
          if(ex_idx == 3'd7) begin bus_end = 1'b1; done = 1'b1; end
          else begin iss = 1'b1; iss_addr = bus_addr + 32'd4; end
        end
      end
      default: ;
    endcase
  end

  if(state == S_IDLE && dbg_we) begin
    w_en = (dbg_sel < 5'd15); w_idx = dbg_sel[3:0]; w_src = W_DBG;
  end
  if(eret_start) begin
    iss = 1'b1; iss_we = 1'b0; iss_addr = rdA; // first word of the frame at SP (ra = 13)
    nstate = S_ERET;
  end
  if(multi_start) nstate = S_MULTI;
  if(mul_start) nstate = S_MULW;
  if(done) nstate = S_IDLE;
  // early fetch: the next instruction's word is not in the buffer and its address does not depend on bus data
  if(EARLY_FETCH && !step_mode && done && pc_en && !iss && !set_fault) begin
    if((pc_src == PC_2 && pc[1]) || pc_src == PC_4) begin early = 1'b1; iss_addr = {pc4[31:2], 2'b00}; end
    else if(pc_src == PC_ALU && aop == OP_ADD) begin early = 1'b1; iss_sum = 1'b1; iss_sum_clr = 3'b011; end
    else if(pc_src == PC_RDB) begin early = 1'b1; iss_addr = {rdB[31:2], 2'b00}; end
    if(early) begin iss = 1'b1; iss_we = 1'b0; iss_fetch = 1'b1; iss_size = 2'd2; nstate = S_FETCH; end
  end
  if(set_fault) begin nstate = S_HALT; iss = 1'b0; bus_end = 1'b1; end
end

wire [31:0] iss_addr_f = iss_sum ? {sum[31:3], sum[2:0] & ~iss_sum_clr} : iss_addr;

assign bus_next_req = iss & ~dbg_now;
assign bus_next_addr = iss_addr_f;

// write-back data
reg [31:0] w_data;
always @* begin
  case(w_src)
    W_ALU: w_data = alu_res;
    W_LOAD: w_data = ld_data;
    W_DBG: w_data = dbg_wdata;
    W_PC2L: w_data = pc2 | 32'd1;
    W_PC4L: w_data = pc4 | 32'd1;
`ifdef MK2
    W_SYS: w_data = sysm[4] ? {31'd0, primask} : {26'd0, ipsr};          // PRIMASK (16) / IPSR (5)
    W_EXCLR: w_data = (state == S_RST1) ? 32'hFFFFFFFF : 32'hFFFFFFF9;   // reset LR / EXC_RETURN
`else
    W_SYS: begin
      case(sysm)
        8'd0, 8'd2: w_data = {flag_n, flag_z, flag_c, flag_v, 28'd0};      // APSR, EAPSR
        8'd1, 8'd3: w_data = {flag_n, flag_z, flag_c, flag_v, 22'd0, ipsr};// IAPSR, xPSR
        8'd5, 8'd7: w_data = {26'd0, ipsr};                                // IPSR, IEPSR
        8'd8, 8'd9: w_data = rf[13];                                       // MSP, PSP
        8'd16: w_data = {31'd0, primask};
        default: w_data = 32'd0;                                           // EPSR, CONTROL
      endcase
    end
    W_EXCLR: w_data = 32'hFFFFFFF9;
`endif
    default: w_data = ml_final;
  endcase
end

`ifdef MK2
wire [31:0] rf_dbg = rf[dbg_rsel[3:0] == 4'd15 ? 4'd14 : dbg_rsel[3:0]];   // a wire: XST rejects memory reads in always @*
`endif
always @* begin
  if(!DEBUG) dbg_rdata = 32'd0;
  else case(dbg_rsel)
    5'd15: dbg_rdata = pc;
    5'd16: dbg_rdata = {flag_n, flag_z, flag_c, flag_v, 28'd0};
    5'd17: dbg_rdata = {31'd0, primask};
    5'd18: dbg_rdata = {26'd0, ipsr};
    5'd20: dbg_rdata = stat_cycles[31:0];
    5'd21: dbg_rdata = stat_cycles[63:32];
    5'd22: dbg_rdata = stat_instr[31:0];
    5'd23: dbg_rdata = stat_instr[63:32];
`ifdef MK2
    default: dbg_rdata = rf_dbg;
`else
    default: dbg_rdata = rf[dbg_rsel[3:0] == 4'd15 ? 4'd14 : dbg_rsel[3:0]];
`endif
  endcase
end
`ifdef MK2

//------------------------------------------------------------------------------
// Register file write: one port (general write, SP write or a debug write; never two at once), no reset,
// asynchronous reads: distributed RAM on the Spartan-3
//------------------------------------------------------------------------------
wire rf_we = !rst && (w_en || (sp_en && !dbg_now));
wire [3:0] rf_wa = w_en ? w_idx : 4'd13;
wire [31:0] rf_wd = w_en ? w_data : sp_val;
always @(posedge clk) if(rf_we) rf[rf_wa] <= rf_wd;
// synthesis translate_off
always @(posedge clk) if(!rst && !dbg_now && w_en && sp_en) $display("[xc_m0] two register writes in one cycle (state %d)", state);
// synthesis translate_on
`endif

//------------------------------------------------------------------------------
// Sequential: apply the micro-op
//------------------------------------------------------------------------------
always @(posedge clk) begin
  if(rst) begin
`ifndef MK2
    rf[13] <= reset_sp;
    rf[14] <= 32'hFFFFFFFF;
`endif
    pc <= reset_pc;
    pc4_ok <= 1'b0;
    flag_n <= 1'b0; flag_z <= 1'b0; flag_c <= 1'b0; flag_v <= 1'b0;
    primask <= 1'b0;
    ipsr <= 6'd0;
`ifdef MK2
    state <= S_RST0;
`else
    state <= S_IDLE;
`endif
    bus_req <= 1'b0; bus_we <= 1'b0; bus_fetch <= 1'b0; bus_size <= 2'd2; bus_addr <= 32'd0; bus_wdata <= 32'd0;
    ibuf_valid <= 1'b0;
    step_done <= 1'b0;
    step_armed <= 1'b0;
    exc_ack <= 1'b0;
    fault <= 1'b0;
    fault_code <= 8'd0;
    sleeping <= 1'b0;
    stat_cycles <= 64'd0;
    stat_instr <= 64'd0;
  end else if(dbg_now) begin
    step_done <= 1'b0;
`ifndef MK2
    if(w_en) rf[w_idx] <= w_data;
`endif
    if(dbg_sel == 5'd15) begin pc <= {dbg_wdata[31:1], 1'b0}; pc4_ok <= 1'b0; end
    else if(dbg_sel == 5'd16) begin flag_n <= dbg_wdata[31]; flag_z <= dbg_wdata[30]; flag_c <= dbg_wdata[29]; flag_v <= dbg_wdata[28]; end
    else if(dbg_sel == 5'd17) primask <= dbg_wdata[0];
    else if(dbg_sel == 5'd18) ipsr <= dbg_wdata[5:0];
    else if(dbg_sel == 5'd19) ibuf_valid <= 1'b0;
  end else begin
    step_done <= done || (state == S_VEC && bus_ready);
    if(done || (state == S_VEC && bus_ready) || set_fault) step_armed <= 1'b0;
    else if(step_go) step_armed <= 1'b1;
    exc_ack <= exc_start;
    sleeping <= is_sleep;
    if(DEBUG && state != S_HALT && (state != S_IDLE || go)) stat_cycles <= stat_cycles + 64'd1;
    if(DEBUG && done) stat_instr <= stat_instr + 64'd1;
    state <= nstate;

`ifndef MK2
    // register file: one general write port + SP port
    if(sp_en) rf[13] <= sp_val;
    if(w_en) rf[w_idx] <= w_data;
`endif

    // flags
    case(fl_src)
      FL_ALU: begin
        if(fl_nz) begin flag_n <= alu_res[31]; flag_z <= (alu_res == 32'd0); end
        if(fl_c) flag_c <= fl_c_sh ? sh_c : sum[32];
        if(fl_v) flag_v <= add_v;
      end
`ifndef MK2
      FL_RDB: begin flag_n <= rdB[31]; flag_z <= rdB[30]; flag_c <= rdB[29]; flag_v <= rdB[28]; end
`endif
      FL_BUS: begin flag_n <= bus_rdata[31]; flag_z <= bus_rdata[30]; flag_c <= bus_rdata[29]; flag_v <= bus_rdata[28]; end
      default: ;
    endcase
    if(primask_en) primask <= primask_val;
    if(ipsr_clear) ipsr <= 6'd0;
    if(ipsr_set) ipsr <= ex_num;

    // program counter
    if(pc_en) begin
      case(pc_src)
        PC_2: begin pc <= pc2; pc4r <= pc4r + 32'd2; end
        PC_4: begin pc <= pc4r; pc4r <= pc4r + 32'd4; end
        PC_ALU: begin pc <= {alu_res[31:1], 1'b0}; pc4_ok <= 1'b0; end
        PC_RDB: begin pc <= {rdB[31:1], 1'b0}; pc4_ok <= 1'b0; end
        default: begin pc <= {bus_rdata[31:1], 1'b0}; pc4_ok <= 1'b0; end
      endcase
    end else if(!pc4_ok) begin
      pc4r <= pc + 32'd4;
      pc4_ok <= 1'b1;
    end

    // fetch buffer
    if(fetch_now) begin
      ibuf <= bus_rdata;
      ibuf_addr <= bus_addr[31:2];
      ibuf_valid <= 1'b1;
    end
    if(ibuf_inval) ibuf_valid <= 1'b0;
    if(exec16_now && !exec32_now) ir1 <= op;   // first half of a 32-bit instruction, or the MUL for S_MULW

    // bus
    if(iss) begin
      bus_req <= 1'b1;
      bus_we <= iss_we;
      bus_fetch <= iss_fetch;
      bus_size <= iss_size;
      bus_addr <= iss_addr_f;
      case(iss_wd)
        WD_PC: bus_wdata <= pc;
        WD_XPSR: bus_wdata <= ex_xpsr;
        default: bus_wdata <= (iss_size == 2'd0) ? {24'd0, rdB[7:0]} : (iss_size == 2'd1) ? {16'd0, rdB[15:0]} : rdB;
      endcase
    end else if(bus_end || fetch_now) begin
      bus_req <= 1'b0; bus_we <= 1'b0; bus_fetch <= 1'b0;
    end

    // single load/store
    if(mem_ld) begin
      m_we <= m_we_d; m_size <= m_size_d; m_signed <= m_sgn_d; m_rt <= m_rt_d;
      m_addr <= sum[31:0];
    end

    // multi-register transfers
    if(multi_start) begin
      ml_list <= ml_list_d;
      ml_idx <= first9(ml_list_d);
      ml_load <= ml_load_d;
      ml_pop <= ml_pop_d;
      ml_wb <= ml_wb_d;
      ml_rn <= f_r8;
      ml_final <= sum[31:0];
    end else if(state == S_MULTI && bus_ready && ml_rest != 9'd0) begin
      ml_list <= ml_rest;
      ml_idx <= first9(ml_rest);
    end

    // exceptions
    if(exc_start) begin
      ex_num <= exc_num;
      ex_sp <= {sum[31:3], 3'b000};
      ex_xpsr <= xpsr | (rdA[2] ? 32'h200 : 32'h0);
      ex_idx <= 3'd0;
    end else if(eret_start) begin
      ex_sp <= rdA;
      ex_idx <= 3'd0;
    end else if((state == S_EXC || state == S_ERET) && bus_ready) begin
      ex_idx <= ex_idx + 3'd1;
    end

    if(set_fault) begin
      fault <= 1'b1;
      fault_code <= fault_val;
    end
  end
end

endmodule
