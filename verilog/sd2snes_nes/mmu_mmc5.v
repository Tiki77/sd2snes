`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// mmu_mmc5.v -- iNES mapper 5 (Nintendo MMC5 / ExROM) for the NES_MMC5 variant of
// sd2snes_nes (verilog/sd2snes_nes_mmc5).  Compiled ONLY with the NES_MMC5 macro;
// the base core never sees this file.
//
// Contents
//   * module MMC5       -- the mapper: PRG/CHR banking (all four modes of each),
//                          the two CHR register sets, nametable mapping (CIRAM
//                          0/1, ExRAM, fill), ExRAM, the scanline IRQ, the in-frame
//                          flag, the hardware multiplier, and the register taps for
//                          the video bridge.
//   * module MultiMapper -- the NES_MMC5 flavour of the mapper multiplexer.  Same
//                          port list as the stock one in mmu.v (nes.v instantiates
//                          one or the other) plus the appended MMC5 ports.  Mapper 5
//                          drives everything; any other mapper number falls back to
//                          MMC0 (the loader only ever hands this core mapper 5, the
//                          fallback just keeps an idle core defined).
//
// Source of truth: the nesdev wiki MMC5 page (public hardware documentation); the
// MMC5 module of the vendored fpganes core was the starting point and differs in
// the places listed below.  Deviations from the fpganes module, each one a bug or
// a gap for THIS core:
//   * every sequential element is gated by `ce` (== ppu_ce, one PPU dot): main.sdc
//     declares the NES:core multicycle on that premise (the clock-enable contract of the NES:core); the
//     fpganes module had a free-running `last_read_ram`.
//   * ExRAM is a true M9K (1K x 8) with ONE registered read port (CPU in the RAM
//     modes, PPU otherwise -- the two never overlap) and ONE write port (CPU or
//     PPUDATA into an ExRAM-mapped nametable).
//   * the ex-attribute byte is LATCHED at the nametable fetch (`ex_byte`) and used
//     for the attribute fetch and both pattern fetches of that tile.
//   * PPUDATA writes into an ExRAM-mapped nametable are honoured.
//   * the scanline IRQ is armed at dot 4 of the target visible line, `$5204` bit 6
//     is a real in-frame flag, PRG-RAM is a 64 KB superset in the CART-RAM window.
//   * 8x16 sprites use the independent B/A register sets ONLY while the PPU is
//     rendering (nesdev: "only when Z is set and at least one E bit is set");
//     CPU/$2007 accesses use the set written last.
//   * `$5130` is stored but not applied to the CHR bank (no licensed title relies
//     on it; Metal Slader Glory banks 4 KB with 8 bits).
// Macros: MMC5_EXATTR (extended attribute mode) and MMC5_SPLIT (vertical split)
// are unit-tested but NOT part of the variant's first step; the variant's
// main.qsf leaves them undefined, so the RTL below them is not synthesized.
//
// Video-bridge taps (all combinational from registers; nes_m5_capture registers
// them under ce): snap_state is the 22-byte payload of CMD_M5_STATE ($17) in wire
// order, ev_* the register-write strobe feeding CMD_M5_EVENTS ($18).
//////////////////////////////////////////////////////////////////////////////////
module MMC5(input clk, input ce, input reset,
            input [31:0] flags,
            input [19:0] ppuflags,                // {scanline[8:0], cycle[8:0], obj_size, is_rendering}
            input [15:0] prg_ain, output [21:0] prg_aout,
            input prg_read, input prg_write, input [7:0] prg_din,
            input cpu_smp,                        // ce edge at which the CPU latches the read data bus
            output reg [7:0] prg_dout, output prg_allow,
            input chr_read, input chr_write, input [7:0] chr_din,
            input [13:0] chr_ain, output reg [21:0] chr_aout,
            output reg [7:0] chr_dout, output has_chr_dout,
            output chr_allow, output vram_a10, output vram_ce, output irq,
            // ---- video-bridge taps ----
            output [175:0] snap_state,   // CMD_M5_STATE payload, byte i at [i*8 +: 8]
            output         ev_we,        // register-write strobe (qualified with ce)
            output [4:0]   ev_idx,
            output [7:0]   ev_val);
  reg [1:0] prg_mode, chr_mode;
  reg prg_protect_1, prg_protect_2;
  reg [1:0] exmode;
  reg [7:0] ntmap, fill_tile;
  reg [1:0] fill_attr;
  reg [2:0] prg_ram_bank;
  reg [7:0] prg_bank_0, prg_bank_1, prg_bank_2;
  reg [6:0] prg_bank_3;
  reg [7:0] cb [0:11];                   // $5120-$512B raw (A0..A7, B0..B3)
  reg [1:0] chr_hi;
  reg chr_last;                          // 0: A set written last, 1: B set
  reg [7:0] irq_scanline;
  reg irq_enable, irq_pending;
  reg [7:0] mul1, mul2;
  wire [15:0] mul_res = mul1 * mul2;
`ifdef MMC5_SPLIT
  reg [4:0] vsplit_tile; reg vsplit_en, vsplit_side; reg [7:0] vsplit_scroll, vsplit_bank;
  wire [7:0] vsplit_bank_w = vsplit_bank;
`else
  wire [7:0] vsplit_bank_w = 8'd0;
`endif

  reg [7:0] ex_byte; reg nt_rd_q;
  reg [7:0] exr_q;
  // Power-up values: the MMC5 has no reset for most of its registers, the games write
  // what they use, and the core is reconfigured for every load.  Fixing them keeps the
  // simulation (and the bridge output) free of X; Cyclone IV registers power up at the
  // `initial` value.
  integer ii;
  initial begin
    prg_mode = 0; chr_mode = 0; prg_protect_1 = 0; prg_protect_2 = 0; exmode = 2'd3;
    ntmap = 0; fill_tile = 0; fill_attr = 0; prg_ram_bank = 0;
    prg_bank_0 = 0; prg_bank_1 = 0; prg_bank_2 = 0; prg_bank_3 = 7'h7F;
    for (ii = 0; ii < 12; ii = ii + 1) cb[ii] = 0;
    chr_hi = 0; chr_last = 0; irq_scanline = 0; irq_enable = 0; irq_pending = 0;
    mul1 = 0; mul2 = 0; ex_byte = 0; nt_rd_q = 0; exr_q = 0;
`ifdef MMC5_SPLIT
    vsplit_tile = 0; vsplit_en = 0; vsplit_side = 0; vsplit_scroll = 0; vsplit_bank = 0;
`endif
  end
  wire [8:0] ppu_scanline = ppuflags[19:11];
  wire [8:0] ppu_cycle    = ppuflags[10:2];
  wire ppu_spr16          = ppuflags[1];
  wire ppu_rendering      = ppuflags[0];
  // visible-line rendering (the pre-render line has scanline == 9'h1FF)
  wire in_frame = ppu_rendering && !ppu_scanline[8];

  // ---------------------------------------------------------------- registers
  wire reg_wr = prg_write && prg_ain[15:10] == 6'b010100;      // $5000-$53FF
  wire [9:0] ra = prg_ain[9:0];
  wire chr_reg_wr = reg_wr && ra[9:4] == 6'h12 && ra[3:2] != 2'b11;   // $5120-$512B
  always @(posedge clk) begin
    if (reset) begin
      prg_mode <= 2'd3; prg_bank_3 <= 7'h7F;
      irq_enable <= 0; exmode <= 2'd3; chr_hi <= 0;
`ifdef MMC5_SPLIT
      vsplit_en <= 0;
`endif
    end else if (ce) begin
      if (reg_wr) begin
        case (ra)
        10'h100: prg_mode <= prg_din[1:0];
        10'h101: chr_mode <= prg_din[1:0];
        10'h102: prg_protect_1 <= (prg_din[1:0] == 2'b10);
        10'h103: prg_protect_2 <= (prg_din[1:0] == 2'b01);
        10'h104: exmode <= prg_din[1:0];
        10'h105: ntmap <= prg_din;
        10'h106: fill_tile <= prg_din;
        10'h107: fill_attr <= prg_din[1:0];
        10'h113: prg_ram_bank <= prg_din[2:0];
        10'h114: prg_bank_0 <= prg_din;
        10'h115: prg_bank_1 <= prg_din;
        10'h116: prg_bank_2 <= prg_din;
        10'h117: prg_bank_3 <= prg_din[6:0];
        10'h130: chr_hi <= prg_din[1:0];
        10'h203: irq_scanline <= prg_din;
        10'h204: irq_enable <= prg_din[7];
        10'h205: mul1 <= prg_din;
        10'h206: mul2 <= prg_din;
`ifdef MMC5_SPLIT
        10'h200: {vsplit_en, vsplit_side, vsplit_tile} <= {prg_din[7:6], prg_din[4:0]};
        10'h201: vsplit_scroll <= prg_din;
        10'h202: vsplit_bank <= prg_din;
`endif
        default: ;
        endcase
        if (chr_reg_wr) begin
          cb[ra[3:0]] <= prg_din;
          chr_last <= ra[3];
        end
      end
    end
  end

  // ------------------------------------------------------------------- ExRAM
  // M9K 1Kx8.  Write: CPU ($5C00-$5FFF) or PPUDATA into an ExRAM-mapped nametable.
  //   CPU modes: 0/1 store the data only while rendering (else 0), 2 free, 3 RO.
  wire [1:0] mirrbits = chr_ain[11:10] == 2'd0 ? ntmap[1:0] :
                        chr_ain[11:10] == 2'd1 ? ntmap[3:2] :
                        chr_ain[11:10] == 2'd2 ? ntmap[5:4] : ntmap[7:6];
  wire cpu_ex = prg_ain[15:10] == 6'b010111;
  wire ex_we_cpu = prg_write && cpu_ex && exmode != 2'd3;
  wire ex_we_ppu = chr_write && chr_ain[13] && mirrbits == 2'd2 && !exmode[1];
  wire [9:0] ex_waddr = ex_we_cpu ? prg_ain[9:0] : chr_ain[9:0];
  wire [7:0] ex_wdata = ex_we_cpu ? ((exmode[1] || in_frame) ? prg_din : 8'h00) : chr_din;
`ifdef MMC5_EXATTR
  localparam EXA = 1'b1;
`else
  localparam EXA = 1'b0;
`endif
  (* ramstyle = "M9K, no_rw_check" *) reg [7:0] exmem [0:1023];
  reg [9:0] exr_addr;
  // synthesis translate_off
  integer ei;
  initial for (ei = 0; ei < 1024; ei = ei + 1) exmem[ei] = 8'h00;
  // synthesis translate_on
`ifdef MMC5_SPLIT
  reg [5:0] cur_tile; reg last_in_split; reg [7:0] vscroll;
  wire [5:0] new_cur_tile = (ppu_cycle[8:3] == 6'd40) ? 6'd0 : (cur_tile + 6'd1);
  reg in_split;
  always @* begin
    in_split = last_in_split;
    if (ppu_cycle[2:0] == 0 && ppu_cycle < 336) begin
      if (new_cur_tile == 0) in_split = !vsplit_side;
      else if (new_cur_tile == {1'b0, vsplit_tile}) in_split = vsplit_side;
      else if (new_cur_tile == 34) in_split = 0;
    end
  end
  always @(posedge clk) if (ce) begin
    last_in_split <= in_split;
    if (ppu_cycle[2:0] == 0 && ppu_cycle < 336) cur_tile <= new_cur_tile;
    if (ppu_cycle == 9'd319) vscroll <= ppu_scanline[8] ? vsplit_scroll :
                                        (vscroll == 8'd239) ? 8'd0 : vscroll + 8'd1;
  end
  wire insplit = in_split && vsplit_en && !exmode[1] && in_frame;
  wire [9:0] loopy = {vscroll[7:3], cur_tile[4:0]};
  wire [9:0] split_addr = (ppu_cycle[1] == 0) ? loopy : {4'b1111, loopy[9:7], loopy[4:2]};
  wire [1:0] split_attr = (!loopy[1] && !loopy[6]) ? exr_q[1:0] : ( loopy[1] && !loopy[6]) ? exr_q[3:2] :
                          (!loopy[1] &&  loopy[6]) ? exr_q[5:4] : exr_q[7:6];
`else
  wire insplit = 1'b0;
  wire [9:0] split_addr = 10'd0;
  wire [1:0] split_attr = 2'd0;
`endif
  always @* exr_addr = exmode[1] ? prg_ain[9:0] : (insplit ? split_addr : chr_ain[9:0]);
  always @(posedge clk) if (ce) begin
    if (ex_we_cpu || ex_we_ppu) exmem[ex_waddr] <= ex_wdata;
    exr_q <= exmem[exr_addr];
  end

  // ------------------------------------------------------- scanline IRQ / status
  reg inframe_q;
  reg rd5204_q = 1'b0;
  always @(posedge clk) begin
    if (reset) begin irq_pending <= 0; inframe_q <= 0; rd5204_q <= 0; end
    else if (ce) begin
      // A $5204 read acknowledges the IRQ at the END of the read cycle: the strobe
      // (`prg_read`, CPU cycle phase 0) only arms `rd5204_q`; the flag is cleared on the edge
      // where the CPU latches the data bus (`cpu_smp`, phase 2), so the byte the CPU sees
      // still carries the pending bit it is acknowledging.
      if (prg_read && prg_ain == 16'h5204) rd5204_q <= 1;
      if (cpu_smp) begin
        rd5204_q <= 0;
        if (rd5204_q) irq_pending <= 0;
      end
      if (ppu_cycle == 9'd4 && in_frame) inframe_q <= 1;
      if (ppu_scanline == 9'd240 && ppu_cycle == 9'd12) inframe_q <= 0;   // PPU idle >= 3 CPU cycles after line 239
      if (ppu_scanline == 9'd241 && ppu_cycle == 9'd1) irq_pending <= 0;  // vblank NMI acknowledges (nesdev)
      if (in_frame && ppu_cycle == 9'd4 && irq_scanline != 8'd0 &&
          ppu_scanline[7:0] == irq_scanline && ppu_scanline < 9'd240)
        irq_pending <= 1;
    end
  end
  assign irq = irq_pending && irq_enable;

  // ------------------------------------------------------------- CPU read side
  always @* begin
    prg_dout = 8'hFF;
    if (cpu_ex && exmode[1])      prg_dout = exr_q;
    else if (prg_ain == 16'h5204) prg_dout = {irq_pending, inframe_q, 6'b000000};
    else if (prg_ain == 16'h5205) prg_dout = mul_res[7:0];
    else if (prg_ain == 16'h5206) prg_dout = mul_res[15:8];
  end

  // ---------------------------------------------------------------- PRG mapping
  reg [7:0] psel;      // {1=ROM, bank}  (ROM: A19..A13 in [6:0]; RAM: bank in [2:0])
  always @* begin
    psel = {1'b0, 4'b0, prg_ram_bank};                        // $6000-$7FFF always RAM
    if (prg_ain[15]) begin
      case (prg_mode)
      2'd0: psel = {1'b1, prg_bank_3[6:2], prg_ain[14:13]};
      2'd1: psel = prg_ain[14] ? {1'b1, prg_bank_3[6:1], prg_ain[13]}
                               : {prg_bank_1[7], prg_bank_1[6:1], prg_ain[13]};
      2'd2: case (prg_ain[14:13])
            2'd0, 2'd1: psel = {prg_bank_1[7], prg_bank_1[6:1], prg_ain[13]};
            2'd2:       psel = prg_bank_2;
            default:    psel = {1'b1, prg_bank_3};
            endcase
      default: case (prg_ain[14:13])
            2'd0: psel = prg_bank_0;
            2'd1: psel = prg_bank_1;
            2'd2: psel = prg_bank_2;
            default: psel = {1'b1, prg_bank_3};
            endcase
      endcase
    end
  end
  wire is_rom = psel[7];
  assign prg_aout = is_rom ? {2'b00, psel[6:0], prg_ain[12:0]}
                           : {4'b1111, 2'b00, psel[2:0], prg_ain[12:0]};  // 0x3C0000 + 64K
  assign prg_allow = (prg_ain >= 16'h6000) && (!prg_write || (!is_rom && prg_protect_1 && prg_protect_2));

  // ---------------------------------------------------------------- CHR mapping
  // 8x16 sprites (and only while the PPU renders): B regs for every BG fetch, A regs
  // for sprite fetches (dots 256-319).  Otherwise the set written LAST serves all.
  wire is_bg_fetch = !(ppu_cycle[8] && !ppu_cycle[6]);
  wire useB = (ppu_spr16 && ppu_rendering) ? is_bg_fetch : chr_last;
  reg [9:0] chrsel;
  wire [2:0] k = chr_ain[12:10];
  reg [7:0] r2a;
  always @* begin
    r2a = 8'd0;
    case (k[2:1])
    2'd0: r2a = cb[1]; 2'd1: r2a = cb[3]; 2'd2: r2a = cb[5]; default: r2a = cb[7];
    endcase
    chrsel = 10'd0;
    case (chr_mode)
    2'd0: chrsel = {(useB ? cb[11][6:0] : cb[7][6:0]), k};                          // 8 KB
    2'd1: chrsel = {(useB ? cb[11] : (k[2] ? cb[7] : cb[3])), k[1:0]};              // 4 KB
    2'd2: chrsel = useB ? {1'b0, (k[1] ? cb[11] : cb[9]), k[0]} : {1'b0, r2a, k[0]}; // 2 KB
    default: chrsel = {2'b00, useB ? cb[8 + k[1:0]] : cb[k]};                       // 1 KB
    endcase
  end
  wire exattr_on = EXA && (exmode == 2'd1) && in_frame && is_bg_fetch && !chr_ain[13];
  always @* begin
    chr_aout = {2'b10, chrsel, chr_ain[9:0]};
    if (insplit)         chr_aout = {2'b10, vsplit_bank_w, chr_ain[11:3], 3'b000};
    else if (exattr_on)  chr_aout = {2'b10, chr_hi, ex_byte[5:0], chr_ain[11:0]};
  end

  // ex-attr byte: latched one ce after the nametable fetch address
  wire is_at = chr_ain[9:6] == 4'b1111;
  always @(posedge clk) if (ce) begin
    nt_rd_q <= EXA && chr_read && chr_ain[13] && !is_at && exmode == 2'd1 && in_frame;
    if (nt_rd_q) ex_byte <= exr_q;
  end

  // ----------------------------------------------------- nametable data override
  wire exattr_at = EXA && (exmode == 2'd1) && in_frame && is_at;
  assign has_chr_dout = chr_ain[13] && (mirrbits[1] || insplit || exattr_at);
  wire [1:0] ov_attr = insplit ? split_attr : (EXA && exmode == 2'd1) ? ex_byte[7:6] : fill_attr;
  always @* begin
    if (!is_at) chr_dout = (insplit || mirrbits[0] == 1'b0) ? (exmode[1] ? 8'h00 : exr_q) : fill_tile;
    else if (!insplit && !exattr_at && mirrbits[0] == 1'b0) chr_dout = exmode[1] ? 8'h00 : exr_q;
    else chr_dout = {ov_attr, ov_attr, ov_attr, ov_attr};
  end
  assign vram_a10 = mirrbits[0];
  assign vram_ce  = chr_ain[13] && !mirrbits[1];
  assign chr_allow = flags[15];

  // ------------------------------------------------------------------- taps
  // CMD_M5_STATE payload (22 bytes, wire order; bridge command $17):
  //   0  flags  {0, exmode[1:0], split_side, split_en, chr_last, spr16, exattr_active}
  //   1  {0000, chr_hi[1:0], chr_mode[1:0]}
  //   2..9   $5120-$5127 (A0..A7)      10..13  $5128-$512B (B0..B3)
  //   14 $5105   15 $5106   16 {000000, $5107[1:0]}
  //   17 split tile   18 split scroll   19 split bank
  //   20 CHR size mask {0, chr_mask[6:0]} (the 1 KB ids the renderer resolves are masked
  //      like the address path: id & {chr_mask, 3'b111})   21 reserved (0)
  wire [7:0] st_flags = {1'b0, exmode,
`ifdef MMC5_SPLIT
                         vsplit_side, vsplit_en,
`else
                         2'b00,
`endif
                         chr_last, ppu_spr16, EXA & (exmode == 2'd1)};
  assign snap_state[7:0]     = st_flags;
  assign snap_state[15:8]    = {4'd0, chr_hi, chr_mode};
  genvar gi;
  generate for (gi = 0; gi < 12; gi = gi + 1) begin : gc
    assign snap_state[(2+gi)*8 +: 8] = cb[gi];
  end endgenerate
  assign snap_state[119:112] = ntmap;
  assign snap_state[127:120] = fill_tile;
  assign snap_state[135:128] = {6'd0, fill_attr};
`ifdef MMC5_SPLIT
  assign snap_state[143:136] = {3'd0, vsplit_tile};
  assign snap_state[151:144] = vsplit_scroll;
  assign snap_state[159:152] = vsplit_bank;
`else
  assign snap_state[159:136] = 24'd0;
`endif
  // 8 KB-bank mask of the CHR size class (flags[13:11]): class c -> (1<<c)-1
  wire [6:0] chr_mask_t = ~(7'h7F << flags[13:11]);
  assign snap_state[167:160] = {1'b0, chr_mask_t};
  assign snap_state[175:168] = 8'd0;

  // Register-write events.  idx: 0-7 = $5120-$5127, 8-11 = $5128-$512B, 13 = $5130,
  // 16 = $5101.  Those are the registers the renderer's CHR view depends on; the
  // nametable/fill/ExRAM-mode ones (idx 12,14,15) and the split ones (17-19) are
  // reserved for the steps that render them and are not logged.
  reg ev_we_r; reg [4:0] ev_idx_r;
  always @* begin
    ev_we_r = 1'b0; ev_idx_r = 5'd0;
    if (chr_reg_wr) begin ev_we_r = 1'b1; ev_idx_r = {1'b0, ra[3:0]}; end
    else if (reg_wr && ra == 10'h130) begin ev_we_r = 1'b1; ev_idx_r = 5'd13; end
    else if (reg_wr && ra == 10'h101) begin ev_we_r = 1'b1; ev_idx_r = 5'd16; end
  end
  assign ev_we  = ev_we_r && ce;
  assign ev_idx = ev_idx_r;
  assign ev_val = prg_din;
endmodule


// ---------------------------------------------------------------------------
// MultiMapper, NES_MMC5 flavour.  Same ports as the stock module (mmu.v) plus the
// appended ones; the nt_snap_* taps follow `$5105`: the nametable ARRANGEMENT the
// bridge publishes is the CIRAM page of each quadrant, where a quadrant mapped to
// ExRAM or fill takes the page of its own column (q & 1) -- the renderer shows
// them as the CIRAM page of that column until the step that draws ExRAM/fill
// pages (the Castlevania III `$5105` values $E4 and $44 both give code $A).
// ---------------------------------------------------------------------------
module MultiMapper(input clk, input ce, input ppu_ce, input reset,
                   input [19:0] ppuflags,
                   input [31:0] flags,
                   input [15:0] prg_ain, output reg [21:0] prg_aout,
                   input prg_read, prg_write,
                   input [7:0] prg_din, output reg [7:0] prg_dout,
                   input [7:0] prg_from_ram,
                   output reg prg_allow,
                   input chr_read,
                   input [13:0] chr_ain, output reg [21:0] chr_aout,
                   output reg [7:0] chr_dout,
                   output reg has_chr_dout,
                   output reg chr_allow,
                   output reg vram_a10,
                   output reg vram_ce,
                   output reg irq,
                   output reg        chr_snap_s1_present,
                   output reg [7:0]  chr_snap_s0_bank,
                   output reg [7:0]  chr_snap_s1_bank,
                   output reg [63:0] chr_snap_win,
                   output reg [7:0]  chr_snap_win_flags,
                   output reg        chr_snap_win_en,
                   output reg [1:0]  nt_snap_arr,
                   output reg [3:0]  nt_snap_code,
                   // ---- NES_MMC5 ports ----
                   input             chr_write,
                   input  [7:0]      chr_din,
                   input             cpu_smp,
                   output            m5_en,
                   output [175:0]    m5_base,
                   output [167:0]    m5_ev,
                   output [3:0]      m5_cnt,
                   output            m5_ovf);
  wire h_mmc5 = (flags[7:0] == 8'd5);

  wire mmc0_prg_allow, mmc0_vram_a10, mmc0_vram_ce, mmc0_chr_allow;
  wire [21:0] mmc0_prg_addr, mmc0_chr_addr;
  MMC0 mmc0(clk, ce, flags, prg_ain, mmc0_prg_addr, prg_read, prg_write, prg_din, mmc0_prg_allow,
                            chr_ain, mmc0_chr_addr, mmc0_chr_allow, mmc0_vram_a10, mmc0_vram_ce);

  wire m5_prg_allow, m5_vram_a10, m5_vram_ce, m5_chr_allow, m5_irq, m5_has_dout;
  wire [21:0] m5_prg_addr, m5_chr_addr;
  wire [7:0] m5_prg_dout, m5_chr_dout;
  wire [175:0] m5_state;
  wire m5_ev_we; wire [4:0] m5_ev_idx; wire [7:0] m5_ev_val;
  MMC5 mmc5(clk, ppu_ce, reset, flags, ppuflags, prg_ain, m5_prg_addr, prg_read, prg_write, prg_din,
            cpu_smp, m5_prg_dout, m5_prg_allow, chr_read, chr_write, chr_din, chr_ain, m5_chr_addr, m5_chr_dout,
            m5_has_dout, m5_chr_allow, m5_vram_a10, m5_vram_ce, m5_irq,
            m5_state, m5_ev_we, m5_ev_idx, m5_ev_val);
  nes_m5_capture m5cap(clk, ppu_ce, reset, ppuflags[19:11], ppuflags[10:2], ppuflags[0],
                       m5_state, m5_ev_we, m5_ev_idx, m5_ev_val,
                       m5_base, m5_ev, m5_cnt, m5_ovf);
  assign m5_en = 1'b1;

  reg [5:0] prg_mask; reg [6:0] chr_mask;
  always @* begin
    case(flags[10:8])
    0: prg_mask = 6'b000000;
    1: prg_mask = 6'b000001;
    2: prg_mask = 6'b000011;
    3: prg_mask = 6'b000111;
    4: prg_mask = 6'b001111;
    5: prg_mask = 6'b011111;
    default: prg_mask = 6'b111111;
    endcase
    case(flags[13:11])
    0: chr_mask = 7'b0000000;
    1: chr_mask = 7'b0000001;
    2: chr_mask = 7'b0000011;
    3: chr_mask = 7'b0000111;
    4: chr_mask = 7'b0001111;
    5: chr_mask = 7'b0011111;
    6: chr_mask = 7'b0111111;
    7: chr_mask = 7'b1111111;
    endcase

    irq          = m5_irq & h_mmc5;
    prg_dout     = h_mmc5 ? m5_prg_dout : 8'hff;
    has_chr_dout = h_mmc5 & m5_has_dout;
    chr_dout     = h_mmc5 ? m5_chr_dout : 8'hff;
    prg_aout     = h_mmc5 ? m5_prg_addr  : mmc0_prg_addr;
    chr_aout     = h_mmc5 ? m5_chr_addr  : mmc0_chr_addr;
    prg_allow    = h_mmc5 ? m5_prg_allow : mmc0_prg_allow;
    chr_allow    = h_mmc5 ? m5_chr_allow : mmc0_chr_allow;
    vram_a10     = h_mmc5 ? m5_vram_a10  : mmc0_vram_a10;
    vram_ce      = h_mmc5 ? m5_vram_ce   : mmc0_vram_ce;

    if (prg_aout[21:20] == 2'b00)
      prg_aout[19:0] = {prg_aout[19:14] & prg_mask, prg_aout[13:0]};
    if (chr_aout[21:20] == 2'b10)
      chr_aout[19:0] = {chr_aout[19:13] & chr_mask, chr_aout[12:0]};
    chr_aout = vram_ce ? {11'b11_0000_0000_0, vram_a10, chr_ain[9:0]} : chr_aout;
    prg_aout = (prg_ain < 'h2000) ? {11'b11_1000_0000_0, prg_ain[10:0]} : prg_aout;
    prg_allow = prg_allow || (prg_ain < 'h2000);
  end

  // The legacy CHR taps stay constant, like mapper 4's: the CHR view of the MMC5
  // travels in CMD_M5_STATE / CMD_M5_EVENTS, and a constant slot pair is what keeps
  // CMD_CHR_SPLITS ($13) unreachable (see the sentinel note in mmu.v).
  // Nametable code per quadrant (nesdev $5105: 0 CIRAM0, 1 CIRAM1, 2 ExRAM, 3 fill).
  wire [7:0] ntmap_w = m5_state[119:112];
  wire [3:0] nt_code_w = { (ntmap_w[7:6] > 2'd1) ? 1'b1 : ntmap_w[6],
                           (ntmap_w[5:4] > 2'd1) ? 1'b0 : ntmap_w[4],
                           (ntmap_w[3:2] > 2'd1) ? 1'b1 : ntmap_w[2],
                           (ntmap_w[1:0] > 2'd1) ? 1'b0 : ntmap_w[0] };
  // legacy 2-bit arrangement from the 4-bit code, lockstep with
  // bridge_sim/mappers.py NTCODE_LEGACY (0=H 1=V 2=1A 3=1B)
  function [1:0] legacy_of;
    input [3:0] code;
    case (code)
      4'h0: legacy_of = 2'd2;  4'h1: legacy_of = 2'd2;  4'h2: legacy_of = 2'd2;  4'h3: legacy_of = 2'd0;
      4'h4: legacy_of = 2'd2;  4'h5: legacy_of = 2'd1;  4'h6: legacy_of = 2'd0;  4'h7: legacy_of = 2'd3;
      4'h8: legacy_of = 2'd2;  4'h9: legacy_of = 2'd0;  4'ha: legacy_of = 2'd1;  4'hb: legacy_of = 2'd3;
      4'hc: legacy_of = 2'd0;  4'hd: legacy_of = 2'd3;  4'he: legacy_of = 2'd3;  4'hf: legacy_of = 2'd3;
    endcase
  endfunction
  always @(posedge clk) begin
    if (reset) begin
      nt_snap_code <= 4'hc; nt_snap_arr <= 2'd0;
    end else if (ce) begin
      if (h_mmc5) begin nt_snap_code <= nt_code_w; nt_snap_arr <= legacy_of(nt_code_w); end
      else begin nt_snap_code <= flags[14] ? 4'ha : 4'hc; nt_snap_arr <= flags[14] ? 2'd1 : 2'd0; end
      chr_snap_s1_present <= 1'b0; chr_snap_s0_bank <= 8'd0; chr_snap_s1_bank <= 8'd0;
      chr_snap_win <= 64'd0; chr_snap_win_flags <= 8'd0; chr_snap_win_en <= 1'b0;
    end
  end
endmodule
