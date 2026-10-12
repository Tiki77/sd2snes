// sd2snes SMS core — SMS subsystem (Z80 + Sega mapper + WRAM + VDP capture).
//
// RTL port of the validated C behavioral model (ciclone sms_model.cpp): the Z80
// runs the SMS ROM (from PSRAM via rom_*) and the VDP captures VRAM/CRAM/regs,
// mirroring the C model that boots Sonic. The SMS->SNES translation bridge (M7.3)
// and the main.v PSRAM/SNES integration (M7.4) come next; VRAM/CRAM are exposed
// via registered read ports for that.
//
// BRAM-friendly (M7.2b): WRAM and VRAM are registered-read so Quartus infers M9K;
// the Z80 is stalled via wait_n for the extra read latency (ROM = PSRAM multi-cyc,
// WRAM = 1 cyc). The Sega mapper bank registers live in dedicated FFs (combinational
// for address mapping) and are mirrored into WRAM for read-back. vcounter + frame
// IRQ come from a free-running 228-cyc/line, 262-line counter (enough for Sonic's
// boot V==$B0 wait + per-frame vint). TODO_SMS_COMPAT: exact VDP timing, line IRQ,
// sprite zoom. cen=1 here (Z80 every clk); real SMS-rate pacing is M7.4.
module sms (
  input         CLK,
  input         RST,
  input         CE,            // SMS clock-enable (~3.58MHz). CE=1 = full-speed (sim).

  // ROM read interface to PSRAM (serviced by the arbiter/host)
  output reg        ROM_RRQ,
  output reg [23:0] ROM_ADDR,
  input      [7:0]  ROM_DATA,
  input             ROM_RDY,

  // SMS controller port $DC (active low; bit0..5 = U D L R B1 B2)
  input      [7:0]  PAD1,
  input             PAUSE,         // pulse: console Pause button -> Z80 NMI

  // translation read ports (M7.3) — registered (1-cycle)
  input      [13:0] TR_VRAM_ADDR,
  output reg [7:0]  TR_VRAM_DATA,
  input      [4:0]  TR_CRAM_ADDR,
  output     [7:0]  TR_CRAM_DATA,

  // debug / status
  output     [15:0] DBG_PC,
  output     [7:0]  DBG_BANK0,
  output     [7:0]  DBG_BANK1,
  output     [7:0]  DBG_BANK2,
  output reg [31:0] DBG_VRAM_WRITES,
  output reg [31:0] DBG_REG_WRITES,
  output reg [31:0] DBG_CRAM_WRITES,
  output     [7:0]  DBG_REG1,
  output     [7:0]  DBG_REG2,
  output            DBG_DISPLAY_ON,
  // for the translation engine + frame sync (M7.4 integration)
  output     [127:0] VREGS,        // all 16 VDP registers, flat (reg0=[7:0]..reg15=[127:120])
  output            FRAME_TICK,    // 1-cycle pulse at the start of ACTIVE display (line 0)
  output            ACTIVE_DISP,   // 1 while the VDP line counter is in the 192 active lines
  output            ACTIVE_LATE,   // 1 from the middle of the active lines on (display-off deadline)
  output reg [7:0]  DIRTY_LO_MIN,  // changed-tile range in the LOW half (tiles 0..255 = BG)
  output reg [7:0]  DIRTY_LO_MAX,  //   (empty range: MIN>MAX)
  output reg [7:0]  DIRTY_HI_MIN,  // changed-tile range in the HIGH half (tiles 256..511 = sprites),
  output reg [7:0]  DIRTY_HI_MAX,  //   index is within the half (absolute tile = 256+index)
  input             DIRTY_SNAP,    // pulse at translation start: publish live ranges + reset them
  input             BACK_SET,      // which double-buffer set is being written this pass (= ~sms_buf_front)
  output reg [31:0] DIRTY_COLS,    // per-set dirty name-table COLUMN bitmap (1 = re-emit that column)
  input             DBG_FORCE_FULL,// test: publish all 32 columns dirty (full tilemap emit)
  output [1:0]      DBG_WAIT,      // Z80 wait-stall live: bit0=ROM read, bit1=WRAM read
  // audio: unipolar PSG mix (0..1020) + the chip's internal 223.72kHz tick.
  // Amplitude conditioning for the DAC lives in sms_core.v (the wrap), not here.
  output reg        CM_MODE,       // Codemasters mapper detected from the ROM header
  // horizontal-scroll splits: reg8 writes inside the active lines of the frame before the pass,
  // up to four (line, value) pairs in write order (only writes that change the value)
  output reg [2:0]  RAST_N,
  output reg [31:0] RAST_LINE,
  output reg [31:0] RAST_VAL,
  output [10:0]     PSG_MIX,
  output            PSG_TICK
);

  // ---------------- Z80 ----------------
  wire        m1_n, mreq_n, iorq_n, rd_n, wr_n, rfsh_n, halt_n, busak_n;
  wire [15:0] A;
  wire [7:0]  dout;
  reg  [7:0]  di;
  reg         wait_n;
  wire        int_n;
  // ROM header probe (Codemasters mapper): after reset the Z80 is held until the four bytes at
  // $7FE6..$7FE9 have been read through the ROM port. word($7FE6) + word($7FE8) == $10000 with a
  // non-zero first word is the Codemasters checksum pair. The prefetch FIFO stays idle meanwhile.
  reg         det_done, det_busy, cm_set;
  reg  [1:0]  det_i;
  reg  [7:0]  det_b0, det_b1, det_b2;
  wire        det_issue = ~det_done & ~det_busy & ~RST;
  wire [16:0] det_sum = {1'b0, det_b1, det_b0} + {1'b0, ROM_DATA, det_b2};
  always @(posedge CLK) begin
    cm_set <= 1'b0;
    if (RST) begin
      det_done <= 1'b0; det_busy <= 1'b0; det_i <= 2'd0; CM_MODE <= 1'b0;
    end else if (~det_done) begin
      if (det_issue) det_busy <= 1'b1;
      if (det_busy & ROM_RDY) begin
        det_busy <= 1'b0; det_i <= det_i + 2'd1;
        case (det_i)
          2'd0: det_b0 <= ROM_DATA;
          2'd1: det_b1 <= ROM_DATA;
          2'd2: det_b2 <= ROM_DATA;
          default: begin
            det_done <= 1'b1;
            if (det_sum == 17'h10000 && {det_b1, det_b0} != 16'd0) begin CM_MODE <= 1'b1; cm_set <= 1'b1; end
          end
        endcase
      end
    end
  end
  // Pause button: stretch the pulse over a few CE ticks so the Z80 sees a falling edge
  reg [2:0]   pause_cnt;
  wire        nmi_n = (pause_cnt == 3'd0);
  always @(posedge CLK) begin
    if (RST) pause_cnt <= 3'd0;
    else if (PAUSE) pause_cnt <= 3'd5;
    else if (CE && pause_cnt != 3'd0) pause_cnt <= pause_cnt - 1'b1;
  end

  tv80s_ce #(.Mode(0)) cpu (
    .reset_n (~RST & det_done), .clk (CLK), .cen (CE), .wait_n (wait_n),
    .int_n (int_n), .nmi_n (nmi_n), .busrq_n (1'b1),
    .m1_n (m1_n), .mreq_n (mreq_n), .iorq_n (iorq_n), .rd_n (rd_n), .wr_n (wr_n),
    .rfsh_n (rfsh_n), .halt_n (halt_n), .busak_n (busak_n),
    .A (A), .di (di), .dout (dout)
  );

  // ---------------- memories (BRAM-inferred) ----------------
  reg [7:0] wram [0:8191];      // 8 KB WRAM (registered read)
  reg [7:0] vram [0:16383];     // 16 KB VDP VRAM (registered reads)
  reg [7:0] cram [0:31];        // 32 B CRAM (small -> logic)
  reg [7:0] vreg [0:15];        // VDP registers (logic)

  // Sega mapper bank registers in dedicated FFs (combinational for mapping)
  reg [7:0] bk_ctrl, bk0, bk1, bk2;
  assign DBG_BANK0 = bk0;
  assign DBG_BANK1 = bk1;
  assign DBG_BANK2 = bk2;
  assign DBG_REG1  = vreg[1];
  assign DBG_REG2  = vreg[2];
  assign DBG_DISPLAY_ON = vreg[1][6];
  assign TR_CRAM_DATA = cram[TR_CRAM_ADDR];

  // ---------------- free-running line/frame timing ----------------
  reg [8:0] line;
  reg [7:0] dot;
  reg       vblank_flag;
  reg       line_tick;
  reg       lint_set;          // 1-cycle pulse: the line counter underflowed on this line
  reg [7:0] lcnt;              // line-interrupt down counter (reloaded from reg10)
  wire [8:0] nline = (line == 9'd261) ? 9'd0 : (line + 1'b1);
  always @(posedge CLK) begin
    line_tick <= 1'b0;
    lint_set  <= 1'b0;
    if (RST) begin dot <= 0; line <= 0; lcnt <= 8'd0; end
    else if (CE) begin                       // line/dot advance at the SMS rate
      if (dot == 8'd227) begin
        dot <= 0;
        line_tick <= 1'b1;
        line <= nline;
        // VDP line counter: active on lines 0..192, reloaded on the rest
        if (nline <= 9'd192) begin
          if (lcnt == 8'd0) begin lcnt <= vreg[10]; lint_set <= 1'b1; end
          else lcnt <= lcnt - 1'b1;
        end else lcnt <= vreg[10];
      end else dot <= dot + 1'b1;
    end
  end
  reg lint_flag;               // line interrupt pending (cleared by a status read)
  wire [7:0] vcounter = (line <= 9'h0DA) ? line[7:0] : (line[7:0] - 8'd6);
  wire frame_start = line_tick && (line == 9'd191);
  assign int_n = ~((vblank_flag & vreg[1][5]) | (lint_flag & vreg[0][4]));
  // translation trigger = start of ACTIVE display (line wrap), NOT vblank start:
  // the game has just finished its vblank VDP writes -> the VDP state is a fresh,
  // consistent frame, and the pass runs while the game does (no CE freeze needed;
  // rare mid-pass writes self-heal via the per-set dirty accumulators next pass).
  assign FRAME_TICK = line_tick && (line == 9'd0);
  assign ACTIVE_DISP = (line < 9'd192);
  assign ACTIVE_LATE = (line >= 9'd96);
  assign VREGS = {vreg[15],vreg[14],vreg[13],vreg[12],vreg[11],vreg[10],vreg[9],vreg[8],
                  vreg[7],vreg[6],vreg[5],vreg[4],vreg[3],vreg[2],vreg[1],vreg[0]};

  // ---------------- address decode ----------------
  wire mem_rd = ~mreq_n & ~rd_n;
  wire io_rd  = ~iorq_n & ~rd_n;
  wire is_rom = A < 16'hC000;            // 0x0000-0xBFFF -> ROM (Sega mapper)
  // cartridge RAM: $FFFC bit3 maps an 8 KB cart RAM (both pages alias) over slot 2 ($8000-$BFFF)
  wire xram_sel = bk_ctrl[3] & (A[15:14] == 2'b10);
  wire rom_rd  = mem_rd & is_rom & ~xram_sel;
  wire wram_rd = mem_rd & ~is_rom;       // 0xC000-0xFFFF -> WRAM
  wire xram_rd = mem_rd & xram_sel;

  reg [23:0] map_addr;
  always @* begin
    if      (A < 16'h0400 && !CM_MODE) map_addr = {8'd0, A};
    // real hardware decodes only the low bank bits: ignore the high ones (32 banks = 512 KB)
    else if (A < 16'h4000) map_addr = {5'd0, bk0[4:0], A[13:0]};
    else if (A < 16'h8000) map_addr = {5'd0, bk1[4:0], A[13:0]};
    else                   map_addr = {5'd0, bk2[4:0], A[13:0]};
  end

  // ---------------- ROM read: streaming prefetch FIFO ----------------
  // Each PSRAM fetch via the SNES-shared arbiter has a long latency (free-slot wait +
  // access) -> a Z80 that fetches one-byte-at-a-time eats wait states -> ~half speed.
  // The arbiter THROUGHPUT (≈1 byte / SNES cycle) easily exceeds the Z80's read rate,
  // so we stream sequential ROM bytes AHEAD of the Z80 into a small FIFO: the Z80's
  // (mostly sequential) fetches then pop with zero wait. A jump / bank cross makes the
  // requested addr != the FIFO head addr -> flush and re-stream from the new address.
  localparam [2:0] PF_N = 3'd4;
  reg  [7:0]  pf_buf [0:3];
  reg  [1:0]  pf_head;
  reg  [2:0]  pf_count;
  reg  [23:0] pf_head_addr;     // mapped addr of pf_buf[pf_head]
  reg  [23:0] pf_next_addr;     // next mapped addr to prefetch (== head_addr + count)
  reg         pf_busy, pf_kill; // a prefetch is in flight / discard it (stale after a flush)
  reg         rom_have;
  reg  [7:0]  rom_lat;

  wire        pf_aligned = (pf_head_addr == map_addr);
  wire        ld_hit  = rom_rd & ~rom_have & pf_aligned & (pf_count != 3'd0);
  wire        ld_miss = rom_rd & ~rom_have & ~pf_aligned;     // jump / bank cross
  wire        pf_done = pf_busy & ROM_RDY;
  wire        do_push = pf_done & ~pf_kill & ~ld_miss;        // commit a fetched byte
  wire [1:0]  pf_tail = pf_head + pf_count[1:0];
  wire [2:0]  cnt_aft = ld_miss ? 3'd0
                       : (pf_count + (do_push?3'd1:3'd0) - (ld_hit?3'd1:3'd0));
  wire [23:0] nxt_aft = ld_miss ? map_addr
                       : (do_push ? pf_next_addr + 24'd1 : pf_next_addr);
  wire        pf_free = ~pf_busy | pf_done;
  wire        do_issue = pf_free & (cnt_aft < PF_N) & det_done;

  always @(posedge CLK) begin
    if (RST) begin
      pf_head <= 2'd0; pf_count <= 3'd0; pf_head_addr <= 24'd0; pf_next_addr <= 24'd0;
      pf_busy <= 1'b0; pf_kill <= 1'b0; rom_have <= 1'b0; ROM_RRQ <= 1'b0;
    end else begin
      ROM_RRQ <= 1'b0;
      if (~rom_rd) rom_have <= 1'b0;                  // Z80 done -> arm the next read
      if (ld_hit) begin rom_lat <= pf_buf[pf_head]; rom_have <= 1'b1; end
      if (do_push) pf_buf[pf_tail] <= ROM_DATA;
      if (ld_miss) begin                              // flush + re-stream from map_addr
        pf_count <= 3'd0; pf_head <= 2'd0;
        pf_head_addr <= map_addr; pf_next_addr <= map_addr;
        if (pf_busy & ~ROM_RDY) pf_kill <= 1'b1;      // the in-flight fetch is now stale
      end else begin
        if (ld_hit) begin pf_head <= pf_head + 2'd1; pf_head_addr <= pf_head_addr + 24'd1; end
        pf_count <= pf_count + (do_push?3'd1:3'd0) - (ld_hit?3'd1:3'd0);
        if (do_push) pf_next_addr <= pf_next_addr + 24'd1;
      end
      if (pf_done) begin pf_busy <= 1'b0; pf_kill <= 1'b0; end
      if (det_issue) begin ROM_RRQ <= 1'b1; ROM_ADDR <= 24'h007FE6 + {22'd0, det_i}; end
      if (do_issue) begin ROM_RRQ <= 1'b1; ROM_ADDR <= nxt_aft; pf_busy <= 1'b1; end
    end
  end

  // ---------------- WRAM: clean single-port BRAM template ----------------
  // addr = A[12:0]; write on wram_we (CPU store to 0xC000-0xFFFF); read every cyc.
  wire wram_we = wr_edge & iorq_n & ~mreq_n & (A >= 16'hC000);  // memory write (iorq_n HIGH)
  reg [7:0] wram_q;
  reg       wram_rd_d;
  always @(posedge CLK) begin
    if (wram_we) wram[A[12:0]] <= dout;
    wram_q <= wram[A[12:0]];
    wram_rd_d <= wram_rd;
  end
  wire wram_have = wram_rd_d;

  // ---------------- cartridge RAM (8 KB, $FFFC bit3) ----------------
  reg [7:0] xram [0:8191];      // 8 KB: 16 KB exceeds the M9K budget, both pages alias
  wire xram_we = wr_edge & iorq_n & ~mreq_n & xram_sel;
  wire [12:0] xram_a = A[12:0];
  reg [7:0] xram_q;
  reg       xram_rd_d;
  always @(posedge CLK) begin
    if (xram_we) xram[xram_a] <= dout;
    xram_q <= xram[xram_a];
    xram_rd_d <= xram_rd;
  end
  wire xram_have = xram_rd_d;

  // ---------------- VRAM: clean true-dual-port BRAM template ----------------
  // Port A = CPU at vdp_addr (R/W); Port B = translation read at TR_VRAM_ADDR.
  // vram_q (port A read) feeds the VDP read buffer. vram_we = data-port write,
  // non-CRAM. (vdp_addr/vdp_code are maintained in the VDP block below.)
  wire vram_we = wr_edge & ~iorq_n & (A[7:6] == 2'b10) & ~A[0] & (vdp_code != 2'b11);
  reg [7:0] vram_q;
  always @(posedge CLK) begin
    if (vram_we) vram[vdp_addr] <= dout;
    vram_q       <= vram[vdp_addr];
    TR_VRAM_DATA <= vram[TR_VRAM_ADDR];
  end

  // dirty-tile RANGES (two halves) and name-table COLUMNS, ONE accumulator each. A pattern-
  // table write extends the live range of its half (SMS BG tiles sit in the LOW half 0..255,
  // sprite tiles in the HIGH half 256..511, reg6 bit2 -> two narrow spans). DIRTY_SNAP (the
  // pass START) publishes the accumulator and re-arms it; a write coinciding with the snap
  // lands in the freshly re-armed range, so it is never lost. Each published range is the
  // set of tiles changed since the PREVIOUS pass: the translation re-reads every tile of the
  // span from the live VRAM, so the buffer holds current data over the whole span and the
  // player uploads each pass exactly once (tr_go waits for ~tr_done and the ready/swap
  // handshake gates the next pass), so no change can slip between two uploads. (The old
  // two-set union existed because a lost pass could skip an upload.)
  wire        m224_w    = vreg[1][4] & vreg[0][1] & ~vreg[1][3];   // 224-line mode: name table at $x700
  wire [13:0] nt_base_w = m224_w ? {vreg[2][3:2], 12'h700} : {vreg[2][3:1], 11'd0};
  wire [8:0]  wtile = vdp_addr[13:5];        // SMS tile = byte/32 (0..511)
  wire        whalf = wtile[8];              // 0 = low half (BG), 1 = high half (sprites)
  wire [7:0]  widx  = wtile[7:0];            // index within the half
  // A write of the byte the VRAM already holds changes nothing on the SNES side: games
  // (Sonic) rewrite whole name-table columns and tiles every frame with identical data, and
  // counting those as dirty made every pass re-upload all 32 columns. vram_q is the port-A
  // read of vdp_addr, settled long before the Z80's next I/O strobe, so at vram_we it holds
  // the OLD byte.
  wire        vchg    = (dout != vram_q);
  wire        wr_tile = vram_we & vchg & (vdp_addr < nt_base_w);
  reg  [7:0]  lo_min, lo_max, hi_min, hi_max;       // live accumulator
  // The qualifiers are registered one clock before they touch the accumulators (the Z80
  // strobe is single-cycle and the I/O is ~260 clocks apart, so the delay is invisible; it
  // keeps the compare + range-extend out of one long path).
  reg         wlo, whi;                             // low/high-half tile write (delayed)
  reg  [7:0]  widx_r;
  always @(posedge CLK) begin
    wlo <= wr_tile & ~whalf; whi <= wr_tile & whalf; widx_r <= widx;
  end
  always @(posedge CLK) begin
    if (RST) begin
      DIRTY_LO_MIN <= 8'd0; DIRTY_LO_MAX <= 8'd255;   // force a full first upload, both halves
      DIRTY_HI_MIN <= 8'd0; DIRTY_HI_MAX <= 8'd255;
      lo_min <= 8'd0; lo_max <= 8'd255; hi_min <= 8'd0; hi_max <= 8'd255;
    end else begin
      if (DIRTY_SNAP) begin
        DIRTY_LO_MIN <= lo_min; DIRTY_LO_MAX <= lo_max;
        DIRTY_HI_MIN <= hi_min; DIRTY_HI_MAX <= hi_max;
      end
      // a write extends; snap re-arms; snap+write -> freshly armed to widx
      if (wlo) lo_min <= (DIRTY_SNAP | (widx_r < lo_min)) ? widx_r : lo_min; else if (DIRTY_SNAP) lo_min <= 8'd255;
      if (wlo) lo_max <= (DIRTY_SNAP | (widx_r > lo_max)) ? widx_r : lo_max; else if (DIRTY_SNAP) lo_max <= 8'd0;
      if (whi) hi_min <= (DIRTY_SNAP | (widx_r < hi_min)) ? widx_r : hi_min; else if (DIRTY_SNAP) hi_min <= 8'd255;
      if (whi) hi_max <= (DIRTY_SNAP | (widx_r > hi_max)) ? widx_r : hi_max; else if (DIRTY_SNAP) hi_max <= 8'd0;
    end
  end

  // dirty name-table COLUMN bitmap: a name-table write dirties its cell's column; a coarse
  // V-scroll change (reg9[7:3]) or a name-table base change shifts the cells of EVERY column
  // -> dirty all 32. The translate FSM re-emits only the dirty columns.
  wire        wr_nt = vram_we & vchg & (vdp_addr >= nt_base_w) & (vdp_addr < (nt_base_w + (m224_w ? 14'h800 : 14'h700)));
  wire [4:0]  wcol  = vdp_addr[5:1];                   // column (nt_base is 0x800-aligned)
  reg  [4:0]  reg9row_p;
  reg  [2:0]  reg2nt_p;
  reg         m224_p;
  wire        reg9_chg = (vreg[9][7:3] != reg9row_p) | (vreg[2][3:1] != reg2nt_p) | (m224_w != m224_p);
  reg         wr_nt_r;  reg [4:0] wcol_r;           // registered like wlo/whi above
  always @(posedge CLK) begin wr_nt_r <= wr_nt; wcol_r <= wcol; end
  wire [31:0] col_set  = (wr_nt_r ? (32'd1 << wcol_r) : 32'd0) | (reg9_chg ? 32'hFFFFFFFF : 32'd0);
  reg  [31:0] cols;
  always @(posedge CLK) begin
    if (RST) begin
      DIRTY_COLS <= 32'hFFFFFFFF; cols <= 32'hFFFFFFFF; reg9row_p <= 5'd0; reg2nt_p <= 3'd0; m224_p <= 1'b0;
    end else begin
      reg9row_p <= vreg[9][7:3]; reg2nt_p <= vreg[2][3:1]; m224_p <= m224_w;
      if (DIRTY_SNAP) DIRTY_COLS <= DBG_FORCE_FULL ? 32'hFFFFFFFF : cols;
      cols <= (DIRTY_SNAP ? 32'd0 : cols) | col_set;  // accumulate; publish+clear on the snap
    end
  end

  // ---------------- VDP port state ----------------
  reg [13:0] vdp_addr;
  reg [1:0]  vdp_code;
  reg [7:0]  vdp_latch_lo;
  reg        vdp_latch;
  reg [7:0]  vdp_readbuf;
  reg        rd_vdp_p, rd_vdp_a0_p;   // VDP port read in flight (latched decode)
  reg [1:0]  pref_cnt;                // read-setup prefetch pipeline

  // I/O read data (combinational; side effects on strobes below)
  // $3F I/O control: TH pins read back the written level when configured as outputs
  // (the region check of the games writes TH and reads $DD to tell export from Japan).
  reg [7:0] io_ctrl;
  wire th_a = io_ctrl[1] | io_ctrl[5];
  wire th_b = io_ctrl[3] | io_ctrl[7];
  wire [7:0] hcnt = ({1'b0, dot[7:1]} + {2'b0, dot[7:2]});  // ~0.75 * dot = 0..170 (H counter approx)
  reg [7:0] io_data;
  always @* begin
    case (A[7:6])
      2'b01: io_data = A[0] ? hcnt : vcounter;
      2'b10: io_data = A[0] ? {vblank_flag, 7'h00} : vdp_readbuf;
      2'b11: io_data = A[0] ? {th_b, th_a, 6'h3F} : PAD1;
      default: io_data = 8'hFF;
    endcase
  end

  // di mux + wait
  always @* begin
    if (~iorq_n & ~m1_n) di = 8'hFF;       // interrupt acknowledge: open bus reads $FF (IM 0 executes RST 38)
    else if (io_rd)   di = io_data;
    else if (xram_sel) di = xram_q;
    else if (is_rom)  di = rom_lat;
    else              di = wram_q;
  end
  always @* wait_n = ((rom_rd & ~rom_have) | (wram_rd & ~wram_have) | (xram_rd & ~xram_have)) ? 1'b0 : 1'b1;
  assign DBG_WAIT = {(wram_rd & ~wram_have), (rom_rd & ~rom_have)};

  // ---------------- bus strobes ----------------
  reg rd_a_d, wr_a_d;
  wire rd_edge = (~rd_n) & ~rd_a_d;
  wire wr_edge = (~wr_n) & ~wr_a_d;
  always @(posedge CLK) begin rd_a_d <= ~rd_n; wr_a_d <= ~wr_n; end

  assign DBG_PC = A;

  // ---------------- PSG (SN76489) ----------------
  // The SMS decodes ANY I/O write in $40-$7F to the sound chip. Same
  // wr_edge + iorq_n qualification the VDP/WRAM write templates use.
  wire psg_we = wr_edge & ~iorq_n & (A[7:6] == 2'b01);

  // ---------------- reg8 split recorder ----------------
  // live = the frame being drawn, prev = the frame before (latched at the line-0 tick),
  // RAST_* = prev latched at the translation start so the pass sees a stable copy.
  wire        reg8_wr = wr_edge & ~iorq_n & (A[7:6] == 2'b10) & A[0] & vdp_latch &
                        (dout[7:6] == 2'b10) & (dout[3:0] == 4'd8);
  reg [2:0]   rl_n, rp_n;
  reg [31:0]  rl_line, rl_val, rp_line, rp_val;
  wire [7:0]  rl_last_line = rl_line[{rl_n[1:0] - 2'd1, 3'd0} +: 8];
  wire        rl_same = (rl_n != 3'd0) && (rl_last_line == line[7:0]);
  always @(posedge CLK) begin
    if (RST) begin rl_n <= 0; rp_n <= 0; RAST_N <= 0; rl_line <= 0; rl_val <= 0; rp_line <= 0; rp_val <= 0; RAST_LINE <= 0; RAST_VAL <= 0; end
    else begin
      if (DIRTY_SNAP) begin RAST_N <= rp_n; RAST_LINE <= rp_line; RAST_VAL <= rp_val; end
      if (FRAME_TICK) begin rp_n <= rl_n; rp_line <= rl_line; rp_val <= rl_val; rl_n <= 3'd0; end
      else if (reg8_wr && line < (m224_w ? 9'd207 : 9'd191) && vdp_latch_lo != vreg[8]) begin
        if (rl_same) rl_val[{rl_n[1:0] - 2'd1, 3'd0} +: 8] <= vdp_latch_lo;
        else if (rl_n < 3'd4) begin
          rl_line[{rl_n[1:0], 3'd0} +: 8] <= line[7:0];
          rl_val[{rl_n[1:0], 3'd0} +: 8]  <= vdp_latch_lo;
          rl_n <= rl_n + 1'b1;
        end
      end
    end
  end

  psg u_psg (
    .CLK(CLK), .RST(RST), .CE(CE),
    .WE(psg_we), .D(dout),
    .MIX(PSG_MIX), .TICK(PSG_TICK)
  );

  // ---------------- VDP/WRAM writes, mapper, VDP read side effects ----------------
  always @(posedge CLK) begin
    if (RST) begin
      vdp_addr <= 0; vdp_code <= 0; vdp_latch <= 0; vdp_latch_lo <= 0; vdp_readbuf <= 0;
      vblank_flag <= 0; lint_flag <= 0; io_ctrl <= 8'hFF; rd_vdp_p <= 0; rd_vdp_a0_p <= 0; pref_cnt <= 0;
      DBG_VRAM_WRITES <= 0; DBG_REG_WRITES <= 0; DBG_CRAM_WRITES <= 0;
      bk_ctrl <= 8'h00; bk0 <= 8'h00; bk1 <= 8'h01; bk2 <= 8'h02;  // Sega mapper power-on
    end else begin
      if (cm_set) bk2 <= 8'h00;                   // Codemasters power-on banks 0,1,0
      if (frame_start) vblank_flag <= 1'b1;
      if (lint_set) lint_flag <= 1'b1;
      if (pref_cnt != 2'd0) begin
        pref_cnt <= pref_cnt - 1'b1;
        if (pref_cnt == 2'd1) begin vdp_readbuf <= vram_q; vdp_addr <= vdp_addr + 1'b1; end   // read-setup prefetch
      end

      // writes (rising edge of the write strobe)
      if (wr_edge) begin
        if (~iorq_n) begin
          if (A[7:6] == 2'b10) begin
            if (A[0]) begin
              // $BF control port
              if (!vdp_latch) begin vdp_latch_lo <= dout; vdp_latch <= 1'b1; end
              else begin
                vdp_latch <= 1'b0;
                vdp_code  <= dout[7:6];
                vdp_addr  <= {dout[5:0], vdp_latch_lo};
                if (dout[7:6] == 2'b10) begin vreg[dout[3:0]] <= vdp_latch_lo; DBG_REG_WRITES <= DBG_REG_WRITES + 1; end
                else if (dout[7:6] == 2'b00) begin pref_cnt <= 2'd2; end  // VRAM read setup: prefetch the byte
              end
            end else begin
              // $BE data port (VRAM/CRAM write handled by the RAM templates; count + advance here)
              vdp_latch <= 1'b0;
              if (vdp_code == 2'b11) begin cram[vdp_addr[4:0]] <= dout; DBG_CRAM_WRITES <= DBG_CRAM_WRITES + 1; end
              else                  DBG_VRAM_WRITES <= DBG_VRAM_WRITES + 1;
              vdp_readbuf <= dout;
              vdp_addr <= vdp_addr + 1'b1;
            end
          end
          // $40-$7F -> PSG (handled by u_psg via psg_we; nothing to do here)
          if (A[7:6] == 2'b00 && A[0]) io_ctrl <= dout;      // $3F I/O port control
        end else if (~mreq_n) begin
          // memory write: WRAM data goes via the wram template (wram_we); here we
          // only latch the Sega mapper bank regs into their dedicated FFs.
          if (CM_MODE) begin
            // Codemasters: a write to $0000/$4000/$8000 selects the bank of that slot; no $FFFC-$FFFF
            if (A[13:0] == 14'd0 && A[15:14] != 2'b11) begin
              if (A[15:14] == 2'b00) bk0 <= dout;
              if (A[15:14] == 2'b01) bk1 <= dout;
              if (A[15:14] == 2'b10) bk2 <= dout;
            end
          end else if (A >= 16'hC000) begin
            case (A[15:0])
              16'hFFFC: bk_ctrl <= dout;
              16'hFFFD: bk0 <= dout;
              16'hFFFE: bk1 <= dout;
              16'hFFFF: bk2 <= dout;
              default: ;
            endcase
          end
        end
      end

      // VDP read side effects. The Z80 samples the data bus at the END of the read strobe, so
      // the effects (flag clear, read-buffer reload, address step) apply once the strobe has
      // ended, from the decode latched while it was active; applying them at its start made
      // the game read the already-cleared flag / the next buffer byte.
      if (~rd_n & ~iorq_n & (A[7:6] == 2'b10)) begin
        rd_vdp_p <= 1'b1; rd_vdp_a0_p <= A[0];
      end else if (rd_n & rd_vdp_p) begin
        rd_vdp_p <= 1'b0;
        vdp_latch <= 1'b0;
        if (rd_vdp_a0_p) begin vblank_flag <= 1'b0; lint_flag <= 1'b0; end
        else begin vdp_readbuf <= vram_q; vdp_addr <= vdp_addr + 1'b1; end
      end
    end
  end

endmodule
