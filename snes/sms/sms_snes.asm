; sd2snes SMS core (M7.4b) — SNES-side player for the REAL FPGA core, LoROM.
;
; Booted by the firmware (FPGA_SMS) as the cart ROM, in place of the .sms itself
; (sms_update_file swaps the filename; the .sms is staged separately to PSRAM
; 0x300000). Each vblank it DMAs the FPGA-translated SMS frame from the $E0-$E3
; buffer banks (PSRAM 0x380000, mapped by address.v) into VRAM/CGRAM/OAM, so the
; SNES PPU draws the live SMS frame; it also forwards the pad to $EF and swaps the
; FPGA's double buffer.
;
; ASSEMBLER: asar (NOT snescom/sneslink like snes/ and snes/nes/) -- see the
; Makefile next to this file. Built into misc/sms_snes.bin by build.sh.
;
; IMPORTANT -- this player initialises only what it needs (BGMODE, BG1/OBJ bases,
; TM, scroll). Everything else in $21xx it INHERITS from whatever the menu left
; behind, because the console reset does not clear that block. That is why
; iris_out (snes/iris.a65) must hand over a disarmed PPU: a stale colour window or
; a stale $2133 pseudo-hires bit lands here as a broken/blurry picture.
;
; Buffer banks (match address.v SMS_BUF_HIT):
;   $E0 tiles(16KB) $E1 tilemap(2KB) $E2 cgram(512) $E3 oam(544)

!TILES_BANK   = $E0
!TILEMAP_BANK = $E1
!CGRAM_BANK   = $E2
!OAM_BANK     = $E3
!BG1_CHAR = $0000
!BG1_MAP  = $2000
; BG2 = the underlay for palette-1 background cells: the SMS shows CRAM[16] where a
; palette-1 tile has colour 0, the SNES draws colour 0 transparent (the backdrop,
; CRAM[0], would show). BG2 holds a solid tile (tile 1, colour 1 of palette 2 =
; CGRAM[33] = CRAM[16]) under every such cell, in low priority: BG1 and the OBJ
; stay on top of it exactly as on the SMS.  Map high byte is constant $08 (palette
; 2), the FPGA supplies one low byte per cell (0 empty / 1 solid) at $E3:1000.
!BG2_CHAR = $3000
!BG2_MAP  = $2400

lorom

org $008000
Reset:
    sei
    clc
    xce
    rep #$38
    ldx.w #$1FFF
    txs
    lda.w #$0000
    tcd
    sep #$20

    lda.b #$01
    sta $04             ; IGR edge latch, born ARMED: a combo still held
                        ; through the $80 reset must be released before it
                        ; can fire again (see the IGR block in the NMI)

    stz $12             ; DP $12 = 0: no front uploaded yet (the 1st NMI uploads for sure)
    lda.b #$80
    sta $1A             ; DP $1A = OAM entries on screen: unknown, so the 1st upload sends all 128
    lda.b #$8F
    sta $2100
    stz $4200
    stz $420C
    stz $210D           ; BG1HOFS = 0
    stz $210D
    stz $210E           ; BG1VOFS = 0
    stz $210E

    lda.b #$01
    sta $2105           ; BGMODE 1 (BG1 4bpp)
    lda.b #$20
    sta $2107           ; BG1SC: map base word $2000, 32x32
    lda.b #$24
    sta $2108           ; BG2SC: map base word $2400, 32x32
    lda.b #$30
    sta $210B           ; BG1 char base word $0000, BG2 char base word $3000
    stz $210F           ; BG2 scroll 0
    stz $210F
    stz $2110
    stz $2110
    stz $20             ; DP $20..$23 = BG2 columns whose VRAM map holds solid cells
    stz $21
    stz $22
    stz $23
    lda.b #$00
    sta $2101           ; OBSEL: OBJ char base $0000
    stz $212C           ; TM off until the first frame (the IRQ opens it, see NMI/IRQ)
    lda.b #$13
    sta $06             ; DP $06 = TM value while the picture is on (0 = VDP display off)
    stz $05             ; DP $05 = IRQ phase (0 = next IRQ opens the picture at line 16,
                        ; 1 = closes at 208, 2 = writes the game HOFS at 32 = scroll lock)
    stz $1F             ; DP $1F = scroll lock (VDP reg0 bit6): top 2 tile rows keep HOFS 0
    lda.b #$10
    sta $4209           ; VTIME = 16: the IRQ fires in the hblank of line 16 (last border row)
    stz $420A
    lda.b #$14
    sta $4207           ; HTIME = $114 = dot 276, inside the hblank
    lda.b #$01
    sta $4208
    lda.b #$80
    sta $4201           ; WRIO bit7 high: $2137 can latch the V counter

    ; borders: window 1 = x 0..7 is the SMS left-column mask (reg0 bit5,
    ; switched per frame through $212E in the NMI); the V-IRQ pair blanks the
    ; lines outside the 192 SMS lines (no HDMA: it breaks the NMI DMAs here).
    ; The menu leaves $21xx behind, so set every window/colour-math register.
    stz $2123
    stz $2124
    stz $2125
    stz $212A
    stz $212B
    stz $212E
    stz $212F
    stz $2130
    stz $2131
    stz $2126           ; W1 left  = 0
    lda.b #$07
    sta $2127           ; W1 right = 7
    lda.b #$22
    sta $2123           ; BG1 + BG2: window 1 enabled (inside = masked)
    sta $2125           ; OBJ: window 1 enabled
    ; backdrop colour: the core keeps CGRAM[0] = CRAM[0] (an opaque BG colour 0),
    ; so the real backdrop (reg7) is painted only where the SMS shows it (masked
    ; column, border lines): colour window 1 clips to black there and the fixed
    ; colour $2132 is added to the backdrop layer. DP $08 = $2130 / DP $09 = W1
    ; right edge while the picture is on; the border band always uses $90 / $FF.
    lda.b #$22
    sta $2125           ; OBJ + colour window: window 1 enabled
    lda.b #$20
    sta $2131           ; colour math on the backdrop only, add
    lda.b #$90
    sta $2130           ; clip to black inside W1, math only inside W1
    lda.b #$FF
    sta $2127           ; border band: W1 = the whole line
    lda.b #$30
    sta $08             ; picture band, no mask: no colour math at all
    lda.b #$07
    sta $09

    ; BG2 VRAM (screen is in forced blank): tile 0 empty, tile 1 solid colour 1,
    ; map = tile 0 / palette 2 everywhere (word $0800)
    lda.b #$80
    sta $2115
    ldx.w #!BG2_CHAR
    stx $2116
    ldx.w #16
Bg2InitT0:
    stz $2118
    stz $2119
    dex
    bne Bg2InitT0
    ldx.w #8
    lda.b #$FF
Bg2InitT1:
    sta $2118           ; bitplane 0 = $FF, plane 1 = 0
    stz $2119
    dex
    bne Bg2InitT1
    ldx.w #8
Bg2InitT1b:
    stz $2118           ; planes 2/3 = 0
    stz $2119
    dex
    bne Bg2InitT1b
    ldx.w #!BG2_MAP
    stx $2116
    ldx.w #$0400
    lda.b #$08
Bg2InitMap:
    stz $2118
    sta $2119
    dex
    bne Bg2InitMap

    jsr SmsApuUnmute    ; open the console's cart-DAC audio path

    lda.b #$B1
    sta $4200           ; NMI + H/V-IRQ + auto-joypad
    lda.b #$0F
    sta $2100
    cli

MainLoop:
    wai
    bra MainLoop

NMI:
    rep #$30
    pha
    phx
    phy
    sep #$20

    ; Same front as the one already in VRAM/CGRAM/OAM (the pass serial at $E2:0210
    ; did not move: swap was empty)? Then there is nothing to upload and the
    ; screen never needs to blank.  DP $10/$11 = serial uploaded, $12 = valid.
    lda $12
    cmp.b #$09
    bcc NeedUpload
    rep #$20
    lda.l $E20210
    cmp $10
    sep #$20
    bne NeedUpload
    jmp SkipUpload
NeedUpload:
    rep #$20
    lda.l $E20210
    sta $10
    sep #$20
    lda $12             ; upload counter: 1..8 = full columns, 9 = trimmed from now on
    cmp.b #$09
    bcs UpCntDone
    inc $12
UpCntDone:

    lda.b #$8F
    sta $2100           ; force blank for DMA

    ; CGRAM: BG 64 B ($E2:0000 -> CGRAM 0) + OBJ 32 B ($E2:0100 -> CGRAM 128)
    stz $2121
    lda.b #$00
    sta $4300
    lda.b #$22
    sta $4301
    ldx.w #$0000
    stx $4302
    lda.b #!CGRAM_BANK
    sta $4304
    ldx.w #$0040
    stx $4305
    lda.b #$01
    sta $420B
    lda.b #$80
    sta $2121
    ldx.w #$0100
    stx $4302
    ldx.w #$0020
    stx $4305
    lda.b #$01
    sta $420B
    lda.b #33
    sta $2121           ; CGRAM[33] = palette 2 colour 1 = CRAM[16] (BG2 solid cell)
    ldx.w #$0100
    stx $4302
    ldx.w #$0002
    stx $4305
    lda.b #$01
    sta $420B

    ; VMAIN +1 word (used by both the tiles and tilemap DMA)
    lda.b #$80
    sta $2115

    ; tiles: two changed-tile spans -> BG (low half 0..255) + sprites (high half 256..511).
    ; A single [min..max] spans both halves whenever BG and sprite tiles both change in a
    ; frame, re-uploading the whole 16KB -> vblank overrun that grows with sprite count.
    ; FPGA emits each half's range at $E2:0204 lo_min / 0205 lo_max / 0206 hi_min /
    ; 0207 hi_max (8-bit; hi is an index, absolute tile = 256+idx). carry-clear = empty.
    sep #$20
    lda.l $E20205            ; lo_max
    sec
    sbc.l $E20204            ; lo_max - lo_min ; carry clear -> empty
    bcc NoLoTiles
    rep #$20
    and.w #$00FF
    inc a                    ; count
    tay
    lda.l $E20204
    and.w #$00FF             ; abs tile = lo_min
    tax
    jsr DmaTileSpan
NoLoTiles:
    sep #$20
    lda.l $E20207            ; hi_max
    sec
    sbc.l $E20206            ; hi_max - hi_min ; carry clear -> empty
    bcc NoHiTiles
    rep #$20
    and.w #$00FF
    inc a                    ; count
    tay
    lda.l $E20206
    and.w #$00FF
    clc
    adc.w #$0100             ; abs tile = 256 + hi_min
    tax
    jsr DmaTileSpan
NoHiTiles:
    sep #$20

    ; tilemap: per-column vertical-increment DMA of only the DIRTY columns. The FPGA emits the
    ; tilemap COLUMN-MAJOR (column c = 64 bytes at $E1:(c*64)) and publishes a 32-bit dirty-column
    ; bitmap at $E2:0208-020B. $2115=$81 (+32-word increment) lays a column down the screen; clean
    ; columns keep their VRAM value (the FPGA didn't re-emit them) -> the 2KB DMA collapses to the
    ; few changed columns, cutting both the FPGA write AND this read out of the PSRAM contention.
    lda.b #$81
    sta $2115                ; VMAIN: +32-word vertical increment (column-down)
    lda.b #$01
    sta $4300                ; DMA mode 1 ($2118/$2119)
    lda.b #$18
    sta $4301
    lda.b #!TILEMAP_BANK
    sta $4304
    rep #$20
    lda.l $E20208
    sta $00                  ; dirty columns 0..15
    lda.l $E2020A
    sta $02                  ; dirty columns 16..31
    ; BG2 upload mask: U = D & (H | nz) -> $24/$26, nz = (nz & ~D) | (H & D), where D = the
    ; dirty columns, H = columns holding a palette-1 cell ($E2:0230) and nz = the columns
    ; whose BG2 map is not all-empty in VRAM (so a column that lost its cells is cleared).
    lda.l $E20230
    sta $28
    lda.l $E20232
    sta $2A
    lda $28
    ora $20
    and $00
    sta $24
    lda $2A
    ora $22
    and $02
    sta $26
    lda $00
    eor.w #$FFFF
    and $20
    sta $20
    lda $28
    and $00
    ora $20
    sta $20
    lda $02
    eor.w #$FFFF
    and $22
    sta $22
    lda $2A
    and $02
    ora $22
    sta $22
    sep #$20
    ; Rows 0,1 and 27..31 of every column are constant blank cells (the FPGA only
    ; builds content in rows 2..26), so after the first uploads (which also lay down
    ; the blank rows) a column is 25 cells = 50 B from row 2 instead of 64 B.
    lda $12
    cmp.b #$09
    rep #$20
    bcs TmTrim
    stz $14                  ; DP $14 = column source offset in $E1 (+64 per column)
    lda.w #!BG1_MAP
    sta $16                  ; DP $16 = VRAM word dest (+1 per column)
    lda.w #$0040
    sta $18                  ; DP $18 = bytes per column
    bra TmStart
TmTrim:
    lda $00
    and $02
    cmp.w #$FFFF
    bne TmTrimGen
    jmp TmFull
TmTrimGen:
    lda.w #$0004
    sta $14                  ; row 2 = byte 4
    lda.w #!BG1_MAP+64
    sta $16                  ; row 2 = +64 words
    lda.w #$0032
    sta $18
TmStart:
    ldx.w #$0000             ; X = column 0
TmCol:                       ; A 16-bit here
    lda $00
    ora $02
    beq TmDone               ; nothing left dirty
    txa
    and.w #$0007
    bne TmBit
    sep #$20
    lda $00
    bne TmBit8               ; low byte has a dirty column
    lda $01                  ; 8 clean columns: shift the bitmap by one byte
    sta $00
    lda $02
    sta $01
    lda $03
    sta $02
    stz $03
    rep #$20
    txa
    clc
    adc.w #$0008
    tax
    lda $14
    clc
    adc.w #$0200
    sta $14
    lda $16
    clc
    adc.w #$0008
    sta $16
    bra TmCol
TmBit8:
    rep #$20
TmBit:
    lsr $02                  ; 32-bit bitmap >> 1: column X -> carry
    ror $00
    bcc TmColSkip            ; clean column -> skip DMA
    lda $16
    sta $2116                ; VRAM dest = BG1_MAP + X (column start)
    lda $14
    sta $4302
    lda $18
    sta $4305                ; column length, reloaded PER column: $4305 decrements to 0
    lda.w #$0001
    sta $420B                ; trigger DMA ch0 ($420C = 0 alongside: no HDMA here)
TmColSkip:
    lda $14
    clc
    adc.w #$0040
    sta $14
    inc $16
    inx
    cpx.w #$0020             ; 32 columns
    bne TmCol
TmDone:
    sep #$20

    ; BG2 underlay: the 25 content rows of each column in $24/$26, one low byte per
    ; cell ($E3:1000 + col*32 + row), mode 0 DMA to $2118 with the +32 step after the
    ; low byte so the constant palette byte ($08) of the map stays untouched.
    rep #$20
    lda $24
    ora $26
    beq Bg2Done
    sep #$20
    lda.b #$01
    sta $2115
    lda.b #$00
    sta $4300
    lda.b #$18
    sta $4301
    lda.b #!OAM_BANK
    sta $4304
    rep #$20
    lda.w #$1002
    sta $14             ; source: row 2 of column 0
    lda.w #!BG2_MAP+64
    sta $16             ; VRAM word: row 2 of column 0
Bg2Col:
    lda $24
    ora $26
    beq Bg2Done
    lsr $26
    ror $24
    bcc Bg2Skip
    lda $16
    sta $2116
    lda $14
    sta $4302
    lda.w #25
    sta $4305
    lda.w #$0001
    sta $420B
Bg2Skip:
    lda $14
    clc
    adc.w #$0020
    sta $14
    inc $16
    bra Bg2Col
Bg2Done:
    sep #$20

    ; OAM: only the entries this pass used or the previous one used (the buffer hides
    ; the rest; everything past max(cur,last) is already hidden on the SNES), then the
    ; 32-byte high table. DP $1A = entries sent last time ($E2:022E = this pass).
    lda.l $E2022E
    cmp.b #$81
    bcc OamCntOk        ; an out-of-range count (older core) sends everything
    lda.b #$80
OamCntOk:
    sta $1B
    cmp $1A
    bcs OamMaxSet
    lda $1A
OamMaxSet:
    sta $1C             ; entries to send
    lda $1B
    sta $1A
    stz $2102
    stz $2103
    lda.b #$00
    sta $4300
    lda.b #$04
    sta $4301
    ldx.w #$0000
    stx $4302
    lda.b #!OAM_BANK
    sta $4304
    lda $1C
    beq OamHigh
    rep #$20
    and.w #$00FF
    asl
    asl
    sta $4305
    sep #$20
    lda.b #$01
    sta $420B
OamHigh:
    stz $2102
    lda.b #$01
    sta $2103           ; OAM word address $100 = the high table
    ldx.w #$0200
    stx $4302
    ldx.w #$0020
    stx $4305
    lda.b #$01
    sta $420B

    lda.b #$0F
    sta $2100           ; screen on right after the last DMA (the rest is plain registers)
SkipUpload:
    ; --- scroll: read hofs/vofs from $E2:0200 (FPGA-emitted) -> BG1 scroll ---
    lda.l $E20200
    sta $1D             ; DP $1D/$1E = the game's HOFS
    lda.l $E20201
    sta $1E
    lda.l $E2020C
    and.b #$40
    sta $1F             ; scroll lock: the top 16 SMS lines stay at HOFS 0,
                        ; the IRQ at line 32 writes the real value for the rest

    ; --- split queue: the game's own mid-frame HOFS writes (frame before) + the lock.
    ; DP $30 = count (0..4), V (SNES line whose hblank gets the write) at $31..$34, HOFS lo at
    ; $35..$38, hi at $39..$3C, ascending by V.  Pairs come from $E2:0234 (count) / $E2:0235
    ; (line, lo, hi) x4; SMS line L takes effect on L+1 = SNES V L+18, so the write goes in hblank L+17.
    ldy.w #$0000        ; Y = queue count
    ldx.w #$0000        ; X = raw pair offset
    lda.l $E20234
    sta $3D             ; raw pairs left
    cmp.b #$05
    bcc BqCntOk
    stz $3D             ; garbage before the first pass
BqCntOk:
    stz $3E             ; 1 = the lock event is queued
    lda $1F
    bne BqLoop
    lda.b #$01
    sta $3E             ; no lock: nothing to queue
BqLoop:
    lda $3D
    beq BqEnd
    lda.l $E20235,x
    cmp.b #191
    bcs BqNext          ; not a valid active line
    sta $3F
    lda $3E
    bne BqNormal
    lda $3F
    cmp.b #15
    bcs BqLockNow
    lda.l $E20236,x     ; pair above the lock line: it only sets the value the lock event writes
    sta $1D
    lda.l $E20237,x
    sta $1E
    bra BqNext
BqLockNow:
    lda.b #32
    sta $40
    lda $1D
    sta $41
    lda $1E
    sta $42
    jsr BqAppend
    lda.b #$01
    sta $3E
BqNormal:
    lda $3F
    clc
    adc.b #17
    sta $40
    lda.l $E20236,x
    sta $41
    lda.l $E20237,x
    sta $42
    jsr BqAppend
BqNext:
    inx
    inx
    inx
    dec $3D
    bra BqLoop
BqEnd:
    lda $3E
    bne BqDone
    lda.b #32
    sta $40
    lda $1D
    sta $41
    lda $1E
    sta $42
    jsr BqAppend
BqDone:
    tya
    sta $30
    lda $1F
    beq HofsGo
    stz $210D
    stz $210D
    stz $210F
    stz $210F           ; BG2 follows BG1 (a write pair each: the scroll latch is shared)
    bra HofsDone
HofsGo:
    lda $1D
    sta $210D           ; BG1HOFS low
    lda $1E
    sta $210D           ; BG1HOFS high
    lda $1D
    sta $210F           ; BG2HOFS low
    lda $1E
    sta $210F           ; BG2HOFS high
HofsDone:
    lda.l $E20202
    sta $210E           ; BG1VOFS low
    lda.l $E20203
    sta $210E           ; BG1VOFS high
    lda.l $E20202
    sta $2110           ; BG2VOFS low
    lda.l $E20203
    sta $2110           ; BG2VOFS high

    ; --- forward SNES pad (auto-joypad JOY1) to the FPGA control window $EF ---
    lda $4218
    sta.l $EF0000       ; JOY1L (A X L R)
    lda $4219
    sta.l $EF0001       ; JOY1H (B Y Sel St U D L R)

    ; --- IGR pad combos (chassis parity). This core keeps cheat.v, but the
    ;     NMI hook that feeds its combo detector never runs under the player,
    ;     so the match lives here. Exact 16-bit compare of {JOY1H,JOY1L},
    ;     same values as cheat.v's case():
    ;       $3030 = L+R+Start+Select -> $80 (reset the game)
    ;       $2070 = L+R+Select+X     -> $81 (back to the menu)
    ;     Neither Select/Start/X/L/R reaches the SMS pad translation, so the
    ;     combos can't collide with gameplay. The command byte goes straight
    ;     to the fork's MCU_CMD mailbox ($2A00, snescmd BRAM -- write window
    ;     opened for this core in main.v) and the MCU game loop serves it.
    ;     DP $04 = edge latch: fire only on release->press (armed at Reset).
IgrWaitPad:
    lda $4212           ; guard: $4218 mid auto-read is garbage and a reset
    lsr                 ; must never fire off a misread -- by this point the
    bcs IgrWaitPad      ; DMA work above outlasted the read, so this falls through
    rep #$20
    lda $4218           ; raw pad16 {JOY1H,JOY1L}
    cmp.w #$3030
    beq IgrReset
    cmp.w #$2070
    beq IgrMenu
    sep #$20
    stz $04             ; no combo held -> re-arm the edge
    bra IgrDone
IgrReset:
    sep #$20
    lda.b #$80
    bra IgrFire
IgrMenu:
    sep #$20
    lda.b #$81
IgrFire:
    xba                 ; park the command in B
    lda $04             ; already fired during this hold?
    bne IgrDone
    lda.b #$01
    sta $04
    xba                 ; command back
    sta.l $002A00       ; MCU_CMD
IgrDone:
    sep #$20

    ; --- double-buffer: done reading the front -> swap if the FPGA has a ready back ---
    sta.l $EF0005

    ; --- borders: left-column mask (reg0 bit5) and the vertical mask / display off (reg1 bit6) ---
    lda.l $E2020C
    and.b #$20
    beq LcmOff
    lda.b #$13
LcmOff:
    sta $212E
    lda.l $E2020D
    and.b #$40
    beq DispOff
    lda.b #$13
DispOff:
    sta $06             ; TM value for the IRQs (0 = VDP display off)
    lda.b #$30
    sta $08             ; picture band, no mask: colour math prevented
    lda.b #$07
    sta $09
    lda.l $E2020C
    and.b #$20
    beq NoMaskMath
    lda.b #$90          ; left column masked: paint the backdrop in x 0..7
    sta $08
NoMaskMath:
    lda $06
    bne PicOnMath
    lda.b #$90          ; display off: the whole picture band is backdrop
    sta $08
    lda.b #$FF
    sta $09
PicOnMath:
    ; fixed colour = the real backdrop (BGR555 at $E2:020E/F)
    lda.l $E2020E
    sta $0A
    lda.l $E2020F
    sta $0B
    lda $0A
    and.b #$1F
    ora.b #$20
    sta $2132           ; R
    rep #$20
    lda $0A
    lsr
    lsr
    lsr
    lsr
    lsr
    sep #$20
    and.b #$1F
    ora.b #$40
    sta $2132           ; G
    rep #$20
    lda $0A
    lsr
    lsr
    lsr
    lsr
    lsr
    lsr
    lsr
    lsr
    lsr
    lsr
    sep #$20
    and.b #$1F
    ora.b #$80
    sta $2132           ; B
    lda.b #$14
    sta $4207           ; HTIME = 276 (the lock phase moves it earlier, below)
    lda.b #$01
    sta $4208
    lda $4211           ; ack a pending IRQ
    lda $2137           ; latch the V counter
    lda $213D
    sta $07             ; V low
    lda $213D
    and.b #$01
    bne TmShut          ; V >= 256: vblank
    lda $07
    cmp.b #17
    bcc TmShut
    cmp.b #209
    bcs TmShut
    lda $06             ; the NMI overran into the picture: show it now
    sta $212C
    lda $08
    sta $2130
    lda $09
    sta $2127
TmPop:
    lda $30
    beq TmClosePh       ; queue empty: nothing left to write this frame
    lda $07
    cmp $31
    bcc TmSched         ; next write is still ahead
    lda $35             ; already past: write it right away
    sta $210D
    lda $39
    sta $210D
    lda $35
    sta $210F
    lda $39
    sta $210F
    jsr BqPop
    bra TmPop
TmSched:
    lda.b #$02
    sta $05             ; next IRQ writes the HOFS of the first queued split
    lda.b #$E6
    sta $4207           ; HTIME 230: the handler's first write lands in the hblank
    stz $4208
    lda $31
    bra TmSetV
TmClosePh:
    lda.b #$01
    sta $05             ; next IRQ closes at 208
    lda.b #208
    bra TmSetV
TmShut:
    stz $212C           ; blank until line 16
    lda.b #$90
    sta $2130
    lda.b #$FF
    sta $2127
    stz $05
    lda.b #16
TmSetV:
    sta $4209
    stz $420A

    lda $4210           ; ack NMI

    rep #$30
    ply
    plx
    pla
    sep #$20
    rti

; X = first absolute tile (0..511), Y = tile count (>=1); A/X/Y 16-bit on entry.
; DMA Y tiles (32B each) $E0:(tile*32) -> VRAM word (tile*16). $2115 = +1 word, ch0.
DmaTileSpan:
    tya
    asl
    asl
    asl
    asl
    asl                      ; count*32 = byte length
    sta $4305
    txa
    asl
    asl
    asl
    asl                      ; tile*16 = VRAM word dest
    sta $2116
    txa
    asl
    asl
    asl
    asl
    asl                      ; tile*32 = src byte offset
    sta $4302
    sep #$20
    lda.b #$01
    sta $4300                ; DMA mode 1 ($2118/$2119)
    lda.b #$18
    sta $4301
    lda.b #!TILES_BANK
    sta $4304                ; src bank $E0
    lda.b #$01
    sta $420B                ; trigger ch0
    rep #$20
    rts

TmFull:
    ; every column dirty (the scrolling case): straight-line code, no loop or
    ; bitmap bookkeeping -- the instruction fetch from ROM is what costs here
    sep #$20
    ldy.w #$0032
    lda.b #$01
    !i #= 0
    while !i < 32
        ldx.w #!BG1_MAP+64+!i
        stx $2116
        ldx.w #4+(64*!i)
        stx $4302
        sty $4305
        sta $420B
        !i #= !i+1
    endwhile
    jmp TmDone

; Queue helpers.  BqAppend: in $40 = V, $41/$42 = HOFS lo/hi, Y = count (16 bit); same V as the
; last entry replaces it, a full queue drops the new one.  BqPop (A/X/Y untouched but A): drops entry 0.
BqAppend:
    cpy.w #$0000
    beq BqaNew
    lda.w $0030,y
    cmp $40
    bne BqaNew
    dey
    bra BqaStore
BqaNew:
    cpy.w #$0004
    bcs BqaFull
BqaStore:
    lda $40
    sta.w $0031,y
    lda $41
    sta.w $0035,y
    lda $42
    sta.w $0039,y
    iny
BqaFull:
    rts

BqPop:
    dec $30
    lda $32
    sta $31
    lda $33
    sta $32
    lda $34
    sta $33
    lda $36
    sta $35
    lda $37
    sta $36
    lda $38
    sta $37
    lda $3A
    sta $39
    lda $3B
    sta $3A
    lda $3C
    sta $3B
    rts

Stub:
    rti

; H/V-IRQ trio (dot 276 = hblank; the V=32 one only with the scroll lock): the one on V=16 opens the layers for V=17 (picture row 16), the one on V=208 closes them (SMS has 192
; lines, the PPU 224 with the picture centred; the rest stays backdrop).
IRQ:
    rep #$20
    pha
    sep #$20
    lda $4211           ; ack
    lda $05
    bne IrqNot0
    lda $06
    sta $212C
    lda $08
    sta $2130
    lda $09
    sta $2127
    lda $30
    bne IrqFirst
    lda.b #$01
    sta $05
    lda.b #208
    sta $4209
    bra IrqDone
IrqFirst:
    lda.b #$02
    sta $05
    lda.b #$E6
    sta $4207           ; HTIME 230: the write lands in the hblank, not past it
    stz $4208
    lda $31
    sta $4209
    bra IrqDone
IrqNot0:
    cmp.b #$02
    bne IrqClose
    lda $35             ; hblank of a queued split line: the rest scrolls to the game value
    sta $210D
    lda $39
    sta $210D
    lda $35
    sta $210F
    lda $39
    sta $210F
    jsr BqPop
    lda $30
    beq IrqLast
    lda $31
    sta $4209           ; next split (HTIME stays 230)
    bra IrqDone
IrqLast:
    lda.b #$14
    sta $4207           ; HTIME back to 276
    lda.b #$01
    sta $4208
    sta $05
    lda.b #208
    sta $4209
    bra IrqDone
IrqClose:
    stz $212C
    lda.b #$90
    sta $2130
    lda.b #$FF
    sta $2127
    stz $05
    lda.b #16
    sta $4209
IrqDone:
    rep #$20
    pla
    rti


; ===========================================================================
; S-DSP unmute -- opens the console's cartridge-DAC audio path.
;
; WHY: with no program uploaded to the S-SMP, the S-DSP stays in its IPL
; reset/mute state, and in that state the console's analogue path GATES the
; cartridge DAC output -- the FPGA can drive I2S perfectly and nothing reaches
; the jack.  Same mechanic, same cure, same 82-byte payload as snes/sfx.a65
; (menu) and snes/nes/nes_apu.a65 (NES renderer, proven on hardware).
;
; ENTRY CONTRACT (true at the call site in Reset): native mode, DBR=$00,
; DP=$0000, NMI off ($4200=0 since the reset), A 8-bit / X,Y 16-bit.  Returns
; carry clear = stub running, carry set = timed out (boot continues, at worst
; silent).
; ===========================================================================

!SMS_APU_BEEP = 0              ; 1 = instrumented payload (209 bytes): the same
                               ;     unmute plus a 0.5 s 2 kHz beep at boot, so
                               ;     "no sound" can be split into "the S-DSP
                               ;     gate is still shut" (no beep) and "the gate
                               ;     is open, the PSG side is the problem"
                               ;     (beep, then silence).  BRING-UP ONLY.
                               ; 0 = PRODUCTION: 82 bytes, unmute only, silent.
                               ;     Set this back to 0 before shipping.

!APUIO0 = $2140
!APUIO1 = $2141
!APUIO2 = $2142

if !SMS_APU_BEEP == 1
!SMS_APU_STUB_ADDR = $0300     ; upload base: DIR table + BRR + code
!SMS_APU_EXEC_ADDR = $0320     ; code entry (DIR must own $0300)
!SMS_APU_STUB_LEN  = $00D1     ; 209
else
!SMS_APU_STUB_ADDR = $0300
!SMS_APU_EXEC_ADDR = $0300
!SMS_APU_STUB_LEN  = $0052     ; 82
endif

!SMS_APU_WAIT_POLLS  = $0800   ; per-wait ceiling
!SMS_APU_POLL_BUDGET = $FFFF   ; aggregate ceiling across the whole routine

SmsApuUnmute:
    sep #$20
    rep #$10
    ldy.w #!SMS_APU_POLL_BUDGET

    ; --- 1) wait for the IPL to publish $BBAA ---------------------------------
    ; own ceiling ~5x the handshake one: the IPL spends ~2.4 ms clearing page
    ; zero before it answers, and a timeout here is indistinguishable from
    ; "the analogue gate was not the cause".
    ldx.w #$4000
SmsApuHello:
    lda !APUIO0
    cmp.b #$aa
    bne SmsApuHelloNext
    lda !APUIO1
    cmp.b #$bb
    beq SmsApuHelloOk
SmsApuHelloNext:
    dey
    beq SmsApuFail
    dex
    bne SmsApuHello
    bra SmsApuFail
SmsApuHelloOk:

    ; --- 2) open the transfer -------------------------------------------------
    ldx.w #!SMS_APU_STUB_ADDR
    stx !APUIO2                 ; $2142/$2143 = destination
    lda.b #$cc
    sta !APUIO1                 ; non-zero starts the transfer
    sta !APUIO0
    jsr SmsApuWait
    bcs SmsApuFail

    ; --- 3) payload, byte by byte --------------------------------------------
    ; X is both the table index and the protocol index; the payload is < 256
    ; bytes so the index never wraps.
    ldx.w #$0000
SmsApuSend:
    lda.l SmsApuStub,x
    sta !APUIO1
    txa
    sta !APUIO0
    jsr SmsApuWait
    bcs SmsApuFail
    inx
    cpx.w #!SMS_APU_STUB_LEN
    bne SmsApuSend

    ; --- 4) end of transfer: entry address + index+2 --------------------------
    ldx.w #!SMS_APU_EXEC_ADDR
    stx !APUIO2
    stz !APUIO1                 ; 0 = execute, no further block
    lda !APUIO0
    inc a
    inc a
    sta !APUIO0
    jsr SmsApuWait
    bcs SmsApuFail
    clc
    rts

SmsApuFail:
    sec
    rts

; A(8) = expected echo, Y(16) = remaining global budget.
; carry clear = echoed, carry set = timed out.  A and X preserved.
SmsApuWait:
    phx
    ldx.w #!SMS_APU_WAIT_POLLS
SmsApuWaitPoll:
    cmp !APUIO0
    beq SmsApuWaitOk
    dey
    beq SmsApuWaitTo
    dex
    bne SmsApuWaitPoll
SmsApuWaitTo:
    plx
    sec
    rts
SmsApuWaitOk:
    plx
    clc
    rts

; ===========================================================================
; SPC700 payload.  BYTE-IDENTICAL to the blobs already proven on hardware --
; do not "improve" it here; change snes/sfx.a65 and re-mirror.
;
; Order is the whole point: the DSP is kept MUTED while every voice and the
; whole echo path are scrubbed, MVOL is restored, and FLG's unmute is the LAST
; DSP write.  Unmuting first lets the power-on garbage in the voice registers
; out as continuous clicking (observed on the Mk.II).
; ===========================================================================
SmsApuStub:
if !SMS_APU_BEEP == 1
; ---- instrumented: DIR table ($0300) + BRR ($0310) + code ($0320) ----------
; entry 0 = { start $0310, loop $0310 }; the BRR block is one looping block,
; header $c3 = shift 12 / filter 0 / loop+end, nibbles 7777777788888888 =
; a 16-sample square -> 32000/16 = 2000 Hz at pitch $1000.
    db $10,$03,$10,$03,$00,$00,$00,$00
    db $00,$00,$00,$00,$00,$00,$00,$00
    db $c3,$77,$77,$77,$77,$88,$88,$88
    db $88,$00,$00,$00,$00,$00,$00,$00
; ---- code (identical to the production 82 bytes up to the MVOL block, then
;      voice 0 setup, KON, ~0.5 s delay, KOFF, voice volumes back to 0) ------
    db $8f,$6c,$f2,$8f,$60,$f3,$8f,$5c
    db $f2,$8f,$ff,$f3,$8f,$4c,$f2,$8f
    db $00,$f3,$8f,$4d,$f2,$8f,$00,$f3
    db $8f,$2c,$f2,$8f,$00,$f3,$8f,$3c
    db $f2,$8f,$00,$f3,$8f,$0d,$f2,$8f
    db $00,$f3,$8d,$00,$e8,$00,$c4,$f2
    db $cb,$f3,$bc,$c4,$f2,$cb,$f3,$60
    db $88,$0f,$68,$80,$d0,$f0,$8f,$5c
    db $f2,$8f,$00,$f3,$8f,$5d,$f2,$8f
    db $03,$f3,$8f,$04,$f2,$8f,$00,$f3
    db $8f,$02,$f2,$8f,$00,$f3,$8f,$03
    db $f2,$8f,$10,$f3,$8f,$05,$f2,$8f
    db $00,$f3,$8f,$06,$f2,$8f,$00,$f3
    db $8f,$07,$f2,$8f,$7f,$f3,$8f,$00
    db $f2,$8f,$40,$f3,$8f,$01,$f2,$8f
    db $40,$f3,$8f,$0c,$f2,$8f,$7f,$f3
    db $8f,$1c,$f2,$8f,$7f,$f3,$8f,$6c
    db $f2,$8f,$20,$f3,$8f,$4c,$f2,$8f
    db $01,$f3,$cd,$00,$8d,$00,$00,$dc
    db $d0,$fc,$1d,$d0,$f7,$8f,$5c,$f2
    db $8f,$ff,$f3,$8f,$00,$f2,$8f,$00
    db $f3,$8f,$01,$f2,$8f,$00,$f3,$2f
    db $fe
else
; ---- production: 82 bytes, byte-identical to sfx_dsp_stub_code -------------
    db $8f,$6c,$f2,$8f,$60,$f3   ; FLG   = $60  soft-reset off, MUTE ON
    db $8f,$5c,$f2,$8f,$ff,$f3   ; KOFF  = $ff
    db $8f,$4c,$f2,$8f,$00,$f3   ; KON   = $00
    db $8f,$4d,$f2,$8f,$00,$f3   ; EON   = $00
    db $8f,$2c,$f2,$8f,$00,$f3   ; EVOL_L= $00
    db $8f,$3c,$f2,$8f,$00,$f3   ; EVOL_R= $00
    db $8f,$0d,$f2,$8f,$00,$f3   ; EFB   = $00
    db $8d,$00,$e8,$00           ; y=0, a=0
    db $c4,$f2,$cb,$f3,$bc       ; loop: VOL_L=0, next reg
    db $c4,$f2,$cb,$f3           ;       VOL_R=0
    db $60,$88,$0f,$68,$80,$d0,$f0 ;     a+=15, until a==$80
    db $8f,$0c,$f2,$8f,$7f,$f3   ; MVOL_L= $7f
    db $8f,$1c,$f2,$8f,$7f,$f3   ; MVOL_R= $7f
    db $8f,$6c,$f2,$8f,$20,$f3   ; FLG   = $20  MUTE OFF (last DSP write)
    db $2f,$fe                   ; bra *
endif


; ---- LoROM header (so smc_id detects LoROM) ----
org $00FFC0
    db "SMS PLAYER           "    ; 21-byte title
org $00FFD5
    db $20                        ; map mode: LoROM
    db $00                        ; cart type: ROM only
    db $08                        ; rom size
    db $00                        ; ram size: none
    db $00                        ; country
    db $00                        ; developer
    db $00                        ; version
    dw $0000                      ; checksum complement
    dw $FFFF                      ; checksum

; ---- vectors ----
org $00FFE4
    dw Stub
    dw Stub
    dw Stub
    dw NMI
    dw Stub
    dw IRQ
org $00FFF4
    dw Stub
    dw Stub
    dw Stub
    dw Stub
    dw Reset
    dw Stub
