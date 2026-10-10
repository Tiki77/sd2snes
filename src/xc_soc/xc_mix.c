/* Xeno Crisis on sd2snes: the RP2040 core 1 audio work, rebuilt for the core 0 soft CPU.
 *
 * On the cartridge, core 1 calls audio_process() in a loop. Here the same work runs as two interrupt
 * handlers on the single soft CPU:
 *   - the tick IRQ (1 kHz) starts a call: lock checks, reset, sound effect triggers and music control.
 *     If a music packet is due it goes to the MCU through the decode mailbox and the call pauses.
 *   - the decode IRQ resumes the call: PCM into the ring, end-of-track handling, then mixing; then it starts
 *     the next call at once (core 1 calls audio_process() back to back), so the next packet goes out without
 *     waiting for a tick.
 *   - while a call is paused, the tick IRQ keeps mixing from the music already decoded (the packet being
 *     decoded is left out) and the sound effects. On the RP2040 the decode takes ~7 ms and the BRR rings
 *     never run low; the MCU may take longer (and the sd2snes firmware has pauses of its own), so the
 *     BRR rings must not wait for it.
 * Mixing produces 16-sample stereo blocks while the BRR rings have room; the BRR encoding is done by the
 * xc_brr hardware block. The per-call semantics are those of the firmware's audio_process() (see
 * MesenCE XcAudio::Process, which is bit-exact with it); only the decode happens asynchronously, like
 * on the RP2040 where decoding a packet takes core 1 about 7 ms.
 *
 * The firmware's core 0 never uses interrupts, so the handlers only have to keep off its locks:
 * interrupts are taken between core 0 instructions, and a handler runs to completion.
 * Built for ARMv6-M (Cortex-M0+ subset) with arm-none-eabi-gcc; linked at 0x10F00000, data in SCRATCH_X.
 */
#include <stdint.h>
#include "xc_soc.h"
uint32_t xc_brr_encode(const int16_t* s, uint32_t stride_bytes, uint32_t* ab);   /* xc_brr_sw.S */

/* ---- firmware RAM layout (Xeno Crisis SNES v1.00, see MesenCE XcAudio.cpp) ---- */
#define DecoderPtr      0x20017594u
#define DecoderErr      0x20017598u
#define ResetFlag       0x20022C69u
#define MusicRequest    0x20017510u
#define MusicCurrent    0x20017504u
#define MusicOffset     0x2001AE18u
#define LoopRequest     0x20022C6Bu
#define MusicLoop       0x200229DBu
#define MusicPaused     0x200229DCu
#define MusicPlaying    0x200229DDu
#define MusicEnded      0x20022C6Au
#define SamplesDecoded  0x2001759Cu
#define PcmRing         0x2001810Cu
#define PcmWrite        (PcmRing + 0x2D00u)
#define PcmRead         (PcmRing + 0x2D04u)
#define PcmFree         (PcmRing + 0x2D08u)
#define PcmRingSize     5760u
#define Voices          0x200175A4u
#define Triggers        0x20017530u
#define Brr             0x2001AE24u
#define BrrRingSize     0x6C0u
#define BrrLeft         Brr
#define BrrLeftWrite    (Brr + 0x6C0u)
#define BrrLeftRead     (Brr + 0x6C4u)
#define BrrLeftFree     (Brr + 0x6C8u)
#define BrrRight        (Brr + 0x6CCu)
#define BrrRightWrite   (Brr + 0xD8Cu)
#define BrrRightRead    (Brr + 0xD90u)
#define BrrRightFree    (Brr + 0xD94u)
#define Locks           0x20022C67u
#define Core1Parked     0x20018108u

#define R8(a)   (*(volatile uint8_t*)(a))
#define R32(a)  (*(volatile uint32_t*)(a))
#define S16(a)  (*(volatile int16_t*)(a))

/* The firmware reads 32-bit values from 4-byte-aligned addresses only; stream data can be anywhere */
static inline uint32_t rd_be32(uint32_t a)
{
	return ((uint32_t)R8(a) << 24) | ((uint32_t)R8(a + 1) << 16) | ((uint32_t)R8(a + 2) << 8) | R8(a + 3);
}

static inline int valid_addr(uint32_t a, uint32_t size)
{
	if(a >= 0x10000000u && a + size <= 0x11000000u) return 1;  /* flash image */
	if(a >= 0x20000000u && a + size <= 0x20042000u) return 1;  /* RAM */
	return 0;
}

/* ---- the firmware's rounding helpers, rewritten without division (exact, see test_mix_math.c) ---- */
/* x / 2^k rounded towards zero */
static inline __attribute__((always_inline)) int32_t div_pow2(int32_t x, int k)
{
	return (x + (int32_t)((uint32_t)(x >> 31) >> (32 - k))) >> k;
}

/* 0x10067CE4: Q15 multiply, q = p / 16384; return q / 2 + q % 2 (C semantics) */
static inline __attribute__((always_inline)) int32_t qmul(int32_t a, int32_t b)
{
	int32_t p = (int32_t)((uint32_t)a * (uint32_t)b);
	int32_t q = div_pow2(p, 14);
	int32_t h = div_pow2(q, 1);
	return h + (q - 2 * h);
}

/* saturation after mixing: if out of range, (x / 32768) * 32760 */
static inline __attribute__((always_inline)) int32_t clamp(int32_t x)
{
	if((uint32_t)x + 0x7FFFu > 0xFFFEu) {
		return (int32_t)((uint32_t)div_pow2(x, 15) * 32760u);
	}
	return x;
}

/* ---- mixer state (SCRATCH_X) ---- */
enum { PH_IDLE = 0, PH_WAIT_DECODE = 1, PH_MIX_DEFERRED = 2 };
static struct {
	uint32_t phase;
	uint32_t held;           /* PCM ring samples reserved for the packet being decoded (0 or 960) */
	uint32_t wr;             /* PCM ring position of the packet being decoded */
	uint32_t expected_range;
	uint32_t last_range;     /* decoder final range after the last packet */
	uint32_t msu;            /* MSU-1 music: from the MSU-1 pack, no decoding */
	uint32_t ms_state;       /* MS_* */
	uint32_t ms_t;           /* game track (struct address) the MSU-1 track belongs to */
	uint32_t ms_next_t;      /* loop part to start when the intro track has played out */
	uint32_t ms_next_rep;
	uint32_t ms_repeat;
	uint32_t ms_paused;
	uint32_t ms_seen_play;   /* the MSU-1 reported playing since the last play command */
	uint32_t ms_age;         /* ticks since the last play command */
	uint32_t coop;           /* mk2 core: no tick IRQ or BRR encoder; run from the firmware's wait loops */
} st;

static void music_after_decode(int have_decode, int32_t ret);
static void music_end(void);
static void mix(void);

/* counters for the MCU's log (MSU-1 music: xc_debug.txt); at 0x20040000 = SRAM chip 0x48000 (xc_mix.ld) */
struct xc_dbg {
	uint32_t magic;          /* "XMIX" once installed */
	uint32_t mode;           /* bit 0: MSU-1 mode; bits 15:8 phase, 23:16 ms_state */
	uint32_t ticks, calls, sfx, blocks, deferred, packets;
	uint32_t msu_writes, msu_status, track;
	uint32_t coop_us;        /* mk2: microseconds spent in the mixer (xc_mix_poll with ticks due), for the load */
	uint32_t fast;           /* mk2: blocks encoded in fast mode (the mixer had fallen behind) */
};
static struct xc_dbg dbg __attribute__((section(".bss.xc_dbg"), used));

/* mk2, read by xc_mix_poll() (xc_mix_entry.S) before it does anything: magic = XC_COOP_MAGIC once
   xc_mix_install_mk2() ran (a magic value, because the mk2 core does not clear the RAM at start); next = time of
   the next tick (microseconds, low 32 bits of the timer) */
#define XC_COOP_MAGIC 0x434F4F50u   /* "COOP" */
struct { uint32_t magic, next; } xc_mix_coop;

/* ---- MSU-1 mode (CTRL bit 3: mk3 core with an MSU-1 pack, mk2 core always) ----
 * The music comes from an MSU-1 pack. The packets are still walked at the same pace (the game's music
 * position, end-of-track and loop logic stay as they are, and core 0 sees the same state), but nothing is
 * decoded or mixed; each track start becomes an MSU-1 track request. Sound effects are mixed as before.
 *
 * MSU-1 track n = the game's music stream n (1-based, the order of the firmware's stream table):
 *  1 MUSIC_0_INTRO_VOICED    2 MUSIC_10_ALARM          3 MUSIC_1_TITLE           4 MUSIC_2_INGAME_0_INTRO
 *  5 MUSIC_2_INGAME_0_LOOP   6 MUSIC_2_INGAME_1        7 MUSIC_2_INGAME_2        8 MUSIC_2_INGAME_3_INTRO
 *  9 MUSIC_2_INGAME_3_LOOP  10 MUSIC_2_INGAME_4_INTRO 11 MUSIC_2_INGAME_4_LOOP  12 MUSIC_2_INGAME_5_INTRO
 * 13 MUSIC_2_INGAME_5_LOOP  14 MUSIC_2_INGAME_6_INTRO 15 MUSIC_2_INGAME_6_LOOP  16 MUSIC_3_BOSS_0
 * 17 MUSIC_3_BOSS_1         18 MUSIC_3_BOSS_2_INTRO   19 MUSIC_3_BOSS_2_LOOP    20 MUSIC_3_BOSS_3
 * 21 MUSIC_4_CUTSCENE       22 MUSIC_5_SHOP           23 MUSIC_6_CONTINUE       24 MUSIC_7_STAGE_CLEAR
 * 25 MUSIC_8_GAME_OVER      26 MUSIC_9_ENDING
 * A track the game loops onto itself plays with MSU-1 repeat (the .pcm loop point decides where). An intro
 * plays once and its loop part follows when the intro .pcm has played out, so a pack whose intro differs in
 * length from the game's leaves no gap and cuts nothing. A missing .pcm is silent. */
#define MSU_VOLUME 0xFFu
static const uint32_t msu_bases[26] = {
	0x102CE930u, 0x102E7120u, 0x102EAFD0u, 0x1030EC20u, 0x10310F40u, 0x10362A10u, 0x103D5930u, 0x1043A4E0u,
	0x10443470u, 0x1049FC80u, 0x104B6290u, 0x10504AF0u, 0x1050C690u, 0x105612B0u, 0x1057E190u, 0x105D5560u,
	0x105FF030u, 0x10618020u, 0x10618970u, 0x106374F0u, 0x10658090u, 0x10679BA0u, 0x1068E6F0u, 0x106A95D0u,
	0x106AEDB0u, 0x106B7720u
};
enum { MS_IDLE = 0, MS_WAIT = 1, MS_PLAY = 2, MS_MISSING = 3 };

static void msu_reg(uint32_t r, uint32_t v) { XC_MSU_REG = r | (v << 8); dbg.msu_writes++; }
static uint32_t msu_ctrl(void) { return (st.ms_paused ? 0u : 1u) | (st.ms_repeat ? 2u : 0u); }
static uint32_t track_next(uint32_t t) { return valid_addr(t, 16) ? R32(t + 0x0C) : 0; }

static void msu_stop(void)
{
	if(st.ms_state == MS_PLAY) msu_reg(7, 0);
	st.ms_state = MS_IDLE; st.ms_t = 0; st.ms_next_t = 0;
}

static void msu_start(uint32_t t, uint32_t repeat)
{
	uint32_t base = valid_addr(t, 16) ? R32(t + 8) : 0, n = 0;
	for(uint32_t i = 0; i < 26; i++) if(msu_bases[i] == base) n = i + 1;
	if(!n) { msu_stop(); return; }
	st.ms_t = t; st.ms_next_t = 0; st.ms_repeat = repeat; st.ms_seen_play = 0;
	msu_reg(6, MSU_VOLUME);
	msu_reg(4, n & 0xFF);
	dbg.track = n;
	msu_reg(5, n >> 8);              /* track request: the MCU opens the .pcm (audio busy until done) */
	st.ms_state = MS_WAIT;
}

/* the game starts streaming track t from its beginning */
static void msu_stream_start(uint32_t t)
{
	uint32_t repeat = R8(MusicLoop) && track_next(t) == 0;
	if(st.ms_state != MS_IDLE && t == st.ms_t) {
		/* the same track again: its self-loop (MSU-1 repeat already covers it), the loop part we started
		   ourselves when the intro .pcm ended before the game's intro, or a track the pack does not have */
		if(st.ms_state == MS_MISSING || st.ms_repeat || st.ms_state == MS_WAIT || (!st.ms_seen_play && st.ms_age < 1000u) || (XC_MSU_STATUS & XC_MSU_ST_PLAYING)) return;
	} else if((st.ms_state == MS_WAIT || st.ms_state == MS_PLAY) && track_next(st.ms_t) == t) {
		/* the game went on from the intro to its loop part: follow when the intro .pcm has played out */
		st.ms_next_t = t; st.ms_next_rep = repeat;
		return;
	}
	msu_start(t, repeat);
}

/* every tick: finish a track request, go on from an intro to its loop part */
static void msu_update(void)
{
	if(st.ms_state == MS_IDLE || st.ms_state == MS_MISSING) return;
	uint32_t s = XC_MSU_STATUS;
	dbg.msu_status = s;
	if(s & XC_MSU_ST_AUDIO_BUSY) return;
	if(st.ms_state == MS_WAIT) {
		if(s & XC_MSU_ST_MISSING) { st.ms_state = MS_MISSING; return; }   /* no .pcm for this track: silent */
		msu_reg(7, msu_ctrl());
		st.ms_state = MS_PLAY;
		st.ms_age = 0;
		return;
	}
	if(s & XC_MSU_ST_PLAYING) { st.ms_seen_play = 1; return; }
	if(st.ms_age < 1000u) st.ms_age++;
	/* not playing: the track has ended, unless the MCU has not answered the play command yet (it may be busy
	   saving for a while; a .pcm too short to be seen playing counts as ended after a second) */
	if((!st.ms_seen_play && st.ms_age < 1000u) || st.ms_paused || st.ms_repeat) return;
	if(st.ms_next_t) {
		msu_start(st.ms_next_t, st.ms_next_rep);
	} else if(R8(MusicLoop) && track_next(st.ms_t)) {
		/* the intro .pcm is shorter than the game's intro: start the loop part now */
		uint32_t t = track_next(st.ms_t);
		msu_start(t, track_next(t) == 0);
	}
}
static void mix_blocks(void);

/* ---- call start (tick IRQ) ---- */
static void call_begin(void)
{
	/* lock 1: core 0 is writing flash and waits until core 1 is parked; lock 0: core 0 is using the rings */
	if(R8(Locks + 1)) {
		R32(Core1Parked) = 1;
		return;
	}
	if(R8(Locks + 0)) {
		return;
	}
	if(!st.coop) R32(Core1Parked) = 0;   /* mk2: core 1 counts as parked whenever core 0 runs (see xc_mix_poll) */
	XC_MIX_EVENT = XC_EV_CALL_BEGIN;
	dbg.calls++;

	if(R8(ResetFlag)) {
		if(st.msu) msu_stop();
		R8(ResetFlag) = 0;
		R32(MusicRequest) = 0;
		R8(MusicLoop) = 0;
		R8(MusicPaused) = 0;
		R8(MusicPlaying) = 0;
		R32(BrrLeftRead) = 0;
		R32(BrrLeftFree) = BrrRingSize;
		R32(BrrRightRead) = 0;
		R32(BrrLeftWrite) = 0;
		R32(BrrRightWrite) = 0;
		R32(BrrRightFree) = BrrRingSize;
		for(uint32_t i = 0; i < 0x80; i += 4) R32(Voices + i) = 0;
		for(uint32_t i = 0; i < 0x60; i += 4) R32(Triggers + i) = 0;
	}

	/* sound effect triggers queued by core 0 */
	for(uint32_t i = 0; i < 8; i++) {
		uint32_t t = Triggers + i * 12;
		if(!R8(t)) continue;
		uint8_t voice = R8(t + 1);
		if(voice <= 3) {
			uint32_t v = Voices + voice * 32;
			R8(v + 0x1C) = 1;
			R8(v + 0x1D) = 0;
			R32(v + 0x08) = 0x4AAA;
			R32(v + 0x00) = R32(t + 8) >> 1;
			R32(v + 0x04) = R32(t + 4);
			R32(v + 0x0C) = 0;
			R8(t) = 0;
			R32(v + 0x10) = 0x3333;
			R32(v + 0x14) = 0x3333;
			dbg.sfx++;
		}
	}

	/* music control */
	uint32_t request = R32(MusicRequest);
	if(request != R32(MusicCurrent)) {
		R32(MusicCurrent) = request;
		R32(MusicOffset) = 0;
		R8(MusicLoop) = R8(LoopRequest);
		R8(MusicEnded) = 0;
		R8(MusicPaused) = 0;
		R8(MusicPlaying) = 0;
		R32(PcmRead) = 0;
		R32(PcmWrite) = 0;
		R32(PcmFree) = PcmRingSize;
	}

	uint32_t track = R32(MusicCurrent);
	if(st.msu) {
		if(track == 0 && st.ms_state != MS_IDLE) msu_stop();
		uint32_t p = R8(MusicPaused) ? 1u : 0u;
		if(p != st.ms_paused) {
			st.ms_paused = p;
			if(st.ms_state == MS_PLAY) {
				msu_reg(7, msu_ctrl());
				st.ms_seen_play = 0;      /* wait for the MCU to answer again (see msu_update) */
				st.ms_age = 0;
			}
		}
	}
	if(track != 0 && !R8(MusicEnded) && !R8(MusicPaused)) {
		if(R32(PcmFree) >= 960) {
			uint32_t base = R32(track + 8);
			uint32_t off = R32(MusicOffset);
			dbg.packets++;
			if(st.msu && off == 0) msu_stream_start(track);
			R32(MusicOffset) = off + 4;
			uint32_t length = valid_addr(base + off, 4) ? rd_be32(base + off) : 0xFFFFFFFFu;
			if(length <= 1500) {
				off = R32(MusicOffset);
				R32(MusicOffset) = off + 4;
				st.expected_range = valid_addr(base + off, 4) ? rd_be32(base + off) : 0;
				if(length != 0) {
					uint32_t wr = R32(PcmWrite);
					R32(PcmFree) = R32(PcmFree) - 960;
					R32(PcmWrite) = (wr + 960) % PcmRingSize;
					off = R32(MusicOffset);
					R32(MusicOffset) = off + length;
					st.wr = wr;
					if(st.msu) {
						/* MSU-1 music: nothing to decode; the packet counts as decoded (480 samples) and its ring
						   slot paces the music position as before */
						st.last_range = st.expected_range;
						music_after_decode(1, 480);
					} else if(valid_addr(base + off, length)) {
						/* hand the packet to the MCU and pause the call until it is decoded */
						uint32_t src = base + off;
						for(uint32_t i = 0; i < length; i += 4) {
							uint32_t w = R8(src + i);
							if(i + 1 < length) w |= (uint32_t)R8(src + i + 1) << 8;
							if(i + 2 < length) w |= (uint32_t)R8(src + i + 2) << 16;
							if(i + 3 < length) w |= (uint32_t)R8(src + i + 3) << 24;
							XC_DEC_PACKET[i >> 2] = w;
						}
						XC_DEC_LEN = length;
						XC_DEC_CTRL = XC_DEC_SUBMIT;
						st.phase = PH_WAIT_DECODE;
						st.held = 960;
						return;
					}
					music_after_decode(1, -4 /* OPUS_INVALID_PACKET: nothing decoded */);
				} else {
					music_after_decode(0, 0);
				}
			}
		}
		if(!R8(MusicPlaying) && R32(PcmFree) == 0) R8(MusicPlaying) = 1;
	}
	mix();
}

/* rest of the music part after a packet (decoded or not) */
static void music_after_decode(int have_decode, int32_t ret)
{
	if(have_decode) {
		R32(SamplesDecoded) = R32(SamplesDecoded) + (uint32_t)ret;
	}
	uint32_t range = st.last_range;
	int end_of_track;
	if(st.expected_range != 0 && range != st.expected_range) {
		end_of_track = 1;  /* corrupted data */
	} else {
		uint32_t cur = R32(MusicCurrent);
		end_of_track = cur != 0 && R32(cur + 4) <= R32(MusicOffset);
	}
	if(end_of_track) music_end();
}

/* 0x10067D10: next track when looping, else stop */
static void music_end(void)
{
	if(R8(MusicLoop)) {
		uint32_t cur = R32(MusicCurrent);
		uint32_t next = cur ? R32(cur + 0x0C) : 0;
		if(next) {
			R32(MusicCurrent) = next;
			R32(MusicRequest) = next;
		}
	}
	R32(MusicOffset) = 0;
	if(!R8(MusicLoop)) R8(MusicEnded) = 1;
}

/* ---- call resume (decode IRQ) ---- */
static void call_resume(void)
{
	int32_t ret = XC_DEC_RET;
	st.last_range = XC_DEC_RANGE;
	if(ret > 0) {
		volatile uint32_t* dst = (volatile uint32_t*)(PcmRing + st.wr * 2);
		for(int32_t i = 0; i < ret; i++) dst[i] = XC_DEC_PCM[i]; /* one stereo sample per word */
	}
	XC_DEC_CTRL = XC_DEC_ACK;
	st.held = 0;
	music_after_decode(1, ret);
	if(!R8(MusicPlaying) && R32(PcmFree) == 0) R8(MusicPlaying) = 1;
	mix();
	/* core 1's loop: the next call follows directly (sends the next packet if one is due) */
	if(st.phase == PH_IDLE) call_begin();
}

/* ---- mixing and BRR output ---- */
static void encode_to_ring(const int32_t* s, uint32_t ring, uint32_t write_reg, uint32_t free_reg)
{
	uint32_t offset = R32(write_reg);
	R32(free_reg) = R32(free_reg) - 9;
	uint32_t dst = ring + offset;
	R32(write_reg) = (offset + 9) % BrrRingSize;
	for(int i = 0; i < 16; i++) XC_BRR_SAMPLE(i) = (uint32_t)s[i];
	XC_BRR_CTRL = 1;
	while(XC_BRR_CTRL & 1) {
	}
	uint32_t w0 = XC_BRR_OUT(0), w1 = XC_BRR_OUT(1), w2 = XC_BRR_OUT(2);
	uint8_t end = (dst == ring + BrrRingSize - 9) ? 1 : 0;
	R8(dst + 0) = (uint8_t)w0 | end;
	R8(dst + 1) = (uint8_t)(w0 >> 8);
	R8(dst + 2) = (uint8_t)(w0 >> 16);
	R8(dst + 3) = (uint8_t)(w0 >> 24);
	R8(dst + 4) = (uint8_t)w1;
	R8(dst + 5) = (uint8_t)(w1 >> 8);
	R8(dst + 6) = (uint8_t)(w1 >> 16);
	R8(dst + 7) = (uint8_t)(w1 >> 24);
	R8(dst + 8) = (uint8_t)w2;
}

static void mix(void)
{
	/* the firmware takes lock 0 for every block; in an interrupt we cannot wait, so finish the call later */
	if(R8(Locks + 0)) {
		st.phase = PH_MIX_DEFERRED;
		dbg.deferred++;
		XC_MIX_EVENT = XC_EV_DEFERRED;
		return;
	}
	mix_blocks();
	st.phase = PH_IDLE;
	XC_MIX_EVENT = XC_EV_CALL_END;
}

/* ---- mk2: mixing and BRR encoding in software ---- */
/* The voice loops give the same results as the one in mix_blocks(), with the fields in registers and the position
   written back once. add = 0: the first voice of the block (the buffer is not set yet). */

/* one voice, mono: buf[k] = samples 2k (bits 15:0) and 2k + 1 (31:16). In assembly (xc_mix_voice.S; the compiled
   C spilled to the stack in the loop, and every store is an SRAM write here); test_mix_voice_asm.py checks it
   against this loop in C. */
void xc_mix_voice_mono(uint32_t v, uint32_t* buf, int add);
#define mix_voice_mono xc_mix_voice_mono

/* one voice, stereo: buf[i] = sample i left (bits 15:0) and right (31:16) */
static void mix_voice_stereo(uint32_t v, uint32_t* buf, int add)
{
	int32_t length = (int32_t)R32(v);
	int32_t pos = (int32_t)R32(v + 0x0C);
	int32_t step = (int32_t)R32(v + 0x08);
	uint32_t base = R32(v + 4);
	int32_t vl = (int32_t)R32(v + 0x10), vr = (int32_t)R32(v + 0x14);
	int i = 0;
	for(; i < 16; i++) {
		int32_t idx = div_pow2(pos, 15);
		if(length <= idx || pos < (int32_t)0xFFFF8001u) {
			R8(v + 0x1C) = 0;
			R8(v + 0x1D) = 1;
			break;
		}
		int32_t sample = S16(base + (uint32_t)idx * 2);
		int32_t l = qmul(sample, vl), r = qmul(sample, vr);
		if(add) {
			uint32_t w = buf[i];
			l += (int32_t)(int16_t)w;
			r += (int32_t)w >> 16;
		}
		l = clamp(l); r = clamp(r);
		buf[i] = ((uint32_t)l & 0xFFFFu) | (uint32_t)r << 16;
		pos += step;
	}
	if(!add) for(; i < 16; i++) buf[i] = 0;
	R32(v + 0x0C) = (uint32_t)pos;
}

/* 9 bytes (h, then a and b most significant byte first) to dst with the fewest aligned stores */
static inline __attribute__((always_inline)) void store9(uint32_t dst, uint32_t h, uint32_t a, uint32_t b)
{
	uint32_t ra = __builtin_bswap32(a), rb = __builtin_bswap32(b);   /* bytes 1-4, 5-8 in memory order */
	uint32_t w0 = h | ra << 8, w1 = ra >> 24 | rb << 8, last = rb >> 24;
	switch(dst & 3) {
	case 0:
		R32(dst) = w0; R32(dst + 4) = w1; R8(dst + 8) = (uint8_t)last;
		break;
	case 1:
		R8(dst) = (uint8_t)h;
		*(volatile uint16_t*)(dst + 1) = (uint16_t)(w0 >> 8);
		R32(dst + 3) = w0 >> 24 | w1 << 8;
		*(volatile uint16_t*)(dst + 7) = (uint16_t)(w1 >> 24 | last << 8);
		break;
	case 2:
		*(volatile uint16_t*)dst = (uint16_t)w0;
		R32(dst + 2) = w0 >> 16 | w1 << 16;
		*(volatile uint16_t*)(dst + 6) = (uint16_t)(w1 >> 16);
		R8(dst + 8) = (uint8_t)last;
		break;
	default:
		R8(dst) = (uint8_t)h;
		R32(dst + 1) = ra;
		R32(dst + 5) = rb;
		break;
	}
}

/* a BRR ring's next block (END flag on the ring's last block) */
static inline __attribute__((always_inline)) void ring_put(uint32_t ring, uint32_t write_reg, uint32_t free_reg,
                                                           uint32_t h, uint32_t a, uint32_t b)
{
	uint32_t offset = R32(write_reg);
	R32(free_reg) = R32(free_reg) - 9;
	uint32_t dst = ring + offset;
	offset += 9;
	R32(write_reg) = offset >= BrrRingSize ? offset - BrrRingSize : offset;
	store9(dst, h | (dst == ring + BrrRingSize - 9 ? 1u : 0u), a, b);
}

/* the BRR block of 16 samples (s[0], s[stride], ...; s = 0: silence, what xc_brr gives for 16 zeros: shift 12,
   loop flag, nibbles 0) to the left (rings bit 0) and/or right (bit 1) ring */
static void __attribute__((noinline)) coop_emit(const int16_t* s, int stride, int rings)
{
	uint32_t h = 0xC2u, a = 0, b = 0;
	if(s) {
		/* (16 zeros come out as silence too: all nibbles 0 give shift 12) */
		uint32_t ab[2];
		/* fast mode (bit 0) when the mixer has fallen behind: the left ring is less than half full */
		uint32_t fast = R32(BrrLeftFree) > BrrRingSize / 2 ? 1u : 0u;
		h = xc_brr_encode(s, (uint32_t)stride * 2u | fast, ab);   /* xc_brr_sw.S: brr_sw_encode() in assembly */
		dbg.fast += fast;
		a = ab[0]; b = ab[1];
	}
	if(rings & 1) ring_put(BrrLeft, BrrLeftWrite, BrrLeftFree, h, a, b);
	if(rings & 2) ring_put(BrrRight, BrrRightWrite, BrrRightFree, h, a, b);
}

/* mk2: up to n blocks while the BRR rings have room (the same pacing of the music position as mix_blocks()) */
static void coop_blocks(uint32_t n)
{
	while(n && R32(BrrLeftFree) > 8) {
		n--;
		/* the MSU-1 plays the music; its ring slot still paces the game's music position */
		if(R8(MusicPlaying)) {
			uint32_t free_count = R32(PcmFree);
			if(((PcmRingSize - free_count - st.held) >> 5) != 0) {
				R32(PcmFree) = free_count + 0x20;
				uint32_t rd = R32(PcmRead) + 0x20;
				R32(PcmRead) = rd >= PcmRingSize ? rd - PcmRingSize : rd;
			} else {
				XC_MIX_EVENT = XC_EV_UNDERRUN;
			}
		}
		/* the voices, into 16-bit samples (the sums are saturated to 16 bits anyway), two per word, since every
		   store costs an SRAM write here. The game's sound effects have the same volume left and right: then
		   one channel is mixed and encoded, and the block goes to both rings. Otherwise the channels are
		   interleaved (left, right, ...). */
		uint32_t act = 0;
		int mono = 1;
		for(uint32_t voice = 0; voice < 4; voice++) {
			uint32_t v = Voices + voice * 32;
			if(R8(v + 0x1C)) {
				act |= 1u << voice;
				if(R32(v + 0x10) != R32(v + 0x14)) mono = 0;
			}
		}
		dbg.blocks++;
		if(!act) {
			coop_emit(0, 1, 3);
			continue;
		}
		uint32_t buf[16];
		int add = 0;
		for(uint32_t voice = 0; voice < 4; voice++) {
			if(!(act & (1u << voice))) continue;
			uint32_t v = Voices + voice * 32;
			if(mono) mix_voice_mono(v, buf, add);
			else mix_voice_stereo(v, buf, add);
			add = 1;
		}
		if(mono) {
			coop_emit((const int16_t*)buf, 1, 3);
		} else {
			coop_emit((const int16_t*)buf, 2, 1);
			coop_emit((const int16_t*)buf + 1, 2, 2);
		}
	}
}

/* 16-sample blocks while the BRR rings have room; music from the decoded part of the PCM ring only */
static void mix_blocks(void)
{
	if(st.coop) return;   /* mk2: xc_mix_poll_body() mixes (coop_blocks) */
	while(R32(BrrLeftFree) > 8) {
		int32_t left[16], right[16];
		int music = 0;
		if(R8(MusicPlaying)) {
			uint32_t free_count = R32(PcmFree);
			if(((PcmRingSize - free_count - st.held) >> 5) != 0) {
				uint32_t rd = R32(PcmRead);
				R32(PcmFree) = free_count + 0x20;
				rd += 0x20;
				R32(PcmRead) = rd >= PcmRingSize ? rd - PcmRingSize : rd;   /* = (rd + 0x20) % PcmRingSize */
				rd -= 0x20;
				uint32_t p = PcmRing + rd * 2;
				if(!st.msu) {
					for(int i = 0; i < 16; i++) {
						left[i] = qmul(S16(p + i * 4), 0x6666);
						right[i] = qmul(S16(p + i * 4 + 2), 0x6666);
					}
					music = 1;
				}
			} else {
				XC_MIX_EVENT = XC_EV_UNDERRUN;
			}
		}
		if(!music) {
			for(int i = 0; i < 16; i++) left[i] = right[i] = 0;
		}

		for(uint32_t voice = 0; voice < 4; voice++) {
			uint32_t v = Voices + voice * 32;
			if(!R8(v + 0x1C)) continue;
			int32_t length = (int32_t)R32(v);
			for(int i = 0; i < 16; i++) {
				int32_t pos = (int32_t)R32(v + 0x0C);
				int32_t idx = div_pow2(pos, 15);
				if(length <= idx || pos < (int32_t)0xFFFF8001u) {
					R8(v + 0x1C) = 0;
					R8(v + 0x1D) = 1;
					break;
				}
				int32_t sample = S16(R32(v + 4) + (uint32_t)idx * 2);
				left[i] = clamp(qmul(sample, (int32_t)R32(v + 0x10)) + left[i]);
				right[i] = clamp(qmul(sample, (int32_t)R32(v + 0x14)) + right[i]);
				R32(v + 0x0C) = (uint32_t)((int32_t)R32(v + 0x08) + pos);
			}
		}

		dbg.blocks++;
		encode_to_ring(left, BrrLeft, BrrLeftWrite, BrrLeftFree);
		encode_to_ring(right, BrrRight, BrrRightWrite, BrrRightFree);
	}
}

/* ---- interrupt entry points (called on the private stack by the wrappers in xc_mix_entry.S) ---- */
void xc_mix_tick(void)
{
	dbg.ticks++;
	dbg.mode = st.msu | st.phase << 8 | st.ms_state << 16;
	if(st.msu) msu_update();
	if(st.phase == PH_WAIT_DECODE) {
		/* call paused until the decode finishes; keep the BRR rings filled meanwhile (not while core 0 holds
		   a lock: lock 1 = flash write, core 0 waits for core 1 to park; lock 0 = core 0 uses the rings) */
		if(!R8(Locks + 1) && !R8(Locks + 0)) mix_blocks();
		return;
	}
	if(st.phase == PH_MIX_DEFERRED) { mix(); return; }
	call_begin();
}

void xc_mix_decoded(void)
{
	if(st.phase == PH_WAIT_DECODE && (XC_DEC_CTRL & 2)) {
		st.phase = PH_IDLE;
		call_resume();
	}
}

/* ---- mk2: the mixer without interrupts ----
 * The mk2 core has no tick timer, interrupts or BRR encoder (no room in the XC3S400). The firmware's wait loops
 * (bus layer, sleep_until: xc_shim.c) call xc_mix_poll() (xc_mix_entry.S). It returns at once, without a single
 * store (every store costs an SRAM write on the mk2 core, and the wait loops call it all the time), unless a tick
 * is due or the left BRR ring has room for a block; then it calls xc_mix_poll_body() on the mixer's stack, which
 * runs the due ticks (up to COOP_CATCHUP; after a longer gap the rest are dropped) and mixes up to COOP_BLOCKS
 * blocks (COOP_BLOCKS_BEHIND, and the encoder's fast mode, while the rings are less than half full, e.g. after core
 * 0 was busy for a while; COOP_BLOCKS_LOW while they are less than a quarter full: without it, busy scenes with
 * many sound effects ran the rings almost empty, heard as crackling). So the mixing fills core 0's waiting time a little at a time (core 0 is held up by that
 * much at most), until the rings (~128 ms) are full. Music: the same track logic as mk3 with an MSU-1 pack. Sound effects: mixed as on mk3,
 * BRR-encoded in software (xc_brr_sw.h). Core 1 counts as parked all the time, because the mixer only ever runs
 * inside a call made by core 0 (core 0's flash save waits for Core1Parked in a loop that calls nothing). */
#define COOP_CATCHUP 16u
#define COOP_BLOCKS 2u          /* per call: ~0.3 ms at 20 MHz at most */
#define COOP_BLOCKS_BEHIND 8u   /* per call while the rings are less than half full (~1.2 ms) */
#define COOP_BLOCKS_LOW 24u     /* per call while the rings are less than a quarter full (~3.5 ms) */
static uint32_t timer_lo(void) { return *(volatile uint32_t*)0x40054028u; }   /* TIMERAWL */

void xc_mix_poll_body(void)
{
	XC_MIX_EVENT = XC_EV_COOP_ENTER;   /* for the simulation's statistics (a fast register write, no SRAM) */
	uint32_t t0 = timer_lo(), now = t0;
	uint32_t n = 0;
	while((int32_t)(now - xc_mix_coop.next) >= 0) {
		if(n++ == COOP_CATCHUP) { xc_mix_coop.next = now + 1000u; break; }
		xc_mix_coop.next += 1000u;
		xc_mix_tick();
	}
	if(st.phase == PH_IDLE && !R8(Locks + 0) && !R8(Locks + 1))
	{
		uint32_t fr = R32(BrrLeftFree);
		coop_blocks(fr > BrrRingSize * 3 / 4 ? COOP_BLOCKS_LOW : fr > BrrRingSize / 2 ? COOP_BLOCKS_BEHIND : COOP_BLOCKS);
	}
	dbg.coop_us += timer_lo() - t0;
	XC_MIX_EVENT = XC_EV_COOP_LEAVE;
}

/* ---- replaces multicore_launch_core1(core1_entry) in the firmware ---- */
extern void xc_irq_tick(void);
extern void xc_irq_decoded(void);
extern uint32_t __bss_start__, __bss_end__;

void xc_mix_install(void (*entry)(void))
{
	(void)entry;
	for(uint32_t* p = &__bss_start__; p < &__bss_end__; p++) *p = 0;

	/* what core1_init_opus() leaves behind: decoder handle, error code, reset request */
	st.msu = (XC_DEC_CTRL & XC_DEC_MSU_MODE) ? 1u : 0u;
	dbg.magic = 0x58494D58u;         /* "XMIX" */
	if(!st.msu) XC_DEC_CTRL = XC_DEC_RESET;
	else msu_reg(7, 0);             /* SNES reset: stop what the MSU-1 was playing */
	R32(DecoderPtr) = 0x0DEC0DE0u;  /* non-null marker (the decoder lives on the MCU) */
	R32(DecoderErr) = 0;
	R8(ResetFlag) = 1;

	volatile uint32_t* vt = (volatile uint32_t*)(*(volatile uint32_t*)0xE000ED08u); /* VTOR */
	vt[16 + XC_IRQ_TICK] = (uint32_t)xc_irq_tick;
	vt[16 + XC_IRQ_DEC] = (uint32_t)xc_irq_decoded;
	XC_TICK_PERIOD = 1000;
	*(volatile uint32_t*)0xE000E100u = (1u << XC_IRQ_TICK) | (1u << XC_IRQ_DEC); /* NVIC ISER */
}

/* mk2 (the "MK2P" table redirects multicore_launch_core1 here): no interrupts, xc_mix_poll() runs the ticks */
void xc_mix_install_mk2(void (*entry)(void))
{
	for(uint32_t* p = &__bss_start__; p < &__bss_end__; p++) *p = 0;
	st.msu = 1;                     /* the mk2 core is MSU-1 only (no decode mailbox) */
	st.coop = 1;
	dbg.magic = 0x58494D58u;         /* "XMIX" */
	if(st.msu) msu_reg(7, 0);       /* SNES reset: stop what the MSU-1 was playing */
	R32(DecoderPtr) = 0x0DEC0DE0u;
	R32(DecoderErr) = 0;
	R8(ResetFlag) = 1;
	R32(Core1Parked) = 1;
	xc_mix_coop.next = timer_lo();
	xc_mix_coop.magic = XC_COOP_MAGIC;
}
