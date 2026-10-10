/* Host test for xc_brr_sw.h (the mk2 software BRR encoder) against the firmware's brute-force encoder
 * (XcAudio::EncodeBrrBlock in MesenCE, xc_brr.v in the mk3 cores).
 *   cc -O2 test_brr_sw.c -lm && ./a.out [blocks.bin ...]
 * blocks.bin: 16 x int16 per block, e.g. from MesenCE with XC_BRRDUMP=file (every block the mixer encodes).
 * Reports how many blocks come out byte-identical, and whether the software encoder is ever worse (squared error
 * of the decoded block) than the firmware's. Without files: 2 million random blocks of all amplitudes. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "xc_brr_sw.h"

static void brr_ref(const int32_t* samples, uint8_t out[9])
{
	int16_t s[16];
	for(int i = 0; i < 16; i++) s[i] = (int16_t)samples[i];
	memset(out, 0, 9);
	int32_t best = 0x7FFFFFFF;
	int found = 0;
	uint8_t header = 0, nib[16];
	for(int shift = 12; shift != 1; shift--) {
		uint32_t half = (uint32_t)(1 << shift) >> 1, err = 0;
		for(int i = 0; i < 16; i++) {
			int32_t r7 = (int32_t)s[i] >> 1;
			int32_t p = (int32_t)((((uint32_t)r7 << 17) >> 16) + half) >> shift;
			if(p > 7) p = 7;
			int32_t n = (int32_t)((((uint32_t)r7 | 0xFFFF8000u) << 1) + half) >> shift;
			if(n + 8 >= 0) { if(n > 7) n = 7; } else n = -8;
			int32_t recP = (int32_t)(int16_t)(((uint32_t)p << shift) & ~1u) >> 1;
			int32_t recN = (int32_t)(int16_t)(((uint32_t)n << shift) & ~1u) >> 1;
			int32_t eP = r7 - recP, eN = r7 - recN;
			int32_t sqP = (int32_t)((uint32_t)eP * (uint32_t)eP), sqN = (int32_t)((uint32_t)eN * (uint32_t)eN);
			if(sqP < sqN) { err += (uint32_t)sqP; nib[i] = (uint8_t)p; } else { err += (uint32_t)sqN; nib[i] = (uint8_t)(n & 0x0F); }
		}
		if((int32_t)err < best) {
			header = (uint8_t)(shift << 4);
			for(int k = 0; k < 8; k++) out[1 + k] = (uint8_t)((nib[k * 2] << 4) | nib[k * 2 + 1]);
			found = 1;
			best = (int32_t)err;
		}
	}
	if(found) out[0] = header;
	out[0] |= 0x02;
}

/* squared error of the decoded block (filter 0, shift <= 12) against the input */
static double block_err(const int32_t* s, const uint8_t* b)
{
	int sh = b[0] >> 4;
	double e = 0;
	for(int i = 0; i < 16; i++) {
		int n = (b[1 + i / 2] >> ((i & 1) ? 0 : 4)) & 15;
		if(n > 7) n -= 16;
		double d = (double)((s[i] >> 1) * 2) - (double)(((n << sh) >> 1) * 2);
		e += d * d;
	}
	return e;
}

static long total, same, worse;
static int fast_mode;   /* FAST=1: the encoder's fast mode (s0 only) */
static double err_ref, err_sw, sig;

static void check(const int32_t* s)
{
	uint8_t a[9], b[9];
	brr_ref(s, a);
	int16_t v[32];   /* both layouts of the mixer: mono (stride 1) and one channel of interleaved stereo (stride 2) */
	int stride = (int)(total & 1) + 1;
	for(int i = 0; i < 16; i++) v[i * stride] = (int16_t)s[i];
	uint32_t na, nb;
	b[0] = (uint8_t)brr_sw_encode(v, stride, &na, &nb, fast_mode);
	for(int k = 0; k < 4; k++) { b[1 + k] = (uint8_t)(na >> (24 - 8 * k)); b[5 + k] = (uint8_t)(nb >> (24 - 8 * k)); }
	total++;
	if(!memcmp(a, b, 9)) same++;
	double ea = block_err(s, a), eb = block_err(s, b);
	if(eb > ea) worse++;
	err_ref += ea; err_sw += eb;
	for(int i = 0; i < 16; i++) sig += (double)s[i] * s[i];
}

static void report(const char* what)
{
	printf("%s: %ld blocks, %ld identical (%.3f%%), software worse in %ld; SNR firmware %.2f dB, software %.2f dB\n", what,
		total, same, 100.0 * same / total, worse, 10 * log10(sig / err_ref), 10 * log10(sig / err_sw));
	total = same = worse = 0; err_ref = err_sw = sig = 0;
}

int main(int argc, char** argv)
{
	fast_mode = getenv("FAST") != NULL;
	for(int f = 1; f < argc; f++) {
		FILE* fp = fopen(argv[f], "rb");
		if(!fp) { perror(argv[f]); return 1; }
		int16_t blk[16];
		while(fread(blk, 2, 16, fp) == 16) {
			int32_t s[16];
			for(int i = 0; i < 16; i++) s[i] = blk[i];
			check(s);
		}
		fclose(fp);
		report(argv[f]);
	}
	if(argc == 1) {
		srand(1);
		for(int k = 0; k < 2000000; k++) {
			int32_t s[16];
			int amp = 1 << (rand() % 16);
			for(int i = 0; i < 16; i++) s[i] = (rand() % (2 * amp + 1)) - amp;
			check(s);
		}
		report("random");
	}
	return 0;
}
