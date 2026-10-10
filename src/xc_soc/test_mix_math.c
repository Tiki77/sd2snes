/* The mixer's division-free rounding helpers must equal the firmware's division-based ones for all inputs. */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
static inline int32_t div_pow2(int32_t x, int k) { return (x + (int32_t)((uint32_t)(x >> 31) >> (32 - k))) >> k; }
static inline int32_t qmul(int32_t a, int32_t b) { int32_t p = (int32_t)((uint32_t)a * (uint32_t)b); int32_t q = div_pow2(p, 14); int32_t h = div_pow2(q, 1); return h + (q - 2 * h); }
static inline int32_t clamp(int32_t x) { if((uint32_t)x + 0x7FFFu > 0xFFFEu) return (int32_t)((uint32_t)div_pow2(x, 15) * 32760u); return x; }
static int32_t ref_qmul(int32_t a, int32_t b) { int32_t p = (int32_t)((uint32_t)a * (uint32_t)b); int32_t q = p / 16384; return q / 2 + q % 2; }
static int32_t ref_clamp(int32_t x) { if((uint32_t)x + 0x7FFF > 0xFFFE) { int32_t q = x / 32768; return (int32_t)((uint32_t)q * 32760u); } return x; }
static uint64_t s = 88172645463325252ull;
static uint32_t rnd(void) { s ^= s << 13; s ^= s >> 7; s ^= s << 17; return (uint32_t)s; }
int main(void)
{
	uint64_t bad = 0, n = 0;
	static const int32_t edge[] = { 0, 1, -1, 16383, 16384, -16384, -16385, 32767, 32768, -32767, -32768, -32769, 0x7FFFFFFF, (int32_t)0x80000000, 0x3333, 0x6666, 0x4AAA, -2, 2 };
	for(int i = 0; i < 19; i++) for(int j = 0; j < 19; j++) {
		n++; if(qmul(edge[i], edge[j]) != ref_qmul(edge[i], edge[j])) bad++;
	}
	for(int i = 0; i < 19; i++) { n++; if(clamp(edge[i]) != ref_clamp(edge[i])) bad++; n++; if(div_pow2(edge[i], 15) != edge[i] / 32768) bad++; }
	for(uint64_t i = 0; i < 200000000ull; i++) {
		int32_t a = (int32_t)rnd(), b = (int32_t)rnd();
		if((i & 3) == 1) { a = (int16_t)a; b = (i & 4) ? 0x6666 : (int16_t)b; }
		n += 3;
		if(qmul(a, b) != ref_qmul(a, b)) bad++;
		if(clamp(a) != ref_clamp(a)) bad++;
		if(div_pow2(a, 15) != a / 32768) bad++;
	}
	/* exhaustive over the mixer's real operand range: every int16 sample times each volume/scale constant */
	static const int32_t vols[] = { 0x6666, 0x3333 };
	for(int v = 0; v < 2; v++) for(int32_t x = -32768; x <= 32767; x++) { n++; if(qmul(x, vols[v]) != ref_qmul(x, vols[v])) bad++; }
	for(int64_t x = -70000; x <= 70000; x++) { n++; if(clamp((int32_t)x) != ref_clamp((int32_t)x)) bad++; }
	printf("%llu comparisons, %llu differences\n", (unsigned long long)n, (unsigned long long)bad);
	return bad != 0;
}
