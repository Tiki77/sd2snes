/* Replacement RP2040 bootrom for the sd2snes SoC (linked at 0x00000000, 16 KB).
 *
 * Only what the pico-sdk runtime looks up is provided. Behaviour follows MesenCE's HLE bootrom
 * (Rp2040.cpp HleBootRomFunc), so a run with this bootrom can be compared with an HLE run:
 *   - rom_table_lookup; P3/R3/L3/T3 bit functions; MS/S4 memset; MC/C4 memcpy; IF/EX/FC/CX return 0
 *   - soft-float / soft-double tables: IEEE arithmetic and conversions via libgcc (exactly rounded, like
 *     the host's float/double operations MesenCE uses), with the same saturation rules;
 *     sqrt, trigonometry, exp/log and atan2 raise XC_PANIC with a code: the game never calls them
 *     during play, and a different implementation would not match the real bootrom bit for bit anyway.
 */
#include <stdint.h>
#include "xc_soc.h"

#define TRAP(code) do { XC_PANIC = 0xB0070000u | (code); for(;;) { } } while(0)

/* ---- table lookup: rom_table_lookup(const uint16_t* table, uint32_t code) ---- */
uint32_t xc_rom_table_lookup(const uint16_t* table, uint32_t code)
{
	for(int i = 0; i < 64; i++) {
		uint16_t c = table[i * 2];
		if(c == 0) break;
		if(c == code) return table[i * 2 + 1];
	}
	return 0;
}

/* ---- bit functions ---- */
uint32_t rom_popcount32(uint32_t v)
{
	v = v - ((v >> 1) & 0x55555555u);
	v = (v & 0x33333333u) + ((v >> 2) & 0x33333333u);
	return (((v + (v >> 4)) & 0x0F0F0F0Fu) * 0x01010101u) >> 24;
}
uint32_t rom_reverse32(uint32_t v)
{
	v = ((v >> 1) & 0x55555555u) | ((v & 0x55555555u) << 1);
	v = ((v >> 2) & 0x33333333u) | ((v & 0x33333333u) << 2);
	v = ((v >> 4) & 0x0F0F0F0Fu) | ((v & 0x0F0F0F0Fu) << 4);
	v = ((v >> 8) & 0x00FF00FFu) | ((v & 0x00FF00FFu) << 8);
	return (v >> 16) | (v << 16);
}
uint32_t rom_clz32(uint32_t v)
{
	if(!v) return 32;
	uint32_t n = 0;
	if(!(v & 0xFFFF0000u)) { n += 16; v <<= 16; }
	if(!(v & 0xFF000000u)) { n += 8; v <<= 8; }
	if(!(v & 0xF0000000u)) { n += 4; v <<= 4; }
	if(!(v & 0xC0000000u)) { n += 2; v <<= 2; }
	if(!(v & 0x80000000u)) { n += 1; }
	return n;
}
uint32_t rom_ctz32(uint32_t v)
{
	if(!v) return 32;
	return 31 - rom_clz32(v & (0u - v));
}

/* ---- memory functions (hot: about 3,300 memcpy calls per second, 24 bytes on average) ---- */
void* rom_memset(void* dst, uint32_t c, uint32_t n)
{
	uint8_t* d = (uint8_t*)dst;
	uint8_t b = (uint8_t)c;
	while(n && ((uint32_t)d & 3)) { *d++ = b; n--; }
	uint32_t w = b * 0x01010101u;
	while(n >= 16) { ((uint32_t*)d)[0] = w; ((uint32_t*)d)[1] = w; ((uint32_t*)d)[2] = w; ((uint32_t*)d)[3] = w; d += 16; n -= 16; }
	while(n >= 4) { *(uint32_t*)d = w; d += 4; n -= 4; }
	while(n) { *d++ = b; n--; }
	return dst;
}
void* rom_memcpy(void* dst, const void* src, uint32_t n); /* xc_bootrom_memcpy.S */
uint32_t rom_return0(void) { return 0; }
void rom_unsupported(void) { TRAP(0xFFFF); }

/* ---- soft float ---- */
typedef union { float f; uint32_t u; } fu;
typedef union { double d; uint64_t u; struct { uint32_t lo, hi; } w; } du;

static int isnan_f(float a) { fu x; x.f = a; return (x.u & 0x7FFFFFFFu) > 0x7F800000u; }
static int isnan_d(double a) { du x; x.d = a; return (x.u & 0x7FFFFFFFFFFFFFFFull) > 0x7FF0000000000000ull; }
/* scale a double by 2^e exactly (no overflow/underflow handling needed for the fixed-point ranges used) */
static double scale2(double v, int e)
{
	du x; x.d = v;
	if((x.u & 0x7FF0000000000000ull) == 0) return v;         /* zero (or subnormal): leave */
	if((x.u & 0x7FF0000000000000ull) == 0x7FF0000000000000ull) return v; /* inf, NaN: leave */
	int64_t ex = (int64_t)((x.u >> 52) & 0x7FF) + e;
	if(ex <= 0) { x.u &= 0x8000000000000000ull; return x.d; } /* underflow to signed zero */
	if(ex >= 0x7FF) { x.u = (x.u & 0x8000000000000000ull) | 0x7FF0000000000000ull; return x.d; }
	x.u = (x.u & ~0x7FF0000000000000ull) | ((uint64_t)ex << 52);
	return x.d;
}
/* MesenCE SatCast: NaN -> 0, clamp to range, truncate towards zero */
static int32_t sat_i32(double v) { if(isnan_d(v)) return 0; if(v <= -2147483648.0) return (int32_t)0x80000000; if(v >= 2147483647.0) return 0x7FFFFFFF; return (int32_t)v; }
static uint32_t sat_u32(double v) { if(isnan_d(v)) return 0; if(v <= 0.0) return 0; if(v >= 4294967295.0) return 0xFFFFFFFFu; return (uint32_t)v; }
static int64_t sat_i64(double v) { if(isnan_d(v)) return 0; if(v <= -9223372036854775808.0) return (int64_t)0x8000000000000000ull; if(v >= 9223372036854775807.0) return 0x7FFFFFFFFFFFFFFFll; return (int64_t)v; }
static uint64_t sat_u64(double v) { if(isnan_d(v)) return 0; if(v <= 0.0) return 0; if(v >= 18446744073709551615.0) return 0xFFFFFFFFFFFFFFFFull; return (uint64_t)v; }

static float f_add(float a, float b) { return a + b; }
static float f_sub(float a, float b) { return a - b; }
static float f_mul(float a, float b) { return a * b; }
static float f_div(float a, float b) { return a / b; }
static int f_cmp(float a, float b) { return (isnan_f(a) || isnan_f(b)) ? 1 : (a < b ? -1 : (a > b ? 1 : 0)); }
int xc_f_cmp_code(float a, float b) { return (isnan_f(a) || isnan_f(b)) ? 2 : (a < b ? -1 : (a > b ? 1 : 0)); }
extern void xc_f_cmp_fast_flags(void); /* xc_bootrom_flags.S */
static int32_t f_to_i(float a) { return sat_i32((double)a); }
static int32_t f_to_fix(float a, int32_t n) { return sat_i32(scale2((double)a, n)); }
static uint32_t f_to_u(float a) { return sat_u32((double)a); }
static uint32_t f_to_ufix(float a, int32_t n) { return sat_u32(scale2((double)a, n)); }
static float i_to_f(int32_t v) { return (float)v; }
static float fix_to_f(int32_t v, int32_t n) { return (float)scale2((double)v, -n); }
static float u_to_f(uint32_t v) { return (float)v; }
static float ufix_to_f(uint32_t v, int32_t n) { return (float)scale2((double)v, -n); }
static float i64_to_f(int64_t v) { return (float)v; }
static float u64_to_f(uint64_t v) { return (float)v; }
static int64_t f_to_i64(float a) { return sat_i64((double)a); }
static uint64_t f_to_u64(float a) { return sat_u64((double)a); }
static double f_to_d(float a) { return (double)a; }
static void f_sqrt(void) { TRAP(0x0118); }
static void f_trig(void) { TRAP(0x013C); }
static void f_exp(void) { TRAP(0x014C); }
static void f_ln(void) { TRAP(0x0150); }
static void f_atan2(void) { TRAP(0x0158); }

static double d_add(double a, double b) { return a + b; }
static double d_sub(double a, double b) { return a - b; }
static double d_mul(double a, double b) { return a * b; }
static double d_div(double a, double b) { return a / b; }
static int d_cmp(double a, double b) { return (isnan_d(a) || isnan_d(b)) ? 1 : (a < b ? -1 : (a > b ? 1 : 0)); }
static int32_t d_to_i(double a) { return sat_i32(a); }
static uint32_t d_to_u(double a) { return sat_u32(a); }
static double i_to_d(int32_t v) { return (double)v; }
static double u_to_d(uint32_t v) { return (double)v; }
static float d_to_f(double a) { return (float)a; }
static void d_sqrt(void) { TRAP(0x0218); }
static void d_trig(void) { TRAP(0x023C); }
static void d_exp(void) { TRAP(0x024C); }
static void d_ln(void) { TRAP(0x0250); }
static void d_atan2(void) { TRAP(0x0258); }

#define E(f) ((uint32_t)(f))
#define U E(rom_unsupported)
/* table offsets as in MesenCE HleBootRomFunc (entry = offset / 4) */
__attribute__((used)) const uint32_t xc_rom_sf_table[32] = {
	E(f_add), E(f_sub), E(f_mul), E(f_div), E(f_cmp), E(xc_f_cmp_fast_flags), E(f_sqrt), E(f_to_i),
	E(f_to_fix), E(f_to_u), E(f_to_ufix), E(i_to_f), E(fix_to_f), E(u_to_f), E(ufix_to_f), E(f_trig),
	E(f_trig), E(f_trig), U, E(f_exp), E(f_ln), E(f_cmp), E(f_atan2), E(i64_to_f),
	U, E(u64_to_f), U, E(f_to_i64), U, E(f_to_u64), U, E(f_to_d),
};
__attribute__((used)) const uint32_t xc_rom_sd_table[32] = {
	E(d_add), E(d_sub), E(d_mul), E(d_div), E(d_cmp), U, E(d_sqrt), E(d_to_i),
	U, E(d_to_u), U, E(i_to_d), U, E(u_to_d), U, E(d_trig),
	E(d_trig), E(d_trig), U, E(d_exp), E(d_ln), E(d_cmp), E(d_atan2), U,
	U, U, U, U, U, U, U, E(d_to_f),
};

/* The 16-bit function/data tables and the header are in xc_bootrom_tables.S (a C initializer cannot
 * truncate an address to 16 bits). The functions they point to: */
void* const xc_rom_funcs[] = {
	(void*)rom_popcount32, (void*)rom_reverse32, (void*)rom_clz32, (void*)rom_ctz32, (void*)rom_memset, (void*)rom_memcpy,
	(void*)rom_return0, (void*)rom_unsupported, (void*)xc_rom_table_lookup,
};
const char xc_rom_copyright[] = "(C) 2020 Raspberry Pi Trading Ltd";
