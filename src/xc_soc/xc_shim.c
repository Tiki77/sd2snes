/* Xeno Crisis on sd2snes: replacements for the firmware functions MesenCE used to run natively (HLE).
 * Each keeps the behaviour of the MesenCE HLE hook it replaces (Rp2040.cpp RunHook), on the sd2snes SoC:
 *   - SNES bus layer (PIO + DMA on the RP2040) -> the $3000 window's descriptor queue and RX FIFO;
 *   - sleep_until, bus_recv timeouts -> the RP2040-compatible microsecond timer;
 *   - puts/printf -> debug port; panic -> panic register;
 *   - flash_do_cmd / flash_range_erase / flash_range_program -> the save area of the flash image, which the
 *     SoC maps writable (on sd2snes it is the cartridge save RAM, saved to the SD card by the MCU).
 * Patched into the firmware by entry-point veneers (see xc_fw_header.c). */
#include <stdint.h>
#include "xc_soc.h"

void xc_mix_poll(void);   /* mk2: runs the mixer ticks that are due (xc_mix.c); nothing on mk3 */

static uint64_t now_us(void)
{
	uint32_t hi, lo;
	do {
		hi = TIMER_RAWH;
		lo = TIMER_RAWL;
	} while(hi != TIMER_RAWH);
	return ((uint64_t)hi << 32) | lo;
}

/* ---- SNES bus layer (firmware 0x10066D94..0x10067274) ---- */
int xc_bus_init(void* cfg)
{
	(void)cfg;
	return 0;
}

int xc_bus_flush(void)
{
	XW_CTRL = 3;
	return 0;
}

int xc_bus_wait_tx(void)
{
	/* the RP2040 version waits for its TX DMA; the PIO FIFO holds 8 more bytes */
	while(XW_TX_PENDING > 8) {
		xc_mix_poll();
	}
	return 0;
}

/* r1: buffer descriptor {+0 base, +8 offset, +0xC length} */
int xc_bus_send(void* unused, volatile uint32_t* desc)
{
	(void)unused;
	uint32_t len = desc[3];
	if(len == 0) {
		return 0;
	}
	while(XW_TX_PENDING > 8 || XW_TX_FREE == 0) {
		xc_mix_poll();
	}
	uint32_t off = desc[2];
	XW_TX_ADDR = desc[0] + off;
	XW_TX_LEN = len;
	desc[2] = off + len + 16 - (len & 15);
	desc[3] = 0;
	return 0;
}

/* read up to count bytes written by the SNES; timeout in ms (-1 = forever); returns the number read */
int xc_bus_recv(void* unused, uint8_t* dst, uint32_t count, uint32_t timeout_ms)
{
	(void)unused;
	uint64_t start = now_us();
	uint32_t got = 0;
	for(;;) {
		while(got < count) {
			uint32_t v = XW_RX_DATA;
			if(!(v & 0x100)) {
				break;
			}
			dst[got++] = (uint8_t)v;
		}
		if(got >= count) {
			return (int)got;
		}
		if(timeout_ms != 0xFFFFFFFFu && now_us() - start >= (uint64_t)timeout_ms * 1000) {
			return (int)got;
		}
		xc_mix_poll();
	}
}

int xc_bus_push(void* unused, const uint8_t* src, uint32_t count)
{
	(void)unused;
	for(uint32_t i = 0; i < count; i++) {
		XW_TX_BYTE = src[i];
	}
	return 0;
}

/* ---- pico-sdk functions ---- */
void xc_sleep_until(uint64_t t)
{
	while(now_us() < t) {
		xc_mix_poll();
	}
}

int xc_puts(const char* s)
{
	while(*s) {
		XC_DEBUG_CHAR = (uint8_t)*s++;
	}
	XC_DEBUG_CHAR = '\n';
	return 0;
}

/* printf is not called by core 0 during play; the format string is passed through unformatted */
int xc_printf(const char* fmt, ...)
{
	while(*fmt) {
		XC_DEBUG_CHAR = (uint8_t)*fmt++;
	}
	return 0;
}

void xc_panic(const char* fmt, ...)
{
	XC_PANIC = (uint32_t)fmt;
	for(;;) {
	}
}

/* ---- flash (saves live in the last 32 KB) ---- */
#define FLASH_BASE 0x10000000u
#define FLASH_SIZE 0x01000000u

void xc_flash_do_cmd(const uint8_t* tx, uint8_t* rx, uint32_t count)
{
	static const uint8_t unique_id[8] = { 0xE6, 0x60, 0x58, 0x38, 0x83, 0x2B, 0x44, 0x2A };
	static const uint8_t jedec[3] = { 0xEF, 0x40, 0x18 }; /* W25Q128 */
	uint8_t cmd = count ? tx[0] : 0;
	for(uint32_t i = 0; i < count; i++) {
		uint8_t v = 0;
		if(cmd == 0x4B && i >= 5 && i < 13) v = unique_id[i - 5];
		else if(cmd == 0x9F && i >= 1 && i < 4) v = jedec[i - 1];
		rx[i] = v;
	}
}

void xc_flash_range_erase(uint32_t offset, uint32_t count)
{
	if(offset + count > FLASH_SIZE) return;
	volatile uint8_t* p = (volatile uint8_t*)(FLASH_BASE + offset);
	for(uint32_t i = 0; i < count; i++) p[i] = 0xFF;
}

void xc_flash_range_program(uint32_t offset, const uint8_t* data, uint32_t count)
{
	if(offset + count > FLASH_SIZE) return;
	volatile uint8_t* p = (volatile uint8_t*)(FLASH_BASE + offset);
	for(uint32_t i = 0; i < count; i++) p[i] &= data[i]; /* NOR flash: programming only clears bits */
}
