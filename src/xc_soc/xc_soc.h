/* Xeno Crisis on sd2snes: SoC registers added around the core 0 soft CPU for the audio split.
 * Addresses are in RP2040 peripheral space that the RP2040 does not use. */
#ifndef XC_SOC_H
#define XC_SOC_H
#include <stdint.h>

/* BRR encoder (xc_brr.v) */
#define XC_BRR_BASE       0x50800000u
#define XC_BRR_SAMPLE(i)  (*(volatile uint32_t*)(XC_BRR_BASE + 4u * (i)))  /* write, low 16 bits */
#define XC_BRR_CTRL       (*(volatile uint32_t*)(XC_BRR_BASE + 0x40u))       /* write 1: start; read bit 0: busy */
#define XC_BRR_OUT(i)     (*(volatile uint32_t*)(XC_BRR_BASE + 0x44u + 4u * (i)))

/* Opus decode mailbox (xc_decbox.v; the MCU decodes) */
#define XC_DEC_BASE       0x50801000u
#define XC_DEC_CTRL       (*(volatile uint32_t*)(XC_DEC_BASE + 0x00u))
#define   XC_DEC_SUBMIT   1u   /* write: decode the packet in the buffer (read: bit 0 = job pending) */
#define   XC_DEC_ACK      2u   /* write: result consumed (read: bit 1 = result ready) */
#define   XC_DEC_RESET    4u   /* write: (re)initialize the decoder (24 kHz stereo) */
#define XC_DEC_LEN        (*(volatile uint32_t*)(XC_DEC_BASE + 0x04u))  /* packet length in bytes */
#define XC_DEC_RET        (*(volatile int32_t*)(XC_DEC_BASE + 0x08u))   /* opus_decode() result */
#define XC_DEC_RANGE      (*(volatile uint32_t*)(XC_DEC_BASE + 0x0Cu))  /* OPUS_GET_FINAL_RANGE after the packet */
#define XC_DEC_PACKET     ((volatile uint32_t*)(XC_DEC_BASE + 0x100u))   /* 1536 bytes */
#define XC_DEC_PCM        ((volatile uint32_t*)(XC_DEC_BASE + 0x800u))   /* 480 stereo int16 samples (1920 bytes) */

/* MSU-1 music: the same register block also has the MSU-1 audio control (0x10, 0x14). The music comes from an
 * MSU-1 pack on the SD card instead of the Opus decoder. */
#define   XC_DEC_MSU_MODE 8u   /* CTRL read: bit 3 set for MSU-1 music (mk3: set by the MCU with a pack; mk2: always) */
#define XC_MSU_REG        (*(volatile uint32_t*)(XC_DEC_BASE + 0x10u))  /* write: bits 2:0 register ($2000 + n), 15:8 value */
#define XC_MSU_STATUS     (*(volatile uint32_t*)(XC_DEC_BASE + 0x14u))  /* read: the MSU-1 status byte ($2000) */
#define   XC_MSU_ST_DATA_BUSY   0x80u
#define   XC_MSU_ST_AUDIO_BUSY  0x40u
#define   XC_MSU_ST_REPEAT      0x20u
#define   XC_MSU_ST_PLAYING     0x10u
#define   XC_MSU_ST_MISSING     0x08u

/* Mixer tick timer and debug/statistics port */
#define XC_TICK_BASE      0x50802000u
#define XC_TICK_PERIOD    (*(volatile uint32_t*)(XC_TICK_BASE + 0x00u))  /* microseconds, 0 = off */
#define XC_MIX_EVENT      (*(volatile uint32_t*)(XC_TICK_BASE + 0x08u))  /* write: event code (statistics only) */
#define   XC_EV_CALL_BEGIN  1u
#define   XC_EV_CALL_END    2u
#define   XC_EV_UNDERRUN    3u  /* music playing but no PCM for a block */
#define   XC_EV_DEFERRED    4u  /* mixing postponed: lock 0 held by core 0 */
#define   XC_EV_COOP_ENTER  0x10u /* mk2: xc_mix_poll_body() starts (simulation statistics) */
#define   XC_EV_COOP_LEAVE  0x11u /* mk2: xc_mix_poll_body() ends */

/* Debug port */
#define XC_DEBUG_CHAR     (*(volatile uint32_t*)(XC_TICK_BASE + 0x10u))  /* write: one character of debug output */
#define XC_PANIC          (*(volatile uint32_t*)(XC_TICK_BASE + 0x14u))  /* write: firmware panic / unsupported bootrom call (code) */

/* $3000 window (xc_stream.v with a DMA send side). Send: a queue of descriptors, each either {address, length}
 * read from memory by the DMA engine, or one immediate byte. The SNES reads them in order. */
#define XW_BASE           0x50803000u
#define XW_TX_ADDR        (*(volatile uint32_t*)(XW_BASE + 0x00u))  /* write: address of the next descriptor */
#define XW_TX_LEN         (*(volatile uint32_t*)(XW_BASE + 0x04u))  /* write: length; posts {TX_ADDR, length} */
#define XW_TX_PENDING     (*(volatile uint32_t*)(XW_BASE + 0x08u))  /* read: bytes queued and not yet read by the SNES */
#define XW_CTRL           (*(volatile uint32_t*)(XW_BASE + 0x0Cu))  /* write: bit 0 flush send side, bit 1 flush receive side */
#define XW_RX_DATA        (*(volatile uint32_t*)(XW_BASE + 0x10u))  /* read: bit 8 valid, bits 7:0 byte (popped) */
#define XW_RX_LEVEL       (*(volatile uint32_t*)(XW_BASE + 0x14u))  /* read: bytes waiting */
#define XW_TX_FREE        (*(volatile uint32_t*)(XW_BASE + 0x18u))  /* read: free descriptor slots */
#define XW_TX_BYTE        (*(volatile uint32_t*)(XW_BASE + 0x1Cu))  /* write: queue one immediate byte */

/* RP2040 timer (kept compatible: the firmware reads it too) */
#define TIMER_RAWH        (*(volatile uint32_t*)0x40054024u)
#define TIMER_RAWL        (*(volatile uint32_t*)0x40054028u)

/* Interrupts (RP2040 IRQ numbers 26-31 are unused) */
#define XC_IRQ_TICK       26
#define XC_IRQ_DEC        27

#endif
