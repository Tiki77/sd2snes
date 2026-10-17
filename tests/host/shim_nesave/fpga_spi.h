/* host stand-in for src/fpga_spi.h: a fake FPGA that speaks the three PSRAM commands nesave.c uses
   (set address, read with auto-increment, write) over a flat fake PSRAM, and whose MCU_RDY line can be
   made to hang.  The unbounded wait ends the process: the whole point of the gate is that nothing in
   nesave.c may use it. */
#ifndef HOST_FPGA_SPI_H
#define HOST_FPGA_SPI_H
#include <stdint.h>
#define FPGA_MEM_AUTOINC   (0x8)
#define FPGA_TGT_MEM       (0x0)
#define FPGA_CMD_SETADDR   (0x00)
#define FPGA_CMD_READMEM   (0x80)
uint8_t fpga_test(void);
void    fake_select(void);
void    fake_deselect(void);
void    fake_tx(uint8_t b);
uint8_t fake_rx(void);
int     fake_wait(int bounded);        /* 1 = timed out (bounded) */
#define FPGA_SELECT()      fake_select()
#define FPGA_DESELECT()    fake_deselect()
#define FPGA_TX_BYTE(x)    fake_tx(x)
#define FPGA_RX_BYTE()     fake_rx()
#define FPGA_WAIT_RDY()                  ((void)fake_wait(0))
#define FPGA_WAIT_RDY_TO(f)              do { if(fake_wait(1)) (f) = 1; } while(0)
#endif
