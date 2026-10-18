/* Host shim for src/uart.h: the firmware's debug prints are silent here (FatFs warns on
   every plain read above 512 bytes) */
#ifndef HOST_FOP_UART_H
#define HOST_FOP_UART_H
#include <stdio.h>
#define printf(...) ((void)0)
#endif
