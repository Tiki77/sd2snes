#ifndef HOST_TIMER_H
#define HOST_TIMER_H
#include <stdint.h>
typedef uint32_t tick_t;
#define MS_TO_TICKS(x) ((x)/10)
tick_t getticks(void);
#endif
