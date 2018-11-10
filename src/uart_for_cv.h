#ifndef UART_FOR_CV
#define UART_FOR_CV

void rx_serial_init(void);
void read_serial(void);

#include "ch.h"
#include "hal.h"

extern uint8_t target;


#endif /* end of include guard: UART_FOR_CV */
