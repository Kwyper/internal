#include "ch.h"
#include "hal.h"
#include "uart_for_cv.h"

uint8_t serial_rx_buffer[10];

void rx_serial_init()
{
  static SerialConfig Serial_1_cfg =
  {
      9600, 					//Baudrate
      USART_CR1_PCE | USART_CR1_M,				//EVEN Parity bit
      0,
      0
  };
  sdStart(&SD1, &Serial_1_cfg);

}

void decode_serial()
{


}

vpid serial_task()
{
  uint16_t = sd

}
