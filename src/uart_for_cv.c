#include "ch.h"
#include "hal.h"
#include "uart_for_cv.h"
#include "oled.h"

uint8_t serial_rx_buffer[10] = {0};
uint8_t target = 1;


void rx_serial_init(void)
{
  static SerialConfig Serial_1_cfg =
  {
      9600, 					//Baudrate
      USART_CR1_PCE | USART_CR1_M,				//EVEN Parity bit
      USART_CR2_STOP1_BITS,
      0
  };
  sdStart(&SD1, &Serial_1_cfg);

}

void read_serial(void)
{

  sdRead(&SD1, serial_rx_buffer, 1);
  //sdWrite(&SD1, serial_rx_buffer, 3);
  target = serial_rx_buffer[0];
  OLED_ShowChar(0,0,serial_rx_buffer[0]);

  //OLED_ShowChar(0,2,serial_rx_buffer[1]);

  //OLED_ShowChar(0,4,serial_rx_buffer[2]);


}

void serial_task(void)
{
  //uint16_t = sd

}
