/*
 * serial_task.c
 *
 *  Created on: 2018Äê11ÔÂ10ÈÕ
 *      Author: Cresc
 */
#include "ch.h"
#include "hal.h"
#include "canBusProcess.h"
#include "dbus.h"
#include "oled.h"

#include "serial_task.h"
volatile uint8_t serial_buffer[4] = {0};
volatile uint8_t terret_state = 0;

uint8_t get_terret_state(void){
  return terret_state;
}

static THD_WORKING_AREA(serial_thread_wa,512);
static THD_FUNCTION(serial_thread, p)
{
  (void)p;
  while(!chThdShouldTerminateX())
  {
    sdRead(&SD1,serial_buffer,1);
    terret_state = serial_buffer[0];
    OLED_ShowNum(0,0,terret_state,1,16);
    chThdSleepMilliseconds(10);
  }
}

void serial_task_Init(void)
{
      OLED_Init();
      OLED_Clear();
      sdStop(&SD1);
      static const SerialConfig serial_rx_config = {
                                                    9600,
                                                    0,
                                                    USART_CR2_STOP1_BITS,
                                                    0,
      };
      sdStart(&SD1, &serial_rx_config);
      chThdCreateStatic(serial_thread_wa, sizeof(serial_thread_wa),
                               NORMALPRIO+1, serial_thread, NULL);
}

