/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @brief This file is based on the MAPLE MINI example from ChibiOS
 *
 * @file main.cpp
 * @author Alex Au
 * @date 2018-09-11
 */

#include "ch.h"
#include "hal.h"

#include "dbus.h"
#include "canBusProcess.h"
#include "chassis_task.h"
#include "configure.h"


#include "uart_for_cv.h"
#include "turrent.h"
#include <stdlib.h>
#include "oled.h"

pid_s_t terret_pid[2];

static THD_WORKING_AREA(turrent_ctrl_thread_wa,512);
static THD_FUNCTION(turrent_ctrl_thread, p)
{
    (void) p;

    pid_init(&terret_pid[0],0.5f,0.0f,5.0f,1000.0f,2000.0f);
    pid_init(&terret_pid[1],0.3f,0.0f,4.0f,1000.0f,8000.0f);
    turrent_calibrate();

	while(true)
	{
    turrent_task(terret_pid);
		chThdSleepMilliseconds(5);
	}
}

static THD_WORKING_AREA(serial_ctrl_thread_wa,512);
static THD_FUNCTION(serial_ctrl_thread, p)
{
    (void) p;

    while(true)
  	{
      read_serial();

  		chThdSleepMilliseconds(5);
  	}
}

/*
 * Application entry point.
 */
int main(void)
{


    halInit();
    chSysInit();

    RC_init();
    can_processInit();
    OLED_Init();
    OLED_Clear();
    rx_serial_init();




    chThdCreateStatic(turrent_ctrl_thread_wa, sizeof(turrent_ctrl_thread_wa),
		  	  	  	 NORMALPRIO, turrent_ctrl_thread, NULL);

    chThdCreateStatic(serial_ctrl_thread_wa, sizeof(serial_ctrl_thread_wa),
             		 NORMALPRIO, serial_ctrl_thread, NULL);



    while (true)
    {
        palTogglePad(GPIOA, GPIOA_LED);
        chThdSleepMilliseconds(50);
    }
}
