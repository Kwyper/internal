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

#include "chassis_task.h"
#include "gripper_task.h"

#include <stdlib.h>


int16_t motor_output[4] = {0};        //Torque command for motors

pid_s_t wheel_pid[4];
pid_s_t gripper_pid;

static THD_WORKING_AREA(motor_ctrl_thread_wa,512);
static THD_FUNCTION(motor_ctrl_thread, p)
{
    (void) p;

    for(int i=0;i<4;i++)
    {
      pid_init(&wheel_pid[i],5.0f,0.03f,5.0f,1000.0f,12000.0f);
    }

	while(true)
	{

    chassis_task(wheel_pid);

		chThdSleepMilliseconds(4);
	}
}

static THD_WORKING_AREA(gripper_ctrl_thread_wa,512);
static THD_FUNCTION(gripper_ctrl_thread, p)
{
    (void) p;
  pid_init(&gripper_pid,0.3f,0.0f,4.0f,1000.0f,2000.0f);

	while(true)
	{
    gripper_task(&gripper_pid);
		chThdSleepMilliseconds(4);
	}
}


int main(void)
{


    halInit();
    chSysInit();

    RC_init();
    can_processInit();
    gripper_init();



    chThdCreateStatic(gripper_ctrl_thread_wa, sizeof(gripper_ctrl_thread_wa),
		  	  	  	 NORMALPRIO, gripper_ctrl_thread, NULL);

    chThdCreateStatic(motor_ctrl_thread_wa, sizeof(motor_ctrl_thread_wa),
             		 NORMALPRIO, motor_ctrl_thread, NULL);


    while (true)
    {
        palTogglePad(GPIOA, GPIOA_LED);
        chThdSleepMilliseconds(500);
    }
}
