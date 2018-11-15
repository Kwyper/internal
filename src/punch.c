/*
 * punch.c
 *
 *  Created on: 2018Äê11ÔÂ10ÈÕ
 *      Author: Cresc
 */
#include "ch.h"
#include "hal.h"
#include"punch.h"

bool shouldPunch = false;

bool* getPunchSituation(){
  return &shouldPunch;
}

void punch(){

        int timest = chVTGetSystemTime();
        palSetPad(GPIOB,8);//high voltage
        chThdSleepMilliseconds(500);
        palClearPad(GPIOB,8);
        shouldPunch = false;
}

static THD_WORKING_AREA(punch_thread_wa,512);
static THD_FUNCTION(punch_thread, p)
{
  (void)p;
  while(!chThdShouldTerminateX())
  {
    punch();
    chThdSleepMilliseconds(10);
  }
}

void punch_Init(){
  chThdCreateStatic(punch_thread_wa, sizeof(punch_thread_wa),
                                 NORMALPRIO, punch_thread, NULL);
}


