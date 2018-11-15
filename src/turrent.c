/*
 * turrent.c
 *
 *  Created on: 2018Äê11ÔÂ10ÈÕ
 *      Author: Cresc
 */
#include "ch.h"
#include "hal.h"
#include "canBusProcess.h"
#include "dbus.h"
#include "punch.h"

#include "turrent.h"
#include "oled.h"

int32_t record_pos[3] = {0};

int16_t lastSituation = 0;

bool flag_isDiff = false;

void pid_init(pid_s_t *pid,float kp,float ki,float kd,uint32_t max_integral,uint32_t max_pid_out)
{
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->i_out = 0;
  pid->last_err = 0;

  pid->max_integral = max_integral;
  pid->max_pid_out = max_pid_out;
}



float pid_calc(pid_s_t* pid, const float set,const float get)
{
  float err = set - get;

  pid->p_out = pid->kp*err;
  pid->i_out += pid->ki*err;
  pid->d_out = pid->kd*(err - pid->last_err);

  pid->last_err = err;

  if(pid->i_out > pid->max_integral)  pid->i_out = pid->max_integral;
  if(pid->i_out < -pid->max_integral)  pid->i_out = -pid->max_integral;

  pid->pid_out = pid->p_out + pid->i_out + pid->d_out;

  if(pid->pid_out > pid->max_pid_out) pid->pid_out = pid->max_pid_out;
  if(pid->pid_out < -pid->max_pid_out) pid->pid_out = -pid->max_pid_out;

  return pid->pid_out;
}


int32_t setPoint_calc(uint8_t fromCV){
    if(fromCV == 1 || fromCV == 51){
      return record_pos[0];
    }
    else if(fromCV == 2 || fromCV == 52){
      return record_pos[1];
    }
    else if(fromCV == 3 || fromCV == 53){
      return record_pos[2];
    }
    else{
      return record_pos[0];
    }
}


void turrent_task(pid_s_t* pid, uint8_t terret_state)
{
//    if(terret_state != lastSituation){
//      flag_isDiff = true;
//      lastSituation = terret_state;
//    }else if(terret_state == lastSituation){
//      flag_isDiff = false;
//    }
    volatile Encoder_canStruct* encoder = can_getEncoder();
    int32_t setPoint = setPoint_calc(terret_state);
    int32_t currentPoint = encoder->total_ecd;
    float finalOutput = pid_calc(pid, setPoint, currentPoint);
    can_motorSetCurrent(0x200,finalOutput,0,0,0);

    /*if(setPoint - currentPoint < 1500 && currentPoint - setPoint > -1500){
      punch();
      }
      else{
      }*/
}

void turrent_basic_task(uint8_t terret_state)
{
  if(terret_state == 1){
    can_motorSetCurrent(0x200,500,0,0,0);
  }
}

void turrent_calibrate(void)
{
  button_state_t button_state = UP;
  uint8_t cursor = 0;
  systime_t gap_time;
  while(true)
  {

    OLED_ShowNum(0,2*cursor,can_getEncoder()->total_ecd,6,16);
    if(!palReadPad(GPIOA,GPIOA_BUTTON)&&button_state==UP)
    {
       button_state = DOWN;
       gap_time = chVTGetSystemTime();
    }
    if(palReadPad(GPIOA,GPIOA_BUTTON)&&button_state==DOWN)
    {
       if(chVTGetSystemTime() - gap_time > TIME_MS2I(1000)) button_state = HOLD;
       else if(chVTGetSystemTime() - gap_time > TIME_MS2I(100)) button_state = PRESSED;
       else button_state = UP;
    }

    if(button_state==HOLD) {
      button_state = UP;
    }

    if(button_state==PRESSED)
    {
      record_pos[cursor++] = can_getEncoder()->total_ecd;
      if(cursor >= 3){
        OLED_Clear();
        OLED_ShowString(0,2,"finish");
        break;
      }
      button_state = UP;
    }
  }
}
