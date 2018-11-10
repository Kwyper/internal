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

#include "turrent.h"

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
      return 12345;
    }
    else if(fromCV == 2 || fromCV == 52){
      return 34567;
    }
    else if(fromCV == 3 || fromCV == 53){
      return 45678;
    }
    else{
      return 0;
    }
}

/*int32_t current_angleCalc(int32_t fromMotor)
{
    while(fromMotor - (27*8192) > 0){
      fromMotor -= 27*8192;
    }
    return fromMotor;
}*/


void turrent_task(pid_s_t* pid, uint8_t terret_state)
{
    volatile Encoder_canStruct* encoder = can_getEncoder();
    int32_t setPoint = setPoint_calc(terret_state);
    int32_t currentPoint = encoder->total_ecd;
    float finalOutput = pid_calc(pid, setPoint, currentPoint);
    can_motorSetCurrent(0x200,finalOutput,0,0,0);
}

void turrent_basic_task(uint8_t terret_state)
{
  if(terret_state == 1){
    can_motorSetCurrent(0x200,500,0,0,0);
  }
}
