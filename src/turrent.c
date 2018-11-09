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

float pid_calc(pid_s_t* pid, const int16_t set,const int16_t get)
{
  float err = set - get;

  pid->d_out = pid->kp*err;
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

void drive_turrent(pid_s_t* pid, Encoder_canStruct* encoder, int16_t setPoint)
{
    float finalOutput = pid_calc(pid, setPoint, encoder->angle_rotor_raw);
    can_motorSetCurrent(0x200,finalOutput,0,0,0);
}


