/*
 * turrent.h
 *
 *  Created on: 2018Äê11ÔÂ10ÈÕ
 *      Author: Cresc
 */

#include "canBusProcess.h"
#include "dBus.h"

#ifndef SRC_TURRENT_H_
#define SRC_TURRENT_H_

typedef struct{
  float kp;
  float ki;
  float kd;

  float p_out;
  float i_out;
  float d_out;

  float last_err;

  float pid_out;

  float max_integral;
  float max_pid_out;

}pid_s_t;

//bool shouldHit = false;

void pid_init(pid_s_t *pid,float kp,float ki,float kd,uint32_t max_integral,uint32_t max_pid_out);

float pid_cal(pid_s_t *pid,const float set,const float get);

void turrent_task(pid_s_t* pid, Encoder_canStruct* encoder, int16_t setPoint);

#endif /* SRC_TURRENT_H_ */
