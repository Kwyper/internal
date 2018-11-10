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

void turrent_task(pid_s_t* pid, uint8_t terret_state);

void turrent_basic_task(uint8_t terret_state);

//int32_t current_angleCalc(int32_t fromMotor);

int32_t setPoint_calc(uint8_t fromCV);

#endif /* SRC_TURRENT_H_ */
