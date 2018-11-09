#ifndef SRC_PID_H_
#define SRC_PID_H_

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

void pid_init(pid_s_t *pid,float kp,float ki,float kd,uint32_t max_integral,uint32_t max_pid_out);

float pid_cal(pid_s_t *pid,const float set,const float get);
