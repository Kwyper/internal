#include "chassis_task.h"


chassis_speed_t chassis_speed;   //speed of chassis
int16_t motor_speed_sp[4];      //speed set-point for motor


void abs_limit(float *data,float max)
{
  if(*data > max) *data = max;
  if(*data < -max) *data = -max;

}


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

float pid_calcu(pid_s_t *pid,float set,float get)
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

//pid部分结束

void get_chassis_speed(void)
{
  chassis_speed.vx = RC_Ctl.channel0 * VX_AMPLIFICATION_TATIO/RC_MAC_RANGE * MAX_CHASSIS_VX_SPEED;
  chassis_speed.vy = RC_Ctl.channel1 * VY_AMPLIFICATION_TATIO/RC_MAC_RANGE * MAX_CHASSIS_VY_SPEED;
  chassis_speed.vw = RC_Ctl.channel2 * VW_AMPLIFICATION_TATIO/RC_MAC_RANGE * MAX_CHASSIS_VW_SPEED;
}

void drive_meccanum(const int16_t vx, const int16_t vy, const int16_t vw)
{
 motor_speed_sp[0] = (+vx - vy + vw * rotate_ratio_f) * motor_speed_sp_ratio;
 motor_speed_sp[1] = (+vx + vy + vw * rotate_ratio_f) * motor_speed_sp_ratio;
 motor_speed_sp[2] = (-vx + vy + vw * rotate_ratio_f) * motor_speed_sp_ratio;
 motor_speed_sp[3] = (-vx - vy + vw * rotate_ratio_f) * motor_speed_sp_ratio;
}