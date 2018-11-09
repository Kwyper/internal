#include "ch.h"
#include "hal.h"
#include "canBusProcess.h"
#include "dbus.h"

#include "gripper.h"

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

/*static float angle_pid_control(const float setPoint,
                               const float currentPoint)
{
    float output;
    float error = setPoint - currentPoint;
    errorSum_angle += error;
    float errorDiff = error - preError_angle;
    preError_angle = error;

    if(errorSum_angle > 50.0f){
          errorSum_angle = 50.0f;
    }else if(errorSum_angle < -50.0f){
          errorSum_angle = -50.0f;
    }
    //limit Sum
    //to be changed: the range

    int16_t pidp = error*kp_angle;
    int16_t pidi = ki_angle*errorSum_angle*0.73f;
    int16_t pidd = kd_angle*errorDiff;

    output = pidp + pidi + pidd;

    if (error <= 5.0f && error >= -5.0f){
      output = 0;
    }
    //to be changed: the range

    if(output > 1000)
        output = 1000;
    else if(output < -1000)
        output = -1000;
    //to be changed: the range

    return output;
    //return a float so that it can be received by speed_pid_control()
}*/

static int16_t speed_pid_control(const float setPoint_fromAngle,
                                 const float currentPoint_fromMotor)
{
    int16_t output;
    float error = setPoint_fromAngle - currentPoint_fromMotor;
    errorSum_speed += error;
    float errorDiff = error - preError_speed;
    preError_speed = error;

    if(errorSum_speed > 10.0f){
          errorSum_speed = 10.0f;
    }else if(errorSum_speed < -10.0f){
          errorSum_speed = -10.0f;
    }

    int16_t pidp = kp_speed*error;
    int16_t pidi = ki_speed*errorSum_speed*0.73f;
    int16_t pidd = kd_speed*errorDiff;

    output = (int)(pidp + pidi + pidd);
    //speed_pid_control should out put a current value to be passed to motor_set_current()

    if(output > 1000)
        output = 1000;
    else if(output < -1000)
        output = -1000;
    // all the ranges above are to be changed
    return output;
}

static int16_t pid_control_all(const float setPoint_forAngle,
                               const float currentPoint_fromMotor,
                               const float currentSpeed_fromMotor)
{
    int16_t output;
    int16_t setPoint_fromAngle = angle_pid_control(setPoint_forAngle,
                                                   currentPoint_fromMotor);
    setPoint_fromAngle *= (6000.0 / 6.28318f);
    //setPoint_fromAngle: a speed setPoint to be passed to speed_pid_control()

    output = speed_pid_control(setPoint_fromAngle,
                               currentSpeed_fromMotor);
    return output;
}

static float currentAngleCalcu(float radian_angle){
    while((radian_angle - 27*8192) > 0){
      radian_angle -= 27*8192;
    }
    radian_angle *= 7.669904e-4f;
    return radian_angle;
}

static THD_WORKING_AREA(motor_ctrl_thread_wa,512);
static THD_FUNCTION(motor_ctrl_thread, p)
{
    (void) p;
    //volatile RC_Ctl_t* rc = RC_get();
    //pid_init(&pid,7.5f,0.03f,0.0f,1000.0f,12000.0f);
	while(true)
	{
	  volatile Encoder_canStruct* encoder = can_getEncoder();
	  angle = currentAngleCalcu(encoder->angle_rotor_raw);
	  motor_final_output = pid_control_all(50.0f,angle,encoder->speed_rpm);

	  can_motorSetCurrent(0x200, motor_final_output,0,0,0);

	  chThdSleepMilliseconds(10);
	}
}

void gripper_init()
{
  chThdCreateStatic(motor_ctrl_thread_wa, sizeof(motor_ctrl_thread_wa),
                  NORMALPRIO + 6 ,
                  motor_ctrl_thread, NULL);
}
