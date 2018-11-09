#include "ch.h"
#include "hal.h"
#include "canBusProcess.h"
#include "dbus.h"
#include "turrent.h"
//#include "chassis_task.h"
//#include "configure.h"
//pid_s_t pid;
//static int16_t motor_final_output;
/*const float kp_angle = 25.0f;     //Proportional for angle
const float ki_angle = 0.01f;     //Integration for angle
const float kd_angle = 10.0f;     //Derivative for angle

const float kp_speed = 0.01f;
const float ki_speed = 0.00012f;
const float kd_speed = 0.7f;

//all to be modified

volatile float errorSum_angle = 0;
volatile float preError_angle = 0;

volatile float errorSum_speed = 0;
volatile float preError_speed = 0;*/
/*volatile float angle;

static float angle_pid_control(const float setPoint,
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
/*static int16_t speed_pid_control(const float setPoint_fromAngle,
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
}*/
/*
static float absValue(float a){
  if (a<0.0f){
    return -a;
  }
  return a;
}
*/
/*static int basicTest(float setPoint_forMotor, float currentPoint_fromMotor){
  if((setPoint_forMotor - currentPoint_fromMotor) < 30.0f &&
      (setPoint_forMotor - currentPoint_fromMotor) > -30.0f){
    return 0;
  }else{
    return 500;
  }
}*/
/**
 * a simple function to link two PID controller
 *
 * setPoint_forAngle: from open CV
 * currentPoint_fromMotor: from Motor feedback
 * currentSpeed_fromMotor: from Motor feedback
 */
/*static int16_t pid_control_all(const float setPoint_forAngle,
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
}*/
/*static float currentAngleCalcu(float radian_angle){
    while((radian_angle - 27*8192) > 0){
      radian_angle -= 27*8192;
    }
    radian_angle *= 7.669904e-4f;
    return radian_angle;
}*/
pid_s_t pid;
uint8_t serial_buffer[4] = {0};
uint8_t terret_state = 0;

static THD_WORKING_AREA(motor_ctrl_thread_wa,512);
static THD_FUNCTION(motor_ctrl_thread, p)
{
    (void) p;
    pid_init(&pid,7.5f,0.03f,0.0f,1000.0f,12000.0f);
	while(true)
	{
	  Encoder_canStruct* encoder = can_getEncoder();

	  /*motor_final_output = pid_calc(&pid,1234,encoder->angle_rotor_raw);

	  can_motorSetCurrent(0x200, motor_final_output,0,0,0);*/

	  turrent_task(&pid,encoder,terret_state);

	  chThdSleepMilliseconds(10);
	}
}

static THD_WORKING_AREA(serial_thread_wa,512);
static THD_FUNCTION(serial_thread, p)
    {
  (void)p;
  while(true)
  {
    sdRead(&SD1,serial_buffer,1);
    terret_state = serial_buffer[0];
    chThdSleepMilliseconds(10);
  }
    }

/*
 * Application entry point.
 */
int main(void)
{

    /*
    * System initializations.
    * - HAL initialization, this also initializes the configured device drivers
    *   and performs the board-specific initializations.
    * - Kernel initialization, the main() function becomes a thread and the
    *   RTOS is active.
    */
    halInit();
    chSysInit();
    RC_init();
    can_processInit();
    sdStop(&SD1);
    static const SerialConfig serial_rx_config = {
                                                  9600,
                                                  0,
                                                  USART_CR2_STOP1_BITS,
                                                  0,
    };
    sdStart(&SD1, &serial_rx_config);

    chThdCreateStatic(serial_thread_wa, sizeof(serial_thread_wa),
                         NORMALPRIO+1, serial_thread_wa, NULL);

    chThdCreateStatic(motor_ctrl_thread_wa, sizeof(motor_ctrl_thread_wa),
		  	  	  	 NORMALPRIO, motor_ctrl_thread, NULL);

    /*
    * Normal main() thread activity
    */
    while (true)
    {
        palTogglePad(GPIOA, GPIOA_LED);
        chThdSleepMilliseconds(500);
    }
}
