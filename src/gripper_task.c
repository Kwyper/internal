#include "gripper_task.h"

float gripper_encoder_state = 0.0;
void gripper_task(pid_s_t* gripper_pid)
{

   const int16_t max_interval = 30000;
   const int16_t min_interval = -1000;
   gripper_pneu();
   RC_Ctl_t* rc = RC_get();
   gripper_encoder_state += rc->channel3*0.1;
   if(gripper_encoder_state > max_interval)  gripper_encoder_state = max_interval;
   if(gripper_encoder_state < min_interval)  gripper_encoder_state = min_interval;

   int16_t gripper_out = pid_calcu(gripper_pid,gripper_encoder_state,_gripper.total_ecd);

   if(_gripper.total_ecd > max_interval||max_interval < min_interval)
   {
     can_motorSetCurrent(
       0x1FF,
       0,
       0,
       0,
       0);

   }
   else{
     can_motorSetCurrent(
       0x1FF,
       gripper_out,
       0,
       0,
       0);

   }

}

void gripper_init(void)
{
  palClearPad(GPIOB,7);
  palClearPad(GPIOB,8);
  palClearPad(GPIOB,9);
}

void gripper_pneu(void)
{
  RC_Ctl_t* rc = RC_get();
  switch (rc->s1) {
    case RC_S_UP:
    palSetPad(GPIOB,7);

    break;
    case RC_S_MIDDLE:
    palClearPad(GPIOB,7);
    //palClearPad(GPIOB,7);

    break;
    case RC_S_DOWN:
    //palSetPad(GPIOB,7);

    break;
    default:
    break;

  }

  switch (rc->s2) {
    case RC_S_UP:
    palSetPad(GPIOB,8);


    break;
    case RC_S_MIDDLE:
    palClearPad(GPIOB,8);
    palClearPad(GPIOB,9);

    break;
    case RC_S_DOWN:
    palSetPad(GPIOB,9);
    //palSetPad(GPIOB,7);

    break;
    default:
    break;

  }

}
