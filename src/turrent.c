#include "ch.h"
#include "hal.h"


#include "turrent.h"
#include "chassis_task.h"
#include "uart_for_cv.h"

float record_pos[3] = {0};
extern int random_number;

void turrent_task(pid_s_t turrent_pid[])
{
  int16_t turrent_output = 0;
  int16_t state;
  target = random_number;
  if(target==1||target==2||target==3){
  state = record_pos[target-1];
  turrent_output = pid_calcu(&turrent_pid[1],record_pos[target-1],_encoder[0].total_ecd);
}
  if(target==51||target==52||target==53){
    state = record_pos[target-51];
  turrent_output = pid_calcu(&turrent_pid[1],record_pos[target-51],_encoder[0].total_ecd);
}
if(_encoder[0].total_ecd - state < 1500&&_encoder[0].total_ecd - state > -1500)  palSetPad(GPIOB,8);
else palClearPad(GPIOB,8);


  can_motorSetCurrent(
    0x200,
    turrent_output,
    0,
    0,
    0);
}
void turrent_calibrate(void)
{
  button_state_t button_state = UP;
  uint8_t cursor = 0;
  systime_t gap_time;
  while(true)
  {

    OLED_ShowNum(0,2*cursor,_encoder[0].total_ecd,6,16);


    if(!palReadPad(GPIOA,GPIOA_BUTTON)&&button_state==UP)
    {
       button_state = DOWN;
       gap_time = chVTGetSystemTime();
    }

    if(palReadPad(GPIOA,GPIOA_BUTTON)&&button_state==DOWN)
    {
       if(chVTGetSystemTime() - gap_time > TIME_MS2I(900)) button_state = HOLD;
       else if(chVTGetSystemTime() - gap_time > TIME_MS2I(100)) button_state = PRESSED;
       else button_state = UP;
    }


    if(button_state==HOLD)
     {
       button_state = UP;
       //break;
     }

    if(button_state==PRESSED)
    {
      record_pos[cursor++] = _encoder[0].total_ecd;
      if(cursor>=3)
       {
         OLED_Clear();
         OLED_ShowString(0,2, "finish");
         break;
       }
      //cursor = cursor%3;
      button_state = UP;
    }


  }
}
