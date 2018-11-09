#include "ch.h"
#include "hal.h"


#include "turrent.h"
#include "chassis_task.h"

int32_t record_pos[3] = {0};

void turrent_task(pid_s_t turrent_pid[])
{

  int16_t turrent_output = pid_calcu(&turrent_pid[1],0.0f,_encoder[0].total_ecd);


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
    if(palReadPad(GPIOA,GPIOA_BUTTON)&&button_state==UP)
    {
       button_state = DOWN;
       gap_time = chVTGetSystemTime();
    }
    if(!palReadPad(GPIOA,GPIOA_BUTTON)&&button_state==DOWN)
    {
       if(chVTGetSystemTime() - gap_time > TIME_MS2I(1000)) button_state = HOLD;
       else if(chVTGetSystemTime() - gap_time > TIME_MS2I(100)) button_state = PRESSED;
       else button_state = UP;
    }

    if(button_state==HOLD)  break;

    if(button_state==PRESSED)
    {
      record_pos[cursor++] = _encoder[0].total_ecd;
      cursor = cursor%3;
      button_state = UP;
    }


  }
}
