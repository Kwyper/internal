#ifndef TURRENT_H
#define TURRENT_H

#include "chassis_task.h"
#include "oled.h"

void turrent_task(pid_s_t turrent_pid[]);
void turrent_calibrate(void);

typedef enum{
  UP = 0,
  DOWN,
  PRESSED,
  HOLD

}button_state_t;

extern int32_t record_pos[3];

#endif /* end of include guard: TURRENT_H */
