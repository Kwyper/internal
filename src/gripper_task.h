#ifndef GRIPPER_TASK_H
#define GRIPPER_TASK_H

#include "ch.h"
#include "hal.h"

#include "chassis_task.h"
#include "canBusProcess.h"

void gripper_task(pid_s_t* gripper_pid);
void gripper_pneu(void);


#endif /* end of include guard: GRIPPER_TASK_H */
