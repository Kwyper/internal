#include "ch.h"
#include "hal.h"
#include "canBusProcess.h"
#include "dbus.h"
#include "turrent.h"
#include "oled.h"
#include "serial_task.h"

pid_s_t pid;
/*uint8_t serial_buffer[4] = {0};
uint8_t terret_state = 0;*/

static THD_WORKING_AREA(motor_ctrl_thread_wa,512);
static THD_FUNCTION(motor_ctrl_thread, p)
{
    (void) p;
    pid_init(&pid,0.3f,0.0f,2.2f,1000.0f,2000.0f);
    turrent_calibrate();
	while(true)
	{

	  turrent_task(&pid,get_terret_state());

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
    serial_task_Init();
    punch_Init();

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
