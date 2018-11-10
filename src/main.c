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
    pid_init(&pid,0.3f,0.0f,2.0f,1000.0f,5000.0f);
	while(true)
	{


	  /*motor_final_output = pid_calc(&pid,1234,encoder->angle_rotor_raw);

	  can_motorSetCurrent(0x200, motor_final_output,0,0,0);*/

	  turrent_task(&pid,get_terret_state());

	  chThdSleepMilliseconds(10);
	}
}

/*static THD_WORKING_AREA(serial_thread_wa,512);
static THD_FUNCTION(serial_thread, p)
{
  (void)p;
  while(true)
  {
    sdRead(&SD1,serial_buffer,1);
    terret_state = serial_buffer[0];
    OLED_ShowNum(0,0,terret_state,1,16);
    chThdSleepMilliseconds(10);
  }
}*/

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

    /*OLED_Init();
    OLED_Clear();
    sdStop(&SD1);
    static const SerialConfig serial_rx_config = {
                                                  9600,
                                                  0,
                                                  USART_CR2_STOP1_BITS,
                                                  0,
    };
    sdStart(&SD1, &serial_rx_config);


    chThdCreateStatic(serial_thread_wa, sizeof(serial_thread_wa),
                         NORMALPRIO+1, serial_thread, NULL);*/

    chThdCreateStatic(motor_ctrl_thread_wa, sizeof(motor_ctrl_thread_wa),
		  	  	  	 NORMALPRIO, motor_ctrl_thread, NULL);

    /*
    * Normal main() thread activity
    */
    while (true)
    {
        //can_motorSetCurrent(0x200,2000,0,0,0);
        palTogglePad(GPIOA, GPIOA_LED);
        chThdSleepMilliseconds(500);
    }
}
