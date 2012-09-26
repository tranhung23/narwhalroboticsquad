/*
 * Control loop of the drone
 */

#include "user_control.h"
#include <RC_Control.h>
#include <narwhal_timer.h>

OS_STK *narwhalControlStack;

#define DEBUG 1

void Control_Init_Task(void) {

	StatusType result;

	sync_printf("Init control starting \r\n");

	float yaw, pitch, roll, throttle;

	while (1) {
		//TODO: wait for UKF filter completation.

		/*New values values in in, update pitch yaw roll throttle*/
		if (CoAcceptSingleFlag(RADIO_PWM_INPUT_FLAG) == E_OK) {
			yaw = RC_SetAngle(CTRL_RAW[YAW], CH1);
			pitch = RC_SetAngle(CTRL_RAW[PITCH], CH2);
			roll = RC_SetAngle(CTRL_RAW[ROLL], CH3);
			throttle = RC_SetAngle(CTRL_RAW[THROTTLE], CH4);

#ifdef DEBUG
			sync_printf("Yaw: %e, Pitch: %e, Roll: %e, Throttle: %f \r\n", yaw,
					pitch, roll, throttle);
#endif
		}
		/*PID Loop update process*/

		CoTickDelay(10);

	}
}
