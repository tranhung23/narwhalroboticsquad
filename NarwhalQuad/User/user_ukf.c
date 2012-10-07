/*
 * Control loop of the drone
 */

#include "user_ukf.h"

OS_STK *narwhalUKFStack;

#define DEBUG 1

void UKF_Init_Task(void) {

	StatusType result;

	sync_printf("UKF control starting \r\n");
	//navUkfInit();

	while (1) {
		//TODO: wait for UKF filter completation.

		CoTickDelay(10);

	}
}
