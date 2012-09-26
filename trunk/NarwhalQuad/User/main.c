#include "narwhal_top.h"
#include <fpu.h>
#include "user_start.h"
#include <narwhal_utils.h>


#include "user_INS.h"
#include "user_control.h"

#include <../example/IOToggle.h>

/* Private function prototypes -----------------------------------------------*/
/* Private functions */
/**
 **===========================================================================
 **
 **  Abstract: main program
 **
 **===========================================================================
 */

int main(void)
{
	fpuInit(); // setup FPU context switching
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	CoInitOS();

	Drivers_Init_Task();

	LEDSignalStack = narwhalStackInit(INIT_STACK_SIZE);
	CoCreateTask((FUNCPtr)IOToggle, (void *)0, INIT_PRIORITY, &LEDSignalStack[INIT_STACK_SIZE-1], INIT_STACK_SIZE);

	// narwhalInitStack = narwhalStackInit(INIT_STACK_SIZE);
	// CoCreateTask((FUNCPtr)Drivers_Init_Task, (void *)0, INIT_PRIORITY+1, &narwhalInitStack[INIT_STACK_SIZE-1], INIT_STACK_SIZE);

	narwhalControlStack = narwhalStackInit(CONTROL_STACK_SIZE);
	CoCreateTask((FUNCPtr)Control_Init_Task, (void *)0, CONTROL_PRIORITY, &narwhalControlStack[CONTROL_STACK_SIZE-1], CONTROL_STACK_SIZE);

	narwhalINSStack = narwhalStackInit(INS_STACK_SIZE);
	CoCreateTask((FUNCPtr)INS_Init_Task, (void *)0, INS_PRIORITY, &narwhalINSStack[INS_STACK_SIZE-1], INS_STACK_SIZE);

	CoStartOS();

	return 0;
}
