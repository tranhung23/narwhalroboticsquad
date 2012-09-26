/* Includes ------------------------------------------------------------------*/
#include "narwhal_top.h"
#include "narwhal_GPIO.h"

OS_STK *LEDSignalStack;

void IOToggle(void) {

	while (1) {
		GPIOToggle(LED3);
		CoTickDelay(50);
		GPIOToggle(LED4);
		CoTickDelay(50);
		GPIOToggle(LED5);
		CoTickDelay(50);
		GPIOToggle(LED6);
		CoTickDelay(50);

	}
}
