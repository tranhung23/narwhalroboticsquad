/*
 * stm32f4_timer.h
 *
 *  Created on: Apr 18, 2012
 *      Author: GrubyGrub
 */

#ifndef STM32F4_TIMER_H_
#define STM32F4_TIMER_H_

#include <narwhal_top.h>

typedef enum
{
	CH1 = 0, CH2 = 1, CH3 = 2, CH4 = 3
} PWM_CTRL_IN_TypeDef;

extern OS_FlagID RADIO_PWM_INPUT_FLAG;

/*Timer Items*/
extern void TIM_PWM_Ctrl_In_Init(PWM_CTRL_IN_TypeDef CH);
extern void PWM_Ctrl_GPIO_Helper(PWM_CTRL_IN_TypeDef CH);

#endif /* STM32F4_TIMER_H_ */
