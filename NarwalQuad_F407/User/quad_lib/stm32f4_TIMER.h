/*
 * stm32f4_timer.h
 *
 *  Created on: Apr 18, 2012
 *      Author: GrubyGrub
 */

#ifndef STM32F4_TIMER_H_
#define STM32F4_TIMER_H_

#include "stm32f4xx.h"
#include "../Control/Motor_Control.h"

/*1 mhz*/
#define TIM3_FREQ						1000000
#define RC_TIMERS_FREQ					1000000

#define RCn                            4
#define RC_TIMER_IRQ_PREPRIO		   10

/*Timer1_CH2 - PE11*/
#define RC1_PIN                         GPIO_Pin_11
#define RC1_PIN_SOURCE                  GPIO_PinSource11
#define RC1_GPIO_PORT                   GPIOE
#define RC1_GPIO_CLK                    RCC_AHB1Periph_GPIOE
#define RC1_GPIO_AF						GPIO_AF_TIM1

#define RC1_TIMER						TIM1
#define RC1_TIMER_CLK					RCC_APB2Periph_TIM1
#define RC1_TIMER_IRQn					TIM1_CC_IRQn
#define RC1_TIMER_IRQ_PREPRIO			RC_TIMER_IRQ_PREPRIO
#define RC1_TIMER_IRQ_SUBPRIO			15
#define RC1_TIMER_CHR					TIM_Channel_2	//Channel Rising
#define RC1_TIMER_CHF					TIM_Channel_1	//Channel Falling
#define RC1_TIMER_IRQ					TIM_IT_CC1

/*
 * RC2
 */
/*Timer1_CH4 - PE14*/
#define RC2_PIN                         GPIO_Pin_14
#define RC2_PIN_SOURCE                  GPIO_PinSource14
#define RC2_GPIO_PORT                   GPIOE
#define RC2_GPIO_CLK                    RCC_AHB1Periph_GPIOE
#define RC2_GPIO_AF						GPIO_AF_TIM1

#define RC2_TIMER						TIM1
#define RC2_TIMER_CLK					RCC_APB2Periph_TIM1
#define RC2_TIMER_IRQn					TIM1_CC_IRQn
#define RC2_TIMER_IRQ_PREPRIO			RC_TIMER_IRQ_PREPRIO
#define RC2_TIMER_IRQ_SUBPRIO			14
#define RC2_TIMER_CHR					TIM_Channel_4	//Channel Rising
#define RC2_TIMER_CHF					TIM_Channel_3	//Channel Falling
#define RC2_TIMER_IRQ					TIM_IT_CC3

/*
 * RC3
 */
/*Timer4_CH2 - PD13*/
#define RC3_PIN                         GPIO_Pin_13
#define RC3_PIN_SOURCE                  GPIO_PinSource13
#define RC3_GPIO_PORT                   GPIOD
#define RC3_GPIO_CLK                    RCC_AHB1Periph_GPIOD
#define RC3_GPIO_AF						GPIO_AF_TIM4

#define RC3_TIMER						TIM4
#define RC3_TIMER_CLK					RCC_APB1Periph_TIM4
#define RC3_TIMER_IRQn					TIM4_IRQn
#define RC3_TIMER_IRQ_PREPRIO			RC_TIMER_IRQ_PREPRIO
#define RC3_TIMER_IRQ_SUBPRIO			13
#define RC3_TIMER_CHR					TIM_Channel_2	//Channel Rising
#define RC3_TIMER_CHF					TIM_Channel_1	//Channel Falling
#define RC3_TIMER_IRQ					TIM_IT_CC1

/*
 * RC4
 */
/* Timer4_CH4 - PD15*/
#define RC4_PIN                         GPIO_Pin_15
#define RC4_PIN_SOURCE                  GPIO_PinSource15
#define RC4_GPIO_PORT                   GPIOD
#define RC4_GPIO_CLK                    RCC_AHB1Periph_GPIOD
#define RC4_GPIO_AF						GPIO_AF_TIM4

#define RC4_TIMER						TIM4
#define RC4_TIMER_CLK					RCC_APB1Periph_TIM4
#define RC4_TIMER_IRQn					TIM4_IRQn
#define RC4_TIMER_IRQ_PREPRIO			RC_TIMER_IRQ_PREPRIO
#define RC4_TIMER_IRQ_SUBPRIO			12
#define RC4_TIMER_CHR					TIM_Channel_4	//Channel Rising
#define RC4_TIMER_CHF					TIM_Channel_3	//Channel Falling
#define RC4_TIMER_IRQ					TIM_IT_CC3

/*
 * TIMER defines
 */
#define TIM_I2C1_READ_CLK					RCC_APB1Periph_TIM3
#define TIM_I2C1_READ						TIM3
#define TIM_I2C1_READ_IRQn					TIM3_IRQn
#define TIM_I2C1_READ_PREPRIO               4
#define TIM_I2C1_READ_SUBPRIO               0

/*Timer is running at 1mhz, so each division is 1uS*/
#define CCR1_Val 	 40961 /**/
#define CCR2_Val 	 5555 /*HMC data rate - 100 hz*/
#define CCR3_Val 	 2700 /*AXDL data rate 400 hz*/
#define CCR4_Val	 2000 /*ITG 3200 at 2khz*/

#define Sensors_IRQHandler	TIM3_IRQHandler

/*Timer Items*/
void TIM_Init(void);
void TIM3_Config(void);
void Sensors_IRQHandler(void);

void TIM_PWM_Ctrl_In_Init(PWM_CTRL_IN_TypeDef CH);
void TIM4_IRQHandler(void);

#endif /* STM32F4_TIMER_H_ */
