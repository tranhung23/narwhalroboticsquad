/*
 * stm32f4_timer.c
 *
 *  Created on: Apr 12, 2012
 *      Author: GrubyGrub
 */

/*
    This file is part of AutoQuad.

    AutoQuad is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    AutoQuad is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with AutoQuad.  If not, see <http://www.gnu.org/licenses/>.

    Copyright © 2011, 2012  Bill Nesbitt
*/

#include "narwhal_MICRO_TIMER.h"
#include <rc_control.h>

/*generates a 1mhz timer signal used to count, overflows every 71 minutes..*/

microsTimerEvents usTimer;

/*this will overflow in 71.58 minutes, find a better way to handle overflow*/
void usTimerInit(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	// Enable the TIMER_TIM global Interrupt
	NVIC_InitStructure.NVIC_IRQChannel = MICROS_TIMER_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = MICROS_TIMER_IRQ_PREPRIO;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = MICROS_TIMER_IRQ_SUBPRIO;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	RCC_APB1PeriphClockCmd(MICROS_TIMER_CLK, ENABLE);

	/* Time base configuration for 1MHz (us)*/
	TIM_TimeBaseStructure.TIM_Period = 0xFFFFFFFF;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(MICROS_TIMER, &TIM_TimeBaseStructure);

	uint16_t PrescalerValue = (uint32_t) ((SystemCoreClock / 2) / MICROS_TIMER_TIMEBASE) - 1;
	TIM_PrescalerConfig(MICROS_TIMER, PrescalerValue, TIM_PSCReloadMode_Immediate);

	// reset
	TIM_SetCounter(MICROS_TIMER, 0);

	// Output Compare for alarms
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Inactive;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	// go...
	TIM_Cmd(MICROS_TIMER, ENABLE);

}

void timerStart(void)
{
	usTimer.timerStart = uTicks();
}

uint32_t timerStop(void)
{
	return uTicks() - usTimer.timerStart;
}

void timerCancelAlarm1(void)
{
	MICROS_TIMER->DIER &= (uint16_t) ~TIM_IT_CC1;
	TIM_ClearITPendingBit(MICROS_TIMER, TIM_IT_CC1);
}

void timerCancelAlarm2(void)
{
	MICROS_TIMER->DIER &= (uint16_t) ~TIM_IT_CC2;
	TIM_ClearITPendingBit(MICROS_TIMER, TIM_IT_CC2);
}

void timerCancelAlarm3(void)
{
	MICROS_TIMER->DIER &= (uint16_t) ~TIM_IT_CC3;
	TIM_ClearITPendingBit(MICROS_TIMER, TIM_IT_CC3);
}

void timerSetAlarm1(int32_t us, timerCallback_t *callback, int parameter)
{
	// schedule it
	usTimer.alarm1Callback = callback;
	usTimer.alarm1Parameter = parameter;

	MICROS_TIMER->SR = (uint16_t) ~TIM_IT_CC1;
	MICROS_TIMER->CCR1 = MICROS_TIMER->CNT + us;
	MICROS_TIMER->DIER |= TIM_IT_CC1;
}

void timerSetAlarm2(int32_t us, timerCallback_t *callback, int parameter)
{
	// schedule it
	usTimer.alarm2Callback = callback;
	usTimer.alarm2Parameter = parameter;

	MICROS_TIMER->SR = (uint16_t) ~TIM_IT_CC2;
	MICROS_TIMER->CCR2 = MICROS_TIMER->CNT + us;
	MICROS_TIMER->DIER |= TIM_IT_CC2;
}

void timerSetAlarm3(int32_t us, timerCallback_t *callback, int parameter)
{
	// schedule it
	usTimer.alarm3Callback = callback;
	usTimer.alarm3Parameter = parameter;

	MICROS_TIMER->SR = (uint16_t) ~TIM_IT_CC3;
	MICROS_TIMER->CCR3 = MICROS_TIMER->CNT + us;
	MICROS_TIMER->DIER |= TIM_IT_CC3;
}

void usTimer_IRQHandler(void)
{
	if (TIM_GetITStatus(MICROS_TIMER, TIM_IT_CC1) != RESET)
	{
		MICROS_TIMER->SR = (uint16_t) ~TIM_IT_CC1;

		// Disable the Interrupt
		MICROS_TIMER->DIER &= (uint16_t) ~TIM_IT_CC1;

		usTimer.alarm1Callback(usTimer.alarm1Parameter);
	}
	else if (TIM_GetITStatus(MICROS_TIMER, TIM_IT_CC2) != RESET)
	{
		MICROS_TIMER->SR = (uint16_t) ~TIM_IT_CC2;

		// Disable the Interrupt
		MICROS_TIMER->DIER &= (uint16_t) ~TIM_IT_CC2;

		usTimer.alarm2Callback(usTimer.alarm2Parameter);
	}
	else if (TIM_GetITStatus(MICROS_TIMER, TIM_IT_CC3) != RESET)
	{
		MICROS_TIMER->SR = (uint16_t) ~TIM_IT_CC3;

		// Disable the Interrupt
		MICROS_TIMER->DIER &= (uint16_t) ~TIM_IT_CC3;

		usTimer.alarm3Callback(usTimer.alarm3Parameter);
	}
	// CC4 is used for RTC calibration at startup
	else if (TIM_GetITStatus(MICROS_TIMER, TIM_IT_CC4) != RESET)
	{
		// Get the Input Capture value
		//rtcData.captureLSI[rtcData.captureNumber++] = TIM_GetCapture4(TIM5);

		// Clear CC4 Interrupt pending bit
		TIM_ClearITPendingBit(MICROS_TIMER, TIM_IT_CC4);
	}
}
