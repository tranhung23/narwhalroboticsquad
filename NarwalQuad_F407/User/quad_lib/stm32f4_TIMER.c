/*
 * stm32f4_timer.c
 *
 *  Created on: Apr 12, 2012
 *      Author: GrubyGrub
 */

#include "stm32f4_quad.h"
#include "stm32f4_TIMER.h"
#include "sensors.h"
#include "../Control/RC_Control.h"

#define DEBUG 0

/*RC Control connected lines def, this is revision 1.0 of quadrotor control board.
 * TODO: Integate TImer9 and 12
 * Timer1_CH2 - PE11
 * Timer1_CH4 - PE14
 * Timer4_CH2 - PD13
 * TImer4_CH4 - PD15
 * Timer9_CH2 - PE6
 * Timer12_CH2 -PB15
 */

__IO int capture = 0;

GPIO_TypeDef* PWM_IN_PORT[RCn] = { RC1_GPIO_PORT, RC2_GPIO_PORT, RC3_GPIO_PORT, RC4_GPIO_PORT };
const uint16_t PWM_IN_PIN[RCn] = { RC1_PIN, RC2_PIN, RC3_PIN, RC4_PIN };
const uint16_t PWM_IN_PINSOURCE[RCn] = { RC1_PIN_SOURCE, RC2_PIN_SOURCE, RC3_PIN_SOURCE, RC4_PIN_SOURCE };
const uint32_t PWM_IN_PIN_CLK[RCn] = { RC1_GPIO_CLK, RC2_GPIO_CLK, RC3_GPIO_CLK, RC4_GPIO_CLK };
const uint16_t PWM_IN_PIN_AF[RCn] = { RC1_GPIO_AF, RC2_GPIO_AF, RC3_GPIO_AF, RC4_GPIO_AF };

TIM_TypeDef * PWM_IN_TIMER[RCn] = { RC1_TIMER, RC2_TIMER, RC3_TIMER, RC4_TIMER };
const uint32_t PWM_IN_TIMER_CLK[RCn] = { RC1_TIMER_CLK, RC2_TIMER_CLK, RC3_TIMER_CLK, RC4_TIMER_CLK };
const uint16_t PWM_IN_TIMER_IRQn[RCn] = { RC1_TIMER_IRQn, RC2_TIMER_IRQn, RC3_TIMER_IRQn, RC4_TIMER_IRQn };
const uint16_t PWM_IN_TIMER_PREPRIO[RCn] = { RC1_TIMER_IRQ_PREPRIO, RC2_TIMER_IRQ_PREPRIO, RC3_TIMER_IRQ_PREPRIO, RC4_TIMER_IRQ_PREPRIO };
const uint16_t PWM_IN_TIMER_SUBPRIO[RCn] = { RC1_TIMER_IRQ_SUBPRIO, RC2_TIMER_IRQ_SUBPRIO, RC3_TIMER_IRQ_SUBPRIO, RC4_TIMER_IRQ_SUBPRIO };

const uint16_t PWM_IN_TIMER_CHR[RCn] = { RC1_TIMER_CHR, RC2_TIMER_CHR, RC3_TIMER_CHR, RC4_TIMER_CHR };
const uint16_t PWM_IN_TIMER_CHF[RCn] = { RC1_TIMER_CHF, RC2_TIMER_CHF, RC3_TIMER_CHF, RC4_TIMER_CHF };
const uint16_t PWM_IN_TIMER_IRQ[RCn] = { RC1_TIMER_IRQ, RC2_TIMER_IRQ, RC3_TIMER_IRQ, RC4_TIMER_IRQ };

static uint16_t PWM_Timer_Duty_Helper(uint16_t TI_Rising, uint16_t TI_Falling)
{
	uint16_t duty = TI_Falling - TI_Rising;

	if (duty > 0)
		return duty;
	return duty - 0xFFFF;
}

static void PWM_Timer_IRQ_Helper(PWM_CTRL_IN_TypeDef CH)
{
	uint16_t TI_Rising;
	uint16_t TI_Falling;

	/*Each calculation only pertians to channel 1 or 3 currently, if 1 or 3 is triggered it will calculate the PWM width.*/
	if (TIM_GetITStatus(PWM_IN_TIMER[CH], TIM_IT_CC1) != RESET)
	{
		TIM_ClearITPendingBit(PWM_IN_TIMER[CH], PWM_IN_TIMER_IRQ[CH]);

		TI_Rising = TIM_GetCapture2(PWM_IN_TIMER[CH]);
		TI_Falling = TIM_GetCapture1(PWM_IN_TIMER[CH]);
		MotorControl_SetAngle(PWM_Timer_Duty_Helper(TI_Rising, TI_Falling), CH);

	}
	else if (TIM_GetITStatus(PWM_IN_TIMER[CH], TIM_IT_CC3) != RESET)
	{
		TIM_ClearITPendingBit(PWM_IN_TIMER[CH], PWM_IN_TIMER_IRQ[CH]);

		TI_Rising = TIM_GetCapture4(PWM_IN_TIMER[CH]);
		TI_Falling = TIM_GetCapture3(PWM_IN_TIMER[CH]);
		MotorControl_SetAngle(PWM_Timer_Duty_Helper(TI_Rising, TI_Falling), CH);

		//RC_Control[CH] = PWM_Timer_Duty_Helper(TI_Rising, TI_Falling);
	}



#if defined(DEBUG)
	async_printf("%d ch: %d\r\n", RC_Control[CH], CH);
#endif
}

/*Handles all the RC timer interrupt items, currenlty we check for timer 1 and 4, need to add addtional timers if needed.*/
static void RC_Timer_Handler()
{
	/*Check timer 1*/
	if (TIM_GetITStatus(PWM_IN_TIMER[CH1], PWM_IN_TIMER_IRQ[CH1]) != RESET)
	{
		PWM_Timer_IRQ_Helper(CH1);
	}
	else if (TIM_GetITStatus(PWM_IN_TIMER[CH2], PWM_IN_TIMER_IRQ[CH2]) != RESET)
	{
		PWM_Timer_IRQ_Helper(CH2);
	}
	/*Check timer 4*/
	if (TIM_GetITStatus(PWM_IN_TIMER[CH3], PWM_IN_TIMER_IRQ[CH3]) != RESET)
	{
		PWM_Timer_IRQ_Helper(CH3);
	}
	else if (TIM_GetITStatus(PWM_IN_TIMER[CH4], PWM_IN_TIMER_IRQ[CH4]) != RESET)
	{
		PWM_Timer_IRQ_Helper(CH4);
	}
}

/*Init timer 1, used for sensor interrupts for now..*/
void TIM_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/* TIM3 clock enable */
	RCC_APB1PeriphClockCmd(TIM_I2C1_READ_CLK, ENABLE);

	/* Enable the TIM3 gloabal Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM_I2C1_READ_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIM_I2C1_READ_PREPRIO;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = TIM_I2C1_READ_SUBPRIO;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM3_Config();
}

void TIM3_Config(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	/* -----------------------------------------------------------------------
	 TIM3 Configuration: Output Compare Timing Mode:

	 In this example TIM3 input clock (TIM3CLK) is set to 2 * APB1 clock (PCLK1),
	 since APB1 prescaler is different from 1.
	 TIM3CLK = 2 * PCLK1
	 PCLK1 = HCLK / 4
	 => TIM3CLK = HCLK / 2 = SystemCoreClock /2

	 To get TIM3 counter clock at 6 MHz, the prescaler is computed as follows:
	 Prescaler = (TIM3CLK / TIM3 counter clock) - 1
	 Prescaler = ((SystemCoreClock /2) /6 MHz) - 1

	 CC1 update rate = TIM3 counter clock / CCR1_Val = 146.48 Hz
	 ==> Toggling frequency = 73.24 Hz

	 C2 update rate = TIM3 counter clock / CCR2_Val = 219.7 Hz
	 ==> Toggling frequency = 109.8 Hz

	 CC3 update rate = TIM3 counter clock / CCR3_Val = 439.4 Hz
	 ==> Toggling frequency = 219.7 Hz

	 CC4 update rate = TIM3 counter clock / CCR4_Val = 878.9 Hz
	 ==> Toggling frequency = 439.4 Hz

	 Note:
	 SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
	 Each time the core clock (HCLK) changes, user had to call SystemCoreClockUpdate()
	 function to update SystemCoreClock variable value. Otherwise, any configuration
	 based on this variable will be incorrect.
	 ----------------------------------------------------------------------- */

	/* Compute the prescaler value, Timer  */
	/*Timer 3 is running at 1 MHZ*/
	uint16_t PrescalerValue = (uint16_t) ((SystemCoreClock / 4) / 1000000) - 1;

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM_I2C1_READ, &TIM_TimeBaseStructure);

	/* Prescaler configuration */
	TIM_PrescalerConfig(TIM_I2C1_READ, PrescalerValue, TIM_PSCReloadMode_Immediate);

	//CHANNEL 1
	/* Output Compare Timing Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC1Init(TIM_I2C1_READ, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM_I2C1_READ, TIM_OCPreload_Disable);

	//CHANNEL 2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = CCR2_Val;

	TIM_OC2Init(TIM_I2C1_READ, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM_I2C1_READ, TIM_OCPreload_Disable);

	//CHANNEL 3
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = CCR3_Val;

	TIM_OC3Init(TIM_I2C1_READ, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM_I2C1_READ, TIM_OCPreload_Disable);

	//CHANNEL 4
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = CCR4_Val;

	TIM_OC4Init(TIM_I2C1_READ, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM_I2C1_READ, TIM_OCPreload_Disable);

	/* TIM Interrupts enable */
	TIM_ITConfig(TIM_I2C1_READ, TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4, ENABLE);

	/* TIM3 enable counter */
	TIM_Cmd(TIM_I2C1_READ, ENABLE);
}

void Sensors_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);

		/* LED1 toggling with frequency = 73.24 Hz */
		GPIOToggle(LED3);

		capture = TIM_GetCapture1(TIM3);
		TIM_SetCompare1(TIM3, capture + CCR1_Val);
	}
	else if (TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET)
	{ //100hz
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);

		/* LED2 toggling with frequency = 109.8 Hz */
		GPIOToggle(LED4);
		//Sensors_Read_async(MAG);

		capture = TIM_GetCapture2(TIM3);
		TIM_SetCompare2(TIM3, capture + CCR2_Val);
	}
	else if (TIM_GetITStatus(TIM3, TIM_IT_CC3) != RESET)
	{ //400hz
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);

		/* LED3 toggling with frequency = 219.7 Hz */
		GPIOToggle(LED5);
		//Sensors_Read_async(ACC);

		capture = TIM_GetCapture3(TIM3);
		TIM_SetCompare3(TIM3, capture + CCR3_Val);
	}
	else
	{ //2khz
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC4);

		//Sensors_Read_async(GYRO);

		capture = TIM_GetCapture4(TIM3);
		TIM_SetCompare4(TIM3, capture + CCR4_Val);
	}
}

/*helps to setup all the GPIO for PWM input*/
/*	PB7 -> Timer 4 CH 1/2
 *
 */
void PWM_Ctrl_GPIO_Helper(PWM_CTRL_IN_TypeDef CH)
{
	RCC_AHB1PeriphClockCmd(PWM_IN_PIN_CLK[CH], ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = PWM_IN_PIN[CH];
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(PWM_IN_PORT[CH], &GPIO_InitStructure);

	/* Connect TIM pin to AF2 */
	GPIO_PinAFConfig(PWM_IN_PORT[CH], PWM_IN_PINSOURCE[CH], PWM_IN_PIN_AF[CH]);

}

/*Init timers, it will give me the duty cycle of the PWM wave, this runs at 1 mhz, so duty cycle in us*/
void TIM_PWM_Ctrl_In_Init(PWM_CTRL_IN_TypeDef CH)
{

	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;

	/*Configures the certain GPIO to respond to timers*/
	PWM_Ctrl_GPIO_Helper(CH);

	/*Each timer will run at 1mhz */
	/* TIM4 clock enable */
	if ((PWM_IN_TIMER_CLK[CH] == RCC_APB2Periph_TIM1) || (PWM_IN_TIMER_CLK[CH] == RCC_APB2Periph_TIM8) || (PWM_IN_TIMER_CLK[CH] == RCC_APB2Periph_TIM9) || (PWM_IN_TIMER_CLK[CH] == RCC_APB2Periph_TIM10) || (PWM_IN_TIMER_CLK[CH] == RCC_APB2Periph_TIM11))
	{
		RCC_APB2PeriphClockCmd(PWM_IN_TIMER_CLK[CH], ENABLE);
		uint16_t PrescalerValue = (uint16_t) ((SystemCoreClock / 1) / 1000000) - 1;
		TIM_PrescalerConfig(PWM_IN_TIMER[CH], PrescalerValue, TIM_PSCReloadMode_Immediate);
	}
	else
	{
		RCC_APB1PeriphClockCmd(PWM_IN_TIMER_CLK[CH], ENABLE);
		uint16_t PrescalerValue = (uint16_t) ((SystemCoreClock / 2) / 1000000) - 1;
		TIM_PrescalerConfig(PWM_IN_TIMER[CH], PrescalerValue, TIM_PSCReloadMode_Immediate);
	}
	//TODO: Fix prescaler value make into macro or sth.

	NVIC_InitStructure.NVIC_IRQChannel = PWM_IN_TIMER_IRQn[CH];
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PWM_IN_TIMER_PREPRIO[CH];
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = PWM_IN_TIMER_SUBPRIO[CH];
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/*One wire connected to PINX, but need to have two timers, rising and falling -
	 * FALLING will be indirected and connected to channel 1 (or 3, different channel)
	 * channel 2 will be directly connected to pin and look at RISING.
	 *
	 */

	/*Channel 1 looks at falling*/
	TIM_ICInitStructure.TIM_Channel = PWM_IN_TIMER_CHF[CH];
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_IndirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0x0;
	TIM_ICInit(PWM_IN_TIMER[CH], &TIM_ICInitStructure);

	/*Channel 2*/
	TIM_ICInitStructure.TIM_Channel = PWM_IN_TIMER_CHR[CH];
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0x0;
	TIM_ICInit(PWM_IN_TIMER[CH], &TIM_ICInitStructure);

	/* TIM enable counter */
	TIM_Cmd(PWM_IN_TIMER[CH], ENABLE);
	TIM_ITConfig(PWM_IN_TIMER[CH], PWM_IN_TIMER_IRQ[CH], ENABLE);

}

/*We are only using Timer 1 and timer 4 currently, ignore others for now.*/
void TIM1_CC_IRQHandler(void)
{
	RC_Timer_Handler();
}

void TIM4_IRQHandler(void)
{
	RC_Timer_Handler();
}

