/* Includes */
#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_rcc.h"

#include "quad_lib/stm32f4_quad.h"
#include "quad_lib/stm32f4_TIMER.h"
#include "quad_lib/sensors.h"

#include "sensors/ITG3200.h"

#include "DSP/ekf.h"
#include "main.h"
#include "Control/Motor_Control.h"


/*Temps*/
#include "Control/RC_Control.h"

#define systick_divider (SystemCoreClock / 1000)

/* Private macro */
/* Private variables */
USART_InitTypeDef USART_InitStructure;
__IO unsigned long millis = 0;

/* Private function prototypes -----------------------------------------------*/
void quad_init(void);

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
	char * output;
	long current_time_ms_gyro = 0;
	long current_time_ms_acc = 0;
	long current_time_ms_mag = 0;
	long current_time_ms_print = 0;
	unsigned long long current_time_gyro = 0;

	quad_init();
	float f = 0.324;


	/*do nothing*/
	for (int i = 0; i < 16800000; i++)
		;

	float dt = 0;


	while (1)
	{
		//I2C_WriteDevice(I2C_COM2, 0x52, 100, 1);

		int z = RC_Control[0];
		sync_printf("Timer 1: %d", RC_Control[0]);
		for (int i = 0; i < 16800000; i++)
				;
		//ADC_print();
		//Sensors_Read(MAG);
//		Sensors_Read(MAG);

		//Sensors_Read(MAG);
		//while(1);
		//output = async_scanf(4);
		//async_printf("WTF?\r\n");
		//sync_printf("omg: %s\r\n", output);

	}

	while (1)
	{

		if (millis - current_time_ms_print > 20)
		{
			//Sensors_Read(-1);
			//I2C_WriteDevice(I2C_COM2, 0x52, 50, 1);

			current_time_ms_print = millis;
			ekf_print();
		}

		/*run gyro processing at 1000hz*/
		if (millis - current_time_ms_gyro > 0)
		{

			unsigned long long l_dt = (unsigned long long) ((millis / 1000.0 + ((float) ((signed) systick_divider - (signed) SysTick->VAL) / 1000.0 / (float) systick_divider)) * 1000000) - current_time_gyro;

			if (l_dt < 1000)
				continue;

			current_time_gyro = (millis / 1000.0 + ((float) ((signed) systick_divider - (signed) SysTick->VAL) / 1000.0 / (float) systick_divider)) * 1000000;
			current_time_ms_gyro = millis;

			dt = l_dt / 1000000.0;

			if (Sensors_Process(GYRO, 0.001))
			{
				current_time_gyro = (millis / 1000.0 + ((float) ((signed) systick_divider - (signed) SysTick->VAL) / 1000.0 / (float) systick_divider)) * 1000000;
				current_time_ms_gyro = millis;
			}

		}

		if (millis - current_time_ms_acc > 1)
		{
			if (Sensors_Process(ACC, 0))
			{
				current_time_ms_acc = millis;
			}

		}
		if (millis - current_time_ms_mag > 9)
		{
			if (Sensors_Process(MAG, 0))
			{
				current_time_ms_mag = millis;
			}
		}
	}
}

void IMU_Init(void)
{
	//Sensors_Init(GYRO);
	//Sensors_Init(ACC);

	/*Start the magnometer*/
	Sensors_Init(MAG);

	ekf_init();
	ADC12_Init();
}

void quad_init(void)
{

	/*Enable sysconfig*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	/*Turn on LED's*/
	GPIOInit(LED3);
	GPIOInit(LED4);
	GPIOInit(LED5);
	GPIOInit(LED6);

	/*Light up LED6 to signal turn on */
	GPIOOn(LED6);

	/*turn off auto zero funtions*/
	GPIOInit(GXY_AZ);
	GPIOInit(GZ_AZ);
	GPIOOff(GXY_AZ);
	GPIOOff(GZ_AZ);

	/* enable EEPROM GPIO */
	GPIOInit(EEPROM);

	/*If systick cannot be set, light up LED 3..*/
	if (SysTick_Config(systick_divider))
	{
		GPIOOn(LED3);
	}

	/*USART*/
	COMInit(USART_COM1);

	sync_printf("HI");

	/*init i2c modules*/
	I2C_LowLevel_Init(I2C_COM1);
	I2C_LowLevel_Init(I2C_COM2);

	/*Init the External config, just the button*/
	EXTILine0_Config();

	/*Init the IMU sensors*/
	//IMU_Init();

	/*Initilize the motor controls*/
	MotorControl_Init();

	/*Init input caputre channels*/
	TIM_PWM_Ctrl_In_Init(CH1);
	TIM_PWM_Ctrl_In_Init(CH2);
	TIM_PWM_Ctrl_In_Init(CH3);
	TIM_PWM_Ctrl_In_Init(CH4);

	//TIM_Init();
}



void inc_millis(void)
{
	millis++;
}
