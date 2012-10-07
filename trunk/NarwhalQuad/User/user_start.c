#include <narwhal_GPIO.h>
#include <narwhal_I2C.h>
#include <narwhal_BTN.h>
#include <narwhal_MICRO_TIMER.h>
#include <narwhal_TIMER.h>
#include <narwhal_ADC.h>

#include "user_start.h"

#include <stdio.h>
#include <settings.h>
#include <sensors.h>
#include <RC_Control.h>


#define PRINTFLASH 1

OS_STK *narwhalInitStack;

void IMU_Init(void) {

	/*Start the magnometer*/
	Sensors_Init(MAG);
	//ekf_init();
	ADC_SENSOR_Init();
}

void Drivers_Init_Task(void) {
	/*Enable sysconfig*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	/*Init GPIO''s*/
	GPIOInit(LED3);
	GPIOInit(LED4);
	GPIOInit(LED5);
	GPIOInit(LED6);

	/*Light up LED6 to signal turn on */
	//TODO: Create process to toggle LED's for indicator
	GPIOOn(LED6);
	GPIOOn(LED3);

	/*turn off auto zero funtions*/
	GPIOInit(GXY_AZ);
	GPIOInit(GZ_AZ);
	GPIOOff(GXY_AZ);
	GPIOOff(GZ_AZ);

#ifdef EEPROM
	GPIOInit(EEPROM);
#endif
	/*USART*/
	COMInit(USART_COM1);
	configFlashWrite();
	configFlashRead();
#ifdef PRINTFLASH
	for (int i = 0; i < CONFIG_NUM_PARAMS; i++)
	{
		sync_printf("%s:%f;\r\n", configParameterStrings[i], p[i]);
	}
#endif

	/*init i2c modules*/
	I2C_LowLevel_Init(I2C_COM1);
	I2C_LowLevel_Init(I2C_COM2);

	/*Init the External config, just the button*/
	EXTILine0_Config();

	/*Init the IMU sensors*/
	IMU_Init();

	/*Initlizize the RC input*/
	RC_Init();

	/*Init input caputre channels*/
	TIM_PWM_Ctrl_In_Init(CH1);
	TIM_PWM_Ctrl_In_Init(CH2);
	TIM_PWM_Ctrl_In_Init(CH3);
	TIM_PWM_Ctrl_In_Init(CH4);

	/*us counter*/
	usTimerInit();
	//CoExitTask();
	//TIM_Init();

	//EEPROM_init();

	setvbuf( stdout, 0, _IONBF, 0 );
	setvbuf( stdin, 0, _IONBF, 0 );
}
