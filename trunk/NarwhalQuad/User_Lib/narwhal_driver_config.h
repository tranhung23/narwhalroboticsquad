/*
 * narwhal_config.h
 *
 *  Created on: May 16, 2012
 *      Author: GrubyGrub
 */

#ifndef NARWHAL_CONFIG_H
#define NARWHAL_CONFIG_H

#define USART_ECHO 1

/*--------------------------------------------------------------------------------------
 *
 * USART Defines
 *
 *--------------------------------------------------------------------------------------
 */
/* Definition for COM1 resources ********************************************/
#define COM1                        USART6
#define COM1_CLK                    RCC_APB2Periph_USART6

#define COM1_TX_PIN                 GPIO_Pin_6
#define COM1_TX_GPIO_PORT           GPIOC
#define COM1_TX_GPIO_CLK            RCC_AHB1Periph_GPIOC
#define COM1_TX_SOURCE              GPIO_PinSource6
#define COM1_TX_AF                  GPIO_AF_USART6

#define COM1_RX_PIN                 GPIO_Pin_7
#define COM1_RX_GPIO_PORT           GPIOC
#define COM1_RX_GPIO_CLK            RCC_AHB1Periph_GPIOC
#define COM1_RX_SOURCE              GPIO_PinSource7
#define COM1_RX_AF                  GPIO_AF_USART6

#define COM1_IRQn                   USART6_IRQn

/* Definition for DMAx resources **********************************************/
#define COM1_DR_ADDRESS                ((uint32_t)COM1 + 0x04)

#define COM1_DMA                       DMA2
#define COM1_DMA_CLK                   RCC_AHB1Periph_DMA2

#define COM1_TX_DMA_CHANNEL            DMA_Channel_5
#define COM1_TX_DMA_STREAM             DMA2_Stream6
#define COM1_TX_DMA_FLAG_FEIF          DMA_FLAG_FEIF6
#define COM1_TX_DMA_FLAG_DMEIF         DMA_FLAG_DMEIF6
#define COM1_TX_DMA_FLAG_TEIF          DMA_FLAG_TEIF6
#define COM1_TX_DMA_FLAG_HTIF          DMA_FLAG_HTIF6
#define COM1_TX_DMA_FLAG_TCIF          DMA_FLAG_TCIF6

#define COM1_RX_DMA_CHANNEL            DMA_Channel_5
#define COM1_RX_DMA_STREAM             DMA2_Stream1
#define COM1_RX_DMA_FLAG_FEIF          DMA_FLAG_FEIF1
#define COM1_RX_DMA_FLAG_DMEIF         DMA_FLAG_DMEIF1
#define COM1_RX_DMA_FLAG_TEIF          DMA_FLAG_TEIF1
#define COM1_RX_DMA_FLAG_HTIF          DMA_FLAG_HTIF1
#define COM1_RX_DMA_FLAG_TCIF          DMA_FLAG_TCIF1

#define COM1_DMA_TX_IRQn               DMA2_Stream6_IRQn
#define COM1_DMA_RX_IRQn               DMA2_Stream1_IRQn
/*Overwriting orginal method pointers*/

#define COM1_DMA_TX_IRQHandler         DMA2_Stream6_IRQHandler
#define COM1_DMA_RX_IRQHandler         DMA2_Stream1_IRQHandler
#define COM1_TX_DMA_PREPRIO               8
#define COM1_TX_DMA_SUBPRIO               8
#define COM1_RX_DMA_PREPRIO               8
#define COM1_RX_DMA_SUBPRIO               7

/*--------------------------------------------------------------------------------------
 *
 * ADC Defines
 *
 *--------------------------------------------------------------------------------------
 */

//These are the places in the ADCDualConvertedValue array in which the gyro/acc values are
#define GYRO_X 0
#define GYRO_Y 2
#define GYRO_Z 4

#define ACC_X 1
#define ACC_Y 3
#define ACC_Z 5

#define MAG_X 0
#define MAG_Y 0
#define MAG_Z 0

#define ADC_CCR_ADDRESS    ((uint32_t)0x40012308)

/*--------------------------------------------------------------------------------------
 *
 * GPIO Defines
 *
 *--------------------------------------------------------------------------------------
 */

#define GPIOn                            7

#define COMn                             1
#define I2Cn                             2

#define LED4_PIN                         GPIO_Pin_0
#define LED4_GPIO_PORT                   GPIOE
#define LED4_GPIO_CLK                    RCC_AHB1Periph_GPIOE

#define LED3_PIN                         GPIO_Pin_1
#define LED3_GPIO_PORT                   GPIOE
#define LED3_GPIO_CLK                    RCC_AHB1Periph_GPIOE

#define LED5_PIN                         GPIO_Pin_2
#define LED5_GPIO_PORT                   GPIOE
#define LED5_GPIO_CLK                    RCC_AHB1Periph_GPIOE

#define LED6_PIN                         GPIO_Pin_3
#define LED6_GPIO_PORT                   GPIOE
#define LED6_GPIO_CLK                    RCC_AHB1Periph_GPIOE

#define GXY_AZ_PIN                       GPIO_Pin_1
#define GXY_AZ_GPIO_PORT                 GPIOD
#define GXY_AZ_GPIO_CLK                  RCC_AHB1Periph_GPIOD

#define GZ_AZ_PIN                        GPIO_Pin_0
#define GZ_AZ_GPIO_PORT                  GPIOD
#define GZ_AZ_GPIO_CLK                   RCC_AHB1Periph_GPIOD

#define EEPROM_PIN						 GPIO_Pin_4
#define EEPROM_GPIO_PORT				 GPIOE
#define EEPROM_GPIO_CLK					 RCC_AHB1Periph_GPIOE

/*--------------------------------------------------------------------------------------
 *
 * External Button Defines
 *
 *--------------------------------------------------------------------------------------
 */
#define BUTTONn                          1

/**
 * @brief Wakeup push-button
 */
#define USER_BUTTON_PIN                GPIO_Pin_0
#define USER_BUTTON_GPIO_PORT          GPIOA
#define USER_BUTTON_GPIO_CLK           RCC_AHB1Periph_GPIOA
#define USER_BUTTON_EXTI_LINE          EXTI_Line0
#define USER_BUTTON_EXTI_PORT_SOURCE   EXTI_PortSourceGPIOA
#define USER_BUTTON_EXTI_PIN_SOURCE    EXTI_PinSource0
#define USER_BUTTON_EXTI_IRQn          EXTI0_IRQn
#define USER_BUTTON_EXTI_PREPRIO	   0x15
#define USER_BUTTON_EXTI_SUBPRIO	   0x0

#define EXTI0_IRQHandler		 	   EXTI0_IRQHandler

/*--------------------------------------------------------------------------------------
 *
 * I2C Defines
 *
 *--------------------------------------------------------------------------------------
 */

#define I2C1_CLK                    RCC_APB1Periph_I2C1

#define I2C1_SPEED					400000
#define I2C2_SPEED					200000

/*I2C1, sensor array*/
#define I2C1_Event_IRQn 			I2C1_EV_IRQn
#define I2C1_Error_IRQn 			I2C1_ER_IRQn

/*overwrite orginal handlers*/
#define I2C1_Event_IRQHandler 		I2C1_EV_IRQHandler
#define I2C1_Error_IRQHandler 		I2C1_ER_IRQHandler

#define I2C1_PREPRIO          6
#define I2C1_SUBPRIO          7

#define I2C1_SCL_PIN                  GPIO_Pin_6                  /* PB.06 */
#define I2C1_SCL_GPIO_PORT            GPIOB                       /* GPIOB */
#define I2C1_SCL_GPIO_CLK             RCC_AHB1Periph_GPIOB
#define I2C1_SCL_SOURCE               GPIO_PinSource6
#define I2C1_SCL_AF                   GPIO_AF_I2C1

#define I2C1_SDA_PIN                  GPIO_Pin_9                  /* PB.09 */
#define I2C1_SDA_GPIO_PORT            GPIOB                       /* GPIOB */
#define I2C1_SDA_GPIO_CLK             RCC_AHB1Periph_GPIOB
#define I2C1_SDA_SOURCE               GPIO_PinSource9
#define I2C1_SDA_AF                   GPIO_AF_I2C1

#define I2C1_DMA                      DMA1
#define I2C1_DMA_CHANNEL              DMA_Channel_1
#define I2C1_DMA_STREAM_TX            DMA1_Stream6
#define I2C1_DMA_STREAM_RX            DMA1_Stream0
#define I2C1_DMA_CLK                  RCC_AHB1Periph_DMA1
#define I2C1_DR_Address               ((uint32_t)I2C1 + 0x10)

#define I2C1_DMA_TX_IRQn              DMA1_Stream6_IRQn
#define I2C1_DMA_RX_IRQn              DMA1_Stream0_IRQn
/*overwrite orginal handlers method ptr*/
#define I2C1_DMA_TX_IRQHandler        DMA1_Stream6_IRQHandler
#define I2C1_DMA_RX_IRQHandler        DMA1_Stream0_IRQHandler
#define I2C1_DMA_PREPRIO              6
#define I2C1_DMA_SUBPRIO              6

#define I2C1_TX_DMA_FLAG_FEIF             DMA_FLAG_FEIF6
#define I2C1_TX_DMA_FLAG_DMEIF            DMA_FLAG_DMEIF6
#define I2C1_TX_DMA_FLAG_TEIF             DMA_FLAG_TEIF6
#define I2C1_TX_DMA_FLAG_HTIF             DMA_FLAG_HTIF6
#define I2C1_TX_DMA_FLAG_TCIF             DMA_FLAG_TCIF6
#define I2C1_RX_DMA_FLAG_FEIF             DMA_FLAG_FEIF0
#define I2C1_RX_DMA_FLAG_DMEIF            DMA_FLAG_DMEIF0
#define I2C1_RX_DMA_FLAG_TEIF             DMA_FLAG_TEIF0
#define I2C1_RX_DMA_FLAG_HTIF             DMA_FLAG_HTIF0
#define I2C1_RX_DMA_FLAG_TCIF             DMA_FLAG_TCIF0

#define I2C_DIRECTION_TX                 0
#define I2C_DIRECTION_RX                 1

#define TIMER1						RCC_APB1Periph_TIM3
#define TIMER1_IRQn					TIM3_IRQn
#define TIMER1_PREPRIO              4
#define TIMER1_SUBPRIO              0

/*I2C2, */
#define I2C2_CLK                    RCC_APB1Periph_I2C2

/*I2C1, sensor array*/
#define I2C2_Event_IRQn 			I2C2_EV_IRQn
#define I2C2_Error_IRQn 			I2C2_ER_IRQn

/*overwrite orginal handlers*/
#define I2C2_Event_IRQHandler 		I2C2_EV_IRQHandler
#define I2C2_Error_IRQHandler 		I2C2_ER_IRQHandler

#define I2C2_PREPRIO          5
#define I2C2_SUBPRIO          7

#define I2C2_SCL_PIN                  GPIO_Pin_10                  /* PB.06 */
#define I2C2_SCL_GPIO_PORT            GPIOB                       /* GPIOB */
#define I2C2_SCL_GPIO_CLK             RCC_AHB1Periph_GPIOB
#define I2C2_SCL_SOURCE               GPIO_PinSource10
#define I2C2_SCL_AF                   GPIO_AF_I2C2

#define I2C2_SDA_PIN                  GPIO_Pin_11                  /* PB.09 */
#define I2C2_SDA_GPIO_PORT            GPIOB                       /* GPIOB */
#define I2C2_SDA_GPIO_CLK             RCC_AHB1Periph_GPIOB
#define I2C2_SDA_SOURCE               GPIO_PinSource11
#define I2C2_SDA_AF                   GPIO_AF_I2C2

#define I2C2_DMA                      DMA1
#define I2C2_DMA_CHANNEL              DMA_Channel_7
#define I2C2_DMA_STREAM_TX            DMA1_Stream7
#define I2C2_DMA_STREAM_RX            DMA1_Stream2
#define I2C2_DMA_CLK                  RCC_AHB1Periph_DMA1
#define I2C2_DR_Address               ((uint32_t)I2C2 + 0x10)

#define I2C2_DMA_TX_IRQn              DMA1_Stream7_IRQn
#define I2C2_DMA_RX_IRQn              DMA1_Stream2_IRQn
/*overwrite orginal handlers method ptr*/
#define I2C2_DMA_TX_IRQHandler        DMA1_Stream7_IRQHandler
#define I2C2_DMA_RX_IRQHandler        DMA1_Stream2_IRQHandler
#define I2C2_DMA_PREPRIO              5
#define I2C2_DMA_SUBPRIO              6

#define I2C2_TX_DMA_FLAG_FEIF             DMA_FLAG_FEIF7
#define I2C2_TX_DMA_FLAG_DMEIF            DMA_FLAG_DMEIF7
#define I2C2_TX_DMA_FLAG_TEIF             DMA_FLAG_TEIF7
#define I2C2_TX_DMA_FLAG_HTIF             DMA_FLAG_HTIF7
#define I2C2_TX_DMA_FLAG_TCIF             DMA_FLAG_TCIF7
#define I2C2_RX_DMA_FLAG_FEIF             DMA_FLAG_FEIF2
#define I2C2_RX_DMA_FLAG_DMEIF            DMA_FLAG_DMEIF2
#define I2C2_RX_DMA_FLAG_TEIF             DMA_FLAG_TEIF2
#define I2C2_RX_DMA_FLAG_HTIF             DMA_FLAG_HTIF2
#define I2C2_RX_DMA_FLAG_TCIF             DMA_FLAG_TCIF2

/*--------------------------------------------------------------------------------------
 *
 * Timer Defines
 *
 *--------------------------------------------------------------------------------------
 */

/*1 mhz*/
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


#define RC4_TIMER						TIM4
#define RC4_TIMER_CLK					RCC_APB1Periph_TIM4
#define RC4_TIMER_IRQn					TIM4_IRQn
#define RC4_TIMER_IRQ_PREPRIO			RC_TIMER_IRQ_PREPRIO
#define RC4_TIMER_IRQ_SUBPRIO			12

#define usTimer_IRQHandler 					TIM5_IRQHandler

#define MICROS_TIMER_TIMEBASE				1000000
#define MICROS_TIMER						TIM5
#define MICROS_TIMER_CLK					RCC_APB1Periph_TIM5
#define MICROS_TIMER_IRQn					TIM5_IRQn
#define MICROS_TIMER_IRQ_PREPRIO			2
#define MICROS_TIMER_IRQ_SUBPRIO			0


///*
// * TIMER defines
// */
//#define TIM_I2C1_READ_CLK					RCC_APB1Periph_TIM3
//#define TIM_I2C1_READ						TIM3
//#define TIM_I2C1_READ_IRQn					TIM3_IRQn
//#define TIM_I2C1_READ_PREPRIO               4
//#define TIM_I2C1_READ_SUBPRIO               0
//
///*Timer is running at 1mhz, so each division is 1uS*/
//#define CCR1_Val 	 40961 /**/
//#define CCR2_Val 	 5555 /*HMC data rate - 100 hz*/
//#define CCR3_Val 	 2700 /*AXDL data rate 400 hz*/
//#define CCR4_Val	 2000 /*ITG 3200 at 2khz*/
//
//#define Sensors_IRQHandler	TIM3_IRQHandler

#endif
