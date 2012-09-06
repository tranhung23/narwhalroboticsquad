/*
 * quad_indicator.h
 *
 *  Created on: Feb 12, 2012
 *      Author: GrubyGrub
 */

#ifndef QUAD_INDICATOR_H_
#define QUAD_INDICATOR_H_

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"

// TODO: make a device tree that auto parse the define file, instead of manually defining the array
typedef enum
{
	LED4 = 0, LED3 = 1, LED5 = 2, LED6 = 3, GXY_AZ = 4, GZ_AZ = 5, EEPROM = 6
} GPIO_PIN_TypeDef;

typedef enum
{
	CH1 = 0, CH2 = 1, CH3 = 2, CH4 = 3
} PWM_CTRL_IN_TypeDef;

typedef enum
{
	BUTTON_USER = 0,
} Button_TypeDef;

typedef enum
{
	BUTTON_MODE_GPIO = 0, BUTTON_MODE_EXTI = 1
} ButtonMode_TypeDef;

typedef enum
{
	I2C_COM1 = 0, I2C_COM2 = 1
} I2C_COM_TypeDef;

typedef enum
{
	USART_COM1 = 0
} COM_TypeDef;

typedef struct
{
	uint8_t DataDirection;
	uint8_t DeviceAddr;
	uint32_t RegisterAddrs;
	uint16_t RegisterAddrLength;
	uint16_t RegisterAddrPtr;
	uint8_t RegisterValue;
	uint16_t TX_DataLength;
	uint16_t RX_DataLength;
	uint8_t var1;
	uint8_t var2;
	uint16_t var3;
} I2C_TRANSMISSION_TypeDef;

typedef enum
{
	IDLE, START_RX, START_TX, START_DMA, SEND_DEVICE_ADDR, SEND_ADDR_RX, SEND_REG_ADDR,SEND_MULTI_REG_ADDR, SEND_DATA, SEND_REG_DATA, STOP
} I2C_IRQ_STATE;

/*LED indicator Lights*/

#define GPIOn                            7

#define COMn                             1
#define I2Cn                             2

/*REV 2 of board
#define LED4_PIN                         GPIO_Pin_12
#define LED4_GPIO_PORT                   GPIOD
#define LED4_GPIO_CLK                    RCC_AHB1Periph_GPIOD

#define LED3_PIN                         GPIO_Pin_13
#define LED3_GPIO_PORT                   GPIOD
#define LED3_GPIO_CLK                    RCC_AHB1Periph_GPIOD

#define LED5_PIN                         GPIO_Pin_14
#define LED5_GPIO_PORT                   GPIOD
#define LED5_GPIO_CLK                    RCC_AHB1Periph_GPIOD

#define LED6_PIN                         GPIO_Pin_15
#define LED6_GPIO_PORT                   GPIOD
#define LED6_GPIO_CLK                    RCC_AHB1Periph_GPIOD
*/

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
/** @addtogroup LEVEL_BUTTON
 * @{
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

/*I2C starts at COM7
 * @brief  I2C EEPROM Interface pins
 */

#define I2C1_CLK                    RCC_APB1Periph_I2C1

#define I2C1_SPEED					400000
#define I2C2_SPEED					200000

/*I2C1, sensor array*/
#define I2C1_Event_IRQn 			I2C1_EV_IRQn
#define I2C1_Error_IRQn 			I2C1_ER_IRQn

/*overwrite orginal handlers*/
#define I2C1_EV_IRQHandler       I2C1_Event_IRQHandler
#define I2C1_ER_IRQHandler       I2C1_Error_IRQHandler

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
#define I2C2_EV_IRQHandler       I2C2_Event_IRQHandler
#define I2C2_ER_IRQHandler       I2C2_Error_IRQHandler

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

void GPIOInit(GPIO_PIN_TypeDef Led);
void GPIOOn(GPIO_PIN_TypeDef Led);
void GPIOOff(GPIO_PIN_TypeDef Led);
void GPIOToggle(GPIO_PIN_TypeDef Led);

void PBInit(Button_TypeDef Button, ButtonMode_TypeDef Button_Mode);
uint32_t PBGetState(Button_TypeDef Button);
void EXTILine0_Config(void);

/*USART Items*/
void COMInit(COM_TypeDef COM);
void COM1_DMA_TX_IRQHandler(void);
void COM1_DMA_RX_IRQHandler(void);
void COM1_TX_COMPLETE(void);
void COM1_RX_COMPLETE(void);

int async_printf(char * fmt, ...);
int sync_printf(char * fmt, ...);
char * sync_scanf(unsigned int length);
char * async_scanf(unsigned int length);

/*I2C Items*/

void I2C_Device_Init(I2C_COM_TypeDef COM);
void I2C_LowLevel_Init(I2C_COM_TypeDef COM);

uint8_t I2C_ReadDeviceRegister(I2C_COM_TypeDef COM, uint8_t DeviceAddr, uint8_t RegisterAddr, uint16_t RX_DataLength, uint32_t ReadBuffer);
uint8_t I2C_ReadDeviceRegister_async(I2C_COM_TypeDef COM, uint8_t DeviceAddr, uint8_t RegisterAddr, uint16_t RX_DataLength, uint32_t ReadBuffer);
uint8_t I2C_ReadDeviceLongRegister_async(I2C_COM_TypeDef COM, uint8_t DeviceAddr, uint32_t RegisterAddrsPointer, uint16_t RegisterLength, uint32_t ReadBuffer, uint16_t RX_DataLength );
uint8_t I2C_ReadDeviceLongRegister(I2C_COM_TypeDef COM, uint8_t DeviceAddr, uint32_t RegisterAddrsPointer, uint16_t RegisterLength, uint32_t ReadBuffer, uint16_t RX_DataLength);
uint8_t I2C_WriteDeviceRegister(I2C_COM_TypeDef COM, uint8_t DeviceAddr, uint8_t RegisterAddr, uint8_t RegisterValue, uint8_t Tx_dataLength);
uint8_t I2C_WriteDeviceRegister_async(I2C_COM_TypeDef COM, uint8_t DeviceAddr, uint8_t RegisterAddr, uint8_t RegisterValue, uint8_t Tx_dataLength);
uint8_t I2C_WriteDeviceRegisterBuffer_async(I2C_COM_TypeDef COM, uint8_t DeviceAddr, uint8_t RegisterAddr, uint32_t WriteBuffer, uint8_t TX_DataLength);
uint8_t I2C_WriteDeviceLongRegister(I2C_COM_TypeDef COM, uint8_t DeviceAddr, uint32_t RegisterAddrsPointer, uint16_t RegisterLength, uint32_t WriteBuffer, uint16_t TX_DataLength);
uint8_t I2C_WriteDeviceLongRegister_async(I2C_COM_TypeDef COM, uint8_t DeviceAddr, uint32_t RegisterAddrsPointer, uint16_t RegisterLength, uint32_t WriteBuffer, uint16_t TX_DataLength );
uint8_t I2C_WriteDevice(I2C_COM_TypeDef COM, uint8_t DeviceAddr, uint8_t data, uint8_t Tx_dataLength);
uint8_t I2C_WriteDevice_async(I2C_COM_TypeDef COM, uint8_t DeviceAddr, uint8_t data, uint8_t Tx_dataLength);

void I2C_Event_Helper(I2C_COM_TypeDef I2C_COM);
void I2C_Error_Helper(I2C_COM_TypeDef I2C_COM);

void I2C1_Error_IRQHandler(void);
void I2C1_Event_IRQHandler(void);

void I2C2_Error_IRQHandler(void);
void I2C2_Event_IRQHandler(void);


#endif /* QUAD_INDICATOR_H_ */
