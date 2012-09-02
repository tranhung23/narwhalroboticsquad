/*
 * stm32f4_LED.c
 *
 *  Created on: Apr 12, 2012
 *      Author: GrubyGrub
 */

#include "stm32f4_quad.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_sdio.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_gpio.h"
#include "stdarg.h"
#include "stdio.h"

#include "sensors.h"


/* LED Def */
GPIO_TypeDef* GPIO_PORT[GPIOn] = {LED4_GPIO_PORT, LED3_GPIO_PORT, LED5_GPIO_PORT,
                                 LED6_GPIO_PORT, GXY_AZ_GPIO_PORT, GZ_AZ_GPIO_PORT, EEPROM_GPIO_PORT};
const uint16_t GPIO_PIN[GPIOn] = {LED4_PIN, LED3_PIN, LED5_PIN,
                                 LED6_PIN, GXY_AZ_PIN, GZ_AZ_PIN, EEPROM_PIN};
const uint32_t GPIO_CLK[GPIOn] = {LED4_GPIO_CLK, LED3_GPIO_CLK, LED5_GPIO_CLK,
                                 LED6_GPIO_CLK, GXY_AZ_GPIO_CLK, GZ_AZ_GPIO_CLK, EEPROM_GPIO_CLK};



/**
 * LED SECTION
 */

void GPIOInit(GPIO_PIN_TypeDef Led)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  /* Enable the GPIO_LED Clock */
  RCC_AHB1PeriphClockCmd(GPIO_CLK[Led], ENABLE);

  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_PIN[Led];
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIO_PORT[Led], &GPIO_InitStructure);
}

/**
  * @brief  Turns selected LED On.
  * @param  Led: Specifies the Led to be set on.
  *   This parameter can be one of following parameters:
  *     @arg LED4
  *     @arg LED3
  *     @arg LED5
  *     @arg LED6
  * @retval None
  */
void GPIOOn(GPIO_PIN_TypeDef Led)
{
  GPIO_PORT[Led]->BSRRL = GPIO_PIN[Led];
}

/**
  * @brief  Turns selected LED Off.
  * @param  Led: Specifies the Led to be set off.
  *   This parameter can be one of following parameters:
  *     @arg LED4
  *     @arg LED3
  *     @arg LED5
  *     @arg LED6
  * @retval None
  */
void GPIOOff(GPIO_PIN_TypeDef Led)
{
  GPIO_PORT[Led]->BSRRH = GPIO_PIN[Led];
}

/**
  * @brief  Toggles the selected LED.
  * @param  Led: Specifies the Led to be toggled.
  *   This parameter can be one of following parameters:
  *     @arg LED4
  *     @arg LED3
  *     @arg LED5
  *     @arg LED6
  * @retval None
  */
void GPIOToggle(GPIO_PIN_TypeDef Led)
{
  GPIO_PORT[Led]->ODR ^= GPIO_PIN[Led];
}
