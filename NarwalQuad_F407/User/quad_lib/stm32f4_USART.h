/*
 * stm32f4_USART.h
 *
 *  Created on: Apr 18, 2012
 *      Author: GrubyGrub
 */

#ifndef STM32F4_USART_H_
#define STM32F4_USART_H_

#include "stm32f4xx.h"

/**
 * @}
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


#endif /* STM32F4_USART_H_ */
