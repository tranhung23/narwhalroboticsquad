/*
 * stm32f4_USART.h
 *
 *  Created on: Apr 18, 2012
 *      Author: kgu
 */

#ifndef NARWHAL_USART_H_
#define NARWHAL_USART_H_

#include <narwhal_top.h>

typedef enum
{
	USART_COM1 = 0
} COM_TypeDef;

void COMInit(COM_TypeDef COM);

extern int sync_printf(char * fmt, ...);
extern int async_printf(char * fmt, ...);
extern char * sync_scanf(unsigned int length);
extern char * async_scanf(unsigned int length);

/*private*/
void COM1_TX_COMPLETE(void);
void COM1_RX_COMPLETE(void);
void COM1_DMA_TX_IRQHandler(void);
void COM1_DMA_RX_IRQHandler(void);

#endif /* STM32F4_USART_H_ */
