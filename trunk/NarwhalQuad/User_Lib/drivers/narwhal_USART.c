/*
 * stm32f4_USART.c
 *
 *  Created on: Apr 12, 2012
 *      Author: GrubyGrub
 */

#include "narwhal_USART.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

unsigned char COM1_TX_BUFFER[256];
unsigned char COM1_RX_BUFFER[256];

/*USART Typedef*/
USART_TypeDef* COM_USART[COMn] = { COM1 };
GPIO_TypeDef* COM_TX_PORT[COMn] = { COM1_TX_GPIO_PORT };
GPIO_TypeDef* COM_RX_PORT[COMn] = { COM1_RX_GPIO_PORT };
const uint32_t COM_USART_CLK[COMn] = { COM1_CLK };

const uint32_t COM_TX_PORT_CLK[COMn] = { COM1_TX_GPIO_CLK };
const uint32_t COM_RX_PORT_CLK[COMn] = { COM1_RX_GPIO_CLK };
const uint16_t COM_TX_PIN[COMn] = { COM1_TX_PIN };
const uint16_t COM_RX_PIN[COMn] = { COM1_RX_PIN };

const uint16_t COM_TX_PIN_SOURCE[COMn] = { COM1_TX_SOURCE };
const uint16_t COM_RX_PIN_SOURCE[COMn] = { COM1_RX_SOURCE };
const uint16_t COM_TX_AF[COMn] = { COM1_TX_AF };
const uint16_t COM_RX_AF[COMn] = { COM1_RX_AF };

const uint32_t COM_TX_BUFFER[COMn] = { (uint32_t) COM1_TX_BUFFER };
const uint32_t COM_RX_BUFFER[COMn] = { (uint32_t) COM1_RX_BUFFER };

/*DMA Vars*/
const uint32_t COM_DR_ADDRESS[COMn] = { COM1_DR_ADDRESS };
const uint32_t COM_DMA_CLK[COMn] = { RCC_AHB1Periph_DMA2 };
DMA_TypeDef * COM_DMA[COMn] = { DMA2 };

const uint32_t COM_TX_DMA_CHANNEL[COMn] = { DMA_Channel_5 };
DMA_Stream_TypeDef * COM_TX_DMA_STREAM[COMn] = { DMA2_Stream6 };

const uint32_t COM_TX_DMA_FLAG_FEIF[COMn] = { DMA_FLAG_FEIF6 };
const uint32_t COM_TX_DMA_FLAG_DMEIF[COMn] = { DMA_FLAG_DMEIF6 };
const uint32_t COM_TX_DMA_FLAG_TEIF[COMn] = { DMA_FLAG_TEIF6 };
const uint32_t COM_TX_DMA_FLAG_HTIF[COMn] = { DMA_FLAG_HTIF6 };
const uint32_t COM_TX_DMA_FLAG_TCIF[COMn] = { DMA_FLAG_TCIF6 };

const uint32_t COM_TX_DMA_IT_TCIF[COMn] = { DMA_IT_TCIF6 };

const uint32_t COM_RX_DMA_CHANNEL[COMn] = { DMA_Channel_5 };
DMA_Stream_TypeDef * COM_RX_DMA_STREAM[COMn] = { DMA2_Stream1 };
const uint32_t COM_RX_DMA_FLAG_FEIF[COMn] = { DMA_FLAG_FEIF1 };
const uint32_t COM_RX_DMA_FLAG_DMEIF[COMn] = { DMA_FLAG_DMEIF1 };
const uint32_t COM_RX_DMA_FLAG_TEIF[COMn] = { DMA_FLAG_TEIF1 };
const uint32_t COM_RX_DMA_FLAG_HTIF[COMn] = { DMA_FLAG_HTIF1 };
const uint32_t COM_RX_DMA_FLAG_TCIF[COMn] = { DMA_FLAG_TCIF1 };

const uint32_t COM_RX_DMA_IT_TCIF[COMn] = { DMA_IT_TCIF1 };

const uint32_t COM_DMA_TX_IRQn[COMn] = { DMA2_Stream6_IRQn };
const uint32_t COM_DMA_RX_IRQn[COMn] = { DMA2_Stream1_IRQn };

const uint32_t COM_TX_DMA_PREPRIO[COMn] = { COM1_TX_DMA_PREPRIO };
const uint32_t COM_TX_DMA_SUBPRIO[COMn] = { COM1_TX_DMA_SUBPRIO };

const uint32_t COM_RX_DMA_PREPRIO[COMn] = { COM1_RX_DMA_PREPRIO };
const uint32_t COM_RX_DMA_SUBPRIO[COMn] = { COM1_RX_DMA_SUBPRIO };

/**
 * USART COMM
 */

void COMInit(COM_TypeDef COM)
{
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable GPIO clock */
	RCC_AHB1PeriphClockCmd(COM_TX_PORT_CLK[COM] | COM_RX_PORT_CLK[COM] | RCC_AHB1Periph_GPIOD, ENABLE);

	if (COM == USART_COM1)
	{
		/* Enable UART clock */
		RCC_APB2PeriphClockCmd(COM_USART_CLK[COM], ENABLE);
	}

	/* Connect PXx to USARTx_Tx*/
	GPIO_PinAFConfig(COM_TX_PORT[COM], COM_TX_PIN_SOURCE[COM], COM_TX_AF[COM]);

	/* Connect PXx to USARTx_Rx*/
	GPIO_PinAFConfig(COM_RX_PORT[COM], COM_RX_PIN_SOURCE[COM], COM_RX_AF[COM]);

	/* Configure USART Tx as alternate function  */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;

	GPIO_InitStructure.GPIO_Pin = COM_TX_PIN[COM];
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(COM_TX_PORT[COM], &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = COM_RX_PIN[COM];
	GPIO_Init(COM_RX_PORT[COM], &GPIO_InitStructure);

	/*USART Init code*/
	USART_OverSampling8Cmd(COM_USART[COM], ENABLE);
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(COM_USART[COM], &USART_InitStructure);

	/*USART Interrupt code*/

	/*USART TX interrupt*/
	NVIC_InitStructure.NVIC_IRQChannel = COM_DMA_TX_IRQn[COM];
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = COM_TX_DMA_PREPRIO[COM];
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = COM_TX_DMA_SUBPRIO[COM];
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStructure);

	/*USART RX interrupt*/
	NVIC_InitStructure.NVIC_IRQChannel = COM_DMA_RX_IRQn[COM];
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = COM_RX_DMA_PREPRIO[COM];
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = COM_RX_DMA_SUBPRIO[COM];
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStructure);

	RCC_AHB1PeriphClockCmd(COM_DMA_CLK[COM], ENABLE);

	/*USART TX clean up*/
	DMA_ClearFlag(COM_TX_DMA_STREAM[COM], COM_TX_DMA_FLAG_FEIF[COM] | COM_TX_DMA_FLAG_DMEIF[COM] | COM_TX_DMA_FLAG_TEIF[COM] | COM_TX_DMA_FLAG_HTIF[COM] | COM_TX_DMA_FLAG_TCIF[COM]);
	DMA_Cmd(COM_TX_DMA_STREAM[COM], DISABLE);
	DMA_DeInit(COM_TX_DMA_STREAM[COM]);

	/*USART RX Clean up*/
	DMA_ClearFlag(COM_RX_DMA_STREAM[COM], COM_RX_DMA_FLAG_FEIF[COM] | COM_RX_DMA_FLAG_DMEIF[COM] | COM_RX_DMA_FLAG_TEIF[COM] | COM_RX_DMA_FLAG_HTIF[COM] | COM_RX_DMA_FLAG_TCIF[COM]);
	DMA_Cmd(COM_RX_DMA_STREAM[COM], DISABLE);
	DMA_DeInit(COM_RX_DMA_STREAM[COM]);

	/*USART TX Config*/
	DMA_InitStructure.DMA_Channel = COM_TX_DMA_CHANNEL[COM];
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) COM_DR_ADDRESS[COM];
	DMA_InitStructure.DMA_Memory0BaseAddr = COM_TX_BUFFER[COM]; /* This parameter will be configured durig communication */
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral; /* This parameter will be configured durig communication */
	DMA_InitStructure.DMA_BufferSize = 0xFF; /* This parameter will be configured durig communication */
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

	DMA_Init(COM_TX_DMA_STREAM[COM], &DMA_InitStructure);
	//DMA_ITConfig(COM_TX_DMA_STREAM[COM], DMA_IT_TC, ENABLE);

	/*USART RX Config*/
	DMA_InitStructure.DMA_Channel = COM_RX_DMA_CHANNEL[COM];
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) COM_DR_ADDRESS[COM];
	DMA_InitStructure.DMA_Memory0BaseAddr = COM_RX_BUFFER[COM]; /* This parameter will be configured durig communication */
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory; /* This parameter will be configured durig communication */

	DMA_Init(COM_RX_DMA_STREAM[COM], &DMA_InitStructure);
	//DMA_ITConfig(COM_RX_DMA_STREAM[COM], DMA_IT_TC, ENABLE);

	/* Enable USART */
	USART_Cmd(COM_USART[COM], ENABLE);
}



/*
 * COM1 DMA Handlers
 */
void COM1_DMA_TX_IRQHandler(void)
{
	if (DMA_GetITStatus(COM_TX_DMA_STREAM[USART_COM1], COM_TX_DMA_IT_TCIF[USART_COM1]))
	{
		COM1_TX_COMPLETE();

		DMA_ITConfig(COM_TX_DMA_STREAM[USART_COM1], DMA_IT_TC, DISABLE);
	}
}

void COM1_RX_COMPLETE(void)
{
	/*Clear the pending bits*/
	DMA_ClearITPendingBit(COM_RX_DMA_STREAM[USART_COM1], COM_RX_DMA_IT_TCIF[USART_COM1]);

	DMA_ClearFlag(COM_RX_DMA_STREAM[USART_COM1], COM_RX_DMA_FLAG_HTIF[USART_COM1] | COM_RX_DMA_FLAG_TCIF[USART_COM1]);

	/* Disable the DMA Rx Stream */
	DMA_Cmd(COM_RX_DMA_STREAM[USART_COM1], DISABLE);

	/* Disable the USART Tx DMA request */
	USART_DMACmd(COM_USART[USART_COM1], USART_DMAReq_Rx, DISABLE);
}

void COM1_TX_COMPLETE(void)
{
	/*Clear the pending bits*/
	DMA_ClearITPendingBit(COM_TX_DMA_STREAM[USART_COM1], COM_TX_DMA_IT_TCIF[USART_COM1]);

	/*Clear the flags*/
	DMA_ClearFlag(COM_TX_DMA_STREAM[USART_COM1], COM_TX_DMA_FLAG_HTIF[USART_COM1] | COM_TX_DMA_FLAG_TCIF[USART_COM1]);

	/* Disable the DMA Streams */
	DMA_Cmd(COM_TX_DMA_STREAM[USART_COM1], DISABLE);

	/* Disable the USART Tx DMA request */
	USART_DMACmd(COM_USART[USART_COM1], USART_DMAReq_Tx, DISABLE);
}

/*COM1 RX area*/
void COM1_DMA_RX_IRQHandler(void)
{
	if (DMA_GetITStatus(COM_RX_DMA_STREAM[USART_COM1], COM_RX_DMA_IT_TCIF[USART_COM1]))
	{
		COM1_RX_COMPLETE();

		DMA_ITConfig(COM_RX_DMA_STREAM[USART_COM1], DMA_IT_TC, DISABLE);
	}
}


/*---------------------------------------------------------------*/



/*
 * USART Printf's
 */
char * async_scanf(unsigned int length)
{
	if (DMA_GetCmdStatus(COM_RX_DMA_STREAM[USART_COM1]))
		return (char *) COM_RX_BUFFER[USART_COM1];

	/*Config the recieve buffer*/
	COM_RX_DMA_STREAM[USART_COM1]->NDTR = (uint16_t) length;
	((char *) COM_RX_BUFFER[USART_COM1])[length] = 0;

	DMA_ITConfig(COM_RX_DMA_STREAM[USART_COM1], DMA_IT_TC, ENABLE);

	DMA_Cmd(COM_RX_DMA_STREAM[USART_COM1], ENABLE);
	USART_DMACmd(COM_USART[USART_COM1], USART_DMAReq_Rx, ENABLE);

	return (char *) COM_RX_BUFFER[USART_COM1];
}

/*scan for a charactor, input lenght of string, will wait until char has been recieved*/
char * sync_scanf(unsigned int length)
{
	if (DMA_GetCmdStatus(COM_RX_DMA_STREAM[USART_COM1]))
		return (char *) COM_RX_BUFFER[USART_COM1];

	/*Config the recieve buffer*/
	COM_RX_DMA_STREAM[USART_COM1]->NDTR = (uint16_t) length;
	((char *) COM_RX_BUFFER[USART_COM1])[length] = 0;

	DMA_Cmd(COM_RX_DMA_STREAM[USART_COM1], ENABLE);
	USART_DMACmd(COM_USART[USART_COM1], USART_DMAReq_Rx, ENABLE);

	while (DMA_GetFlagStatus(COM_RX_DMA_STREAM[USART_COM1], COM_RX_DMA_FLAG_TCIF[USART_COM1]) == RESET)
	{
	}
	COM1_RX_COMPLETE();
	return (char *) COM_RX_BUFFER[USART_COM1];
}

/*Printf while waiting for it to finish, DMA based*/
int sync_printf(char * fmt, ...)
{
	if (DMA_GetCmdStatus(COM_TX_DMA_STREAM[USART_COM1]))
		return -1;
	int n;
	va_list ap;
	va_start(ap, fmt);
	n = vsprintf((char *) COM_TX_BUFFER[USART_COM1], fmt, ap);
	va_end(ap);

	COM_TX_DMA_STREAM[USART_COM1]->NDTR = (uint32_t) n;
	USART_DMACmd(COM_USART[USART_COM1], USART_DMAReq_Tx, ENABLE);
	/* Clear the TC bit in the SR register by writing 0 to it */
	USART_ClearFlag(COM_USART[USART_COM1], USART_FLAG_TC);
	/* Enable the DMA TX Stream, USART will start sending the command code (2bytes) */
	DMA_Cmd(COM_TX_DMA_STREAM[USART_COM1], ENABLE);

	/*wait for sigal to be reset*/
	while ((USART_GetFlagStatus(COM_USART[USART_COM1], USART_FLAG_TC) == RESET))
	{
	}
	COM1_TX_COMPLETE();

	return n;
}

/*Async printf, DMA interrupt based*/
int async_printf(char * fmt, ...)
{
	if (DMA_GetCmdStatus(COM_TX_DMA_STREAM[USART_COM1]))
		return -1;

	int n;
	va_list ap;
	va_start(ap, fmt);
	n = vsprintf((char *) COM_TX_BUFFER[USART_COM1], fmt, ap);
	va_end(ap);

	COM_TX_DMA_STREAM[USART_COM1]->NDTR = (uint32_t) n;
	DMA_ITConfig(COM_TX_DMA_STREAM[USART_COM1], DMA_IT_TC, ENABLE);
	USART_DMACmd(COM_USART[USART_COM1], USART_DMAReq_Tx, ENABLE);
	/* Clear the TC bit in the SR register by writing 0 to it */
	USART_ClearFlag(COM_USART[USART_COM1], USART_FLAG_TC);
	/* Enable the DMA TX Stream, USART will start sending the command code (2bytes) */
	DMA_Cmd(COM_TX_DMA_STREAM[USART_COM1], ENABLE);

	return n;
}

