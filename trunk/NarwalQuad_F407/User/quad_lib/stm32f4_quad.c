/*
 * quad_indicator.c
 *
 *  Created on: Feb 12, 2012
 *      Author: GrubyGrub
 */
#include "stm32f4_quad.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_sdio.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_rcc.h"
#include "misc.h"

#include "stdarg.h"
#include "stdio.h"
#include "sensors.h"

unsigned char I2C1_TX_Buffer[256];
unsigned char I2C1_RX_Buffer[256];

unsigned char I2C2_TX_Buffer[256];
unsigned char I2C2_RX_Buffer[256];

static uint8_t I2C_STATE[I2Cn] = { IDLE, IDLE };
static I2C_TRANSMISSION_TypeDef I2C1_TransmissionObj = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
static I2C_TRANSMISSION_TypeDef I2C2_TransmissionObj = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

/**
 * I2C Comm
 */

I2C_TypeDef * I2C[I2Cn] = { I2C1, I2C2 };
const uint32_t I2C_CLK[I2Cn] = { I2C1_CLK, I2C2_CLK };

const uint32_t I2C_SPEED[I2Cn] = { I2C1_SPEED, I2C2_SPEED };

const uint16_t I2C_SCL_PIN[I2Cn] = { I2C1_SCL_PIN, I2C2_SCL_PIN }; /* PB.06 */
GPIO_TypeDef* I2C_SCL_GPIO_PORT[I2Cn] = { I2C1_SCL_GPIO_PORT, I2C2_SCL_GPIO_PORT }; /* GPIOB */
const uint32_t I2C_SCL_GPIO_CLK[I2Cn] = { I2C1_SCL_GPIO_CLK, I2C2_SCL_GPIO_CLK };
const uint16_t I2C_SCL_SOURCE[I2Cn] = { I2C1_SCL_SOURCE, I2C2_SCL_SOURCE };
const uint16_t I2C_SCL_AF[I2Cn] = { I2C1_SCL_AF, I2C2_SCL_AF };

const uint16_t I2C_SDA_PIN[I2Cn] = { I2C1_SDA_PIN, I2C2_SDA_PIN }; /* PB.09 */
GPIO_TypeDef* I2C_SDA_GPIO_PORT[I2Cn] = { I2C1_SDA_GPIO_PORT, I2C2_SDA_GPIO_PORT }; /* GPIOB */
const uint32_t I2C_SDA_GPIO_CLK[I2Cn] = { I2C1_SDA_GPIO_CLK, I2C2_SDA_GPIO_CLK };
const uint16_t I2C_SDA_SOURCE[I2Cn] = { I2C1_SDA_SOURCE, I2C2_SDA_SOURCE };
const uint16_t I2C_SDA_AF[I2Cn] = { I2C1_SDA_AF, I2C2_SDA_AF };

DMA_TypeDef * I2C_DMA[I2Cn] = { I2C1_DMA, I2C1_DMA };
const uint32_t I2C_DMA_CHANNEL[I2Cn] = { I2C1_DMA_CHANNEL, I2C2_DMA_CHANNEL };

DMA_Stream_TypeDef * I2C_DMA_STREAM_TX[I2Cn] = { I2C1_DMA_STREAM_TX, I2C2_DMA_STREAM_TX };
DMA_Stream_TypeDef * I2C_DMA_STREAM_RX[I2Cn] = { I2C1_DMA_STREAM_RX, I2C2_DMA_STREAM_RX };

const uint32_t I2C_DMA_CLK[I2Cn] = { I2C1_DMA_CLK, I2C2_DMA_CLK };
const uint32_t I2C_DR_Address[I2Cn] = { I2C1_DR_Address, I2C2_DR_Address };

const uint32_t I2C_DMA_TX_IRQn[I2Cn] = { I2C1_DMA_TX_IRQn, I2C2_DMA_TX_IRQn };
const uint32_t I2C_DMA_RX_IRQn[I2Cn] = { I2C1_DMA_RX_IRQn, I2C2_DMA_RX_IRQn };
const uint32_t I2C_DMA_PREPRIO[I2Cn] = { I2C1_DMA_PREPRIO, I2C2_DMA_PREPRIO };
const uint32_t I2C_DMA_SUBPRIO[I2Cn] = { I2C1_DMA_SUBPRIO, I2C2_DMA_SUBPRIO };

const uint32_t I2C_TX_DMA_FLAG_FEIF[I2Cn] = { I2C1_TX_DMA_FLAG_FEIF, I2C2_TX_DMA_FLAG_FEIF };
const uint32_t I2C_TX_DMA_FLAG_DMEIF[I2Cn] = { I2C1_TX_DMA_FLAG_DMEIF, I2C2_TX_DMA_FLAG_DMEIF };
const uint32_t I2C_TX_DMA_FLAG_TEIF[I2Cn] = { I2C1_TX_DMA_FLAG_TEIF, I2C2_TX_DMA_FLAG_TEIF };
const uint32_t I2C_TX_DMA_FLAG_HTIF[I2Cn] = { I2C1_TX_DMA_FLAG_HTIF, I2C2_TX_DMA_FLAG_HTIF };
const uint32_t I2C_TX_DMA_FLAG_TCIF[I2Cn] = { I2C1_TX_DMA_FLAG_TCIF, I2C2_TX_DMA_FLAG_TCIF };

const uint32_t I2C_RX_DMA_FLAG_FEIF[I2Cn] = { I2C1_RX_DMA_FLAG_FEIF, I2C2_RX_DMA_FLAG_FEIF };
const uint32_t I2C_RX_DMA_FLAG_DMEIF[I2Cn] = { I2C1_RX_DMA_FLAG_DMEIF, I2C2_RX_DMA_FLAG_DMEIF };
const uint32_t I2C_RX_DMA_FLAG_TEIF[I2Cn] = { I2C1_RX_DMA_FLAG_TEIF, I2C2_RX_DMA_FLAG_TEIF };
const uint32_t I2C_RX_DMA_FLAG_HTIF[I2Cn] = { I2C1_RX_DMA_FLAG_HTIF, I2C2_RX_DMA_FLAG_HTIF };
const uint32_t I2C_RX_DMA_FLAG_TCIF[I2Cn] = { I2C1_RX_DMA_FLAG_TCIF, I2C2_RX_DMA_FLAG_TCIF };

const uint32_t I2C_TX_BUFFER[I2Cn] = { (uint32_t) I2C1_TX_Buffer, (uint32_t) I2C2_TX_Buffer };

const uint32_t I2C_RX_BUFFER[I2Cn] = { (uint32_t) I2C1_RX_Buffer, (uint32_t) I2C2_RX_Buffer };

I2C_TRANSMISSION_TypeDef * I2C_TRANSMISSION[I2Cn] = { &I2C1_TransmissionObj, &I2C2_TransmissionObj };

const uint8_t I2C_PREPRIO[I2Cn] = { I2C1_PREPRIO, I2C2_PREPRIO };
const uint8_t I2C_SUBPRIO[I2Cn] = { I2C1_SUBPRIO, I2C2_SUBPRIO };

const uint8_t I2C_EVENT_IRQn[I2Cn] = { I2C1_Event_IRQn, I2C2_Event_IRQn };
const uint8_t I2C_ERROR_IRQn[I2Cn] = { I2C1_Error_IRQn, I2C2_Error_IRQn };

DMA_InitTypeDef DMA_InitStructure;
NVIC_InitTypeDef NVIC_InitStructure;
I2C_InitTypeDef I2C_InitStructure;

/*
 * Init's the I2C device
 */
void I2C_Device_Init(I2C_COM_TypeDef COM)
{
    /* IOE_I2C configuration */
    I2C_DeInit(I2C[COM]);

    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = I2C_SPEED[COM];

    /* Configure and enable I2C TX Channel interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = I2C_ERROR_IRQn[COM];
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = I2C_PREPRIO[COM];
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = I2C_SUBPRIO[COM];
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Configure and enable I2C TX Channel interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = I2C_EVENT_IRQn[COM];
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = I2C_PREPRIO[COM];
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = I2C_SUBPRIO[COM];
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    I2C_Cmd(I2C[COM], ENABLE);

    I2C_ITConfig(I2C[COM], I2C_IT_EVT, ENABLE);
    I2C_ITConfig(I2C[COM], I2C_IT_BUF, DISABLE);
    I2C_ITConfig(I2C[COM], I2C_IT_ERR, ENABLE);

    I2C_Init(I2C[COM], &I2C_InitStructure);
    //I2C_DMACmd(I2C[COM], ENABLE);

}

void I2C_LowLevel_Init(I2C_COM_TypeDef COM)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /*!< _I2C Periph clock enable */
    RCC_APB1PeriphClockCmd(I2C_CLK[COM], ENABLE);

    /*!< _I2C_SCL_GPIO_CLK and _I2C_SDA_GPIO_CLK Periph clock enable */
    RCC_AHB1PeriphClockCmd(I2C_SCL_GPIO_CLK[COM] | I2C_SDA_GPIO_CLK[COM], ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    /* Reset _I2C IP */
    RCC_APB1PeriphResetCmd(I2C_CLK[COM], ENABLE);

    /* Release reset signal of _I2C IP */
    RCC_APB1PeriphResetCmd(I2C_CLK[COM], DISABLE);

    /*!< GPIO configuration */
    /*!< Configure _I2C pins: SCL */
    GPIO_InitStructure.GPIO_Pin = I2C_SCL_PIN[COM];
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(I2C_SCL_GPIO_PORT[COM], &GPIO_InitStructure);

    /*!< Configure _I2C pins: SDA */
    GPIO_InitStructure.GPIO_Pin = I2C_SDA_PIN[COM];
    GPIO_Init(I2C_SDA_GPIO_PORT[COM], &GPIO_InitStructure);

    /* Connect PXx to I2C_SCL*/
    GPIO_PinAFConfig(I2C_SCL_GPIO_PORT[COM], I2C_SCL_SOURCE[COM], I2C_SCL_AF[COM]);

    /* Connect PXx to I2C_SDA*/
    GPIO_PinAFConfig(I2C_SDA_GPIO_PORT[COM], I2C_SDA_SOURCE[COM], I2C_SDA_AF[COM]);

    /* Configure and enable I2C DMA TX Channel interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = I2C_DMA_TX_IRQn[COM];
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = I2C_DMA_PREPRIO[COM];
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = I2C_DMA_SUBPRIO[COM];
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Configure and enable I2C DMA RX Channel interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = I2C_DMA_RX_IRQn[COM];
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = I2C_DMA_PREPRIO[COM];
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = I2C_DMA_SUBPRIO[COM];
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /*!< I2C DMA TX and RX channels configuration */
    /* Enable the DMA clock */
    RCC_AHB1PeriphClockCmd(I2C_DMA_CLK[COM], ENABLE);

    /*Just diable the steams to do ops on them*/
    /* Clear any pending flag on Tx Stream  */
    DMA_ClearFlag(I2C_DMA_STREAM_TX[COM], I2C_TX_DMA_FLAG_FEIF[COM] | I2C_TX_DMA_FLAG_DMEIF[COM] | I2C_TX_DMA_FLAG_TEIF[COM] | I2C_TX_DMA_FLAG_HTIF[COM] | I2C_TX_DMA_FLAG_TCIF[COM]);
    /* Disable the EE I2C Tx DMA stream */
    DMA_Cmd(I2C_DMA_STREAM_TX[COM], DISABLE);
    /* Configure the DMA stream for the EE I2C peripheral TX direction */
    DMA_DeInit(I2C_DMA_STREAM_TX[COM]);

    /*DMA Init of both RX and TX*/
    DMA_InitStructure.DMA_Channel = I2C_DMA_CHANNEL[COM];
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) I2C_DR_Address[COM];
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) I2C_TX_BUFFER[COM]; /* This parameter will be configured durig communication */
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral; /* This parameter will be configured durig communication */
    DMA_InitStructure.DMA_BufferSize = 0xFF; /* This parameter will be configured durig communication */
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

    DMA_Init(I2C_DMA_STREAM_TX[COM], &DMA_InitStructure);

    /* Clear any pending flag on Rx Stream */
    DMA_ClearFlag(I2C_DMA_STREAM_RX[COM], I2C_RX_DMA_FLAG_FEIF[COM] | I2C_RX_DMA_FLAG_DMEIF[COM] | I2C_RX_DMA_FLAG_TEIF[COM] | I2C_RX_DMA_FLAG_HTIF[COM] | I2C_RX_DMA_FLAG_TCIF[COM]);
    /* Disable the EE I2C DMA Rx stream */
    DMA_Cmd(I2C_DMA_STREAM_RX[COM], DISABLE);

    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) I2C_RX_BUFFER[COM]; /* This parameter will be configured durig communication */
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory; /* This parameter will be configured durig communication */
    DMA_InitStructure.DMA_BufferSize = 0xFF; /* This parameter will be configured durig communication */

    /* Configure the DMA stream for the EE I2C peripheral RX direction */
    DMA_DeInit(I2C_DMA_STREAM_RX[COM]);
    DMA_Init(I2C_DMA_STREAM_RX[COM], &DMA_InitStructure);

    /* Enable the DMA Channels Interrupts */
    DMA_ITConfig(I2C_DMA_STREAM_TX[COM], DMA_IT_TC, ENABLE);
    DMA_ITConfig(I2C_DMA_STREAM_RX[COM], DMA_IT_TC, ENABLE);

    I2C_Device_Init(COM);

}

/*
 *
 *
 *
 *
 *
 *
 *
 *
 *
 * I2C write functions
 *
 *
 *
 *
 *
 *
 *
 *
 */

/*Writes to the device, this is a blocking statement, will wait for write to finish before returning*/
uint8_t I2C_WriteDevice(I2C_COM_TypeDef COM, uint8_t DeviceAddr, uint8_t data, uint8_t Tx_dataLength)
{
    I2C_WriteDevice_async(COM, DeviceAddr, data, Tx_dataLength);
    while (I2C_GetFlagStatus(I2C[COM], I2C_FLAG_BUSY))
    {

    }

    return 1;
}

/*Writes to the device, this is a non-blocking statement, will wait for write to finish before returning*/
uint8_t I2C_WriteDevice_async(I2C_COM_TypeDef COM, uint8_t DeviceAddr, uint8_t data, uint8_t Tx_dataLength)
{
    return I2C_WriteDeviceRegister_async(COM, DeviceAddr, -1, data, Tx_dataLength);
}

uint8_t I2C_WriteDeviceRegister(I2C_COM_TypeDef COM, uint8_t DeviceAddr, uint8_t RegisterAddr, uint8_t RegisterValue, uint8_t Tx_dataLength)
{
    I2C_WriteDeviceRegister_async(COM, DeviceAddr, RegisterAddr, RegisterValue, Tx_dataLength);
    while (I2C_GetFlagStatus(I2C[COM], I2C_FLAG_BUSY))
    {

    }
    return 1;
}

/*Writes to the register async*/
uint8_t I2C_WriteDeviceRegister_async(I2C_COM_TypeDef COM, uint8_t DeviceAddr, uint8_t RegisterAddr, uint8_t RegisterValue, uint8_t Tx_dataLength)
{
    return I2C_WriteDeviceRegisterBuffer_async(COM, DeviceAddr, RegisterAddr, (uint32_t) RegisterValue, Tx_dataLength);
}

/*Write device register async with a buffer*/
uint8_t I2C_WriteDeviceRegisterBuffer_async(I2C_COM_TypeDef COM, uint8_t DeviceAddr, uint8_t RegisterAddr, uint32_t WriteBuffer, uint8_t TX_DataLength)
{
    return I2C_WriteDeviceLongRegister_async(COM, DeviceAddr, RegisterAddr, (RegisterAddr > -1) ? 1 : 0, TX_DataLength, WriteBuffer);
}
uint8_t I2C_WriteDeviceLongRegister(I2C_COM_TypeDef COM, uint8_t DeviceAddr, uint32_t RegisterAddrsPointer, uint16_t RegisterLength, uint16_t TX_DataLength, uint32_t WriteBuffer)
{
    I2C_WriteDeviceLongRegister_async(COM, DeviceAddr, RegisterAddrsPointer, RegisterLength, TX_DataLength, WriteBuffer);
    while (I2C_GetFlagStatus(I2C[COM], I2C_FLAG_BUSY))
    {

    }
    return 1;
}
// to write to a register that's more than one byte in length
uint8_t I2C_WriteDeviceLongRegister_async(I2C_COM_TypeDef COM, uint8_t DeviceAddr, uint32_t RegisterAddrsPointer, uint16_t RegisterLength, uint16_t TX_DataLength, uint32_t WriteBuffer)
{
    if (I2C_GetFlagStatus(I2C[COM], I2C_FLAG_BUSY))
    {
        return -1;
    }

    I2C_TRANSMISSION[COM]->DeviceAddr = DeviceAddr;

    /*Deal with multipal device addresses*/
    I2C_TRANSMISSION[COM]->RegisterAddrs = RegisterAddrsPointer;
    I2C_TRANSMISSION[COM]->RegisterAddrLength = RegisterLength;
    I2C_TRANSMISSION[COM]->RegisterAddrPtr = RegisterLength;

    //TODO: If we dereference the write buffer, it breaks raw writes, but if we are to write a buffer then we need to dereference....
    I2C_TRANSMISSION[COM]->RegisterValue = (uint8_t) (WriteBuffer);
    I2C_TRANSMISSION[COM]->DataDirection = I2C_DIRECTION_TX;
    I2C_TRANSMISSION[COM]->TX_DataLength = TX_DataLength;

    if (TX_DataLength > 1)
    {
        I2C_DMA_STREAM_TX[COM]->NDTR = TX_DataLength;
        I2C_DMA_STREAM_TX[COM]->M0AR = WriteBuffer;
        I2C_DMACmd(I2C[COM], ENABLE);
    }

    //I2C1_STATE = SEND_ADDR_TX;
    I2C_STATE[COM] = SEND_DEVICE_ADDR;
    I2C_GenerateSTART(I2C[COM], ENABLE);

    return 1;
}
/*
 *
 *
 *
 *
 *
 *
 * I2C Read Register functions
 *
 *
 *
 *
 *
 */

uint8_t I2C_ReadDeviceRegister(I2C_COM_TypeDef COM, uint8_t DeviceAddr, uint8_t RegisterAddr, uint16_t RX_DataLength, uint32_t ReadBuffer)
{
    I2C_ReadDeviceRegister_async(COM, DeviceAddr, RegisterAddr, RX_DataLength, ReadBuffer);

    while (I2C_GetFlagStatus(I2C[COM], I2C_FLAG_BUSY))
    {
    }
    return 0;
}
uint8_t I2C_ReadDeviceLongRegister(I2C_COM_TypeDef COM, uint8_t DeviceAddr, uint32_t RegisterAddrsPointer, uint16_t RegisterLength, uint16_t RX_DataLength, uint32_t ReadBuffer)
{
    I2C_ReadDeviceLongRegister_async(COM, DeviceAddr, RegisterAddrsPointer, RegisterLength, RX_DataLength, ReadBuffer);

    while (I2C_GetFlagStatus(I2C[COM], I2C_FLAG_BUSY))
    {
    }
    return 1;
}
// to read from a register that's more than one byte in length
uint8_t I2C_ReadDeviceLongRegister_async(I2C_COM_TypeDef COM, uint8_t DeviceAddr, uint32_t RegisterAddrsPointer, uint16_t RegisterLength, uint16_t RX_DataLength, uint32_t ReadBuffer)
{
    // if I2C port is busy, return
    if (I2C_GetFlagStatus(I2C[COM], I2C_FLAG_BUSY))
        return -1;

    I2C_TRANSMISSION[COM]->DeviceAddr = DeviceAddr;

    /*Deal with multipal device addresses*/
    I2C_TRANSMISSION[COM]->RegisterAddrs = RegisterAddrsPointer;
    I2C_TRANSMISSION[COM]->RegisterAddrLength = RegisterLength;
    I2C_TRANSMISSION[COM]->RegisterAddrPtr = RegisterLength;

    I2C_TRANSMISSION[COM]->RegisterValue = 0x0; /*This value is ignroed, but we are going to reset it here*/
    I2C_TRANSMISSION[COM]->DataDirection = I2C_DIRECTION_RX;
    I2C_TRANSMISSION[COM]->RX_DataLength = RX_DataLength;
    /*Set the DMA streams*/
    I2C_DMA_STREAM_RX[COM]->NDTR = RX_DataLength;
    I2C_DMA_STREAM_RX[COM]->M0AR = ReadBuffer;

    I2C_DMACmd(I2C[COM], ENABLE);

    I2C_STATE[COM] = SEND_DEVICE_ADDR;
    I2C_GenerateSTART(I2C[COM], ENABLE);
    return 0;
}

uint8_t I2C_ReadDeviceRegister_async(I2C_COM_TypeDef COM, uint8_t DeviceAddr, uint8_t RegisterAddr, uint16_t RX_DataLength, uint32_t ReadBuffer)
{
    return I2C_ReadDeviceLongRegister_async(COM, DeviceAddr, RegisterAddr, 1, RX_DataLength, ReadBuffer);
}

/*
 *
 *
 *
 *
 *
 *
 * I2C Other
 *
 *
 *
 *
 *
 */

//TODO: Implement fast DMA update
void LowLevel_DMA_Update(DMA_Stream_TypeDef * stream, uint32_t pBuffer, uint32_t BufferSize, uint32_t Direction)
{

}

/*
 * I2C Event Hanlders
 * I2C1 Area
 */
void I2C1_Event_IRQHandler(void)
{
    I2C_Event_Helper(I2C_COM1);
    return;
}

void I2C1_Error_IRQHandler(void)
{
    I2C_Error_Helper(I2C_COM1);
    return;
}

// TODO: generalize COMs
void I2C1_DMA_TX_IRQHandler(void)
{
    /* Check if the DMA transfer is complete */
    if (DMA_GetFlagStatus(I2C_DMA_STREAM_TX[I2C_COM1], I2C_TX_DMA_FLAG_TCIF[I2C_COM1]) != RESET)
    {
        I2C[I2C_COM1]->SR1;
        I2C[I2C_COM1]->SR2;
        /*!< Send STOP Condition */
        //I2C1_STATE = IDLE;
        I2C_STATE[I2C_COM1] = IDLE;
        I2C_GenerateSTOP(I2C[I2C_COM1], ENABLE);

        I2C_DMACmd(I2C[I2C_COM1], DISABLE);

        /* Disable the DMA Rx Stream and Clear TC Flag */
        DMA_Cmd(I2C_DMA_STREAM_TX[I2C_COM1], DISABLE);
        DMA_ClearFlag(I2C_DMA_STREAM_TX[I2C_COM1], I2C_TX_DMA_FLAG_TCIF[I2C_COM1]);

    }
}

void I2C1_DMA_RX_IRQHandler(void)
{
    /* Check if the DMA transfer is complete */
    if (DMA_GetFlagStatus(I2C_DMA_STREAM_RX[I2C_COM1], I2C_RX_DMA_FLAG_TCIF[I2C_COM1]) != RESET)
    {
        I2C[I2C_COM1]->SR1;
        I2C[I2C_COM1]->SR2;
        /*!< Send STOP Condition */
        //I2C1_STATE = IDLE;
        I2C_STATE[I2C_COM1] = IDLE;
        I2C_GenerateSTOP(I2C[I2C_COM1], ENABLE);

        I2C_DMACmd(I2C[I2C_COM1], DISABLE);

        /* Disable the DMA Rx Stream and Clear TC Flag */
        SensorIRQ_Complete();
        DMA_Cmd(I2C_DMA_STREAM_RX[I2C_COM1], DISABLE);
        DMA_ClearFlag(I2C_DMA_STREAM_RX[I2C_COM1], I2C_RX_DMA_FLAG_TCIF[I2C_COM1]);

    }
}

/**
 * I2C2 IRQ Hanlders
 */

void I2C2_Event_IRQHandler(void)
{
    I2C_Event_Helper(I2C_COM2);
    return;
}

void I2C2_Error_IRQHandler(void)
{
    I2C_Error_Helper(I2C_COM2);
    return;
}
void I2C2_DMA_TX_IRQHandler(void)
{

}

void I2C2_DMA_RX_IRQHandler(void)
{

}

/*Global error and event helpers*/

void I2C_Error_Helper(I2C_COM_TypeDef I2C_COM)
{
    if (I2C_GetITStatus(I2C[I2C_COM2], I2C_IT_TIMEOUT))
    {
        async_printf("Timeout");
    }
    else if (I2C_GetITStatus(I2C[I2C_COM2], I2C_IT_OVR))
    {
        async_printf("Overflow");
    }
    else if (I2C_GetITStatus(I2C[I2C_COM2], I2C_IT_AF))
    {
        async_printf("AF");
    }
    else if (I2C_GetITStatus(I2C[I2C_COM2], I2C_IT_ARLO))
    {
        async_printf("ARLO");
    }
    else if (I2C_GetITStatus(I2C[I2C_COM2], I2C_IT_BERR))
    {
        async_printf("BERR");
    }
}

void I2C_Event_Helper(I2C_COM_TypeDef I2C_COM)
{
    uint8_t I2C_State = I2C_STATE[I2C_COM];
    uint8_t newState = I2C_State;
    switch (I2C_State)
    {
    /*Start bit is set, send the device address*/
    case SEND_DEVICE_ADDR:
        if (I2C_GetITStatus(I2C[I2C_COM], I2C_IT_SB))
        {
            I2C_Send7bitAddress(I2C[I2C_COM], I2C_TRANSMISSION[I2C_COM]->DeviceAddr, I2C_Direction_Transmitter);

            //If there is Register address, then send register address first, else just send the data.
            if (I2C_TRANSMISSION[I2C_COM]->RegisterAddrLength == 1)
                newState = SEND_REG_ADDR;
            else if (I2C_TRANSMISSION[I2C_COM]->RegisterAddrLength > 1)
                newState = SEND_MULTI_REG_ADDR;
            else
                newState = SEND_DATA;
        }
        break;
    case SEND_MULTI_REG_ADDR:
        if (I2C_GetITStatus(I2C[I2C_COM], I2C_IT_ADDR | I2C_IT_TXE))
        {

            I2C[I2C_COM1]->SR2;
            I2C_TRANSMISSION[I2C_COM]->RegisterAddrPtr--;
            I2C_SendData(I2C[I2C_COM], ((uint8_t *) I2C_TRANSMISSION[I2C_COM]->RegisterAddrs)[I2C_TRANSMISSION[I2C_COM]->RegisterAddrPtr]);
            //sync_printf("Sending: %x", ((uint8_t *) I2C_TRANSMISSION[I2C_COM]->RegisterAddrs)[I2C_TRANSMISSION[I2C_COM]->RegisterAddrPtr]);
            if ((I2C_TRANSMISSION[I2C_COM]->RegisterAddrPtr) > 0)
            {
                newState = SEND_MULTI_REG_ADDR;
            }
            else
            {
                //Done with sending multiple regs, now send the actaul data.
                newState = SEND_REG_DATA;
                //We are going to recieve data
                if (I2C_TRANSMISSION[I2C_COM]->DataDirection == I2C_DIRECTION_RX)
                    newState = START_RX;
            }
        }
        break;
        /* Device at address exists, send the register address, if the direction is RX, then start RX, else, go to sending register data*/
    case SEND_REG_ADDR:
        if (I2C_GetITStatus(I2C[I2C_COM], I2C_IT_ADDR))
        {
            I2C[I2C_COM]->SR2;
            I2C_SendData(I2C[I2C_COM], I2C_TRANSMISSION[I2C_COM]->RegisterAddrs);
            newState = SEND_REG_DATA;
            if (I2C_TRANSMISSION[I2C_COM]->DataDirection == I2C_DIRECTION_RX)
                newState = START_RX;
        }
        break;
        // only send one byte of data to the device and stop
    case SEND_DATA:
        if (I2C_GetITStatus(I2C[I2C_COM], I2C_IT_ADDR))
        {
            I2C[I2C_COM]->SR2;
            I2C_SendData(I2C[I2C_COM], I2C_TRANSMISSION[I2C_COM]->RegisterValue);
            newState = STOP;
        }
        break;
    case SEND_REG_DATA: //TODO: handle multipal transmission by use of DMA.

        if (I2C_GetITStatus(I2C[I2C_COM], I2C_IT_TXE) && I2C_TRANSMISSION[I2C_COM]->TX_DataLength)
        {
            //If transferring only 1 byte, dont use DMA.
            if (I2C_TRANSMISSION[I2C_COM]->TX_DataLength == 1)
            {
                I2C_SendData(I2C[I2C_COM], I2C_TRANSMISSION[I2C_COM]->RegisterValue);
                //I2C_TRANSMISSION[I2C_COM]->TX_DataLength--;
                newState = STOP;
            }
            else
            {
                I2C_DMALastTransferCmd(I2C[I2C_COM], ENABLE);

                /* Enable the DMA Rx Stream */
                DMA_Cmd(I2C_DMA_STREAM_TX[I2C_COM], ENABLE);
                newState = IDLE;
            }
        }
        break;
    case STOP:
        if (I2C_GetITStatus(I2C[I2C_COM], I2C_IT_TXE | I2C_FLAG_BTF))
        {
            I2C_GenerateSTOP(I2C[I2C_COM], ENABLE);
            newState = IDLE;
        }
        break;

        /*RX Section*/
    case START_RX:
        if (I2C_GetITStatus(I2C[I2C_COM], I2C_IT_TXE | I2C_FLAG_BTF))
        {
            I2C[I2C_COM]->SR2;
            I2C_GenerateSTART(I2C[I2C_COM], ENABLE);
            newState = SEND_ADDR_RX;
        }
        break;
    case SEND_ADDR_RX:
        if (I2C_GetITStatus(I2C[I2C_COM], I2C_IT_SB))
        {
            I2C_Send7bitAddress(I2C[I2C_COM], I2C_TRANSMISSION[I2C_COM]->DeviceAddr, I2C_Direction_Receiver);
            newState = START_DMA;
        }
        break;
    case START_DMA:
        if (I2C_GetITStatus(I2C[I2C_COM], I2C_IT_ADDR | I2C_FLAG_BTF))
        {
            I2C[I2C_COM]->SR2;

            //I2C_ITConfig(I2C[I2C_COM1],I2C_IT_EVT,DISABLE);
            /* Configure the DMA Rx Channel with the buffer address and the buffer size */
            //I2C_DMACmd(I2C[COM], ENABLE);
            /* Inform the DMA that the next End Of Transfer Signal will be the last one */
            I2C_DMALastTransferCmd(I2C[I2C_COM], ENABLE);

            /* Enable the DMA Rx Stream */
            DMA_Cmd(I2C_DMA_STREAM_RX[I2C_COM], ENABLE);
            newState = IDLE;
        }
        break;
    case IDLE:
        break;
    }
    I2C_STATE[I2C_COM] = newState;
    return;
}
