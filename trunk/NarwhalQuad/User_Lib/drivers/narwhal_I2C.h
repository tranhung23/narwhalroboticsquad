/*
 * quad_indicator.h
 *
 *  Created on: Feb 12, 2012
 *      Author: GrubyGrub
 */

#ifndef QUAD_INDICATOR_H_
#define QUAD_INDICATOR_H_

#include <narwhal_top.h>

extern OS_FlagID I2C1_FLAG;
extern OS_FlagID I2C2_FLAG;


typedef enum
{
	I2C_COM1 = 0, I2C_COM2 = 1
} I2C_COM_TypeDef;

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

/*I2C Items*/

extern void I2C_Device_Init(I2C_COM_TypeDef COM);
extern void I2C_LowLevel_Init(I2C_COM_TypeDef COM);

extern uint8_t I2C_ReadDeviceRegister(I2C_COM_TypeDef COM, uint8_t DeviceAddr, uint8_t RegisterAddr, uint16_t RX_DataLength, uint32_t ReadBuffer);
extern uint8_t I2C_ReadDeviceRegister_async(I2C_COM_TypeDef COM, uint8_t DeviceAddr, uint8_t RegisterAddr, uint16_t RX_DataLength, uint32_t ReadBuffer);
extern uint8_t I2C_ReadDeviceLongRegister_async(I2C_COM_TypeDef COM, uint8_t DeviceAddr, uint32_t RegisterAddrsPointer, uint16_t RegisterLength, uint32_t ReadBuffer, uint16_t RX_DataLength );
extern uint8_t I2C_ReadDeviceLongRegister(I2C_COM_TypeDef COM, uint8_t DeviceAddr, uint32_t RegisterAddrsPointer, uint16_t RegisterLength, uint32_t ReadBuffer, uint16_t RX_DataLength);
extern uint8_t I2C_WriteDeviceRegister(I2C_COM_TypeDef COM, uint8_t DeviceAddr, uint8_t RegisterAddr, uint8_t RegisterValue, uint8_t Tx_dataLength);
extern uint8_t I2C_WriteDeviceRegister_async(I2C_COM_TypeDef COM, uint8_t DeviceAddr, uint8_t RegisterAddr, uint8_t RegisterValue, uint8_t Tx_dataLength);
extern uint8_t I2C_WriteDeviceRegisterBuffer_async(I2C_COM_TypeDef COM, uint8_t DeviceAddr, uint8_t RegisterAddr, uint32_t WriteBuffer, uint8_t TX_DataLength);
extern uint8_t I2C_WriteDeviceLongRegister(I2C_COM_TypeDef COM, uint8_t DeviceAddr, uint32_t RegisterAddrsPointer, uint16_t RegisterLength, uint32_t WriteBuffer, uint16_t TX_DataLength);
extern uint8_t I2C_WriteDeviceLongRegister_async(I2C_COM_TypeDef COM, uint8_t DeviceAddr, uint32_t RegisterAddrsPointer, uint16_t RegisterLength, uint32_t WriteBuffer, uint16_t TX_DataLength );
extern uint8_t I2C_WriteDevice(I2C_COM_TypeDef COM, uint8_t DeviceAddr, uint8_t data, uint8_t Tx_dataLength);
extern uint8_t I2C_WriteDevice_async(I2C_COM_TypeDef COM, uint8_t DeviceAddr, uint8_t data, uint8_t Tx_dataLength);

void I2C_Event_Helper(I2C_COM_TypeDef I2C_COM);
void I2C_Error_Helper(I2C_COM_TypeDef I2C_COM);

void I2C1_Error_IRQHandler(void);
void I2C1_Event_IRQHandler(void);

void I2C2_Error_IRQHandler(void);
void I2C2_Event_IRQHandler(void);

#endif /* QUAD_INDICATOR_H_ */
