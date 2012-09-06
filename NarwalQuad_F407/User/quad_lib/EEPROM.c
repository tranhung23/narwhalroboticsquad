/*
 * EEPROM.c
 *
 *  Created on: Sep 3, 2012
 *      Author: GrubyGrub
 */

#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f4_quad.h"
#include "EEPROM.h"

static uint8_t EEPROM_buffer[2] = {(SERIAL_ADDR>> 8) & 0xFF,SERIAL_ADDR & 0xFF};
static uint8_t data_buffer[5] = {(SERIAL_NUMBER>>56) & 0xFF,(SERIAL_NUMBER>>48) & 0xFF,(SERIAL_NUMBER>>40) & 0xFF,(SERIAL_NUMBER>>32) & 0xFF,(SERIAL_NUMBER>>24) & 0xFF,(SERIAL_NUMBER>>16) & 0xFF,(SERIAL_NUMBER>>8) & 0xFF,SERIAL_NUMBER & 0xFF};
static uint8_t read_buffer[256];

int EEPROM_init(void)
{
	//I2C_ReadDeviceRegister(I2C_COM1, M24C64_ADDRESS, )
    I2C_WriteDeviceLongRegister(I2C_COM1, M24C64_ADDRESS, (uint32_t)EEPROM_buffer, 2, (uint32_t)data_buffer, 5);

    /*do nothing*/
    for (int i = 0; i < 16800000; i++)
        ;

    I2C_ReadDeviceLongRegister(I2C_COM1, M24C64_ADDRESS, (uint32_t)EEPROM_buffer, 2,(uint32_t)read_buffer, 5);
    //for (int i = 0; i < 16800; i++)
          ;
    sync_printf("PRINTING: 0x%x%x%x%x%x  ", read_buffer[0],read_buffer[1], read_buffer[2],read_buffer[3],read_buffer[4]);
    return 0;

    EEPROM_read(I2C_COM1,(uint32_t)EEPROM_buffer,(uint32_t)read_buffer, 8);
}

uint32_t EEPROM_read(int I2CPort, uint32_t RegAddress, uint32_t ReadDestAddress, int DataLength){
    I2C_ReadDeviceLongRegister(I2CPort, M24C64_ADDRESS, RegAddress, 2, ReadDestAddress, DataLength);
    return ReadDestAddress;
}

void EEPROM_write(int I2CPort, uint32_t RegAddress, uint32_t DataAddress, int DataLength){
    I2C_WriteDeviceLongRegister(I2CPort, M24C64_ADDRESS, RegAddress, 2, DataAddress, 5);
}
