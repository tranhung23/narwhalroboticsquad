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

static uint8_t EEPROM_buffer[2] = {01,02};
static uint8_t data_buffer[1] = {0xEA};
static uint8_t read_buffer[256];

int EEPROM_init(void)
{
	//I2C_ReadDeviceRegister(I2C_COM1, M24C64_ADDRESS, )
    I2C_WriteDeviceLongRegister_async(I2C_COM1, M24C64_ADDRESS, (uint32_t)EEPROM_buffer, 2, 1, (uint32_t)data_buffer);

    /*do nothing*/
    for (int i = 0; i < 16800000; i++)
        ;

    I2C_ReadDeviceLongRegister_async(I2C_COM1, M24C64_ADDRESS, (uint32_t)EEPROM_buffer, 2, 1, (uint32_t)read_buffer);
    for (int i = 0; i < 16800; i++)
          ;
    sync_printf("PRINTING: 0x%x ", read_buffer[0]);
    return 0;
}
