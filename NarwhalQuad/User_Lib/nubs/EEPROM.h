/*
 * EEPROM.h
 *
 *  Created on: Sep 3, 2012
 *      Author: GrubyGrub
 */

#ifndef EEPROM_H_
#define EEPROM_H_

#include <narwhal_top.h>

#define M24C64_ADDRESS (0xAE)//0b1010111x address of EEPROM
#define SERIAL_ADDR 0x0001
#define SERIAL_NUMBER 0x4913E523817AFFFF

int EEPROM_init(void);

#endif /* EEPROM_H_ */
