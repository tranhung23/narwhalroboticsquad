/*
 * EEPROM.h
 *
 *  Created on: Sep 3, 2012
 *      Author: GrubyGrub
 */

#ifndef EEPROM_H_
#define EEPROM_H_

#define M24C64_ADDRESS (0xAE) //0b1010111x address of EEPROM
int EEPROM_init(void);
#endif /* EEPROM_H_ */
