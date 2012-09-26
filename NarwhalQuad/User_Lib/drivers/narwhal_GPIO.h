/*
 * narwhal_GPIO.h
 *
 *  Created on: May 16, 2012
 *      Author: GrubyGrub
 */

#ifndef narwhal_GPIO_H_
#define narwhal_GPIO_H_

#include <narwhal_top.h>

// TODO: make a device tree that auto parse the define file, instead of manually defining the array
typedef enum
{
	LED4 = 0, LED3 = 1, LED5 = 2, LED6 = 3, GXY_AZ = 4, GZ_AZ = 5, EEPROM = 6
} GPIO_PIN_TypeDef;


extern void GPIOInit(GPIO_PIN_TypeDef Led);
extern void GPIOOn(GPIO_PIN_TypeDef Led);
extern void GPIOOff(GPIO_PIN_TypeDef Led);
extern void GPIOToggle(GPIO_PIN_TypeDef Led);


#endif
