/*
 * narwhal_btn.h
 *
 *  Created on: Sep 3, 2012
 *      Author: GrubyGrub
 */

#ifndef narwhal_btn_H_
#define narwhal_btn_H_

#include <narwhal_top.h>

typedef enum
{
	BUTTON_USER = 0,
} Button_TypeDef;

typedef enum
{
	BUTTON_MODE_GPIO = 0, BUTTON_MODE_EXTI = 1
} ButtonMode_TypeDef;


extern void EXTILine0_Config(void);

#endif /* EEPROM_H_ */
