/*
 * Motor_Control.h
 *
 *  Created on: May 3, 2012
 *      Author: GrubyGrub
 */

#ifndef RC_CONTROL_H_
#define RC_CONTROL_H_

#include <narwhal_top.h>
#include <narwhal_TIMER.h>
#include <narwhal_I2C.h>


typedef enum
{
	YAW = 0,
	PITCH = 1,
	ROLL = 2,
	THROTTLE = 3
}CTRL_LINES_TypeDef;


#define RC_PWM_INPUT 4
extern __IO unsigned int CTRL_RAW[RC_PWM_INPUT];



extern void RC_Init(void);
extern int RC_SetAngle(int rawValue, PWM_CTRL_IN_TypeDef CH);


#endif /* MOTOR_CONTROL_H_ */
