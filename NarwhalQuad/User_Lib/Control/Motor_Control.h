/*
 * Motor_Control.h
 *
 *  Created on: May 3, 2012
 *      Author: GrubyGrub
 */

#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_

#include <narwhal_top.h>
#include <narwhal_TIMER.h>
#include <narwhal_I2C.h>

/*number of motor conttrollers*/
#define CTRL_MOTORn 4

/*number of control lines*/
#define CTRL_LINEn 4

#define PWM_INPUT 4

/*Pitch angle, Roll angle, Yaw Angle, throttle, keep*/
extern int Quad_Ctrl_Angle[CTRL_MOTORn];

/*Cap pitch and roll at 45 degrees for now, throttle at 200, pitch, roll, yaw, throttle*/
extern int Quad_Ctrl_Limit_H[CTRL_LINEn];
extern int Quad_Ctrl_Limit_L[CTRL_LINEn];


extern void MotorControl_Init(void);
extern void MotorControl_SetAngle(int rawValue, PWM_CTRL_IN_TypeDef CH);
extern void MotorControl_CalculateIntegralControl_RC(void);
extern int MotorControl_GetControlAngle(PWM_CTRL_IN_TypeDef CH);

#endif /* MOTOR_CONTROL_H_ */
