/*
 * Motor_Control.h
 *
 *  Created on: May 3, 2012
 *      Author: GrubyGrub
 */

#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_

#define PITCH_LEFT_ADDR 0x52
#define PITCH_RIGHT_ADDR 0x54
#define ROLL_BACK_ADDR 0x56
#define ROLL_FORWARD_ADDR 0x58

#define CTRL_MOTORn 4
#define CTRL_LINEn 4


#define MOTOR_CTRL_MIN 0
#define MOTOR_CTRL_MAX 255

/*Pitch angle, Roll angle, Yaw Angle, throttle*/
static int Quad_Ctrl_Angle[CTRL_MOTORn] = {0, 0, 0, 0};

/*Cap pitch and roll at 45 degrees for now, throttle at 200, pitch, roll, yaw, throttle*/
static int Quad_Ctrl_Limit_H[CTRL_LINEn] = {45, 45, 180, 200};
static int Quad_Ctrl_Limit_L[CTRL_LINEn] = {-45, -45, -180, 0};

typedef enum
{
	PITCH,
	ROLL,
	YAW,
	THROTTLE
}CTRL_LINES_TypeDef;


void MotorControl_Init(void);

void MotorControl_SetAngle(int rawValue, PWM_CTRL_IN_TypeDef CH);
void MotorControl_CalculateIntegralControl_RC(void);
int MotorControl_GetControlAngle(PWM_CTRL_IN_TypeDef CH);

#endif /* MOTOR_CONTROL_H_ */
