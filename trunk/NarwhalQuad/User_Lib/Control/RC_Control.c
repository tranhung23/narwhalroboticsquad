/*
 * Motor_Control.c
 *
 *  Created on: May 3, 2012
 *      Author: GrubyGrub
 */
#include "Motor_Control.h"
#include "PID_Controller.h"

unsigned int CTRL_RAW[PWM_INPUT] = {0,0,0,0};

static int RC_Line_Max;//Max of the RC pwm input
static int RC_Line_Min;//Min of the RC pwm input

/*Checks for out of bounds errors in Direct Control Inputs*/
static inline int Direct_Control_RangeCalibrate(int Direct_Input, int ControlLine)
{
	if (Direct_Input > Quad_Ctrl_Limit_L[ControlLine])
	{
		return Quad_Ctrl_Limit_L[ControlLine];
	}
	else if (Direct_Input < Quad_Ctrl_Limit_H[ControlLine])
	{
		return Quad_Ctrl_Limit_H[ControlLine];
	}
	return Direct_Input;
}

/*Init default values*/
void RC_Init(void)
{
	RC_Line_Max = (long)p[RC_CONTROL_MAX];
	RC_Line_Min = (long)p[RC_CONTROL_MIN];
	//TODO: Currently all motors are default PID values, need to read PID values from the register or save them
	//TODO: Yaw angle needs to be set
}

/*Calculate the motor control (angles) values from the RC inputs, we have 4 basic controls: PITCH/YAW/ROLL/THROTTLE*/
int RC_SetAngle(int rawValue, PWM_CTRL_IN_TypeDef CH)
{
	//RC_Control[CH] = rawValue;

	//(x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
	if(rawValue > RC_Line_Max)
		rawValue = RC_Line_Max;
	if(rawValue < RC_Line_Min)
		rawValue = RC_Line_Min;

	return (rawValue - RC_Line_Min) * (Quad_Ctrl_Limit_H[CH]- Quad_Ctrl_Limit_L[CH]) /(RC_Line_Max - RC_Line_Min) + Quad_Ctrl_Limit_L[CH];
}

