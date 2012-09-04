/*
 * Motor_Control.c
 *
 *  Created on: May 3, 2012
 *      Author: GrubyGrub
 */
#include "../quad_lib/stm32f4_quad.h"
#include "Motor_Control.h"
#include "RC_Control.h"
#include "PID_Controller.h"


/*Checks for out of bounds errors in Direct Control Inputs*/
static int Direct_Control_RangeCalibrate(int Direct_Input, int ControlLine)
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


void MotorControl_Init(void)
{
	//TODO: Currently all motors are default PID values, need to read PID values from the register or save them
	//TODO: Yaw angle needs to be set

}

void MotorControl_ChangePID(PID_ITEM_TypeDef PIDItem, float Kp, float Kd, float Ki)
{
	PID_Kp[PIDItem] = Kp;
	PID_Kd[PIDItem] = Kd;
	PID_Ki[PIDItem] = Ki;
	/*Save PID in eeprom*/
}

/*Calculate the motor control (angles) values from the RC inputs, this item is called at timer hz, we have 4 basic controls: PITCH/YAW/ROLL/THROTTLE*/
void MotorControl_SetAngle(int rawValue, PWM_CTRL_IN_TypeDef CH)
{
	//RC_Control[CH] = rawValue;

	//(x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
	if(rawValue > RC_Control_Max)
		rawValue = RC_Control_Max;
	if(rawValue < RC_Control_Min)
		rawValue = RC_Control_Min;

	Quad_Ctrl_Angle[CH] = (rawValue - RC_Control_Min) * (Quad_Ctrl_Limit_H[CH]- Quad_Ctrl_Limit_L[CH]) /(RC_Control_Max - RC_Control_Min) - Quad_Ctrl_Limit_L[CH];

	//async_printf("Value: %d\r\n", Quad_Ctrl_Angle[CH]);
}


/*TODO: Integral Control*/
void MotorControl_CalculateIntegralControl_RC(void)
{
	//TODO: Integral control, current contorl + rawcontrol;
}

/*Directly insert desired angles into the array*/
void MotorControl_CalculateDirect(int pitch, int roll, int yaw, int throttle)
{
	Quad_Ctrl_Angle[PITCH] = Direct_Control_RangeCalibrate(pitch, PITCH);
	Quad_Ctrl_Angle[ROLL] = Direct_Control_RangeCalibrate(roll, ROLL);
	Quad_Ctrl_Angle[YAW] = Direct_Control_RangeCalibrate(yaw, YAW);
	Quad_Ctrl_Angle[THROTTLE] = Direct_Control_RangeCalibrate(throttle, THROTTLE);
}

int MotorControl_GetControlAngle(PWM_CTRL_IN_TypeDef CH)
{
	return Quad_Ctrl_Angle[CH];
}

//Sends the motors actual control values;
void MotorControl_SendMotorCmd()
{
	//TODO: Implement method
	I2C_WriteDevice_async(I2C_COM2, 0x52, 50, 1);
}
