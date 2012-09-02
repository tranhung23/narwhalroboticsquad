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

/*Controls the RC range, makes sure nothing is out of bounds, output range is 0-RC_Control_MAX*/
static int RC_Control_RangeCalibrate(int RC_Input)
{
	if (RC_Input > RC_Control_Max)
		return RC_Range;
	if (RC_Input < RC_Control_Min)
		return 0;

	return RC_Input - RC_Control_Min;
}
/*Reads from the RC_Control array for the RC input and converts it into an angle based on the angle limitations*/
static int RC_Control_RangeToAngle(int rawValue, CTRL_LINES_TypeDef ControlLine)
{
	return (int) ((float) (RC_Control_RangeCalibrate(rawValue) - RC_Range_Half) / (float) RC_Range * (float) Quad_Ctrl_Angle_Limit[ControlLine]);
}

/*Checks for out of bounds errors in Direct Control Inputs*/
static int Direct_Control_RangeCalibrate(int Direct_Input, int ControlLine)
{
	if (Direct_Input > Quad_Ctrl_Angle_Limit[ControlLine])
	{
		return Quad_Ctrl_Angle_Limit[ControlLine];
	}
	else if (Direct_Input < -Quad_Ctrl_Angle_Limit[ControlLine])
	{
		return -Quad_Ctrl_Angle_Limit[ControlLine];
	}
	return Direct_Input;
}

void MotorControl_Init(void)
{
	//TODO: Currently all motors are default PID values, need to read PID values from the register or save them
	//TODO: Yaw angle needs to be set

	RC_Range = RC_Control_Max - RC_Control_Min;
	RC_Range_Half = RC_Range / 2;
}

void MotorControl_ChangePID(PID_ITEM_TypeDef PIDItem, float Kp, float Kd, float Ki)
{
	PID_Kp[PIDItem] = Kp;
	PID_Kd[PIDItem] = Kd;
	PID_Ki[PIDItem] = Ki;
	/*Save PID in eeprom*/
}

/*Calculate the motor control (angles) values from the RC inputs, this item is called at timer hz*/
void MotorControl_SetAngle(int rawValue, PWM_CTRL_IN_TypeDef CH)
{
	//RC_Control[CH] = rawValue;

	//(x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

	Quad_Ctrl_Angle[CH] = (rawValue - RC_Control_Min) * (Quad_Ctrl_Angle_Limit[CH]*2) /(RC_Range) - Quad_Ctrl_Angle_Limit[CH];

	async_printf("Value: %d\r\n", Quad_Ctrl_Angle[CH]);
}


/*TODO: Integral Control*/
void MotorControl_CalculateIntegralControl_RC();

/*Directly insert desired angles into the array*/
void MotorControl_CalculateDirect(int pitch, int roll, int yaw, int throttle)
{
	Quad_Ctrl_Angle[PITCH] = Direct_Control_RangeCalibrate(pitch, PITCH);
	Quad_Ctrl_Angle[ROLL] = Direct_Control_RangeCalibrate(roll, ROLL);
	Quad_Ctrl_Angle[YAW] = Direct_Control_RangeCalibrate(yaw, YAW);
	Quad_Ctrl_Angle[THROTTLE] = Direct_Control_RangeCalibrate(throttle, THROTTLE);
}

void MotorControl_SendMotorCmd()
{
	I2C_WriteDevice_async(I2C_COM2, 0x52, 50, 1);
}
