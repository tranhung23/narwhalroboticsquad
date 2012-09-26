/*
 * Motor_Control.c
 *
 *  Created on: May 3, 2012
 *      Author: GrubyGrub
 */
#include "Motor_Control.h"
#include "PID_Controller.h"


/*Pitch angle, Roll angle, Yaw Angle, throttle, keep*/
int Quad_Ctrl_Angle[CTRL_MOTORn] = {0, 0, 0, 0};

/*Cap pitch and roll at 45 degrees for now, throttle at 200, pitch, roll, yaw, throttle*/
int Quad_Ctrl_Limit_H[CTRL_LINEn] = {45, 45, 180, 200};
int Quad_Ctrl_Limit_L[CTRL_LINEn] = {-45, -45, -180, 0};

static int RC_Line_Max;//Max of the RC pwm input
static int RC_Line_Min;//Min of the RC pwm input


/*Init default values*/
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

/*TODO: Integral Control*/
void MotorControl_CalculateIntegralControl_RC(void)
{
	//TODO: Integral control, current contorl + rawcontrol;
}


//Sends the motors actual control values;
void MotorControl_SendMotorCmd()
{
	//TODO: Implement method
	I2C_WriteDevice_async(I2C_COM2, 0x52, 50, 1);
}
