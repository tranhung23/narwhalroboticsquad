#include "PID_Controller.h"
#include <stdio.h>
#include <stdlib.h>
/*
 * PID_Controller.c
 *
 *  Created on: May 3, 2012
 *      Author: GrubyGrub
 */

float PIDcal(float setpoint, float actual_position, PID_ITEM_TypeDef PIDItem)
{

	float error;
	float derivative;
	float output;

	//Caculate P,I,D
	error = setpoint - actual_position;

	//In case of error too small then stop intergration
	if (abs(error) > PID_Epsilon[PIDItem])
	{
		PID_Integral[PIDItem] = (float)PID_Integral[PIDItem] + (error * (float)PID_DT[PIDItem]);
	}
	derivative = (error - PID_PreError[PIDItem]) / PID_DT[PIDItem];
	output = PID_Kp[PIDItem] * error + PID_Ki[PIDItem] * PID_Integral[PIDItem] + PID_Kd[PIDItem] * derivative;

	//Saturation Filter
	if (output > PID_MAX[PIDItem])
	{
		output = PID_MAX[PIDItem];
	} else if (output < PID_MIN[PIDItem])
	{
		output = PID_MIN[PIDItem];
	}
	//Update error
	PID_PreError[PIDItem] = error;

	return output;
}
