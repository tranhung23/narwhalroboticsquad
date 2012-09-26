#include "PID_Controller.h"
#include <stdio.h>
#include <stdlib.h>
/*
 * PID_Controller.c
 *
 *  Created on: May 3, 2012
 *      Author: GrubyGrub
 */

//TODO: Refactor code, make sure array values are in sync with paramaeter array
void PIDinit(void)
{
    /*PID array values, epsilon is the min value to do integral*/
    PID_Epsilon[0]= p[PID_YAW_EPSILON];
    PID_Epsilon[1]= p[PID_PITCH_EPSILON];
    PID_Epsilon[2]= p[PID_ROLL_EPSILON];

    /*Contains the last time that the dirivative changed*/
    PID_DT[0]= p[PID_YAW_DT];
    PID_DT[1]= p[PID_PITCH_DT];
    PID_DT[2]= p[PID_ROLL_DT];

    /*the max that the pitch and yaw roll should be set at*/
    PID_MAX[0]= p[PID_YAW_MAX];
    PID_MAX[1]= p[PID_PITCH_MAX];
    PID_MAX[2]= p[PID_ROLL_MAX];

    PID_MIN[0]= p[PID_YAW_MIN];
    PID_MIN[1]= p[PID_PITCH_MIN];
    PID_MIN[2]= p[PID_ROLL_MIN];

    PID_Kp[0]= p[PID_YAW_KP];
    PID_Kp[1]= p[PID_PITCH_KP];
    PID_Kp[2]= p[PID_ROLL_KP];

    PID_Ki[0]= p[PID_YAW_KI];
    PID_Ki[1]= p[PID_PITCH_KI];
    PID_Ki[2]= p[PID_ROLL_KI];

    PID_Kd[0]= p[PID_YAW_KD];
    PID_Kd[1]= p[PID_PITCH_KD];
    PID_Kd[2]= p[PID_ROLL_KD];
}

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
