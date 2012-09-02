/*
 * RC_Control.h
 *
 *  Created on: May 3, 2012
 *      Author: GrubyGrub
 */

#ifndef RC_CONTROL_H_
#define RC_CONTROL_H_

#define RC_CTRLn 4


//PITCH, ROLL, YAW, THROTTLE
volatile static int RC_Control[RC_CTRLn];

/*Define the mins and max of the control stick*/
static int RC_Control_Min = 1000;
static int RC_Control_Max = 2000;






#endif /* RC_CONTROL_H_ */
