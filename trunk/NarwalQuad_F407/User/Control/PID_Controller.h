#include "../quad_lib/config.h"
#ifndef PID_H_
#define PID_H_

#define PIDn	3

typedef enum
{
    PID_PITCH, PID_ROLL, PID_YAW, PID_ALTITUDE
} PID_ITEM_TypeDef;

static float PID_Epsilon[PIDn];
static float PID_DT[PIDn];

/*the max that the pitch and yaw roll should be set at*/
static float PID_MAX[PIDn];
static float PID_MIN[PIDn];

static float PID_Kp[PIDn];
static float PID_Kd[PIDn];
static float PID_Ki[PIDn];
/*Previous error*/
static float PID_PreError[PIDn] = { 0.0, 0.0, 0.0 };

/*Integral value*/
static float PID_Integral[PIDn] = { 0.0, 0.0, 0.0 };

#endif /*PID_H_*/
