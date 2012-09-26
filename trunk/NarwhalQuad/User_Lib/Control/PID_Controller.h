//TODO: Header


#ifndef PID_H_
#define PID_H_

#include <narwhal_top.h>


/*number of PID contorlers */
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
static float PID_PreError[PIDn];
/*Integral value*/
static float PID_Integral[PIDn];


extern void PIDinit(void);
extern float PIDcal(float setpoint, float actual_position, PID_ITEM_TypeDef PIDItem);

#endif /*PID_H_*/
