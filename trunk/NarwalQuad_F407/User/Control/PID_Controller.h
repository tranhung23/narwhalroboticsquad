#ifndef PID_H_
#define PID_H_

#define PIDn	3

/*Yaw default PID parameters*/
#define YAW_epsilon 0.01
#define YAW_dt 0.01             //100ms loop time

#define YAW_MAX  4                   //For Current Saturation
#define YAW_MIN -4

#define YAW_Kp  0.1
#define YAW_Kd  0.01
#define YAW_Ki  0.005


/*Roll default PID parameters*/
#define ROLL_epsilon 0.01
#define ROLL_dt 0.01             //100ms loop time

#define ROLL_MAX  4.0                   //For Current Saturation
#define ROLL_MIN -4.0

#define ROLL_Kp  0.1
#define ROLL_Kd  0.01
#define ROLL_Ki  0.005

/*Pitch default PID parameters */
#define PITCH_epsilon 0.01
#define PITCH_dt 0.01             //100ms loop time

#define PITCH_MAX  4                   //For Current Saturation
#define PITCH_MIN -4

#define PITCH_Kp  0.1
#define PITCH_Kd  0.01
#define PITCH_Ki  0.005

typedef enum
{
	PID_PITCH,
	PID_ROLL,
	PID_YAW,
	PID_ALTITUDE
}PID_ITEM_TypeDef;


/*PID array values, epsilon is the min value to do integral*/
static float PID_Epsilon[PIDn] = {PITCH_epsilon, ROLL_epsilon, YAW_epsilon};

/*Contains the last time that the dirivative changed*/
static float PID_DT[PIDn] = {PITCH_dt, ROLL_dt, YAW_dt};

/*the max that the pitch and yaw roll should be set at*/
static float PID_MAX[PIDn] = {PITCH_MAX, ROLL_MAX, YAW_MAX};
static float PID_MIN[PIDn] = {PITCH_MIN, ROLL_MIN, YAW_MIN};

static float PID_Kp[PIDn] = {PITCH_Kp, ROLL_Kp, YAW_Kp};
static float PID_Kd[PIDn] = {PITCH_Kd, ROLL_Kd, YAW_Kd};
static float PID_Ki[PIDn] = {PITCH_Ki, ROLL_Ki, YAW_Ki};

/*Previous error*/
static float PID_PreError[PIDn]  = {0.0,0.0,0.0};

/*Integral value*/
static float PID_Integral[PIDn]  = {0.0,0.0,0.0};


#endif /*PID_H_*/
