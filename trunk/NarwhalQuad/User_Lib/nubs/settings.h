/*
 * config.h
 *
 *  Created on: Sep 9, 2012
 *      Author: GrubyGrub
 */
#include "stm32f4xx.h"

#ifndef CONFIG_H_
#define CONFIG_H_
#define CONFIG_HEADER       "config_default.h"

enum configParameters{
    CONFIG_VERSION = 0,
    DEG2RAD,
    RAD2DEG,
    IMU_P_COV,
    IMU_Q_COV,
    IMU_ACC_COV,
    IMU_ACC_BIAS_X,
    IMU_ACC_BIAS_Y,
    IMU_ACC_BIAS_Z,
    IMU_ACC_REF_X,
    IMU_ACC_REF_Y,
    IMU_ACC_REF_Z,
    IMU_MAG_COV,
    IMU_MAG_BIAS_X,
    IMU_MAG_BIAS_Y,
    IMU_MAG_BIAS_Z,
    IMU_MAG_REF_X,
    IMU_MAG_REF_Y,
    IMU_MAG_REF_Z,
    IMU_MAG_CALI_XX,
    IMU_MAG_CALI_XY,
    IMU_MAG_CALI_XZ,
    IMU_MAG_CALI_YX,
    IMU_MAG_CALI_YY,
    IMU_MAG_CALI_YZ,
    IMU_MAG_CALI_ZX,
    IMU_MAG_CALI_ZY,
    IMU_MAG_CALI_ZZ,
    IMU_GYO_500DPS,
    IMU_GYO_DEGLSB,
    IMU_GYO_BIAS_X,
    IMU_GYO_BIAS_Y,
    IMU_GYO_BIAS_Z,
    MOTOR_PITCH_LEFT_ADDR,
    MOTOR_PITCH_RIGHT_ADDR,
    MOTOR_ROLL_BACK_ADDR,
    MOTOR_ROLL_FORWARD_ADDR,
    MOTOR_CTRL_MIN,
    MOTOR_CTRL_MAX,
    PID_N,
    PID_YAW_EPSILON,
    PID_YAW_DT,
    PID_YAW_MAX,
    PID_YAW_MIN,
    PID_YAW_KP,
    PID_YAW_KI,
    PID_YAW_KD,
    PID_PITCH_EPSILON,
    PID_PITCH_DT,
    PID_PITCH_MAX,
    PID_PITCH_MIN,
    PID_PITCH_KP,
    PID_PITCH_KI,
    PID_PITCH_KD,
    PID_ROLL_EPSILON,
    PID_ROLL_DT,
    PID_ROLL_MAX,
    PID_ROLL_MIN,
    PID_ROLL_KP,
    PID_ROLL_KI,
    PID_ROLL_KD,
    RC_CONTROL_N,
    RC_CONTROL_MIN,
    RC_CONTROL_MAX,
    CONFIG_NUM_PARAMS
};

extern float p[CONFIG_NUM_PARAMS];
extern const char *configParameterStrings[];

extern void configFlashRead(void);
extern unsigned char configFlashWrite(void);
#endif /* CONFIG_H_ */
