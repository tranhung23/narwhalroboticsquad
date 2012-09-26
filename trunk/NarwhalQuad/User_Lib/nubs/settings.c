/*
 * config.c
 *
 *  Created on: Sep 9, 2012
 *      Author: GrubyGrub
 */
#include <narwhal_top.h>
#include "flash.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include CONFIG_HEADER

float p[CONFIG_NUM_PARAMS]  __attribute__((section(".ccm")));

const char *configParameterStrings[] = {
        "CONFIG_VERSION",
        "DEG2RAD",
        "RAD2DEG",
        "IMU_P_COV",
        "IMU_Q_COV",
        "IMU_ACC_COV",
        "IMU_ACC_BIAS_X",
        "IMU_ACC_BIAS_Y",
        "IMU_ACC_BIAS_Z",
        "IMU_ACC_REF_X",
        "IMU_ACC_REF_Y",
        "IMU_ACC_REF_Z",
        "IMU_MAG_COV",
        "IMU_MAG_BIAS_X",
        "IMU_MAG_BIAS_Y",
        "IMU_MAG_BIAS_Z",
        "IMU_MAG_REF_X",
        "IMU_MAG_REF_Y",
        "IMU_MAG_REF_Z",
        "IMU_MAG_CALI_XX",
        "IMU_MAG_CALI_XY",
        "IMU_MAG_CALI_XZ",
        "IMU_MAG_CALI_YX",
        "IMU_MAG_CALI_YY",
        "IMU_MAG_CALI_YZ",
        "IMU_MAG_CALI_ZX",
        "IMU_MAG_CALI_ZY",
        "IMU_MAG_CALI_ZZ",
        "IMU_GYO_500DPS",
        "IMU_GYO_DEGLSB",
        "IMU_GYO_BIAS_X",
        "IMU_GYO_BIAS_Y",
        "IMU_GYO_BIAS_Z",
        "MOTOR_PITCH_LEFT_ADDR",
        "MOTOR_PITCH_RIGHT_ADDR",
        "MOTOR_ROLL_BACK_ADDR",
        "MOTOR_ROLL_FORWARD_ADDR",
        "MOTOR_CTRL_MIN",
        "MOTOR_CTRL_MAX",
        "PID_N",
        "PID_YAW_EPSILON",
        "PID_YAW_DT",
        "PID_YAW_MAX",
        "PID_YAW_MIN",
        "PID_YAW_KP",
        "PID_YAW_KI",
        "PID_YAW_KD",
        "PID_PITCH_EPSILON",
        "PID_PITCH_DT",
        "PID_PITCH_MAX",
        "PID_PITCH_MIN",
        "PID_PITCH_KP",
        "PID_PITCH_KI",
        "PID_PITCH_KD",
        "PID_ROLL_EPSILON",
        "PID_ROLL_DT",
        "PID_ROLL_MAX",
        "PID_ROLL_MIN",
        "PID_ROLL_KP",
        "PID_ROLL_KI",
        "PID_ROLL_KD",
        "RC_CONTROL_N",
        "RC_CONTROL_MIN",
        "RC_CONTROL_MAX",
};

void configLoadDefault(void){
    p[CONFIG_VERSION] = DEFAULT_CONFIG_VERSION;
    p[DEG2RAD] = DEFAULT_DEG2RAD;
    p[RAD2DEG] = DEFAULT_RAD2DEG;
    p[IMU_P_COV] = DEFAULT_IMU_P_COV;
    p[IMU_Q_COV] = DEFAULT_IMU_Q_COV;
    p[IMU_ACC_COV] = DEFAULT_IMU_ACC_COV;
    p[IMU_ACC_BIAS_X] = DEFAULT_IMU_ACC_BIAS_X;
    p[IMU_ACC_BIAS_Y] = DEFAULT_IMU_ACC_BIAS_Y;
    p[IMU_ACC_BIAS_Z] = DEFAULT_IMU_ACC_BIAS_Z;
    p[IMU_ACC_REF_X] = DEFAULT_IMU_ACC_REF_X;
    p[IMU_ACC_REF_Y] = DEFAULT_IMU_ACC_REF_Y;
    p[IMU_ACC_REF_Z] = DEFAULT_IMU_ACC_REF_Z;
    p[IMU_MAG_COV] = DEFAULT_IMU_MAG_COV;
    p[IMU_MAG_BIAS_X] = DEFAULT_IMU_MAG_BIAS_X;
    p[IMU_MAG_BIAS_Y] = DEFAULT_IMU_MAG_BIAS_Y;
    p[IMU_MAG_BIAS_Z] = DEFAULT_IMU_MAG_BIAS_Z;
    p[IMU_MAG_REF_X] = DEFAULT_IMU_MAG_REF_X;
    p[IMU_MAG_REF_Y] = DEFAULT_IMU_MAG_REF_Y;
    p[IMU_MAG_REF_Z] = DEFAULT_IMU_MAG_REF_Z;
    p[IMU_MAG_CALI_XX] = DEFAULT_IMU_MAG_CALI_XX;
    p[IMU_MAG_CALI_XY] = DEFAULT_IMU_MAG_CALI_XY;
    p[IMU_MAG_CALI_XZ] = DEFAULT_IMU_MAG_CALI_XZ;
    p[IMU_MAG_CALI_YX] = DEFAULT_IMU_MAG_CALI_YX;
    p[IMU_MAG_CALI_YY] = DEFAULT_IMU_MAG_CALI_YY;
    p[IMU_MAG_CALI_YZ] = DEFAULT_IMU_MAG_CALI_YZ;
    p[IMU_MAG_CALI_ZX] = DEFAULT_IMU_MAG_CALI_ZX;
    p[IMU_MAG_CALI_ZY] = DEFAULT_IMU_MAG_CALI_ZY;
    p[IMU_MAG_CALI_ZZ] = DEFAULT_IMU_MAG_CALI_ZZ;
    p[IMU_GYO_500DPS] = DEFAULT_IMU_GYO_500DPS;
    p[IMU_GYO_DEGLSB] = DEFAULT_IMU_GYO_DEGLSB;
    p[IMU_GYO_BIAS_X] = DEFAULT_IMU_GYO_BIAS_X;
    p[IMU_GYO_BIAS_Y] = DEFAULT_IMU_GYO_BIAS_Y;
    p[IMU_GYO_BIAS_Z] = DEFAULT_IMU_GYO_BIAS_Z;
    p[MOTOR_PITCH_LEFT_ADDR] = DEFAULT_MOTOR_PITCH_LEFT_ADDR;
    p[MOTOR_PITCH_RIGHT_ADDR] = DEFAULT_MOTOR_PITCH_RIGHT_ADDR;
    p[MOTOR_ROLL_BACK_ADDR] = DEFAULT_MOTOR_ROLL_BACK_ADDR;
    p[MOTOR_ROLL_FORWARD_ADDR] = DEFAULT_MOTOR_ROLL_FORWARD_ADDR;
    p[MOTOR_CTRL_MIN] = DEFAULT_MOTOR_CTRL_MIN;
    p[MOTOR_CTRL_MAX] = DEFAULT_MOTOR_CTRL_MAX;
    p[PID_N] = DEFAULT_PID_N;
    p[PID_YAW_EPSILON] = DEFAULT_PID_YAW_EPSILON;
    p[PID_YAW_DT] = DEFAULT_PID_YAW_DT;
    p[PID_YAW_MAX] = DEFAULT_PID_YAW_MAX;
    p[PID_YAW_MIN] = DEFAULT_PID_YAW_MIN;
    p[PID_YAW_KP] = DEFAULT_PID_YAW_KP;
    p[PID_YAW_KI] = DEFAULT_PID_YAW_KI;
    p[PID_YAW_KD] = DEFAULT_PID_YAW_KD;
    p[PID_PITCH_EPSILON] = DEFAULT_PID_PITCH_EPSILON;
    p[PID_PITCH_DT] = DEFAULT_PID_PITCH_DT;
    p[PID_PITCH_MAX] = DEFAULT_PID_PITCH_MAX;
    p[PID_PITCH_MIN] = DEFAULT_PID_PITCH_MIN;
    p[PID_PITCH_KP] = DEFAULT_PID_PITCH_KP;
    p[PID_PITCH_KI] = DEFAULT_PID_PITCH_KI;
    p[PID_PITCH_KD] = DEFAULT_PID_PITCH_KD;
    p[PID_ROLL_EPSILON] = DEFAULT_PID_ROLL_EPSILON;
    p[PID_ROLL_DT] = DEFAULT_PID_ROLL_DT;
    p[PID_ROLL_MAX] = DEFAULT_PID_ROLL_MAX;
    p[PID_ROLL_MIN] = DEFAULT_PID_ROLL_MIN;
    p[PID_ROLL_KP] = DEFAULT_PID_ROLL_KP;
    p[PID_ROLL_KI] = DEFAULT_PID_ROLL_KI;
    p[PID_ROLL_KD] = DEFAULT_PID_ROLL_KD;
    p[RC_CONTROL_N] = DEFAULT_RC_CONTROL_N;
    p[RC_CONTROL_MIN] = DEFAULT_RC_CONTROL_MIN;
    p[RC_CONTROL_MAX] = DEFAULT_RC_CONTROL_MAX;
}

void configFlashRead(void){
    memcpy(&p, (char *)flashStartAddr(), sizeof(p));
}

unsigned char configFlashWrite(void){
    configLoadDefault();
    return flashAddress(flashStartAddr(), (uint32_t *)&p, sizeof(p));
}
