/*
 * config.c
 *
 *  Created on: Sep 9, 2012
 *      Author: GrubyGrub
 */
#include "stm32f4xx.h"
#include "config.h"
#include "flash.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include CONFIG_HEADER

float p[CONFIG_NUM_PARAMS];

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
}

void configFlashRead(void){
    memcpy(&p, (char *)flashStartAddr(), sizeof(p));
}

unsigned char configFlashWrite(void){
    configLoadDefault();
    return flashAddress(flashStartAddr(), (uint32_t *)&p, sizeof(p));
}
