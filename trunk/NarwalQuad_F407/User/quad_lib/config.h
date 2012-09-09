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
    CONFIG_NUM_PARAMS
};

extern float p[CONFIG_NUM_PARAMS];
extern const char *configParameterStrings[];

extern void configFlashRead(void);
extern unsigned char configFlashWrite(void);
#endif /* CONFIG_H_ */
