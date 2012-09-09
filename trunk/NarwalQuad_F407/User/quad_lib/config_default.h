/*
 * config_default.h
 *
 *  Created on: Sep 9, 2012
 *      Author: GrubyGrub
 */
#include "stm32f4xx.h"

#ifndef CONFIG_DEFAULT_H_
#define CONFIG_DEFAULT_H_

#define DEFAULT_CONFIG_VERSION      100
#define DEFAULT_DEG2RAD             0.0174532925
#define DEFAULT_RAD2DEG             57.2957796

#define DEFAULT_IMU_P_COV           0.5
#define DEFAULT_IMU_Q_COV           0.000001

#define DEFAULT_IMU_ACC_COV         0.6
#define DEFAULT_IMU_ACC_BIAS_X      -1.65
#define DEFAULT_IMU_ACC_BIAS_Y      -1.65
#define DEFAULT_IMU_ACC_BIAS_Z      -1.65
#define DEFAULT_IMU_ACC_REF_X       0
#define DEFAULT_IMU_ACC_REF_Y       0
#define DEFAULT_IMU_ACC_REF_Z       1

#define DEFAULT_IMU_MAG_COV         0.6
#define DEFAULT_IMU_MAG_BIAS_X      -33.4495
#define DEFAULT_IMU_MAG_BIAS_Y      287.7760
#define DEFAULT_IMU_MAG_BIAS_Z      -142.0700
#define DEFAULT_IMU_MAG_REF_X       -0.2525
#define DEFAULT_IMU_MAG_REF_Y       -0.9674
#define DEFAULT_IMU_MAG_REF_Z       0.0177
#define DEFAULT_IMU_MAG_CALI_XX     0.0
#define DEFAULT_IMU_MAG_CALI_XY     -1.0
#define DEFAULT_IMU_MAG_CALI_XZ     0.0
#define DEFAULT_IMU_MAG_CALI_YX     0.0
#define DEFAULT_IMU_MAG_CALI_YY     0.0
#define DEFAULT_IMU_MAG_CALI_YZ     1.0
#define DEFAULT_IMU_MAG_CALI_ZX     1.0
#define DEFAULT_IMU_MAG_CALI_ZY     0.0
#define DEFAULT_IMU_MAG_CALI_ZZ     0.0

#define DEFAULT_IMU_GYO_500DPS      17.50
#define DEFAULT_IMU_GYO_DEGLSB      (1/14.375)
#define DEFAULT_IMU_GYO_BIAS_X      -22.8819
#define DEFAULT_IMU_GYO_BIAS_Y      14.2851
#define DEFAULT_IMU_GYO_BIAS_Z      5.2421

#endif /* CONFIG_DEFAULT_H_ */
