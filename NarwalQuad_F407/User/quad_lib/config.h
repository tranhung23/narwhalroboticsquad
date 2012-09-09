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
    IMU_ACC_BIAS_X,
    IMU_ACC_BIAS_Y,
    IMU_ACC_BIAS_Z,
    CONFIG_NUM_PARAMS
};

extern float p[CONFIG_NUM_PARAMS];
extern const char *configParameterStrings[];

extern void configFlashRead(void);
extern unsigned char configFlashWrite(void);
#endif /* CONFIG_H_ */
