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
        "IMU_ACC_BIAS_X",
        "IMU_ACC_BIAS_Y",
        "IMU_ACC_BIAS_Z"
};

void configLoadDefault(void){
    p[CONFIG_VERSION] = DEFAULT_CONFIG_VERSION;
    p[IMU_ACC_BIAS_X] = DEFAULT_IMU_ACC_BIAS_X;
    p[IMU_ACC_BIAS_Y] = DEFAULT_IMU_ACC_BIAS_Y;
    p[IMU_ACC_BIAS_Z] = DEFAULT_IMU_ACC_BIAS_Z;
}

void configFlashRead(void){
    memcpy(&p, (char *)flashStartAddr(), sizeof(p));
}

unsigned char configFlashWrite(void){
    configLoadDefault();
    return flashAddress(flashStartAddr(), (uint32_t *)&p, sizeof(p));
}
