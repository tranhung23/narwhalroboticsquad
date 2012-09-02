/*
 * ADC_Sensors.h
 *
 *  Created on: May 16, 2012
 *      Author: GrubyGrub
 */

#ifndef ADC_SENSORS_H_
#define ADC_SENSORS_H_

#include "stm32f4xx.h"

static __IO uint16_t ADCDualConvertedValue[6];

#define GYRO_X 0
#define GYRO_Y 2
#define GYRO_Z 4

#define ACC_X 1
#define ACC_Y 3
#define ACC_Z 5

#define MAG_X 0
#define MAG_Y 0
#define MAG_Z 0

#endif /* ADC_SENSORS_H_ */
