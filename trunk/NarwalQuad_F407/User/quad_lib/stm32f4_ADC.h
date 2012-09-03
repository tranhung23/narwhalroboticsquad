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

//These are the places in the ADCDualConvertedValue array in which the gyro/acc values are
#define GYRO_X 0
#define GYRO_Y 2
#define GYRO_Z 4

#define ACC_X 1
#define ACC_Y 3
#define ACC_Z 5

#define MAG_X 0
#define MAG_Y 0
#define MAG_Z 0

#define ADC_CCR_ADDRESS    ((uint32_t)0x40012308)

void ADC_Sensor_DMA_Config(void);
void ADC_Sensor_GPIO_Config(void);
void ADC1_Sensor_Config(void);
void ADC2_Sensor_Config(void);

void ADC_print(void);

#endif /* ADC_SENSORS_H_ */
