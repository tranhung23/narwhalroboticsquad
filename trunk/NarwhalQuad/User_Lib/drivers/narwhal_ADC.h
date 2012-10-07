/*
 * ADC_Sensors.h
 *
 *  Created on: May 16, 2012
 *      Author: GrubyGrub
 */

#ifndef ADC_SENSORS_H_
#define ADC_SENSORS_H_

#include <narwhal_top.h>





/*ADC clock is running at 42 mhz, each sample takes X cycles, conversion takes 12 cycles, there are Z channels.
 * X = Sample time, Z is samples
 * ((X + 12) * Z)/42000000 -> seconds per sample interrupt
 *
 * Interrupt fires at 14.28 us
 * 500 hz -> 2000 us
 * 140 iterations per hz
 *
 * Want 500hz runtime -> 0.002 seconds
 *
 * multiplier ->135
 *
 */

#define ADC_SAMPLE_TICKS 		ADC_SampleTime_28Cycles
#define ADC_ITERATIONS			131

#define ADC_SENSOR_NUM			7
#define ADC_SAMPLE_NUM			16
#define ADC_NUM					2 /*Using 2 adc's currently*/
#define ADC_BUFFER_MULTIPLIER	2 /*Using 2x the larger buffer to get non-overlap while conversion*/
#define ADC_RESOLUTION			12
#define ADC_SAMPLE_CNT			5 /*Number of samples each IRQ*/

#define ADC_REF_V		3.300f
#define ADC_DIVISOR		(((double)ADC_REF_V / (double)(1<<ADC_RESOLUTION) / (double)ADC_ITERATIONS))/(double)ADC_SAMPLE_CNT


#define ADC_TEMP_SHIFT			25.0f
#define ADC_MVC					1.0f/(2.5f/1000.0f)
#define ADC_TEMP_REF			0.76f
#define ADC_DIVISOR_TEMP		(((double)ADC_REF_V / (double)(1<<ADC_RESOLUTION) / (double)ADC_ITERATIONS))





#define	ADC_CH_GYROX	ADC_Channel_1
#define	ADC_CH_GYROY	ADC_Channel_2
#define	ADC_CH_GYROZ	ADC_Channel_3

#define	ADC_CH_ACCX		ADC_Channel_4
#define	ADC_CH_ACCY		ADC_Channel_5
#define	ADC_CH_ACCZ		ADC_Channel_6


#define ADC_Interrupt DMA2_Stream0_IRQHandler

//static __IO uint16_t ADCDualConvertedValue[ADC_SAMPLE_NUM];

typedef struct RawADCStruct{
    uint16_t adcRaw1[ADC_SAMPLE_NUM*ADC_NUM];
    uint16_t adcRaw2[ADC_SAMPLE_NUM*ADC_NUM];
} RawADCStruct;

typedef struct FilteredADCStruct{
	uint32_t adcSum[ADC_SENSOR_NUM]; /*the sum to use is placed here*/
	uint32_t adcIRQSum[ADC_SENSOR_NUM]; /*running totoal during IRQ*/
	uint32_t adcIRQIterations; /*number of IRQ iterations*/
	uint32_t dt;
	uint32_t previousSample;
}FilteredADCStruct;

extern RawADCStruct RawADCData;
extern FilteredADCStruct FilteringADCData;

void ADC_Sensor_DMA_Config(void);
void ADC_Sensor_GPIO_Config(void);
void ADC1_Sensor_Config(void);
void ADC2_Sensor_Config(void);
void ADC_print(void);
void ADC_SENSOR_Init(void);
void ADC_Interrupt(void);

extern OS_FlagID ADC_FLAG;


#endif /* ADC_SENSORS_H_ */
