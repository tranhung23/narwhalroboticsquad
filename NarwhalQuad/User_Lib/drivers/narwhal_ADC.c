/*
 * adc_sensors.c
 *
 *  Created on: Mar 23, 2012
 *      Author: GrubyGrub
 */

#include "narwhal_ADC.h"

/*TODO: clean up code*/

/* ADC1 - CH1 - G_X - PA1
 * ADC1 - CH2 - G_Y - PA2
 * ADC1 - CH3 - G_Z - PA3
 *
 * ADC2 - CH4 - G_X - PA4
 * ADC2 - CH5 - G_Y - PA5
 * ADC2 - CH6 - G_Z - PA6
 */
//TODO: Clean up this file

/*Clock is APB2/2 > (168/2)/2 mhz */

struct RawADCStruct RawADCData;
struct FilteredADCStruct FilteringADCData;
OS_FlagID ADC_FLAG;

void ADC_SENSOR_Init(void)
{

	memset(&RawADCData, 0x0, sizeof(RawADCData));
	memset(&FilteringADCData, 0x0, sizeof(FilteringADCData));

	ADC_CommonInitTypeDef ADC_CommonInitStructure;

	/* Enable peripheral clocks *************************************************/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2 | RCC_APB2Periph_ADC3, ENABLE);

	/* ADCs configuration ------------------------------------------------------*/
	/* Configure ADC Channel10, 11, 12 pin as analog input */
	ADC_Sensor_GPIO_Config();

	/* DMA2 Stream0 channel0 configuration **************************************/
	ADC_Sensor_DMA_Config();

	/* ADC Common Init */
	ADC_CommonInitStructure.ADC_Mode = ADC_DualMode_RegSimult;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
	//ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_10Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);

	/* ADC1 regular channels 10, 11 configuration */
	ADC1_Sensor_Config();

	/* ADC2 regular channels 11, 12 configuration */
	ADC2_Sensor_Config();

	/* Enable DMA request after last transfer (Multi-ADC mode)  */
	ADC_MultiModeDMARequestAfterLastTransferCmd(ENABLE);
	ADC_TempSensorVrefintCmd(ENABLE);

	ADC_FLAG = CoCreateFlag(AUTO_RESET, NONREADY_STATE);
	if (ADC_FLAG < 0)
	{
		//TODO: Panic
	}
	FilteringADCData.adcIRQIterations = ADC_ITERATIONS;

	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);

	/* Enable ADC2 */
	ADC_Cmd(ADC2, ENABLE);

	/* Start ADC1 Software Conversion */
	ADC_SoftwareStartConv(ADC1);
	ADC_SoftwareStartConv(ADC2);

}

void ADC_Sensor_GPIO_Config(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

}

void ADC_Sensor_DMA_Config(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	DMA_InitStructure.DMA_Channel = DMA_Channel_0;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) &RawADCData.adcRaw1;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) ADC_CCR_ADDRESS;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = ADC_SAMPLE_NUM * ADC_NUM * ADC_BUFFER_MULTIPLIER;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream0, &DMA_InitStructure);

	/*Interrupt when half of DMA is done, so we do not overwrite our values when processing*/
	DMA_ITConfig(DMA2_Stream0, DMA_IT_HT | DMA_IT_TC, ENABLE);
	DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TEIF0 | DMA_IT_DMEIF0 | DMA_IT_FEIF0 | DMA_IT_TCIF0 | DMA_IT_HTIF0);

	// Enable the DMA2_Stream0 global Interrupt
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* DMA2_Stream0 enable */
	DMA_Cmd(DMA2_Stream0, ENABLE);

}

void ADC1_Sensor_Config(void)
{
	ADC_InitTypeDef ADC_InitStructure;

	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = ADC_SAMPLE_NUM;
	ADC_Init(ADC1, &ADC_InitStructure);

	/* ADC1 regular channels 10, 11 configuration */
	ADC_RegularChannelConfig(ADC1, ADC_CH_GYROX, 1, ADC_SAMPLE_TICKS);
	ADC_RegularChannelConfig(ADC1, ADC_CH_GYROX, 2, ADC_SAMPLE_TICKS);
	ADC_RegularChannelConfig(ADC1, ADC_CH_GYROX, 3, ADC_SAMPLE_TICKS);
	ADC_RegularChannelConfig(ADC1, ADC_CH_GYROX, 4, ADC_SAMPLE_TICKS);
	ADC_RegularChannelConfig(ADC1, ADC_CH_GYROX, 5, ADC_SAMPLE_TICKS);

	ADC_RegularChannelConfig(ADC1, ADC_CH_GYROY, 6, ADC_SAMPLE_TICKS);
	ADC_RegularChannelConfig(ADC1, ADC_CH_GYROY, 7, ADC_SAMPLE_TICKS);
	ADC_RegularChannelConfig(ADC1, ADC_CH_GYROY, 8, ADC_SAMPLE_TICKS);
	ADC_RegularChannelConfig(ADC1, ADC_CH_GYROY, 9, ADC_SAMPLE_TICKS);
	ADC_RegularChannelConfig(ADC1, ADC_CH_GYROY, 10, ADC_SAMPLE_TICKS);

	ADC_RegularChannelConfig(ADC1, ADC_CH_GYROZ, 11, ADC_SAMPLE_TICKS);
	ADC_RegularChannelConfig(ADC1, ADC_CH_GYROZ, 12, ADC_SAMPLE_TICKS);
	ADC_RegularChannelConfig(ADC1, ADC_CH_GYROZ, 13, ADC_SAMPLE_TICKS);
	ADC_RegularChannelConfig(ADC1, ADC_CH_GYROZ, 14, ADC_SAMPLE_TICKS);
	ADC_RegularChannelConfig(ADC1, ADC_CH_GYROZ, 15, ADC_SAMPLE_TICKS);

	ADC_RegularChannelConfig(ADC1, ADC_Channel_TempSensor, 16, ADC_SAMPLE_TICKS);
}

/**
 * @brief  ADC2 regular channels 11, 12 configuration
 * @param  None
 * @retval None
 */
void ADC2_Sensor_Config(void)
{
	ADC_InitTypeDef ADC_InitStructure;

	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = ADC_SAMPLE_NUM;
	ADC_Init(ADC2, &ADC_InitStructure);

	/* ADC2 regular channels 11, 12 configuration */
	ADC_RegularChannelConfig(ADC2, ADC_CH_ACCX, 1, ADC_SAMPLE_TICKS);
	ADC_RegularChannelConfig(ADC2, ADC_CH_ACCX, 2, ADC_SAMPLE_TICKS);
	ADC_RegularChannelConfig(ADC2, ADC_CH_ACCX, 3, ADC_SAMPLE_TICKS);
	ADC_RegularChannelConfig(ADC2, ADC_CH_ACCX, 4, ADC_SAMPLE_TICKS);
	ADC_RegularChannelConfig(ADC2, ADC_CH_ACCX, 5, ADC_SAMPLE_TICKS);

	ADC_RegularChannelConfig(ADC2, ADC_CH_ACCY, 6, ADC_SAMPLE_TICKS);
	ADC_RegularChannelConfig(ADC2, ADC_CH_ACCY, 7, ADC_SAMPLE_TICKS);
	ADC_RegularChannelConfig(ADC2, ADC_CH_ACCY, 8, ADC_SAMPLE_TICKS);
	ADC_RegularChannelConfig(ADC2, ADC_CH_ACCY, 9, ADC_SAMPLE_TICKS);
	ADC_RegularChannelConfig(ADC2, ADC_CH_ACCY, 10, ADC_SAMPLE_TICKS);

	ADC_RegularChannelConfig(ADC2, ADC_CH_ACCZ, 11, ADC_SAMPLE_TICKS);
	ADC_RegularChannelConfig(ADC2, ADC_CH_ACCZ, 12, ADC_SAMPLE_TICKS);
	ADC_RegularChannelConfig(ADC2, ADC_CH_ACCZ, 13, ADC_SAMPLE_TICKS);
	ADC_RegularChannelConfig(ADC2, ADC_CH_ACCZ, 14, ADC_SAMPLE_TICKS);
	ADC_RegularChannelConfig(ADC2, ADC_CH_ACCZ, 15, ADC_SAMPLE_TICKS);

	ADC_RegularChannelConfig(ADC2, ADC_CH_ACCZ, 16, ADC_SAMPLE_TICKS);
}

void ADC_print(void)
{
	//sync_printf("%d %d %d %d %d %d\r\n", ADCDualConvertedValue[GYRO_X], ADCDualConvertedValue[GYRO_Y], ADCDualConvertedValue[GYRO_Z], ADCDualConvertedValue[ACC_X], ADCDualConvertedValue[ACC_Y], ADCDualConvertedValue[ACC_Z]);
}

void ADC_Interrupt(void)
{
	register uint32_t flag = DMA2->LISR;
	register uint16_t *adcValue;
	register uint32_t *sum, *sumbuffer;
	register int i;

	// clear intr flags
	DMA2->LIFCR = (uint32_t) (DMA_IT_TEIF0 | DMA_IT_DMEIF0 | DMA_IT_FEIF0 | DMA_IT_TCIF0 | DMA_IT_HTIF0);

	// second half?
	adcValue = ((flag & DMA_IT_TCIF0) == RESET) ? RawADCData.adcRaw1 : RawADCData.adcRaw2;

	sum = FilteringADCData.adcIRQSum;

	*sum++ += (adcValue[0] + adcValue[2] + adcValue[4] + adcValue[6] + adcValue[8]);/*Gyro X*/
	*sum++ += (adcValue[10] + adcValue[12] + adcValue[14] + adcValue[16] + adcValue[18]);/*Gyro Y*/
	*sum++ += (adcValue[20] + adcValue[22] + adcValue[24] + adcValue[26] + adcValue[28]);/*Gyro Z*/

	*sum++ += (adcValue[1] + adcValue[3] + adcValue[5] + adcValue[7] + adcValue[9]);/*ACC X*/
	*sum++ += (adcValue[11] + adcValue[13] + adcValue[15] + adcValue[17] + adcValue[19]);/*ACC Y*/
	*sum++ += (adcValue[21] + adcValue[23] + adcValue[25] + adcValue[27] + adcValue[29]);/*ACC Z*/
	*sum++ += adcValue[30];



	/*Save 1 cycle by counting down...*/
	FilteringADCData.adcIRQIterations--;

	if (FilteringADCData.adcIRQIterations == 0)
	{
		sum = FilteringADCData.adcIRQSum;
		sumbuffer = (uint32_t *) FilteringADCData.adcSum;

		FilteringADCData.adcIRQIterations = ADC_ITERATIONS;

		/*Copy reg from IRQ sum to use sum*/
		/*possible mutex use here, but unneeded since ctx switch shoudl not happen here..*/
		for (i = 0; i < ADC_SENSOR_NUM; i++)
		{
			*sumbuffer++ = *sum;
			*sum++ = 0;
		}
		CoEnterISR();
		isr_SetFlag(ADC_FLAG);
		CoExitISR();
	}

}
