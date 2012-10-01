/*
 * Control loop of the drone
 */

#include "user_INS.h"
#include <narwhal_ADC.h>
#include <narwhal_I2C.h>
#include <sensors.h>
#include <stdio.h>

OS_STK *narwhalINSStack;
INS_Orientation OrientationValues;
#define DEBUG 1

//Shoudl be ready for analog and digital readings with a few changes
void INS_Init_Task(void)
{
	StatusType result;
	U64 ticks = 0;
	sync_printf("Init INS starting \r\n");

	/*reset all values to 0*/
	memset(&OrientationValues, 0, sizeof(OrientationValues));

	CoTickDelay(10);
	//TODO: Init task

	while (1)
	{
		//TODO: wait for UKF filter completation.

		// Update the magnometer at 150 hz.
		if (ticks % 7 == 0)
		{
			Sensors_Read_async(MAG);
		}

		/*wait for ADC to finish processing*/
		result = CoWaitForSingleFlag(ADC_FLAG, 0);
		/*Convert all the ADC values to understandalbe values*/
		INS_Sensor_ValueCalibrate(ticks);

		/*PID Loop update process*/
		ticks++;
	}
}

static void INS_Sensor_ValueCalibrate(U64 ticks)
{
	/*Current values are now in volts*/
	StatusType result;
	float ax, ay, az, gx, gy, gz, mx, my, mz, tempst;
	uint32_t currAvgIdx = ticks % AVERAGE_SAMPLES;

	/*New values values in in, update pitch yaw roll throttle*/
	if (CoAcceptSingleFlag(I2C1_FLAG) == E_OK)
	{
		/* use a union*/
		mx = (float) ((int) ((int16_t)((MARG_SENSORS[MAG]->SensorRawValue[1]) | (MARG_SENSORS[MAG]->SensorRawValue[0] << 8))));
		my = (float) ((int) ((int16_t)((MARG_SENSORS[MAG]->SensorRawValue[3]) | (MARG_SENSORS[MAG]->SensorRawValue[2] << 8))));
		mz = (float) ((int) ((int16_t)((MARG_SENSORS[MAG]->SensorRawValue[5]) | (MARG_SENSORS[MAG]->SensorRawValue[4] << 8))));

		/*TODO: Calculate temp biases*/
		mx = mx + p[IMU_MAG_BIAS_X];
		my = my + p[IMU_MAG_BIAS_Y];
		mz = mz + p[IMU_MAG_BIAS_Z];

		/*circular average buffer*/
		OrientationValues.avg_magx_sum -= OrientationValues.magx[currAvgIdx];
		OrientationValues.avg_magy_sum -= OrientationValues.magy[currAvgIdx];
		OrientationValues.avg_magz_sum -= OrientationValues.magz[currAvgIdx];

		OrientationValues.avg_magx_sum += mx;
		OrientationValues.avg_magy_sum += my;
		OrientationValues.avg_magz_sum += mz;

		OrientationValues.magx[currAvgIdx] = mx;
		OrientationValues.magy[currAvgIdx] = my;
		OrientationValues.magz[currAvgIdx] = mz;

		/*get the average of the current sample*/
		OrientationValues.avg_magx = OrientationValues.avg_magx_sum / (double) AVERAGE_SAMPLES;
		OrientationValues.avg_magy = OrientationValues.avg_magy_sum / (double) AVERAGE_SAMPLES;
		OrientationValues.avg_magz = OrientationValues.avg_magz_sum / (double) AVERAGE_SAMPLES;

#ifdef DEBUG
		//async_printf("mag x: %f, y: %f, z: %f, %lld\r\n", x, y, z, CoGetOSTime());
#endif
	}

	ax = (double) FilteringADCData.adcSum[ACCX] * ADC_DIVISOR;
	ay = (double) FilteringADCData.adcSum[ACCY] * ADC_DIVISOR;
	az = (double) FilteringADCData.adcSum[ACCZ] * ADC_DIVISOR;
	gx = (double) FilteringADCData.adcSum[GYROX] * ADC_DIVISOR;
	gy = (double) FilteringADCData.adcSum[GYROY] * ADC_DIVISOR;
	gz = (double) FilteringADCData.adcSum[GYROZ] * ADC_DIVISOR;
	tempst = (double) FilteringADCData.adcSum[TEMPST] * ADC_DIVISOR_TEMP;

	//TODO: temperature calibration data
	/*now we have voltages, do temperature calibration*/
	ax = ax + p[IMU_ACC_BIAS_X];
	ay = ay + p[IMU_ACC_BIAS_Y];
	az = az + p[IMU_ACC_BIAS_Z];
	tempst = (tempst - ADC_TEMP_REF) / ADC_MVC + ADC_TEMP_SHIFT;

	/*gyros do not need to be averaged, just temp calibrated.
	 * TODO: Temperature calibration
	 * */
	OrientationValues.avg_gyrox = gx + p[IMU_GYO_BIAS_X];
	OrientationValues.avg_gyroy = gy + p[IMU_GYO_BIAS_Y];
	OrientationValues.avg_gyroz = gz + p[IMU_GYO_BIAS_Z];

	/*subtract current average index from average values*/
	OrientationValues.avg_accx_sum -= OrientationValues.accx[currAvgIdx];
	OrientationValues.avg_accy_sum -= OrientationValues.accy[currAvgIdx];
	OrientationValues.avg_accz_sum -= OrientationValues.accz[currAvgIdx];
	OrientationValues.avg_temperatureST_sum -= OrientationValues.temperatureST[currAvgIdx];

	/*add current value to average*/
	OrientationValues.avg_accx_sum += ax;
	OrientationValues.avg_accy_sum += ay;
	OrientationValues.avg_accz_sum += az;
	OrientationValues.avg_temperatureST_sum += tempst;

	/*ACCLERMETER: set current average index with current value*/
	OrientationValues.accx[currAvgIdx] = ax;
	OrientationValues.accy[currAvgIdx] = ay;
	OrientationValues.accz[currAvgIdx] = az;
	OrientationValues.temperatureST[currAvgIdx] = tempst;

	/*IF WE ARE NOT DOING ANYTHING, then DO THIS CALIBRATION ROUTINE*/
	/*GYROSCOPE: set current average index with current value*/
	OrientationValues.gyrox[currAvgIdx] = gx;
	OrientationValues.gyroy[currAvgIdx] = gy;
	OrientationValues.gyroz[currAvgIdx] = gz;
	INS_Sensor_Static();

	OrientationValues.avg_accx = OrientationValues.avg_accx_sum / AVERAGE_SAMPLES;
	OrientationValues.avg_accy = OrientationValues.avg_accy_sum / AVERAGE_SAMPLES;
	OrientationValues.avg_accz = OrientationValues.avg_accz_sum / AVERAGE_SAMPLES;
	OrientationValues.avg_temperatureST = OrientationValues.avg_temperatureST_sum / AVERAGE_SAMPLES;

#ifdef DEBUG
	async_printf("%f %f %f %f %f %f %f\r\n", OrientationValues.avg_accx_sum, OrientationValues.avg_accy, OrientationValues.avg_accz, OrientationValues.avg_gyrox, OrientationValues.avg_gyroy, OrientationValues.avg_gyroz, OrientationValues.avg_temperatureST);
	//async_printf("Acc x: %f, y: %f, z: %f, GYRO x: %f, y: %f, z: %f\r\n", OrientationValues.accx, OrientationValues.accy, OrientationValues.accz,OrientationValues.gyrox,OrientationValues.gyroy,OrientationValues.gyroz);
#endif
}

/*record temperature and value and update the calibration table*/
static void INS_Sensor_Static(void)
{
	float gyroxstd, gyroystd, gyrozstd;
	arm_std_f32(OrientationValues.gyrox,AVERAGE_SAMPLES,&gyroxstd);
	arm_std_f32(OrientationValues.gyroy,AVERAGE_SAMPLES,&gyroystd);
	arm_std_f32(OrientationValues.gyroz,AVERAGE_SAMPLES,&gyrozstd);
	if(gyroxstd < 0.00500f)
	{
		CoWaitForSingleFlag(ADC_FLAG, 0);
	}
	printf("STD: %f, %f, %f\r\n",gyroxstd, gyroystd, gyrozstd);
}
