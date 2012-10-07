/*
 * Control loop of the drone
 */

#include "user_INS.h"
#include <narwhal_ADC.h>
#include <narwhal_I2C.h>
#include <narwhal_MICRO_TIMER.h>
#include <sensors.h>
#include <stdio.h>

OS_STK *narwhalINSStack;
INS_Orientation OrientationValues __attribute__ ((section(".ccm")));
Calibration_SampleSpace Calibration_SS __attribute__ ((section(".ccm")));
#define DEBUG 1

//Shoudl be ready for analog and digital readings with a few changes
void INS_Init_Task(void)
{
	StatusType result;
	U64 ticks = 0;
	sync_printf("Init INS starting \r\n");

	/*reset all values to 0*/
	memset(&OrientationValues, 0, sizeof(OrientationValues));
	memset(&Calibration_SS, 0, sizeof(Calibration_SS));

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
		/*Loop update process*/
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
		//TODO: Ticks are broken here..
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
	//OrientationValues.dt = FilteringADCData.dt;

	ax = (double) FilteringADCData.adcSum[ACCX] * ADC_DIVISOR;
	ay = (double) FilteringADCData.adcSum[ACCY] * ADC_DIVISOR;
	az = (double) FilteringADCData.adcSum[ACCZ] * ADC_DIVISOR;
	gx = (double) FilteringADCData.adcSum[GYROX] * ADC_DIVISOR;
	gy = (double) FilteringADCData.adcSum[GYROY] * ADC_DIVISOR;
	gz = (double) FilteringADCData.adcSum[GYROZ] * ADC_DIVISOR;
	tempst = (double) FilteringADCData.adcSum[TEMPST] * ADC_DIVISOR_TEMP;

	OrientationValues.dt = FilteringADCData.dt;


	//TODO: temperature calibration data
	/*now we have voltages, do temperature calibration*/
	ax = ax + p[IMU_ACC_BIAS_X];
	ay = ay + p[IMU_ACC_BIAS_Y];
	az = az + p[IMU_ACC_BIAS_Z];
	tempst = (tempst - ADC_TEMP_REF) / ADC_MVC + ADC_TEMP_SHIFT;

	/*gyros do not need to be averaged, just temp calibrated.
	 * TODO: Temperature calibration
	 * */
	OrientationValues.avg_gyrox = (gx + p[IMU_GYO_BIAS_X])*500;
	OrientationValues.avg_gyroy = (gy + p[IMU_GYO_BIAS_Y])*p[IMU_GYO_DEGLSB];
	OrientationValues.avg_gyroz = (gz + p[IMU_GYO_BIAS_Z])*p[IMU_GYO_DEGLSB];

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
	if (1)
	{
		OrientationValues.gyrox[currAvgIdx] = gx;
		OrientationValues.gyroy[currAvgIdx] = gy;
		OrientationValues.gyroz[currAvgIdx] = gz;
		/*do this every average samples*/
		if (ticks % AVERAGE_SAMPLES == 0)
			INS_Sensor_Static();
	}

	OrientationValues.avg_accx = OrientationValues.avg_accx_sum / AVERAGE_SAMPLES;
	OrientationValues.avg_accy = OrientationValues.avg_accy_sum / AVERAGE_SAMPLES;
	OrientationValues.avg_accz = OrientationValues.avg_accz_sum / AVERAGE_SAMPLES;
	OrientationValues.avg_temperatureST = OrientationValues.avg_temperatureST_sum / AVERAGE_SAMPLES;

#ifdef DEBUG
	if(ticks%10 == 0)
	async_printf("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%u \r\n", OrientationValues.avg_accx, OrientationValues.avg_accy, OrientationValues.avg_accz, OrientationValues.avg_gyrox, OrientationValues.avg_gyroy, OrientationValues.avg_gyroz,OrientationValues.avg_magx,OrientationValues.avg_magy,OrientationValues.avg_magz, OrientationValues.avg_temperatureST, OrientationValues.dt);
	//async_printf("Acc x: %f, y: %f, z: %f, GYRO x: %f, y: %f, z: %f\r\n", OrientationValues.accx, OrientationValues.accy, OrientationValues.accz,OrientationValues.gyrox,OrientationValues.gyroy,OrientationValues.gyroz);
#endif
}

/*record temperature and value and update the calibration table*/
static void INS_Sensor_Static(void)
{
	float acc_xstd, acc_ystd, acc_zstd;

	arm_std_f32(OrientationValues.accx, AVERAGE_SAMPLES, &acc_xstd);
	arm_std_f32(OrientationValues.accy, AVERAGE_SAMPLES, &acc_ystd);
	arm_std_f32(OrientationValues.accz, AVERAGE_SAMPLES, &acc_zstd);

	//printf("STD: %f, %f, %f\r\n", acc_xstd, acc_ystd, acc_zstd);
	/*dont do anything if the std is too high, that means we are not static*/
	if ((acc_xstd + acc_ystd + acc_zstd) > STEADY_STATE_STD_ACC * 3.0f)
	{
		return;
	}

	/*set the temp when we are starting a new sample space*/
	if (Calibration_SS.calibration_samplecnt == 0)
	{
		Calibration_SS.calibration_temperature = OrientationValues.avg_temperatureST;
	}
	/*If the current sample space's temperature vary by 1 degree, then reset sample space*/
	else if ((int) Calibration_SS.calibration_temperature != (int) OrientationValues.avg_temperatureST)
	{
		Calibration_SS.calibration_samplecnt = 0;
		return;
	}

	/*everything checks out..*/
	arm_copy_f32(OrientationValues.gyrox,&(Calibration_SS.gyro_x_samplespace[Calibration_SS.calibration_samplecnt * AVERAGE_SAMPLES]), AVERAGE_SAMPLES);
	arm_copy_f32(OrientationValues.gyroy,&(Calibration_SS.gyro_y_samplespace[Calibration_SS.calibration_samplecnt * AVERAGE_SAMPLES]), AVERAGE_SAMPLES);
	arm_copy_f32(OrientationValues.gyroz,&(Calibration_SS.gyro_z_samplespace[Calibration_SS.calibration_samplecnt * AVERAGE_SAMPLES]), AVERAGE_SAMPLES);

	Calibration_SS.calibration_samplecnt++;

	/*Calculate the current mean for gyros*/
	float meanx, meany, meanz;
	if (Calibration_SS.calibration_samplecnt == TEMP_CALIBRATION_SAMPLE)
	{
		arm_mean_f32(Calibration_SS.gyro_x_samplespace, TEMP_CALIBRATION_SAMPLE * AVERAGE_SAMPLES, &meanx);
		arm_mean_f32(Calibration_SS.gyro_y_samplespace, TEMP_CALIBRATION_SAMPLE * AVERAGE_SAMPLES, &meany);
		arm_mean_f32(Calibration_SS.gyro_z_samplespace, TEMP_CALIBRATION_SAMPLE * AVERAGE_SAMPLES, &meanz);

		p[IMU_GYO_BIAS_X] = -meanx;
		p[IMU_GYO_BIAS_Y] = -meany;
		p[IMU_GYO_BIAS_Z] = -meanz;
		Calibration_SS.calibration_samplecnt = 0;
 		return;
	}
}
