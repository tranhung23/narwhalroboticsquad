/*
 This file is part of AutoQuad.

 AutoQuad is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 AutoQuad is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 You should have received a copy of the GNU General Public License
 along with AutoQuad.  If not, see <http://www.gnu.org/licenses/>.

 Copyright © 2011, 2012  Bill Nesbitt
 */

#include <narwhal_top.h>
#include "ukf.h"

navUkfStruct_t navUkfData;

/*Init UKF*/
void navUkfInit(void)
{
	float Q[SIM_S]; // state variance
	float V[SIM_V]; // process variance
	float mag[3];

	//memset((void *) &navUkfData, 0, sizeof(navUkfData));

	navUkfData.v0a[0] = 0.0f;
	navUkfData.v0a[1] = 0.0f;
	navUkfData.v0a[2] = -1.0f;

	// calculate mag vector based on inclination
	//mag[0] = cosf(p[IMU_MAG_INCL] * DEG2RAD);
	//mag[1] = 0.0f;
	//mag[2] = -sinf(p[IMU_MAG_INCL] * DEG2RAD);

	// rotate local mag vector to align with true north
	//navUkfData.v0m[0] = mag[0] * cosf(p[IMU_MAG_DECL] * DEG_TO_RAD) - mag[1] * sinf(p[IMU_MAG_DECL] * DEG_TO_RAD);
	//navUkfData.v0m[1] = mag[1] * cosf(p[IMU_MAG_DECL] * DEG_TO_RAD) + mag[0] * sinf(p[IMU_MAG_DECL] * DEG_TO_RAD);
	//navUkfData.v0m[2] = mag[2];

	//navUkfData.kf = srcdkfInit(SIM_S, SIM_M, SIM_V, SIM_N, navUkfTimeUpdate);

	//navUkfData.x = srcdkfGetState(navUkfData.kf);

	Q[0] = p[UKF_VEL_Q];
	Q[1] = p[UKF_VEL_Q];
	Q[2] = p[UKF_VEL_ALT_Q];
	Q[3] = p[UKF_POS_Q];
	Q[4] = p[UKF_POS_Q];
	Q[5] = p[UKF_POS_ALT_Q];
	Q[6] = p[UKF_ACC_BIAS_Q];
	Q[7] = p[UKF_ACC_BIAS_Q];
	Q[8] = p[UKF_ACC_BIAS_Q];
	Q[9] = p[UKF_GYO_BIAS_Q];
	Q[10] = p[UKF_GYO_BIAS_Q];
	Q[11] = p[UKF_GYO_BIAS_Q];
	Q[12] = p[UKF_QUAT_Q];
	Q[13] = p[UKF_QUAT_Q];
	Q[14] = p[UKF_QUAT_Q];
	Q[15] = p[UKF_QUAT_Q];
	Q[16] = p[UKF_PRES_ALT_Q];

	V[0] = p[UKF_ACC_BIAS_V];
	V[1] = p[UKF_ACC_BIAS_V];
	V[2] = p[UKF_ACC_BIAS_V];
	V[3] = p[UKF_GYO_BIAS_V];
	V[4] = p[UKF_GYO_BIAS_V];
	V[5] = p[UKF_GYO_BIAS_V];
	V[6] = p[UKF_RATE_V];
	V[7] = p[UKF_RATE_V];
	V[8] = p[UKF_RATE_V];
	V[9] = p[UKF_PRES_ALT_V];
	V[10] = p[UKF_VEL_V];
	V[11] = p[UKF_VEL_V];
	V[12] = p[UKF_ALT_VEL_V];
	V[13] = p[UKF_POS_V];
	V[14] = p[UKF_POS_V];
	V[15] = p[UKF_ALT_POS_V];

	//srcdkfSetVariance(navUkfData.kf, Q, V, 0, 0);

	//navUkfInitState();

}

void navUkfInitState(void)
{
//	uint32_t lastUpdate;
//	float acc[3], mag[3];
//	float estAcc[3], estMag[3];
//	float vX[UKF_GYO_AVG_NUM];
//	float vY[UKF_GYO_AVG_NUM];
//	float vZ[UKF_GYO_AVG_NUM];
//	float stdX, stdY, stdZ;
//	float m[3 * 3];
//	int i, j;

	// vel
	UKF_VELN = 0.0;
	UKF_VELE = 0.0;
	UKF_VELD = 0.0;

	// pos
	UKF_POSN = 0.0;
	UKF_POSE = 0.0;
	//UKF_POSD = navUkfPresToAlt(AQ_PRESSURE);

	// acc bias
	UKF_ACC_BIAS_X = 0.0;
	UKF_ACC_BIAS_Y = 0.0;
	UKF_ACC_BIAS_Z = 0.0;

	// gyo bias
	UKF_GYO_BIAS_X = 0.0;
	UKF_GYO_BIAS_Y = 0.0;
	UKF_GYO_BIAS_Z = 0.0;

	// quat
	UKF_Q1 = 1.0f;
	UKF_Q2 = 0.0f;
	UKF_Q3 = 0.0f;
	UKF_Q4 = 0.0f;

	//UKF_PRES_ALT = navUkfPresToAlt(AQ_PRESSURE);

}


void navUkfFinish(void)
{
	navUkfNormalizeQuat(&UKF_Q1, &UKF_Q1);
	navUkfQuatExtractEuler(&UKF_Q1, &navUkfData.yaw, &navUkfData.pitch, &navUkfData.roll);
	//navUkfData.yaw = compassNormalize(navUkfData.yaw * RAD2DEG);
	navUkfData.pitch *= RAD2DEG;
	navUkfData.roll *= RAD2DEG;

	//    x' = x cos f - y sin f
	//    y' = y cos f + x sin f
	navUkfData.yawCos = cosf(navUkfData.yaw * DEG2RAD);
	navUkfData.yawSin = sinf(navUkfData.yaw * DEG2RAD);
}


void navUkfRateUpdate(float *u, float *x, float *noise, float *y) {
    y[0] = -x[9+(int)u[0]] + noise[0];
}


/*used for calibration, zero's the gyroscope rates on a particlar axies*/
void navUkfZeroRate(float rate, int axis)
{
	float noise[1]; // measurement variance
	float y[1]; // measurment(s)
	float u[1]; // user data

	noise[0] = 0.00001f;
	y[0] = rate;
	u[0] = (float) axis;

	srcdkfMeasurementUpdate(navUkfData.kf, u, y, 1, 1, noise, navUkfRateUpdate);
}
