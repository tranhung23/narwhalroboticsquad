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

/*
 * Pressure sensor measurement update
 */

/*Acclermeter has arrived, update the estimation (LOCALTION CORRECTION: ALTITUDE)*/
void simDoPresUpdate(float pres)
{
	float noise[1]; // measurement variance
	float y[1]; // measurment(s)

	noise[0] = p[UKF_ALT_N];
	//if (!(supervisorData.state & STATE_FLYING))
	//	noise[0] *= 0.001f;

	y[0] = navUkfPresToAlt(pres);

	srcdkfMeasurementUpdate(navUkfData.kf, 0, y, 1, 1, noise, navUkfPresUpdate);
}

/*Function to get altitude from sigma pts*/
void navUkfPresUpdate(float *u, float *x, float *noise, float *y)
{
	y[0] = x[16] + noise[0]; // return altitude
}




/*
 * Accelerometer measurement update
 */

/*Acclermeter has arrived, update the estimation (ATTITUDE CORRECTION: PITCH,ROLL)*/
void simDoAccUpdate(float accX, float accY, float accZ)
{
	float noise[3]; // measurement variance
	float y[3]; // measurement(s)
	float norm;

	// remove bias
	accX += UKF_ACC_BIAS_X;
	accY += UKF_ACC_BIAS_Y;
	accZ += UKF_ACC_BIAS_Z;

	// normalize vector
	norm = sqrtf(accX * accX + accY * accY + accZ * accZ);
	y[0] = accX / norm;
	y[1] = accY / norm;
	y[2] = accZ / norm;

	noise[0] = p[UKF_ACC_N] + fabsf(GRAVITY - norm) * p[UKF_DIST_N];
	/*if (!(supervisorData.state & STATE_FLYING))
	{
		accX -= UKF_ACC_BIAS_X;
		accY -= UKF_ACC_BIAS_Y;
		noise[0] *= 0.001f;
	}*/

	noise[1] = noise[0];
	noise[2] = noise[0];

	srcdkfMeasurementUpdate(navUkfData.kf, 0, y, 3, 3, noise, navUkfAccUpdate);
}

void navUkfAccUpdate(float *u, float *x, float *noise, float *y)
{
	navUkfRotateVectorByRevQuat(y, navUkfData.v0a, &x[12]);
	y[0] += noise[0];
	y[1] += noise[1];
	y[2] += noise[2];
}


/*
 * Magnometer measurement update
 */


/*Magnometer has come in, update the estimation (ATTITUDE CORRECTION: YAW)*/
void simDoMagUpdate(float magX, float magY, float magZ)
{
	float noise[3]; // measurement variance
	float y[3]; // measurement(s)
	float norm;

	noise[0] = p[UKF_MAG_N];
	//    if (!(supervisorData.state & STATE_FLYING))
	//	noise[0] *= 0.001f;
	//if (!(supervisorData.state & STATE_FLYING))
	//	noise[0] *= 0.001f;

	noise[1] = noise[0];
	noise[2] = noise[0];

	// normalize vector
	norm = 1.0f / sqrtf(magX * magX + magY * magY + magZ * magZ);
	y[0] = magX * norm;
	y[1] = magY * norm;
	y[2] = magZ * norm;

	srcdkfMeasurementUpdate(navUkfData.kf, 0, y, 3, 3, noise, navUkfMagUpdate);
}
void navUkfMagUpdate(float *u, float *x, float *noise, float *y)
{
	navUkfRotateVectorByRevQuat(y, navUkfData.v0m, &x[12]);
	y[0] += noise[0];
	y[1] += noise[1];
	y[2] += noise[2];
}

//
//
///*
// * GPS Position measurement update
// */
//
//
///*We currently have actaul GPS data, update the state estimation based on GPS data*/
//void navUkfGpsPosUpate(uint32_t gpsMicros, double lat, double lon, float alt, float hAcc, float vAcc)
//{
//	float y[3];
//	float noise[3];
//	float posDelta[3];
//	int histIndex;
//
//	if (hAcc < 4.0f && gpsData.tDOP != 0.0f)
//	{
//		if (navUkfData.holdLat == 0.0)
//		{
//			navUkfData.holdLat = lat;
//			navUkfData.holdLon = lon;
//			navUkfCalcEarthRadius(lat);
//			navUkfSetGlobalPositionTarget(lat, lon);
//			navUkfResetPosition(-UKF_POSN, -UKF_POSE, alt - UKF_POSD);
//		}
//		else
//		{
//			navUkfCalcDistance(lat, lon, &y[0], &y[1]);
//			y[2] = alt;
//
//			// determine how far back this GPS position update came from
//			histIndex = (timerMicros() - (gpsMicros + p[UKF_POS_DELAY])) / (int) (1e6f * AQ_TIMESTEP);
//			histIndex = navUkfData.navHistIndex - histIndex;
//			if (histIndex < 0)
//				histIndex += UKF_HIST;
//			if (histIndex < 0 || histIndex >= UKF_HIST)
//				histIndex = 0;
//
//			// calculate delta from current position
//			posDelta[0] = UKF_POSN - navUkfData.posN[histIndex];
//			posDelta[1] = UKF_POSE - navUkfData.posE[histIndex];
//			posDelta[2] = UKF_POSD - navUkfData.posD[histIndex];
//
//			// set current position state to historic data
//			UKF_POSN = navUkfData.posN[histIndex];
//			UKF_POSE = navUkfData.posE[histIndex];
//			UKF_POSD = navUkfData.posD[histIndex];
//
//			noise[0] = p[UKF_GPS_POS_N] + hAcc * sqrtf(gpsData.tDOP * gpsData.tDOP + gpsData.nDOP * gpsData.nDOP) * p[UKF_GPS_POS_M_N];
//			noise[1] = p[UKF_GPS_POS_N] + hAcc * sqrtf(gpsData.tDOP * gpsData.tDOP + gpsData.eDOP * gpsData.eDOP) * p[UKF_GPS_POS_M_N];
//			noise[2] = p[UKF_GPS_ALT_N] + vAcc * sqrtf(gpsData.tDOP * gpsData.tDOP + gpsData.vDOP * gpsData.vDOP) * p[UKF_GPS_ALT_M_N];
//
//			srcdkfMeasurementUpdate(navUkfData.kf, 0, y, 3, 3, noise, navUkfPosUpdate);
//
//			// add the historic position delta back to the current state
//			UKF_POSN += posDelta[0];
//			UKF_POSE += posDelta[1];
//			UKF_POSD += posDelta[2];
//
//		}
//	}
//	else
//	{
//		y[0] = 0.0f;
//		y[1] = 0.0f;
//		y[2] = UKF_PRES_ALT;
//
//		if (supervisorData.state & STATE_FLYING)
//		{
//			noise[0] = 1e1f;
//			noise[1] = 1e1f;
//			noise[2] = 1e2f;
//		}
//		else
//		{
//			noise[0] = 1e-7f;
//			noise[1] = 1e-7f;
//			noise[2] = 1e2f;
//		}
//
//		srcdkfMeasurementUpdate(navUkfData.kf, 0, y, 3, 3, noise, navUkfPosUpdate);
//	}
//}
//
///*Function to get position from sigma pts*/
//void navUkfPosUpdate(float *u, float *x, float *noise, float *y)
//{
//	y[0] = x[3] + noise[0]; // return position
//	y[1] = x[4] + noise[1];
//	y[2] = x[5] + noise[2];
//}
//
//
//
///*
// * GPS Velocity measurement update
// */
//
//
///*Updates the velocity of the system based on GPS's velocity measurement.*/
//void navUkfGpsVelUpate(uint32_t gpsMicros, float velN, float velE, float velD, float sAcc)
//{
//	float y[3];
//	float noise[3];
//	float velDelta[3];
//	int histIndex;
//
//	if (sAcc < 2.0f && gpsData.tDOP != 0.0f)
//	{
//		y[0] = velN;
//		y[1] = velE;
//		y[2] = velD;
//
//		// determine how far back this GPS velocity update came from
//		histIndex = (timerMicros() - (gpsMicros + p[UKF_VEL_DELAY])) / (int) (1e6f * AQ_TIMESTEP);
//		histIndex = navUkfData.navHistIndex - histIndex;
//		if (histIndex < 0)
//			histIndex += UKF_HIST;
//		if (histIndex < 0 || histIndex >= UKF_HIST)
//			histIndex = 0;
//
//		// calculate delta from current position
//		velDelta[0] = UKF_VELN - navUkfData.velN[histIndex];
//		velDelta[1] = UKF_VELE - navUkfData.velE[histIndex];
//		velDelta[2] = UKF_VELD - navUkfData.velD[histIndex];
//
//		// set current position state to historic data
//		UKF_VELN = navUkfData.velN[histIndex];
//		UKF_VELE = navUkfData.velE[histIndex];
//		UKF_VELD = navUkfData.velD[histIndex];
//
//		noise[0] = p[UKF_GPS_VEL_N] + sAcc * sqrtf(gpsData.tDOP * gpsData.tDOP + gpsData.nDOP * gpsData.nDOP) * p[UKF_GPS_VEL_M_N];
//		noise[1] = p[UKF_GPS_VEL_N] + sAcc * sqrtf(gpsData.tDOP * gpsData.tDOP + gpsData.eDOP * gpsData.eDOP) * p[UKF_GPS_VEL_M_N];
//		noise[2] = p[UKF_GPS_VD_N] + sAcc * sqrtf(gpsData.tDOP * gpsData.tDOP + gpsData.vDOP * gpsData.vDOP) * p[UKF_GPS_VD_M_N];
//
//		srcdkfMeasurementUpdate(navUkfData.kf, 0, y, 3, 3, noise, navUkfVelUpdate);
//
//		// add the historic position delta back to the current state
//		UKF_VELN += velDelta[0];
//		UKF_VELE += velDelta[1];
//		UKF_VELD += velDelta[2];
//
//	}
//	else
//	{
//
//		y[0] = 0.0f;
//		y[1] = 0.0f;
//		y[2] = 0.0f;
//
//		if (supervisorData.state & STATE_FLYING)
//		{
//			noise[0] = 5.0f;
//			noise[1] = 5.0f;
//			noise[2] = 2.0f;
//		}
//		else
//		{
//			noise[0] = 1e-7;
//			noise[1] = 1e-7;
//			noise[2] = 1e-7;
//		}
//
//		srcdkfMeasurementUpdate(navUkfData.kf, 0, y, 3, 3, noise, navUkfVelUpdate);
//	}
//}
//
///*Function to get velocity from sigma pts*/
//void navUkfVelUpdate(float *u, float *x, float *noise, float *y)
//{
//	y[0] = x[0] + noise[0]; // return velocity
//	y[1] = x[1] + noise[1];
//	y[2] = x[2] + noise[2];
//}
