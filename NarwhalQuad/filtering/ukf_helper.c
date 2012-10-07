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

/*Convert pressure to altitude*/
float navUkfPresToAlt(float pressure)
{
	return (1.0f - powf(pressure / UKF_P0, 0.19f)) * (1.0f / 22.558e-6f);
}

// reset current sea level static pressure based on better GPS estimate
void UKFPressureAdjust(float altitude)
{
	navUkfData.presAltOffset = altitude - UKF_PRES_ALT;
}

/*get the current radius of earth based on latitude*/
void navUkfCalcEarthRadius(double lat)
{
	double sinLat2;

	sinLat2 = sin(lat * (double) DEG2RAD);
	sinLat2 = sinLat2 * sinLat2;

	navUkfData.r1 = (double) NAV_EQUATORIAL_RADIUS * (double) DEG2RAD * ((double) 1.0 - (double) NAV_E_2) / pow((double) 1.0 - ((double) NAV_E_2 * sinLat2), ((double) 3.0 / (double) 2.0));
	navUkfData.r2 = (double) NAV_EQUATORIAL_RADIUS * (double) DEG2RAD / sqrt((double) 1.0 - ((double) NAV_E_2 * sinLat2)) * cos(lat * (double) DEG2RAD);
}

void navUkfCalcDistance(double lat, double lon, float *posNorth, float *posEast)
{
	*posNorth = (lat - navUkfData.holdLat) * navUkfData.r1;
	*posEast = (lon - navUkfData.holdLon) * navUkfData.r2;
}

void navUkfResetPosition(float deltaN, float deltaE, float deltaD)
{
	int i;

	for (i = 0; i < UKF_HIST; i++)
	{
		navUkfData.posN[i] += deltaN;
		navUkfData.posE[i] += deltaE;
		navUkfData.posD[i] += deltaD;
	}

	UKF_POSN += deltaN;
	UKF_POSE += deltaE;
	UKF_POSD += deltaD;

	//navResetHoldAlt(deltaD);
}

void navUkfSetGlobalPositionTarget(double lat, double lon)
{
	float oldPosN, oldPosE;
	float newPosN, newPosE;

	navUkfCalcDistance(lat, lon, &oldPosN, &oldPosE);

	navUkfData.holdLat = lat;
	navUkfData.holdLon = lon;

	navUkfCalcDistance(lat, lon, &newPosN, &newPosE);

	navUkfResetPosition(newPosN - oldPosN, newPosE - oldPosE, 0.0f);
}

void navUkfNormalizeVec3(float *vr, float *v)
{
	float norm;

	norm = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);

	vr[0] = v[0] / norm;
	vr[1] = v[1] / norm;
	vr[2] = v[2] / norm;
}

void navUkfNormalizeQuat(float *qr, float *q)
{
	float norm;

	norm = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);

	qr[0] = q[0] / norm;
	qr[1] = q[1] / norm;
	qr[2] = q[2] / norm;
	qr[3] = q[3] / norm;
}

void crossVector3(float *vr, float *va, float *vb)
{
	vr[0] = va[1] * vb[2] - vb[1] * va[2];
	vr[1] = va[2] * vb[0] - vb[2] * va[0];
	vr[2] = va[0] * vb[1] - vb[0] * va[1];
}

float dotVector3(float *va, float *vb)
{
	return va[0] * vb[0] + va[1] * vb[1] + va[2] * vb[2];
}

void navUkfRotateVectorByQuat(float *vr, float *v, float *q)
{
	float w, x, y, z;

	w = q[0];
	x = q[1];
	y = q[2];
	z = q[3];

	vr[0] = w * w * v[0] + 2.0 * y * w * v[2] - 2.0 * z * w * v[1] + x * x * v[0] + 2.0 * y * x * v[1] + 2.0 * z * x * v[2] - z * z * v[0] - y * y * v[0];
	vr[1] = 2.0 * x * y * v[0] + y * y * v[1] + 2.0 * z * y * v[2] + 2.0 * w * z * v[0] - z * z * v[1] + w * w * v[1] - 2.0 * x * w * v[2] - x * x * v[1];
	vr[2] = 2.0 * x * z * v[0] + 2.0 * y * z * v[1] + z * z * v[2] - 2.0 * w * y * v[0] - y * y * v[2] + 2.0 * w * x * v[1] - x * x * v[2] + w * w * v[2];
}

void navUkfRotateVectorByRevQuat(float *vr, float *v, float *q)
{
	float qc[4];

	qc[0] = q[0];
	qc[1] = -q[1];
	qc[2] = -q[2];
	qc[3] = -q[3];

	navUkfRotateVectorByQuat(vr, v, qc);
}

void navUkfRotateVecByMatrix(float *vr, float *v, float *m)
{
	vr[0] = m[0 * 3 + 0] * v[0] + m[0 * 3 + 1] * v[1] + m[0 * 3 + 2] * v[2];
	vr[1] = m[1 * 3 + 0] * v[0] + m[1 * 3 + 1] * v[1] + m[1 * 3 + 2] * v[2];
	vr[2] = m[2 * 3 + 0] * v[0] + m[2 * 3 + 1] * v[1] + m[2 * 3 + 2] * v[2];
}

void navUkfRotateVecByRevMatrix(float *vr, float *v, float *m)
{
	vr[0] = m[0 * 3 + 0] * v[0] + m[1 * 3 + 0] * v[1] + m[2 * 3 + 0] * v[2];
	vr[1] = m[0 * 3 + 1] * v[0] + m[1 * 3 + 1] * v[1] + m[2 * 3 + 1] * v[2];
	vr[2] = m[0 * 3 + 2] * v[0] + m[1 * 3 + 2] * v[1] + m[2 * 3 + 2] * v[2];
}

void navUkfQuatToMatrix(float *m, float *q, int normalize)
{
	float sqw = q[0] * q[0];
	float sqx = q[1] * q[1];
	float sqy = q[2] * q[2];
	float sqz = q[3] * q[3];
	float tmp1, tmp2;
	float invs;

	// get the invert square length
	if (normalize)
		invs = 1.0f / (sqx + sqy + sqz + sqw);
	else
		invs = 1.0f;

	// rotation matrix is scaled by inverse square length
	m[0 * 3 + 0] = (sqx - sqy - sqz + sqw) * invs;
	m[1 * 3 + 1] = (-sqx + sqy - sqz + sqw) * invs;
	m[2 * 3 + 2] = (-sqx - sqy + sqz + sqw) * invs;

	tmp1 = q[1] * q[2];
	tmp2 = q[3] * q[0];
	m[1 * 3 + 0] = 2.0 * (tmp1 + tmp2) * invs;
	m[0 * 3 + 1] = 2.0 * (tmp1 - tmp2) * invs;

	tmp1 = q[1] * q[3];
	tmp2 = q[2] * q[0];
	m[2 * 3 + 0] = 2.0 * (tmp1 - tmp2) * invs;
	m[0 * 3 + 2] = 2.0 * (tmp1 + tmp2) * invs;

	tmp1 = q[2] * q[3];
	tmp2 = q[1] * q[0];
	m[2 * 3 + 1] = 2.0 * (tmp1 + tmp2) * invs;
	m[1 * 3 + 2] = 2.0 * (tmp1 - tmp2) * invs;
}

void navUkfMatrixExtractEuler(float *m, float *yaw, float *pitch, float *roll)
{
	if (m[1 * 3 + 0] > 0.998f)
	{ // singularity at north pole
		*pitch = atan2f(m[0 * 3 + 2], m[2 * 3 + 2]);
		*yaw = MATH_PI / 2.0f;
		*roll = 0.0f;
	}
	else if (m[1 * 3 + 0] < -0.998f)
	{ // singularity at south pole
		*pitch = atan2f(m[0 * 3 + 2], m[2 * 3 + 2]);
		*yaw = -MATH_PI / 2.0f;
		*roll = 0.0f;
	}
	else
	{
		*pitch = atan2f(-m[2 * 3 + 0], m[0 * 3 + 0]);
		*yaw = asinf(m[1 * 3 + 0]);
		*roll = atan2f(-m[1 * 3 + 2], m[1 * 3 + 1]);
	}
}

void navUkfQuatExtractEuler(float *q, float *yaw, float *pitch, float *roll)
{
	float q0, q1, q2, q3;

	q0 = q[1];
	q1 = q[2];
	q2 = q[3];
	q3 = q[0];

	*yaw = atan2f((2.0f * (q0 * q1 + q3 * q2)), (q3 * q3 - q2 * q2 - q1 * q1 + q0 * q0));
	*pitch = asinf(-2.0f * (q0 * q2 - q1 * q3));
	*roll = atanf((2.0f * (q1 * q2 + q0 * q3)) / (q3 * q3 + q2 * q2 - q1 * q1 - q0 * q0));
}

// result and source can be the same
void navUkfRotateQuat(float *qr, float *q, float *rate, float dt)
{
	float q1[4];
	float s, t, lg;
	float qMag;

	s = sqrtf(rate[0] * rate[0] + rate[1] * rate[1] + rate[2] * rate[2]) * 0.5f;
	t = -(0.5f * sinf(s) / s);
	rate[0] *= t;
	rate[1] *= t;
	rate[2] *= t;

	// create Lagrange factor to control quat's numerical integration errors
	qMag = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
	lg = cosf(s) + (1.0f - qMag * qMag) * dt * dt;

	// rotate
	q1[0] = q[0];
	q1[1] = q[1];
	q1[2] = q[2];
	q1[3] = q[3];

	qr[0] = lg * q1[0] + rate[0] * q1[1] + rate[1] * q1[2] + rate[2] * q1[3];
	qr[1] = -rate[0] * q1[0] + lg * q1[1] - rate[2] * q1[2] + rate[1] * q1[3];
	qr[2] = -rate[1] * q1[0] + rate[2] * q1[1] + lg * q1[2] - rate[0] * q1[3];
	qr[3] = -rate[2] * q1[0] - rate[1] * q1[1] + rate[0] * q1[2] + lg * q1[3];
}
