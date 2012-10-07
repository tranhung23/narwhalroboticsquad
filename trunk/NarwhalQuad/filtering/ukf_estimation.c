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

void navUkfTimeUpdate(float *in, float *noise, float *out, float *u, float dt)
{
	float tmp[3], acc[3];
	float rate[3];
	float mat3x3[3 * 3];

	// acc bias
	out[6] = in[6] + noise[0] * dt;
	out[7] = in[7] + noise[1] * dt;
	out[8] = in[8] + noise[2] * dt;

	// gbias
	out[9] = in[9] + noise[3] * dt;
	out[10] = in[10] + noise[4] * dt;
	out[11] = in[11] + noise[5] * dt;

	// rate = rate + bias + noise
	rate[0] = (u[3] + out[9] + noise[6]) * dt;
	rate[1] = (u[4] + out[10] + noise[7]) * dt;
	rate[2] = (u[5] + out[11] + noise[8]) * dt;

	// rotate
	navUkfRotateQuat(&out[12], &in[12], rate, dt);
	navUkfQuatToMatrix(mat3x3, &out[12], 1);

	// acc
	tmp[0] = u[0] + out[6];
	tmp[1] = u[1] + out[7];
	tmp[2] = u[2] + out[8];

	// rotate acc to world frame
	navUkfRotateVecByMatrix(acc, tmp, mat3x3);
	acc[2] += GRAVITY;

	// vel
	out[0] = in[0] + acc[0] * dt + noise[10];
	out[1] = in[1] + acc[1] * dt + noise[11];
	out[2] = in[2] + acc[2] * dt + noise[12];

	// pos
	out[3] = in[3] + (in[0] + out[0]) * 0.5f * dt + noise[13];
	out[4] = in[4] + (in[1] + out[1]) * 0.5f * dt + noise[14];
	out[5] = in[5] - (in[2] + out[2]) * 0.5f * dt + noise[15];

	// pres alt
	out[16] = in[16] - (in[2] + out[2]) * 0.5f * dt + noise[9];
}

void navUkfInertialUpdate(void)
{
	float u[6];

	u[0] = IMU_ACCX;
	u[1] = IMU_ACCY;
	u[2] = IMU_ACCZ;

	u[3] = IMU_RATEX;
	u[4] = IMU_RATEY;
	u[5] = IMU_RATEZ;

	srcdkfTimeUpdate(navUkfData.kf, u, IMU_TIMESTEP);

	// store history
	navUkfData.posN[navUkfData.navHistIndex] = UKF_POSN;
	navUkfData.posE[navUkfData.navHistIndex] = UKF_POSE;
	navUkfData.posD[navUkfData.navHistIndex] = UKF_POSD;

	navUkfData.velN[navUkfData.navHistIndex] = UKF_VELN;
	navUkfData.velE[navUkfData.navHistIndex] = UKF_VELE;
	navUkfData.velD[navUkfData.navHistIndex] = UKF_VELD;

	navUkfData.navHistIndex = (navUkfData.navHistIndex + 1) % UKF_HIST;
}

