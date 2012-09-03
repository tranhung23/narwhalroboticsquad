/*
 * ekf.c
 *
 * Created: 9/17/2011 2:09:46 PM
 *  Author: GrubyGrub
 */

#include <math.h>
#include "ekf.h"
#include "matrix_math.h"
#include "../quad_lib/stm32f4_quad.h"

/*acc reference vectors*/
static float acc_ref_x;
static float acc_ref_y;
static float acc_ref_z;
static float acc_ref[3];
static float acc_frame[3];

/*mag reference vectors*/
static float mag_ref_x;
static float mag_ref_y;
static float mag_ref_z;
static float mag_ref[3];
static float mag_frame[3];

/*gyro bias*/
static float gyro_bias_x;
static float gyro_bias_y;
static float gyro_bias_z;

/*mag bias*/
static float mag_bias_x;
static float mag_bias_y;
static float mag_bias_z;

/* mag calibration matrix (may not be exactly the same
 MagCMatrix = [xx xy xz;
 yx yy yz;
 zx zy zz]
 */
static float mag_cali_xx;
static float mag_cali_xy;
static float mag_cali_xz;

static float mag_cali_yx;
static float mag_cali_yy;
static float mag_cali_yz;

static float mag_cali_zx;
static float mag_cali_zy;
static float mag_cali_zz;

/*predicted estimate covariance*/
static float P[7][7];

/*Innovation covariance*/
static float Q[7][7];

/*State Jacobian matrix*/
static float Fj[7][7];

/*swaps*/
static float swap1[7][7];
static float swap2[7][7];

/*acclermeter covariance*/
static float Racc[3][3];

/*magnetomer covariance*/
static float Rmag[3][3];

/*the kalman state vector*/
static float state_q0;
static float state_q1;
static float state_q2;
static float state_q3;
static float state_gb0;
static float state_gb1;
static float state_gb2;

/*magnetomer reference rotation matrix*/
static float MagRefM[3][3];
static float MagFrameM[3][3];

/*state vector*/
static float state[7];

void ekf_init(void) {
	/*set the state vector*/
	state_q0 = 1.00;
	state_q1 = state_q2 = state_q3 = state_gb0 = state_gb1 = state_gb2 = 0.00;

	/*set gyro bias*/
	gyro_bias_x = GYRO_BX;
	gyro_bias_y = GYRO_BY;
	gyro_bias_z = GYRO_BZ;

	/*set magnetomer bias*/
	mag_bias_x = MAG_BX;
	mag_bias_y = MAG_BY;
	mag_bias_z = MAG_BZ;

	/*set mag cali matrix*/
	mag_cali_xx = MAG_CXX;
	mag_cali_xy = MAG_CXY;
	mag_cali_xz = MAG_CXZ;

	mag_cali_yx = MAG_CYX;
	mag_cali_yy = MAG_CYY;
	mag_cali_yz = MAG_CYZ;

	mag_cali_zx = MAG_CZX;
	mag_cali_zy = MAG_CZY;
	mag_cali_zz = MAG_CZZ;

	/*set Acc Ref vectors, default vector when device is sitting still on level platform*/
	acc_ref[0] = acc_ref_x = ACC_RX;
	acc_ref[1] = acc_ref_y = ACC_RY;
	acc_ref[2] = acc_ref_z = ACC_RZ;

	/*set Acc Ref vectors, default vector when device is sitting still on level platform*/
	mag_ref[0] = mag_ref_x = MAG_RX;
	mag_ref[1] = mag_ref_y = MAG_RY;
	mag_ref[2] = mag_ref_z = MAG_RZ;

	/*init predicted estimate covariance*/
	Minit((float*) P, 7, 7);
	P[4][4] = P_COV;
	P[5][5] = P_COV;
	P[6][6] = P_COV;

	/*init Innovation (or residual) covariance*/
	Minit((float*) Q, 7, 7);
	Q[0][0] = Q_COV;
	Q[1][1] = Q_COV;
	Q[2][2] = Q_COV;
	Q[3][3] = Q_COV;
	Q[4][4] = Q_COV;
	Q[5][5] = Q_COV;
	Q[6][6] = Q_COV;

	/*init Acc covariance matrix*/
	Minit((float*) Racc, 3, 3);
	Racc[0][0] = ACC_COV;
	Racc[1][1] = ACC_COV;
	Racc[2][2] = ACC_COV;

	/*init mag covariance matrix*/
	Minit((float*) Rmag, 3, 3);
	Rmag[0][0] = MAG_COV;
	Rmag[1][1] = MAG_COV;
	Rmag[2][2] = MAG_COV;

	/*jacobian matrix of F*/
	Minit((float*) Fj, 7, 7);
	Fj[4][4] = 1;
	Fj[5][5] = 1;
	Fj[6][6] = 1;

	process_magaccvector();
	//READ ALL RAM VARIABLES!
}

void process_magaccvector(void) {

	float MagCrossAcc[3];
	float MagCross[3];

	/*magnetomer rotational reference matrix*/
	/*
	 sh1 = ae;
	 sh2 = cross(ae,me)/norm(cross(ae,me));
	 sh3 = cross(sh1, sh2);*/

	acc_frame[0] = acc_ref_x;
	acc_frame[1] = acc_ref_y;
	acc_frame[2] = acc_ref_z;

	MagRefM[0][0] = acc_ref_x;
	MagRefM[0][1] = acc_ref_y;
	MagRefM[0][2] = acc_ref_z;

	VectorCross(acc_ref, mag_ref, MagCrossAcc);

	float MagCrossAccNorm = sqrt(
			MagCrossAcc[0] * MagCrossAcc[0] + MagCrossAcc[1] * MagCrossAcc[1]
					+ MagCrossAcc[2] * MagCrossAcc[2]);

	MagCrossAcc[0] = MagCrossAcc[0] / MagCrossAccNorm;
	MagCrossAcc[1] = MagCrossAcc[1] / MagCrossAccNorm;
	MagCrossAcc[2] = MagCrossAcc[2] / MagCrossAccNorm;

	MagRefM[1][0] = MagCrossAcc[0];
	MagRefM[1][1] = MagCrossAcc[1];
	MagRefM[1][2] = MagCrossAcc[2];

	VectorCross(acc_ref, MagCrossAcc, MagCross);
	MagRefM[2][0] = MagCross[0];
	MagRefM[2][1] = MagCross[1];
	MagRefM[2][2] = MagCross[2];

}

void process_gyro(float gx, float gy, float gz, float dT) {
	float quatnorm;

	float halfdt = 0.5 * dT;
	float neghalfdt = -0.5 * dT;

	/*
	 correct gyro with reference (need temp correction here), multiply by
	 step to get degrees, and then multiply by rad
	 Gyro = ([w(4); w(5); w(6)] - gyrocali500) * 17.50/1000 * deg2rad;
	 */
	gx = ((gx - gyro_bias_x + state_gb0) * GYRO_DEGLSB) * DEG2RAD;
	gy = ((gy - gyro_bias_y + state_gb1) * GYRO_DEGLSB) * DEG2RAD;
	gz = ((gz - gyro_bias_z + state_gb2) * GYRO_DEGLSB) * DEG2RAD;
	/*add gyro rotation into quaternion*/
	state_q0 = state_q0
			+ ((.5) * (-gx * state_q1 - gy * state_q2 - gz * state_q3) * dT);
	state_q1 = state_q1
			+ ((.5) * (gx * state_q0 - gy * state_q3 + gz * state_q2) * dT);
	state_q2 = state_q2
			+ ((.5) * (gx * state_q3 + gy * state_q0 - gz * state_q1) * dT);
	state_q3 = state_q3
			+ ((.5) * (-gx * state_q2 + gy * state_q1 + gz * state_q0) * dT);
	state_gb0 = state_gb0 + 0;
	state_gb1 = state_gb1 + 0;
	state_gb2 = state_gb2 + 0;

	/*normlize quaternions*/
	quatnorm = sqrtf(
			(state_q0 * state_q0) + (state_q1 * state_q1)
					+ (state_q2 * state_q2) + (state_q3 * state_q3));
	state_q0 = state_q0 / quatnorm;
	state_q1 = state_q1 / quatnorm;
	state_q2 = state_q2 / quatnorm;
	state_q3 = state_q3 / quatnorm;

	/*row major order, this matrix code is NOT OPTIMIZED, needs optimization..*/
	Fj[0][0] = 1;
	Fj[0][1] = gx * neghalfdt;
	Fj[0][2] = gy * neghalfdt;
	Fj[0][3] = gz * neghalfdt;
	Fj[0][4] = state_q1 * halfdt;
	Fj[0][5] = state_q2 * halfdt;
	Fj[0][6] = state_q3 * halfdt;

	Fj[1][0] = gx * halfdt;
	Fj[1][1] = 1;
	Fj[1][2] = gz * halfdt;
	Fj[1][3] = gy * neghalfdt;
	Fj[1][4] = state_q0 * neghalfdt;
	Fj[1][5] = state_q3 * halfdt;
	Fj[1][6] = state_q2 * neghalfdt;

	Fj[2][0] = gy * halfdt;
	Fj[2][1] = gz * neghalfdt;
	Fj[2][2] = 1;
	Fj[2][3] = gx * halfdt;
	Fj[2][4] = state_q3 * halfdt;
	Fj[2][5] = state_q0 * neghalfdt;
	Fj[2][6] = state_q1 * halfdt;

	Fj[3][0] = gz * halfdt;
	Fj[3][1] = gy * neghalfdt;
	Fj[3][2] = gx * neghalfdt;
	Fj[3][3] = 1;
	Fj[3][4] = state_q2 * halfdt;
	Fj[3][5] = state_q1 * neghalfdt;
	Fj[3][6] = state_q0 * neghalfdt;

	MMmult((float*) Fj, (float*) P, (float*) swap1, 7, 7, 7);
	transpose((float*) Fj, (float*) swap2, 7, 7);
	MMmult((float*) swap1, (float*) swap2, (float*) P, 7, 7, 7);
	MMadd((float*) P, (float*) Q, (float*) P, 7, 7);

}

void process_mag(float mx, float my, float mz) {
	float mag_norm;
	float MagCrossAccNorm;
	float mag_triad[3][3];

	float Zmagx;
	float Zmagy;
	float Zmagz;

	float Epr[3];
	float Hpr[3][7];
	float Ppr[3][7];
	float Kpr[3][3];

	float AccCrossMag[3];
	float AccCrossAcM[3];

	float t_x, t_y, t_z;

	/*calibrate the magnetometer
	 MagCalibrate = MagCMatrix * (Mag - MagMMatrix);
	 MagNorm = (MagCalibrate/norm(MagCalibrate));*/
	mx = mx - mag_bias_x;
	my = my - mag_bias_y;
	mz = mz - mag_bias_z;

	//t_x = mz;
	//t_y = -mx;
	//t_z = my;

	t_x = (mag_cali_xx * mx) + (mag_cali_yx * my) + (mag_cali_zx * mz);
	t_y = (mag_cali_xy * mx) + (mag_cali_yy * my) + (mag_cali_zy * mz);
	t_z = (mag_cali_xz * mx) + (mag_cali_yz * my) + (mag_cali_zz * mz);

	mx = t_x;
	my = t_y;
	mz = t_z;

	/*normlize the magnetomer vector*/
	mag_norm = sqrt(mx * mx + my * my + mz * mz);
	mag_frame[0] = mx = mx / mag_norm;
	mag_frame[1] = my = my / mag_norm;
	mag_frame[2] = mz = mz / mag_norm;

	/*  r1 = AccNorm;
	 r2 = cross(AccNorm,MagNorm)/norm(cross(AccNorm,MagNorm));
	 r3 = cross(r1, r2);

	 mm = [r1,r2,r3];
	 mr = [sh1,sh2, sh3];
	 ma = (mm*mr');*/

	/*transpose matrix on the fly, TRIAD algo to get mag direction*/
	/*r1'*/

	acc_frame[0] = 2 * (state_q1 * state_q3 - state_q0 * state_q2);
	acc_frame[1] = 2 * (state_q2 * state_q3 + state_q0 * state_q1);
	acc_frame[2] = 1 - 2 * (state_q1 * state_q1 + state_q2 * state_q2);

	VectorCross(acc_frame, mag_frame, AccCrossMag);

	MagCrossAccNorm = sqrt(
			AccCrossMag[0] * AccCrossMag[0] + AccCrossMag[1] * AccCrossMag[1]
					+ AccCrossMag[2] * AccCrossMag[2]);

	/*r2'*/
	AccCrossMag[0] = MagFrameM[0][1] = AccCrossMag[0] / MagCrossAccNorm;
	AccCrossMag[1] = MagFrameM[1][1] = AccCrossMag[1] / MagCrossAccNorm;
	AccCrossMag[2] = MagFrameM[2][1] = AccCrossMag[2] / MagCrossAccNorm;

	VectorCross(acc_frame, AccCrossMag, AccCrossAcM);

	/*r3'*/
	MagFrameM[0][2] = AccCrossAcM[0];
	MagFrameM[1][2] = AccCrossAcM[1];
	MagFrameM[2][2] = AccCrossAcM[2];
	/*triad method*/
	MMmult((float*) MagFrameM, (float*) MagRefM, (float*) mag_triad, 3, 3, 3);

	//PrintMatrix(mag_triad,3,3);

	Zmagx = 1 - 2 * (state_q2 * state_q2 + state_q3 * state_q3);
	Zmagy = 2 * (state_q1 * state_q2 - state_q0 * state_q3);
	Zmagz = 2 * (state_q0 * state_q2 + state_q1 * state_q3);

	/*yaw estimation error*/
	Epr[0] = mag_triad[0][0] - Zmagx;
	Epr[1] = mag_triad[1][0] - Zmagy;
	Epr[2] = mag_triad[2][0] - Zmagz;

	Hpr[0][0] = 0;
	Hpr[0][1] = 0;
	Hpr[0][2] = -4 * state_q2;
	Hpr[0][3] = -4 * state_q3;
	Hpr[0][4] = 0;
	Hpr[0][5] = 0;
	Hpr[0][6] = 0;

	Hpr[1][0] = -2 * state_q3;
	Hpr[1][1] = 2 * state_q2;
	Hpr[1][2] = 2 * state_q1;
	Hpr[1][3] = -2 * state_q0;
	Hpr[1][4] = 0;
	Hpr[1][5] = 0;
	Hpr[1][6] = 0;

	Hpr[2][0] = 2 * state_q2;
	Hpr[2][1] = 2 * state_q3;
	Hpr[2][2] = 2 * state_q0;
	Hpr[2][3] = 2 * state_q0;
	Hpr[2][4] = 0;
	Hpr[2][5] = 0;
	Hpr[2][6] = 0;
	//PrintMatrix(Hpr,7,3);

	// roll-pitch estimation error covariance matrix
	//Ppr = Hpr * P * (Hpr') + Racc;
	//swap1 = Hpr * P
	MMmult((float*) Hpr, (float*) P, (float*) swap1, 3, 7, 7);
	//swap2 = Hpr'
	transpose((float*) Hpr, (float*) swap2, 3, 7);
	//ppr = swap1*swap2
	MMmult((float*) swap1, (float*) swap2, (float*) Ppr, 3, 7, 3);
	//ppr + rcc
	MMadd((float*) Ppr, (float*) Racc, (float*) Ppr, 3, 3);

	// roll-pitch kalman gain
	//Kpr = P * (Hpr') / Ppr;
	//swap1 = p * swap2
	MMmult((float*) P, (float*) swap2, (float*) swap1, 7, 7, 3);
	//swap2 = inv(Ppr)
	inv((float*) Ppr, (float*) swap2, 3);
	//Kpr = swap1*swap2
	MMmult((float*) swap1, (float*) swap2, (float*) Kpr, 7, 3, 3);

	//update system state
	//state = state + Kpr*Epr;
	MMmult((float*) Kpr, (float*) Epr, (float*) swap1, 7, 3, 1);
	MVmult((float*) Kpr, (float*) Epr, state, 7, 3);

	state_q0 = state_q0 + state[0];
	state_q1 = state_q1 + state[1];
	state_q2 = state_q2 + state[2];
	state_q3 = state_q3 + state[3];
	state_gb0 = state_gb0 + state[4];
	state_gb1 = state_gb1 + state[5];
	state_gb2 = state_gb2 + state[6];

	//update system state covariance matrix
	//P = P - Kpr * Hpr * P;
	//swap1 = Kpr*Hpr
	MMmult((float*) Kpr, (float*) Hpr, (float*) swap1, 7, 3, 7);
	//swap2= swap1 * P
	MMmult((float*) swap1, (float*) P, (float*) swap2, 7, 7, 7);
	MMsub((float*) P, (float*) swap2, (float*) P, 7, 7);

}

void process_acc(float ax, float ay, float az) {
	float acc_norm;
	float Zex;
	float Zey;
	float Zez;

	float Epr[3];
	float Hpr[3][7];
	float Ppr[3][7];
	float Kpr[3][3];

	acc_norm = sqrt(ax * ax + ay * ay + az * az);
	acc_frame[0] = ax = ax / acc_norm;
	acc_frame[1] = ay = ay / acc_norm;
	acc_frame[2] = az = az / acc_norm;

	/* Ze = [2*(q1*q3 - q0*q2);
	 2*(q2*q3 + q0*q1);
	 1 - 2*(q1*q1 + q2*q2)];
	 */

	Zex = 2 * (state_q1 * state_q3 - state_q0 * state_q2);
	Zey = 2 * (state_q2 * state_q3 + state_q0 * state_q1);
	Zez = 1 - 2 * (state_q1 * state_q1 + state_q2 * state_q2);

	/*pitch roll estimation error*/
	Epr[0] = ax - Zex;
	Epr[1] = ay - Zey;
	Epr[2] = az - Zez;

	Hpr[0][0] = -2 * state_q2;
	Hpr[0][1] = 2 * state_q3;
	Hpr[0][2] = -2 * state_q0;
	Hpr[0][3] = 2 * state_q1;
	Hpr[0][4] = 0;
	Hpr[0][5] = 0;
	Hpr[0][6] = 0;

	Hpr[1][0] = 2 * state_q1;
	Hpr[1][1] = 2 * state_q0;
	Hpr[1][2] = 2 * state_q3;
	Hpr[1][3] = 2 * state_q2;
	Hpr[1][4] = 0;
	Hpr[1][5] = 0;
	Hpr[1][6] = 0;

	Hpr[2][0] = 0;
	Hpr[2][1] = -4 * state_q1;
	Hpr[2][2] = -4 * state_q2;
	Hpr[2][3] = 0;
	Hpr[2][4] = 0;
	Hpr[2][5] = 0;
	Hpr[2][6] = 0;
	//PrintMatrix(Hpr,7,3);

	// roll-pitch estimation error covariance matrix
	//Ppr = Hpr * P * (Hpr') + Racc;
	//swap1 = Hpr * P
	MMmult((float*) Hpr, (float*) P, (float*) swap1, 3, 7, 7);
	//swap2 = Hpr'
	transpose((float*) Hpr, (float*) swap2, 3, 7);
	//ppr = swap1*swap2
	MMmult((float*) swap1, (float*) swap2, (float*) Ppr, 3, 7, 3);
	//ppr + rcc
	MMadd((float*) Ppr, (float*) Racc, (float*) Ppr, 3, 3);

	// roll-pitch kalman gain
	//Kpr = P * (Hpr') / Ppr;
	//swap1 = p * swap2
	MMmult((float*) P, (float*) swap2, (float*) swap1, 7, 7, 3);
	//swap2 = inv(Ppr)
	inv((float*) Ppr, (float*) swap2, 3);
	//Kpr = swap1*swap2
	MMmult((float*) swap1, (float*) swap2, (float*) Kpr, 7, 3, 3);

	//update system state
	//state = state + Kpr*Epr;
	MMmult((float*) Kpr, (float*) Epr, (float*) swap1, 7, 3, 1);
	MVmult((float*) Kpr, (float*) Epr, state, 7, 3);

//PrintMatrix(swap1,7,7);

	state_q0 = state_q0 + state[0];
	state_q1 = state_q1 + state[1];
	state_q2 = state_q2 + state[2];
	state_q3 = state_q3 + state[3];
	state_gb0 = state_gb0 + state[4];
	state_gb1 = state_gb1 + state[5];
	state_gb2 = state_gb2 + state[6];

	//update system state covariance matrix
	//P = P - Kpr * Hpr * P;
	//swap1 = Kpr*Hpr
	MMmult((float*) Kpr, (float*) Hpr, (float*) swap1, 7, 3, 7);
	//swap2= swap1 * P
	MMmult((float*) swap1, (float*) P, (float*) swap2, 7, 7, 7);
	MMsub((float*) P, (float*) swap2, (float*) P, 7, 7);
}

void ekf_print(void) {
	async_printf("%f %f %f %f\r\n", state_q0, state_q1, state_q2, state_q3);
}
