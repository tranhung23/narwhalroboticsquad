/*
 * config_default.h
 *
 *  Created on: Sep 9, 2012
 *      Author: GrubyGrub
 */

#ifndef CONFIG_DEFAULT_H_
#define CONFIG_DEFAULT_H_

#define DEFAULT_CONFIG_VERSION      100
#define DEFAULT_DEG2RAD             0.0174532925
#define DEFAULT_RAD2DEG             57.2957796

#define DEFAULT_IMU_P_COV           0.5
#define DEFAULT_IMU_Q_COV           0.000001

#define DEFAULT_IMU_ACC_COV         0.6
#define DEFAULT_IMU_ACC_BIAS_X      -1.67505631840194
#define DEFAULT_IMU_ACC_BIAS_Y      -1.66038642251816
#define DEFAULT_IMU_ACC_BIAS_Z      -1.66738642251816
#define DEFAULT_IMU_ACC_REF_X       0
#define DEFAULT_IMU_ACC_REF_Y       0
#define DEFAULT_IMU_ACC_REF_Z       1

#define DEFAULT_IMU_MAG_COV         0.6
#define DEFAULT_IMU_MAG_BIAS_X      -33.4495
#define DEFAULT_IMU_MAG_BIAS_Y      287.7760
#define DEFAULT_IMU_MAG_BIAS_Z      -142.0700
#define DEFAULT_IMU_MAG_REF_X       -0.2525
#define DEFAULT_IMU_MAG_REF_Y       -0.9674
#define DEFAULT_IMU_MAG_REF_Z       0.0177
#define DEFAULT_IMU_MAG_CALI_XX     0.0
#define DEFAULT_IMU_MAG_CALI_XY     -1.0
#define DEFAULT_IMU_MAG_CALI_XZ     0.0
#define DEFAULT_IMU_MAG_CALI_YX     0.0
#define DEFAULT_IMU_MAG_CALI_YY     0.0
#define DEFAULT_IMU_MAG_CALI_YZ     1.0
#define DEFAULT_IMU_MAG_CALI_ZX     1.0
#define DEFAULT_IMU_MAG_CALI_ZY     0.0
#define DEFAULT_IMU_MAG_CALI_ZZ     0.0

#define DEFAULT_IMU_GYO_500DPS      17.50
#define DEFAULT_IMU_GYO_DEGLSB      500.0 /*2 mw for one degree*/
#define DEFAULT_IMU_GYO_BIAS_X      -1.37865199515738
#define DEFAULT_IMU_GYO_BIAS_Y      -1.35971325060533
#define DEFAULT_IMU_GYO_BIAS_Z      -1.36872717675545

#define DEFAULT_MOTOR_PITCH_LEFT_ADDR       0x52
#define DEFAULT_MOTOR_PITCH_RIGHT_ADDR      0x54
#define DEFAULT_MOTOR_ROLL_BACK_ADDR        0x56
#define DEFAULT_MOTOR_ROLL_FORWARD_ADDR     0x58
#define DEFAULT_MOTOR_CTRL_MIN              0
#define DEFAULT_MOTOR_CTRL_MAX              255

#define DEFAULT_PID_N               3
#define DEFAULT_PID_YAW_EPSILON     0.01
#define DEFAULT_PID_YAW_DT          0.01
#define DEFAULT_PID_YAW_MAX         4.0
#define DEFAULT_PID_YAW_MIN         -4.0
#define DEFAULT_PID_YAW_KP          0.1
#define DEFAULT_PID_YAW_KI          0.01
#define DEFAULT_PID_YAW_KD          0.005
#define DEFAULT_PID_PITCH_EPSILON   0.01
#define DEFAULT_PID_PITCH_DT        0.01
#define DEFAULT_PID_PITCH_MAX       4.0
#define DEFAULT_PID_PITCH_MIN       -4.0
#define DEFAULT_PID_PITCH_KP        0.1
#define DEFAULT_PID_PITCH_KI        0.01
#define DEFAULT_PID_PITCH_KD        0.005
#define DEFAULT_PID_ROLL_EPSILON    0.01
#define DEFAULT_PID_ROLL_DT         0.01
#define DEFAULT_PID_ROLL_MAX        4.0
#define DEFAULT_PID_ROLL_MIN        -4.0
#define DEFAULT_PID_ROLL_KP         0.1
#define DEFAULT_PID_ROLL_KI         0.01
#define DEFAULT_PID_ROLL_KD         0.005

#define DEFAULT_RC_CONTROL_N         4
/*Define the mins and max of the control stick*/
#define DEFAULT_RC_CONTROL_MIN       1000
#define DEFAULT_RC_CONTROL_MAX       2000
<<<<<<< .mine


#define DEFAULT_UKF_VEL_Q               +7.6020e-02     // +0.076019661680       0.000109304521 -0.001443424436
#define DEFAULT_UKF_VEL_ALT_Q           +1.4149e-01     // +0.141489724652       0.000109419473 +0.000987597731
#define DEFAULT_UKF_POS_Q               +6.0490e+03     // +6048.951523179588    0.000109199532 +97.58772834123110
#define DEFAULT_UKF_POS_ALT_Q           +4.5576e+03     // +4557.622475819297    0.000109580650 +19.44625975731340
#define DEFAULT_UKF_ACC_BIAS_Q          +9.3722e-04     // +0.000937220476       0.000109347614 +0.000009865862
#define DEFAULT_UKF_GYO_BIAS_Q          +4.6872e-02     // +0.046871534288       0.000109380732 -0.000123894440
#define DEFAULT_UKF_QUAT_Q              +7.3021e-04     // +0.000730213283       0.000109472899 +0.000000995669
#define DEFAULT_UKF_PRES_ALT_Q          +6.5172e+01     // +65.171935456104      0.000109418082 -0.2151891180844
#define DEFAULT_UKF_ACC_BIAS_V          +2.7535e-07     // +0.000000275353       0.000109561088 -0.000000004212
#define DEFAULT_UKF_GYO_BIAS_V          +8.2738e-07     // +0.000000827379       0.000107923369 +0.000000009107
#define DEFAULT_UKF_RATE_V              +6.0568e-05     // +0.000060568461       0.000109458065 +0.000000498081
#define DEFAULT_UKF_PRES_ALT_V          +1.0204e-04     // +0.000102039667       0.000109254406 -0.000002050090
#define DEFAULT_UKF_POS_V               +6.4505e-08     // +0.000000064505       0.000109587486 -0.000000000240
#define DEFAULT_UKF_VEL_V               +1.0980e-07     // +0.000000109802       0.000109537353 -0.000000000134
#define DEFAULT_UKF_ALT_POS_V           +5.3821e-09     // +0.000000005382       0.000109525531 +0.000000000093
#define DEFAULT_UKF_ALT_VEL_V           +2.8103e-07     // +0.000000281035       0.000109279082 +0.000000000639
#define DEFAULT_UKF_GPS_POS_N           +1.7620e-05     // +0.000017619672       0.000109467204 -0.000000022679
#define DEFAULT_UKF_GPS_POS_M_N         +4.7413e-05     // +0.000047413187       0.000108906551 -0.000000419440
#define DEFAULT_UKF_GPS_ALT_N           +7.6558e-05     // +0.000076558177       0.000109472033 -0.000000162714
#define DEFAULT_UKF_GPS_ALT_M_N         +3.8535e-05     // +0.000038534766       0.000109525552 +0.000000007101
#define DEFAULT_UKF_GPS_VEL_N           +4.6256e-02     // +0.046255979499       0.000109061365 +0.000395208418
#define DEFAULT_UKF_GPS_VEL_M_N         +1.2336e-02     // +0.012336395925       0.000109431436 +0.000140398236
#define DEFAULT_UKF_GPS_VD_N            +3.7820e+00     // +3.782028700864       0.000109323731 -0.028830318912
#define DEFAULT_UKF_GPS_VD_M_N          +1.5841e-02     // +0.015840936058       0.000109475273 -0.000030160915
#define DEFAULT_UKF_ALT_N               +1.7077e-01     // +0.170768080733       0.000109571562 +0.000084225765
#define DEFAULT_UKF_ACC_N               +9.5468e-05     // +0.000095468045       0.000109331710 -0.000000932407
#define DEFAULT_UKF_DIST_N              +1.8705e-02     // +0.018704747883       0.000109457024 +0.000038618360
#define DEFAULT_UKF_MAG_N               +3.8226e-01     // +0.382258731690       0.000109407461 +0.002851611558
#define DEFAULT_UKF_POS_DELAY           +2.0574e+03     // +2057.421963899194    0.001097611925 -48.37809534324900
#define DEFAULT_UKF_VEL_DELAY           -1.0373e+05     // -103727.997010331557  0.000109494449 -293.522967971236500
=======


#define DEFAULT_UKF_VEL_Q		+7.6020e-02	// +0.076019661680	 0.000109304521 -0.001443424436
#define DEFAULT_UKF_VEL_ALT_Q		+1.4149e-01	// +0.141489724652	 0.000109419473 +0.000987597731
#define DEFAULT_UKF_POS_Q		+6.0490e+03	// +6048.951523179588	 0.000109199532 +97.58772834123110
#define DEFAULT_UKF_POS_ALT_Q		+4.5576e+03	// +4557.622475819297	 0.000109580650 +19.44625975731340
#define DEFAULT_UKF_ACC_BIAS_Q		+9.3722e-04	// +0.000937220476	 0.000109347614 +0.000009865862
#define DEFAULT_UKF_GYO_BIAS_Q		+4.6872e-02	// +0.046871534288	 0.000109380732 -0.000123894440
#define DEFAULT_UKF_QUAT_Q		+7.3021e-04	// +0.000730213283	 0.000109472899 +0.000000995669
#define DEFAULT_UKF_PRES_ALT_Q		+6.5172e+01	// +65.171935456104	 0.000109418082 -0.2151891180844
#define DEFAULT_UKF_ACC_BIAS_V		+2.7535e-07	// +0.000000275353	 0.000109561088 -0.000000004212
#define DEFAULT_UKF_GYO_BIAS_V		+8.2738e-07	// +0.000000827379	 0.000107923369 +0.000000009107
#define DEFAULT_UKF_RATE_V		+6.0568e-05	// +0.000060568461	 0.000109458065 +0.000000498081
#define DEFAULT_UKF_PRES_ALT_V		+1.0204e-04	// +0.000102039667	 0.000109254406 -0.000002050090
#define DEFAULT_UKF_POS_V		+6.4505e-08	// +0.000000064505	 0.000109587486 -0.000000000240
#define DEFAULT_UKF_VEL_V		+1.0980e-07	// +0.000000109802	 0.000109537353 -0.000000000134
#define DEFAULT_UKF_ALT_POS_V		+5.3821e-09	// +0.000000005382	 0.000109525531 +0.000000000093
#define DEFAULT_UKF_ALT_VEL_V		+2.8103e-07	// +0.000000281035	 0.000109279082 +0.000000000639
#define DEFAULT_UKF_GPS_POS_N		+1.7620e-05	// +0.000017619672	 0.000109467204 -0.000000022679
#define DEFAULT_UKF_GPS_POS_M_N		+4.7413e-05	// +0.000047413187	 0.000108906551 -0.000000419440
#define DEFAULT_UKF_GPS_ALT_N		+7.6558e-05	// +0.000076558177	 0.000109472033 -0.000000162714
#define DEFAULT_UKF_GPS_ALT_M_N		+3.8535e-05	// +0.000038534766	 0.000109525552 +0.000000007101
#define DEFAULT_UKF_GPS_VEL_N		+4.6256e-02	// +0.046255979499	 0.000109061365 +0.000395208418
#define DEFAULT_UKF_GPS_VEL_M_N		+1.2336e-02	// +0.012336395925	 0.000109431436 +0.000140398236
#define DEFAULT_UKF_GPS_VD_N		+3.7820e+00	// +3.782028700864	 0.000109323731 -0.028830318912
#define DEFAULT_UKF_GPS_VD_M_N		+1.5841e-02	// +0.015840936058	 0.000109475273 -0.000030160915
#define DEFAULT_UKF_ALT_N		+1.7077e-01	// +0.170768080733	 0.000109571562 +0.000084225765
#define DEFAULT_UKF_ACC_N		+9.5468e-05	// +0.000095468045	 0.000109331710 -0.000000932407
#define DEFAULT_UKF_DIST_N		+1.8705e-02	// +0.018704747883	 0.000109457024 +0.000038618360
#define DEFAULT_UKF_MAG_N		+3.8226e-01	// +0.382258731690	 0.000109407461 +0.002851611558
#define DEFAULT_UKF_POS_DELAY		+2.0574e+03	// +2057.421963899194	 0.001097611925 -48.37809534324900
#define DEFAULT_UKF_VEL_DELAY		-1.0373e+05	// -103727.997010331557	 0.000109494449 -293.522967971236500
>>>>>>> .r25
#endif /* CONFIG_DEFAULT_H_ */