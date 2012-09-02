/*
 * ekf.h
 *
 * Created: 9/17/2011 2:17:38 PM
 *  Author: GrubyGrub
 */


#ifndef EKF_H_
#define EKF_H_

#define DEG2RAD 0.0174532925
#define RAD2DEG 57.2957796


#define GYRO_500DPS 17.50

#define GYRO_DEGLSB (1/14.375)

#define MAG_COV 0.6
#define ACC_COV 0.6
#define P_COV 0.5
#define Q_COV 0.000001

//Gyro Bias
static float GYRO_BX = -22.8819;
static float GYRO_BY = 14.2851;
static float GYRO_BZ = 5.2421;

//Acc Reference for Magnatomer
static float  ACC_RX = 0;
static float  ACC_RY = 0;
static float  ACC_RZ = 1;

//Mag reference
static float  MAG_RX = -0.2525;
static float  MAG_RY = -0.9674;
static float  MAG_RZ = 0.0177;

//Mag bias
static float  MAG_BX = -33.4495;
static float  MAG_BY = 287.7760;
static float  MAG_BZ = -142.0700;

//Mag Calibration matrix
static float  MAG_CXX = 0.0;
static float  MAG_CXY = -1.0;
static float  MAG_CXZ = 0.0;

static float  MAG_CYX = 0.0;
static float  MAG_CYY = 0.0;
static float  MAG_CYZ = 1.0;

static float  MAG_CZX = 1.0;
static float  MAG_CZY = 0.0;
static float  MAG_CZZ = 0.0;






void ekf_init(void);
void process_gyro(float gx, float gy, float gz, float dT);
void process_mag(float mx, float my, float mz);
void process_acc(float ax, float ay, float az);


void ekf_print(void);












#endif /* EKF_H_ */
