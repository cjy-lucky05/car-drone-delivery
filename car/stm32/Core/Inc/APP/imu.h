#ifndef __APP_IMU_H_
#define __APP_IMU_H_
#include <stdint.h>


typedef struct{
	float fPitch;
	float fRoll;
	float fYaw;
	float pfEuler[3];
	float pfQuat[4];
	float pfGyro[3];
	float pfAccel[3];
	float ftemperature;
}Imu_t;

extern Imu_t xImu;
extern const float gyro_dps2rad;
extern const float accel_dps2grav;
extern const float accel_dps2accel;
#endif
