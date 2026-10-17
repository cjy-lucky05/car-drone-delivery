#include "APP/imu.h"

const float gyro_dps2rad = 2048.0f/32768.0f*3.14159f/180.0f;
const float accel_dps2grav = 16.0f/32768.0f;
const float accel_dps2accel = 9.8f*16.0f/32768.0f;
Imu_t xImu;



