#ifndef __APP_UART_IMU_H_
#define __APP_UART_IMU_H_

#include "main.h"
#include <stdbool.h>

__packed typedef struct
{
	int16_t acc[3];
	int16_t gyro[3];
	int16_t roll;
	int16_t pitch;
	int16_t yaw;
	uint8_t leve;
	int16_t temp;
	int16_t mag[3];
	int16_t quat[4];
} imu_info;

extern imu_info imu_data;

bool bParseImuData(uint8_t *data,uint8_t length);

#endif
