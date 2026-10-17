#include "APP/uart_imu.h"
#include <string.h>

#define IMU_DEVICE_ID 0xA4
#define IMU_READ_CODE 0X03
#define IMU_START_REG 0x08

imu_info imu_data;


bool bParseImuData(uint8_t *data,uint8_t length)
{
	uint8_t check_sum = 0;
	
	if(data[0] == IMU_DEVICE_ID && data[1] == IMU_READ_CODE && data[2] == IMU_START_REG)
	{
		for(uint8_t i=0; i < data[3]+4; i++ )
		{
			check_sum += data[i];
		}
		if(check_sum ==  data[data[3]+4]) 
		{
			memcpy(&imu_data,&data[4],data[3]);
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		return false;
	}
}
