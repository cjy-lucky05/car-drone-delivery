#ifndef __APP_CONFIG_H_
#define __APP_CONFIG_H_
#include "main.h"

#define DEBUG_MODE	0
#ifdef DEBUG_MODE
	#define DEBUG_PERIOD	100
#endif

//#define DMA_PRINT 1

#define VERSION_INFO
#ifdef VERSION_INFO
#define SOFTWARE_MAIN_VERSION	1
#define SOFTWARE_SUBVERSION   8
#define SOFTWARE_PATCHLEVEL   2

#define HARDWARE_MAIN_VERSION	1
#define HARDWARE_SUBVERSION   6
#define HARDWARE_PATCHLEVEL   0
#endif


#define BASE_ACKERMAN 1

#define ACKERMAN  3


#define	_37GB520  2

#define ENCODER_CW   1
#define ENCODER_CCW  -1

#define BATTERY_VOLTAGE_CHECK_PERIOD	100
#define BATTERY_CURRENT_CHECK_PERIOD	5
#define MOTOR_CONTROL_PERIOD	10
#define PS2_CHECK_PERIOID			20
#define BATTERY_LOW_VOLTAGE		9.6f
#define DISCONNECT_WATI_TIME	5000
#define VELOCITY_LAST_TIME		800
#define SONAR_CHECK_PERIOD	  50
#define INDICATE_PERIOD				10

#define K_P    0.25f // P constant
#define K_I    0.15f // I constant
#define K_D    0.0f // D constant

#define VOLTAGE_ADC_CHANNEL ADC_CHANNEL_1
#define CURRENT_ADC_CHANNEL	ADC_CHANNEL_12

#define VOLTAGE_R1	47.0f
#define VOLTAGE_R2	10.0f
#define VOLTAGE_ERROR 0.30f
#define VOLTAGE_STATE_CHANGE_GAP 0.2f

#define CURRENT_RES	0.01f
#define CURRENT_SCALE	20

#define SONAR_CHANNEL_NUM 1

//#define BASE_TYPE	BASE_4WD


//GB520 Motor Param
#define MOTOR_TYPE_2 			_37GB520
#define MOTOR_MAX_RPM_2 	10000
#define ENCODER_PPR_2     11
#define RATIO_2						30.00f	// reduction ratio param 1
#define ENCODER_DIR_2			ENCODER_CW



//NanoCar Chassis Param(Ackerman Steering)
#define WHEEL_TREAD_2			0.158f
#define WHEEL_BASE_2			0.1445f
#define WHEEL_DIAMETER_2  0.064f //wheel's diameter in meters
#define MAX_X_VELOCITY_2	MOTOR_MAX_RPM_2/60.0/RATIO_2*WHEEL_DIAMETER_2*3.14159f*0.8f
#define MAX_Y_VELOCITY_2  MOTOR_MAX_RPM_2/60.0/RATIO_2*WHEEL_DIAMETER_2*3.14159f*0.8f
#define MAX_Z_ANGLE_2			MAX_X_VELOCITY_2/(WHEEL_TREAD_2/2.0f)
#define KINETICS_TYPE_2   ACKERMAN 


#endif

