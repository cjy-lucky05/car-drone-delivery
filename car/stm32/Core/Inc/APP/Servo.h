#ifndef _APP_SERVO_H_
#define _APP_SERVO_H_
#include "BSP/bsp_servo.h"
#include <stdbool.h>
typedef struct{
//	uint16_t usMaxRpm;
//	uint16_t usEncoderValuePerRound;
//	uint32_t uiLastEncoderReadTime;
//	int32_t iLastEncoderReadValue;
//	encoder_t xEncoder;
//	motor_t xMotor;
	
	float fCurrentAngle; 
	float fTargetAngle;
	uint16_t usPwm;
	double fPrevError;
	double fIntegral;
	double fDerivative;
	float fOutPutAngle;
	float fVelocity;
}Servo_t;

extern Servo_t xServo;
void vServoStart(Servo_t* pxServo);
void vSetAngleOffset(int16_t sPwm);
void vSetServoAngle(Servo_t* pxServo, bool bPid);
	
#endif
