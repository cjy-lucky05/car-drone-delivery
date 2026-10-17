#include "APP/Kinematics.h"
#include "APP/Motor.h"
#include "APP/config.h"
#include "APP/servo.h"

#include <stdbool.h>

//#include "APP/IMU.h"


#define PI		3.14159f

#define MAX_TURN_ANGLE 0.5
#define MIN_TURN_ANGLE -0.5

	
/*
MOTOR Position Define:
		 FORWORD
	M3				M4
			CAR
	M1				M2
		BACKWORD
*/
extern	Motor_t xMotor1;
extern	Motor_t xMotor2;
#if BASE_TYPE == BASE_4WD
	extern	Motor_t xMotor3;
	extern	Motor_t xMotor4;
#endif
extern float fGyroZ;

float fWheelThead = 1;
float fWheelBase = 1;
float fWheelDiameter = 1.0;
uint8_t ucBaseType = 0;
float fMaxXVelocity = 0;
float fMaxYVelocity = 0;	
float fMaxZAngle = 0;	
uint8_t ucKinematicsType = 0;
// 速度实列
Velocity_t xVelocity;
Odom_t xOdom;

/**

*/
void vSetKinematicsConfig(uint8_t param1,uint8_t param2){
	uint8_t param = param1+param2*2;
	switch(param){
		case 0:
#ifdef WHEEL_BASE_1			
			ucBaseType = BASE_2WD;
			fWheelThead = WHEEL_TREAD_1;
			fWheelBase = WHEEL_BASE_1;
			fWheelDiameter = WHEEL_DIAMETER_1;
			fMaxXVelocity = MAX_X_VELOCITY_1;
			fMaxYVelocity = MAX_Y_VELOCITY_1;
			fMaxZAngle = MAX_Z_ANGLE_1;
			ucKinematicsType = KINETICS_TYPE_1;
			break;
#endif		
#ifdef WHEEL_BASE_2
		case 1:
			ucBaseType = BASE_ACKERMAN;
			fWheelThead = WHEEL_TREAD_2;
			fWheelBase = WHEEL_BASE_2;
			fWheelDiameter = WHEEL_DIAMETER_2;
			fMaxXVelocity = MAX_X_VELOCITY_2;
			fMaxYVelocity = MAX_Y_VELOCITY_2;
			fMaxZAngle = MAX_Z_ANGLE_2;
			ucKinematicsType = KINETICS_TYPE_2;
			break;		
#endif
#ifdef WHEEL_BASE_2
		case 3:
			ucBaseType = BASE_ACKERMAN;
			fWheelThead = WHEEL_TREAD_2;
			fWheelBase = WHEEL_BASE_2;
			fWheelDiameter = WHEEL_DIAMETER_2;
			fMaxXVelocity = MAX_X_VELOCITY_2;
			fMaxYVelocity = MAX_Y_VELOCITY_2;
			fMaxZAngle = MAX_Z_ANGLE_2;
			ucKinematicsType = KINETICS_TYPE_2;
			break;		
#endif	
#ifdef WHEEL_BASE_3
		case 2:
			ucBaseType = BASE_2WD;
			fWheelThead = WHEEL_TREAD_3;
			fWheelBase = WHEEL_BASE_3;
			fWheelDiameter = WHEEL_DIAMETER_3;
			fMaxXVelocity = MAX_X_VELOCITY_3;
			fMaxYVelocity = MAX_Y_VELOCITY_3;
			fMaxZAngle = MAX_Z_ANGLE_3;
			ucKinematicsType = KINETICS_TYPE_3;
			break;		
#endif

		default:
			break;
	}
}
/**

*/
uint16_t usConvertAcc2MotorAcc(float fAccel){
	return (uint16_t)(((fAccel*60)/(1000/MOTOR_CONTROL_PERIOD))/(fWheelDiameter*PI));
}
/**

*/
void vComputeMotorSpeed(float fLineX,float fLineY,float fAngle){
	static float fLineXMinute,fLineYMinute,fAngleMinute;  //convert m/s to m/min
	static float fTanVel;  //convert m/s to m/min
	fLineXMinute = fLineX*60.0f;

	fLineYMinute = 0.0f;
	
	
	if(ucBaseType == BASE_ACKERMAN)
	{
////		float R = fWheelBase/tan(MAX_TURN_ANGLE);
//		if(fAngle > abs(fLineX/0.4)) fAngle = abs(fLineX/0.4);
//		if(fAngle < -abs(fLineX/0.4)) fAngle = -abs(fLineX/0.4);
//		fAngleMinute = fAngle*60.0f;
//		
//		if(fabs(fLineX) > 0.001) //avoid zero division
//			fAngle = atanf(fAngle*fWheelBase/fLineX);//convert Omega to steering angle  		
//		else 
//			fAngle = 0;//convert Omega to steering angle  
////		(fAngle > MAX_TURN_ANGLE)?(fAngle = MAX_TURN_ANGLE):((fAngle < MIN_TURN_ANGLE)?(fAngle = MIN_TURN_ANGLE):fAngle);
//		xServo.fTargetAngle = fAngle;
//
		if(fabs(fLineX) > 0.001)
		{
			fAngle = atanf(fAngle*fWheelBase/fLineX);//convert Omega to steering angle  
		}
		else{
			fAngle = 0.0;
		}
		if(fAngle > 0.5f) fAngle = 0.5f;
		if(fAngle < -0.5f) fAngle = -0.5f;
		xServo.fTargetAngle = fAngle;
		fAngleMinute = fAngle*60;		
		xMotor1.sTargetRpm = fLineXMinute/(PI*fWheelDiameter)*(1-fWheelThead*tan(fAngle)/2/fWheelBase);
		xMotor2.sTargetRpm = fLineXMinute/(PI*fWheelDiameter)*(1+fWheelThead*tan(fAngle)/2/fWheelBase);		
	}
	else
	{
		fAngleMinute = fAngle*60.0f;
		fTanVel = fAngleMinute * (fWheelThead/2);
		xMotor1.sTargetRpm = fLineXMinute/(PI*fWheelDiameter) - fTanVel/(PI*fWheelDiameter);
		xMotor2.sTargetRpm = fLineXMinute/(PI*fWheelDiameter) + fTanVel/(PI*fWheelDiameter);		
	}
	

}

/**

*/
void vComputeCarMotorSpeed(float fLineX,float fAccelX,float fAngle){
	int16_t sLeftRpm,sRightRpm; 
	static float fLineXMinute;  //convert m/s to m/min

	(fAngle > MAX_TURN_ANGLE)?(fAngle = MAX_TURN_ANGLE):((fAngle < MIN_TURN_ANGLE)?(fAngle = MIN_TURN_ANGLE):fAngle);
	
//	fLineXMinute = fLineX*60;
//	sLeftRpm = fLineXMinute/(PI*fWheelDiameter)*(1-fWheelThead*tan(xServo.fTargetAngle)/2/fWheelBase);
//	sRightRpm = fLineXMinute/(PI*fWheelDiameter)*(1+fWheelThead*tan(xServo.fTargetAngle)/2/fWheelBase);
	float R = 0.0;
	if(fAngle != 0.0)
	{
		R = fWheelBase/tan(fAngle);
		sLeftRpm = (int16_t)(60*(fLineX/R)*(R-fWheelThead/2)/(PI*fWheelDiameter));
		sRightRpm = (int16_t)(60*(fLineX/R)*(R+fWheelThead/2)/(PI*fWheelDiameter));	
	}
	else
	{
		sLeftRpm = (int16_t)(60*(fLineX)/(PI*fWheelDiameter));
		sRightRpm = (int16_t)(60*(fLineX)/(PI*fWheelDiameter));		
	}
	//motor speed threshold
	if(sLeftRpm > usMaxRpm)
		sLeftRpm = usMaxRpm;
	else if(sLeftRpm < -usMaxRpm)
		sLeftRpm = -usMaxRpm;

	if(sRightRpm > usMaxRpm)
		sRightRpm = usMaxRpm;
	else if(sRightRpm < -usMaxRpm)
		sRightRpm = -usMaxRpm;	
	//set value to motor
	xMotor1.sTargetRpm = sLeftRpm;
//	xMotor3.sTargetRpm = sLeftRpm;
	xMotor2.sTargetRpm = sRightRpm;
//	xMotor4.sTargetRpm = sRightRpm;
	//set servo angle
	xServo.fTargetAngle = fAngle;
//	xServo.fCurrentAngle = atanf(xImu.pfGyro[2]*L/(xVelocity.fLineX));	//calculate current real angle
//	CONVERT_OMEGA2ANGLE(xImu.pfGyro[2],xVelocity.fLineX);
}
/**

*/
void vGetSpeed(Velocity_t* xVelocity){
	double fAverageRpsX,fAverageRpsY;
	double fAverageRpsA; 
	
	fAverageRpsX  = (xMotor1.sCurrentRpm + xMotor2.sCurrentRpm)/2;
	fAverageRpsY  = 0;
	fAverageRpsA = ((xMotor2.sCurrentRpm ) - (xMotor1.sCurrentRpm ));
	
	fAverageRpsX /= 60.0f;
	xVelocity->fLineX = fAverageRpsX*(fWheelDiameter*PI);
	fAverageRpsY /= 60.0f;	
	xVelocity->fLineY = fAverageRpsY*(fWheelDiameter*PI);
	fAverageRpsA /= 60.0f;
	
	
		xVelocity->fAngleZ = fAverageRpsA*(fWheelDiameter*PI)/(1/cos(atanf(fWheelThead/fWheelBase)))/(fWheelBase)/(1/cos(atanf(fWheelThead/fWheelBase)));
}

