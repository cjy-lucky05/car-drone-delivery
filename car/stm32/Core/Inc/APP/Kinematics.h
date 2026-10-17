#ifndef __APP_KINEMATICS_H_
#define __APP_KINEMATICS_H_
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#define CONVERT_OMEGA2ANGLE(Omega,v,wheel_base)  (float)atanf((float)(Omega*wheel_base/v))

typedef struct{
	float fLineX;
	float fLineY;
	float fAngleZ;
}Velocity_t;

typedef struct{
	float x;
	float y;
}Position_t;

typedef struct{
	Position_t xPos;
}Odom_t;

extern Velocity_t xVelocity;
extern Odom_t xOdom;

extern float fWheelDiameter;
extern float fWheelThead;
extern float fWheelBase;
extern uint8_t ucBaseType;

extern float fMaxXVelocity;
extern float fMaxYVelocity;	
extern float fMaxZAngle;	


void vSetKinematicsConfig(uint8_t param1,uint8_t param2);
uint16_t usConvertAcc2MotorAcc(float fAccel);
void vComputeMotorSpeed(float fLineX,float fLineY,float fAngle);
void vComputeCarMotorSpeed(float fLineX,float fAccelX,float fAngle);
void vGetSpeed(Velocity_t* xVelocity);
#endif
