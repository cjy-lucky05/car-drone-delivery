#include "APP/servo.h"
#include "APP/Kinematics.h"



#define K 700.0f
Servo_t xServo;

#define SERVO_UP_PWM 1870
#define SERVO_DOWN_PWM 1160
#define RAD2DEG 57.295

void vServoStart(Servo_t* pxServo){
	vServoBspInit();
	pxServo->fTargetAngle = 0;
	pxServo->fIntegral = 0.0;
	pxServo->fPrevError = 0.0;
	pxServo->fDerivative = 0.0;
}

void vSetAngleOffset(int16_t sPwm){
	vSetServoPwmOffset(sPwm);
}
void vSetServoAngle(Servo_t* pxServo, bool bPid){
	double fError;
	float fAngleToServo = 0;
	if (pxServo->fTargetAngle > 0.0f)
		fAngleToServo = K;
	else
		fAngleToServo = K*1.25f;
	if (!bPid){
		pxServo->usPwm= SERVO_INIT - (pxServo->fTargetAngle)*fAngleToServo - sServoPwmOffset;
	}
	else{
			float fOutPutAngle = 0.0;
			if((pxServo->fVelocity < 0.01f)&&(pxServo->fVelocity > -0.01f)){			
				fOutPutAngle = pxServo->fTargetAngle;
				pxServo->fIntegral = 0.0;
				pxServo->fPrevError = 0.0;
				pxServo->fDerivative = 0.0;
			}
			else{
				fError = pxServo->fTargetAngle - pxServo->fCurrentAngle;
				if(fabs(fError)>0.02){
					pxServo->fIntegral += fError;
					pxServo->fDerivative = fError - pxServo->fPrevError;
					if(pxServo->fIntegral > 1)		// Integral threshold 
						pxServo->fIntegral = 1;
					else if (pxServo->fIntegral < -1)
						pxServo->fIntegral = -1;
					else;
				}
				fOutPutAngle = pxServo->fTargetAngle + fError*0.1 + pxServo->fIntegral*0.05 + pxServo->fDerivative*0.0;
				pxServo->fPrevError = fError;
			}
			pxServo->fOutPutAngle = fOutPutAngle;
			//printf("Angle %f\r\n",fOutPutAngle);
			pxServo-> usPwm = SERVO_INIT - (fOutPutAngle)*fAngleToServo - sServoPwmOffset;
	}
	//	printf("fTargetAngle %f usPwm a %d\r\n",pxServo->fTargetAngle,pxServo->usPwm);
	if(pxServo-> usPwm > SERVO_UP_PWM) pxServo-> usPwm = SERVO_UP_PWM;
	if(pxServo-> usPwm < SERVO_DOWN_PWM) pxServo-> usPwm = SERVO_DOWN_PWM; 
	vSetServoPwm(0,pxServo->usPwm);

//	vSetServoPwm(pxServo->usPwm);
}
