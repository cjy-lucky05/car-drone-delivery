#include "APP/Motor.h"
#include "APP/config.h"
PID_t xPid;

uint16_t usMotorMaxRpm = 0;
float fRatio = 0;
uint16_t usEncoderPPR = 0;
uint16_t usMaxRpm = 0;
uint16_t usAccelDecel =50;
uint8_t ucMotorType = 0;
uint8_t cEncoderDir = 0;

Motor_t xMotor1,xMotor2;


void vSetMotorConfig(uint8_t param){

		ucMotorType = MOTOR_TYPE_2;
		usMotorMaxRpm = MOTOR_MAX_RPM_2;
		fRatio = RATIO_2;
		usEncoderPPR = ENCODER_PPR_2*4*fRatio;
		cEncoderDir = ENCODER_DIR_2;	
	
	usMaxRpm = usMotorMaxRpm/(fRatio);
}

void vSetMotorAccelDecel(uint16_t usSetAccelDecel){
	usAccelDecel = usSetAccelDecel;
}
/**

*/
int16_t sRpm2PWM(double sRpm){
	double fTemp = 0;
	//remap scale of target RPM vs usMaxRpm to PWM
	if(sRpm > usMaxRpm) sRpm = usMaxRpm;
	if(sRpm < -usMaxRpm) sRpm = -usMaxRpm;
	fTemp = (double)sRpm / (double)175.00;
	fTemp *= 1000;
	return (int16_t)fTemp;
}

void vPidInit(void){
	xPid.p = K_P;
	xPid.i = K_I;
	xPid.d = K_D;
}

void vMotorInit(Motor_t *pxMotor,	motor_t xMotor, encoder_t xEncoder){
	pxMotor->xEncoder = xEncoder;
	pxMotor->xMotor = xMotor;
	pxMotor->usEncoderValuePerRound = usEncoderPPR;
	pxMotor->iLastEncoderReadValue = 0;
	pxMotor->uiLastEncoderReadTime = 0;
}

void vGetMotorSpeed(Motor_t *pxMotor){
	int32_t iEncoderDelta = 0;
	uint32_t uiTickGap = 0;
	if(cEncoderDir == ENCODER_CW)
		iEncoderDelta = -(iGetEncoderValue(pxMotor->xEncoder) - pxMotor->iLastEncoderReadValue);
	else
		iEncoderDelta = (iGetEncoderValue(pxMotor->xEncoder) - pxMotor->iLastEncoderReadValue);
	pxMotor->iLastEncoderReadValue = iGetEncoderValue(pxMotor->xEncoder);
	uiTickGap = HAL_GetTick() - pxMotor->uiLastEncoderReadTime;
	pxMotor->uiLastEncoderReadTime = HAL_GetTick();
	pxMotor->sCurrentRpm = (60000*iEncoderDelta)/(int32_t)(uiTickGap*(pxMotor->usEncoderValuePerRound));
}

void vSetMotorSpeed(Motor_t *pxMotor,int16_t sRpm){
	pxMotor->sTargetRpm = sRpm;
}

void vExecuteMotorSpeed(Motor_t *pxMotor,double sRpm){
	vSetMotorPWM(pxMotor->xMotor,sRpm2PWM(sRpm));
}

double fMotorPidCompute(Motor_t *pxMotor){
	double fError;
	double fPidResult;

	if(pxMotor->sTargetRpm > pxMotor->sSmoothTargetRpm){
		pxMotor->sSmoothTargetRpm += usAccelDecel;
		if(pxMotor->sSmoothTargetRpm > pxMotor->sTargetRpm)
			pxMotor->sSmoothTargetRpm = pxMotor->sTargetRpm;	
	}
	else if(pxMotor->sTargetRpm < pxMotor->sSmoothTargetRpm){
		pxMotor->sSmoothTargetRpm -= usAccelDecel;
		if(pxMotor->sSmoothTargetRpm < pxMotor->sTargetRpm)
			pxMotor->sSmoothTargetRpm = pxMotor->sTargetRpm;	
	}
	
//	pxMotor->sSmoothTargetRpm = pxMotor->sTargetRpm;

	fError = pxMotor->sSmoothTargetRpm - pxMotor->sCurrentRpm;
	pxMotor->fIntegral += fError;
	pxMotor->fDerivative = fError - pxMotor->fPrevError;
	pxMotor->fPrevError = fError;
	if(pxMotor->sSmoothTargetRpm == 0.0 && fError == 0.0){
		pxMotor->fIntegral = 0;
	}
	//·ÀÖ¹»ý·Ö±¥ºÍ
	if(pxMotor->fIntegral > 2000)
		pxMotor->fIntegral = 2000;
	else if (pxMotor->fIntegral < -2000)
		pxMotor->fIntegral = -2000;
	
	fPidResult = fError*xPid.p + pxMotor->fIntegral*xPid.i + pxMotor->fDerivative*xPid.d;
	pxMotor->sSetRpm = (uint16_t)fPidResult;
	return fPidResult;
}


void vMotorControlInit(void){
	vMotorBspInit();
	vEncoderBspInit();
	vPidInit();
	
	vMotorInit(&xMotor1,MOTOR1,ENCODER1);
	vMotorInit(&xMotor2,MOTOR2,ENCODER2);
	
}
