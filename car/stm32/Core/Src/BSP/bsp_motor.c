#include "BSP/bsp_motor.h"
#include <stdlib.h>

#define MOTOR_FULL_PWM 999

extern TIM_HandleTypeDef htim2;

#define MOTOR_Handle	htim2

TIM_TypeDef*  MOTOR_TIMER[] = {TIM2,TIM2};

GPIO_TypeDef*  MOTOR_AN1_PORT[] = {MOTOR1_AN1_GPIO_Port,MOTOR2_AN1_GPIO_Port};
const uint16_t	MOTOR_AN1_PIN[] = {MOTOR1_AN1_Pin,MOTOR2_AN1_Pin};
GPIO_PinState MOTOR_CCW_DIR[] = {GPIO_PIN_SET,GPIO_PIN_RESET};
GPIO_PinState MOTOR_CW_DIR[] = {GPIO_PIN_RESET,GPIO_PIN_SET};

const uint32_t MOTOR_TIMER_CHANNELA[] = {TIM_CHANNEL_3,TIM_CHANNEL_4};


/**
set motor run direction
CW and CCW is defined as from motor output shaft,shaft run direction
xMotor:motor ID
xDir:motor run direction
*/
static void vSetMotorDir(motor_t xMotor,motor_dir_t xDir){
	if(xDir == MOTOR_CCW){
		HAL_GPIO_WritePin(MOTOR_AN1_PORT[xMotor],MOTOR_AN1_PIN[xMotor],MOTOR_CW_DIR[xMotor]);
	}
	else if(xDir == MOTOR_CW){
		HAL_GPIO_WritePin(MOTOR_AN1_PORT[xMotor],MOTOR_AN1_PIN[xMotor],MOTOR_CCW_DIR[xMotor]);
	}
}

/**
set motor pwm
xMotor:motor ID
sPwm:>0 is CW direction,<0 is CCW direction
*/
void vSetMotorPWM(motor_t xMotor,int16_t sPwm){
	(sPwm > MOTOR_FULL_PWM)?(sPwm = MOTOR_FULL_PWM):((sPwm < -MOTOR_FULL_PWM)?(sPwm = -MOTOR_FULL_PWM):sPwm);
//	(sPwm > 200)?(sPwm = 200):((sPwm < -200)?(sPwm = -200):sPwm);
	if(sPwm > 0){
		vSetMotorDir(xMotor,MOTOR_CW);
	}
	else if(sPwm < 0){
		vSetMotorDir(xMotor,MOTOR_CCW);
	}
	else{	/*TODO motor break*/
		vSetMotorDir(xMotor,MOTOR_CW);
	}
	__HAL_TIM_SET_COMPARE(&MOTOR_Handle,(MOTOR_TIMER_CHANNELA[xMotor]),MOTOR_FULL_PWM-abs(sPwm));
}

void vMotorBspInit(void)
{
	HAL_TIM_PWM_Start(&MOTOR_Handle,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&MOTOR_Handle,TIM_CHANNEL_4);
	vSetMotorPWM(MOTOR1,0);
	vSetMotorPWM(MOTOR2,0);
}
