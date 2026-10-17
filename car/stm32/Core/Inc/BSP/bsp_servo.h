#ifndef __BSP_BSP_SERVO_H_
#define __BSP_BSP_SERVO_H_
#include "main.h"


#define SERVO_INIT 1500


typedef enum{
	SERVO1 = 0,
	SERVO2,	
	SERVO3,
	SERVO4
}servo_t;

extern int16_t sServoPwm;
extern int16_t sServoPwmOffset;
void vServoCallback(TIM_HandleTypeDef *htim);
void vServoRun(void);
void vSetServoPwm(servo_t ucServoID,uint16_t usPwm);
void vSetServoPwmOffset(int16_t sPwm);
void vServoBspInit(void);
#endif
