#include "BSP/bsp_servo.h"
#include <stdbool.h>
#define FULL_PWM		20000

#define SERVO_UP_PWM 2500
#define SERVO_DOWN_PWM 500
#define SERVO_TIMER	TIM6

uint16_t usServoPwm = SERVO_INIT;
int16_t sServoPwmOffset = 0;
bool bServoFlag = false;
extern TIM_HandleTypeDef htim6;

uint16_t usServoPWM[4] = {1500,1500,1500,1500};
uint16_t usServoPWMBuf[4] = {1500,1500,1500,1500};
uint8_t ucServoPWMSort[4] = {0,1,2,3};
uint8_t ucCurrentServo = 0;
uint16_t usServoGap = 0;

GPIO_TypeDef*  	SERVO_PORT[] = {SERVO_SIG1_GPIO_Port};
const uint16_t	SERVO_PIN[]  = {SERVO_SIG1_Pin};

/**
call in HAL_TIM_PeriodElapsedCallback function
*/
void vServoCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == SERVO_TIMER){

			if(ucCurrentServo == 0)
			{
				HAL_GPIO_WritePin(SERVO_SIG1_GPIO_Port,SERVO_SIG1_Pin,GPIO_PIN_SET);
				SERVO_TIMER->ARR = usServoPWM[ucServoPWMSort[ucCurrentServo]];	
				ucCurrentServo++;				
			}
			else if(ucCurrentServo == 1 )
			{
				while(ucCurrentServo < 2)
				{
					HAL_GPIO_WritePin(SERVO_PORT[ucServoPWMSort[ucCurrentServo-1]],SERVO_PIN[ucServoPWMSort[ucCurrentServo-1]],GPIO_PIN_RESET);
					if(ucCurrentServo < 1)
					{
						if(usServoPWM[ucServoPWMSort[ucCurrentServo]] != usServoPWM[ucServoPWMSort[ucCurrentServo-1]])
						{
							SERVO_TIMER->ARR = usServoPWM[ucServoPWMSort[ucCurrentServo]] - usServoPWM[ucServoPWMSort[ucCurrentServo-1]];
							ucCurrentServo++;
							break;
						}				
						else{
								ucCurrentServo++;
						}					
					}
					else{
						ucCurrentServo = 0;
						SERVO_TIMER->ARR = (FULL_PWM - usServoPWM[ucServoPWMSort[0]]);
						for(uint8_t i = 0;i<1;i++)
						{
							usServoPWM[i] = usServoPWMBuf[i];
						}
						break;
					}
				}
			}
			else{
				SERVO_TIMER->ARR = (FULL_PWM - usServoPWM[ucServoPWMSort[0]]);
				for(uint8_t i = 0;i<1;i++)
				{
					usServoPWM[i] = usServoPWMBuf[i];
				}		
				ucCurrentServo = 0;				
			}
	}
}

void vSetServoPwm(servo_t ucServoID,uint16_t usPwm){
	(usPwm > SERVO_UP_PWM)?(usPwm = SERVO_UP_PWM):((usPwm < SERVO_DOWN_PWM)?(usPwm = SERVO_DOWN_PWM):usPwm);
	usServoPWMBuf[ucServoID] = usPwm;	
}

void vSetServoPwmOffset(int16_t sPwm){
	sServoPwmOffset = sPwm;
}
void vServoBspInit(void){
	HAL_TIM_Base_Start_IT(&htim6);
}

