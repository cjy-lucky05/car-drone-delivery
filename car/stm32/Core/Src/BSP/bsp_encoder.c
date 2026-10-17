#include "BSP/bsp_encoder.h"
#include "APP/config.h"

extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim1;

#define ENCODER1_TIMER	TIM4
#define ENCODER2_TIMER	TIM1

#define ENCODER1_Handle	htim4
#define ENCODER2_Handle	htim1


#define ENCODER_TIM_PERIOD	50000

volatile int32_t uiEncoder1OverflowCount = 0;
volatile int32_t uiEncoder2OverflowCount = 0;


extern void vServoCallback(TIM_HandleTypeDef *htim);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
//	vServoCallback(htim);
	if(htim->Instance == ENCODER1_TIMER){
		if(__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
			uiEncoder1OverflowCount--; 
		else
			uiEncoder1OverflowCount++;
	}
	else if(htim->Instance == ENCODER2_TIMER){
		if(__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
			uiEncoder2OverflowCount--; 
		else
			uiEncoder2OverflowCount++;
	}
	else{
		__nop();
	}	
}

int32_t iGetEncoderValue(encoder_t xEncoder){
	int32_t iEncoderValue = 0;
	switch(xEncoder){
		case ENCODER1:
			iEncoderValue = -(ENCODER_TIM_PERIOD*uiEncoder1OverflowCount + __HAL_TIM_GET_COUNTER(&ENCODER1_Handle));
		break;
		case ENCODER2:
			iEncoderValue = -(ENCODER_TIM_PERIOD*uiEncoder2OverflowCount + __HAL_TIM_GET_COUNTER(&ENCODER2_Handle));
			break;
		default:
			break;
	}
	return iEncoderValue;
}

void vEncoderBspInit(void)
{
	__HAL_TIM_CLEAR_IT(&ENCODER1_Handle, TIM_IT_UPDATE); 
	__HAL_TIM_URS_ENABLE(&ENCODER1_Handle);               
	__HAL_TIM_ENABLE_IT(&ENCODER1_Handle,TIM_IT_UPDATE);

	__HAL_TIM_CLEAR_IT(&ENCODER2_Handle, TIM_IT_UPDATE); 
	__HAL_TIM_URS_ENABLE(&ENCODER2_Handle);               
	__HAL_TIM_ENABLE_IT(&ENCODER2_Handle,TIM_IT_UPDATE);

	HAL_TIM_Encoder_Start(&ENCODER1_Handle,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&ENCODER2_Handle,TIM_CHANNEL_ALL);
	
}
