#include "app/sonar.h"
#include "app/config.h"

uint8_t ucSonarGroupNum = 0;

uint32_t puiEnterTick[2] = {0};
uint32_t puiEnterVal[2] = {0};

uint8_t ucRasingFlag[2] = {0};
uint32_t uiEndTick[2] = {0};
uint32_t uiEndValue[2] = {0};
uint32_t uiSonarus[4] = {0};


/**

*/
static void delay_us(uint32_t nus)
{		
	uint32_t ticks;
	uint32_t told,tnow,tcnt=0;
	uint32_t reload = SysTick->LOAD;				
	ticks=nus*84; 						//72 is system freq
	told=SysTick->VAL;        
	while(1)
	{
		tnow=SysTick->VAL;	
		if(tnow!=told)
		{	    
			if(tnow<told) tcnt+=told-tnow;
			else tcnt+=reload-tnow+told;	    
			told=tnow;
			if(tcnt>=ticks) break;
		}  
	};
}
/**

*/
void vTrigSonar(uint8_t ucGroupNum){
	if(ucGroupNum == 1){
			HAL_GPIO_WritePin(SR04_TRIG_1_GPIO_Port,SR04_TRIG_1_Pin,GPIO_PIN_SET);
			delay_us(12);
			HAL_GPIO_WritePin(SR04_TRIG_1_GPIO_Port,SR04_TRIG_1_Pin,GPIO_PIN_RESET);
			ucSonarGroupNum = 1;
	}
	#if SONAR_CHANNEL_NUM > 1
	else if (ucGroupNum == 2){
			HAL_GPIO_WritePin(SR04_TRIG_2_GPIO_Port,SR04_TRIG_2_Pin,GPIO_PIN_SET);
			delay_us(12);
			HAL_GPIO_WritePin(SR04_TRIG_2_GPIO_Port,SR04_TRIG_2_Pin,GPIO_PIN_RESET);		
			ucSonarGroupNum = 2;
	}
	#endif
}

void vEchoSonar(uint8_t ucGroup,GPIO_PinState IO ){
	uint32_t tnow=SysTick->VAL;	
	uint32_t uiTickNow = HAL_GetTick();
	uint32_t uiGapus = 0;
	
	if(ucSonarGroupNum == 1){
		if(IO == GPIO_PIN_RESET){
			if(tnow<puiEnterVal[0]) uiGapus = ((uiTickNow-puiEnterTick[0])*1000 + (puiEnterVal[0] - tnow)/84);
			else uiGapus = ((uiTickNow-puiEnterTick[0]-1)*1000 + (SysTick->LOAD - tnow + puiEnterVal[0])/84);
			if(ucGroup == 1){
				uiSonarus[0] = uiGapus;
			}
			else if(ucGroup == 2){
				uiSonarus[2] = uiGapus;
			}			
		}
		else{
			puiEnterTick[0] = HAL_GetTick();
			puiEnterVal[0] = SysTick->VAL;	
		}

	}
	else if(ucSonarGroupNum == 2){
		if(IO == GPIO_PIN_RESET){
			if(tnow<puiEnterVal[1]) uiGapus = ((uiTickNow-puiEnterTick[1])*1000 + (puiEnterVal[1] - tnow)/84);
			else uiGapus = ((uiTickNow-puiEnterTick[1]-1)*1000 + (SysTick->LOAD - tnow + puiEnterVal[1])/84);
			if(ucGroup == 1){
				uiSonarus[3] = uiGapus;
			}
			else if(ucGroup == 2){
				uiSonarus[1] = uiGapus;
			}						
		}
		else{
			puiEnterTick[1] = HAL_GetTick();
			puiEnterVal[1] = SysTick->VAL;	
		}
	}
}

