#include "APP/Indicate.h"
#include "bsp/ws2812.h"

#define FREQ2PERIOD(N)	1000.0f/N
 
#define SLOW    0.5f 
#define NORMAL  1.0f
#define	FAST		2.0f
#define RUSH		4.0f
 
Indicate_t xIndicate;

#define BRIGHTNESS 20
#define WS2812_ID  0
uint8_t rgb_color[3] ={0,0,0};
/**

*/
static void prvIndicateDeviceInit(IndicateDeviceInfo_t* xDevice,IndicateDeviceName_t xDeviceName){
	xDevice->xDevice = xDeviceName;
	xDevice->fFreq = 0.0;
	xDevice->uiStartTick = 0;
	xDevice->uiLastTick = 0;
	xDevice->uiDuration = 0;
}
/**

*/
static void prvIndicateDeviceSwitch(IndicateDeviceName_t xDeviceName,IndicateDeviceState_t xDeviceState){
	
	switch (xDeviceName){
		case LEDR:
//			HAL_GPIO_WritePin(LED_RED_GPIO_Port,LED_RED_Pin,(GPIO_PinState)xDeviceState);
			if (INDICATE_DEVICE_OFF == xDeviceState) rgb_color[0] = 0;
			else rgb_color[0] = BRIGHTNESS;
			vSetLEDColor(WS2812_ID,rgb_color);
			break;
		case LEDG:
//			HAL_GPIO_WritePin(LED_GREEN_GPIO_Port,LED_GREEN_Pin,(GPIO_PinState)xDeviceState);
			if (INDICATE_DEVICE_OFF == xDeviceState) rgb_color[1] = 0;
			else rgb_color[1] = BRIGHTNESS;
			vSetLEDColor(WS2812_ID,rgb_color);
			break;
		case LEDB:
//			HAL_GPIO_WritePin(LED_BLUE_GPIO_Port,LED_BLUE_Pin,(GPIO_PinState)xDeviceState);
			if (INDICATE_DEVICE_OFF == xDeviceState) rgb_color[2] = 0;
			else rgb_color[2] = BRIGHTNESS;
			vSetLEDColor(WS2812_ID,rgb_color);
			break;
		case BEEP:
			HAL_GPIO_WritePin(BEEP_GPIO_Port,BEEP_Pin,(GPIO_PinState)xDeviceState);
			break;
		default:
			break;
	}
//	vRefreshLEDColor();
}
/**

*/
static void prvIndicateDeviceToggle(IndicateDeviceName_t xDeviceName){
	switch (xDeviceName){
		case LEDR:
			if(rgb_color[0] == 0)  rgb_color[0] = BRIGHTNESS;
			else rgb_color[0] = 0;
			vSetLEDColor(WS2812_ID,rgb_color);
//			HAL_GPIO_TogglePin(LED_RED_GPIO_Port,LED_RED_Pin);
			break;
		case LEDG:
			if(rgb_color[1] == 0)  rgb_color[1] = BRIGHTNESS;
			else rgb_color[1] = 0;
			vSetLEDColor(WS2812_ID,rgb_color);			
//			HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port,LED_GREEN_Pin);
			break;
		case LEDB:
			if(rgb_color[2] == 0)  rgb_color[2] = BRIGHTNESS;
			else rgb_color[2] = 0;
			vSetLEDColor(WS2812_ID,rgb_color);			
//			HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port,LED_BLUE_Pin);
			break;
		case BEEP:
			HAL_GPIO_TogglePin(BEEP_GPIO_Port,BEEP_Pin);
			break;
		default:
			break;
	}
	
}

/**

*/
void vIndicateInit(void){
	prvIndicateDeviceInit(&xIndicate.xLedR,LEDR);
	prvIndicateDeviceInit(&xIndicate.xLedG,LEDG);
	prvIndicateDeviceInit(&xIndicate.xLedB,LEDB);
	prvIndicateDeviceInit(&xIndicate.xBeep,BEEP);
}
/**

*/
void vIndicate(void){
	uint32_t uiTick = HAL_GetTick();
	IndicateDeviceInfo_t* pxIndicateInfo = NULL;
	pxIndicateInfo = &xIndicate.xLedR;
//	
	for(uint8_t i=0;i<DEVICE_NUM;i++){
		switch(i){
			case LEDR:
				pxIndicateInfo = &xIndicate.xLedR;
				break;
			case LEDG:
				pxIndicateInfo = &xIndicate.xLedG;
				break;
			case LEDB:
				pxIndicateInfo = &xIndicate.xLedB;
				break;
			case BEEP:
				pxIndicateInfo = &xIndicate.xBeep;
				break;
		}
		if(pxIndicateInfo->fFreq > 0){
			if(uiTick - (pxIndicateInfo->uiLastTick)>FREQ2PERIOD(pxIndicateInfo->fFreq)){
				pxIndicateInfo->uiLastTick = uiTick;
				prvIndicateDeviceToggle(pxIndicateInfo->xDevice);
				if((pxIndicateInfo->uiDuration!=0)&&(uiTick - pxIndicateInfo->uiStartTick > pxIndicateInfo->uiDuration)){
					pxIndicateInfo->fFreq = 0;
					prvIndicateDeviceSwitch(pxIndicateInfo->xDevice,INDICATE_DEVICE_OFF);		/*switch device OFF*/
				}
			}
		}
		else{
			prvIndicateDeviceSwitch(pxIndicateInfo->xDevice,INDICATE_DEVICE_OFF);		/*switch device OFF*/
		}
	}
	vRefreshLEDColor();	
}
/**

*/
void vSetIndicate(IndicateDeviceName_t xIndicateDevice,float fFreq,uint32_t uiDuration){
	IndicateDeviceInfo_t* pxIndicateInfo = NULL;
	switch(xIndicateDevice){
		case LEDR:
			pxIndicateInfo = &xIndicate.xLedR;
			break;
		case LEDG:
			pxIndicateInfo = &xIndicate.xLedG;
			break;
		case LEDB:
			pxIndicateInfo = &xIndicate.xLedB;
			break;
		case BEEP:
			pxIndicateInfo = &xIndicate.xBeep;
			break;
		default:
			break;
	}
	if(pxIndicateInfo!=NULL){
		pxIndicateInfo->fFreq = fFreq;
		pxIndicateInfo->uiDuration = uiDuration;
		pxIndicateInfo->uiStartTick = HAL_GetTick();
		pxIndicateInfo->uiLastTick = HAL_GetTick() - 1500;
	}
}
