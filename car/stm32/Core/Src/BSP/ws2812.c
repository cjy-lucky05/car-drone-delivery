#include "main.h"
#include "bsp/ws2812.h"





#define RGB_LED_HIGH WS2812_GPIO_Port->BSRR = (uint32_t)WS2812_Pin;
#define RGB_LED_LOW  WS2812_GPIO_Port->BRR = (uint32_t)WS2812_Pin;

uint8_t ucLEDGreen[LED_NUM] = {0};
uint8_t ucLEDBlue[LED_NUM] = {0};
uint8_t ucLEDRed[LED_NUM] = {0};

/********************************************************/
//
/********************************************************/
void RGB_LED_Write0(void)
{
	RGB_LED_HIGH;
  //250ns delay,mesure by oscilloscope
	for(uint32_t i =0; i<1; i++)   __nop();
	RGB_LED_LOW;
	for(uint32_t i =0; i<3; i++)   __nop();
	//750ns delay,mesure by oscilloscope
}

/********************************************************/
//
/********************************************************/

void RGB_LED_Write1(void)
{
	RGB_LED_HIGH;
	for(uint32_t i =0; i<5; i++)   __nop();//750 delay,mesure by oscilloscope
	RGB_LED_LOW;
//	for(uint32_t i =0; i<2; i++)   __nop();
	//250ns delay,mesure by oscilloscope
	
}

void RGB_LED_Reset(void)
{
	RGB_LED_LOW;
//	for(uint32_t i =0; i<40; i++)   __nop();//750 delay,mesure by oscilloscope
}

void RGB_LED_Write_Byte(uint8_t byte)
{
	uint8_t i;

	for(i=0;i<8;i++)
		{
			if(byte&0x80)
			{
				RGB_LED_HIGH;
				for(uint32_t i =0; i<5; i++)   __nop();//750 delay,mesure by oscilloscope
				RGB_LED_LOW;
//					RGB_LED_Write1();
			}
			else
			{
				RGB_LED_HIGH;
				//250ns delay,mesure by oscilloscope
				for(uint32_t i =0; i<1; i++)   __nop();
				RGB_LED_LOW;
				for(uint32_t i =0; i<3; i++)   __nop();				
//					RGB_LED_Write0();
			}
		byte <<= 1;
	}
}

void RGB_LED_Write_24Bits(uint8_t green,uint8_t red,uint8_t blue)
{
	RGB_LED_Write_Byte(green);
	RGB_LED_Write_Byte(red);
	RGB_LED_Write_Byte(blue);
}

void RGB_LED_Red(void)
{
	RGB_LED_Write_24Bits(0, 0x0f, 0);
}

void RGB_LED_Green(void)
{
	RGB_LED_Write_24Bits(0x0f, 0, 0);
}

void RGB_LED_Blue(void)
{
	RGB_LED_Write_24Bits(0, 0, 0x0f);
}

void RGB_LED_OFF(void){
	RGB_LED_Write_24Bits(0, 0, 0);
}

void RGB_LED(uint32_t rgb){
	switch(rgb){
		case 0:
			RGB_LED_Blue();
		  break;
		case 1:
			RGB_LED_Green();
			break;
		case 2:
			RGB_LED_Red();
			break;
		default:
			RGB_LED_OFF();
			break;
	}
}

/**

*/
void vLEDInit(void){
	for(uint8_t i = 0; i < LED_NUM; i++){
		ucLEDGreen[i] = 0x00;
		ucLEDBlue[i] = 0x00;
		ucLEDRed[i] = 0x00;
	}
	vRefreshLEDColor();
}
/**
rgb is a victor 3 length array,8Bit red green bule color 
*/
void vSetLEDColor(uint8_t i,uint8_t *rgb){
	if (i > LED_NUM-1) return;
	ucLEDRed[i] = rgb[0];
	ucLEDGreen[i] = rgb[1];
	ucLEDBlue[i] = rgb[2];
}
/**

*/
void vRefreshLEDColor(void){
	__disable_irq();
	for(uint8_t i = 0; i < LED_NUM; i++ ){
		RGB_LED_Write_24Bits(ucLEDGreen[i],ucLEDRed[i],ucLEDBlue[i]);	
	//	printf("%d %d %d %d\r\n",ucLEDGreen[i],ucLEDRed[i],ucLEDBlue[i],i);
	}	
	__enable_irq();
	RGB_LED_Reset();
}
