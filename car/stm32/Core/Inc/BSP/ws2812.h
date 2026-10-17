#ifndef __BSP_WS2812_H
#define __BSP_WS2812_H
#include "main.h"

#define LED_NUM 1

void RGB_LED_Write0(void);
void RGB_LED_Write1(void);
void RGB_LED_Write_24Bits(uint8_t green,uint8_t red,uint8_t blue);
void RGB_LED_Reset(void);
void RGB_LED_Red(void);
void RGB_LED_Green(void);
void RGB_LED_Blue(void);
void RGB_LED_OFF(void);
void RGB_LED(uint32_t rgb);
void vLEDInit(void);
void vSetLEDColor(uint8_t i,uint8_t *rgb);
void vRefreshLEDColor(void);
#endif

