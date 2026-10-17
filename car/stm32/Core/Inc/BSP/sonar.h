#ifndef __APP_SONAR_H_
#define __APP_SONAR_H_
#include "main.h"

extern uint32_t uiSonarus[4];
extern uint8_t ucSonarGroupNum;

void vTrigSonar(uint8_t ucGroupNum);
void vEchoSonar(uint8_t ucGroup,GPIO_PinState IO);

#endif
