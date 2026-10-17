#ifndef __MAINLOOP_H_
#define __MAINLOOP_H_

#include "main.h"
#include <string.h>
#include <stdbool.h>
#include <stdio.h>

#include "APP/Config.h"
#include "APP/Motor.h"
#include "APP/Kinematics.h"
#include "APP/PS2.h"
#include "APP/Battery.h"
#include "APP/uart_imu.h"
#include "APP/imu.h"
#include "APP/Communication.h"
#include "APP/Indicate.h"
#include "app/sonar.h"
#include "APP/Servo.h"

#define UART_RX_BUF_SIZE		256
#define UART_TX_BUF_SIZE		256

extern volatile uint8_t ucUart1RecvEndCounter;
extern volatile uint8_t ucUart1RxBufLen;
extern uint8_t pucUart1RxBuf[UART_RX_BUF_SIZE];
extern uint8_t pucUart1RxBufCache[UART_RX_BUF_SIZE];

extern volatile uint8_t ucUart2RecvEndCounter;
extern volatile uint8_t ucUart2RxBufLen;
extern uint8_t pucUart2RxBuf[UART_RX_BUF_SIZE];
extern uint8_t pucUart2RxBufCache[UART_RX_BUF_SIZE];

extern volatile uint8_t ucUart4RxBufLen ;
extern volatile uint8_t ucUart4RecvEndCounter ;
extern uint8_t pucUart4RxBuf[UART_RX_BUF_SIZE] ;
extern uint8_t pucUart4RxBufCache[UART_RX_BUF_SIZE];

void vMainLoop(void);

#endif
