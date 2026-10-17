#ifndef __APP_SERVO_H_
#define __APP_SERVO_H_


#define FLAG_RECV            1
#define FLAG_RXBUFF          2

#define MEDIAN_VALUE         2000

#define MID_ID5_MAX          3700
#define MID_ID5_MIN          380
#define MID_VAL_ID5          1486
#define MID_VAL_ID6          3100


#define ARM_MAX_VALUE        (4000)
#define ARM_MIN_VALUE        (96)


#define RX_MAX_BUF           8
#define MAX_SERVO_NUM        6

#include "stm32f3xx_hal.h"



void UartServo_Clear_Flag(void);
void UartServo_Ctrl(uint8_t id, uint16_t value, uint16_t time);
void UartServo_Set_Snyc_Buffer(uint16_t s1, uint16_t s2, uint16_t s3, uint16_t s4, uint16_t s5, uint16_t s6);
void UartServo_Sync_Write(uint16_t sync_time);
void UartServo_Set_Torque(uint8_t enable);
void UartServo_Set_ID(uint8_t id);
void UartServo_Get_Angle(uint8_t id);
void UartServo_Send_ARM_Angle(uint8_t id, uint16_t value);
void UartServo_Send_ARM_Angle_Array(void);
void UartServo_Clear_Arm_Read_Vlaue(void);
int16_t UartServo_Get_Median_Offset(uint8_t id);
int8_t UartServo_Revice(uint8_t* pucCommandBuf,uint8_t ucCommandLen);
uint8_t UartServo_Rx_Parse(uint8_t flag);
#endif
