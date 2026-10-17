#include "APP/uart_servo.h"
#include "stm32f3xx_hal.h"
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
extern void vSendCommand(uint8_t* pucSendBuf,uint8_t ucSendBufLen);
extern UART_HandleTypeDef huart4;
unsigned char crc8_chk_value(unsigned char *message, unsigned char len);
uint16_t sync_servo[] = {MEDIAN_VALUE, MEDIAN_VALUE, MEDIAN_VALUE, MEDIAN_VALUE, MID_VAL_ID5, MID_VAL_ID6};

uint8_t RecvFlag = 0;
uint8_t Rx_Data[RX_MAX_BUF] = {0};
uint8_t Rx_index = 0;
uint8_t Rx_Flag = 0;

// ��λƫ��ֵ
uint16_t g_arm_median_value[] = {MEDIAN_VALUE, MEDIAN_VALUE, MEDIAN_VALUE, MEDIAN_VALUE, MID_VAL_ID5, MID_VAL_ID6};
// ��ȡ���ݵ�״̬����ֵ
uint16_t servo_read_state = 0;
uint16_t g_arm_read_value[6] = {0};

uint8_t g_test_read_ok = 0;




/* ���ƶ�� */
void UartServo_Ctrl(uint8_t id, uint16_t value, uint16_t time)
{
    uint8_t head1 = 0xff;
    uint8_t head2 = 0xff;
    uint8_t s_id = id & 0xff;
    uint8_t len = 0x07;
    uint8_t cmd = 0x03;
    uint8_t addr = 0x2a;

    value = value + UartServo_Get_Median_Offset(s_id);

    if (value > ARM_MAX_VALUE)
        value = ARM_MAX_VALUE;
    else if (value < ARM_MIN_VALUE)
        value = ARM_MIN_VALUE;

    uint8_t pos_H = (value >> 8) & 0xff;
    uint8_t pos_L = value & 0xff;

    uint8_t time_H = (time >> 8) & 0xff;
    uint8_t time_L = time & 0xff;

    uint8_t checknum = (~(s_id + len + cmd + addr +
                          pos_H + pos_L + time_H + time_L)) &
                       0xff;
    uint8_t data[] = {head1, head2, s_id, len, cmd,
                      addr, pos_H, pos_L, time_H, time_L, checknum};
	
    HAL_UART_Transmit(&huart4,data,sizeof(data),100);
	
}

/* ����ͬ��д�Ļ���ֵ */
void UartServo_Set_Snyc_Buffer(uint16_t s1, uint16_t s2, uint16_t s3, uint16_t s4, uint16_t s5, uint16_t s6)
{
	//	printf("\r\n %d %d %d %d %d %d\r\n ",s1,s2,s3,s4,s5,s6);
    sync_servo[0] = s1 + UartServo_Get_Median_Offset(1);
    sync_servo[1] = s2 + UartServo_Get_Median_Offset(2);
    sync_servo[2] = s3 + UartServo_Get_Median_Offset(3);
    sync_servo[3] = s4 + UartServo_Get_Median_Offset(4);
    sync_servo[4] = s5 + UartServo_Get_Median_Offset(5);
    sync_servo[5] = s6 + UartServo_Get_Median_Offset(6);

    for (int i = 0; i < 6; i++)
    {
        if (sync_servo[i] > ARM_MAX_VALUE)
        {
            sync_servo[i] = MEDIAN_VALUE;
        }
        else if (sync_servo[i] < ARM_MIN_VALUE)
        {
            sync_servo[i] = MEDIAN_VALUE;
        }
    }
}

/* ͬʱ�������д�벻ͬ�Ĳ��� */
void UartServo_Sync_Write(uint16_t sync_time)
{
    uint8_t head1 = 0xff;
    uint8_t head2 = 0xff;
    uint8_t s_id = 0xfe;
    uint8_t len = 0x22; /* ���ݳ��ȣ���Ҫ����ʵ��ID�����޸ģ�len= sizeof(data)-4*/
    uint8_t cmd = 0x83;
    uint8_t addr = 0x2a;
    uint8_t data_len = 0x04; /* ʵ�ʵ���д�������ֽ���������Ҫ�޸� */

    uint8_t ctrl_id[MAX_SERVO_NUM] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};

    uint8_t pos_n_H[MAX_SERVO_NUM] = {0};
    uint8_t pos_n_L[MAX_SERVO_NUM] = {0};

    uint8_t pos_sum = 0;
    uint8_t time_H = (sync_time >> 8) & 0xff;
    uint8_t time_L = sync_time & 0xff;

    for (uint8_t i = 0; i < MAX_SERVO_NUM; i++)
    {
        pos_n_H[i] = (sync_servo[i] >> 8) & 0xff;
        pos_n_L[i] = sync_servo[i] & 0xff;
        pos_sum = (pos_sum + ctrl_id[i] + pos_n_H[i] + pos_n_L[i] + time_H + time_L) & 0xff;
    }

    uint8_t checknum = (~(s_id + len + cmd + addr + data_len + pos_sum)) & 0xff;

    uint8_t data[] = {head1, head2, s_id, len, cmd, addr, data_len,
                      ctrl_id[0], +pos_n_H[0], pos_n_L[0], time_H, time_L,
                      ctrl_id[1], pos_n_H[1], pos_n_L[1], time_H, time_L,
                      ctrl_id[2], pos_n_H[2], pos_n_L[2], time_H, time_L,
                      ctrl_id[3], pos_n_H[3], pos_n_L[3], time_H, time_L,
                      ctrl_id[4], pos_n_H[4], pos_n_L[4], time_H, time_L,
                      ctrl_id[5], pos_n_H[5], pos_n_L[5], time_H, time_L,
                      checknum};
    HAL_UART_Transmit(&huart4,data,sizeof(data),100);
}

/* �������߶����Ť�ؿ���,0Ϊ�رգ�1Ϊ����*/
void UartServo_Set_Torque(uint8_t enable)
{
    uint8_t on_off = 0x00; /* Ť�ز�����0Ϊ�رգ�1Ϊ���� */
    if (enable)
    {
        on_off = 0x01;
    }
    uint8_t head1 = 0xff;
    uint8_t head2 = 0xff;
    uint8_t s_id = 0xfe; /* ���͹㲥��ID */
    uint8_t len = 0x04;
    uint8_t cmd = 0x03;
    uint8_t addr = 0x28;

    uint8_t checknum = (~(s_id + len + cmd + addr + on_off)) & 0xff;
    uint8_t data[] = {head1, head2, s_id, len, cmd,
                      addr, on_off, checknum};
			HAL_UART_Transmit(&huart4,data,sizeof(data),100);
}

/* д��Ŀ��ID(1~250) */
void UartServo_Set_ID(uint8_t id)
{
    if ((id >= 1) && (id <= 250))
    {
        uint8_t head1 = 0xff;
        uint8_t head2 = 0xff;
        uint8_t s_id = 0xfe; /* ���͹㲥��ID */
        uint8_t len = 0x04;
        uint8_t cmd = 0x03;
        uint8_t addr = 0x05;
        uint8_t set_id = id; /* ʵ��д���ID */

        uint8_t checknum = (~(s_id + len + cmd + addr + set_id)) & 0xff;
        uint8_t data[] = {head1, head2, s_id, len, cmd,
                          addr, set_id, checknum};
        HAL_UART_Transmit(&huart4,data,sizeof(data),100);
    }
}

/*****************************������*********************************/
/* ��ȡ�����ǰ�Ƕ� */
void UartServo_Get_Angle(uint8_t id)
{
	//printf("UartServo_Get_Angle\r\n");
    uint8_t head1 = 0xff;
    uint8_t head2 = 0xff;
    uint8_t s_id = id & 0xff;
    uint8_t len = 0x04;
    uint8_t cmd = 0x02;
    uint8_t param_H = 0x38;
    uint8_t param_L = 0x02;

    uint8_t checknum = (~(s_id + len + cmd + param_H + param_L)) & 0xff;
    uint8_t data[] = {head1, head2, s_id, len, cmd, param_H, param_L, checknum};
    if(HAL_UART_Transmit(&huart4,data,sizeof(data),100)!=HAL_OK)
			printf("HAL_UART_Transmit error\r\n");
	

//		HAL_Delay(50);	
}

/* ��ȡ���߶����־λ */
uint8_t UartServo_Get_Flag()
{
     return RecvFlag;
}
/* ������߶����־λ�ͻ������� */
void UartServo_Clear_Flag()
{
    RecvFlag = 0;
}




// ���ʹ��ڶ����λ�����ݵ�����
uint8_t  buf[20];

void UartServo_Send_ARM_Angle(uint8_t id, uint16_t value)
{
		
		buf[0] = 0x5a;
		buf[1] = 9;
		buf[2] = 0x01;
    uint16_t send_value = value;
    send_value = value - UartServo_Get_Median_Offset(id);
    buf[3] = 0x48;     // ������
    buf[4] = id;                  // ID
		buf[5] = (send_value >> 8) & 0xff; // λ�ø�λ
    buf[6] = send_value & 0xff;        // λ�õ�λ 
		buf[7] = 0x00;
	
		
		buf[8] = crc8_chk_value(buf,8);
		vSendCommand(buf,buf[1]);
}

// ���ͻ�е�۵������ؽ�λ�õ���λ��
void UartServo_Send_ARM_Angle_Array(void)
{
	//printf("UartServo_Send_ARM_Angle_Array\r\n");
		buf[0] = 0x5a;
		buf[1] = 18;
		buf[2] = 0x01;
		 uint16_t send_val[6] = {0};
		 int i;
    for ( i = 0; i < 6; i++)
    {
        send_val[i] = g_arm_read_value[i] - UartServo_Get_Median_Offset(i+1);
    }
    buf[3] = 0x4a;              // ������
    buf[4] = (send_val[0] >> 8) & 0xff;  // λ�ø�λ
    buf[5] = send_val[0] & 0xff;         // λ�õ�λ  
		
    buf[6] = (send_val[1] >> 8) & 0xff;  // λ�ø�λ
    buf[7] = send_val[1] & 0xff;         // λ�õ�λ  
    buf[8] = (send_val[2] >> 8) & 0xff;  // λ�ø�λ
    buf[9] = send_val[2] & 0xff;         // λ�õ�λ  
    buf[10] = (send_val[3] >> 8) & 0xff; // λ�ø�λ  
    buf[11] = send_val[3] & 0xff;        // λ�õ�λ
    buf[12] = (send_val[4] >> 8) & 0xff; // λ�ø�λ
    buf[13] = send_val[4] & 0xff;        // λ�õ�λ  
    buf[14] = (send_val[5] >> 8) & 0xff; // λ�ø�λ 
    buf[15] = send_val[5] & 0xff;        // λ�õ�λ
		buf[16] = 0x00;
		buf[17] = crc8_chk_value(buf,17);
		vSendCommand(buf,buf[1]);
}

// ��մ��ڶ�ȡ��е�۵Ļ�������, ����Ϊ��λƫ���ֵ��������������ݷ������ݲ���0��
void UartServo_Clear_Arm_Read_Vlaue(void)
{
    for (int i = 0; i < 6; i++)
    {
        // g_arm_read_value[i] = 0;
        g_arm_read_value[i] = UartServo_Get_Median_Offset(i+1);
    }
}





/* ��ȡ��λƫ���ȫ�ֱ�����ֵ */
int16_t UartServo_Get_Median_Offset(uint8_t id)
{
    int16_t value = 0;
    if ((id >= 1) && (id <= MAX_SERVO_NUM))
    {
        if (id == 5)
        {
            value = g_arm_median_value[id-1] - MID_VAL_ID5;
        }
        else if (id == 6)
        {
            value = g_arm_median_value[id-1] - MID_VAL_ID6;
        }
        else
        {
            value = g_arm_median_value[id-1] - MEDIAN_VALUE;
        }
    }
    return value;
}
int8_t UartServo_Revice(uint8_t* pucCommandBuf,uint8_t ucCommandLen1)
{
	int8_t ucCommandLen=ucCommandLen1;
	while(ucCommandLen>=8)
	{
		if(pucCommandBuf[0] != 0xff&&pucCommandBuf[1] != 0xf5&&pucCommandBuf[2]!=0x4){
			ucCommandLen--;
			pucCommandBuf++;
			continue ;
			}
		memcpy(Rx_Data,pucCommandBuf,8);
		RecvFlag = FLAG_RECV;
			break;
	}
	if(ucCommandLen>=0)
		return 0;
    
	return  -1;
}

/* ������������, flag 1 Ϊ��������� 2Ϊ���������������, ���ͳɹ�����1�� ���򷵻�0 */
uint8_t UartServo_Rx_Parse(uint8_t flag)
{
	uint8_t result = 0;
			if(UartServo_Get_Flag() == FLAG_RECV)
			{
				UartServo_Clear_Flag();
				uint8_t checknum = (~(Rx_Data[2] + Rx_Data[3] + Rx_Data[4] + Rx_Data[5] + Rx_Data[6])) & 0xff;
				if (checknum == Rx_Data[7])
				{
					result=1;
						uint8_t s_id = Rx_Data[2];
						 uint16_t read_value = Rx_Data[5] << 8 | Rx_Data[6];
						if (flag == 2) // ���浽��е�۶�ȡ���нǶȵ���ʱֵ
							{
									if (s_id > 0 && s_id <= 6)
									{
											g_arm_read_value[s_id-1] = read_value;
									}
							}
						else // ���͵�����
						 {
									UartServo_Send_ARM_Angle(s_id, read_value);
						 }
			
					 }
				
			}
			

    return result ;
}




