//this is Ackerman

#include "mainloop.h"
#include "APP/uart_servo.h"
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart4;

static uint8_t ucSelfTest(uint8_t ucTestMode);

//TODO TEST
#include "BSP/bsp_servo.h"
/*set print */
int fputc(int ch, FILE *f)
{      
	HAL_UART_Transmit(&huart5, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}

uint8_t pucSoftwareVersion[3]={SOFTWARE_MAIN_VERSION,SOFTWARE_SUBVERSION,SOFTWARE_PATCHLEVEL};
uint8_t pucHardwareVersion[3]={HARDWARE_MAIN_VERSION,HARDWARE_SUBVERSION,HARDWARE_PATCHLEVEL};
//uart1
volatile uint8_t ucUart1RxBufLen = 0;
volatile uint8_t ucUart1RecvEndCounter = 0;
uint8_t pucUart1RxBuf[UART_RX_BUF_SIZE] = {0};
uint8_t pucUart1RxBufCache[UART_RX_BUF_SIZE] = {0};

uint8_t pucUart1TxBuf[UART_TX_BUF_SIZE] = {0};
uint8_t pucUart1TxBufCache[UART_TX_BUF_SIZE] = {0};
volatile uint16_t usUart1TxCacheCounter = 0;
volatile uint8_t ucUart1TxDMAOverFlag = 1;


//uart2
volatile uint8_t ucUart2RxBufLen = 0;
volatile uint8_t ucUart2RecvEndCounter = 0;
uint8_t pucUart2RxBuf[UART_RX_BUF_SIZE] = {0};
uint8_t pucUart2RxBufCache[UART_RX_BUF_SIZE] = {0};

uint8_t pucUart2TxBuf[UART_TX_BUF_SIZE] = {0};
uint8_t pucUart2TxBufCache[UART_TX_BUF_SIZE] = {0};
volatile uint16_t usUart2TxCacheCounter = 0;
volatile uint8_t ucUart2TxDMAOverFlag = 1;


//uart4
volatile uint8_t ucUart4RxBufLen = 0;
volatile uint8_t ucUart4RecvEndCounter = 0;
uint8_t pucUart4RxBuf[UART_RX_BUF_SIZE] = {0};
uint8_t pucUart4RxBufCache[UART_RX_BUF_SIZE] = {0};

uint8_t pucUart4TxBuf[UART_TX_BUF_SIZE] = {0};
uint8_t pucUart4TxBufCache[UART_TX_BUF_SIZE] = {0};
volatile uint16_t usUart4TxCacheCounter = 0;
volatile uint8_t ucUart4TxDMAOverFlag = 1;

float pfGyroZBuf[6] = {0.0};
float fGyroZ = 0.0;

uint32_t puiUID[3] = {0};
uint8_t ucCommandSendErrorFlag = 0;
uint8_t ucCommandSendOverFlag = 1;
bool bUartUsed = true;

extern volatile uint32_t uiLastVelCmdTick;

static bool bTestMode = false;

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART1){
		ucCommandSendErrorFlag = 1;
	}
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART1){
		HAL_UART_DMAStop(&huart1);
		ucCommandSendOverFlag = 1;
	}
	if(huart->Instance == USART2){
		HAL_UART_DMAStop(&huart2);
	}
	if(huart->Instance == UART4){
		HAL_UART_DMAStop(&huart4);
	}
}
/**

*/
void vSendCommand(uint8_t* pucSendBuf,uint8_t ucSendBufLen){

	if(bUartUsed){
		if(usUart1TxCacheCounter + ucSendBufLen < UART_TX_BUF_SIZE){
			memcpy(pucUart1TxBufCache+usUart1TxCacheCounter,pucSendBuf,ucSendBufLen);
			usUart1TxCacheCounter += ucSendBufLen;
		}
		else{
//			Errorprint("TX Cache is Full %d %d %d %d",ucSendBufLen,usUart1TxCacheCounter,ucUart1TxDMAOverFlag,__HAL_DMA_GET_COUNTER(&hdma_usart1_tx));
		}
	}
}

uint32_t uiimu_calibrate = 0;

/**

*/
void imu_calibrate(void)
{
	uint8_t temp_buf[5] = {0x00,0x06,0x05,0x57,0x62};
	memcpy(pucUart2TxBuf,temp_buf,5);
	HAL_UART_Transmit_DMA(&huart2,pucUart2TxBuf,5);
}
/**

*/
void ahrs_reset(void)
{
	uint8_t temp_buf[5] = {0x00,0x06,0x05,0x56,0x61};
	memcpy(pucUart2TxBuf,temp_buf,5);
	HAL_UART_Transmit_DMA(&huart2,pucUart2TxBuf,5);
}
/**
set imu module report gap 10ms
*/
void ahrs_gap_set(void)
{
	uint8_t temp_buf[5] = {0x00,0x06,0x02,0x0a,0x12};
	memcpy(pucUart2TxBuf,temp_buf,5);
	HAL_UART_Transmit_DMA(&huart2,pucUart2TxBuf,5);
}



/**

*/
void vUartLoopSend(void){
	if((usUart1TxCacheCounter>0) && (ucUart1TxDMAOverFlag)){
		memcpy(pucUart1TxBuf,pucUart1TxBufCache,usUart1TxCacheCounter);
		if(HAL_OK != HAL_UART_Transmit_DMA(&huart1,pucUart1TxBuf,usUart1TxCacheCounter)){
			printf("DMA Faild %d \r\n",HAL_UART_Transmit_DMA(&huart1,pucUart1TxBuf,usUart1TxCacheCounter));
		}
		usUart1TxCacheCounter = 0;
		ucUart1TxDMAOverFlag = 0;
	}
}


/**
Main Loop Function
*/
void vMainLoop(void){
	uint32_t uiMotorLastContorTick = 0;
	uint32_t uiPS2LastCheckTick = 0;
	uint32_t ucImuLastReadTick = 0;
	uint32_t uiBatteryCurrentLastCheckTick = 0;
	uint32_t uiBatteryVoltageLastCheckTick = 0;
	uint32_t uiLastRecvProtocolTick = 0;
	uint32_t uiLastIndicateTick = 0;
	uint32_t uiSonarCheckTick = 0;
	#ifdef DEBUG_MODE
	uint32_t uiLastDebugTick = 0;
	#endif
	uint32_t uiKeyOneCounter = 0;
	bool bImuReadyFlag = true;
	bool bUartConnectionFlag = false;
	bool bUartLostConnectionMark = false;
	bool bUartEstablishConnectionMark = false;
	bool bBatteryLowFlag = false;
	bool bBatteryStateChangeFlag = false;
	bool bPS2OperateFlag = false;
	uint8_t ucTakeCacheResult = 0;
	bool bGyroStable = false;
		
	uint8_t config_1,config_2;
	if(HAL_GPIO_ReadPin(SWITCH1_GPIO_Port,SWITCH1_Pin) == GPIO_PIN_SET){
		config_1 = 0;
	}
	else{
		config_1 = 1;
	}
	if(HAL_GPIO_ReadPin(SWITCH2_GPIO_Port,SWITCH2_Pin) == GPIO_PIN_SET){
		config_2 = 0;
	}
	else{
		config_2 = 1;
	}
	vSetMotorConfig(config_1+config_2); 
	vSetKinematicsConfig(config_1,config_2);
	if(config_2 == 0){
		bGyroStable = true;
	}
	vServoStart(&xServo);	
	vMotorControlInit();
	vIndicateInit();
	vPS2Init();
	vInitCommandCache();

	extern int8_t cEncoderDir;
	extern uint8_t ucMotorType;
	printf("%d %d %d %d\r\n",config_1,config_2,ucMotorType ,cEncoderDir);
	extern uint8_t ucKinematicsType;
	printf("%f %f %f %f %f %f %d\r\n",fWheelThead,fWheelBase,fWheelDiameter,fMaxXVelocity,fMaxYVelocity,fMaxZAngle,ucKinematicsType);
	
	//Read Chip UID
	puiUID[0] = HAL_GetUIDw0();
	puiUID[1] = HAL_GetUIDw1();
	puiUID[2] = HAL_GetUIDw2();	
	ahrs_gap_set();
	HAL_Delay(1000);	
	
	//Uart1 Relative 
	__HAL_UART_CLEAR_IDLEFLAG(&huart1);
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
	__HAL_UART_CLEAR_IDLEFLAG(&huart1);

	ucUart1RecvEndCounter = 0;
	ucUart1RxBufLen = 0;
	HAL_UART_Receive_DMA(&huart1,pucUart1RxBuf,UART_RX_BUF_SIZE);
	
	//Uart2 Relative 
	__HAL_UART_CLEAR_IDLEFLAG(&huart2);
	__HAL_UART_ENABLE_IT(&huart2,UART_IT_IDLE);
	__HAL_UART_CLEAR_IDLEFLAG(&huart2);

	ucUart2RecvEndCounter = 0;
	ucUart2RxBufLen = 0;
	HAL_UART_Receive_DMA(&huart2,pucUart2RxBuf,UART_RX_BUF_SIZE);
	
	//Uart4 Relative 
	__HAL_UART_CLEAR_IDLEFLAG(&huart4);
	__HAL_UART_ENABLE_IT(&huart4,UART_IT_IDLE);
	__HAL_UART_CLEAR_IDLEFLAG(&huart4);

	ucUart4RecvEndCounter = 0;
	ucUart4RxBufLen = 0;
	HAL_UART_Receive_DMA(&huart4,pucUart4RxBuf,20);
	
	vSetIndicate(LEDG,0.5,0);

	vSetMotorAccelDecel(usConvertAcc2MotorAcc(3.0));
//	UartServo_Ctrl(1, 2048, 1000);
	uint32_t uiSilenceTick = 0;
	
/*	UartServo_Ctrl(1,2048,1000);
	HAL_Delay(1000);	
	UartServo_Ctrl(2,2048,1000);
	HAL_Delay(1000);	
	UartServo_Ctrl(3,2048,1000);
	HAL_Delay(1000);	
	UartServo_Ctrl(4,2048,1000);
	HAL_Delay(1000);	*/
	while(1){
		//UartServo_Ctrl(1,2048,1000);
		/*execute indicat device*/
		if(HAL_GetTick()-uiLastIndicateTick > INDICATE_PERIOD){
			vIndicate();	/*Indicate,include RGB-LED,BUZZER*/		
			uiLastIndicateTick = HAL_GetTick();
		}
		/*establish connection*/
		if(bUartEstablishConnectionMark){
			vSetIndicate(LEDG,2,0);
			/*init IMU to set data to zero when first establish connection*/
			if(bImuReadyFlag){
//				ahrs_reset();
				uiimu_calibrate = HAL_GetTick();
				uiSilenceTick = HAL_GetTick()+ 500;
			}
			printf("establish connection \r\n");
			vSetIndicate(BEEP,0.5,4000);/*TODO:need beep 2S here,this methode  may not a good idea*/
			bUartConnectionFlag = true;
			bUartEstablishConnectionMark = false;
		}
		/*connection lost*/
		if(bUartLostConnectionMark){
			printf("connection lost\r\n");
			vSetIndicate(LEDG,0.5,0);
			vSetIndicate(BEEP,2,3000);
			bUartConnectionFlag = false;
			bUartLostConnectionMark = false;
		}
		/*battery low*/
		if(bBatteryStateChangeFlag){
			bBatteryStateChangeFlag = false;
			if(bBatteryLowFlag){
				vSetIndicate(LEDB,2,0);
				vSetIndicate(LEDG,0,0);
				vSetIndicate(BEEP,0.5,0);
			}
			else{
				vSetIndicate(LEDB,0,0);
				vSetIndicate(BEEP,0,0);				
				if(bImuReadyFlag){
					if(bUartConnectionFlag){
						vSetIndicate(LEDG,2,0);
					}
					else{
						vSetIndicate(LEDG,0.5,0);
					}
				}
				else{
					vSetIndicate(BEEP,1,0);
					vSetIndicate(LEDR,0.5,0);
				}		
			}
		}
		/*uartservo received data*/
/*		if(ucUart4RecvEndCounter != 0){
			ucUart4RecvEndCounter = 0;
			printf("uartservo received data\r\n");
			UartServo_Revice(pucUart4RxBufCache,ucUart4RxBufLen);
			int i=0;
			for(i=0;i<ucUart4RxBufLen;i++)
				printf("%x ",pucUart4RxBufCache[i]);
			printf("\r\nuartservo finsih data\r\n");
			ucUart4RxBufLen = 0;
			
		}*/
		
		/*uart received data*/
		if(ucUart1RecvEndCounter != 0){
			ucUart1RecvEndCounter = 0;
//			ucRxCounter+=ucUart1RxBufLen;
			ucSaveCommandToCache(ucUart1RxBufLen,pucUart1RxBufCache);
			memset(pucUart1RxBufCache,0,UART_RX_BUF_SIZE);
			ucUart1RxBufLen = 0;
			bUartUsed = true;
		}
		vUartLoopSend();
		/*take data from cache*/
		ucTakeCacheResult = usTakeCommandFromCache();
		while(ucTakeCacheResult==0){	/*take cache until cache is empty*/
			if(bUartConnectionFlag == false){	/*change connection mark when connect state form disconn switch to connect*/
				bUartEstablishConnectionMark = true;
			}
			uiLastRecvProtocolTick = HAL_GetTick();
			ucCommandInterpretation(ucCommandTempLen,pucCommandTempBuf);
			ucTakeCacheResult = usTakeCommandFromCache();
		}
		/* */
		if(ucTakeCacheResult > 1){
//			Errorprint("Take Cache Error %d %d %d %d ",ucTakeCacheResult,uiGetCacheCounter,uiTakeCacheCounter,HAL_GetTick());
		}
		/*uart2 received data*/
		if(ucUart2RecvEndCounter != 0){
			ucUart2RecvEndCounter = 0;
			if(bParseImuData(pucUart2RxBufCache,ucUart2RxBufLen)){
				
				xImu.fPitch = (float)imu_data.pitch/100.0f;
				xImu.fRoll = (float)imu_data.roll/100.0f;
				xImu.fYaw = (float)imu_data.yaw/100.0f;
				xImu.pfEuler[0] = xImu.fPitch ;
				xImu.pfEuler[1] = xImu.fRoll ;
				xImu.pfEuler[2] = xImu.fYaw ;
				
				xImu.pfAccel[0] = imu_data.acc[0]*accel_dps2accel;
				xImu.pfAccel[1] = imu_data.acc[1]*accel_dps2accel;
				xImu.pfAccel[2] = imu_data.acc[2]*accel_dps2accel;
				xImu.pfGyro[0] = imu_data.gyro[0]*gyro_dps2rad;
				xImu.pfGyro[1] = imu_data.gyro[1]*gyro_dps2rad;
				xImu.pfGyro[2] = imu_data.gyro[2]*gyro_dps2rad;
				xImu.pfQuat[0] = imu_data.quat[0]*0.0001f;
				xImu.pfQuat[1] = imu_data.quat[1]*0.0001f;
				xImu.pfQuat[2] = imu_data.quat[2]*0.0001f;
				xImu.pfQuat[3] = imu_data.quat[3]*0.0001f;
				xImu.ftemperature = imu_data.temp*0.01;
				fGyroZ = xImu.pfGyro[2];
				pfGyroZBuf[5] = xImu.pfGyro[2];
				for(uint8_t i = 0;i < 5;i++)
				{
					pfGyroZBuf[i] = pfGyroZBuf[i+1];
					fGyroZ += pfGyroZBuf[i];
				}
				fGyroZ /= 5;
				ucImuLastReadTick = HAL_GetTick();
			}
			memset(pucUart2RxBufCache,0,UART_RX_BUF_SIZE);
			ucUart2RxBufLen = 0;
		//	printf("%f %f %f  %f \r\n",	xImu.pfQuat[0],xImu.pfQuat[1],xImu.pfQuat[2],xImu.pfQuat[3]);
		}	
		if(ucUart4RecvEndCounter != 0){
			ucUart4RecvEndCounter = 0;
			if(bParseImuData(pucUart4RxBufCache,ucUart4RxBufLen)){
				
				
			}
	
			ucUart2RxBufLen = 0;
	
		}	
		/*motor control*/
		if((HAL_GetTick() - uiMotorLastContorTick > MOTOR_CONTROL_PERIOD)){
			vGetMotorSpeed(&xMotor1);
			vGetMotorSpeed(&xMotor2);			
//			vGetMotorSpeed(&xMotor3);
//			vGetMotorSpeed(&xMotor4);	
		if(!bTestMode){
			vGetSpeed(&xVelocity);
			vExecuteMotorSpeed(&xMotor1,fMotorPidCompute(&xMotor1));
			vExecuteMotorSpeed(&xMotor2,fMotorPidCompute(&xMotor2));
//			vExecuteMotorSpeed(&xMotor3,fMotorPidCompute(&xMotor3));
//			vExecuteMotorSpeed(&xMotor4,fMotorPidCompute(&xMotor4));	
			uiMotorLastContorTick = HAL_GetTick();						
			if(ucBaseType == BASE_ACKERMAN){
				vSetServoAngle(&xServo,bGyroStable);	
				xServo.fCurrentAngle = CONVERT_OMEGA2ANGLE((xVelocity.fAngleZ),(xVelocity.fLineX),fWheelBase);
				xServo.fVelocity = xVelocity.fLineX;
			}		
		}		

		}		
		/* check last communication time*/
		if((HAL_GetTick()-uiLastRecvProtocolTick>DISCONNECT_WATI_TIME)&&bUartConnectionFlag){
			bUartLostConnectionMark = true;
		}		
		/*PS2 Control Read*/
		if(HAL_GetTick() - uiPS2LastCheckTick > PS2_CHECK_PERIOID){
			uiPS2LastCheckTick = HAL_GetTick();
			bPS2OperateFlag = bReadPS2Data();
		
		}		
		/*battery current check*/
		if(HAL_GetTick() - uiBatteryCurrentLastCheckTick > BATTERY_CURRENT_CHECK_PERIOD){	
			uiBatteryCurrentLastCheckTick = HAL_GetTick();
			static uint16_t usTempCounter = 0;
			vGetBatteryCurrentInfo(&xBattery);
			usTempCounter++;
			if(usTempCounter>500){
				usTempCounter = 0;
			}
		}		
		/*battery voltage check*/
		if(HAL_GetTick() - uiBatteryVoltageLastCheckTick > BATTERY_VOLTAGE_CHECK_PERIOD){	
			uiBatteryVoltageLastCheckTick = HAL_GetTick();
			vGetBatteryVoltageInfo(&xBattery);
			static uint8_t prvucCounter = 0;
			if(xBattery.fVoltage < BATTERY_LOW_VOLTAGE){
				if(bBatteryLowFlag==false)
					prvucCounter++;
			}
			else{
				if(prvucCounter>0){
					prvucCounter = 0;
				}
				if(bBatteryLowFlag){
					if(xBattery.fVoltage > BATTERY_LOW_VOLTAGE + VOLTAGE_STATE_CHANGE_GAP){
						bBatteryStateChangeFlag = true;
						bBatteryLowFlag  =false;
					}
				}
			}
			if(prvucCounter > 3){
				bBatteryStateChangeFlag = true;
				bBatteryLowFlag = true;
			}			
		}			
		
		/*No Control Source Check*/
		if(HAL_GetTick() - uiLastVelCmdTick > VELOCITY_LAST_TIME  && bPS2OperateFlag == false){
			vComputeMotorSpeed(0.0,0.0,0.0);
		}	
		/*Sonar data check */
		#if SONAR_CHANNEL_NUM > 0
		if((HAL_GetTick()-uiSonarCheckTick > SONAR_CHECK_PERIOD)){
			if(ucSonarGroupNum == 1 && SONAR_CHANNEL_NUM > 1){
				vTrigSonar(2);
			}
			else{
				vTrigSonar(1);
			}
			uiSonarCheckTick  = HAL_GetTick();
//			printf("%d %d %d %d %d \r\n",uiSonarus[0],uiSonarus[1],uiSonarus[2],uiSonarus[3],SysTick->LOAD);
		}	
		#endif	
		#ifdef DEBUG_MODE
		/* debug info output*/
		if(HAL_GetTick() - uiLastDebugTick > DEBUG_PERIOD){		
			uiLastDebugTick = HAL_GetTick();	
		}
		#endif	
		/*check imu calibrate*/
		if(uiimu_calibrate!=0 && HAL_GetTick()>uiimu_calibrate)
		{
			imu_calibrate();
			uiimu_calibrate = 0;
		}		
		/* check key press Action*/
		static uint8_t ucPressCounter = 0;
		static bool bKeyRelease = true;
		if(HAL_GPIO_ReadPin(KEY_1_GPIO_Port,KEY_1_Pin) == GPIO_PIN_RESET){
			if(bKeyRelease){
				uiKeyOneCounter = HAL_GetTick();
				ucPressCounter++;
				bKeyRelease = false;
			}
		}
		else{
			if(HAL_GetTick() - uiKeyOneCounter > 300 && uiKeyOneCounter){
				switch (ucPressCounter){
					case 1:
//						Debugprint("Single Click\r\n");
						break;
					case 2:
//						Debugprint("Double Click\r\n");
						uiimu_calibrate = HAL_GetTick()+3000;
						break;
					case 3:
//						Debugprint("Triple Click\r\n");
						bTestMode = true;
						vSetIndicate(BEEP,10,200);
						break;
					default:
//						Debugprint("Unknow Click %d \r\n",ucPressCounter);
						break;
				}
				ucPressCounter = 0;
				uiKeyOneCounter = 0;
				bKeyRelease = true;
			}
			bKeyRelease = true;
		}	
		/*Test Mode*/
		if(bTestMode){
			static uint8_t ucTestResult = 0;
			ucTestResult = ucSelfTest(0);
			if(ucTestResult){
				printf("Test Result %d",ucTestResult);
				vSetIndicate(BEEP,ucTestResult,0);
			}
		}		
	}
}

/**
return value:
1 : imu error
4 : bat senser error
10 : Motor or encoder error
*/
static uint8_t ucSelfTest(uint8_t ucTestMode){
	static uint8_t ucTastItem = 0;
	static uint32_t uiTestTick = 0;
	static uint16_t usTestGap = 0;
	static int32_t iDeltaEncoder[2];
	if(HAL_GetTick() - uiTestTick > usTestGap){
		switch (ucTastItem){
			case 0:{
				uiTestTick = HAL_GetTick();
				usTestGap = 10;
				ucTastItem++;
				printf("ACCEL %f %f %f",xImu.pfAccel[0],xImu.pfAccel[1],xImu.pfAccel[2]);
				if(((xImu.pfAccel[0]==0.0)&&(xImu.pfAccel[1]==0.0))||(fabs(xImu.pfAccel[2]-10))>5){
					return ucTastItem;
				}
				break;
			}
			case 1:{
				uiTestTick = HAL_GetTick();
				usTestGap = 10;
				ucTastItem++;			
//				Debugprint("GYRO %f %f %f",xImu.pfGyro[0],xImu.pfGyro[1],xImu.pfGyro[2]);
				break;
			}
			case 2:{
				uiTestTick = HAL_GetTick();
				usTestGap = 10;
				ucTastItem++;			
//				Debugprint("Euler %f %f %f",xImu.pfEuler[0],xImu.pfEuler[1],xImu.pfEuler[2]);
				break;
			}
			case 3:{
				uiTestTick = HAL_GetTick();
				usTestGap = 100;
				ucTastItem++;			
//				Debugprint("BAT %f %f",xBattery.fVoltage ,xBattery.fCurrent);
//				if((xBattery.fVoltage<7.0) || (xBattery.fVoltage > 13.0) || (xBattery.fCurrent < 0.01) || (xBattery.fCurrent > 3.0)){
//					return ucTastItem;
//				}
				break;
			}
			extern void vSetServoPwm(servo_t ucServoID,uint16_t usPwm);
			case 4:{
				uiTestTick = HAL_GetTick();
				usTestGap = 500;
				ucTastItem++;	
				vSetServoPwm(0,1300);
				break;				
			}				
			case 5:{
				uiTestTick = HAL_GetTick();
				usTestGap = 1000;
				ucTastItem++;	
				vSetServoPwm(0,1700);
				break;				
			}	
			case 6:{
				uiTestTick = HAL_GetTick();
				usTestGap = 500;
				ucTastItem++;	
				vSetServoPwm(0,1500);
				break;				
			}		
			case 7:{
				uiTestTick = HAL_GetTick();
				usTestGap = 1000;
				ucTastItem++;	
//				Debugprint("CW Start Encoder:%d %d",xMotor1.iLastEncoderReadValue,xMotor2.iLastEncoderReadValue);
				iDeltaEncoder[0] = xMotor1.iLastEncoderReadValue;
				iDeltaEncoder[1] = xMotor2.iLastEncoderReadValue;			
				vSetMotorPWM(MOTOR1,300);
				vSetMotorPWM(MOTOR2,300);
				break;
			}			
			case 8:{
				uiTestTick = HAL_GetTick();
				usTestGap = 100;
				ucTastItem++;	
				vSetMotorPWM(MOTOR1,0);
				vSetMotorPWM(MOTOR2,0);
//				Debugprint("CW End Encoder:%d %d",xMotor1.iLastEncoderReadValue,xMotor2.iLastEncoderReadValue);
				iDeltaEncoder[0] -= xMotor1.iLastEncoderReadValue;
				iDeltaEncoder[1] -= xMotor2.iLastEncoderReadValue;
				break;				
			}	
			case 9:{
				uiTestTick = HAL_GetTick();
				usTestGap = 100;
				ucTastItem++;	
//				Debugprint("CW Delta Encoder:%d %d",iDeltaEncoder[0],iDeltaEncoder[1]);
				if((iDeltaEncoder[0] > 0) && (iDeltaEncoder[1] > 0)){ //Both two encoder equip
					static float fEncoderScale;
					fEncoderScale = ((float)iDeltaEncoder[0])/((float)iDeltaEncoder[1]);
					if((fEncoderScale>1.1f) || (fEncoderScale<0.9f)){
						return ucTastItem;
					}
				}
				else if((iDeltaEncoder[0]+iDeltaEncoder[1]!=0)&&(iDeltaEncoder[0]*iDeltaEncoder[1]==0)){ //only one encoder equip and another missed
					return ucTastItem;
				}
				else; //Both two encoder NOT equip,we allow this case in board test mode
				break;				
			}			
			case 10:{
				uiTestTick = HAL_GetTick();
				usTestGap = 1000;
				ucTastItem++;	
//				Debugprint("CCW Start Encoder:%d %d",xMotor1.iLastEncoderReadValue,xMotor2.iLastEncoderReadValue);
				iDeltaEncoder[0] = xMotor1.iLastEncoderReadValue;
				iDeltaEncoder[1] = xMotor2.iLastEncoderReadValue;			
				vSetMotorPWM(MOTOR1,-300);
				vSetMotorPWM(MOTOR2,-300);
				break;
			}			
			case 11:{
				uiTestTick = HAL_GetTick();
				usTestGap = 100;
				ucTastItem++;	
				vSetMotorPWM(MOTOR1,0);
				vSetMotorPWM(MOTOR2,0);
//				Debugprint("CCW End Encoder:%d %d",xMotor1.iLastEncoderReadValue,xMotor2.iLastEncoderReadValue);
				iDeltaEncoder[0] = xMotor1.iLastEncoderReadValue - iDeltaEncoder[0];
				iDeltaEncoder[1] = xMotor2.iLastEncoderReadValue - iDeltaEncoder[1];
				break;				
			}	
			case 12:{
				uiTestTick = HAL_GetTick();
				usTestGap = 100;
				ucTastItem++;	
//				Debugprint("CCW Delta Encoder:%d %d",iDeltaEncoder[0],iDeltaEncoder[1]);
				if((iDeltaEncoder[0] > 0) && (iDeltaEncoder[1] > 0)){ //Both two encoder equip
					static float fEncoderScale;
					fEncoderScale = ((float)iDeltaEncoder[0])/((float)iDeltaEncoder[1]);
					if((fEncoderScale>1.1f) || (fEncoderScale<0.9f)){
						return 10;
					}
				}
				else if((iDeltaEncoder[0]+iDeltaEncoder[1]!=0)&&(iDeltaEncoder[0]*iDeltaEncoder[1]==0)){ //only one encoder equip and another missed
					return 10;
				}
				else; //Both two encoder NOT equip,we allow this case in board test mode
				break;				
			}					
			default:{
				bTestMode = false;
				ucTastItem = 0;
				usTestGap = 0;
				break;
			}
		}	
	}
	return 0;
}



