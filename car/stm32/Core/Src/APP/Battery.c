#include "APP/Battery.h"
#include "APP/config.h"

#define ADC_FULL	4096
#define ADC_VOLTAGE	3.3f
#define ADC_REF_INIT	(1.2f*4096/3.3f)

#define VOLTAGE_FILTER_LENGTH	5
#define CURRENT_FILTER_LENGTH	24

extern ADC_HandleTypeDef hadc3;

Battery_t xBattery;

uint16_t pusVoltageFilter[VOLTAGE_FILTER_LENGTH] = {0};
uint16_t pusCurrentFilter[CURRENT_FILTER_LENGTH] = {0};
static float fADCRefScale = 1.0;
static uint16_t ucADCRefValue = (uint16_t)ADC_REF_INIT;
float fErrorScale = 1.0;
uint8_t ucUseErrorScale = 1;

static uint16_t  usGetADC3Value(uint32_t ch){
	ADC_ChannelConfTypeDef ADC1_ChanConf;
	ADC1_ChanConf.Channel = ch;                                   //通道
	ADC1_ChanConf.Rank=1;                                       //第1个序列，序列1
	ADC1_ChanConf.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;      //采样时间    
	HAL_ADC_ConfigChannel(&hadc3,&ADC1_ChanConf);        //通道配置
	HAL_ADC_Start(&hadc3);                               //开启ADC
	HAL_ADC_PollForConversion(&hadc3,10);                //轮询转换
	return (uint16_t)HAL_ADC_GetValue(&hadc3);	        	//返回最近一次ADC1规则组的转换结果
}  

void vADCRefInit(void){
	uint32_t uiTemp = 0;
	uint16_t usTemp = 0;
	/*triple measure to get more accurace value*/
	for(uint8_t i=0;i<32;i++){
		usTemp = usGetADC3Value(ADC_CHANNEL_17);
		uiTemp += usTemp;
	}
	fADCRefScale = (float)ucADCRefValue*32.0f/(float)uiTemp;
}

float fGetVoltage(void){
	uint16_t uiADCValue = 0;
	uint16_t usADCValueSum = 0;
	float fVoltage = 0.0;
	uint16_t uiMaxADCValue,uiMinADCValue;
	uiMinADCValue = 4096;
	uiMaxADCValue = 0;
	uiADCValue = usGetADC3Value(VOLTAGE_ADC_CHANNEL);
	for(uint8_t i = 0;i<VOLTAGE_FILTER_LENGTH-1;i++){
		if(pusVoltageFilter[i+1] == 0)
			pusVoltageFilter[i+1] = uiADCValue;
		pusVoltageFilter[i] = pusVoltageFilter[i+1];
		usADCValueSum += pusVoltageFilter[i];
		if(pusVoltageFilter[i] > uiMaxADCValue)	uiMaxADCValue = pusVoltageFilter[i];
		if(pusVoltageFilter[i] < uiMinADCValue)	uiMinADCValue = pusVoltageFilter[i];		
	}
	pusVoltageFilter[VOLTAGE_FILTER_LENGTH-1] = uiADCValue;
	usADCValueSum += pusVoltageFilter[VOLTAGE_FILTER_LENGTH-1];
	
	if(pusVoltageFilter[VOLTAGE_FILTER_LENGTH-1] > uiMaxADCValue)	uiMaxADCValue = pusVoltageFilter[VOLTAGE_FILTER_LENGTH-1];
	if(pusVoltageFilter[VOLTAGE_FILTER_LENGTH-1] < uiMinADCValue)	uiMinADCValue = pusVoltageFilter[VOLTAGE_FILTER_LENGTH-1];
	usADCValueSum -= (uiMaxADCValue + uiMinADCValue);
	uiADCValue = usADCValueSum/(VOLTAGE_FILTER_LENGTH-2);
	
	fVoltage = ADC_VOLTAGE * (float)uiADCValue / (float)ADC_FULL;
	fVoltage *=((VOLTAGE_R1+VOLTAGE_R2)/VOLTAGE_R2);
	fVoltage *= fADCRefScale;
	if(ucUseErrorScale)
		fVoltage *= fErrorScale;
	return fVoltage;
}

float fGetCurrent(void){
	int16_t uiADCValue = 0;
	int usADCValueSum = 0;
	float fCurrent = 0.0;
	int16_t uiMaxADCValue,uiMinADCValue;
	uiMinADCValue = 4096;
	uiMaxADCValue = 0;
	uiADCValue = usGetADC3Value(CURRENT_ADC_CHANNEL);
	for(uint8_t i = 0;i<CURRENT_FILTER_LENGTH-1;i++){
		if(pusCurrentFilter[i+1] == 0)
			pusCurrentFilter[i+1] = uiADCValue;
		pusCurrentFilter[i] = pusCurrentFilter[i+1];
		usADCValueSum += pusCurrentFilter[i];
		if(pusCurrentFilter[i] > uiMaxADCValue)	uiMaxADCValue = pusCurrentFilter[i];
		if(pusCurrentFilter[i] < uiMinADCValue)	uiMinADCValue = pusCurrentFilter[i];	
	}
	pusCurrentFilter[CURRENT_FILTER_LENGTH-1] = uiADCValue;
	usADCValueSum += pusCurrentFilter[CURRENT_FILTER_LENGTH-1];
	if(pusCurrentFilter[CURRENT_FILTER_LENGTH-1] > uiMaxADCValue)	uiMaxADCValue = pusCurrentFilter[CURRENT_FILTER_LENGTH-1];
	if(pusCurrentFilter[CURRENT_FILTER_LENGTH-1] < uiMinADCValue)	uiMinADCValue = pusCurrentFilter[CURRENT_FILTER_LENGTH-1];
	
	usADCValueSum -= (uiMaxADCValue + uiMinADCValue);
	uiADCValue = usADCValueSum/(CURRENT_FILTER_LENGTH-2);
	
	if(uiADCValue < 2000 || uiADCValue > 2096 ) uiADCValue = 2048 - uiADCValue;
	else uiADCValue = 0;
	fCurrent = ADC_VOLTAGE * (float)uiADCValue / (float)ADC_FULL;
	fCurrent /= (CURRENT_RES*CURRENT_SCALE);
	fCurrent *= fADCRefScale;
	fCurrent *= fErrorScale;
	
//	if((pucHardwareVersion[1] == 0) || (pucHardwareVersion[1] == 1) || (pucHardwareVersion[1] == 2)){
//		fCurrent = ADC_VOLTAGE * (float)uiADCValue / (float)ADC_FULL;
//		fCurrent /= (CURRENT_RES*CURRENT_SCALE);
//		fCurrent *= fADCRefScale;
//		fCurrent *= fErrorScale;
//	}
//	else{
//		if(uiADCValue < 1980) uiADCValue = 1980 - uiADCValue;
//		else uiADCValue = 0;
//		fCurrent = ADC_VOLTAGE * (float)uiADCValue / (float)ADC_FULL;
//		fCurrent /= (0.005f*50);
//		fCurrent *= fADCRefScale;
//		fCurrent *= fErrorScale;	
//	}
	return fCurrent;
}

float fDoExternalCalibrate(float fVoltage){
	float fVoltageM	= 0.0;
	//disable error scale to get raw voltage
	ucUseErrorScale = 0;
	for(uint8_t i=0;i < 5;i++){
		fVoltageM = fGetVoltage();
	}
	//enable error scale to get calibrated voltage
	ucUseErrorScale = 1;
	return fVoltage/fVoltageM;
}


void vGetBatteryInfo(Battery_t *pxBattery){
	pxBattery->fVoltage = fGetVoltage();
	pxBattery->fCurrent = fGetCurrent();
}


void vGetBatteryVoltageInfo(Battery_t *pxBattery){
	pxBattery->fVoltage = fGetVoltage();
}

void vGetBatteryCurrentInfo(Battery_t *pxBattery){
	pxBattery->fCurrent = fGetCurrent();
}
