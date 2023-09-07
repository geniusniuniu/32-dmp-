#include "stm32f10x.h"                  // Device header

void AD_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);//六分频成12MHz
	
	GPIO_InitTypeDef GPIO_InitStructure;
	//ADC专属模式，断开了GPIO口的输入输出，防止对模拟电压造成干扰
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	//配置规则组输入通道ADC1，0号通道，序列1，采样周期是55.5个ADCCLK周期
	//转换周期是固定的12.5个周期
	// 总用时（采样周期+转换周期）/ADCCLk
	ADC_RegularChannelConfig(ADC1,ADC_Channel_0,1,ADC_SampleTime_55Cycles5);
	
	ADC_InitTypeDef ADC_InitStructure;
	ADC_InitStructure.ADC_Mode =ADC_Mode_Independent;//各通道之间独立
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; //数据右对齐
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;//软件触发	
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ADC1,&ADC_InitStructure);
	
	ADC_Cmd(ADC1,ENABLE);
	
	// 对ADC进行复位校准，校准完成后自动清0
	ADC_ResetCalibration(ADC1);
	//读取复位校准状态，等待复位校准完成
	while(ADC_GetResetCalibrationStatus(ADC1));
	//启动校准
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));
		
}

uint16_t AD_GetValue()  //单通道，单次，非扫描
{
	//软件触发转换(启动)
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);
	//ADC_FLAG_EOC   转换完成标志位  1:转换完成，0未完成
	//ADC_FLAG_START 规则组开始转换标志位
	while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC) == RESET);//（等待）
	//ADC获取转换值,该函数返回值是ADC转换的结果
	return ADC_GetConversionValue(ADC1);//（读取）
	
}
