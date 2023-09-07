#include "stm32f10x.h"                  // Device header

void AD_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);//����Ƶ��12MHz
	
	GPIO_InitTypeDef GPIO_InitStructure;
	//ADCר��ģʽ���Ͽ���GPIO�ڵ������������ֹ��ģ���ѹ��ɸ���
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	//���ù���������ͨ��ADC1��0��ͨ��������1������������55.5��ADCCLK����
	//ת�������ǹ̶���12.5������
	// ����ʱ����������+ת�����ڣ�/ADCCLk
	ADC_RegularChannelConfig(ADC1,ADC_Channel_0,1,ADC_SampleTime_55Cycles5);
	
	ADC_InitTypeDef ADC_InitStructure;
	ADC_InitStructure.ADC_Mode =ADC_Mode_Independent;//��ͨ��֮�����
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; //�����Ҷ���
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;//�������	
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ADC1,&ADC_InitStructure);
	
	ADC_Cmd(ADC1,ENABLE);
	
	// ��ADC���и�λУ׼��У׼��ɺ��Զ���0
	ADC_ResetCalibration(ADC1);
	//��ȡ��λУ׼״̬���ȴ���λУ׼���
	while(ADC_GetResetCalibrationStatus(ADC1));
	//����У׼
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));
		
}

uint16_t AD_GetValue()  //��ͨ�������Σ���ɨ��
{
	//�������ת��(����)
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);
	//ADC_FLAG_EOC   ת����ɱ�־λ  1:ת����ɣ�0δ���
	//ADC_FLAG_START �����鿪ʼת����־λ
	while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC) == RESET);//���ȴ���
	//ADC��ȡת��ֵ,�ú�������ֵ��ADCת���Ľ��
	return ADC_GetConversionValue(ADC1);//����ȡ��
	
}
