#include "stm32f10x.h"                  // Device header

void IC_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	
	//ѡ��ʱ���ڲ�ʱ��
	TIM_InternalClockConfig(TIM3);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	TIM_TimeBaseInitTypeDef TimeBaseInitStructure;
	TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TimeBaseInitStructure.TIM_Period = 65536-1;  //ARR  //�����ֵ��ֹ���
	TimeBaseInitStructure.TIM_Prescaler = 72-1; //PSC  //1MHz��Ƶ
	TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3,&TimeBaseInitStructure);
	
	//�������벶��ͨ��
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICFilter  = 0xF; //�˲�����ȥ���ţ�����Խ���˲�Ч��Խ��
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; //�ź������ش������벶��
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; ///Ԥ��Ƶ����Ϊ����Ƶ
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;  //ѡ��ֱ��ͨ��
	TIM_PWMIConfig(TIM3,&TIM_ICInitStructure);
	
	//����Դѡ��
	TIM_SelectInputTrigger(TIM3,TIM_TS_TI1FP1);
	
	//���ô�ģʽ
	TIM_SelectSlaveMode(TIM3,TIM_SlaveMode_Reset);
	
	
	//������ʱ��
	TIM_Cmd(TIM3,ENABLE);
	/*������ʱ��֮��CNT�ͻ����ڲ�ʱ�������²���������
	�����ź�����ʱ�򣬼�⵽��һ��������
	�ͻᴥ�����벶��Ĵ�ģʽ���CNT��
	���ڶ��ν��յ��ߵ�ƽ��ʱ��ͻ�
	��CNT��ֵ�洢��CCR���벶��Ĵ���*/
	
}

uint32_t IC_GetFreq(void)  //���ܷ���Ƶ��(Hz) Freq = 72MHz/��PSC-1��/CCR
{
	return 1000000/TIM_GetCapture1(TIM3);
}

uint32_t IC_GetDuty(void)
{
	return TIM_GetCapture2(TIM3)/TIM_GetCapture1(TIM3); 
}
