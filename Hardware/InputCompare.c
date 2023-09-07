#include "stm32f10x.h"                  // Device header

void IC_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	
	//选择定时器内部时钟
	TIM_InternalClockConfig(TIM3);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	TIM_TimeBaseInitTypeDef TimeBaseInitStructure;
	TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TimeBaseInitStructure.TIM_Period = 65536-1;  //ARR  //给最大值防止溢出
	TimeBaseInitStructure.TIM_Prescaler = 72-1; //PSC  //1MHz分频
	TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3,&TimeBaseInitStructure);
	
	//配置输入捕获通道
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICFilter  = 0xF; //滤波器消去干扰，数字越大滤波效果越好
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; //信号上升沿触发输入捕获
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; ///预分频设置为不分频
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;  //选择直连通道
	TIM_PWMIConfig(TIM3,&TIM_ICInitStructure);
	
	//触发源选择
	TIM_SelectInputTrigger(TIM3,TIM_TS_TI1FP1);
	
	//配置从模式
	TIM_SelectSlaveMode(TIM3,TIM_SlaveMode_Reset);
	
	
	//开启定时器
	TIM_Cmd(TIM3,ENABLE);
	/*开启定时器之后，CNT就会在内部时钟驱动下不断自增，
	当有信号来的时候，检测到第一次上升沿
	就会触发输入捕获的从模式清空CNT，
	当第二次接收到高电平的时候就会
	将CNT的值存储到CCR输入捕获寄存器*/
	
}

uint32_t IC_GetFreq(void)  //测周法测频率(Hz) Freq = 72MHz/（PSC-1）/CCR
{
	return 1000000/TIM_GetCapture1(TIM3);
}

uint32_t IC_GetDuty(void)
{
	return TIM_GetCapture2(TIM3)/TIM_GetCapture1(TIM3); 
}
