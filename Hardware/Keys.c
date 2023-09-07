#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "Keys.h"

uint8_t KeyValue = 0;

void Key_Init(void)
{		
		GPIO_InitTypeDef GPIO_InitStructure;
		EXTI_InitTypeDef EXTI_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);	//使能复用功能时钟
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_2;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   
		GPIO_Init(GPIOA, &GPIO_InitStructure);	
		
		GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource1|GPIO_PinSource2); //将PA1,2映射到中断线12
		EXTI_InitStructure.EXTI_Line=EXTI_Line1|EXTI_Line2;
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;     //下降沿触发
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStructure);	  	
		
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //2位抢占优先级，2位响应优先级
		NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;		
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; //抢占优先级0 
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//响应优先级0
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;						
		NVIC_Init(&NVIC_InitStructure);
		
		NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;		
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; 
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;						
		NVIC_Init(&NVIC_InitStructure);
}


//按键处理函数
void EXTI1_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line1)==SET)
	{
		 if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) == RESET)
         {
			KeyValue = 1;
		 }
		EXTI_ClearITPendingBit(EXTI_Line1);
	}
}

void EXTI2_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line2)==SET)
	{
		 if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2) == RESET)
         {
			KeyValue = 2;
		 }
		 EXTI_ClearITPendingBit(EXTI_Line2);
	}
}

uint8_t KEY_Scan(void)
{
	return KeyValue;
}
