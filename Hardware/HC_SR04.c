#include "stm32f10x.h"
#include "Delay.h"
#include "OLED.h"

#define Echo_PIN GPIO_Pin_3		//HC-SR04模块的Echo脚接GPIOA3
#define Trig_PIN GPIO_Pin_4		//HC-SR04模块的Trig脚接GPIOA4

void HC_SR04_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);	//启用GPIOA的外设时钟	
	GPIO_InitTypeDef GPIO_InitStructure;					//定义结构体
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		//设置GPIO口为推挽输出
	GPIO_InitStructure.GPIO_Pin = Trig_PIN;						//设置GPIO口5
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		//设置GPIO口速度50Mhz
	GPIO_Init(GPIOA,&GPIO_InitStructure);					//初始化GPIOA
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;			//设置GPIO口为下拉输入模式
	GPIO_InitStructure.GPIO_Pin = Echo_PIN;						//设置GPIO口6
	GPIO_Init(GPIOA,&GPIO_InitStructure);					//初始化GPIOA
	GPIO_WriteBit(GPIOA,Trig_PIN,(BitAction)0);						//输出低电平
	Delay_us(15);											//延时15微秒
}


float time_end = 0;

float sonar(void) 
{
	float Distance = 0;
    GPIO_WriteBit(GPIOA, Trig_PIN, Bit_SET);
    Delay_us(10);  // 触发超声脉冲至少10微秒
    GPIO_WriteBit(GPIOA, Trig_PIN, Bit_RESET);
    
    while (GPIO_ReadInputDataBit(GPIOA, Echo_PIN) == 0)
	{
		TIM_Cmd(TIM2,DISABLE);
	}
	TIM_Cmd(TIM2,ENABLE); 
	TIM_SetCounter(TIM2,0); // 记录开始时间
    while (GPIO_ReadInputDataBit(GPIOA, Echo_PIN) == 1);
    time_end = TIM_GetCounter(TIM2); // 计算时间差
	TIM_Cmd(TIM2,DISABLE);
	//OLED_ShowNum(3,1,time_end,3);
	if(time_end>300 && time_end<650)
	{
		float sound_speed = 0.0346; // 声速
		Distance = (sound_speed * time_end) / 2.0; 
	}
	else if(time_end>650)
	{
		return 50;
	}
	return Distance;
}

