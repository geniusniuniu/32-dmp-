#include "stm32f10x.h"
#include "Delay.h"
#include "OLED.h"

#define Echo_PIN GPIO_Pin_3		//HC-SR04ģ���Echo�Ž�GPIOA3
#define Trig_PIN GPIO_Pin_4		//HC-SR04ģ���Trig�Ž�GPIOA4

void HC_SR04_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);	//����GPIOA������ʱ��	
	GPIO_InitTypeDef GPIO_InitStructure;					//����ṹ��
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		//����GPIO��Ϊ�������
	GPIO_InitStructure.GPIO_Pin = Trig_PIN;						//����GPIO��5
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		//����GPIO���ٶ�50Mhz
	GPIO_Init(GPIOA,&GPIO_InitStructure);					//��ʼ��GPIOA
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;			//����GPIO��Ϊ��������ģʽ
	GPIO_InitStructure.GPIO_Pin = Echo_PIN;						//����GPIO��6
	GPIO_Init(GPIOA,&GPIO_InitStructure);					//��ʼ��GPIOA
	GPIO_WriteBit(GPIOA,Trig_PIN,(BitAction)0);						//����͵�ƽ
	Delay_us(15);											//��ʱ15΢��
}


float time_end = 0;

float sonar(void) 
{
	float Distance = 0;
    GPIO_WriteBit(GPIOA, Trig_PIN, Bit_SET);
    Delay_us(10);  // ����������������10΢��
    GPIO_WriteBit(GPIOA, Trig_PIN, Bit_RESET);
    
    while (GPIO_ReadInputDataBit(GPIOA, Echo_PIN) == 0)
	{
		TIM_Cmd(TIM2,DISABLE);
	}
	TIM_Cmd(TIM2,ENABLE); 
	TIM_SetCounter(TIM2,0); // ��¼��ʼʱ��
    while (GPIO_ReadInputDataBit(GPIOA, Echo_PIN) == 1);
    time_end = TIM_GetCounter(TIM2); // ����ʱ���
	TIM_Cmd(TIM2,DISABLE);
	//OLED_ShowNum(3,1,time_end,3);
	if(time_end>300 && time_end<650)
	{
		float sound_speed = 0.0346; // ����
		Distance = (sound_speed * time_end) / 2.0; 
	}
	else if(time_end>650)
	{
		return 50;
	}
	return Distance;
}

