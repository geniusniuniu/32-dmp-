#include "MPU_Exti.h"
#include "sys.h"
#include "mpu6050.h"   
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "OLED.h"
#include "delay.h"
#include "PID_Control.h"
#include "Tim_Encoder.h"  //定时器编码器模式
#include "PWM.h"

/*
MPU的数据通常是以一定的频率产生的，如果在主函数中读取，
需要通过循环等待来判断是否有新的数据到来，
这样会浪费CPU资源，并且可能会导致数据的丢失或延迟。
而在中断中，可以通过配置传感器的中断输出引脚，
当有新的数据到来时，触发中断，立即执行中断服务程序来读取数据，
保证数据的实时性。
*/
short gx, gy,gz;
float Pitch,Roll,Yaw; //欧拉角原始数据
float Speed_Left;
float Speed_Right;

#define GPIO_W_12(x)		GPIO_WriteBit(GPIOB, GPIO_Pin_12, (BitAction)(x))
#define GPIO_W_13(x)		GPIO_WriteBit(GPIOB, GPIO_Pin_13, (BitAction)(x))
#define GPIO_W_14(x)		GPIO_WriteBit(GPIOB, GPIO_Pin_14, (BitAction)(x))
#define GPIO_W_15(x)		GPIO_WriteBit(GPIOB, GPIO_Pin_15, (BitAction)(x))

#define GPIO_R_12        	GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_12)
#define GPIO_R_13        	GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_13)
#define GPIO_R_14        	GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_14)
#define GPIO_R_15        	GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_15)
 
void IO_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	GPIO_SetBits(GPIOB,GPIO_Pin_13|GPIO_Pin_15); //初始默认高电平
	GPIO_ResetBits(GPIOB,GPIO_Pin_12|GPIO_Pin_14); //初始默认高电平
}



 void GPIO_SetPolarity(void)
{
	if(PID_Struct.Pid_out > 0)
	{
		GPIO_W_12(0);
		GPIO_W_13(1);
		GPIO_W_14(0);
		GPIO_W_15(1);
	}
	else if (PID_Struct.Pid_out < 0)
	{
		GPIO_W_12(1);
		GPIO_W_13(0);
		GPIO_W_14(1);
		GPIO_W_15(0);
	}
}

//函数功能：初始化接收陀螺仪中断的引脚
void MPU_Exti_Init()
{
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
 	GPIO_InitTypeDef GPIO_InitStructure;
	
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);	//使能复用功能时钟
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_12;	 //PA12
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //设置成上拉输入
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOA, &GPIO_InitStructure); 
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource12); //将PA12映射到中断线12
	EXTI_InitStructure.EXTI_Line=EXTI_Line12;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;     //下降沿触发
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);	  	
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //2位抢占优先级，2位响应优先级
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;		
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //抢占优先级0 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//响应优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;						
	NVIC_Init(&NVIC_InitStructure);
}


////外部中断线12服务程序（10ms中断）
//void EXTI15_10_IRQHandler(void)
//{
//	if(EXTI_GetFlagStatus(EXTI_Line12) == SET)
//	{	
//		EXTI_ClearITPendingBit(EXTI_Line12);
//		mpu_dmp_get_data(&Pitch,&Roll,&Yaw);
//		PID_Struct.Vertical_now_value = Roll;
//		MPU_Get_Gyroscope(&gx,&gy,&gz);	
//		Speed_Left = TimEncoder_Get(TIM1); //10ms的脉冲数
//		Speed_Right = TimEncoder_Get(TIM3);
//		
//		//Turn_P(&PID_Struct);
//		PID_Struct.Vertical_expect_value = PID_Struct.Pid_Speed_out-3;
//		Balance_PD(&PID_Struct);
//		Speed_PI(&PID_Struct,Speed_Left,Speed_Right);
//		PID_Struct.Pid_out = PID_Struct.Pid_Vertical_out;
//		
//		if(PID_Struct.Pid_out>7000)
//			PID_Struct.Pid_out = 7000;
//		else if(PID_Struct.Pid_out<-7000)
//			PID_Struct.Pid_out = -7000;
//		if(Roll > 50 || Roll < -50 || Roll == 0.0)
//		{
//			PWM_SetCompare1(0);
//			PWM_SetCompare2(0);
//		}
//		else
//		{
//			GPIO_SetPolarity();	
//			PWM_SetCompare1(PID_Struct.Pid_out-Turn_P(0));
//			PWM_SetCompare2(PID_Struct.Pid_out+Turn_P(0));
//		}
//	}
//}

