#include "stm32f10x.h"                  // Device header
#include "OLED.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "inv_mpu.h"
#include "Delay.h"
#include "MPU6050.h"
#include "Serial.h"
#include "math.h"
#include "MPU_Exti.h"
#include "Tim_Encoder.h"  //定时器编码器模式
#include "PWM.h" 
#include "PID_Control.h"
#include "Timer.h"
#include "HC_SR04.h"

uint8_t Exti_Flag = 0;
uint8_t ID;
float Dis = 0;
uint8_t Timer_Flag = 0;

void MISC_Init(void)
{
	OLED_Init();
	Timer_Init();
	//Key_Init();
	PWM_Init();
	Encoder_Init();
	PID_Init(&PID_Struct);
	//Serial_Init();
	IO_Init(); //反转电平极性
	
	MPU_Init();
	mpu_dmp_init();
	MPU_Exti_Init(); //陀螺仪外部中断初始化
}


void PID_SetParam(PID_Structure *pid)
{
	pid->Kp_Vertical = -200;
	pid->Kd_Vertical = 1.5;
	
	pid->Kp_Speed = 0.9;	//给正值，是正反馈
	pid->Ki_Speed = pid->Kp_Speed / 200;
	pid->Speed_expect_value = 1;
	
	pid->Turn_Param = 2;  //负反馈，参数极性>0
}

int main(void)
{
	MISC_Init();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	PID_SetParam(&PID_Struct);
	OLED_ShowString(1,1,"Roll: ");
	OLED_ShowString(2,1,"Gyro:");
	OLED_ShowString(4,1,"Distance:");
	OLED_ShowFloatNum(1,7,Roll,2,2);
	OLED_ShowSignedNum(2,7,gx,5);
	OLED_ShowSignedNum(2,13,PID_Struct.Turn_Param,3);
	OLED_ShowNum(3,1,PID_Struct.Kp_Vertical,4);
	OLED_ShowFloatNum(3,6,PID_Struct.Kd_Vertical,1,2);
	OLED_ShowFloatNum(3,11,PID_Struct.Kp_Speed,2,2);
	while(1)
	{	
		if(Exti_Flag == 1)
		{
			mpu_dmp_get_data(&Pitch,&Roll,&Yaw);
			PID_Struct.Vertical_now_value = Roll;
			MPU_Get_Gyroscope(&gx,&gy,&gz);	
			Speed_Left = TimEncoder_Get(TIM1); //10ms的脉冲数
			Speed_Right = TimEncoder_Get(TIM3);
			
			Turn(&PID_Struct);
			PID_Struct.Vertical_expect_value = PID_Struct.Pid_Speed_out-2.3;
			Balance_PD(&PID_Struct);
			Speed_PI(&PID_Struct,Speed_Left,Speed_Right);
			PID_Struct.Pid_out = PID_Struct.Pid_Vertical_out;
			if(PID_Struct.Pid_out>7000)
				PID_Struct.Pid_out = 7000;
			else if(PID_Struct.Pid_out<-7000)
				PID_Struct.Pid_out = -7000;
			if(Roll > 50 || Roll < -50 || Roll == 0.0)
			{
				PWM_SetCompare1(0);
				PWM_SetCompare2(0);
			}
			else
			{
				GPIO_SetPolarity();	
				PWM_SetCompare1(PID_Struct.Pid_out+PID_Struct.PID_Turn_Out);
				PWM_SetCompare2(PID_Struct.Pid_out-PID_Struct.PID_Turn_Out);
			}
			Exti_Flag = 0;			
		}
		if(Timer_Flag == 1)
		{
			Dis = sonar();
			OLED_ShowFloatNum(4,10,Dis,3,2);
			Timer_Flag = 0;
		}
	}
}


//外部中断线12服务程序（10ms中断）
void EXTI15_10_IRQHandler(void)
{
	if(EXTI_GetFlagStatus(EXTI_Line12) == SET)
	{
		EXTI_ClearITPendingBit(EXTI_Line12); //清除LINE上的中断标志位
		Exti_Flag = 1;
	}
}

void TIM2_IRQHandler(void)			//更新中断函数，
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)		//获取TIM2定时器的更新中断标志位
	{
		Timer_Flag = 1;
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);			//清除更新中断标志位
	}
}
