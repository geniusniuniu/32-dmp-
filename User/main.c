#include "stm32f10x.h"                  // Device header
#include "OLED.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "inv_mpu.h"
#include "Delay.h"
#include "MPU6050.h"
#include "Serial.h"
#include "MPU_Exti.h"
#include "Tim_Encoder.h"  //定时器编码器模式
#include "PWM.h" 
#include "PID_Control.h"
#include "math.h"

#define Max_Speed 3
#define Max_Turn_Speed 4

extern uint8_t Flag_Front;
extern uint8_t Flag_Left;
extern uint8_t Flag_Right;
extern uint8_t Flag_Back;
extern float Encoder_Integral;
extern float Turn_Kp;
extern float Turn_Kd;//负反馈，参数极性>0


float Target_Speed = 0;
float Target_Turn_Speed = 0;

void MISC_Init(void)
{
	OLED_Init();
	PWM_Init();
	Encoder_Init();
	PID_Init(&PID_Struct);
	IO_Init(); //反转电平极性
	//Serial_Init();
	
	MPU_Init();
	mpu_dmp_init();
	MPU_Exti_Init(); //陀螺仪外部中断初始化
}


void PID_SetParam(PID_Structure *pid)
{
	pid->Kp_Vertical = -204;
	pid->Kd_Vertical = 1.40;
	
	pid->Kp_Speed = 0.83;	//给正值，是正反馈
	pid->Ki_Speed = pid->Kp_Speed / 220;
	pid->Speed_expect_value = Target_Speed;//大于0，向右
	
}

void OLED_ShowParam(void)
{
	OLED_ShowFloatNum(1,7,Roll,2,2);
	OLED_ShowSignedNum(2,7,gx,5);
	//OLED_ShowFloatNum(4,6,Dis,2,2);
}

//

int main(void)
{
	MISC_Init();
	PID_SetParam(&PID_Struct);
	OLED_ShowString(1,1,"Roll: ");
	OLED_ShowString(2,1,"Gyro:");
	OLED_ShowString(4, 1,"Dir:");
	//OLED_ShowString(3, 1, "Rx:");
	while(1)
	{	
		//Serial_Command();
		OLED_ShowParam();
		OLED_ShowNum(3,1,Flag_Front,1);
		OLED_ShowNum(3,3,Flag_Back,1);
		OLED_ShowNum(3,5,Flag_Left,1);
		OLED_ShowNum(3,7,Flag_Right,1);
		OLED_ShowFloatNum(3,9,PID_Struct.Pid_out,4,2);
		if(Flag_Front == 1) 		{OLED_ShowString(4,6,"farward");}
		else if(Flag_Back == 1) 	{OLED_ShowString(4,6,"Back");}
		else if(Flag_Left == 1) 	{OLED_ShowString(4,6,"Left");}
		else if(Flag_Right == 1) 	{OLED_ShowString(4,6,"Right");}
		else 						{OLED_ShowString(4,6,"Stop");}
	}
}

//外部中断线12服务程序（10ms中断）
void EXTI15_10_IRQHandler(void)
{
	if(EXTI_GetFlagStatus(EXTI_Line12) == SET)
	{	
		EXTI_ClearITPendingBit(EXTI_Line12);
		mpu_dmp_get_data(&Pitch,&Roll,&Yaw);
		PID_Struct.Vertical_now_value = Roll;
		MPU_Get_Gyroscope(&gx,&gy,&gz);	
		Speed_Left = TimEncoder_Get(TIM1); //10ms的脉冲数
		Speed_Right = TimEncoder_Get(TIM3);
//		//前后
//		if(Flag_Front == 1) Target_Speed++;
//		if(Flag_Back == 1) Target_Speed--;
//		if((Flag_Front == 0)&&(Flag_Back == 0)) Target_Speed = 0;//停止
//		Target_Speed = Target_Speed>Max_Speed?Max_Speed:(Target_Speed<-Max_Speed?-Max_Speed:Target_Speed); //前后限幅
//		
//		//左右
//		if(Flag_Left == 1) Target_Turn_Speed--;
//		if(Flag_Right == 1) Target_Turn_Speed++;
//		if((Flag_Left == 0)&&(Flag_Right == 0))
//			{Target_Turn_Speed = 0;Turn_Kd = 0.3;}
//		else if((Flag_Left == 1)||(Flag_Right == 1)) 
//			Turn_Kd = 0; //停止
//		Target_Turn_Speed = Target_Turn_Speed>Max_Turn_Speed?Max_Turn_Speed:(Target_Turn_Speed<-Max_Turn_Speed?-Max_Turn_Speed:Target_Turn_Speed);	
		
		Speed_PI(&PID_Struct,Speed_Left,Speed_Right);
		PID_Struct.Vertical_expect_value = PID_Struct.Pid_Speed_out-2.1;
		Balance_PD(&PID_Struct);
		
		PID_Struct.Pid_out = PID_Struct.Pid_Vertical_out;
		
		if(PID_Struct.Pid_out>7000)
			PID_Struct.Pid_out = 7000;
		else if(PID_Struct.Pid_out<-7000)
			PID_Struct.Pid_out = -7000;
		
		if((Roll-2.1>35)||(2.1-Roll >35)||Roll == 0)
		{
			PWM_SetCompare1(0); 
			PWM_SetCompare2(0);
			Encoder_Integral = 0;
		}
		else
		{
			GPIO_SetPolarity();	
			PWM_SetCompare1(PID_Struct.Pid_out - Turn_P(Target_Turn_Speed));
			PWM_SetCompare2(PID_Struct.Pid_out + Turn_P(Target_Turn_Speed));
		}
	}
}

