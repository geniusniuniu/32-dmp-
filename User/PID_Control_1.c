#include "stm32f10x.h"                  // Device header
#include "PID_Control.h"
#include <stdio.h>
#include "MPU_Exti.h"

PID_Structure PID_Struct;

void PID_Init(PID_Structure *pid){
	
	//直立环
	pid->Kp_Vertical = 0;
	pid->Kd_Vertical = 0;
	pid->Kp_Vertical_out = 0;
	pid->Kd_Vertical_out = 0;
	pid->Pid_Vertical_out = 0;
	pid->Vertical_expect_value = 0;//期望值
	pid->Vertical_now_value = 0;//当前值
	
	//速度环
	pid->Kp_Speed = 0;
	pid->Ki_Speed = 0;	
	pid->Kp_Speed_out = 0;
	pid->Ki_Speed_out = 0;
	pid->Pid_Speed_out = 0;
	pid->Speed_expect_value = 0;//期望值
	pid->Speed_now_value = 0;//当前值
	
	pid->PID_Turn_Out = 0;
	pid->Turn_Param = 0;
	
	pid->Pid_out = 0;
}

void Balance_PD(PID_Structure *pid)
{
	static float error;
	error = pid->Vertical_now_value - pid->Vertical_expect_value;
	pid->Pid_Vertical_out = pid->Kp_Vertical*error- pid->Kd_Vertical*(float)gx;
	if(pid->Pid_Vertical_out>=0) pid->Pid_Vertical_out+=4000;
	if(pid->Pid_Vertical_out<0) pid->Pid_Vertical_out-=4000;
}

//速度PI控制 修改前进后退速度
//入口参数：左轮编码器、右轮编码器脉冲值
//返回  值：速度控制PWM
//**************************************************************************/
void Speed_PI(PID_Structure* pid,float Encoder_Left,float Encoder_Right)
{  
	static float Encoder,Encoder_last;
	static float Encoder_Integral;
	//测量速度（左右编码器之和）- 目标速度（此处为零） 
	pid->Speed_now_value = (Encoder_Left + Encoder_Right) - pid->Speed_expect_value; 
														
	Encoder = pid->Speed_now_value*0.3 + Encoder_last* 0.7; //===一阶低通滤波器    
	Encoder_last = Encoder;
	Encoder_Integral += Encoder;
	if(Encoder_Integral>10000)  	
	Encoder_Integral=10000; //===积分限幅
	if(Encoder_Integral<-10000)	Encoder_Integral=-10000;  //===积分限幅	
	pid->Pid_Speed_out=Encoder*pid->Kp_Speed+Encoder_Integral*pid->Ki_Speed;    //===速度控制	
}
//转向环
void Turn(PID_Structure* pid)
{
	pid->PID_Turn_Out = (pid->Turn_Param)*(float)gz;
}






















////入口参数：左轮编码器、右轮编码器、Z轴陀螺仪
////返回  值：转向控制PWM
////**************************************************************************/
//int turn(int encoder_left,int encoder_right,float gyro)//转向控制
//{
//	 static float Turn_Target,Turn,Encoder_temp,Turn_Convert=0.9,Turn_Count;
//	  float Turn_Amplitude=88/Flag_sudu,Kp=42,Kd=0;     
//	  //=============遥控左右旋转部分=======================//
//  	if(1==Flag_Left||1==Flag_Right)                      //这一部分主要是根据旋转前的速度调整速度的起始速度，增加小车的适应性
//		{
//			if(++Turn_Count==1)
//			Encoder_temp=myabs(encoder_left+encoder_right);
//			Turn_Convert=50/Encoder_temp;
//			if(Turn_Convert<0.6)Turn_Convert=0.6;
//			if(Turn_Convert>3)Turn_Convert=3;
//		}	
//	  else
//		{
//			Turn_Convert=0.9;
//			Turn_Count=0;
//			Encoder_temp=0;
//		}			
//		if(1==Flag_Left)	           Turn_Target-=Turn_Convert;
//		else if(1==Flag_Right)	     Turn_Target+=Turn_Convert; 
//		else Turn_Target=0;
//	
//    if(Turn_Target>Turn_Amplitude)  Turn_Target=Turn_Amplitude;    //===转向速度限幅
//	  if(Turn_Target<-Turn_Amplitude) Turn_Target=-Turn_Amplitude;
//		if(Flag_Qian==1||Flag_Hou==1)  Kd=0.5;        
//		else Kd=0;   //转向的时候取消陀螺仪的纠正 有点模糊PID的思想
//  	//=============转向PD控制器=======================//
//		Turn=-Turn_Target*Kp -gyro*Kd;                 //===结合Z轴陀螺仪进行PD控制
//	  return Turn;
//}

