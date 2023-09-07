#include "stm32f10x.h"                  // Device header
#include "PID_Control.h"
#include <stdio.h>
#include "MPU_Exti.h"

PID_Structure PID_Struct;
float Turn_Kp = 2,Turn_Kd = 0.3;//负反馈，参数极性>0
float Encoder_Integral;
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
	
	//测量速度（左右编码器之和）- 目标速度（此处为零） 
	pid->Speed_now_value = (Encoder_Left + Encoder_Right) - pid->Speed_expect_value; 
														
	Encoder = pid->Speed_now_value*0.3 + Encoder_last* 0.7; //===一阶低通滤波器    
	Encoder_last = Encoder;
	Encoder_Integral += Encoder;
	if(Encoder_Integral>8000)  	
	Encoder_Integral=8000; //===积分限幅
	if(Encoder_Integral<-8000)	Encoder_Integral=-8000;  //===积分限幅	
	pid->Pid_Speed_out=Encoder*pid->Kp_Speed+Encoder_Integral*pid->Ki_Speed;    //===速度控制	
}

//转向环
float Turn_P(int Turn_Control)
{
	//Kd针对转向约束，Kp针对遥控转向,前半是约束，后半是遥控
	float PID_Turn_Out = Turn_Kd*(float)gz+Turn_Kp*Turn_Control;
	PID_Turn_Out = PID_Turn_Out>20?20:(PID_Turn_Out<-20?-20:PID_Turn_Out);	
	return 0;
}





