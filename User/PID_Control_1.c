#include "stm32f10x.h"                  // Device header
#include "PID_Control.h"
#include <stdio.h>
#include "MPU_Exti.h"

PID_Structure PID_Struct;

void PID_Init(PID_Structure *pid){
	
	//ֱ����
	pid->Kp_Vertical = 0;
	pid->Kd_Vertical = 0;
	pid->Kp_Vertical_out = 0;
	pid->Kd_Vertical_out = 0;
	pid->Pid_Vertical_out = 0;
	pid->Vertical_expect_value = 0;//����ֵ
	pid->Vertical_now_value = 0;//��ǰֵ
	
	//�ٶȻ�
	pid->Kp_Speed = 0;
	pid->Ki_Speed = 0;	
	pid->Kp_Speed_out = 0;
	pid->Ki_Speed_out = 0;
	pid->Pid_Speed_out = 0;
	pid->Speed_expect_value = 0;//����ֵ
	pid->Speed_now_value = 0;//��ǰֵ
	
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

//�ٶ�PI���� �޸�ǰ�������ٶ�
//��ڲ��������ֱ����������ֱ���������ֵ
//����  ֵ���ٶȿ���PWM
//**************************************************************************/
void Speed_PI(PID_Structure* pid,float Encoder_Left,float Encoder_Right)
{  
	static float Encoder,Encoder_last;
	static float Encoder_Integral;
	//�����ٶȣ����ұ�����֮�ͣ�- Ŀ���ٶȣ��˴�Ϊ�㣩 
	pid->Speed_now_value = (Encoder_Left + Encoder_Right) - pid->Speed_expect_value; 
														
	Encoder = pid->Speed_now_value*0.3 + Encoder_last* 0.7; //===һ�׵�ͨ�˲���    
	Encoder_last = Encoder;
	Encoder_Integral += Encoder;
	if(Encoder_Integral>10000)  	
	Encoder_Integral=10000; //===�����޷�
	if(Encoder_Integral<-10000)	Encoder_Integral=-10000;  //===�����޷�	
	pid->Pid_Speed_out=Encoder*pid->Kp_Speed+Encoder_Integral*pid->Ki_Speed;    //===�ٶȿ���	
}
//ת��
void Turn(PID_Structure* pid)
{
	pid->PID_Turn_Out = (pid->Turn_Param)*(float)gz;
}






















////��ڲ��������ֱ����������ֱ�������Z��������
////����  ֵ��ת�����PWM
////**************************************************************************/
//int turn(int encoder_left,int encoder_right,float gyro)//ת�����
//{
//	 static float Turn_Target,Turn,Encoder_temp,Turn_Convert=0.9,Turn_Count;
//	  float Turn_Amplitude=88/Flag_sudu,Kp=42,Kd=0;     
//	  //=============ң��������ת����=======================//
//  	if(1==Flag_Left||1==Flag_Right)                      //��һ������Ҫ�Ǹ�����תǰ���ٶȵ����ٶȵ���ʼ�ٶȣ�����С������Ӧ��
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
//    if(Turn_Target>Turn_Amplitude)  Turn_Target=Turn_Amplitude;    //===ת���ٶ��޷�
//	  if(Turn_Target<-Turn_Amplitude) Turn_Target=-Turn_Amplitude;
//		if(Flag_Qian==1||Flag_Hou==1)  Kd=0.5;        
//		else Kd=0;   //ת���ʱ��ȡ�������ǵľ��� �е�ģ��PID��˼��
//  	//=============ת��PD������=======================//
//		Turn=-Turn_Target*Kp -gyro*Kd;                 //===���Z�������ǽ���PD����
//	  return Turn;
//}

