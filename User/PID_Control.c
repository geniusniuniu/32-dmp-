#include "stm32f10x.h"                  // Device header
#include "PID_Control.h"
#include <stdio.h>
#include "MPU_Exti.h"

PID_Structure PID_Struct;
float Turn_Kp = 2,Turn_Kd = 0.3;//����������������>0
float Encoder_Integral;
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
	
	//�����ٶȣ����ұ�����֮�ͣ�- Ŀ���ٶȣ��˴�Ϊ�㣩 
	pid->Speed_now_value = (Encoder_Left + Encoder_Right) - pid->Speed_expect_value; 
														
	Encoder = pid->Speed_now_value*0.3 + Encoder_last* 0.7; //===һ�׵�ͨ�˲���    
	Encoder_last = Encoder;
	Encoder_Integral += Encoder;
	if(Encoder_Integral>8000)  	
	Encoder_Integral=8000; //===�����޷�
	if(Encoder_Integral<-8000)	Encoder_Integral=-8000;  //===�����޷�	
	pid->Pid_Speed_out=Encoder*pid->Kp_Speed+Encoder_Integral*pid->Ki_Speed;    //===�ٶȿ���	
}

//ת��
float Turn_P(int Turn_Control)
{
	//Kd���ת��Լ����Kp���ң��ת��,ǰ����Լ���������ң��
	float PID_Turn_Out = Turn_Kd*(float)gz+Turn_Kp*Turn_Control;
	PID_Turn_Out = PID_Turn_Out>20?20:(PID_Turn_Out<-20?-20:PID_Turn_Out);	
	return 0;
}





