#ifndef __PID_Control_H
#define __PID_Control_H

extern float Turn_Kp;
extern float Turn_Kd;//负反馈，参数极性>0
extern float Encoder_Integral;

typedef struct PID
{
	//直立环
	float Kp_Vertical;
	float Ki_Vertical;
	float Kd_Vertical;
	float Kp_Vertical_out;
	float Ki_Vertical_out;	
	float Kd_Vertical_out;
	float Pid_Vertical_out;
	float Vertical_expect_value;//期望值
	float Vertical_now_value;//当前值
	
	//速度环
	float Kp_Speed;
	float Ki_Speed;	
	float Kp_Speed_out;
	float Ki_Speed_out;
	float Pid_Speed_out;
	float Speed_expect_value;//期望值
	float Speed_now_value;//当前值
	
	float Pid_out;
		
}PID_Structure;

extern PID_Structure PID_Struct;

void PID_Init(PID_Structure *pid);
void Balance_PD(PID_Structure *pid); //平衡环PD控制
void Speed_PI(PID_Structure *pid,float Encoder_Left,float Encoder_Right);
float Turn_P(int Turn_Control);


#endif
