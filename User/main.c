#include "stm32f10x.h"                  // Device header
#include "OLED.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "inv_mpu.h"
#include "Delay.h"
#include "MPU6050.h"
#include "Serial.h"
#include "MPU_Exti.h"
#include "Tim_Encoder.h"  //��ʱ��������ģʽ
#include "PWM.h" 
#include "PID_Control.h"
#include "Xunji.h"

#define Max_Speed 15
#define Max_Turn_Speed 10

float Target_Speed = 10;
float Target_Turn_Speed = 0;

int Flag;

void MISC_Init(void)
{
	OLED_Init();
	PWM_Init();
	Encoder_Init();
	PID_Init(&PID_Struct);
	IO_Init(); //��ת��ƽ����
	Serial_Init();
	Xunji_Init();
	
	MPU_Init();
	mpu_dmp_init();
	MPU_Exti_Init(); //�������ⲿ�жϳ�ʼ��
}


void PID_SetParam(PID_Structure *pid)
{
	pid->Kp_Vertical = -200;
	pid->Kd_Vertical = 1.42;
	
	pid->Kp_Speed = 0.83;	//����ֵ����������
	pid->Ki_Speed = pid->Kp_Speed / 220;
	pid->Speed_expect_value = Target_Speed;//����0������
	
}

void OLED_ShowParam(void)
{
	OLED_ShowFloatNum(1,7,Roll,2,2);
	OLED_ShowSignedNum(2,7,gx,5);
	OLED_ShowNum(3,1,Flag_Front,1);
	OLED_ShowNum(3,3,Flag_Back,1);
	OLED_ShowNum(3,5,Flag_Left,1);
	OLED_ShowNum(3,7,Flag_Right,1);
	OLED_ShowFloatNum(3,9,PID_Struct.Speed_expect_value,4,2);
}

void Xunji_GetValue(void)
{
	if((Roll < 18 && Roll > -20))
	{
		if(C13_Value == 1)    Flag_Right = 1; //����⵽������
		if(C14_Value == 1)	  Flag_Left = 1;  //�Ҳ��⵽������
	}
}

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
		Flag = 1;
		Serial_Command();
		Xunji_GetValue();
		OLED_ShowParam();
		if(Flag_Front == 1) 		{OLED_ShowString(4,6,"farward");Flag += 1;printf("Go Forward\r\n");}
		else if(Flag_Back == 1) 	{OLED_ShowString(4,6,"Back");Flag += 1;printf("Go Back\r\n");}
		else if(Flag_Left == 1) 	{OLED_ShowString(4,6,"Left");Flag += 1;printf("Turn Left\r\n");}
		else if(Flag_Right == 1) 	{OLED_ShowString(4,6,"Right");Flag += 1;printf("Turn Right\r\n");}
		else 						{OLED_ShowString(4,6,"Stop");Flag += 1;/*printf("No Signal\r\n");*/}
		
		while(Flag)
		{	
			Target_Speed = 10;
			Flag_Front = 0;
			Flag_Back = 0;
			Flag_Left = 0;
			Flag_Right = 0;
			Flag--;
		}
	}
}

//�ⲿ�ж���12�������10ms�жϣ�
void EXTI15_10_IRQHandler(void)
{
	if(EXTI_GetFlagStatus(EXTI_Line12) == SET)
	{	
		EXTI_ClearITPendingBit(EXTI_Line12);
		mpu_dmp_get_data(&Pitch,&Roll,&Yaw);
		PID_Struct.Vertical_now_value = Roll;
		MPU_Get_Gyroscope(&gx,&gy,&gz);	
		Speed_Left = TimEncoder_Get(TIM1); //10ms��������
		Speed_Right = TimEncoder_Get(TIM3);
		//ǰ��
		if(Flag_Front == 1) Target_Speed--;
		if(Flag_Back == 1) Target_Speed++;
		if((Flag_Front == 0)&&(Flag_Back == 0)) Target_Speed = 5;//ֹͣ
		Target_Speed = Target_Speed>Max_Speed?Max_Speed:(Target_Speed<-Max_Speed?-Max_Speed:Target_Speed); //ǰ���޷�
		
		//左右转
		if(Flag_Left == 1) Target_Turn_Speed--;
		if(Flag_Right == 1) Target_Turn_Speed++;
		if((Flag_Left == 0)&&(Flag_Right == 0))
			{Target_Turn_Speed = 0;Turn_Kd = 0.4;}
		else if((Flag_Left == 1)||(Flag_Right == 1)) 
			Turn_Kd = 0; //ֹͣ
		Target_Turn_Speed = Target_Turn_Speed>Max_Turn_Speed?Max_Turn_Speed:(Target_Turn_Speed<-Max_Turn_Speed?-Max_Turn_Speed:Target_Turn_Speed);	
		
		PID_Struct.Speed_expect_value = Target_Speed;
		Speed_PI(&PID_Struct,Speed_Left,Speed_Right);
		PID_Struct.Vertical_expect_value = PID_Struct.Pid_Speed_out-2.1;
		Balance_PD(&PID_Struct);
		
		PID_Struct.Pid_out = PID_Struct.Pid_Vertical_out;
		
		if(PID_Struct.Pid_out>7000)
			PID_Struct.Pid_out = 7000;
		else if(PID_Struct.Pid_out<-7000)
			PID_Struct.Pid_out = -7000;
		
		if((Roll-2.1>40)||(2.1-Roll >40)||Roll == 0)
		{
			PID_Struct.Pid_out = 0;
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

