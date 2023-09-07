#ifndef __MPU_Exti_H
#define __MPU_Exti_H

#include "PID_Control.h"

extern short gx, gy,gz;
extern float Pitch,Roll,Yaw; //欧拉角原始数据
extern float Speed_Left;
extern float Speed_Right;
void MPU_Exti_Init(void);
void PID_SetParam(PID_Structure *pid);
void GPIO_SetPolarity(void);
void IO_Init(void);
void Angle_Calculate(void);

#endif
