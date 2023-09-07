#ifndef __SERIAL_H
#define __SERIAL_H
#include <stdio.h>

void Serial_Init(void);
//发送
void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array,uint16_t Length);
void Serial_SendString(uint8_t *String);
uint32_t Serial_Pow(int x,int y);
void Serial_SendNumber(uint32_t Number,uint8_t Length);
void Serial_SendFloat(float Value,int8_t Length);

//接收 
void USART1_IRQHandler(void);  
uint8_t Serila_GetRxData(void);
uint8_t Serila_GetRxFlag(void); //读取输入标志位后清零

#endif
