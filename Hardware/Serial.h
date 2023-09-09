#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdio.h>

extern uint8_t Flag_Front;
extern uint8_t Flag_Left;
extern uint8_t Flag_Right;
extern uint8_t Flag_Back;

void Serial_Init(void);
void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array, uint16_t Length);
void Serial_SendString(char *String);
void Serial_SendNumber(uint32_t Number, uint8_t Length);
void Serial_Printf(char *format, ...);
uint8_t Serial_Command(void);
void USART1_IRQHandler(void);
#endif
