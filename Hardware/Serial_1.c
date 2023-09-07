#include "stm32f10x.h"                  // Device header
#include <stdio.h>

uint8_t Serial_RxData;
uint8_t Serial_RxFlag;


void Serial_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;//TX
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//RX
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART1,&USART_InitStructure);
	
	//RXNE标志位置1，向NVIC申请中断
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);//开启RXNE位到NVIC中断配置
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn; //中断通道
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
	USART_Cmd(USART1,ENABLE);
}

void Serial_SendByte(uint8_t Byte)
{
	USART_SendData(USART1,Byte);
	// USART_FLAG_TXE ==0 数据还没有被移位寄存器转移
	// USART_FLAG_TXE ==1 数据已经被移位寄存器转移
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET); //等待数据转移
}

void Serial_SendArray(uint8_t *Array,uint16_t Length)
{
	uint16_t i;
	for(i = 0;i < Length;i ++)
		Serial_SendByte(Array[i]);
}

void Serial_SendString(uint8_t *String)
{
	uint16_t i;
	for(i = 0;String[i] != '0'; i ++)
		Serial_SendByte(String[i]);
}

uint32_t Serial_Pow(int x,int y)
{
	while(y --)
	{
		x *= x;
	}
	return x;
}


void Serial_SendNumber(int32_t Number)
{
  int32_t temp = Number;
  uint8_t digits[10] = {0};
  uint8_t index = 0;

  // 提取每一位数字
  while(temp > 0) {
    digits[index] = temp % 10;
    temp /= 10;
    index++;
  }

  // 逆序发送数字
  for(int8_t i = index - 1; i >= 0; i--) {
    Serial_SendByte(digits[i] + '0');
  }
}

void Serial_SendFloat(float Value,int8_t Length)
{
  Serial_SendNumber((int32_t)Value);  // 发送整数部分

  Serial_SendByte('.');  // 发送小数点

  // 提取小数点后两位
  int32_t Decimal = (int32_t)((Value - (int32_t)Value) * 100);

  // 发送小数部分
  Serial_SendNumber(Decimal);
}

int fputc(int ch,FILE*f)  //printf函数重定向
{
	Serial_SendByte(ch);
	return ch;
}

uint8_t Serila_GetRxFlag(void) //读取输入标志位后清零
{
	if(Serial_RxFlag == 1)
	{
		Serial_RxFlag = 0;
		return 1;
	}	
	return 0;
}

uint8_t Serila_GetRxData(void)
{
	return Serial_RxData;
}


/*
当有数据通过串口接收到时，会触发串口中断，
此时系统会自动调用串口中断服务函数来处理接收到的数据。
*/
void USART1_IRQHandler(void)  
{
	if(USART_GetFlagStatus(USART1,USART_FLAG_RXNE))
	{
		Serial_RxData = USART_ReceiveData(USART1);//对读入数据转存
		Serial_RxFlag = 1;
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);
	}
}
