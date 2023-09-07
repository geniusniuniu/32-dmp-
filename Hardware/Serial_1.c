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
	
	//RXNE��־λ��1����NVIC�����ж�
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);//����RXNEλ��NVIC�ж�����
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn; //�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
	USART_Cmd(USART1,ENABLE);
}

void Serial_SendByte(uint8_t Byte)
{
	USART_SendData(USART1,Byte);
	// USART_FLAG_TXE ==0 ���ݻ�û�б���λ�Ĵ���ת��
	// USART_FLAG_TXE ==1 �����Ѿ�����λ�Ĵ���ת��
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET); //�ȴ�����ת��
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

  // ��ȡÿһλ����
  while(temp > 0) {
    digits[index] = temp % 10;
    temp /= 10;
    index++;
  }

  // ����������
  for(int8_t i = index - 1; i >= 0; i--) {
    Serial_SendByte(digits[i] + '0');
  }
}

void Serial_SendFloat(float Value,int8_t Length)
{
  Serial_SendNumber((int32_t)Value);  // ������������

  Serial_SendByte('.');  // ����С����

  // ��ȡС�������λ
  int32_t Decimal = (int32_t)((Value - (int32_t)Value) * 100);

  // ����С������
  Serial_SendNumber(Decimal);
}

int fputc(int ch,FILE*f)  //printf�����ض���
{
	Serial_SendByte(ch);
	return ch;
}

uint8_t Serila_GetRxFlag(void) //��ȡ�����־λ������
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
��������ͨ�����ڽ��յ�ʱ���ᴥ�������жϣ�
��ʱϵͳ���Զ����ô����жϷ�������������յ������ݡ�
*/
void USART1_IRQHandler(void)  
{
	if(USART_GetFlagStatus(USART1,USART_FLAG_RXNE))
	{
		Serial_RxData = USART_ReceiveData(USART1);//�Զ�������ת��
		Serial_RxFlag = 1;
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);
	}
}
