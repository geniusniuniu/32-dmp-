#ifndef __key_H
#define	__key_H
#include "sys.h" 

#define KEY1_PRES 	  1		//KEY1����
#define KEY2_PRES	  2		//KEY2����
#define KEY3_PRES     3		//KEY3����
/* function -------------------------------------------------------------------------------------------------------------*/
void Key_Init(void);
uint8_t KEY_Scan(void);
extern uint8_t KeyValue;

#endif
