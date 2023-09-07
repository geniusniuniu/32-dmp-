#ifndef __HC_SR04_H
#define __HC_SR04_H

extern float Dis;
void HC_SR04_Init(void);
float sonar(void);								//测距并返回单位为毫米的距离结果

#endif
