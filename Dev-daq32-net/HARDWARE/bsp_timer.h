/***************************************************************
Timer.h
Ϊ���ⶨʱ�жϣ���TIM2��ΪϵͳTick��������ʱ������lwip����ʱ��
****************************************************************/
#ifndef _BSP_TIMER_H
#define _BSP_TIMER_H															    

#include "stm32f4xx.h"


void Tick_TIM2_Config(void);
uint32_t GetTick(void);
//uint32_t sys_now(void);
void delay_us(uint32_t nus);
void delay_ms(uint16_t nms);

#endif
