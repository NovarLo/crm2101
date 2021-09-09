/***************************************************************
Timer.c
为避免定时中断，用TIM2作为系统Tick，用于延时函数及lwip本地时间
****************************************************************/

#include "bsp_timer.h"


extern uint32_t lwip_localtime;	//lwip本地时间计数器,单位ms

/***************************************************
    定时器TIM2定时配置
时钟预分频至1us(1MHz),计数周期=计满 
用作我的tick，无中断 
**************************************************/
void Tick_TIM2_Config(void)
{	
	TIM_TimeBaseInitTypeDef    TIM_TimeBaseStructure;
	
	// 使能TIM2时钟，TIM2CLK 为168/4=42M
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
    // TIM2基本定时器配置
    TIM_TimeBaseStructure.TIM_Period = 0xffffffff;       			//定时周期  
    TIM_TimeBaseStructure.TIM_Prescaler = 42-1;       				//预分频至1M
    TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;    			    //时钟分频系数
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  	//向上计数模式
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	// 使能TIM2
    TIM_Cmd(TIM2, ENABLE);
}
/********************************************* 
    获得当前TIM2做的Tick数(us)
*********************************************/
uint32_t GetTick(void)
{
    return TIM_GetCounter(TIM2);
}
//获得lwip_localtime
uint32_t get_lwip_localtime(void)
{
	return GetTick()/1000;
}
/*为LWIP提供时钟
uint32_t sys_now(void)
{
	lwip_localtime = get_lwip_localtime();	//ms
	return lwip_localtime;
}*/


/********************************************* 
    用TIM2做的us延时函数
输入:nus=延时us数 
*********************************************/
void delay_us(uint32_t nus)
{
    uint32_t ntick;

    ntick = GetTick();
    while(GetTick() < ntick+nus); 
}
/********************************************* 
    用TIM2做的ms延时函数
输入:nms=延时ms数 
*********************************************/
void delay_ms(uint16_t nms)
{
    uint32_t ntick;

    ntick = GetTick();
    while(GetTick() < ntick+(uint32_t)nms*1000); 
}

