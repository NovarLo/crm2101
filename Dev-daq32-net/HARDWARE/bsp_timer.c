/***************************************************************
Timer.c
Ϊ���ⶨʱ�жϣ���TIM2��ΪϵͳTick��������ʱ������lwip����ʱ��
****************************************************************/

#include "bsp_timer.h"


extern uint32_t lwip_localtime;	//lwip����ʱ�������,��λms

/***************************************************
    ��ʱ��TIM2��ʱ����
ʱ��Ԥ��Ƶ��1us(1MHz),��������=���� 
�����ҵ�tick�����ж� 
**************************************************/
void Tick_TIM2_Config(void)
{	
	TIM_TimeBaseInitTypeDef    TIM_TimeBaseStructure;
	
	// ʹ��TIM2ʱ�ӣ�TIM2CLK Ϊ168/4=42M
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
    // TIM2������ʱ������
    TIM_TimeBaseStructure.TIM_Period = 0xffffffff;       			//��ʱ����  
    TIM_TimeBaseStructure.TIM_Prescaler = 42-1;       				//Ԥ��Ƶ��1M
    TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;    			    //ʱ�ӷ�Ƶϵ��
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  	//���ϼ���ģʽ
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	// ʹ��TIM2
    TIM_Cmd(TIM2, ENABLE);
}
/********************************************* 
    ��õ�ǰTIM2����Tick��(us)
*********************************************/
uint32_t GetTick(void)
{
    return TIM_GetCounter(TIM2);
}
//���lwip_localtime
uint32_t get_lwip_localtime(void)
{
	return GetTick()/1000;
}
/*ΪLWIP�ṩʱ��
uint32_t sys_now(void)
{
	lwip_localtime = get_lwip_localtime();	//ms
	return lwip_localtime;
}*/


/********************************************* 
    ��TIM2����us��ʱ����
����:nus=��ʱus�� 
*********************************************/
void delay_us(uint32_t nus)
{
    uint32_t ntick;

    ntick = GetTick();
    while(GetTick() < ntick+nus); 
}
/********************************************* 
    ��TIM2����ms��ʱ����
����:nms=��ʱms�� 
*********************************************/
void delay_ms(uint16_t nms)
{
    uint32_t ntick;

    ntick = GetTick();
    while(GetTick() < ntick+(uint32_t)nms*1000); 
}

