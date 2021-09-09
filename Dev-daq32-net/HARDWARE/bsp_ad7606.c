/************************************************* 
AD7606���� 
16bit,���� 
����PWM�����ΪAD�����ź�Դ��ʵ���Զ����������� 
*************************************************/
#include "bsp_AD7606.h"
#include "bsp_io.h"
#include "main.h"
#include "delay.h"

static AD_ARRAY oneFrame;   //����ת���е�1֡����
uint8_t AD_Run;             //AD����״̬0=ֹͣ��1=����
/******************************************
������AD7606��IO�ڳ�ʼ�� 
���룺��
�������
*****************************************/
static void AD7606_GPIO_Config(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

/* ʹ����ص�GPIOʱ�� */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA |
						   RCC_AHB1Periph_GPIOB |
						   RCC_AHB1Periph_GPIOC |
						   RCC_AHB1Periph_GPIOD |
						   RCC_AHB1Periph_GPIOE |
						   RCC_AHB1Periph_GPIOG,
						   ENABLE);

// ͨ�� GPIO ���� */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;

// D0~D15�����ź��� �����������
	GPIO_InitStructure.GPIO_Pin = AD7606_D0_GPIO_PIN;
	GPIO_Init(AD7606_D0_GPIO_PORT, &GPIO_InitStructure);
	GPIO_PinAFConfig(AD7606_D0_GPIO_PORT, AD7606_D0_GPIO_PinSource, GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin = AD7606_D1_GPIO_PIN;
	GPIO_Init(AD7606_D1_GPIO_PORT, &GPIO_InitStructure);
	GPIO_PinAFConfig(AD7606_D1_GPIO_PORT, AD7606_D1_GPIO_PinSource, GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin = AD7606_D2_GPIO_PIN;
	GPIO_Init(AD7606_D2_GPIO_PORT, &GPIO_InitStructure);
	GPIO_PinAFConfig(AD7606_D2_GPIO_PORT, AD7606_D2_GPIO_PinSource, GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin = AD7606_D3_GPIO_PIN;
	GPIO_Init(AD7606_D3_GPIO_PORT, &GPIO_InitStructure);
	GPIO_PinAFConfig(AD7606_D3_GPIO_PORT, AD7606_D3_GPIO_PinSource, GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin = AD7606_D4_GPIO_PIN;
	GPIO_Init(AD7606_D4_GPIO_PORT, &GPIO_InitStructure);
	GPIO_PinAFConfig(AD7606_D4_GPIO_PORT, AD7606_D4_GPIO_PinSource, GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin = AD7606_D5_GPIO_PIN;
	GPIO_Init(AD7606_D5_GPIO_PORT, &GPIO_InitStructure);
	GPIO_PinAFConfig(AD7606_D5_GPIO_PORT, AD7606_D5_GPIO_PinSource, GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin = AD7606_D6_GPIO_PIN;
	GPIO_Init(AD7606_D6_GPIO_PORT, &GPIO_InitStructure);
	GPIO_PinAFConfig(AD7606_D6_GPIO_PORT, AD7606_D6_GPIO_PinSource, GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin = AD7606_D7_GPIO_PIN;
	GPIO_Init(AD7606_D7_GPIO_PORT, &GPIO_InitStructure);
	GPIO_PinAFConfig(AD7606_D7_GPIO_PORT, AD7606_D7_GPIO_PinSource, GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin = AD7606_D8_GPIO_PIN;
	GPIO_Init(AD7606_D8_GPIO_PORT, &GPIO_InitStructure);
	GPIO_PinAFConfig(AD7606_D8_GPIO_PORT, AD7606_D8_GPIO_PinSource, GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin = AD7606_D9_GPIO_PIN;
	GPIO_Init(AD7606_D9_GPIO_PORT, &GPIO_InitStructure);
	GPIO_PinAFConfig(AD7606_D9_GPIO_PORT, AD7606_D9_GPIO_PinSource, GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin = AD7606_D10_GPIO_PIN;
	GPIO_Init(AD7606_D10_GPIO_PORT, &GPIO_InitStructure);
	GPIO_PinAFConfig(AD7606_D10_GPIO_PORT, AD7606_D10_GPIO_PinSource, GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin = AD7606_D11_GPIO_PIN;
	GPIO_Init(AD7606_D11_GPIO_PORT, &GPIO_InitStructure);
	GPIO_PinAFConfig(AD7606_D11_GPIO_PORT, AD7606_D11_GPIO_PinSource, GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin = AD7606_D12_GPIO_PIN;
	GPIO_Init(AD7606_D12_GPIO_PORT, &GPIO_InitStructure);
	GPIO_PinAFConfig(AD7606_D12_GPIO_PORT, AD7606_D12_GPIO_PinSource, GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin = AD7606_D13_GPIO_PIN;
	GPIO_Init(AD7606_D13_GPIO_PORT, &GPIO_InitStructure);
	GPIO_PinAFConfig(AD7606_D13_GPIO_PORT, AD7606_D13_GPIO_PinSource, GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin = AD7606_D14_GPIO_PIN;
	GPIO_Init(AD7606_D14_GPIO_PORT, &GPIO_InitStructure);
	GPIO_PinAFConfig(AD7606_D14_GPIO_PORT, AD7606_D14_GPIO_PinSource, GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin = AD7606_D15_GPIO_PIN;
	GPIO_Init(AD7606_D15_GPIO_PORT, &GPIO_InitStructure);
	GPIO_PinAFConfig(AD7606_D15_GPIO_PORT, AD7606_D15_GPIO_PinSource, GPIO_AF_FSMC);

// �����ź���
	GPIO_InitStructure.GPIO_Pin = AD7606_CS1_GPIO_PIN;
	GPIO_Init(AD7606_CS1_GPIO_PORT, &GPIO_InitStructure);
	GPIO_PinAFConfig(AD7606_CS1_GPIO_PORT, AD7606_CS1_GPIO_PinSource, GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin = AD7606_CS2_GPIO_PIN;
	GPIO_Init(AD7606_CS2_GPIO_PORT, &GPIO_InitStructure);
	GPIO_PinAFConfig(AD7606_CS2_GPIO_PORT, AD7606_CS2_GPIO_PinSource, GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin = AD7606_CS3_GPIO_PIN;
	GPIO_Init(AD7606_CS3_GPIO_PORT, &GPIO_InitStructure);
	GPIO_PinAFConfig(AD7606_CS3_GPIO_PORT, AD7606_CS3_GPIO_PinSource, GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin = AD7606_CS4_GPIO_PIN;
	GPIO_Init(AD7606_CS4_GPIO_PORT, &GPIO_InitStructure);
	GPIO_PinAFConfig(AD7606_CS4_GPIO_PORT, AD7606_CS4_GPIO_PinSource, GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin = AD7606_RD_GPIO_PIN;
	GPIO_Init(AD7606_RD_GPIO_PORT, &GPIO_InitStructure);
	GPIO_PinAFConfig(AD7606_RD_GPIO_PORT, AD7606_RD_GPIO_PinSource, GPIO_AF_FSMC);

// ͨ�����GPIO
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;

	GPIO_InitStructure.GPIO_Pin = AD7606_RANGE_GPIO_PIN;
	GPIO_Init(AD7606_RANGE_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = AD7606_RST_GPIO_PIN;
	GPIO_Init(AD7606_RST_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = AD7606_OS0_GPIO_PIN;
	GPIO_Init(AD7606_OS0_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = AD7606_OS1_GPIO_PIN;
	GPIO_Init(AD7606_OS1_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = AD7606_OS2_GPIO_PIN;
	GPIO_Init(AD7606_OS2_GPIO_PORT, &GPIO_InitStructure);
}
/**********************************************************************
AD7606_FSMC_Init
��������ʼ��FSMC����
���룺�� 
������� 

AD7606�����Ҫ��(3.3Vʱ)��RD���źŵ͵�ƽ���������21ns���ߵ�ƽ������̿��15ns��
ѡ��3-0-6-1-0-0:         ������������ ������������Ϊ�˺�ͬBANK��LCD������ͬ��
3-0-5-1-0-0  : RD��75ns���͵�ƽ����50ns.  1us���ڿɶ�ȡ8·�������ݵ��ڴ档
1-0-1-1-0-0  : RD��75ns���͵�ƽִ��12ns���ң��½��ز��Ҳ12ns.  ���ݶ�ȡ��ȷ��
*********************************************************************/
static void AD7606_FSMC_Init(void)
{
	FSMC_NORSRAMInitTypeDef  Init;
	FSMC_NORSRAMTimingInitTypeDef  Timing;

// ʹ��FSMC����ʱ��
	RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC, ENABLE);

//��ַ����ʱ�䣨ADDSET��Ϊ1��HCLK,1/168M = 6ns
	Timing.FSMC_AddressSetupTime = 0x03;
//��ַ����ʱ�䣨ADDHLD��
	Timing.FSMC_AddressHoldTime = 0x00;
//���ݱ���ʱ�䣨DATAST��+ 1��HCLK = 9/168M=54ns
	Timing.FSMC_DataSetupTime = 0x06;
//��������ת������
	Timing.FSMC_BusTurnAroundDuration = 0x01;
//����ʱ�ӷ�Ƶ
	Timing.FSMC_CLKDivision = 0x01; //0x00;
//���ݱ���ʱ��
	Timing.FSMC_DataLatency = 0x00;
//ѡ��ƥ���ģʽ
	Timing.FSMC_AccessMode = FSMC_AccessMode_A;
//CS1
//ѡ��FSMCӳ��Ĵ洢����Bank1 sram1
	Init.FSMC_Bank = FSMC_Bank1_NORSRAM1;
//���õ�ַ���������������Ƿ��ã�������NOR
	Init.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
//����Ҫ���ƵĴ洢�����ͣ�SRAM����
	Init.FSMC_MemoryType = FSMC_MemoryType_SRAM;
//�洢�����ݿ�ȣ�16λ
	Init.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;
//�����Ƿ�ʹ��ͻ������ģʽ
	Init.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;
//�����Ƿ�ʹ�ܵȴ��ź�
	Init.FSMC_AsynchronousWait = FSMC_AsynchronousWait_Disable;
//���õȴ��źŵ���Ч����
	Init.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
//�����Ƿ�֧�ְѷǶ����ͻ������
	Init.FSMC_WrapMode = FSMC_WrapMode_Disable;
//���õȴ��źŲ����ʱ�䣬������ͬ�����͵Ĵ洢��
	Init.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
//�洢��д��ֹ  //ʹ��
	Init.FSMC_WriteOperation = FSMC_WriteOperation_Disable; //FSMC_WriteOperation_Enable;
//��ʹ�õȴ��ź�
	Init.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
//��ʹ����չģʽ����дʹ����ͬ��ʱ��
	Init.FSMC_ExtendedMode = FSMC_ExtendedMode_Disable;
//ͻ��д����
	Init.FSMC_WriteBurst = FSMC_WriteBurst_Disable;
//��дʱ������
	Init.FSMC_ReadWriteTimingStruct = &Timing;
//��дͬ��ʱ��ʹ����չģʽʱ������ò���Ч
	Init.FSMC_WriteTimingStruct = &Timing;
//��ʼ��FSMC����
	FSMC_NORSRAMInit(&Init);
// ʹ��BANK
	FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM1, ENABLE);
//CS2
//ѡ��FSMCӳ��Ĵ洢����Bank1 sram2
	Init.FSMC_Bank = FSMC_Bank1_NORSRAM2;
//��ʼ��FSMC����
	FSMC_NORSRAMInit(&Init);
// ʹ��BANK
	FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM2, ENABLE);
//CS3
//ѡ��FSMCӳ��Ĵ洢����Bank1 sram3
	Init.FSMC_Bank = FSMC_Bank1_NORSRAM3;
//��ʼ��FSMC����
	FSMC_NORSRAMInit(&Init);
// ʹ��BANK
	FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM3, ENABLE);
//CS4
//ѡ��FSMCӳ��Ĵ洢����Bank1 sram4
	Init.FSMC_Bank = FSMC_Bank1_NORSRAM4;
//��ʼ��FSMC����
	FSMC_NORSRAMInit(&Init);
// ʹ��BANK
	FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM4, ENABLE);
}
/***************************************** 
NVIC_Configuration        
����������Ƕ�������жϿ�����NVIC 
����4·�ⲿ�ж�
���룺��
�������
*****************************************/
static void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

// ����NVICΪ���ȼ���0
//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
// �����ж�Դ��BUSY1
	NVIC_InitStructure.NVIC_IRQChannel = AD7606_BUSY1_EXTI_IRQ;
// ������ռ���ȼ� (��0,�޴�ֵ)
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
// ���������ȼ� (��0,ֵ=0..15)
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
// ʹ���ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
//BUSY2
	NVIC_InitStructure.NVIC_IRQChannel = AD7606_BUSY2_EXTI_IRQ;
	NVIC_Init(&NVIC_InitStructure);
//BUSY3
	NVIC_InitStructure.NVIC_IRQChannel = AD7606_BUSY3_EXTI_IRQ;
	NVIC_Init(&NVIC_InitStructure);
//BUSY4
	NVIC_InitStructure.NVIC_IRQChannel = AD7606_BUSY4_EXTI_IRQ;
	NVIC_Init(&NVIC_InitStructure);
}

/**************************************************
EXTI_BUSY_Config     
����������BUSY1..4Ϊ���жϿڣ��������ж����ȼ�
*************************************************/
static void EXTI_BUSY_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;

// ��������GPIO�ڵ�ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);
// ʹ�� SYSCFG ʱ�� ��ʹ��GPIO�ⲿ�ж�ʱ����ʹ��SYSCFGʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
// ���� NVIC
	NVIC_Configuration();

// ѡ��BUSY1������
	GPIO_InitStructure.GPIO_Pin = AD7606_BUSY1_GPIO_PIN;
// ��������Ϊ����ģʽ
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
// ������������
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(AD7606_BUSY1_GPIO_PORT, &GPIO_InitStructure);
// ���� EXTI �ж�Դ ��BUSY1����
	SYSCFG_EXTILineConfig(AD7606_BUSY1_EXTI_PortSource, AD7606_BUSY1_EXTI_PinSource);
// ѡ�� EXTI �ж�Դ
	EXTI_InitStructure.EXTI_Line = AD7606_BUSY1_EXTI_LINE;
// �ж�ģʽ
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
// �½��ش���
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //EXTI_Trigger_Rising;
// ʹ���ж�/�¼���
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

// ѡ��BUSY2������
	GPIO_InitStructure.GPIO_Pin = AD7606_BUSY2_GPIO_PIN;
	GPIO_Init(AD7606_BUSY2_GPIO_PORT, &GPIO_InitStructure);
// ���� EXTI �ж�Դ ��BUSY2����
	SYSCFG_EXTILineConfig(AD7606_BUSY2_EXTI_PortSource, AD7606_BUSY2_EXTI_PinSource);
// ѡ�� EXTI �ж�Դ
	EXTI_InitStructure.EXTI_Line = AD7606_BUSY2_EXTI_LINE;
	EXTI_Init(&EXTI_InitStructure);

// ѡ��BUSY3������
	GPIO_InitStructure.GPIO_Pin = AD7606_BUSY3_GPIO_PIN;
	GPIO_Init(AD7606_BUSY3_GPIO_PORT, &GPIO_InitStructure);
// ���� EXTI �ж�Դ ��BUSY3����
	SYSCFG_EXTILineConfig(AD7606_BUSY3_EXTI_PortSource, AD7606_BUSY3_EXTI_PinSource);
// ѡ�� EXTI �ж�Դ
	EXTI_InitStructure.EXTI_Line = AD7606_BUSY3_EXTI_LINE;
	EXTI_Init(&EXTI_InitStructure);

// ѡ��BUSY4������
	GPIO_InitStructure.GPIO_Pin = AD7606_BUSY4_GPIO_PIN;
	GPIO_Init(AD7606_BUSY4_GPIO_PORT, &GPIO_InitStructure);
// ���� EXTI �ж�Դ ��BUSY4����
	SYSCFG_EXTILineConfig(AD7606_BUSY4_EXTI_PortSource, AD7606_BUSY4_EXTI_PinSource);
// ѡ�� EXTI �ж�Դ
	EXTI_InitStructure.EXTI_Line = AD7606_BUSY4_EXTI_LINE;
	EXTI_Init(&EXTI_InitStructure);
}
/**************************************************
TIMx_GPIO_Config 
����������TIM�������PWMʱ�õ���I/O
**************************************************/
static void TIMx_GPIO_Config(void)
{
/*����һ��GPIO_InitTypeDef���͵Ľṹ��*/
	GPIO_InitTypeDef GPIO_InitStructure;

/*������ص�GPIO����ʱ��*/
	RCC_AHB1PeriphClockCmd(OCPWM1_GPIO_CLK, ENABLE);
/* ��ʱ��ͨ�����Ÿ��� */
	GPIO_PinAFConfig(OCPWM1_GPIO_PORT, OCPWM1_PinSource, OCPWM1_AF);

/* ��ʱ��ͨ���������� */
	GPIO_InitStructure.GPIO_Pin = OCPWM1_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(OCPWM1_GPIO_PORT, &GPIO_InitStructure);
}
/****************************************************************************** 
TIM_PWMOUTPUT_Config 
����������TIM���PWM����(TIM5-CH1) 
********************************************************************************/
static void TIM_PWMOUTPUT_Config(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

// ����TIMx_CLK,x[2,3,4,5,12,13,14]
	RCC_APB1PeriphClockCmd(OCPWM1_TIM_CLK, ENABLE);

// TIM_Period����
	TIM_TimeBaseStructure.TIM_Period = T50K - 1;
// Ԥ��Ƶ��
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
// ����ʱ�ӷ�Ƶ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
// ������ʽ
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(OCPWM1_TIM, &TIM_TimeBaseStructure);

/*PWMģʽ����*/
/* PWM Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;       //����ΪPWMģʽ1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 4;  //�͵�ƽ����=4*(1/84MHz)=48ns
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;     //����ʱ������ֵС��CCR1_ValʱΪ�ߵ�ƽ
	TIM_OC1Init(OCPWM1_TIM, &TIM_OCInitStructure);    //ʹ��ͨ��1

/*ʹ��ͨ��1����*/
	TIM_OC1PreloadConfig(OCPWM1_TIM, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(OCPWM1_TIM, ENABLE);

// ��ֹOC1���
	TIM_CCxCmd(OCPWM1_TIM, TIM_Channel_1, DISABLE);

// ʹ�ܶ�ʱ��
	TIM_Cmd(OCPWM1_TIM, ENABLE);
}

/*************************************** 
TIMx_PWMOut_Config 
��������ʼ����ʱ������Pwm���
***************************************/
static void TIMx_PWMOut_Config(void)
{
	TIMx_GPIO_Config();
	TIM_PWMOUTPUT_Config();
// ��ֹOC1���
	TIM_CCxCmd(OCPWM1_TIM, TIM_Channel_1, DISABLE);
}
/********************************** 
PWMOut_Start 
����������PWM��� 
���룺n=0Ƶ�ʲ��ı䣬n=1Ƶ�ʸı�
84MHz/(Autoreload+1)  
**********************************/
static void PWMOut_Start(uint8_t n)
{
	uint32_t Autoreload;

	if (n)
	{
		Autoreload = 84000000 / WorkInfo.ADParam.AD_freq - 1;
// ��ֹ��ʱ��
		TIM_Cmd(OCPWM1_TIM, DISABLE);
// ��������PWM�������
		TIM_SetAutoreload(OCPWM1_TIM, Autoreload);
		TIM_SetCompare1(OCPWM1_TIM, 4);
		TIM_SetCounter(OCPWM1_TIM, 0);
// ʹ�ܶ�ʱ��
		TIM_Cmd(OCPWM1_TIM, ENABLE);
	}
// ʹ��OC1���
	TIM_CCxCmd(OCPWM1_TIM, TIM_Channel_1, ENABLE);
}
/********************************** 
ֹͣPWM��� 
**********************************/
static void PWM1Out_Stop(void)
{
// ��ֹOC1���
	TIM_CCxCmd(OCPWM1_TIM, TIM_Channel_1, DISABLE);
}


/******************************************
AD7606_Init
������AD7606��ʼ�� 
���룺sn=AD7606оƬ���(1~4,��Ƭѡ����) 
*****************************************/
void AD7606_Init(void)
{
	AD7606_GPIO_Config();
	AD7606_FSMC_Init();
	AD7606_Reset();
	EXTI_BUSY_Config();
	TIMx_PWMOut_Config();
	EXTI_ClearITPendingBit(AD7606_BUSY1_EXTI_LINE | AD7606_BUSY2_EXTI_LINE | AD7606_BUSY3_EXTI_LINE | AD7606_BUSY4_EXTI_LINE);
	AD7606_StartConvst(1);
}

/*************************************************************
    AD7606_Reset
����: ��λAD7606
����: ��
����ֵ: ��
**************************************************************/
void AD7606_Reset(void)
{
	RST_H();
	delay_us(1);
	RST_L();
	delay_us(1);
}

/*************************************************************
AD7606_StartConvst
����: ����1��ADCת�� 
CONVST�ߵ�ƽ�����Ⱥ͵͵�ƽ���������25ns
CONVSTƽʱΪ��,�����ش���ת��
����: flag=0,����Ƶ�ʲ��䣬flag=1,����Ƶ�ʸı�
**************************************************************/
void AD7606_StartConvst(uint8_t flag)
{
	if (WorkInfo.ADParam.AD_gain == 0) RANGE_10V();
	else RANGE_5V();
	switch (WorkInfo.ADParam.AD_os)
	{
	case 1:
		AD_OS_X2    break;
	case 2:
		AD_OS_X4    break;
	case 3:
		AD_OS_X8    break;
	case 4:
		AD_OS_X16   break;
	case 5:
		AD_OS_X32   break;
	case 6:
		AD_OS_X64   break;
	default:
		AD_OS_NO    break;
	}
	PWMOut_Start(flag);
	AD_Run = 1;
	LED1_ON();
	LED2_ON();
	LED3_ON();
	LED4_ON();
}
/*************************************************************
AD7606_Stop
����: ֹͣADCת�� 
ֹͣPWM���
**************************************************************/
void AD7606_Stop(void)
{
	PWM1Out_Stop();
	AD_Run = 0;
	LED1_OFF();
	LED2_OFF();
	LED3_OFF();
	LED4_OFF();
}

/********************************************************* 
��������1֡������м���1֡   
*******************************************************/
static void WriteArray(void)
{
	if ((ADArrayCtr.ChannelCnt & 0x0f) == 0x0f)
	{ //32��ͨ��ȫд��,֡������+1
		ADArrayCtr.ChannelCnt = 0;
		ADArrayCtr.frameCnt++;
		if (ADArrayCtr.frameCnt >= AD_FRAMENum)
		{ //��10��
			ADArrayCtr.frameCnt = 0;
//ˢ�µ�ǰ1֡����
			memcpy(&NewFrame, &oneFrame, sizeof(AD_ARRAY));
			ADArrayCtr.NewframeOK = 1;
//����м���1֡����
			memcpy(ADArrayCtr.Wpoint, &NewFrame, sizeof(AD_ARRAY));
//дָ��+1����
			ADArrayCtr.Wpoint++;
			if (ADArrayCtr.Wpoint >= Array + AD_ARRAYNum) ADArrayCtr.Wpoint = Array;
		}
	}
}
/*************************************************************
BUSY1_IRQHandler
����: ��1ƬAD7606ת�������жϷ��� 
��ȡ1~8·��� 
����: ��
���: �� 
*************************************************************/
void BUSY1_IRQHandler(void)
{
	if (IS_BUSY1(0))
	{ //ȷ���Ƿ������EXTI Line�ж�
		LED1_ON();
		oneFrame.AD_Result[ADArrayCtr.frameCnt][0] = AD7606_1_RESULT(); //��ͨ��1���
		oneFrame.AD_Result[ADArrayCtr.frameCnt][1] = AD7606_1_RESULT(); //��ͨ��2���
		oneFrame.AD_Result[ADArrayCtr.frameCnt][2] = AD7606_1_RESULT(); //��ͨ��3���
		oneFrame.AD_Result[ADArrayCtr.frameCnt][3] = AD7606_1_RESULT(); //��ͨ��4���
		oneFrame.AD_Result[ADArrayCtr.frameCnt][4] = AD7606_1_RESULT(); //��ͨ��5���
		oneFrame.AD_Result[ADArrayCtr.frameCnt][5] = AD7606_1_RESULT(); //��ͨ��6���
		oneFrame.AD_Result[ADArrayCtr.frameCnt][6] = AD7606_1_RESULT(); //��ͨ��7���
		oneFrame.AD_Result[ADArrayCtr.frameCnt][7] = AD7606_1_RESULT(); //��ͨ��8���
		ADArrayCtr.ChannelCnt |= 0x01;
//����д�����
		WriteArray();
		LED1_OFF();
		LED2_ON();
		oneFrame.AD_Result[ADArrayCtr.frameCnt][8] = AD7606_2_RESULT();    //��ͨ��9���
		oneFrame.AD_Result[ADArrayCtr.frameCnt][9] = AD7606_2_RESULT();    //��ͨ��10���
		oneFrame.AD_Result[ADArrayCtr.frameCnt][10] = AD7606_2_RESULT();    //��ͨ��11���
		oneFrame.AD_Result[ADArrayCtr.frameCnt][11] = AD7606_2_RESULT();    //��ͨ��12���
		oneFrame.AD_Result[ADArrayCtr.frameCnt][12] = AD7606_2_RESULT();    //��ͨ��13���
		oneFrame.AD_Result[ADArrayCtr.frameCnt][13] = AD7606_2_RESULT();    //��ͨ��14���
		oneFrame.AD_Result[ADArrayCtr.frameCnt][14] = AD7606_2_RESULT();    //��ͨ��15���
		oneFrame.AD_Result[ADArrayCtr.frameCnt][15] = AD7606_2_RESULT();    //��ͨ��16���
		ADArrayCtr.ChannelCnt |= 0x02;
//����д�����
		WriteArray();
		LED2_OFF();
	}

//����жϱ�־λ
	EXTI_ClearITPendingBit(AD7606_BUSY1_EXTI_LINE);
}
/**************************************************************
BUSY2_IRQHandler
����: ��2ƬAD7606ת�������жϷ��� 
��ȡ9~16·��� 
����: ��
���: �� 
*************************************************************/
void BUSY2_IRQHandler(void)
{
	if (IS_BUSY2(0))
	{ //ȷ���Ƿ������EXTI Line�ж�
		LED2_ON();
		oneFrame.AD_Result[ADArrayCtr.frameCnt][8] = AD7606_2_RESULT();    //��ͨ��9���
		oneFrame.AD_Result[ADArrayCtr.frameCnt][9] = AD7606_2_RESULT();    //��ͨ��10���
		oneFrame.AD_Result[ADArrayCtr.frameCnt][10] = AD7606_2_RESULT();    //��ͨ��11���
		oneFrame.AD_Result[ADArrayCtr.frameCnt][11] = AD7606_2_RESULT();    //��ͨ��12���
		oneFrame.AD_Result[ADArrayCtr.frameCnt][12] = AD7606_2_RESULT();    //��ͨ��13���
		oneFrame.AD_Result[ADArrayCtr.frameCnt][13] = AD7606_2_RESULT();    //��ͨ��14���
		oneFrame.AD_Result[ADArrayCtr.frameCnt][14] = AD7606_2_RESULT();    //��ͨ��15���
		oneFrame.AD_Result[ADArrayCtr.frameCnt][15] = AD7606_2_RESULT();    //��ͨ��16���
		ADArrayCtr.ChannelCnt |= 0x02;
//����д�����
		WriteArray();
		LED2_OFF();
	}
//����жϱ�־λ
	EXTI_ClearITPendingBit(AD7606_BUSY2_EXTI_LINE);
}
/**************************************************************
BUSY3_IRQHandler
����: ��3ƬAD7606ת�������жϷ��� 
��ȡ17~24·��� 
����: ��
���: �� 
*************************************************************/
void BUSY3_IRQHandler(void)
{
	if (IS_BUSY3(0))
	{ //ȷ���Ƿ������EXTI Line�ж�
		LED3_ON();
		oneFrame.AD_Result[ADArrayCtr.frameCnt][16] = AD7606_3_RESULT();    //��ͨ��17���
		oneFrame.AD_Result[ADArrayCtr.frameCnt][17] = AD7606_3_RESULT();    //��ͨ��18���
		oneFrame.AD_Result[ADArrayCtr.frameCnt][18] = AD7606_3_RESULT();    //��ͨ��19���
		oneFrame.AD_Result[ADArrayCtr.frameCnt][19] = AD7606_3_RESULT();    //��ͨ��20���
		oneFrame.AD_Result[ADArrayCtr.frameCnt][20] = AD7606_3_RESULT();    //��ͨ��21���
		oneFrame.AD_Result[ADArrayCtr.frameCnt][21] = AD7606_3_RESULT();    //��ͨ��22���
		oneFrame.AD_Result[ADArrayCtr.frameCnt][22] = AD7606_3_RESULT();    //��ͨ��23���
		oneFrame.AD_Result[ADArrayCtr.frameCnt][23] = AD7606_3_RESULT();    //��ͨ��24���
		ADArrayCtr.ChannelCnt |= 0x04;
//����д�����
		WriteArray();
		LED3_OFF();
	}
//����жϱ�־λ
	EXTI_ClearITPendingBit(AD7606_BUSY3_EXTI_LINE);
}
/**************************************************************
BUSY4_IRQHandler
����: ��4ƬAD7606ת�������жϷ��� 
��ȡ25~32·��� 
����: ��
���: �� 
*************************************************************/
void BUSY4_IRQHandler(void)
{
	if (IS_BUSY4(0))
	{ //ȷ���Ƿ������EXTI Line�ж�
		LED4_ON();
		oneFrame.AD_Result[ADArrayCtr.frameCnt][24] = AD7606_4_RESULT();    //��ͨ��25���
		oneFrame.AD_Result[ADArrayCtr.frameCnt][25] = AD7606_4_RESULT();    //��ͨ��26���
		oneFrame.AD_Result[ADArrayCtr.frameCnt][26] = AD7606_4_RESULT();    //��ͨ��27���
		oneFrame.AD_Result[ADArrayCtr.frameCnt][27] = AD7606_4_RESULT();    //��ͨ��28���
		oneFrame.AD_Result[ADArrayCtr.frameCnt][28] = AD7606_4_RESULT();    //��ͨ��29���
		oneFrame.AD_Result[ADArrayCtr.frameCnt][29] = AD7606_4_RESULT();    //��ͨ��30���
		oneFrame.AD_Result[ADArrayCtr.frameCnt][30] = AD7606_4_RESULT();    //��ͨ��31���
		oneFrame.AD_Result[ADArrayCtr.frameCnt][31] = AD7606_4_RESULT();    //��ͨ��32���
		ADArrayCtr.ChannelCnt |= 0x08;
//����д�����
		WriteArray();
		LED4_OFF();
	}
//����жϱ�־λ
	EXTI_ClearITPendingBit(AD7606_BUSY4_EXTI_LINE);
}
