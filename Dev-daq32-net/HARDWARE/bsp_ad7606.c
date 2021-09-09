/************************************************* 
AD7606驱动 
16bit,并口 
采用PWM输出作为AD启动信号源，实现自动连续采样。 
*************************************************/
#include "bsp_AD7606.h"
#include "bsp_io.h"
#include "main.h"
#include "delay.h"

static AD_ARRAY oneFrame;   //正在转换中的1帧数据
uint8_t AD_Run;             //AD工作状态0=停止，1=运行
/******************************************
描述：AD7606的IO口初始化 
输入：无
输出：无
*****************************************/
static void AD7606_GPIO_Config(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

/* 使能相关的GPIO时钟 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA |
						   RCC_AHB1Periph_GPIOB |
						   RCC_AHB1Periph_GPIOC |
						   RCC_AHB1Periph_GPIOD |
						   RCC_AHB1Periph_GPIOE |
						   RCC_AHB1Periph_GPIOG,
						   ENABLE);

// 通用 GPIO 配置 */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;

// D0~D15数据信号线 针对引脚配置
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

// 控制信号线
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

// 通用输出GPIO
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
描述：初始化FSMC外设
输入：无 
输出：无 

AD7606规格书要求(3.3V时)：RD读信号低电平脉冲宽度最短21ns，高电平脉冲最短宽度15ns。
选择3-0-6-1-0-0:         按照如下配置 读数均正常。为了和同BANK的LCD配置相同，
3-0-5-1-0-0  : RD高75ns，低电平持续50ns.  1us以内可读取8路样本数据到内存。
1-0-1-1-0-0  : RD高75ns，低电平执行12ns左右，下降沿差不多也12ns.  数据读取正确。
*********************************************************************/
static void AD7606_FSMC_Init(void)
{
	FSMC_NORSRAMInitTypeDef  Init;
	FSMC_NORSRAMTimingInitTypeDef  Timing;

// 使能FSMC外设时钟
	RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC, ENABLE);

//地址建立时间（ADDSET）为1个HCLK,1/168M = 6ns
	Timing.FSMC_AddressSetupTime = 0x03;
//地址保持时间（ADDHLD）
	Timing.FSMC_AddressHoldTime = 0x00;
//数据保持时间（DATAST）+ 1个HCLK = 9/168M=54ns
	Timing.FSMC_DataSetupTime = 0x06;
//设置总线转换周期
	Timing.FSMC_BusTurnAroundDuration = 0x01;
//设置时钟分频
	Timing.FSMC_CLKDivision = 0x01; //0x00;
//数据保持时间
	Timing.FSMC_DataLatency = 0x00;
//选择匹配的模式
	Timing.FSMC_AccessMode = FSMC_AccessMode_A;
//CS1
//选择FSMC映射的存储区域：Bank1 sram1
	Init.FSMC_Bank = FSMC_Bank1_NORSRAM1;
//设置地址总线与数据总线是否复用，仅用于NOR
	Init.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
//设置要控制的存储器类型：SRAM类型
	Init.FSMC_MemoryType = FSMC_MemoryType_SRAM;
//存储器数据宽度：16位
	Init.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;
//设置是否使用突发访问模式
	Init.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;
//设置是否使能等待信号
	Init.FSMC_AsynchronousWait = FSMC_AsynchronousWait_Disable;
//设置等待信号的有效极性
	Init.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
//设置是否支持把非对齐的突发操作
	Init.FSMC_WrapMode = FSMC_WrapMode_Disable;
//设置等待信号插入的时间，仅用于同步类型的存储器
	Init.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
//存储器写禁止  //使能
	Init.FSMC_WriteOperation = FSMC_WriteOperation_Disable; //FSMC_WriteOperation_Enable;
//不使用等待信号
	Init.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
//不使用扩展模式，读写使用相同的时序
	Init.FSMC_ExtendedMode = FSMC_ExtendedMode_Disable;
//突发写操作
	Init.FSMC_WriteBurst = FSMC_WriteBurst_Disable;
//读写时序配置
	Init.FSMC_ReadWriteTimingStruct = &Timing;
//读写同样时序，使用扩展模式时这个配置才有效
	Init.FSMC_WriteTimingStruct = &Timing;
//初始化FSMC配置
	FSMC_NORSRAMInit(&Init);
// 使能BANK
	FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM1, ENABLE);
//CS2
//选择FSMC映射的存储区域：Bank1 sram2
	Init.FSMC_Bank = FSMC_Bank1_NORSRAM2;
//初始化FSMC配置
	FSMC_NORSRAMInit(&Init);
// 使能BANK
	FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM2, ENABLE);
//CS3
//选择FSMC映射的存储区域：Bank1 sram3
	Init.FSMC_Bank = FSMC_Bank1_NORSRAM3;
//初始化FSMC配置
	FSMC_NORSRAMInit(&Init);
// 使能BANK
	FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM3, ENABLE);
//CS4
//选择FSMC映射的存储区域：Bank1 sram4
	Init.FSMC_Bank = FSMC_Bank1_NORSRAM4;
//初始化FSMC配置
	FSMC_NORSRAMInit(&Init);
// 使能BANK
	FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM4, ENABLE);
}
/***************************************** 
NVIC_Configuration        
描述：配置嵌套向量中断控制器NVIC 
配置4路外部中断
输入：无
输出：无
*****************************************/
static void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

// 配置NVIC为优先级组0
//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
// 配置中断源：BUSY1
	NVIC_InitStructure.NVIC_IRQChannel = AD7606_BUSY1_EXTI_IRQ;
// 配置抢占优先级 (组0,无此值)
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
// 配置子优先级 (组0,值=0..15)
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
// 使能中断通道
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
描述：配置BUSY1..4为线中断口，并设置中断优先级
*************************************************/
static void EXTI_BUSY_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;

// 开启按键GPIO口的时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);
// 使能 SYSCFG 时钟 ，使用GPIO外部中断时必须使能SYSCFG时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
// 配置 NVIC
	NVIC_Configuration();

// 选择BUSY1的引脚
	GPIO_InitStructure.GPIO_Pin = AD7606_BUSY1_GPIO_PIN;
// 设置引脚为输入模式
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
// 设置引脚上拉
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(AD7606_BUSY1_GPIO_PORT, &GPIO_InitStructure);
// 连接 EXTI 中断源 到BUSY1引脚
	SYSCFG_EXTILineConfig(AD7606_BUSY1_EXTI_PortSource, AD7606_BUSY1_EXTI_PinSource);
// 选择 EXTI 中断源
	EXTI_InitStructure.EXTI_Line = AD7606_BUSY1_EXTI_LINE;
// 中断模式
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
// 下降沿触发
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //EXTI_Trigger_Rising;
// 使能中断/事件线
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

// 选择BUSY2的引脚
	GPIO_InitStructure.GPIO_Pin = AD7606_BUSY2_GPIO_PIN;
	GPIO_Init(AD7606_BUSY2_GPIO_PORT, &GPIO_InitStructure);
// 连接 EXTI 中断源 到BUSY2引脚
	SYSCFG_EXTILineConfig(AD7606_BUSY2_EXTI_PortSource, AD7606_BUSY2_EXTI_PinSource);
// 选择 EXTI 中断源
	EXTI_InitStructure.EXTI_Line = AD7606_BUSY2_EXTI_LINE;
	EXTI_Init(&EXTI_InitStructure);

// 选择BUSY3的引脚
	GPIO_InitStructure.GPIO_Pin = AD7606_BUSY3_GPIO_PIN;
	GPIO_Init(AD7606_BUSY3_GPIO_PORT, &GPIO_InitStructure);
// 连接 EXTI 中断源 到BUSY3引脚
	SYSCFG_EXTILineConfig(AD7606_BUSY3_EXTI_PortSource, AD7606_BUSY3_EXTI_PinSource);
// 选择 EXTI 中断源
	EXTI_InitStructure.EXTI_Line = AD7606_BUSY3_EXTI_LINE;
	EXTI_Init(&EXTI_InitStructure);

// 选择BUSY4的引脚
	GPIO_InitStructure.GPIO_Pin = AD7606_BUSY4_GPIO_PIN;
	GPIO_Init(AD7606_BUSY4_GPIO_PORT, &GPIO_InitStructure);
// 连接 EXTI 中断源 到BUSY4引脚
	SYSCFG_EXTILineConfig(AD7606_BUSY4_EXTI_PortSource, AD7606_BUSY4_EXTI_PinSource);
// 选择 EXTI 中断源
	EXTI_InitStructure.EXTI_Line = AD7606_BUSY4_EXTI_LINE;
	EXTI_Init(&EXTI_InitStructure);
}
/**************************************************
TIMx_GPIO_Config 
描述：配置TIM复用输出PWM时用到的I/O
**************************************************/
static void TIMx_GPIO_Config(void)
{
/*定义一个GPIO_InitTypeDef类型的结构体*/
	GPIO_InitTypeDef GPIO_InitStructure;

/*开启相关的GPIO外设时钟*/
	RCC_AHB1PeriphClockCmd(OCPWM1_GPIO_CLK, ENABLE);
/* 定时器通道引脚复用 */
	GPIO_PinAFConfig(OCPWM1_GPIO_PORT, OCPWM1_PinSource, OCPWM1_AF);

/* 定时器通道引脚配置 */
	GPIO_InitStructure.GPIO_Pin = OCPWM1_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(OCPWM1_GPIO_PORT, &GPIO_InitStructure);
}
/****************************************************************************** 
TIM_PWMOUTPUT_Config 
描述：配置TIM输出PWM参数(TIM5-CH1) 
********************************************************************************/
static void TIM_PWMOUTPUT_Config(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

// 开启TIMx_CLK,x[2,3,4,5,12,13,14]
	RCC_APB1PeriphClockCmd(OCPWM1_TIM_CLK, ENABLE);

// TIM_Period周期
	TIM_TimeBaseStructure.TIM_Period = T50K - 1;
// 预分频器
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
// 采样时钟分频
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
// 计数方式
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(OCPWM1_TIM, &TIM_TimeBaseStructure);

/*PWM模式配置*/
/* PWM Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;       //配置为PWM模式1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 4;  //低电平脉宽=4*(1/84MHz)=48ns
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;     //当定时器计数值小于CCR1_Val时为高电平
	TIM_OC1Init(OCPWM1_TIM, &TIM_OCInitStructure);    //使能通道1

/*使能通道1重载*/
	TIM_OC1PreloadConfig(OCPWM1_TIM, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(OCPWM1_TIM, ENABLE);

// 禁止OC1输出
	TIM_CCxCmd(OCPWM1_TIM, TIM_Channel_1, DISABLE);

// 使能定时器
	TIM_Cmd(OCPWM1_TIM, ENABLE);
}

/*************************************** 
TIMx_PWMOut_Config 
描述：初始化定时器用作Pwm输出
***************************************/
static void TIMx_PWMOut_Config(void)
{
	TIMx_GPIO_Config();
	TIM_PWMOUTPUT_Config();
// 禁止OC1输出
	TIM_CCxCmd(OCPWM1_TIM, TIM_Channel_1, DISABLE);
}
/********************************** 
PWMOut_Start 
描述：启动PWM输出 
输入：n=0频率不改变，n=1频率改变
84MHz/(Autoreload+1)  
**********************************/
static void PWMOut_Start(uint8_t n)
{
	uint32_t Autoreload;

	if (n)
	{
		Autoreload = 84000000 / WorkInfo.ADParam.AD_freq - 1;
// 禁止定时器
		TIM_Cmd(OCPWM1_TIM, DISABLE);
// 重新配置PWM输出参数
		TIM_SetAutoreload(OCPWM1_TIM, Autoreload);
		TIM_SetCompare1(OCPWM1_TIM, 4);
		TIM_SetCounter(OCPWM1_TIM, 0);
// 使能定时器
		TIM_Cmd(OCPWM1_TIM, ENABLE);
	}
// 使能OC1输出
	TIM_CCxCmd(OCPWM1_TIM, TIM_Channel_1, ENABLE);
}
/********************************** 
停止PWM输出 
**********************************/
static void PWM1Out_Stop(void)
{
// 禁止OC1输出
	TIM_CCxCmd(OCPWM1_TIM, TIM_Channel_1, DISABLE);
}


/******************************************
AD7606_Init
描述：AD7606初始化 
输入：sn=AD7606芯片序号(1~4,按片选定义) 
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
描述: 复位AD7606
输入: 无
返回值: 无
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
描述: 启动1次ADC转换 
CONVST高电平脉冲宽度和低电平脉冲宽度最短25ns
CONVST平时为高,上升沿触发转换
输入: flag=0,采样频率不变，flag=1,采样频率改变
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
描述: 停止ADC转换 
停止PWM输出
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
判数据满1帧则向队列加入1帧   
*******************************************************/
static void WriteArray(void)
{
	if ((ADArrayCtr.ChannelCnt & 0x0f) == 0x0f)
	{ //32个通道全写完,帧计数器+1
		ADArrayCtr.ChannelCnt = 0;
		ADArrayCtr.frameCnt++;
		if (ADArrayCtr.frameCnt >= AD_FRAMENum)
		{ //够10条
			ADArrayCtr.frameCnt = 0;
//刷新当前1帧数据
			memcpy(&NewFrame, &oneFrame, sizeof(AD_ARRAY));
			ADArrayCtr.NewframeOK = 1;
//向队列加入1帧数据
			memcpy(ADArrayCtr.Wpoint, &NewFrame, sizeof(AD_ARRAY));
//写指针+1处理
			ADArrayCtr.Wpoint++;
			if (ADArrayCtr.Wpoint >= Array + AD_ARRAYNum) ADArrayCtr.Wpoint = Array;
		}
	}
}
/*************************************************************
BUSY1_IRQHandler
描述: 第1片AD7606转换结束中断服务 
读取1~8路结果 
输入: 无
输出: 无 
*************************************************************/
void BUSY1_IRQHandler(void)
{
	if (IS_BUSY1(0))
	{ //确保是否产生了EXTI Line中断
		LED1_ON();
		oneFrame.AD_Result[ADArrayCtr.frameCnt][0] = AD7606_1_RESULT(); //读通道1结果
		oneFrame.AD_Result[ADArrayCtr.frameCnt][1] = AD7606_1_RESULT(); //读通道2结果
		oneFrame.AD_Result[ADArrayCtr.frameCnt][2] = AD7606_1_RESULT(); //读通道3结果
		oneFrame.AD_Result[ADArrayCtr.frameCnt][3] = AD7606_1_RESULT(); //读通道4结果
		oneFrame.AD_Result[ADArrayCtr.frameCnt][4] = AD7606_1_RESULT(); //读通道5结果
		oneFrame.AD_Result[ADArrayCtr.frameCnt][5] = AD7606_1_RESULT(); //读通道6结果
		oneFrame.AD_Result[ADArrayCtr.frameCnt][6] = AD7606_1_RESULT(); //读通道7结果
		oneFrame.AD_Result[ADArrayCtr.frameCnt][7] = AD7606_1_RESULT(); //读通道8结果
		ADArrayCtr.ChannelCnt |= 0x01;
//数据写入队列
		WriteArray();
		LED1_OFF();
		LED2_ON();
		oneFrame.AD_Result[ADArrayCtr.frameCnt][8] = AD7606_2_RESULT();    //读通道9结果
		oneFrame.AD_Result[ADArrayCtr.frameCnt][9] = AD7606_2_RESULT();    //读通道10结果
		oneFrame.AD_Result[ADArrayCtr.frameCnt][10] = AD7606_2_RESULT();    //读通道11结果
		oneFrame.AD_Result[ADArrayCtr.frameCnt][11] = AD7606_2_RESULT();    //读通道12结果
		oneFrame.AD_Result[ADArrayCtr.frameCnt][12] = AD7606_2_RESULT();    //读通道13结果
		oneFrame.AD_Result[ADArrayCtr.frameCnt][13] = AD7606_2_RESULT();    //读通道14结果
		oneFrame.AD_Result[ADArrayCtr.frameCnt][14] = AD7606_2_RESULT();    //读通道15结果
		oneFrame.AD_Result[ADArrayCtr.frameCnt][15] = AD7606_2_RESULT();    //读通道16结果
		ADArrayCtr.ChannelCnt |= 0x02;
//数据写入队列
		WriteArray();
		LED2_OFF();
	}

//清除中断标志位
	EXTI_ClearITPendingBit(AD7606_BUSY1_EXTI_LINE);
}
/**************************************************************
BUSY2_IRQHandler
描述: 第2片AD7606转换结束中断服务 
读取9~16路结果 
输入: 无
输出: 无 
*************************************************************/
void BUSY2_IRQHandler(void)
{
	if (IS_BUSY2(0))
	{ //确保是否产生了EXTI Line中断
		LED2_ON();
		oneFrame.AD_Result[ADArrayCtr.frameCnt][8] = AD7606_2_RESULT();    //读通道9结果
		oneFrame.AD_Result[ADArrayCtr.frameCnt][9] = AD7606_2_RESULT();    //读通道10结果
		oneFrame.AD_Result[ADArrayCtr.frameCnt][10] = AD7606_2_RESULT();    //读通道11结果
		oneFrame.AD_Result[ADArrayCtr.frameCnt][11] = AD7606_2_RESULT();    //读通道12结果
		oneFrame.AD_Result[ADArrayCtr.frameCnt][12] = AD7606_2_RESULT();    //读通道13结果
		oneFrame.AD_Result[ADArrayCtr.frameCnt][13] = AD7606_2_RESULT();    //读通道14结果
		oneFrame.AD_Result[ADArrayCtr.frameCnt][14] = AD7606_2_RESULT();    //读通道15结果
		oneFrame.AD_Result[ADArrayCtr.frameCnt][15] = AD7606_2_RESULT();    //读通道16结果
		ADArrayCtr.ChannelCnt |= 0x02;
//数据写入队列
		WriteArray();
		LED2_OFF();
	}
//清除中断标志位
	EXTI_ClearITPendingBit(AD7606_BUSY2_EXTI_LINE);
}
/**************************************************************
BUSY3_IRQHandler
描述: 第3片AD7606转换结束中断服务 
读取17~24路结果 
输入: 无
输出: 无 
*************************************************************/
void BUSY3_IRQHandler(void)
{
	if (IS_BUSY3(0))
	{ //确保是否产生了EXTI Line中断
		LED3_ON();
		oneFrame.AD_Result[ADArrayCtr.frameCnt][16] = AD7606_3_RESULT();    //读通道17结果
		oneFrame.AD_Result[ADArrayCtr.frameCnt][17] = AD7606_3_RESULT();    //读通道18结果
		oneFrame.AD_Result[ADArrayCtr.frameCnt][18] = AD7606_3_RESULT();    //读通道19结果
		oneFrame.AD_Result[ADArrayCtr.frameCnt][19] = AD7606_3_RESULT();    //读通道20结果
		oneFrame.AD_Result[ADArrayCtr.frameCnt][20] = AD7606_3_RESULT();    //读通道21结果
		oneFrame.AD_Result[ADArrayCtr.frameCnt][21] = AD7606_3_RESULT();    //读通道22结果
		oneFrame.AD_Result[ADArrayCtr.frameCnt][22] = AD7606_3_RESULT();    //读通道23结果
		oneFrame.AD_Result[ADArrayCtr.frameCnt][23] = AD7606_3_RESULT();    //读通道24结果
		ADArrayCtr.ChannelCnt |= 0x04;
//数据写入队列
		WriteArray();
		LED3_OFF();
	}
//清除中断标志位
	EXTI_ClearITPendingBit(AD7606_BUSY3_EXTI_LINE);
}
/**************************************************************
BUSY4_IRQHandler
描述: 第4片AD7606转换结束中断服务 
读取25~32路结果 
输入: 无
输出: 无 
*************************************************************/
void BUSY4_IRQHandler(void)
{
	if (IS_BUSY4(0))
	{ //确保是否产生了EXTI Line中断
		LED4_ON();
		oneFrame.AD_Result[ADArrayCtr.frameCnt][24] = AD7606_4_RESULT();    //读通道25结果
		oneFrame.AD_Result[ADArrayCtr.frameCnt][25] = AD7606_4_RESULT();    //读通道26结果
		oneFrame.AD_Result[ADArrayCtr.frameCnt][26] = AD7606_4_RESULT();    //读通道27结果
		oneFrame.AD_Result[ADArrayCtr.frameCnt][27] = AD7606_4_RESULT();    //读通道28结果
		oneFrame.AD_Result[ADArrayCtr.frameCnt][28] = AD7606_4_RESULT();    //读通道29结果
		oneFrame.AD_Result[ADArrayCtr.frameCnt][29] = AD7606_4_RESULT();    //读通道30结果
		oneFrame.AD_Result[ADArrayCtr.frameCnt][30] = AD7606_4_RESULT();    //读通道31结果
		oneFrame.AD_Result[ADArrayCtr.frameCnt][31] = AD7606_4_RESULT();    //读通道32结果
		ADArrayCtr.ChannelCnt |= 0x08;
//数据写入队列
		WriteArray();
		LED4_OFF();
	}
//清除中断标志位
	EXTI_ClearITPendingBit(AD7606_BUSY4_EXTI_LINE);
}
