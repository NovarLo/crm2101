/***********************************************************************
	main.c
********************************************************************/
#include "main.h"
#include "bsp_AD7606.h"
#include "bsp_io.h"
#include "bsp_W25qxx.h"
#include "bsp_timer.h"
#include "lwip_comm.h"
#include "udp_server.h"
#ifdef USART_DEBUG
#include "usart.h"
#endif
#include "delay.h"

/**************************************************************************/
uint8_t   CPUIDbuf[12];   //CPUID
WORKINFO  WorkInfo;       //工作参数
AD_ARRAY  NewFrame;       //最新1帧数据
ADARRAYCTR ADArrayCtr;    //数据队列控制
//帧数据队列
AD_ARRAY  Array[AD_ARRAYNum]; // __attribute__ ((at(0x10000000)));
//uint16_t  ADIntCnt[4];
/*****************************************
byte:{0x01,0x02,0x03,0x04}->int:0x04030201
****************************************/
uint32_t byte2int(uint8_t *ps)
{
	uint8_t i;
	B4_I1  utemp;

	for (i = 0; i < 4; i++) utemp.v[i] = ps[i];
	return utemp.v32;
}
/*****************************************
int:0x01020304->byte:{0x04,0x03,0x02,0x01}
****************************************/
void int2byte(uint32_t dat, uint8_t *pd)
{
	uint8_t i;
	B4_I1  utemp;

	utemp.v32 = dat;
	for (i = 0; i < 4; i++) pd[i] = utemp.v[i];
}

/****************************************************** 
	GetCPUID
描述：读STM32F407唯一ID，12字节,96bit
输出: DecGuidStr= 设备唯一码字符串格式 
******************************************************/
static void GetCPUID(void)
{
	uint8_t i;

	for (i = 0; i < 12; i++)
	{
		CPUIDbuf[i] = *(uint8_t *)(0x1FFF7A10 + i);
	}
}

/*********************************************** 
	GetWorkInfo
描述：读上次使用的工作参数
**********************************************/
void GetWorkInfo(void)
{
	W25qX_Read(SPI1, (uint8_t *)&WorkInfo, W25_WORKINFO_ADD, sizeof(WORKINFO));
	if (WorkInfo.FirstFlag != 0xad1e)
	{ //首次上电，赋默认值
		WorkInfo.FirstFlag = 0xad1e;
		WorkInfo.ADParam.AD_gain = 0;   //10V
		WorkInfo.ADParam.AD_os = 0;     //无过过采样率
		WorkInfo.ADParam.AD_freq = 48000;   //采样频率=48KHz
		WorkInfo.ADParam.AD_num = 10;

		WorkInfo.NetSet.IP[0] = 192;
		WorkInfo.NetSet.IP[1] = 168;
		WorkInfo.NetSet.IP[2] = 1;
		WorkInfo.NetSet.IP[3] = 30;
		WorkInfo.NetSet.SubNet[0] = 255;
		WorkInfo.NetSet.SubNet[1] = 255;
		WorkInfo.NetSet.SubNet[2] = 255;
		WorkInfo.NetSet.SubNet[3] = 0;
		WorkInfo.NetSet.Gate[0] = 192;
		WorkInfo.NetSet.Gate[1] = 168;
		WorkInfo.NetSet.Gate[2] = 1;
		WorkInfo.NetSet.Gate[3] = 1;
		WorkInfo.NetSet.Port = 5050;
		WorkInfo.NetSet.Mac[0] = 2;
		WorkInfo.NetSet.Mac[1] = 0;
		WorkInfo.NetSet.Mac[2] = 0;
		WorkInfo.NetSet.Mac[3] = CPUIDbuf[0];
		WorkInfo.NetSet.Mac[4] = CPUIDbuf[1];
		WorkInfo.NetSet.Mac[5] = CPUIDbuf[2];

		W25qX_Write(SPI1, (uint8_t *)&WorkInfo, W25_WORKINFO_ADD, sizeof(WORKINFO));
	}
}
/*********************************************** 
	SetWorkInfo
描述：保存当前使用的工作参数
***********************************************/
void SetWorkInfo(void)
{
	W25qX_Write(SPI1, (uint8_t *)&WorkInfo, W25_WORKINFO_ADD, sizeof(WORKINFO));
}

/*********************************************************** 
	AD_Array_Init
描述：队列初始化 
***********************************************************/
void AD_Array_Init(void)
{
	//u8_t i;
	ADArrayCtr.Rpoint = Array;
	ADArrayCtr.Wpoint = Array;
	ADArrayCtr.frameCnt = 0;
	ADArrayCtr.ChannelCnt = 0;
	ADArrayCtr.NewframeOK = 0;
	//for(i=0;i<4;i++)
	//	ADIntCnt[i] = 0;
}

//复位AD及相关参数
void AD_SoftReset(void)
{
	AD7606_Stop();
	AD7606_Init();
	AD_Array_Init();

	NVIC_SystemReset(); //系统复位
}
/**************************************************************************
							 main 函数
**************************************************************************/
int main(void)
{
	// 嵌套向量中断控制器组选择
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	//delay_init(168);
	// 初始化基本IO
	IO_GPIO_Config();
	// 初始化W25q16(SPI1)
	if (W25qX_Init() < 0)
	{ //flash错，死循环
		BELL_ON();
		while (1);
	}
	//节拍定时器(TIM2)初始化
	Tick_TIM2_Config();
#ifdef USART_DEBUG
	//初始化调试串口RS232(串口1)
	uart_init(115200);
#endif
	//工作参数初始化
	memset((char *)&WorkInfo, 0, sizeof(WORKINFO));
	//读CPUID
	GetCPUID();
	GetWorkInfo();
	//AD7606初始化
	AD7606_Init();
	//数据队列初始化
	AD_Array_Init();

	//网络初始化
	while (lwip_comm_init()) //lwip初始化
	{
		// 失败
		BELL_ON();
		delay_ms(1200);
	}
	//初始化OK
	BELL_OFF();
	delay_ms(1000);
	//UDP服务器端初始化
	udp_echoserver_init();

	BELL_ON();
	delay_ms(500);
	BELL_OFF();

	while (1)
	{
		if (udp_server_flag & 0x03)
		{
			LED5_ON();
			ContinueSendTo();
			LED5_OFF();
		}
	}
}


