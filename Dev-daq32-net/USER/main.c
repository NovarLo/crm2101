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
WORKINFO  WorkInfo;       //��������
AD_ARRAY  NewFrame;       //����1֡����
ADARRAYCTR ADArrayCtr;    //���ݶ��п���
//֡���ݶ���
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
��������STM32F407ΨһID��12�ֽ�,96bit
���: DecGuidStr= �豸Ψһ���ַ�����ʽ 
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
���������ϴ�ʹ�õĹ�������
**********************************************/
void GetWorkInfo(void)
{
	W25qX_Read(SPI1, (uint8_t *)&WorkInfo, W25_WORKINFO_ADD, sizeof(WORKINFO));
	if (WorkInfo.FirstFlag != 0xad1e)
	{ //�״��ϵ磬��Ĭ��ֵ
		WorkInfo.FirstFlag = 0xad1e;
		WorkInfo.ADParam.AD_gain = 0;   //10V
		WorkInfo.ADParam.AD_os = 0;     //�޹���������
		WorkInfo.ADParam.AD_freq = 48000;   //����Ƶ��=48KHz
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
���������浱ǰʹ�õĹ�������
***********************************************/
void SetWorkInfo(void)
{
	W25qX_Write(SPI1, (uint8_t *)&WorkInfo, W25_WORKINFO_ADD, sizeof(WORKINFO));
}

/*********************************************************** 
	AD_Array_Init
���������г�ʼ�� 
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

//��λAD����ز���
void AD_SoftReset(void)
{
	AD7606_Stop();
	AD7606_Init();
	AD_Array_Init();

	NVIC_SystemReset(); //ϵͳ��λ
}
/**************************************************************************
							 main ����
**************************************************************************/
int main(void)
{
	// Ƕ�������жϿ�������ѡ��
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	//delay_init(168);
	// ��ʼ������IO
	IO_GPIO_Config();
	// ��ʼ��W25q16(SPI1)
	if (W25qX_Init() < 0)
	{ //flash����ѭ��
		BELL_ON();
		while (1);
	}
	//���Ķ�ʱ��(TIM2)��ʼ��
	Tick_TIM2_Config();
#ifdef USART_DEBUG
	//��ʼ�����Դ���RS232(����1)
	uart_init(115200);
#endif
	//����������ʼ��
	memset((char *)&WorkInfo, 0, sizeof(WORKINFO));
	//��CPUID
	GetCPUID();
	GetWorkInfo();
	//AD7606��ʼ��
	AD7606_Init();
	//���ݶ��г�ʼ��
	AD_Array_Init();

	//�����ʼ��
	while (lwip_comm_init()) //lwip��ʼ��
	{
		// ʧ��
		BELL_ON();
		delay_ms(1200);
	}
	//��ʼ��OK
	BELL_OFF();
	delay_ms(1000);
	//UDP�������˳�ʼ��
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


