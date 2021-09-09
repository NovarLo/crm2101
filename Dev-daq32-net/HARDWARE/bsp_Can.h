/*************************************************** 
        bsp_Can.h                  
��׼��ʽ�� 
   |<--�ٲ���-->|<-������->|<--������-->|
SOF|11λ��ʶ|RTR|IDE|r0|DLC|1֡���ݼ�CRC| 
��չ��ʽ�� 
   |<------�ٲ���--------------->|<-������->|<--������-->|
SOF|11λ��ʶ|SRR|IDE|18λ��ʶ|RTR|r0|r0|DLC |1֡���ݼ�CRC| 
 
��׼��ʽ�� 
SOF��֡��ʼ��1bit��λ(0) 
11λ��ʶ:ID28~ID18,���и�7λ����ȫΪ����(1) 
RTR��Զ�̷�������λ,0=����֡��1=Զ��֡
IDE��0=��׼��ʽ,1=��չ��ʽ 
r0������λ=0 
DLC3~DLC0�������ֽ�����0~8 
���ݣ�0~8���ֽڣ�MSB�ȷ� 
CRC����15λCRC���к�1λCRC�綨��(1)��� 
 
CAN���߲��÷ǹ���(NRZ)���룬���Խڵ�"����"���������� 
���ö�֡��ʽ��ÿ1֡��Ч�ֽ���<=8�ֽ� 
����=1MB/sʱ����<40m, �ڵ����ɴ�30�� 
 
�������� 
FiR2������ĵ�λ��'1',�����ĵ�λ��'0' 
FiR1����ű�׼ֵ(�����ĵ�Ϊ��Ч) 
 
CAN_FilterInitTypeDef: ����������˵��
{                                    ����           �б�                                            ����       �б�
    32λ��[CAN_FilterIdHigh]     ID�ĸ�16bit   ��1��ID��16bit;   16λ��[CAN_FilterMaskIdHigh] ��2��ID     ��3��ID
    32λ��[CAN_FilterIdLow]      ID�ĵ�16bit   ��1��ID��16bit;   16λ��[CAN_FilterIdLow]      ��1��ID     ��1��ID 
    32λ��[CAN_FilterMaskIdHigh] MASK�ĸ�16bit ��2��ID��16bit;   16λ��[CAN_FilterMaskIdHigh] ��2��MASK   ��4��ID
    32λ��[CAN_FilterMaskIdLow]  MASK�ĵ�16bit ��2��ID��16bit;   16λ��[CAN_FilterMaskIdLow]  ��1��MASK   ��2��ID
    �ù��˱������FIFO(0:CAN_Filter_FIFO0/1:CAN_Filter_FIFO1)
    ���������(0~13)��(����=28��ʹ��Ĭ��ֵ��CAN1��CAN2��ռ14��)
    ����������ģʽ��0=����(CAN_FilterMode_IdMask),1=�б�(CAN_FilterMode_IdList) 
    ������λ��:0=2��16bit,1=1��32bit
    ʹ�ܻ��ֹ�ù�������
} 
 
�����CAN���߹��� 
1������ͨѶ��1�����˽ڵ�A��1~16��ĩ�˽ڵ�B�� 
2��A��������B��������ɽ�������B���������� ��
3��Bֻ����A�����ݣ�ֻ��A�������ݣ�B�໥֮���޽����� 
4��A��ID=100 0000 0000(0x400), B��ID=100 0000 xxxx(0x401~0x40f)
5����������ƣ�A��16λ������ģʽ=���� MASK=111 1111 0000 
               B��16λ������ģʽ=�б� =100 0000 0000(0x400)����A��ID
****************************************************/

#ifndef __BSP_CAN_H
#define	__BSP_CAN_H

#include "stm32f4xx.h"

//----------------------------------------------
#define CANx                    CAN2
#define CAN_CLK                 RCC_APB1Periph_CAN1 | RCC_APB1Periph_CAN2
#define CAN_RX_IRQ				CAN2_RX0_IRQn
#define CAN_RX_IRQHandler		CAN2_RX0_IRQHandler
//Can-Rx
#define CAN_RX_GPIO_PORT        GPIOB
#define CAN_RX_PIN              GPIO_Pin_12
#define CAN_RX_GPIO_CLK         RCC_AHB1Periph_GPIOB
#define CAN_RX_SOURCE           GPIO_PinSource12
#define CAN_RX_AF               GPIO_AF_CAN2
//Can-Tx
#define CAN_TX_GPIO_PORT        GPIOB
#define CAN_TX_PIN              GPIO_Pin_13
#define CAN_TX_GPIO_CLK         RCC_AHB1Periph_GPIOB
#define CAN_TX_SOURCE           GPIO_PinSource13 
#define CAN_TX_AF               GPIO_AF_CAN2
//----------------------------------------------
//CANͨѶ���� 
typedef struct
{
    INT16U  TxLen;          // ���������ֽ��� 
    INT16U  TxCnt;          // �ѷ����ֽ���
    INT16U  RxLen[2];       // ���������ֽ���
    INT16U  RxCnt[2];       // ��ǰ�ѽ����ֽ���
    INT8U   TxBuf[512];     // ���ͻ�����
    INT8U   RxBuf[2][512];  // ���ջ�����

    INT8U   TxStartFlag;    // ��������1֡��־
    INT8U   ReTxIvlFlag;    // �ط�1֡�����ʱ��־
    INT16U  ReTxIvlCnt;     // �ط�1֡�����ʱ��
    INT8U   RxEndFlag[2];   // ����1�����ݱ�־
}CANCOM;
extern CANCOM  CANCom;

/******************************************
��׼��ʶ������
D10~D4:7bit�� = 100 0000  ͬһ����
D3~D0: 4bit�ڵ�=0000~1111 ���16���ڵ�
*******************************************/
//�ҵı�׼��ʶ��(�ڵ�=1) 
#define DevBStdID    0x401   

//----------------------------------------------
static void CAN_GPIO_Config(void);
static void CAN_NVIC_Config(void);
static void CAN_Mode_Config(void);
static void CAN_Filter_Config(void);
void CAN_Config(void);
void CAN_SetMsg(CanTxMsg *TxMessage);
void Init_RxMes(CanRxMsg *RxMessage);

#endif

