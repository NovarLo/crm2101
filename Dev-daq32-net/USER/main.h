/***********************************************************************
    main.h
********************************************************************/ 
#ifndef _MAIN_H
#define _MAIN_H

#include "stm32f4xx.h"

//�������������ַ      
#define W25_WORKINFO_ADD    0

//1֡���ݱ���=10��(ÿ֡64*10�ֽ�)
#define AD_FRAMENum         10
//���ݶ������=10֡ 
#define AD_ARRAYNum         10

/*************************************************************************/ 
typedef union
{
    uint8_t v[4];
    uint32_t v32;
}B4_I1;
//���������ṹ(12)
typedef struct
{
    uint16_t tag;      
    uint8_t  AD_gain;   // ����(1=5V��0=10V)
    uint8_t  AD_os;     // ��������(0=��,1=2��,2=4��,3=8��,4=16��,5=32��,6=64��)
    uint32_t AD_freq;   // AD����Ƶ��(Hz)
    uint32_t AD_num;    // ������������
}ADPARAM;
//�������(16)
typedef struct
{
    uint8_t  IP[4];     //IP��ַ
    uint8_t  SubNet[4]; //��������
    uint8_t  Gate[4];   //����
    uint16_t Port;      //�˿�
    uint8_t  Mac[6];    //MAC
}NETSET;

//��������
typedef struct
{
    //�����ϵ��־
    uint16_t FirstFlag;
    uint16_t tag;       
    //��������
    ADPARAM  ADParam;
    //�������
    NETSET  NetSet;
}WORKINFO;

//֡��������(10�����=640�ֽ�)
typedef struct
{
    uint16_t AD_Result[AD_FRAMENum][32];  
}AD_ARRAY;

//֡���ݽ������й���
typedef struct
{
    AD_ARRAY  *Rpoint;            //��ָ��
    AD_ARRAY  *Wpoint;            //дָ��
    uint8_t   ChannelCnt;         //32ͨ����ɱ�־(D0:1~8ͨ��,D1:9~16ͨ��,D2:17~24ͨ��,D3:25~32ͨ��
    uint8_t   frameCnt;           //֡��������(0~9)
    uint8_t   NewframeOK;         //1=ˢ����1֡���ݣ�0=�տ�ʼδ���1֡����
    uint8_t   tag;  
}ADARRAYCTR;

//-----------------------------
extern WORKINFO  WorkInfo;          //��������
extern ADARRAYCTR ADArrayCtr;       //֡���ݶ���
extern AD_ARRAY  NewFrame;          //����1֡����
extern AD_ARRAY Array[AD_ARRAYNum]; //֡���ݶ���                                                                  
extern uint8_t   CPUIDbuf[12];      //CPUID
//extern uint16_t  ADIntCnt[4];
//------------------------------------------------
uint32_t byte2int(uint8_t *ps);
void int2byte(uint32_t dat,uint8_t *pd);
void GetWorkInfo(void);
void SetWorkInfo(void);
void AD_SoftReset(void);
void AD_Array_Init(void);
#endif
/********************************END OF FILE****************************/
