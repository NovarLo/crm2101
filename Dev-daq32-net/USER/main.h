/***********************************************************************
    main.h
********************************************************************/ 
#ifndef _MAIN_H
#define _MAIN_H

#include "stm32f4xx.h"

//工作参数保存地址      
#define W25_WORKINFO_ADD    0

//1帧数据表长度=10条(每帧64*10字节)
#define AD_FRAMENum         10
//数据队列深度=10帧 
#define AD_ARRAYNum         10

/*************************************************************************/ 
typedef union
{
    uint8_t v[4];
    uint32_t v32;
}B4_I1;
//采样参数结构(12)
typedef struct
{
    uint16_t tag;      
    uint8_t  AD_gain;   // 增益(1=5V，0=10V)
    uint8_t  AD_os;     // 过采样率(0=无,1=2倍,2=4倍,3=8倍,4=16倍,5=32倍,6=64倍)
    uint32_t AD_freq;   // AD采样频率(Hz)
    uint32_t AD_num;    // 连续采样次数
}ADPARAM;
//网络参数(16)
typedef struct
{
    uint8_t  IP[4];     //IP地址
    uint8_t  SubNet[4]; //子网掩码
    uint8_t  Gate[4];   //网关
    uint16_t Port;      //端口
    uint8_t  Mac[6];    //MAC
}NETSET;

//工作参数
typedef struct
{
    //初次上电标志
    uint16_t FirstFlag;
    uint16_t tag;       
    //采样参数
    ADPARAM  ADParam;
    //网络参数
    NETSET  NetSet;
}WORKINFO;

//帧队列数据(10条结果=640字节)
typedef struct
{
    uint16_t AD_Result[AD_FRAMENum][32];  
}AD_ARRAY;

//帧数据交互队列管理
typedef struct
{
    AD_ARRAY  *Rpoint;            //读指针
    AD_ARRAY  *Wpoint;            //写指针
    uint8_t   ChannelCnt;         //32通道完成标志(D0:1~8通道,D1:9~16通道,D2:17~24通道,D3:25~32通道
    uint8_t   frameCnt;           //帧数计数器(0~9)
    uint8_t   NewframeOK;         //1=刷新了1帧数据，0=刚开始未完成1帧数据
    uint8_t   tag;  
}ADARRAYCTR;

//-----------------------------
extern WORKINFO  WorkInfo;          //工作参数
extern ADARRAYCTR ADArrayCtr;       //帧数据队列
extern AD_ARRAY  NewFrame;          //最新1帧数据
extern AD_ARRAY Array[AD_ARRAYNum]; //帧数据队列                                                                  
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
