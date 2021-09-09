/*************************************************** 
        bsp_Can.h                  
标准格式： 
   |<--仲裁区-->|<-控制器->|<--数据区-->|
SOF|11位标识|RTR|IDE|r0|DLC|1帧数据及CRC| 
扩展格式： 
   |<------仲裁区--------------->|<-控制器->|<--数据区-->|
SOF|11位标识|SRR|IDE|18位标识|RTR|r0|r0|DLC |1帧数据及CRC| 
 
标准格式下 
SOF：帧起始，1bit显位(0) 
11位标识:ID28~ID18,其中高7位不可全为隐性(1) 
RTR：远程发送请求位,0=数据帧，1=远程帧
IDE：0=标准格式,1=扩展格式 
r0：保留位=0 
DLC3~DLC0：数据字节数，0~8 
数据：0~8个字节，MSB先发 
CRC：由15位CRC序列和1位CRC界定符(1)组成 
 
CAN总线采用非归零(NRZ)编码，所以节点"线与"连接至总线 
采用短帧格式，每1帧有效字节数<=8字节 
速率=1MB/s时距离<40m, 节点数可达30个 
 
过滤器： 
FiR2：需关心的位置'1',不关心的位清'0' 
FiR1：存放标准值(仅关心的为有效) 
 
CAN_FilterInitTypeDef: 过滤器配置说明
{                                    屏蔽           列表                                            屏蔽       列表
    32位宽：[CAN_FilterIdHigh]     ID的高16bit   第1个ID高16bit;   16位宽：[CAN_FilterMaskIdHigh] 第2个ID     第3个ID
    32位宽：[CAN_FilterIdLow]      ID的低16bit   第1个ID低16bit;   16位宽：[CAN_FilterIdLow]      第1个ID     第1个ID 
    32位宽：[CAN_FilterMaskIdHigh] MASK的高16bit 第2个ID高16bit;   16位宽：[CAN_FilterMaskIdHigh] 第2个MASK   第4个ID
    32位宽：[CAN_FilterMaskIdLow]  MASK的低16bit 第2个ID低16bit;   16位宽：[CAN_FilterMaskIdLow]  第1个MASK   第2个ID
    该过滤被分配的FIFO(0:CAN_Filter_FIFO0/1:CAN_Filter_FIFO1)
    过滤器组号(0~13)，(总数=28，使用默认值，CAN1和CAN2各占14个)
    过滤器工作模式：0=屏蔽(CAN_FilterMode_IdMask),1=列表(CAN_FilterMode_IdList) 
    过滤器位宽:0=2个16bit,1=1个32bit
    使能或禁止该过滤器组
} 
 
本设计CAN总线规则： 
1、两级通讯，1个顶端节点A，1~16个末端节点B。 
2、A可向所有B发送命令，可接收所有B传来的数据 。
3、B只接受A的数据，只向A发送数据，B相互之间无交互。 
4、A的ID=100 0000 0000(0x400), B的ID=100 0000 xxxx(0x401~0x40f)
5、过滤器设计，A：16位宽，工作模式=屏蔽 MASK=111 1111 0000 
               B：16位宽，工作模式=列表 =100 0000 0000(0x400)，即A的ID
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
//CAN通讯控制 
typedef struct
{
    INT16U  TxLen;          // 发送数据字节数 
    INT16U  TxCnt;          // 已发送字节数
    INT16U  RxLen[2];       // 接收数据字节数
    INT16U  RxCnt[2];       // 当前已接收字节数
    INT8U   TxBuf[512];     // 发送缓冲区
    INT8U   RxBuf[2][512];  // 接收缓冲区

    INT8U   TxStartFlag;    // 启动发送1帧标志
    INT8U   ReTxIvlFlag;    // 重发1帧间隔定时标志
    INT16U  ReTxIvlCnt;     // 重发1帧间隔定时器
    INT8U   RxEndFlag[2];   // 收齐1包数据标志
}CANCOM;
extern CANCOM  CANCom;

/******************************************
标准标识符规则：
D10~D4:7bit域 = 100 0000  同一网络
D3~D0: 4bit节点=0000~1111 最多16个节点
*******************************************/
//我的标准标识符(节点=1) 
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

