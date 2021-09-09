/************************************************************ 
    串口1发送和接收
不同的串口挂载的总线不一样，时钟使能函数也不一样
串口1和6是      RCC_APB2PeriphClockCmd
串口2/3/4/5是   RCC_APB1PeriphClockCmd 
------------------------------------------------- 
串口通讯协议：
                     头(2)    命令字(1)    数据(6)                      
启动     PC->407    0xaa 0x55    0x80     [测量范围,过采样率,采样频率]   
停止     PC->407    0xaa 0x55    0x90     00 00 00 00 00 00               
                     头(2)    数据长度(1)  数据(64)   
传数据   407->PC    0x5a 0xa5    64       [采样数据]                     
***********************************************************/
#ifndef __UART1_H
#define	__UART1_H

#include "stm32f4xx.h"

//USART1引脚定义
#define DEBUG_232                         USART1

#define DEBUG_232_CLK                     RCC_APB2Periph_USART1
#define DEBUG_232_BAUDRATE                115200  
             
#define DEBUG_232_RX_GPIO_PORT            GPIOA
#define DEBUG_232_RX_GPIO_CLK             RCC_AHB1Periph_GPIOA
#define DEBUG_232_RX_PIN                  GPIO_Pin_10
#define DEBUG_232_RX_AF                   GPIO_AF_USART1
#define DEBUG_232_RX_SOURCE               GPIO_PinSource10

#define DEBUG_232_TX_GPIO_PORT            GPIOA
#define DEBUG_232_TX_GPIO_CLK             RCC_AHB1Periph_GPIOA
#define DEBUG_232_TX_PIN                  GPIO_Pin_9
#define DEBUG_232_TX_AF                   GPIO_AF_USART1
#define DEBUG_232_TX_SOURCE               GPIO_PinSource9

#define DEBUG_232_Rx_IRQ                  USART1_IRQn

#define DEBUG_232_RECIVEBUFF_SIZE         9             // 接收的数据量


/************************************************************/
void DEBUG_232_Init(void);
void Urat_Tx(USART_TypeDef* USARTx,uint8_t *pData, uint32_t length);

void DisUart1RxInt(void);
void EnUart1RxInt(void);
//中断服务程序
void DEBUG_232_Rx_IRQHandler(void);

#endif /* __USART1_H */
