/************************************************************ 
    ����1���ͺͽ���
��ͬ�Ĵ��ڹ��ص����߲�һ����ʱ��ʹ�ܺ���Ҳ��һ��
����1��6��      RCC_APB2PeriphClockCmd
����2/3/4/5��   RCC_APB1PeriphClockCmd 
------------------------------------------------- 
����ͨѶЭ�飺
                     ͷ(2)    ������(1)    ����(6)                      
����     PC->407    0xaa 0x55    0x80     [������Χ,��������,����Ƶ��]   
ֹͣ     PC->407    0xaa 0x55    0x90     00 00 00 00 00 00               
                     ͷ(2)    ���ݳ���(1)  ����(64)   
������   407->PC    0x5a 0xa5    64       [��������]                     
***********************************************************/
#ifndef __UART1_H
#define	__UART1_H

#include "stm32f4xx.h"

//USART1���Ŷ���
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

#define DEBUG_232_RECIVEBUFF_SIZE         9             // ���յ�������


/************************************************************/
void DEBUG_232_Init(void);
void Urat_Tx(USART_TypeDef* USARTx,uint8_t *pData, uint32_t length);

void DisUart1RxInt(void);
void EnUart1RxInt(void);
//�жϷ������
void DEBUG_232_Rx_IRQHandler(void);

#endif /* __USART1_H */
