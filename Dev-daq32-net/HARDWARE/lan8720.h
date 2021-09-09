#ifndef __LAN8720_H
#define __LAN8720_H
#include "sys.h"
#include "stm32f4x7_eth.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//LAN8720 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/8/15
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 
//LAN8720????
//ETH_MDIO	
#define ETH_MDIO_GPIO_CLK       RCC_AHB1Periph_GPIOA
#define ETH_MDIO_PIN            GPIO_Pin_2               
#define ETH_MDIO_GPIO_PORT      GPIOA                     
#define ETH_MDIO_PinSource      GPIO_PinSource2
#define ETH_MDIO_AF             GPIO_AF_ETH
//ETH_MDC	
#define ETH_MDC_GPIO_CLK        RCC_AHB1Periph_GPIOC
#define ETH_MDC_PIN             GPIO_Pin_1               
#define ETH_MDC_GPIO_PORT       GPIOC                     
#define ETH_MDC_PinSource       GPIO_PinSource1
#define ETH_MDC_AF              GPIO_AF_ETH
//ETH_RMII_REF_CLK	
#define ETH_RMII_CLK_GPIO_CLK   RCC_AHB1Periph_GPIOA
#define ETH_RMII_CLK_PIN        GPIO_Pin_1               
#define ETH_RMII_CLK_GPIO_PORT  GPIOA                     
#define ETH_RMII_CLK_PinSource  GPIO_PinSource1
#define ETH_RMII_CLK_AF         GPIO_AF_ETH
//ETH_RMII_CRS_DV	
#define ETH_RMII_DV_GPIO_CLK    RCC_AHB1Periph_GPIOA
#define ETH_RMII_DV_PIN         GPIO_Pin_7               
#define ETH_RMII_DV_GPIO_PORT   GPIOA                     
#define ETH_RMII_DV_PinSource   GPIO_PinSource7
#define ETH_RMII_DV_AF          GPIO_AF_ETH
//ETH_RMII_RXD0	
#define ETH_RMII_RXD0_GPIO_CLK   RCC_AHB1Periph_GPIOC
#define ETH_RMII_RXD0_PIN        GPIO_Pin_4               
#define ETH_RMII_RXD0_GPIO_PORT  GPIOC                     
#define ETH_RMII_RXD0_PinSource  GPIO_PinSource4
#define ETH_RMII_RXD0_AF         GPIO_AF_ETH
//ETH_RMII_RXD1	
#define ETH_RMII_RXD1_GPIO_CLK   RCC_AHB1Periph_GPIOC
#define ETH_RMII_RXD1_PIN        GPIO_Pin_5               
#define ETH_RMII_RXD1_GPIO_PORT  GPIOC                     
#define ETH_RMII_RXD1_PinSource  GPIO_PinSource5
#define ETH_RMII_RXD1_AF         GPIO_AF_ETH
//ETH_RMII_TX_EN	
#define ETH_RMII_TXEN_GPIO_CLK   RCC_AHB1Periph_GPIOB   //RCC_AHB1Periph_GPIOG
#define ETH_RMII_TXEN_PIN        GPIO_Pin_11               
#define ETH_RMII_TXEN_GPIO_PORT  GPIOB                  //GPIOG                     
#define ETH_RMII_TXEN_PinSource  GPIO_PinSource11
#define ETH_RMII_TXEN_AF         GPIO_AF_ETH
//ETH_RMII_TXD0	
#define ETH_RMII_TXD0_GPIO_CLK   RCC_AHB1Periph_GPIOB   //RCC_AHB1Periph_GPIOG
#define ETH_RMII_TXD0_PIN        GPIO_Pin_12            //GPIO_Pin_13               
#define ETH_RMII_TXD0_GPIO_PORT  GPIOB                  //GPIOG                     
#define ETH_RMII_TXD0_PinSource  GPIO_PinSource12       //GPIO_PinSource13
#define ETH_RMII_TXD0_AF         GPIO_AF_ETH
//ETH_RMII_TXD1	
#define ETH_RMII_TXD1_GPIO_CLK   RCC_AHB1Periph_GPIOB   //RCC_AHB1Periph_GPIOG
#define ETH_RMII_TXD1_PIN        GPIO_Pin_13            //GPIO_Pin_14               
#define ETH_RMII_TXD1_GPIO_PORT  GPIOB                  //GPIOG                     
#define ETH_RMII_TXD1_PinSource  GPIO_PinSource13       //GPIO_PinSource14
#define ETH_RMII_TXD1_AF         GPIO_AF_ETH
//ETH_RESET
#define LAN8720_RST_GPIO_CLK    RCC_AHB1Periph_GPIOD
#define LAN8720_RST_PIN         GPIO_Pin_3               
#define LAN8720_RST_GPIO_PORT   GPIOD                     
#define LAN8720_RST_L()         LAN8720_RST_GPIO_PORT->BSRRH = LAN8720_RST_PIN     
#define LAN8720_RST_H()         LAN8720_RST_GPIO_PORT->BSRRL = LAN8720_RST_PIN                   


#define LAN8720_PHY_ADDRESS  	0x00				//LAN8720 PHY芯片地址.
//#define LAN8720_RST 		   	PDout(3) 			//LAN8720复位引脚	 

extern ETH_DMADESCTypeDef *DMARxDscrTab;			//以太网DMA接收描述符数据结构体指针
extern ETH_DMADESCTypeDef *DMATxDscrTab;			//以太网DMA发送描述符数据结构体指针 
extern uint8_t *Rx_Buff; 							//以太网底层驱动接收buffers指针 
extern uint8_t *Tx_Buff; 							//以太网底层驱动发送buffers指针
extern ETH_DMADESCTypeDef  *DMATxDescToSet;			//DMA发送描述符追踪指针
extern ETH_DMADESCTypeDef  *DMARxDescToGet; 		//DMA接收描述符追踪指针 
extern ETH_DMA_Rx_Frame_infos *DMA_RX_FRAME_infos;	//DMA最后接收到的帧信息指针
 

u8 LAN8720_Init(void);
u8 LAN8720_Get_Speed(void);
u8 ETH_MACDMA_Config(void);
FrameTypeDef ETH_Rx_Packet(void);
u8 ETH_Tx_Packet(u16 FrameLength);
u32 ETH_GetCurrentTxBuffer(void);
u8 ETH_Mem_Malloc(void);
void ETH_Mem_Free(void);
#endif 

