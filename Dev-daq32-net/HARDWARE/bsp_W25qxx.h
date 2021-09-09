/*********************************************************************
        BSP_W25Qxx.h
xx=16/32/64
SPI接口，使用SPI1 
4Kbytes为一个Sector,16个扇区为1个Block
256bytes为一页,16页为一个Sector 
 
W25Q16：容量为2M字节,共有32个Block,512个Sector 
W25Q32：容量为4M字节,共有64个Block,1024个Sector 
W25Q64：容量为8M字节,共有128个Block,2048个Sector  
*********************************************************************/
#ifndef _BSP_W25X_H
#define _BSP_W25X_H

#include "stm32f4xx.h"

/*****SPI1接口定义****************************/
#define W25X_SPI                    SPI1
#define W25X_SPI_CLK                RCC_APB2Periph_SPI1

#define W25X_SPI_SCK_PIN            GPIO_Pin_3                  
#define W25X_SPI_SCK_GPIO_PORT      GPIOB                       
#define W25X_SPI_SCK_GPIO_CLK       RCC_AHB1Periph_GPIOB
#define W25X_SPI_SCK_PINSOURCE      GPIO_PinSource3
#define W25X_SPI_SCK_AF             GPIO_AF_SPI1
        
#define W25X_SPI_MISO_PIN           GPIO_Pin_4                
#define W25X_SPI_MISO_GPIO_PORT     GPIOB                   
#define W25X_SPI_MISO_GPIO_CLK      RCC_AHB1Periph_GPIOB
#define W25X_SPI_MISO_PINSOURCE     GPIO_PinSource4
#define W25X_SPI_MISO_AF            GPIO_AF_SPI1
        
#define W25X_SPI_MOSI_PIN           GPIO_Pin_5                
#define W25X_SPI_MOSI_GPIO_PORT     GPIOB                     
#define W25X_SPI_MOSI_GPIO_CLK      RCC_AHB1Periph_GPIOB
#define W25X_SPI_MOSI_PINSOURCE     GPIO_PinSource5
#define W25X_SPI_MOSI_AF            GPIO_AF_SPI1
        
#define W25X_SPI_CS_PIN             GPIO_Pin_15               
#define W25X_SPI_CS_GPIO_PORT       GPIOG                     
#define W25X_SPI_CS_GPIO_CLK        RCC_AHB1Periph_GPIOG
        
#define W25X_CS_L()      W25X_SPI_CS_GPIO_PORT->BSRRH = W25X_SPI_CS_PIN     
#define W25X_CS_H()      W25X_SPI_CS_GPIO_PORT->BSRRL = W25X_SPI_CS_PIN                   

/*****W25Q定义****************************/
//指令表 
#define W25X_WriteEnable		(0x06)  //写使能
#define W25X_WriteDisable		(0x04)  //写禁止
#define W25X_ReadStatusReg		(0x05)  //读状态寄存器1
#define W25X_ReadStatusReg2		(0x35) 	//读状态寄存器2
#define W25X_ReadStatusReg3		(0x15) 	//读状态寄存器3
#define W25X_WriteStatusReg     (0x01)  //写状态寄存器1
#define W25X_WriteStatusReg2    (0x31)  //写状态寄存器2
#define W25X_WriteStatusReg3    (0x11) 	//写状态寄存器3
#define W25X_ReadData			(0x03)  //读取数据
#define W25X_FastReadData		(0x0B) 
#define W25X_FastReadDual		(0x3B) 
#define W25X_PageProgram		(0x02)  //写页命令
#define W25X_BlockErase			(0xD8)  //块擦除
#define W25X_SectorErase		(0x20)  //扇区擦除 
#define W25X_ChipErase			(0xC7) 	//芯片擦除								
#define W25X_PowerDown			(0xB9)  //关机
#define W25X_ReleasePowerDown	(0xAB)  //释放关机
#define W25X_DeviceOnlyID       (0x4B)  //读取设备唯一ID 
#define W25X_ManufactDeviceID	(0x90)  //读取芯片类型ID 
#define W25X_JedecDeviceID		(0x9F) 
#define W25X_Enable4ByteAddr    (0xB7)
#define W25X_Exit4ByteAddr      (0xE9)

//W25Qxx芯片型号ID
#define W25Q80_ID 				(0xEF13) 	
#define W25Q16_ID 				(0xEF14)
#define W25Q32_ID 				(0xEF15)
#define W25Q64_ID 				(0xEF16)
#define W25Q128_ID				(0xEF17)
#define W25Q256_ID 				(0xEF18)

//1个扇区字节数=4k
#define eFLASH_SECTOR_SIZE      (0x1000)

/*******************************************/ 
int16_t W25qX_Init(void);
void W25qX_Read(SPI_TypeDef *SPIx,uint8_t *pd,uint32_t ReadAddr,uint16_t ReadSize);
void W25qX_Write(SPI_TypeDef *SPIx,uint8_t *pr,uint32_t WriteAddr,uint16_t WriteSize);
void W25qX_EraseChip(SPI_TypeDef *SPIx);

#endif
