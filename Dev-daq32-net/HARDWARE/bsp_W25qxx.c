/*********************************************************************
        bsp_W25Qxx.c
xx=16/32/64
SPI接口，使用SPI1 
4Kbytes为一个Sector,16个扇区为1个Block
256bytes为一页,16页为一个Sector 
 
W25Q16：容量为2M字节,共有32个Block,512个Sector 
W25Q32：容量为4M字节,共有64个Block,1024个Sector 
W25Q64：容量为8M字节,共有128个Block,2048个Sector  
*********************************************************************/
//#ifdef __DEBUG
#include "bsp_W25qxx.h"

extern void delay_us(uint32_t);
//读写用缓存区
static uint8_t  W25X_RWBuf[4096];

/****************************************** 
    SPI1初始化
********************************************/
static void bsp_SPI1Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef  SPI_InitStructure;

    RCC_APB2PeriphClockCmd(W25X_SPI_CLK, ENABLE);
    RCC_AHB1PeriphClockCmd (RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOG, ENABLE);

    // GPIO初始化
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    // SCK
    GPIO_InitStructure.GPIO_Pin = W25X_SPI_SCK_PIN;
    GPIO_Init(W25X_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(W25X_SPI_SCK_GPIO_PORT,W25X_SPI_SCK_PINSOURCE,W25X_SPI_SCK_AF); 
    // MOSI
    GPIO_InitStructure.GPIO_Pin = W25X_SPI_MOSI_PIN;
    GPIO_Init(W25X_SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);  
    GPIO_PinAFConfig(W25X_SPI_MOSI_GPIO_PORT,W25X_SPI_MOSI_PINSOURCE,W25X_SPI_MOSI_AF); 
    // MISO
    GPIO_InitStructure.GPIO_Pin = W25X_SPI_MISO_PIN;
    GPIO_Init(W25X_SPI_MISO_GPIO_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(W25X_SPI_MISO_GPIO_PORT,W25X_SPI_MISO_PINSOURCE,W25X_SPI_MISO_AF); 
    // CS
    GPIO_InitStructure.GPIO_Pin = W25X_SPI_CS_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(W25X_SPI_CS_GPIO_PORT, &GPIO_InitStructure);

    W25X_CS_H();
    // FLASH芯片 支持SPI模式0及模式3，据此设置CPOL CPHA
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;  //84/2=42M
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(W25X_SPI, &SPI_InitStructure);

    /* 使能 FLASH_SPI  */
    SPI_Cmd(W25X_SPI, ENABLE);
}
/*************************************************************
    SPI_SendByte
描述：写1字节(8bit传输) 
输入：*SPIx=指定的串口 
      dat=发送的数据
返回：读回来的字节
************************************************************/
static uint8_t SPI_SendByte(SPI_TypeDef *SPIx, uint8_t dat)
{
    while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET); 
    SPIx->DR = dat;

    while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET); 
    return SPIx->DR;
}

/*************************************************************
    SPI_ReadByte
描述：读1字节(8bit传输)  
输入：*SPIx=指定的串口 
返回：读回来的字节
************************************************************/
static uint8_t SPI_ReadByte(SPI_TypeDef *SPIx)
{
    while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET); 
    SPIx->DR = 0xff;

    while(SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET); 
    return SPIx->DR;
}
/******************************************* 
        W25qX_ReadSR
描述：读取W25Qxx的状态寄存器 
BIT7  6   5   4   3   2   1   0 
SPR   SEC  TB  BP2 BP1 BP0 WEL BUSY 
SPR:默认0,状态寄存器保护位,配合WP使用 
SEC,TB,BP2,BP1,BP0:FLASH区域写保护设置 
WEL:写使能锁定 
BUSY:忙标记位(1,忙;0,空闲) 
默认:0x00 
*******************************************/ 
static uint8_t W25qX_ReadSR(SPI_TypeDef *SPIx)
{
    uint8_t byte = 0; 
    W25X_CS_L();    //使能器件 
    SPI_SendByte(SPIx,W25X_ReadStatusReg);  //发送读取状态寄存器命令 
    byte = SPI_ReadByte(SPIx);              //读取一个字节
    W25X_CS_H();                          //取消片选 
    return byte;
}

/******************************************* 
    W25qX_WaitBusy
描述：等待空闲或超时 
输入：OverTime=超时时长(ms) 
*******************************************/
static void W25qX_WaitBusy(SPI_TypeDef *SPIx,uint16_t OverTime)
{
    uint32_t tick = 0;
    while(tick < OverTime)
    {
        if((W25qX_ReadSR(SPIx)&0x01)==0x00)//等待BUSY位清空
            break;
        tick++;
        delay_us(1000);
    }
}
/******************************************* 
    W25qX_WriteSR
描述：写W25Qxx状态寄存器,只有SPR,SEC,TB,BP2,BP1,BP0可以写 
输入：sr=寄存器值 
******************************************
static void W25qX_WriteSR(SPI_TypeDef *SPIx,uint8_t sr) 
{
    W25X_CS_L();    //使能器件 
    SPI_SendByte(SPIx,W25X_WriteStatusReg); //发送写取状态寄存器命令 
    SPI_SendByte(SPIx,sr);                  //写入一个字节 
    W25X_CS_H(); 
}*/ 
/******************************************* 
    W25qX_WriteEnable
描述：W25Qxx写使能，将WEL置位
*******************************************/ 
static void W25qX_WriteEnable(SPI_TypeDef *SPIx) 
{ 
    W25X_CS_L();
    SPI_SendByte(SPIx,W25X_WriteEnable);  //发送写使能 
    W25X_CS_H();
}
/******************************************* 
    W25qX_WriteDisable
描述：W25Qxx写禁止，将WEL清零
*******************************************/ 
static void W25qX_WriteDisable(SPI_TypeDef *SPIx) 
{ 
    W25X_CS_L();
    SPI_SendByte(SPIx,W25X_WriteDisable);  //发送写禁止指令
    W25X_CS_H();
}
/******************************************* 
    W25qX_ReadChipID
描述：读取芯片ID 
*******************************************/ 
static uint16_t W25qX_ReadChipID(SPI_TypeDef *SPIx) 
{  
    uint16_t Temp = 0;
    W25X_CS_L();
    SPI_SendByte(SPIx,W25X_ManufactDeviceID);   //发送读取ID命令
    SPI_SendByte(SPIx,0x00);
    SPI_SendByte(SPIx,0x00);
    SPI_SendByte(SPIx,0x00);
    Temp |= SPI_ReadByte(SPIx)<<8; 
    Temp |= SPI_ReadByte(SPIx); 
    W25X_CS_H();
    return Temp;
}
/******************************************* 
    W25qX_Check
描述：检测当前Flash是否为W25Q16/32/64 
返回：失败返回-1 
      W25Q16返回16
      W25Q32返回32
      W25Q64返回64
	  W25Q128返回128
	  W25Q256返回256
******************************************/ 
static int16_t W25qX_Check(void) 
{ 
    uint16_t flashId = 0;
    flashId = W25qX_ReadChipID(W25X_SPI); 
    if(flashId == W25Q16_ID)
        return 16;
    else if(flashId == W25Q32_ID)
        return 32;
    else if(flashId == W25Q64_ID)
        return 32;
	else if(flashId == W25Q128_ID)
		return 128;
	else if(flashId == W25Q256_ID)
		return 256;	
    else
        return -1;
}
/************************************************
    W25qX_WritePage
描述：在一页(0~65535)内写入少于256个字节的数据
输入：ps=待写的数据存储区
      WriteAddr=开始写入的地址(24bit)
      WriteSize=要写入的字节数(最大256且不应该超过该页的剩余字节数)
**********************************************/ 
static void W25qX_WritePage(SPI_TypeDef *SPIx,uint8_t *ps,uint32_t WriteAddr,uint16_t WriteSize) 
{ 
    uint16_t i;
    W25qX_WriteEnable(SPIx);
    W25X_CS_L();
    SPI_SendByte(SPIx,W25X_PageProgram);        //发送写页命令 
    SPI_SendByte(SPIx,(uint8_t)(WriteAddr>>16));  //发送24bit地址 
    SPI_SendByte(SPIx,(uint8_t)(WriteAddr>>8)); 
    SPI_SendByte(SPIx,(uint8_t)WriteAddr); 
    for(i=0;i<WriteSize;i++)
        SPI_SendByte(SPIx,ps[i]);   //循环写数 
    W25X_CS_H();
    W25qX_WaitBusy(SPIx,20);       //等待写入结束 
    W25qX_WriteDisable(SPIx);
}
/************************************************
    W25qX_WriteNoCheck
描述：无检验写,从指定地址开始写入指定长度的数据 
      被写区域应已被擦除,自动换页。
      先写入首地址所处于页内剩余空间，若待写字节数<剩余空间则结束。
      否则再判1页够写剩余字节够否，够则剩余字节全写入即结束。不够则先写满1页，继续如此判断直至结束
输入：ps=待写的数据存储区
      WriteAddr=开始写入的地址(24bit)
      WriteSize=要写入的字节数(最大65535)
************************************************/
static void W25qX_WriteNoCheck(SPI_TypeDef *SPIx,uint8_t *ps,uint32_t WriteAddr,uint16_t WriteSize)
{ 
    uint16_t pageremain;
    pageremain = 256-WriteAddr%256; //单页剩余的字节数
    if(WriteSize <= pageremain)
        pageremain = WriteSize;     //不大于256个字节(写1次页就结束)
    while(1)
    {    
        W25qX_WritePage(SPIx,ps,WriteAddr,pageremain); //写1页
        if(WriteSize==pageremain)
            break;  //写入结束了
        else
        {//WriteSize>pageremain 
            ps += pageremain; 
            WriteAddr += pageremain; 
            WriteSize -= pageremain;    //减去已经写入了的字节数 
            if(WriteSize>256)
                pageremain = 256;       //一次可以写入256个字节 
            else
                pageremain = WriteSize; //不够256个字节了 
        }
    }
}
/************************************************
    W25qX_EraseSector
描述：擦除一个扇区，擦除一个扇区的最少时间:150ms
输入：SectorCnt=扇区地址0~511
************************************************/
static void W25qX_EraseSector(SPI_TypeDef *SPIx,uint32_t SectorCnt) 
{ 
    uint32_t SectorAddr;
    SectorAddr = SectorCnt*4096; 
    W25qX_WriteEnable(SPIx);
    W25qX_WaitBusy(SPIx,10); 
    W25X_CS_L();       //使能器件 
    SPI_SendByte(SPIx,W25X_SectorErase);        //发送扇区擦除指令 
    SPI_SendByte(SPIx,(uint8_t)(SectorAddr>>16)); //发送24bit地址 
    SPI_SendByte(SPIx,(uint8_t)(SectorAddr>>8)); 
    SPI_SendByte(SPIx,(uint8_t)SectorAddr); 
    W25X_CS_H();       //取消片选
    W25qX_WaitBusy(SPIx,200); //等待擦除完成 
    W25qX_WriteDisable(SPIx);
}
/******************************************* 
    W25qX_Init
描述：配置接口SPI，检查芯片型号 
返回：失败返回-1 
      W25Q16返回16
      W25Q32返回32
      W25Q64返回64
******************************************/ 
int16_t W25qX_Init(void)
{
    bsp_SPI1Init();    // 初始化SPI接口
    return W25qX_Check();
} 
/************************************************
    W25qX_Read
描述：在指定地址开始读取指定长度的数据
输入：pd=读出的数据存储区
      ReadAddr=开始读出的地址(24bit)
      ReadSize=要读出的字节数(最大65535)
**********************************************/ 
void W25qX_Read(SPI_TypeDef *SPIx,uint8_t *pd,uint32_t ReadAddr,uint16_t ReadSize)
{ 
    uint16_t i;
    W25X_CS_L();
    SPI_SendByte(SPIx,W25X_ReadData);   //发送读取命令
    SPI_SendByte(SPIx,(uint8_t)(ReadAddr>>16));  //发送24bit地址 
    SPI_SendByte(SPIx,(uint8_t)(ReadAddr>>8)); 
    SPI_SendByte(SPIx,(uint8_t)ReadAddr); 
    for(i=0;i<ReadSize;i++) 
    { 
        pd[i] = SPI_ReadByte(SPIx); //循环读数 
    } 
    W25X_CS_H();
}
/************************************************
    W25qX_Write
描述：从指定地址开始写入指定长度的数据，自带擦除操作 
      先写入首地址所处于扇区内剩余空间，若待写字节数<剩余空间则结束。
      否则再判1扇区够写剩余字节否，够则剩余字节全写入即结束。不够则先写满1扇区，继续如此判断直至结束
输入：ps=待写的数据存储区
      WriteAddr=开始写入的地址(24bit)
      WriteSize=要写入的字节数(最大65535)
************************************************/
void W25qX_Write(SPI_TypeDef *SPIx,uint8_t *pr,uint32_t WriteAddr,uint16_t WriteSize)
{
    uint32_t secpos;
    uint16_t secoff;
    uint16_t secremain;
    uint16_t i;

    secpos = WriteAddr/4096;    //扇区地址:4k为1扇区
    secoff = WriteAddr%4096;    //扇区内的偏移地址 
    secremain = 4096-secoff;    //扇区内剩余空间大小
                                 
    if(WriteSize<=secremain)
        secremain = WriteSize;  //本次所写内容在1个扇区内(<=4096个字节)

    while(1)
    {
        W25qX_Read(SPIx,W25X_RWBuf,secpos*4096,4096); //读出当前扇区整个内容到W25RWBuf中
        for(i=0;i<secremain;i++)
        {//校验数据
            if(W25X_RWBuf[secoff+i] != 0xff)
                break;  //不空          
        }   
        if(i<secremain)
        {//需要擦除 
            W25qX_EraseSector(SPIx,secpos);//擦除这个扇区　
            memcpy((char *)&W25X_RWBuf[secoff],(char *)pr,secremain);
            W25qX_WriteNoCheck(SPIx,W25X_RWBuf,secpos*4096,4096);//写入整个扇区
        }
        else
        {//不需要擦除
            W25qX_WriteNoCheck(SPIx,pr,WriteAddr,secremain);//直接写入扇区剩余区间
        }
        if(WriteSize==secremain)
            break;
        else
        {
            secpos++;           //扇区地址增1
            secoff = 0;         //偏移位置为0
            pr += secremain;    //指针偏移
            WriteAddr += secremain;//写地址偏移
            WriteSize -= secremain;//字节数递减
            if(WriteSize>4096)
                secremain = 4096;   //下一个扇区还是写不完
            else
                secremain = WriteSize;//下一个扇区可以写完了
        }
    }
}
/************************************************
    W25qX_EraseChip
描述：擦除整个芯片，最少时间:25s
************************************************/
void W25qX_EraseChip(SPI_TypeDef *SPIx) 
{
    W25qX_WriteEnable(SPIx);
    W25qX_WaitBusy(SPIx,10); 
    W25X_CS_L();     //使能器件 
    SPI_SendByte(SPIx,W25X_ChipErase);      //发送片擦除命令 
    W25X_CS_H();       //取消片选
    W25qX_WaitBusy(SPIx,30000); //等待擦除完成
    W25qX_WriteDisable(SPIx);
}

//#endif
