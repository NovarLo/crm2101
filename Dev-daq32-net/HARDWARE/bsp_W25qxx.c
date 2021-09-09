/*********************************************************************
        bsp_W25Qxx.c
xx=16/32/64
SPI�ӿڣ�ʹ��SPI1 
4KbytesΪһ��Sector,16������Ϊ1��Block
256bytesΪһҳ,16ҳΪһ��Sector 
 
W25Q16������Ϊ2M�ֽ�,����32��Block,512��Sector 
W25Q32������Ϊ4M�ֽ�,����64��Block,1024��Sector 
W25Q64������Ϊ8M�ֽ�,����128��Block,2048��Sector  
*********************************************************************/
//#ifdef __DEBUG
#include "bsp_W25qxx.h"

extern void delay_us(uint32_t);
//��д�û�����
static uint8_t  W25X_RWBuf[4096];

/****************************************** 
    SPI1��ʼ��
********************************************/
static void bsp_SPI1Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef  SPI_InitStructure;

    RCC_APB2PeriphClockCmd(W25X_SPI_CLK, ENABLE);
    RCC_AHB1PeriphClockCmd (RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOG, ENABLE);

    // GPIO��ʼ��
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
    // FLASHоƬ ֧��SPIģʽ0��ģʽ3���ݴ�����CPOL CPHA
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

    /* ʹ�� FLASH_SPI  */
    SPI_Cmd(W25X_SPI, ENABLE);
}
/*************************************************************
    SPI_SendByte
������д1�ֽ�(8bit����) 
���룺*SPIx=ָ���Ĵ��� 
      dat=���͵�����
���أ����������ֽ�
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
��������1�ֽ�(8bit����)  
���룺*SPIx=ָ���Ĵ��� 
���أ����������ֽ�
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
��������ȡW25Qxx��״̬�Ĵ��� 
BIT7  6   5   4   3   2   1   0 
SPR   SEC  TB  BP2 BP1 BP0 WEL BUSY 
SPR:Ĭ��0,״̬�Ĵ�������λ,���WPʹ�� 
SEC,TB,BP2,BP1,BP0:FLASH����д�������� 
WEL:дʹ������ 
BUSY:æ���λ(1,æ;0,����) 
Ĭ��:0x00 
*******************************************/ 
static uint8_t W25qX_ReadSR(SPI_TypeDef *SPIx)
{
    uint8_t byte = 0; 
    W25X_CS_L();    //ʹ������ 
    SPI_SendByte(SPIx,W25X_ReadStatusReg);  //���Ͷ�ȡ״̬�Ĵ������� 
    byte = SPI_ReadByte(SPIx);              //��ȡһ���ֽ�
    W25X_CS_H();                          //ȡ��Ƭѡ 
    return byte;
}

/******************************************* 
    W25qX_WaitBusy
�������ȴ����л�ʱ 
���룺OverTime=��ʱʱ��(ms) 
*******************************************/
static void W25qX_WaitBusy(SPI_TypeDef *SPIx,uint16_t OverTime)
{
    uint32_t tick = 0;
    while(tick < OverTime)
    {
        if((W25qX_ReadSR(SPIx)&0x01)==0x00)//�ȴ�BUSYλ���
            break;
        tick++;
        delay_us(1000);
    }
}
/******************************************* 
    W25qX_WriteSR
������дW25Qxx״̬�Ĵ���,ֻ��SPR,SEC,TB,BP2,BP1,BP0����д 
���룺sr=�Ĵ���ֵ 
******************************************
static void W25qX_WriteSR(SPI_TypeDef *SPIx,uint8_t sr) 
{
    W25X_CS_L();    //ʹ������ 
    SPI_SendByte(SPIx,W25X_WriteStatusReg); //����дȡ״̬�Ĵ������� 
    SPI_SendByte(SPIx,sr);                  //д��һ���ֽ� 
    W25X_CS_H(); 
}*/ 
/******************************************* 
    W25qX_WriteEnable
������W25Qxxдʹ�ܣ���WEL��λ
*******************************************/ 
static void W25qX_WriteEnable(SPI_TypeDef *SPIx) 
{ 
    W25X_CS_L();
    SPI_SendByte(SPIx,W25X_WriteEnable);  //����дʹ�� 
    W25X_CS_H();
}
/******************************************* 
    W25qX_WriteDisable
������W25Qxxд��ֹ����WEL����
*******************************************/ 
static void W25qX_WriteDisable(SPI_TypeDef *SPIx) 
{ 
    W25X_CS_L();
    SPI_SendByte(SPIx,W25X_WriteDisable);  //����д��ָֹ��
    W25X_CS_H();
}
/******************************************* 
    W25qX_ReadChipID
��������ȡоƬID 
*******************************************/ 
static uint16_t W25qX_ReadChipID(SPI_TypeDef *SPIx) 
{  
    uint16_t Temp = 0;
    W25X_CS_L();
    SPI_SendByte(SPIx,W25X_ManufactDeviceID);   //���Ͷ�ȡID����
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
��������⵱ǰFlash�Ƿ�ΪW25Q16/32/64 
���أ�ʧ�ܷ���-1 
      W25Q16����16
      W25Q32����32
      W25Q64����64
	  W25Q128����128
	  W25Q256����256
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
��������һҳ(0~65535)��д������256���ֽڵ�����
���룺ps=��д�����ݴ洢��
      WriteAddr=��ʼд��ĵ�ַ(24bit)
      WriteSize=Ҫд����ֽ���(���256�Ҳ�Ӧ�ó�����ҳ��ʣ���ֽ���)
**********************************************/ 
static void W25qX_WritePage(SPI_TypeDef *SPIx,uint8_t *ps,uint32_t WriteAddr,uint16_t WriteSize) 
{ 
    uint16_t i;
    W25qX_WriteEnable(SPIx);
    W25X_CS_L();
    SPI_SendByte(SPIx,W25X_PageProgram);        //����дҳ���� 
    SPI_SendByte(SPIx,(uint8_t)(WriteAddr>>16));  //����24bit��ַ 
    SPI_SendByte(SPIx,(uint8_t)(WriteAddr>>8)); 
    SPI_SendByte(SPIx,(uint8_t)WriteAddr); 
    for(i=0;i<WriteSize;i++)
        SPI_SendByte(SPIx,ps[i]);   //ѭ��д�� 
    W25X_CS_H();
    W25qX_WaitBusy(SPIx,20);       //�ȴ�д����� 
    W25qX_WriteDisable(SPIx);
}
/************************************************
    W25qX_WriteNoCheck
�������޼���д,��ָ����ַ��ʼд��ָ�����ȵ����� 
      ��д����Ӧ�ѱ�����,�Զ���ҳ��
      ��д���׵�ַ������ҳ��ʣ��ռ䣬����д�ֽ���<ʣ��ռ��������
      ��������1ҳ��дʣ���ֽڹ��񣬹���ʣ���ֽ�ȫд�뼴��������������д��1ҳ����������ж�ֱ������
���룺ps=��д�����ݴ洢��
      WriteAddr=��ʼд��ĵ�ַ(24bit)
      WriteSize=Ҫд����ֽ���(���65535)
************************************************/
static void W25qX_WriteNoCheck(SPI_TypeDef *SPIx,uint8_t *ps,uint32_t WriteAddr,uint16_t WriteSize)
{ 
    uint16_t pageremain;
    pageremain = 256-WriteAddr%256; //��ҳʣ����ֽ���
    if(WriteSize <= pageremain)
        pageremain = WriteSize;     //������256���ֽ�(д1��ҳ�ͽ���)
    while(1)
    {    
        W25qX_WritePage(SPIx,ps,WriteAddr,pageremain); //д1ҳ
        if(WriteSize==pageremain)
            break;  //д�������
        else
        {//WriteSize>pageremain 
            ps += pageremain; 
            WriteAddr += pageremain; 
            WriteSize -= pageremain;    //��ȥ�Ѿ�д���˵��ֽ��� 
            if(WriteSize>256)
                pageremain = 256;       //һ�ο���д��256���ֽ� 
            else
                pageremain = WriteSize; //����256���ֽ��� 
        }
    }
}
/************************************************
    W25qX_EraseSector
����������һ������������һ������������ʱ��:150ms
���룺SectorCnt=������ַ0~511
************************************************/
static void W25qX_EraseSector(SPI_TypeDef *SPIx,uint32_t SectorCnt) 
{ 
    uint32_t SectorAddr;
    SectorAddr = SectorCnt*4096; 
    W25qX_WriteEnable(SPIx);
    W25qX_WaitBusy(SPIx,10); 
    W25X_CS_L();       //ʹ������ 
    SPI_SendByte(SPIx,W25X_SectorErase);        //������������ָ�� 
    SPI_SendByte(SPIx,(uint8_t)(SectorAddr>>16)); //����24bit��ַ 
    SPI_SendByte(SPIx,(uint8_t)(SectorAddr>>8)); 
    SPI_SendByte(SPIx,(uint8_t)SectorAddr); 
    W25X_CS_H();       //ȡ��Ƭѡ
    W25qX_WaitBusy(SPIx,200); //�ȴ�������� 
    W25qX_WriteDisable(SPIx);
}
/******************************************* 
    W25qX_Init
���������ýӿ�SPI�����оƬ�ͺ� 
���أ�ʧ�ܷ���-1 
      W25Q16����16
      W25Q32����32
      W25Q64����64
******************************************/ 
int16_t W25qX_Init(void)
{
    bsp_SPI1Init();    // ��ʼ��SPI�ӿ�
    return W25qX_Check();
} 
/************************************************
    W25qX_Read
��������ָ����ַ��ʼ��ȡָ�����ȵ�����
���룺pd=���������ݴ洢��
      ReadAddr=��ʼ�����ĵ�ַ(24bit)
      ReadSize=Ҫ�������ֽ���(���65535)
**********************************************/ 
void W25qX_Read(SPI_TypeDef *SPIx,uint8_t *pd,uint32_t ReadAddr,uint16_t ReadSize)
{ 
    uint16_t i;
    W25X_CS_L();
    SPI_SendByte(SPIx,W25X_ReadData);   //���Ͷ�ȡ����
    SPI_SendByte(SPIx,(uint8_t)(ReadAddr>>16));  //����24bit��ַ 
    SPI_SendByte(SPIx,(uint8_t)(ReadAddr>>8)); 
    SPI_SendByte(SPIx,(uint8_t)ReadAddr); 
    for(i=0;i<ReadSize;i++) 
    { 
        pd[i] = SPI_ReadByte(SPIx); //ѭ������ 
    } 
    W25X_CS_H();
}
/************************************************
    W25qX_Write
��������ָ����ַ��ʼд��ָ�����ȵ����ݣ��Դ��������� 
      ��д���׵�ַ������������ʣ��ռ䣬����д�ֽ���<ʣ��ռ��������
      ��������1������дʣ���ֽڷ񣬹���ʣ���ֽ�ȫд�뼴��������������д��1��������������ж�ֱ������
���룺ps=��д�����ݴ洢��
      WriteAddr=��ʼд��ĵ�ַ(24bit)
      WriteSize=Ҫд����ֽ���(���65535)
************************************************/
void W25qX_Write(SPI_TypeDef *SPIx,uint8_t *pr,uint32_t WriteAddr,uint16_t WriteSize)
{
    uint32_t secpos;
    uint16_t secoff;
    uint16_t secremain;
    uint16_t i;

    secpos = WriteAddr/4096;    //������ַ:4kΪ1����
    secoff = WriteAddr%4096;    //�����ڵ�ƫ�Ƶ�ַ 
    secremain = 4096-secoff;    //������ʣ��ռ��С
                                 
    if(WriteSize<=secremain)
        secremain = WriteSize;  //������д������1��������(<=4096���ֽ�)

    while(1)
    {
        W25qX_Read(SPIx,W25X_RWBuf,secpos*4096,4096); //������ǰ�����������ݵ�W25RWBuf��
        for(i=0;i<secremain;i++)
        {//У������
            if(W25X_RWBuf[secoff+i] != 0xff)
                break;  //����          
        }   
        if(i<secremain)
        {//��Ҫ���� 
            W25qX_EraseSector(SPIx,secpos);//�������������
            memcpy((char *)&W25X_RWBuf[secoff],(char *)pr,secremain);
            W25qX_WriteNoCheck(SPIx,W25X_RWBuf,secpos*4096,4096);//д����������
        }
        else
        {//����Ҫ����
            W25qX_WriteNoCheck(SPIx,pr,WriteAddr,secremain);//ֱ��д������ʣ������
        }
        if(WriteSize==secremain)
            break;
        else
        {
            secpos++;           //������ַ��1
            secoff = 0;         //ƫ��λ��Ϊ0
            pr += secremain;    //ָ��ƫ��
            WriteAddr += secremain;//д��ַƫ��
            WriteSize -= secremain;//�ֽ����ݼ�
            if(WriteSize>4096)
                secremain = 4096;   //��һ����������д����
            else
                secremain = WriteSize;//��һ����������д����
        }
    }
}
/************************************************
    W25qX_EraseChip
��������������оƬ������ʱ��:25s
************************************************/
void W25qX_EraseChip(SPI_TypeDef *SPIx) 
{
    W25qX_WriteEnable(SPIx);
    W25qX_WaitBusy(SPIx,10); 
    W25X_CS_L();     //ʹ������ 
    SPI_SendByte(SPIx,W25X_ChipErase);      //����Ƭ�������� 
    W25X_CS_H();       //ȡ��Ƭѡ
    W25qX_WaitBusy(SPIx,30000); //�ȴ��������
    W25qX_WriteDisable(SPIx);
}

//#endif
