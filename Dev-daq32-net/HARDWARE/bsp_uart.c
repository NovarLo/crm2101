/*************************************************** 
    Usart 驱动 
Usart1:调试口，查询发送，字节中断接收，115200 
***************************************************/
#include "usart.h"

#include "main.h"
#include "bsp_ad7606.h"

uint8_t Uart1_ReciveBuf[DEBUG_232_RECIVEBUFF_SIZE];


//关闭UART1接收字节中断
void DisUart1RxInt(void)
{
    USART_ITConfig(DEBUG_232, USART_IT_RXNE, DISABLE);
}
//打开UART1接收字节中断
void EnUart1RxInt(void)
{
    USART_ITConfig(DEBUG_232, USART_IT_RXNE, ENABLE);
}
//---------------USART1------------------------------------------
/*****************************************
    接收空闲中断 配置嵌套向量中断控制器NVIC(0-6)
*****************************************/
static void UART1_RXNE_NVIC_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* 嵌套向量中断控制器组选择 */
    //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

    /* 配置USART为中断源 */
    NVIC_InitStructure.NVIC_IRQChannel = DEBUG_232_Rx_IRQ;
    /* 抢断优先级为 */
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    /* 子优先级为 */
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 6;
    /* 使能中断 */
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    /* 初始化配置NVIC */
    NVIC_Init(&NVIC_InitStructure);
}
/**********************************************************
    UART1配置
描述：使用USART1，工作模式：115200 8-N-1， 
      接收：USART字节接收中断
      发送：查询发送 
**********************************************************/
static void UART1_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    RCC_AHB1PeriphClockCmd(DEBUG_232_RX_GPIO_CLK|DEBUG_232_TX_GPIO_CLK,ENABLE);

    /* 使能 USART 时钟 */
    RCC_APB2PeriphClockCmd(DEBUG_232_CLK, ENABLE);

    /* GPIO初始化 */
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    /* 配置Tx引脚为复用功能  */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = DEBUG_232_TX_PIN;  
    GPIO_Init(DEBUG_232_TX_GPIO_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(DEBUG_232_TX_GPIO_PORT,DEBUG_232_TX_SOURCE,DEBUG_232_TX_AF);

    /* 配置Rx引脚为复用功能 */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = DEBUG_232_RX_PIN;
    GPIO_Init(DEBUG_232_RX_GPIO_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(DEBUG_232_RX_GPIO_PORT,DEBUG_232_RX_SOURCE,DEBUG_232_RX_AF);

    /* 波特率设置 */
    USART_InitStructure.USART_BaudRate = DEBUG_232_BAUDRATE;
    /* 字长(数据位+校验位)：8 */
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    /* 停止位：1个停止位 */
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    /* 校验位选择：不使用校验 */
    USART_InitStructure.USART_Parity = USART_Parity_No;
    /* 硬件流控制：不使用硬件流 */
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    /* USART模式控制：同时使能接收和发送 */
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    /* 完成USART初始化配置 */
    USART_Init(DEBUG_232, &USART_InitStructure); 
}

/**********************************************************
    DEBUG_232初始化
描述：使用USART1，工作模式：115200 8-N-1， 
      接收：DMA方式，USART空闲中断
      发送：DMA发送，DMA发送完成中断
**********************************************************/
void DEBUG_232_Init(void)
{
    UART1_Config();
    UART1_RXNE_NVIC_Config();
    // 使能串口 
    USART_Cmd(DEBUG_232, ENABLE);
}
/************************************************ 
描述：接收字节中断处理 
************************************************/
static void Uart_Rx_Data(USART_TypeDef* USARTx)
{


}
/************************************************ 
    DEBUG_232(串口1)中断服务函数
描述：接收启用串口总线字节中断 
************************************************/
void DEBUG_232_Rx_IRQHandler(void)
{
	if(USART_GetITStatus(DEBUG_232,USART_IT_RXNE)!=RESET)
	{//是接收中断		
        Uart_Rx_Data(DEBUG_232);        
        USART_ReceiveData(DEBUG_232);  /* 清除标志位 */
	}	 
}
/********************************************** 
    串口查询发送
***********************************************/
void Urat_Tx(USART_TypeDef* USARTx,uint8_t *pData, uint32_t length)
{
	uint32_t i = 0;
	
	while(i++ < length)
	{
		while((USARTx->SR & 0X40) == 0);
		USARTx->DR = *pData++;
	}
}
