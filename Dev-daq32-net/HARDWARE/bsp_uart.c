/*************************************************** 
    Usart ���� 
Usart1:���Կڣ���ѯ���ͣ��ֽ��жϽ��գ�115200 
***************************************************/
#include "usart.h"

#include "main.h"
#include "bsp_ad7606.h"

uint8_t Uart1_ReciveBuf[DEBUG_232_RECIVEBUFF_SIZE];


//�ر�UART1�����ֽ��ж�
void DisUart1RxInt(void)
{
    USART_ITConfig(DEBUG_232, USART_IT_RXNE, DISABLE);
}
//��UART1�����ֽ��ж�
void EnUart1RxInt(void)
{
    USART_ITConfig(DEBUG_232, USART_IT_RXNE, ENABLE);
}
//---------------USART1------------------------------------------
/*****************************************
    ���տ����ж� ����Ƕ�������жϿ�����NVIC(0-6)
*****************************************/
static void UART1_RXNE_NVIC_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Ƕ�������жϿ�������ѡ�� */
    //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

    /* ����USARTΪ�ж�Դ */
    NVIC_InitStructure.NVIC_IRQChannel = DEBUG_232_Rx_IRQ;
    /* �������ȼ�Ϊ */
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    /* �����ȼ�Ϊ */
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 6;
    /* ʹ���ж� */
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    /* ��ʼ������NVIC */
    NVIC_Init(&NVIC_InitStructure);
}
/**********************************************************
    UART1����
������ʹ��USART1������ģʽ��115200 8-N-1�� 
      ���գ�USART�ֽڽ����ж�
      ���ͣ���ѯ���� 
**********************************************************/
static void UART1_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    RCC_AHB1PeriphClockCmd(DEBUG_232_RX_GPIO_CLK|DEBUG_232_TX_GPIO_CLK,ENABLE);

    /* ʹ�� USART ʱ�� */
    RCC_APB2PeriphClockCmd(DEBUG_232_CLK, ENABLE);

    /* GPIO��ʼ�� */
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    /* ����Tx����Ϊ���ù���  */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = DEBUG_232_TX_PIN;  
    GPIO_Init(DEBUG_232_TX_GPIO_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(DEBUG_232_TX_GPIO_PORT,DEBUG_232_TX_SOURCE,DEBUG_232_TX_AF);

    /* ����Rx����Ϊ���ù��� */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin = DEBUG_232_RX_PIN;
    GPIO_Init(DEBUG_232_RX_GPIO_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(DEBUG_232_RX_GPIO_PORT,DEBUG_232_RX_SOURCE,DEBUG_232_RX_AF);

    /* ���������� */
    USART_InitStructure.USART_BaudRate = DEBUG_232_BAUDRATE;
    /* �ֳ�(����λ+У��λ)��8 */
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    /* ֹͣλ��1��ֹͣλ */
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    /* У��λѡ�񣺲�ʹ��У�� */
    USART_InitStructure.USART_Parity = USART_Parity_No;
    /* Ӳ�������ƣ���ʹ��Ӳ���� */
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    /* USARTģʽ���ƣ�ͬʱʹ�ܽ��պͷ��� */
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    /* ���USART��ʼ������ */
    USART_Init(DEBUG_232, &USART_InitStructure); 
}

/**********************************************************
    DEBUG_232��ʼ��
������ʹ��USART1������ģʽ��115200 8-N-1�� 
      ���գ�DMA��ʽ��USART�����ж�
      ���ͣ�DMA���ͣ�DMA��������ж�
**********************************************************/
void DEBUG_232_Init(void)
{
    UART1_Config();
    UART1_RXNE_NVIC_Config();
    // ʹ�ܴ��� 
    USART_Cmd(DEBUG_232, ENABLE);
}
/************************************************ 
�����������ֽ��жϴ��� 
************************************************/
static void Uart_Rx_Data(USART_TypeDef* USARTx)
{


}
/************************************************ 
    DEBUG_232(����1)�жϷ�����
�������������ô��������ֽ��ж� 
************************************************/
void DEBUG_232_Rx_IRQHandler(void)
{
	if(USART_GetITStatus(DEBUG_232,USART_IT_RXNE)!=RESET)
	{//�ǽ����ж�		
        Uart_Rx_Data(DEBUG_232);        
        USART_ReceiveData(DEBUG_232);  /* �����־λ */
	}	 
}
/********************************************** 
    ���ڲ�ѯ����
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
