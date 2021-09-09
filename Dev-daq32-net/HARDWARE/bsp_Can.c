/************************************************************* 
    bsp_Can.c
������Can�������� 
ʹ�ñ�׼��ʽ(11λ��ʶ) 
*************************************************************/
#include "bsp_Can.h"

/************************************************************ 
    CAN_GPIO_Config
������Canʹ��GPIO��ʼ��
************************************************************/
static void CAN_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // Enable GPIO clock
    RCC_AHB1PeriphClockCmd(CAN_TX_GPIO_CLK|CAN_RX_GPIO_CLK, ENABLE);
	
    // Configure CAN TX pins  
    GPIO_InitStructure.GPIO_Pin = CAN_TX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(CAN_TX_GPIO_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(CAN_TX_GPIO_PORT, CAN_RX_SOURCE, CAN_TX_AF);
	
	// Configure CAN RX  pins  
    GPIO_InitStructure.GPIO_Pin = CAN_RX_PIN ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init(CAN_RX_GPIO_PORT, &GPIO_InitStructure);
    GPIO_PinAFConfig(CAN_RX_GPIO_PORT, CAN_TX_SOURCE, CAN_RX_AF); 
}

/*
 * ��������CAN_NVIC_Config
 * ����  ��CAN��NVIC ����,��1���ȼ��飬0��0���ȼ�
 * ����  ����
 * ���  : ��
 * ����  ���ڲ�����
 */
/****************************************************
    CAN�����ж� ����Ƕ�������жϿ�����NVIC (0-7) 
****************************************************/
static void CAN_NVIC_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    // Ƕ�������жϿ�������ѡ��  
    //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
    // ����USARTΪ�ж�Դ  
    NVIC_InitStructure.NVIC_IRQChannel = CAN_RX_IRQ;
    // �������ȼ�Ϊ  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    // �����ȼ�Ϊ  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 7;
    // ʹ���ж�   
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    // ��ʼ������NVIC  
    NVIC_Init(&NVIC_InitStructure);
}
/****************************************************
    CAN����ģʽ��ʼ��
****************************************************/
static void CAN_Mode_Config(void)
{
	CAN_InitTypeDef        CAN_InitStructure;
	// CANͨ�Ų������� 
	// Enable CAN clock 
    RCC_APB1PeriphClockCmd(CAN_CLK, ENABLE);

	//CAN�Ĵ�����ʼ�� 
	CAN_DeInit(CANx);
	CAN_StructInit(&CAN_InitStructure);

	//CAN��Ԫ��ʼ�� 
	CAN_InitStructure.CAN_TTCM = DISABLE;			// ��ֹʱ�䴥��ͨ��ģʽ
	CAN_InitStructure.CAN_ABOM = ENABLE;			// �Զ����߹��� 
	CAN_InitStructure.CAN_AWUM = ENABLE;			// ʹ���Զ�����ģʽ
	CAN_InitStructure.CAN_NART = DISABLE;			// ʹ�ñ����Զ��ش� 
	CAN_InitStructure.CAN_RFLM = ENABLE;			// ����FIFO ����ģʽ(���ʱ�±��Ĳ��Ḳ��ԭ�б���)
	CAN_InitStructure.CAN_TXFP = ENABLE;			// ��˳����FIFO(������ʶ�������ȼ�) 
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;   // ��������ģʽ
	CAN_InitStructure.CAN_SJW = CAN_SJW_2tq;		// ����ͬ����Ծ��� 2��ʱ�䵥Ԫ
	 
	/* ss=1 bs1=4 bs2=2 λʱ����Ϊ(1+4+2) �����ʼ�Ϊʱ������tq*(1+4+2)  */
	CAN_InitStructure.CAN_BS1 = CAN_BS1_4tq;		//BTR-TS1 ʱ���1 ռ����4��ʱ�䵥Ԫ
	CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;		//BTR-TS1 ʱ���2 ռ����2��ʱ�䵥Ԫ	
	
	/* CAN Baudrate = 1 MBps  (CAN ʱ��Ƶ��Ϊ APB1 = 42 MHz) */
	CAN_InitStructure.CAN_Prescaler = 6;		    // �����ʷ�Ƶ��  ������ʱ�䵥Ԫ��ʱ�䳤�� 42/(1+4+2)/6=1 Mbps
	CAN_Init(CANx, &CAN_InitStructure);
}

/***************************************************
    CAN�Ĺ��������ó�ʼ��
****************************************************/
static void CAN_Filter_Config(void)
{
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;

	//CANɸѡ����ʼ�� 
	CAN_FilterInitStructure.CAN_FilterNumber = 0;					//ɸѡ����=0��1��
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdList;	//�������б�ģʽ
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_16bit;//ɸѡ��λ��Ϊ����16λ��
	//ʹ��ɸѡ�������ձ�־�����ݽ��бȶ�ɸѡ����չID�������µľ����������ǵĻ��������FIFO0��
    //�б��н�1��ID�����ڵ�1��λ�ã�����λ��ȫд1 
    //ID[10:0]|RTR|IDE|EXID[17:15] 
    CAN_FilterInitStructure.CAN_FilterIdLow = ((u16)DevBStdID<<5)|0x0007;
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0xFFFF;		 
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0xFFFF;			 
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0xFFFF;			 

    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0 ;//ɸѡ����������FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;			//ʹ��ɸѡ��
	CAN_FilterInit(&CAN_FilterInitStructure);
	//CANͨ���ж�ʹ�� 
	CAN_ITConfig(CANx, CAN_IT_FMP0, ENABLE);
}


/****************************************************************
    CAN_Config
����  ����������CAN�Ĺ���
****************************************************************/
void CAN_Config(void)
{
    CAN_GPIO_Config();
    CAN_NVIC_Config();
    CAN_Mode_Config();
    CAN_Filter_Config();   
}

/****************************************************************
��������ʼ�� Rx Message���ݽṹ��
���룺RxMessage: ָ��Ҫ��ʼ�������ݽṹ��
****************************************************************/
void Init_RxMes(CanRxMsg *RxMessage)
{
    uint8_t ubCounter = 0;

	//�ѽ��սṹ������ 
    RxMessage->StdId = 0x00;
    RxMessage->ExtId = 0x00;
    RxMessage->IDE = CAN_ID_STD;
    RxMessage->DLC = 0;
    RxMessage->FMI = 0;
    for (ubCounter = 0; ubCounter < 8; ubCounter++)
    {
        RxMessage->Data[ubCounter] = 0x00;
    }
}
/****************************************************************
������CANͨ�ű�����������,����һ����������Ϊ0-7�����ݰ�
���룺TxMessage: ���ͱ��Ľṹ��
****************************************************************/
void CAN_SetMsg(CanTxMsg *TxMessage)
{	  
	uint8_t ubCounter = 0;

    //TxMessage.StdId=0x00;						 
    TxMessage->ExtId = 0x1314;				 //ʹ�õ���չID
    TxMessage->IDE = CAN_ID_EXT;			 //��չģʽ
    TxMessage->RTR = CAN_RTR_DATA;			 //���͵�������
    TxMessage->DLC = 8;						 //���ݳ���Ϊ8�ֽ�
	
	//����Ҫ���͵�����0-7 
	for (ubCounter = 0; ubCounter < 8; ubCounter++)
    {
        TxMessage->Data[ubCounter] = ubCounter;
    }
}




 
/********************************************************** 
    CAN1_TxReq
��������������
���룺TxMessage=���͵���Ϣ 
���أ�-1=����ʧ��,0=�ѷ��� 
*********************************************************
static INT16 CAN1_TxReq(CanTxMsg TxMessage)
{
    INT8U TxMailboxNo = 0;
  
    TxMailboxNo = CAN_Transmit(CAN1,&TxMessage);
    if(CAN_NO_MB == TxMailboxNo)
    {//�޿�����,����ʧ��
        return -1;
    }
    else
    {//�ҵ�,ʹ�ܷ����ж�         
        MailboxFlag[TxMailboxNo] = 1;
        CAN_ITConfig(CAN1,CAN_IT_TME, ENABLE);
        return 0;      
    }
}*/
/********************************************************** 
    CAN1_TxFrame
����������1֡���� 
���룺len=���ݳ���
      *pdata=�����͵�����, 
���أ�-1=����ʧ��,0=�ѷ���
**********************************************************/
static INT16 CAN1_TxFrame(INT8U len,INT8U *pdata)
{
    CanTxMsg TxMessage;
    INT8U TxMailboxNo = 0;
    
    //����Э��ͷ����
    TxMessage.StdId = myStdId;      //�趨��׼��ʶ��
    TxMessage.ExtId = 0;            //������չ��ʶ��
    TxMessage.RTR = CAN_RTR_DATA;   //�趨��������Ϣ��֡����=����
    TxMessage.IDE = CAN_ID_STD;     //�趨��Ϣ��ʶ��������=��׼
    TxMessage.DLC = len;            //���ݳ���
    memcpy((char *)TxMessage.Data,(char *)pdata,len);
    //��������
    TxMailboxNo = CAN_Transmit(CAN1,&TxMessage);
    if(CAN_NO_MB == TxMailboxNo)
    {//�޿�����,����ʧ��
        return -1;
    }
    else
    {//�ҵ�,ʹ�ܷ����ж�         
        MailboxFlag[TxMailboxNo] = 1;
        CAN_ITConfig(CAN1,CAN_IT_TME, ENABLE);
        return 0;      
    }
}
/********************************************************** 
    USB_HP_CAN_TX_IRQHandler
������CAN�����жϷ������ 
**********************************************************/
void USB_HP_CAN_TX_IRQHandler(void)
{
    INT8U flag=0;
    if(MailboxFlag[0])
    {//��������0�ж�
        if(CAN_GetITStatus(CAN1,CAN_IT_RQCP0))
        {//������,���־,�ط����ж�
            CAN_ClearITPendingBit(CAN1,CAN_IT_RQCP0);
            CAN_ITConfig(CAN1,CAN_IT_TME, DISABLE); //�ر��ж�
            MailboxFlag[0] = 0;
            flag = 1;
        }
    }
    if(MailboxFlag[1])
    {
        if(CAN_GetITStatus(CAN1,CAN_IT_RQCP1))
        {
            CAN_ClearITPendingBit(CAN1,CAN_IT_RQCP1);
            CAN_ITConfig(CAN1,CAN_IT_TME, DISABLE);
            MailboxFlag[1] = 0;
            flag = 1;
        }
    }  
    if(MailboxFlag[2])
    {
        if(CAN_GetITStatus(CAN1,CAN_IT_RQCP2))
        {
            CAN_ClearITPendingBit(CAN1,CAN_IT_RQCP2);
            CAN_ITConfig(CAN1,CAN_IT_TME, DISABLE);
            MailboxFlag[2] = 0;
            flag = 1;
        }
    }
    if(flag)
    {//�ѷ���1֡
        CANCom.TxCnt += 8;
        if(CANCom.TxCnt<CANCom.TxLen)
        {//1������δ���꣬������1֡
            CANCom.TxStartFlag = 1;
        }
    }
}
/******************************************************* 
    USB_LP_CAN_RX0_IRQHandler
������CAN FIFO0�����жϷ������
*******************************************************/
void USB_LP_CAN_RX0_IRQHandler(void)
{
     CanRxMsg RxMessage;
     if(CAN_GetITStatus(CAN1,CAN_IT_FF0))
     {//FIFO0��
         CAN_ClearITPendingBit(CAN1,CAN_IT_FF0);
     }
     else if(CAN_GetITStatus(CAN1,CAN_IT_FOV0))
     {//FIFO0���
         CAN_ClearITPendingBit(CAN1,CAN_IT_FOV0);
     }
     else
     {//�յ�1�±���
         CAN_Receive(CAN1,CAN_FIFO0,&RxMessage);
         if((RxMessage.StdId==myStdId)&&(RxMessage.IDE==CAN_ID_STD))
         {//ȷ���Ƿ����ҵ�
             memcpy((char *)&CANCom.RxBuf[0][CANCom.RxCnt],(char *)RxMessage.Data,RxMessage.DLC);
             if(CANCom.RxCnt[0]==0)
             {//��1֡����,����������ֽ���(����Э��:�ٶ���2��3�ֽ�Ϊ�����ֽ���)
                 CANCom.RxLen[0] = 4+((INT16U)CANCom.RxBuf[0][3]<<8)|CANCom.RxBuf[0][2];//???
             }
             CANCom.RxCnt[0] += RxMessage.DLC;
             if(CANCom.RxCnt[0]>=CANCom.RxLen[0])
             {//����1������
                 CANCom.RxEndFlag[0] = 1;
             }
         }
     }
}
/******************************************************* 
    CAN_RX1_IRQHandler
������CAN FIFO1�����жϷ������
*******************************************************/
void CAN_RX1_IRQHandler(void)
{
    CanRxMsg RxMessage;
    if(CAN_GetITStatus(CAN1,CAN_IT_FF0))
    {
        CAN_ClearITPendingBit(CAN1,CAN_IT_FF0);
    }
    else if(CAN_GetITStatus(CAN1,CAN_IT_FOV0))
    {
        CAN_ClearITPendingBit(CAN1,CAN_IT_FOV0);
    }
    else
    {//�յ�1�±���
        CAN_Receive(CAN1,CAN_FIFO1,&RxMessage);
        if((RxMessage.StdId==myStdId)&&(RxMessage.IDE==CAN_ID_STD))
        {//ȷ���Ƿ����ҵ�
            if(CANCom.RxCnt[1]==0)
            {//��1֡����,����������ֽ���(����Э��:�ٶ���2��3�ֽ�Ϊ�����ֽ���)
                CANCom.RxLen[1] = 4+((INT16U)CANCom.RxBuf[1][3]<<8)|CANCom.RxBuf[1][2];//???
            }
            memcpy((char *)&CANCom.RxBuf[1][CANCom.RxCnt],(char *)RxMessage.Data,RxMessage.DLC);
            CANCom.RxCnt[1] += RxMessage.DLC;
            if(CANCom.RxCnt[1]>=CANCom.RxLen[1])
            {//����1������
                CANCom.RxEndFlag[1] = 1;
            }
        }
    }
}
/********************************************* 
    CANCom_TxPro
����������1�����ݣ� 
�����Ѵ��뷢�ͻ�������֡����,֡�������Ѹ�ֵ 
δ�ڹ涨��ʱ������ȷ����1֡���ط� 
��������CANCom.TxStartFlag=1ʱ���� 
*********************************************/
void CANCom_TxPro(void)
{
    INT16 status;
    INT8U n;
    if(CANCom.TxCnt>=CANCom.TxLen)
        return;
    if(CANCom.TxCnt+8<=CANCom.TxLen)
    {//��8�ֽ�
        n = 8;
    }
    else
    {//����8�ֽ�    
        n = CANCom.TxLen-CANCom.TxCnt;
    }
    status = CAN1_TxFrame(n,CANCom.TxBuf+CANCom.TxCnt);
    if(status<0)
    {//��������ʧ���ط���ʱ
        CANCom.ReTxIvlCnt = CANCOM_RETXIVLTM;
        CANCom.ReTxIvlFlag = 1;
    }
}
/***************************************** 
    ReTxIvlDly
�������ط������ʱ,��ʱ�жϵ��� 
*****************************************/
void ReTxIvlDly(void)
{
    if(CANCom.ReTxIvlFlag)
    {
        if(--CANCom.ReTxIvlCnt==0)
        {
            CANCom.ReTxIvlFlag = 0;
            CANCom.TxStartFlag = 1; //�����ط�
        }
    }
}
