/************************************************************* 
    bsp_Can.c
描述：Can总线驱动 
使用标准格式(11位标识) 
*************************************************************/
#include "bsp_Can.h"

/************************************************************ 
    CAN_GPIO_Config
描述：Can使用GPIO初始化
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
 * 函数名：CAN_NVIC_Config
 * 描述  ：CAN的NVIC 配置,第1优先级组，0，0优先级
 * 输入  ：无
 * 输出  : 无
 * 调用  ：内部调用
 */
/****************************************************
    CAN接收中断 配置嵌套向量中断控制器NVIC (0-7) 
****************************************************/
static void CAN_NVIC_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    // 嵌套向量中断控制器组选择  
    //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
    // 配置USART为中断源  
    NVIC_InitStructure.NVIC_IRQChannel = CAN_RX_IRQ;
    // 抢断优先级为  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    // 子优先级为  
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 7;
    // 使能中断   
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    // 初始化配置NVIC  
    NVIC_Init(&NVIC_InitStructure);
}
/****************************************************
    CAN工作模式初始化
****************************************************/
static void CAN_Mode_Config(void)
{
	CAN_InitTypeDef        CAN_InitStructure;
	// CAN通信参数设置 
	// Enable CAN clock 
    RCC_APB1PeriphClockCmd(CAN_CLK, ENABLE);

	//CAN寄存器初始化 
	CAN_DeInit(CANx);
	CAN_StructInit(&CAN_InitStructure);

	//CAN单元初始化 
	CAN_InitStructure.CAN_TTCM = DISABLE;			// 禁止时间触发通信模式
	CAN_InitStructure.CAN_ABOM = ENABLE;			// 自动离线管理 
	CAN_InitStructure.CAN_AWUM = ENABLE;			// 使用自动唤醒模式
	CAN_InitStructure.CAN_NART = DISABLE;			// 使用报文自动重传 
	CAN_InitStructure.CAN_RFLM = ENABLE;			// 接收FIFO 锁定模式(溢出时新报文不会覆盖原有报文)
	CAN_InitStructure.CAN_TXFP = ENABLE;			// 按顺序发送FIFO(不按标识符的优先级) 
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;   // 正常工作模式
	CAN_InitStructure.CAN_SJW = CAN_SJW_2tq;		// 重新同步跳跃宽度 2个时间单元
	 
	/* ss=1 bs1=4 bs2=2 位时间宽度为(1+4+2) 波特率即为时钟周期tq*(1+4+2)  */
	CAN_InitStructure.CAN_BS1 = CAN_BS1_4tq;		//BTR-TS1 时间段1 占用了4个时间单元
	CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;		//BTR-TS1 时间段2 占用了2个时间单元	
	
	/* CAN Baudrate = 1 MBps  (CAN 时钟频率为 APB1 = 42 MHz) */
	CAN_InitStructure.CAN_Prescaler = 6;		    // 波特率分频器  定义了时间单元的时间长度 42/(1+4+2)/6=1 Mbps
	CAN_Init(CANx, &CAN_InitStructure);
}

/***************************************************
    CAN的过滤器配置初始化
****************************************************/
static void CAN_Filter_Config(void)
{
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;

	//CAN筛选器初始化 
	CAN_FilterInitStructure.CAN_FilterNumber = 0;					//筛选器组=0第1组
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdList;	//工作在列表模式
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_16bit;//筛选器位宽为两个16位。
	//使能筛选器，按照标志的内容进行比对筛选，扩展ID不是如下的就抛弃掉，是的话，会存入FIFO0。
    //列表中仅1个ID，放在第1个位置，其他位置全写1 
    //ID[10:0]|RTR|IDE|EXID[17:15] 
    CAN_FilterInitStructure.CAN_FilterIdLow = ((u16)DevBStdID<<5)|0x0007;
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0xFFFF;		 
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0xFFFF;			 
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0xFFFF;			 

    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0 ;//筛选器被关联到FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;			//使能筛选器
	CAN_FilterInit(&CAN_FilterInitStructure);
	//CAN通信中断使能 
	CAN_ITConfig(CANx, CAN_IT_FMP0, ENABLE);
}


/****************************************************************
    CAN_Config
描述  ：完整配置CAN的功能
****************************************************************/
void CAN_Config(void)
{
    CAN_GPIO_Config();
    CAN_NVIC_Config();
    CAN_Mode_Config();
    CAN_Filter_Config();   
}

/****************************************************************
描述：初始化 Rx Message数据结构体
输入：RxMessage: 指向要初始化的数据结构体
****************************************************************/
void Init_RxMes(CanRxMsg *RxMessage)
{
    uint8_t ubCounter = 0;

	//把接收结构体清零 
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
描述：CAN通信报文内容设置,设置一个数据内容为0-7的数据包
输入：TxMessage: 发送报文结构体
****************************************************************/
void CAN_SetMsg(CanTxMsg *TxMessage)
{	  
	uint8_t ubCounter = 0;

    //TxMessage.StdId=0x00;						 
    TxMessage->ExtId = 0x1314;				 //使用的扩展ID
    TxMessage->IDE = CAN_ID_EXT;			 //扩展模式
    TxMessage->RTR = CAN_RTR_DATA;			 //发送的是数据
    TxMessage->DLC = 8;						 //数据长度为8字节
	
	//设置要发送的数据0-7 
	for (ubCounter = 0; ubCounter < 8; ubCounter++)
    {
        TxMessage->Data[ubCounter] = ubCounter;
    }
}




 
/********************************************************** 
    CAN1_TxReq
描述：发送请求
输入：TxMessage=发送的消息 
返回：-1=发送失败,0=已发送 
*********************************************************
static INT16 CAN1_TxReq(CanTxMsg TxMessage)
{
    INT8U TxMailboxNo = 0;
  
    TxMailboxNo = CAN_Transmit(CAN1,&TxMessage);
    if(CAN_NO_MB == TxMailboxNo)
    {//无空邮箱,发送失败
        return -1;
    }
    else
    {//找到,使能发送中断         
        MailboxFlag[TxMailboxNo] = 1;
        CAN_ITConfig(CAN1,CAN_IT_TME, ENABLE);
        return 0;      
    }
}*/
/********************************************************** 
    CAN1_TxFrame
描述：发送1帧数据 
输入：len=数据长度
      *pdata=待发送的数据, 
返回：-1=发送失败,0=已发送
**********************************************************/
static INT16 CAN1_TxFrame(INT8U len,INT8U *pdata)
{
    CanTxMsg TxMessage;
    INT8U TxMailboxNo = 0;
    
    //配置协议头部分
    TxMessage.StdId = myStdId;      //设定标准标识符
    TxMessage.ExtId = 0;            //设置扩展标识符
    TxMessage.RTR = CAN_RTR_DATA;   //设定待传输消息的帧类型=数据
    TxMessage.IDE = CAN_ID_STD;     //设定消息标识符的类型=标准
    TxMessage.DLC = len;            //数据长度
    memcpy((char *)TxMessage.Data,(char *)pdata,len);
    //发送请求
    TxMailboxNo = CAN_Transmit(CAN1,&TxMessage);
    if(CAN_NO_MB == TxMailboxNo)
    {//无空邮箱,发送失败
        return -1;
    }
    else
    {//找到,使能发送中断         
        MailboxFlag[TxMailboxNo] = 1;
        CAN_ITConfig(CAN1,CAN_IT_TME, ENABLE);
        return 0;      
    }
}
/********************************************************** 
    USB_HP_CAN_TX_IRQHandler
描述：CAN发送中断服务程序 
**********************************************************/
void USB_HP_CAN_TX_IRQHandler(void)
{
    INT8U flag=0;
    if(MailboxFlag[0])
    {//发送邮箱0中断
        if(CAN_GetITStatus(CAN1,CAN_IT_RQCP0))
        {//发送完,清标志,关发送中断
            CAN_ClearITPendingBit(CAN1,CAN_IT_RQCP0);
            CAN_ITConfig(CAN1,CAN_IT_TME, DISABLE); //关闭中断
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
    {//已发出1帧
        CANCom.TxCnt += 8;
        if(CANCom.TxCnt<CANCom.TxLen)
        {//1包数据未发完，继续下1帧
            CANCom.TxStartFlag = 1;
        }
    }
}
/******************************************************* 
    USB_LP_CAN_RX0_IRQHandler
描述：CAN FIFO0接收中断服务程序
*******************************************************/
void USB_LP_CAN_RX0_IRQHandler(void)
{
     CanRxMsg RxMessage;
     if(CAN_GetITStatus(CAN1,CAN_IT_FF0))
     {//FIFO0满
         CAN_ClearITPendingBit(CAN1,CAN_IT_FF0);
     }
     else if(CAN_GetITStatus(CAN1,CAN_IT_FOV0))
     {//FIFO0溢出
         CAN_ClearITPendingBit(CAN1,CAN_IT_FOV0);
     }
     else
     {//收到1新报文
         CAN_Receive(CAN1,CAN_FIFO0,&RxMessage);
         if((RxMessage.StdId==myStdId)&&(RxMessage.IDE==CAN_ID_STD))
         {//确认是发给我的
             memcpy((char *)&CANCom.RxBuf[0][CANCom.RxCnt],(char *)RxMessage.Data,RxMessage.DLC);
             if(CANCom.RxCnt[0]==0)
             {//第1帧数据,获得数据总字节数(依据协议:假定第2、3字节为后续字节数)
                 CANCom.RxLen[0] = 4+((INT16U)CANCom.RxBuf[0][3]<<8)|CANCom.RxBuf[0][2];//???
             }
             CANCom.RxCnt[0] += RxMessage.DLC;
             if(CANCom.RxCnt[0]>=CANCom.RxLen[0])
             {//收满1包数据
                 CANCom.RxEndFlag[0] = 1;
             }
         }
     }
}
/******************************************************* 
    CAN_RX1_IRQHandler
描述：CAN FIFO1接收中断服务程序
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
    {//收到1新报文
        CAN_Receive(CAN1,CAN_FIFO1,&RxMessage);
        if((RxMessage.StdId==myStdId)&&(RxMessage.IDE==CAN_ID_STD))
        {//确认是发给我的
            if(CANCom.RxCnt[1]==0)
            {//第1帧数据,获得数据总字节数(依据协议:假定第2、3字节为后续字节数)
                CANCom.RxLen[1] = 4+((INT16U)CANCom.RxBuf[1][3]<<8)|CANCom.RxBuf[1][2];//???
            }
            memcpy((char *)&CANCom.RxBuf[1][CANCom.RxCnt],(char *)RxMessage.Data,RxMessage.DLC);
            CANCom.RxCnt[1] += RxMessage.DLC;
            if(CANCom.RxCnt[1]>=CANCom.RxLen[1])
            {//收满1包数据
                CANCom.RxEndFlag[1] = 1;
            }
        }
    }
}
/********************************************* 
    CANCom_TxPro
描述：发送1包数据， 
数据已存入发送缓冲区，帧数量,帧计数器已赋值 
未在规定的时间内正确发出1帧则重发 
主程序在CANCom.TxStartFlag=1时调用 
*********************************************/
void CANCom_TxPro(void)
{
    INT16 status;
    INT8U n;
    if(CANCom.TxCnt>=CANCom.TxLen)
        return;
    if(CANCom.TxCnt+8<=CANCom.TxLen)
    {//够8字节
        n = 8;
    }
    else
    {//不足8字节    
        n = CANCom.TxLen-CANCom.TxCnt;
    }
    status = CAN1_TxFrame(n,CANCom.TxBuf+CANCom.TxCnt);
    if(status<0)
    {//启动发送失败重发定时
        CANCom.ReTxIvlCnt = CANCOM_RETXIVLTM;
        CANCom.ReTxIvlFlag = 1;
    }
}
/***************************************** 
    ReTxIvlDly
描述：重发间隔延时,定时中断调用 
*****************************************/
void ReTxIvlDly(void)
{
    if(CANCom.ReTxIvlFlag)
    {
        if(--CANCom.ReTxIvlCnt==0)
        {
            CANCom.ReTxIvlFlag = 0;
            CANCom.TxStartFlag = 1; //启动重发
        }
    }
}
