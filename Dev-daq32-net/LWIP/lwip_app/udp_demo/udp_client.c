#include "udp_demo.h" 
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "key.h"
#include "lcd.h"
#include "malloc.h"
#include "stdio.h"
#include "string.h" 
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//UDP 测试代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/8/15
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
//*******************************************************************************
//修改信息
//无
////////////////////////////////////////////////////////////////////////////////// 	   
 
//UDP接收数据缓冲区
u8 udp_demo_recvbuf[UDP_DEMO_RX_BUFSIZE];	//UDP接收数据缓冲区 
//UDP发送数据内容
const u8 *tcp_demo_sendbuf="Explorer STM32F407 UDP demo send data\r\n";

//UDP 测试全局状态标记变量
//bit7:没有用到
//bit6:0,没有收到数据;1,收到数据了.
//bit5:0,没有连接上;1,连接上了.
//bit4~0:保留
u8 udp_demo_flag;

/* UDP客户端初始化配置 */
void UDP_Client_Initialization(void)
{
	ip_addr_t DestIPaddr;
	err_t err;
	struct udp_pcb *upcb;
	char data[] = "This is a Client.";

/* 设置服务器端的IP地址 */
	IP4_ADDR(&DestIPaddr, udpServerIP[0], udpServerIP[1], udpServerIP[2], udpServerIP[3]);

/* 创建一个新的UDP控制块 */
	upcb = udp_new();

	if (upcb != NULL)
	{
		/* 服务器端地址、端口配置 */
		err = udp_connect(upcb, &DestIPaddr, UDP_ECHO_SERVER_PORT);

		if (err == ERR_OK)
		{
			/* 注册回调函数 */
			udp_recv(upcb, UDPClientCallback, NULL);
			/**数据发送，第一次连接时客户端发送数据至服务器端，发送函数中会遍历查找源IP地址的配置，如果源IP地址未配置，则数据发送失败。该处出现的问题在后面总结中提到了**/
			UdpClientSendPacket(upcb, data);
		}
	}
}

/* 定义UDP客户端数据处理回调函数 */
static void UDPClientCallback(void *arg,struct udp_pcb *upcb,struct pbuf *p,const ip_addr_t *addr,u16_t port)
{
  udp_send(upcb, p);     //数据回显
 
  pbuf_free(p);
}
 
/* 客户端数据发送函数 */
void UdpClientSendPacket(struct udp_pcb *upcb,char* data)
{
  struct pbuf *p;
 
  /* 分配内存空间 */
  p = pbuf_alloc(PBUF_TRANSPORT,strlen((char*)data), PBUF_POOL);
 
  if (p != NULL)
  {
 
    /* 复制数据到pbuf */
    pbuf_take(p, (char*)data, strlen((char*)data));
 
    /* 发送数据 */
    udp_send(upcb, p);     //发送数据
 
    /* 释放pbuf */
    pbuf_free(p);
  }
}




















