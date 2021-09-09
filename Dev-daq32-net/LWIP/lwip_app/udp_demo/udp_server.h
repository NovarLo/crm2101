#ifndef __UDP_SERVER_H
#define __UDP_SERVER_H
#include "sys.h"
#include "lwip_comm.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "lwip/tcp.h"
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
 
#define UDP_SERVER_RX_BUFSIZE	20		//定义udp最大接收数据长度 
#define UDP_SERVER_TX_BUFSIZE   640     //定义udp最大发送数据长度 

#define UDP_SERVER_PORT			5050	//定义udp连接的端口 
#define UDP_RDID_PORT			5051	//定义udp读取ID端口

extern  u8 udp_server_flag;
//声明接收数据回调函数，在初始化函数中指定
void udp_echoserver_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, ip_addr_t *addr, u16_t port);
void udp_echoserver_receive_callback2(void *arg, struct udp_pcb *upcb, struct pbuf *p, ip_addr_t *addr, u16_t port);
void udp_echoserver_init(void);
void ContinueSendTo(void);
#endif

