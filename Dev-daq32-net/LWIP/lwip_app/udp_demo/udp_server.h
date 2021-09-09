#ifndef __UDP_SERVER_H
#define __UDP_SERVER_H
#include "sys.h"
#include "lwip_comm.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "lwip/tcp.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//UDP ���Դ���	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/8/15
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
//*******************************************************************************
//�޸���Ϣ
//��
////////////////////////////////////////////////////////////////////////////////// 	   
 
#define UDP_SERVER_RX_BUFSIZE	20		//����udp���������ݳ��� 
#define UDP_SERVER_TX_BUFSIZE   640     //����udp��������ݳ��� 

#define UDP_SERVER_PORT			5050	//����udp���ӵĶ˿� 
#define UDP_RDID_PORT			5051	//����udp��ȡID�˿�

extern  u8 udp_server_flag;
//�����������ݻص��������ڳ�ʼ��������ָ��
void udp_echoserver_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, ip_addr_t *addr, u16_t port);
void udp_echoserver_receive_callback2(void *arg, struct udp_pcb *upcb, struct pbuf *p, ip_addr_t *addr, u16_t port);
void udp_echoserver_init(void);
void ContinueSendTo(void);
#endif

