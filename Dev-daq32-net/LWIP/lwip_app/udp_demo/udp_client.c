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
 
//UDP�������ݻ�����
u8 udp_demo_recvbuf[UDP_DEMO_RX_BUFSIZE];	//UDP�������ݻ����� 
//UDP������������
const u8 *tcp_demo_sendbuf="Explorer STM32F407 UDP demo send data\r\n";

//UDP ����ȫ��״̬��Ǳ���
//bit7:û���õ�
//bit6:0,û���յ�����;1,�յ�������.
//bit5:0,û��������;1,��������.
//bit4~0:����
u8 udp_demo_flag;

/* UDP�ͻ��˳�ʼ������ */
void UDP_Client_Initialization(void)
{
	ip_addr_t DestIPaddr;
	err_t err;
	struct udp_pcb *upcb;
	char data[] = "This is a Client.";

/* ���÷������˵�IP��ַ */
	IP4_ADDR(&DestIPaddr, udpServerIP[0], udpServerIP[1], udpServerIP[2], udpServerIP[3]);

/* ����һ���µ�UDP���ƿ� */
	upcb = udp_new();

	if (upcb != NULL)
	{
		/* �������˵�ַ���˿����� */
		err = udp_connect(upcb, &DestIPaddr, UDP_ECHO_SERVER_PORT);

		if (err == ERR_OK)
		{
			/* ע��ص����� */
			udp_recv(upcb, UDPClientCallback, NULL);
			/**���ݷ��ͣ���һ������ʱ�ͻ��˷����������������ˣ����ͺ����л��������ԴIP��ַ�����ã����ԴIP��ַδ���ã������ݷ���ʧ�ܡ��ô����ֵ������ں����ܽ����ᵽ��**/
			UdpClientSendPacket(upcb, data);
		}
	}
}

/* ����UDP�ͻ������ݴ���ص����� */
static void UDPClientCallback(void *arg,struct udp_pcb *upcb,struct pbuf *p,const ip_addr_t *addr,u16_t port)
{
  udp_send(upcb, p);     //���ݻ���
 
  pbuf_free(p);
}
 
/* �ͻ������ݷ��ͺ��� */
void UdpClientSendPacket(struct udp_pcb *upcb,char* data)
{
  struct pbuf *p;
 
  /* �����ڴ�ռ� */
  p = pbuf_alloc(PBUF_TRANSPORT,strlen((char*)data), PBUF_POOL);
 
  if (p != NULL)
  {
 
    /* �������ݵ�pbuf */
    pbuf_take(p, (char*)data, strlen((char*)data));
 
    /* �������� */
    udp_send(upcb, p);     //��������
 
    /* �ͷ�pbuf */
    pbuf_free(p);
  }
}




















