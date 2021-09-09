#include "udp_server.h"
#include "bsp_timer.h"
#include "usart.h"
#include "malloc.h"
#include "main.h"
#include "bsp_ad7606.h"
#include "bsp_io.h"
//Ӧ������
const uint8_t ConnectAnsTab[] = { 0xaa, 0x14, 0x00, 0x00, 0x00, 0x09 };
const uint8_t SingleAnsTab[] = { 0xaa, 0xa8, 0x00, 0x00, 0x00, 0x01 };
const uint8_t MultAnsTab[] = { 0xaa, 0x86, 0x01, 0x01, 0x00, 0x02 };
const uint8_t ContinueAnsTab[] = { 0xaa, 0xff, 0xff, 0xff, 0xff, 0x03 };

//UDP���ջ�����
uint8_t udp_server_recvbuf[UDP_SERVER_RX_BUFSIZE];
// UDP���ͻ�����
uint8_t udp_server_sendbuf[UDP_SERVER_TX_BUFSIZE];
//��ǰ���ڷ��͵�ͨѶ����
struct udp_pcb  Now_upcb;
//struct pbuf     *Now_p;
ip_addr_t       Now_addr;
u16_t           Now_port;

//UDP ����ȫ��״̬��Ǳ���
//bit7:û���õ�
//bit6:0,�˿�5050û���յ�����;1,�յ�������.
//bit5:0,�˿�5050û��������;1,��������.
//bit4:����
//bit3:0,�˿�5051û���յ�����;1,�յ�������.
//bit2:0,�˿�5051û��������;1,��������.

//bit0=1:���跢��
//bit1=1:��η���
u8 udp_server_flag;

//UDP�������˳�ʼ������
void udp_echoserver_init(void)
{
	struct udp_pcb *upcb,*upcb2;
	err_t err;

	/* Create a new UDP control block  */
	upcb = udp_new();  //����һ���µ�UDP���ƿ�

	if (upcb)
	{
		/* Bind the upcb to the UDP_PORT port */
		/* Using IP_ADDR_ANY allow the upcb to be used by any local interface */
		err = udp_bind(upcb, IP_ADDR_ANY, UDP_SERVER_PORT);   //�󶨱���IP��ַ���˿�

		if (err == ERR_OK)
		{
			/* Set a receive callback for the upcb */
			udp_recv(upcb, udp_echoserver_receive_callback, NULL);   //ע��������ݻص�����
			udp_server_flag |= 1 << 5;            //����Ѿ�������
		}
		else
		{
			udp_remove(upcb);
		}
	}

	/* Create a new UDP control block  */
	upcb2 = udp_new();  //����һ���µ�UDP���ƿ�

	if (upcb2)
	{
		/* Bind the upcb to the UDP_PORT port */
		/* Using IP_ADDR_ANY allow the upcb to be used by any local interface */
		err = udp_bind(upcb2, IP_ADDR_ANY, UDP_RDID_PORT);   //�󶨱���IP��ַ���˿�

		if (err == ERR_OK)
		{
			/* Set a receive callback for the upcb */
			udp_recv(upcb2, udp_echoserver_receive_callback2, NULL);   //ע��������ݻص�����
			udp_server_flag |= 1 << 2;            //����Ѿ�������
		}
		else
		{
			udp_remove(upcb2);
		}
	}

}
//�ݴ浱ǰͨѶ����
static void saveComm(struct udp_pcb *upcb, ip_addr_t *addr, u16_t port)
{
	memcpy(&Now_upcb, upcb, sizeof(struct udp_pcb));
	memcpy(&Now_addr, addr, sizeof(ip_addr_t));
	Now_port = port;
}


//������ 5051�˽������ݻص�����
void udp_echoserver_receive_callback2(void *arg, struct udp_pcb *upcb, struct pbuf *p, ip_addr_t *addr, u16_t port)
{
	// Tell the client that we have accepted it
	//udp_sendto(upcb, p, addr, port);  //��������
	// Free the p buffer
	//pbuf_free(p);
	uint32_t len;
	struct pbuf *pudp_buf;
	//���մ���
	if (p->len > UDP_SERVER_RX_BUFSIZE) len = UDP_SERVER_RX_BUFSIZE;
	else len = p->len;
	memcpy(udp_server_recvbuf, p->payload, len);
	//�������
	if (udp_server_recvbuf[0] != 0x55)
	{
		pbuf_free(p);
		return;
	}
	LED5_ON();
	if (udp_server_recvbuf[5] == 0x09)
	{
		pbuf_free(p);
		pudp_buf = pbuf_alloc(PBUF_TRANSPORT, 20, PBUF_RAM);
		if (pudp_buf)
		{
			memcpy(udp_server_sendbuf, ConnectAnsTab, 6);
			memcpy(udp_server_sendbuf + 6, CPUIDbuf, 12);
			udp_server_sendbuf[18] = 0x00;
			udp_server_sendbuf[19] = 0x00;
			memcpy(pudp_buf->payload, udp_server_sendbuf, 20);
			udp_sendto(upcb, pudp_buf, addr, port);
			pbuf_free(pudp_buf);
		}
	}
	LED5_OFF();
}

//������ 5050�˽������ݻص�����
void udp_echoserver_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, ip_addr_t *addr, u16_t port)
{
	uint32_t len;
	struct pbuf *pudp_buf;
	//���մ���
	if (p->len > UDP_SERVER_RX_BUFSIZE) len = UDP_SERVER_RX_BUFSIZE;
	else len = p->len;
	memcpy(udp_server_recvbuf, p->payload, len);
	//�������
	if (udp_server_recvbuf[0] != 0x55)
	{
		pbuf_free(p);
		return;
	}
	LED5_ON();
	switch (udp_server_recvbuf[5])
	{
	case 0x0c:  //��λ
		AD_SoftReset(); //��λAD����ز���
		pbuf_free(p);
		break;
	case 0x0a:  //����
		//��ȡ���ò���������
		memcpy(&WorkInfo.NetSet, &udp_server_recvbuf[6], 12);
		SetWorkInfo();
		//Ӧ��
		pudp_buf = pbuf_alloc(PBUF_TRANSPORT, 20, PBUF_RAM);
		if (pudp_buf)
		{
			memcpy(udp_server_sendbuf, udp_server_recvbuf, 20);
			pbuf_free(p);
			udp_server_sendbuf[0] = 0xaa;
			memcpy(pudp_buf->payload, udp_server_sendbuf, 20);
			udp_sendto(upcb, pudp_buf, addr, port);
			pbuf_free(pudp_buf);
		}
		/*�����ʼ��
		while (lwip_comm_init()) //lwip��ʼ��
		{
			// ʧ��
			BELL_ON();
			delay_ms(1200);
		}
		//OK
		BELL_OFF();
		delay_ms(1000);
		//UDP����˳�ʼ��
		udp_echoserver_init();*/
		break;
	case 0x09:  //����
		pbuf_free(p);
		pudp_buf = pbuf_alloc(PBUF_TRANSPORT, 20, PBUF_RAM);
		if (pudp_buf)
		{
			memcpy(udp_server_sendbuf, ConnectAnsTab, 6);
			memcpy(udp_server_sendbuf + 6, CPUIDbuf, 12);
			udp_server_sendbuf[18] = 0x00;
			udp_server_sendbuf[19] = 0x00;
			memcpy(pudp_buf->payload, udp_server_sendbuf, 20);
			udp_sendto(upcb, pudp_buf, addr, port);
			pbuf_free(pudp_buf);
		}
		break;
	case 0x01:  //���β���
		if ((WorkInfo.ADParam.AD_os != udp_server_recvbuf[6]) || (WorkInfo.ADParam.AD_gain != udp_server_recvbuf[7]))
		{ //ADC�������͸ı�
			AD7606_Stop();
			WorkInfo.ADParam.AD_os = udp_server_recvbuf[6];
			WorkInfo.ADParam.AD_gain = udp_server_recvbuf[7];
			SetWorkInfo();
			//ADC��ʼ��
			AD_Array_Init();
		}
		pbuf_free(p);
		pudp_buf = pbuf_alloc(PBUF_TRANSPORT, 6, PBUF_RAM);
		if (pudp_buf)
		{
			memcpy(udp_server_sendbuf, SingleAnsTab, 6);
			memcpy(pudp_buf->payload, udp_server_sendbuf, 6);
			udp_sendto(upcb, pudp_buf, addr, port);
			pbuf_free(pudp_buf);
		}		
		if (AD_Run == 0)
		{
			AD7606_StartConvst(0);
		}
		//�ȴ�32��ͨ�����1��ת��
		//while(ADArrayCtr.NewframeOK==0);
		//Ӧ��
		pudp_buf = pbuf_alloc(PBUF_TRANSPORT, 640, PBUF_RAM);
		if (pudp_buf)
		{
			memcpy(udp_server_sendbuf, &NewFrame, sizeof(AD_ARRAY));
			memcpy(pudp_buf->payload, udp_server_sendbuf, 640);
			udp_sendto(upcb, pudp_buf, addr, port);
			pbuf_free(pudp_buf);
		}
		break;
	case 0x02:  //��β���
		// ��Ӧ��
		pbuf_free(p);
		pudp_buf = pbuf_alloc(PBUF_TRANSPORT, 6, PBUF_RAM);
		if (pudp_buf)
		{
			memcpy(udp_server_sendbuf,MultAnsTab, 6);
			memcpy(pudp_buf->payload, udp_server_sendbuf, 6);
			udp_sendto(upcb, pudp_buf, addr, port);
			pbuf_free(pudp_buf);
		}

		len = byte2int((uint8_t *)&udp_server_recvbuf[12]);
		WorkInfo.ADParam.AD_num = len / 320;
		len = byte2int((uint8_t *)&udp_server_recvbuf[8]);
		if ((WorkInfo.ADParam.AD_os != udp_server_recvbuf[6]) ||
			(WorkInfo.ADParam.AD_gain != udp_server_recvbuf[7]) ||
			(len != WorkInfo.ADParam.AD_freq))
		{ //ADC�������͸ı�
			uint8_t flag = 0;
			AD7606_Stop();
			WorkInfo.ADParam.AD_os = udp_server_recvbuf[6];
			WorkInfo.ADParam.AD_gain = udp_server_recvbuf[7];
			if (len != WorkInfo.ADParam.AD_freq)
			{ //Ƶ�ʸı�
				WorkInfo.ADParam.AD_freq = len;
				flag = 1;
			}
			SetWorkInfo();
			//��λ����
			AD_Array_Init();
			//����ADC
			AD7606_StartConvst(flag);
		}
		else if (AD_Run == 0)
		{
			AD7606_StartConvst(0);
		}
		/*
		pbuf_free(p);
		//Ӧ��
		pudp_buf = pbuf_alloc(PBUF_TRANSPORT, 6, PBUF_RAM);
		if (pudp_buf)
		{
			memcpy(pudp_buf->payload, MultAnsTab, 6);
			udp_sendto(upcb, pudp_buf, addr, port);
			pbuf_free(pudp_buf);
			//�ݴ�(upcb, addr, port)
			saveComm(upcb, addr, port);
			//��η��ͱ�־
			udp_server_flag |= 0x02;
		}*/
		saveComm(upcb, addr, port);
		udp_server_flag |= 0x02;
		break;
	case 0x03:  //����

		// ��Ӧ��
		pbuf_free(p);
		pudp_buf = pbuf_alloc(PBUF_TRANSPORT, 6, PBUF_RAM);
		if (pudp_buf)
		{
			memcpy(udp_server_sendbuf,ContinueAnsTab, 6);
			memcpy(pudp_buf->payload, udp_server_sendbuf, 6);
			udp_sendto(upcb, pudp_buf, addr, port);
			pbuf_free(pudp_buf);
		}

		len = byte2int((uint8_t *)&udp_server_recvbuf[8]);
		if ((WorkInfo.ADParam.AD_os != udp_server_recvbuf[6]) || (WorkInfo.ADParam.AD_gain != udp_server_recvbuf[7]) || (len != WorkInfo.ADParam.AD_freq))
		{ //ADC�������͸ı�
			uint8_t flag = 0;
			AD7606_Stop();
			WorkInfo.ADParam.AD_os = udp_server_recvbuf[6];
			WorkInfo.ADParam.AD_gain = udp_server_recvbuf[7];
			if (len != WorkInfo.ADParam.AD_freq)
			{
				WorkInfo.ADParam.AD_freq = len;
				flag = 1;
			}
			SetWorkInfo();
			//��λ����
			AD_Array_Init();
			//����ADC
			AD7606_StartConvst(flag);
		}
		else if (AD_Run == 0)
		{
			AD7606_StartConvst(0);
		}
		/*
		pbuf_free(p);
		//Ӧ��
		pudp_buf = pbuf_alloc(PBUF_TRANSPORT, 6, PBUF_RAM);
		if (pudp_buf)
		{
			memcpy(pudp_buf->payload, ContinueAnsTab, 6);
			udp_sendto(upcb, pudp_buf, addr, port);
			pbuf_free(pudp_buf);
			//�ݴ�(upcb, pudp_buf, addr, port)
			saveComm(upcb, addr, port);
			//�������ͱ�־
			udp_server_flag |= 0x01;
		}*/
		//�ݴ�(upcb, pudp_buf, addr, port)
		saveComm(upcb, addr, port);
		//�������ͱ�־
		udp_server_flag |= 0x01;
		break;
	case 0x04:  //ֹͣ
		udp_server_flag &= 0xfc;
		AD7606_Stop();
		pbuf_free(p);
		break;
	default:
		pbuf_free(p);
		break;
	}
	LED5_OFF();
}

/*********************************************** 
	ContinueSendTo
��������֡����,����������� 
************************************************/
void ContinueSendTo(void)
{
	struct pbuf *pudp_buf;
	if (udp_server_flag & 0x02)
	{
		if (WorkInfo.ADParam.AD_num == 0)
		{ //��η��ʹ�����
			return;
		}
	}
	//��������
	if ((uint32_t)ADArrayCtr.Rpoint != (uint32_t)ADArrayCtr.Wpoint)
	{ //���в���
		pudp_buf = pbuf_alloc(PBUF_TRANSPORT, sizeof(AD_ARRAY), PBUF_RAM);
		if (pudp_buf)
		{
			memcpy(pudp_buf->payload, ADArrayCtr.Rpoint, sizeof(AD_ARRAY));
			udp_sendto(&Now_upcb, pudp_buf, &Now_addr, Now_port);
			pbuf_free(pudp_buf);
		}
		//����+1
		ADArrayCtr.Rpoint++;
		if (ADArrayCtr.Rpoint >= Array + AD_ARRAYNum) ADArrayCtr.Rpoint = Array;
		if (udp_server_flag & 0x02)
		{
			WorkInfo.ADParam.AD_num--;
		}
	}
}

