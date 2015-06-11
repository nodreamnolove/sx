/********************************************************************************
    ���������5�����֣�
    	1. W5100��ʼ��
    	2. W5100��Socket��ʼ��
    	3. Socket����
    	   ���Socket����ΪTCP������ģʽ�������Socket_Listen()����,W5100��������״̬��ֱ��Զ�̿ͻ����������ӡ�
    	   ���Socket����ΪTCP�ͻ���ģʽ�������Socket_Connect()������
    	                                  ÿ����һ��Socket_Connect(s)����������һ�����ӣ�
    	                                  ������Ӳ��ɹ����������ʱ�жϣ�Ȼ������ٵ��øú����������ӡ�
    	   ���Socket����ΪUDPģʽ,�����Socket_UDP����
    	4. Socket���ݽ��պͷ���
    	5. W5100�жϴ���

    ��W5100Ϊ������ģʽ�ĵ��ù��̣�W5100_Init()-->Socket_Init(s)-->Socket_Listen(s)�����ù��̼���ɣ��ȴ��ͻ��˵����ӡ�
    ��W5100Ϊ�ͻ���ģʽ�ĵ��ù��̣�W5100_Init()-->Socket_Init(s)-->Socket_Connect(s)�����ù��̼���ɣ�����Զ�̷��������ӡ�
    ��W5100ΪUDPģʽ�ĵ��ù��̣�W5100_Init()-->Socket_Init(s)-->Socket_UDP(s)�����ù��̼���ɣ�������Զ������UDPͨ�š�

    W5100���������ӳɹ�����ֹ���ӡ��������ݡ��������ݡ���ʱ���¼��������Դ��ж�״̬�л�á�
********************************************************************************/
#define	__W5100_C
#include "W5100.h"
#include "WT_Task.h"
uint32	DARec;

uint32	u32Interrupt_Num = 0;         //��¼W5100�жϣ��������жϴ���	

// ��ʱ1ms
void W5100_Delay_1MS(uint32 m)
{
	uint32 i;

	for(; m != 0; m--)	
 		for (i=0; i<8300; i++);
}

/*****************************************************************
��������Read_W5100
����: ��ַ
���: ��
����: ��ȡ������
˵������W5100ָ���ĵ�ַ��ȡһ���ֽ�
*****************************************************************/
uint8 Read_W5100(uint32 addr)
{
	unsigned char i;

	volatile unsigned char *ip;
	
	ip = (volatile unsigned char *)addr;
	i = *ip;
	
	return i;
}

/*****************************************************************
��������Write_W5100
����: ��ַ���ֽ�����
���: ��
����: ��
˵������һ���ֽ�д��W5100ָ���ĵ�ַ
*****************************************************************/
void Write_W5100(uint32 addr, uint8 dat)
{
	volatile unsigned char *ip;

	ip = (volatile unsigned char *)addr;
	*ip = dat;
}

/*------------------------------------------------------------------------------
						W5100��ʼ������
��ʹ��W5100֮ǰ����W5100��ʼ��
------------------------------------------------------------------------------*/
void W5100_Init(void)
{
	unsigned char i;

	Write_W5100(W5100_MODE,MODE_RST);		/*��λW5100*/

	W5100_Delay_1MS(100);						/*��ʱ100ms���Լ�����ú���*/

	/*��������(Gateway)��IP��ַ��4�ֽ� */
	/*ʹ�����ؿ���ʹͨ��ͻ�������ľ��ޣ�ͨ�����ؿ��Է��ʵ��������������Internet*/
	for(i=0;i<4;i++)
		Write_W5100(W5100_GAR+i,Gateway_IP[i]);			/*Gateway_IPΪ4�ֽ�unsigned char����,�Լ�����*/

	/*������������(MASK)ֵ��4�ֽڡ���������������������*/
	for(i=0;i<4;i++)
		Write_W5100(W5100_SUBR+i,Sub_Mask[i]);			/*SUB_MASKΪ4�ֽ�unsigned char����,�Լ�����*/

	/*���������ַ��6�ֽڣ�����Ψһ��ʶ�����豸�������ֵַ
	�õ�ֵַ��Ҫ��IEEE���룬����OUI�Ĺ涨��ǰ3���ֽ�Ϊ���̴��룬�������ֽ�Ϊ��Ʒ���
	����Լ����������ַ��ע���һ���ֽڱ���Ϊż��*/
	
	for(i=0;i<6;i++)
		Write_W5100(W5100_SHAR+i,Phy_Addr[i]);			/*PHY_ADDR6�ֽ�unsigned char����,�Լ�����*/

	/*���ñ�����IP��ַ��4���ֽ�
	ע�⣬����IP�����뱾��IP����ͬһ�����������򱾻����޷��ҵ�����*/
	for(i=0;i<4;i++)
		Write_W5100(W5100_SIPR+i,IP_Addr[i]);			/*IP_ADDRΪ4�ֽ�unsigned char����,�Լ�����*/

	/*���÷��ͻ������ͽ��ջ������Ĵ�С���ο�W5100�����ֲ�*/
 	Write_W5100(W5100_RMSR,0x55);		/*Socket Rx memory size=2k*/

//	Write_W5100(W5100_RMSR,0xAA);		/*Socket Rx memory size=2k*/
	Write_W5100(W5100_TMSR,0x55);		/*Socket Tx mempry size=2k*/

	/* ��������ʱ�䣬Ĭ��Ϊ2000(200ms) *///������Ϊ200msʱ��ʵ��Ϊ190~200ms֮�䣬�ʽ����ʵ�������Ϊ250ms by wqf 20100930
	Write_W5100(W5100_RTR,0x09);
	Write_W5100(W5100_RTR+1,0xC4);

	/* �������Դ�����Ĭ��Ϊ8�� */
	Write_W5100(W5100_RCR,8);

	/* �����жϣ��ο�W5100�����ֲ�ȷ���Լ���Ҫ���ж�����
	IMR_CONFLICT��IP��ַ��ͻ�쳣�ж�
	IMR_UNREACH��UDPͨ��ʱ����ַ�޷�������쳣�ж�
	������Socket�¼��жϣ�������Ҫ��� */
	Write_W5100(W5100_IMR,(IMR_CONFLICT|IMR_UNREACH|IMR_S0_INT|IMR_S1_INT|IMR_S2_INT|IMR_S3_INT));
}
/****************************************************************************
                            Detect Gateway
input:  	None
Output: 	None
Return: 	if fail to detect gateway, return FALSE
		if detect the gateway, return TRUE
****************************************************************************/
uint8 Detect_Gateway(void)
{
	unsigned char i;

	Write_W5100((W5100_S0_MR),S_MR_TCP);		/*����socket0ΪTCPģʽ*/

	Write_W5100((W5100_S0_CR),S_CR_OPEN);		/*��socket0*/

	if(Read_W5100(W5100_S0_SSR)!=S_SSR_INIT)
	{
		Write_W5100((W5100_S0_CR),S_CR_CLOSE);	/*�򿪲��ɹ����ر�Socket��Ȼ�󷵻�*/
		return FALSE;
	}

	/*������ؼ���ȡ���ص������ַ*/
	for(i=0;i<4;i++)
		Write_W5100((W5100_S0_DIPR+i),IP_Addr[i]+1);	/*��Ŀ�ĵ�ַ�Ĵ���д���뱾��IP��ͬ��IPֵ*/

	Write_W5100((W5100_S0_CR),S_CR_CONNECT);		/*��socket0��TCP����*/

	W5100_Delay_1MS(20);						/* ��ʱ20ms */

	i=Read_W5100(W5100_S0_DHAR);			/*��ȡĿ�������������ַ���õ�ַ�������ص�ַ*/

	Write_W5100((W5100_S0_CR),S_CR_CLOSE);			/*�ر�socket0*/

	if(i==0xff)
	{
		/**********û���ҵ����ط���������û�������ط������ɹ�����***********/
		/**********              �Լ���Ӵ������                ***********/
		return FALSE;
	}
	return TRUE;
}

/******************************************************************************
                           Socket����, ����3��Socket�Ĵ���ɲ��մ˳���
*****************************************************************************

						Socket��ʼ��
����ɹ��򷵻�true, ���򷵻�false
-----------------------------------------------------------------------------*/
void Socket_Init(SOCKET s)
{
	unsigned char i;

	/*���÷�Ƭ���ȣ��ο�W5100�����ֲᣬ��ֵ���Բ��޸�*/
	Write_W5100((W5100_S0_MSS+s*0x100),0x05);		/*����Ƭ�ֽ���=1460*/
	Write_W5100((W5100_S0_MSS+s*0x100+1),0xb4);

	/* Set Socket Port number */
	switch(s)
	{
		case 0:
			Write_W5100(W5100_S0_PORT,S0_Port[0]);	/* Set Local Socket Port number */
			Write_W5100(W5100_S0_PORT+1,S0_Port[1]);

			Write_W5100(W5100_S0_DPORT,S0_DPort[0]);	/* Set Destination port number */
			Write_W5100(W5100_S0_DPORT+1,S0_DPort[1]);

			for(i=0;i<4;i++)
				Write_W5100(W5100_S0_DIPR+i,S0_DIP[i]);	/* Set Destination IP Address */
			break;
		case 1:
			Write_W5100(W5100_S1_PORT,S1_Port[0]);	/* Set Local Socket Port number */
			Write_W5100(W5100_S1_PORT+1,S1_Port[1]);

			Write_W5100(W5100_S1_DPORT,S1_DPort[0]);	/* Set Destination port number */
			Write_W5100(W5100_S1_DPORT+1,S1_DPort[1]);

			for(i=0;i<4;i++)
				Write_W5100(W5100_S1_DIPR+i,S1_DIP[i]);	/* Set Destination IP Address */
			break;
		case 2:
			Write_W5100(W5100_S2_PORT,S2_Port[0]);	/* Set Local Socket Port number */
			Write_W5100(W5100_S2_PORT+1,S2_Port[1]);

			Write_W5100(W5100_S2_DPORT,S2_DPort[0]);	/* Set Destination port number */
			Write_W5100(W5100_S2_DPORT+1,S2_DPort[1]);

			for(i=0;i<4;i++)
				Write_W5100(W5100_S2_DIPR+i,S2_DIP[i]);	/* Set Destination IP Address */
			break;
		case 3:
			Write_W5100(W5100_S3_PORT,S3_Port[0]);	/* Set Local Socket Port number */
			Write_W5100(W5100_S3_PORT+1,S3_Port[1]);

			Write_W5100(W5100_S3_DPORT,S3_DPort[0]);	/* Set Destination port number */
			Write_W5100(W5100_S3_DPORT+1,S3_DPort[1]);

			for(i=0;i<4;i++)
				Write_W5100(W5100_S3_DIPR+i,S3_DIP[i]);	/* Set Destination IP Address */
			break;
		default:
			break;
	}
}
/*-----------------------------------------------------------------------------
                           ����SocketΪ�ͻ�����Զ�̷���������
������Socket�����ڿͻ���ģʽʱ�����øó�����Զ�̷�������������
������óɹ��򷵻�true�����򷵻�false
����������Ӻ���ֳ�ʱ�жϣ��������������ʧ�ܣ���Ҫ���µ��øó�������
�ó���ÿ����һ�Σ��������������һ������
------------------------------------------------------------------------------*/
uint8 Socket_Connect(SOCKET s)
{
	Write_W5100((W5100_S0_MR+s*0x100), S_MR_TCP);		/*����socketΪTCPģʽ */
	Write_W5100((W5100_S0_CR+s*0x100), S_CR_OPEN);		/*��Socket*/

	if(Read_W5100(W5100_S0_SSR+s*0x100)!=S_SSR_INIT)
	{
		Write_W5100(W5100_S0_CR+s*0x100,S_CR_CLOSE);	/*�򿪲��ɹ����ر�Socket��Ȼ�󷵻�*/
		return FALSE;
	}

	Write_W5100((W5100_S0_CR+s*0x100),S_CR_CONNECT);		/*����SocketΪConnectģʽ*/

	return TRUE;

	/*���������Socket�Ĵ����ӹ������������Ƿ���Զ�̷������������ӣ�����Ҫ�ȴ�Socket�жϣ�
	���ж�Socket�������Ƿ�ɹ����ο�W5100�����ֲ��Socket�ж�״̬*/
}

/*-----------------------------------------------------------------------------
                           ����Socket��Ϊ�������ȴ�Զ������������
������Socket�����ڷ�����ģʽʱ�����øó��򣬵ȵ�Զ������������
������óɹ��򷵻�true, ���򷵻�false
�ó���ֻ����һ�Σ���ʹW5100����Ϊ������ģʽ
-----------------------------------------------------------------------------*/
uint8 Socket_Listen(SOCKET s)
{
	Write_W5100((W5100_S0_MR+s*0x100), S_MR_TCP);		/*����socketΪTCPģʽ */
	Write_W5100((W5100_S0_CR+s*0x100), S_CR_OPEN);		/*��Socket*/

	if(Read_W5100(W5100_S0_SSR+s*0x100)!=S_SSR_INIT)
	{
		Write_W5100((W5100_S0_CR+s*0x100),S_CR_CLOSE);	/*�򿪲��ɹ����ر�Socket��Ȼ�󷵻�*/
		return FALSE;
	}

	Write_W5100((W5100_S0_CR+s*0x100), S_CR_LISTEN);		/*����SocketΪ����ģʽ*/

	if(Read_W5100(W5100_S0_SSR+s*0x100)!=S_SSR_LISTEN)
	{
		Write_W5100((W5100_S0_CR+s*0x100), S_CR_CLOSE);		/*���ò��ɹ����ر�Socket��Ȼ�󷵻�*/
		return FALSE;
	}

	return TRUE;

	/*���������Socket�Ĵ򿪺�������������������Զ�̿ͻ����Ƿ������������ӣ�����Ҫ�ȴ�Socket�жϣ�
	���ж�Socket�������Ƿ�ɹ����ο�W5100�����ֲ��Socket�ж�״̬
	�ڷ���������ģʽ����Ҫ����Ŀ��IP��Ŀ�Ķ˿ں�*/
}

/*-----------------------------------------------------------------------------
					����SocketΪUDPģʽ
���Socket������UDPģʽ�����øó�����UDPģʽ�£�Socketͨ�Ų���Ҫ��������
������óɹ��򷵻�true, ���򷵻�false
�ó���ֻ����һ�Σ���ʹW5100����ΪUDPģʽ
-----------------------------------------------------------------------------*/
uint8 Socket_UDP(SOCKET s)
{
	Write_W5100((W5100_S0_MR+s*0x100), S_MR_UDP);		/*����SocketΪUDPģʽ*/
	Write_W5100((W5100_S0_CR+s*0x100), S_CR_OPEN);		/*��Socket*/

	if(Read_W5100(W5100_S0_SSR+s*0x100)!=S_SSR_UDP)
	{
		Write_W5100((W5100_S0_CR+s*0x100), S_CR_CLOSE);	/*�򿪲��ɹ����ر�Socket��Ȼ�󷵻�*/
		return FALSE;
	}
	else
		return TRUE;

	/*���������Socket�Ĵ򿪺�UDPģʽ���ã�������ģʽ��������Ҫ��Զ��������������
	��ΪSocket����Ҫ�������ӣ������ڷ�������ǰ����������Ŀ������IP��Ŀ��Socket�Ķ˿ں�
	���Ŀ������IP��Ŀ��Socket�Ķ˿ں��ǹ̶��ģ������й�����û�иı䣬��ôҲ��������������*/
}


/******************************************************************************
                              ����Socket���պͷ��͵�����
******************************************************************************/
/*-----------------------------------------------------------------------------
���Socket�����������ݵ��жϣ������øó�����д���
�ó���Socket�Ľ��յ������ݻ��浽Rx_buffer�����У������ؽ��յ������ֽ���
-----------------------------------------------------------------------------*/
uint16 S_rx_process(SOCKET s)
{
	unsigned int i,j;
	unsigned short rx_size,rx_offset;



	/*��ȡ�������ݵ��ֽ���*/
	rx_size=Read_W5100(W5100_S0_RX_RSR+s*0x100);
	rx_size*=256;
	rx_size+=Read_W5100(W5100_S0_RX_RSR+s*0x100+1);

	/*��ȡ���ջ�������ƫ����*/
	rx_offset=Read_W5100(W5100_S0_RX_RR+s*0x100);
	rx_offset*=256;
	rx_offset+=Read_W5100(W5100_S0_RX_RR+s*0x100+1);

	i=rx_offset/S_RX_SIZE;				/*����ʵ�ʵ�����ƫ������S0_RX_SIZE��Ҫ��ǰ��#define�ж���*/
								/*ע��S_RX_SIZE��ֵ��W5100_Init()������W5100_RMSR��ȷ��*/
	rx_offset=rx_offset-i*S_RX_SIZE;

	j=W5100_RX+s*S_RX_SIZE+rx_offset;		/*ʵ�������ַΪW5100_RX+rx_offset*/
	for(i=0;i<rx_size;i++)
	{
		if(rx_offset>=S_RX_SIZE)
		{
			j=W5100_RX+s*S_RX_SIZE;
			rx_offset=0;
		}
		Rx_Buffer[i+s*Max_Size]=Read_W5100(j);		/*�����ݻ��浽Rx_buffer������*/
//		Rx_Buffer[DARec+i]=Read_W5100(j);		 //hong
		j++;
		rx_offset++;
	}
		
//		DARec=DARec+rx_size;


	/*������һ��ƫ����*/
	rx_offset=Read_W5100(W5100_S0_RX_RR+s*0x100);
	rx_offset*=256;
	rx_offset+=Read_W5100(W5100_S0_RX_RR+s*0x100+1);

	rx_offset+=rx_size;
	Write_W5100((W5100_S0_RX_RR+s*0x100), (rx_offset/256));
	Write_W5100((W5100_S0_RX_RR+s*0x100+1), rx_offset);

	Write_W5100((W5100_S0_CR+s*0x100), S_CR_RECV);			/*����RECV����ȵ���һ�ν���*/

	return rx_size;								/*���ؽ��յ������ֽ���*/
}

/*-----------------------------------------------------------------------------
���Ҫͨ��Socket�������ݣ������øó���
Ҫ���͵����ݻ�����Tx_buffer��, size����Ҫ���͵��ֽڳ���
-----------------------------------------------------------------------------*/
uint8 S_tx_process(SOCKET s,uint8 *buf, uint32 size)
{
	unsigned int i,j;
	unsigned short tx_free_size,tx_offset;

	/*��ȡ������ʣ��ĳ���*/
	tx_free_size=Read_W5100(W5100_S0_TX_FSR+s*0x100);
	tx_free_size*=256;
	tx_free_size+=Read_W5100(W5100_S0_TX_FSR+s*0x100+1);
	if(tx_free_size<size)						/*���ʣ����ֽڳ���С�ڷ����ֽڳ���,�򷵻�*/
		return FALSE;

	/*��ȡ���ͻ�������ƫ����*/
	tx_offset=Read_W5100(W5100_S0_TX_WR+s*0x100);
	tx_offset*=256;
	tx_offset+=Read_W5100(W5100_S0_TX_WR+s*0x100+1);

	i=tx_offset/S_TX_SIZE;					/*����ʵ�ʵ�����ƫ������S0_TX_SIZE��Ҫ��ǰ��#define�ж���*/
									/*ע��S0_TX_SIZE��ֵ��W5100_Init()������W5100_TMSR��ȷ��*/
	tx_offset=tx_offset-i*S_TX_SIZE;
	j=W5100_TX+s*S_TX_SIZE+tx_offset;		/*ʵ�������ַΪW5100_TX+tx_offset*/

	for(i=0;i<size;i++)
	{
		if(tx_offset>=S_TX_SIZE)
		{
			j=W5100_TX+s*S_TX_SIZE;
			tx_offset=0;
		}
		Write_W5100(j,buf[i]);						/*��Tx_buffer�������е�����д�뵽���ͻ�����*/
		j++;
		tx_offset++;
	}

	/*������һ�ε�ƫ����*/
	tx_offset=Read_W5100(W5100_S0_TX_WR+s*0x100);
	tx_offset*=256;
	tx_offset+=Read_W5100(W5100_S0_TX_WR+s*0x100+1);


	tx_offset+=size;
	Write_W5100((W5100_S0_TX_WR+s*0x100),(tx_offset/256));
	Write_W5100((W5100_S0_TX_WR+s*0x100+1),tx_offset);

	Write_W5100((W5100_S0_CR+s*0x100), S_CR_SEND);			/*����SEND����,��������*/

	return TRUE;								/*���سɹ�*/
}


/******************************************************************************
					W5100�жϴ��������
******************************************************************************/
void W5100_Interrupt_Process(void)
{
	unsigned char i,j;

	W5100_Interrupt=0;

	i=Read_W5100(W5100_IR);
LOOP:
	Write_W5100(W5100_IR, (i&0xf0));					/*��д����жϱ�־*/

	
	if((i & IR_CONFLICT) == IR_CONFLICT)	 	/*IP��ַ��ͻ�쳣�����Լ���Ӵ���*/
	{

	}

	if((i & IR_UNREACH) == IR_UNREACH)			/*UDPģʽ�µ�ַ�޷������쳣�����Լ���Ӵ���*/

	{
	}

	/* Socket�¼����� */
	if((i & IR_S0_INT) == IR_S0_INT)
	{
		j=Read_W5100(W5100_S0_IR);
		Write_W5100(W5100_S0_IR, j);		/* ��д���жϱ�־ */

		if(j&S_IR_CON)				/* ��TCPģʽ��,Socket0�ɹ����� */
		{
			S0_State|=S_CONN;
		}
		if(j&S_IR_DISCON)				/* ��TCPģʽ��Socket�Ͽ����Ӵ����Լ���Ӵ��� */
		{
			Write_W5100(W5100_S0_CR, S_CR_CLOSE);		/* �رն˿ڣ��ȴ����´����� */   
			S0_State=0;
		}
		if(j&S_IR_SENDOK)				/* Socket0���ݷ�����ɣ������ٴ�����S_tx_process()������������ */
		{
			S0_Data|=S_TRANSMITOK;
		}
		if(j&S_IR_RECV)				/* Socket���յ����ݣ���������S_rx_process()���� */
		{
			S0_Data|=S_RECEIVE;
			OSSemPost(g_JG0flag);	   //�����ź���
		}
		if(j&S_IR_TIMEOUT)			/* Socket���ӻ����ݴ��䳬ʱ���� */
		{
			Write_W5100(W5100_S0_CR, S_CR_CLOSE);		/* �رն˿ڣ��ȴ����´����� */
			S0_State=0;
		}
	}

	/* Socket1�¼����� */
	if((i&IR_S1_INT)==IR_S1_INT)
	{
		j=Read_W5100(W5100_S1_IR);
		Write_W5100(W5100_S1_IR, j);		/* ��д���жϱ�־ */

		if(j&S_IR_CON)				/* ��TCPģʽ��,Socket1�ɹ����� */
		{
			S1_State|=S_CONN;
		}
		if(j&S_IR_DISCON)		/* ��TCPģʽ��Socket1�Ͽ����Ӵ����Լ���Ӵ��� */
		{
			Write_W5100(W5100_S1_CR, S_CR_CLOSE);		/* �رն˿ڣ��ȴ����´����� */
			S1_State=0;
		}
		if(j&S_IR_SENDOK)		/* Socket1���ݷ�����ɣ������ٴ�����S_tx_process()������������ */
		{
			S1_Data|=S_TRANSMITOK;
		}
		if(j&S_IR_RECV)	  	/* Socket1���յ����ݣ���������S_rx_process()���� */
		{
			S1_Data|=S_RECEIVE;
			OSSemPost(g_JG1flag);  //�����ź���
		}
		if(j&S_IR_TIMEOUT)	/* Socket1���ӻ����ݴ��䳬ʱ���� */
		{
			Write_W5100(W5100_S1_CR, S_CR_CLOSE);		/*�رն˿ڣ��ȴ����´����� */
			S1_State=0;
		}
	}

	/* Socket2�¼����� */
	if((i&IR_S2_INT)==IR_S2_INT)
	{
		j=Read_W5100(W5100_S2_IR);
		Write_W5100(W5100_S2_IR, j);		/*��д���жϱ�־ */

		if(j&S_IR_CON)		/* ��TCPģʽ��,Socket2�ɹ����� */
		{
			S2_State|=S_CONN;
		}
		if(j&S_IR_DISCON)		/* ��TCPģʽ��Socket2�Ͽ����Ӵ����Լ���Ӵ��� */
		{
			Write_W5100(W5100_S2_CR, S_CR_CLOSE);		/* �رն˿ڣ��ȴ����´����� */
			S2_State=0;
		}
		if(j&S_IR_SENDOK)		/* Socket2���ݷ�����ɣ������ٴ�����S_tx_process()������������ */
		{
			S2_Data|=S_TRANSMITOK;
		}
		if(j&S_IR_RECV)		/* Socket2���յ����ݣ���������S_rx_process()���� */
		{
			S2_Data|=S_RECEIVE;
			OSSemPost(g_JG2flag);	   //�����ź���
		}
		if(j&S_IR_TIMEOUT)	/* Socket2���ӻ����ݴ��䳬ʱ���� */
		{
			Write_W5100(W5100_S2_CR, S_CR_CLOSE);		/*�رն˿ڣ��ȴ����´����� */
			S2_State=0;
		}
	}

	/* Socket3�¼����� */
	if((i&IR_S3_INT)==IR_S3_INT)
	{
		j=Read_W5100(W5100_S3_IR);
		Write_W5100(W5100_S3_IR, j);		/* ��д���жϱ�־ */

		if(j&S_IR_CON)		/* ��TCPģʽ��,Socket3�ɹ����� */
		{
			S3_State|=S_CONN;
		}
		if(j&S_IR_DISCON)		/* ��TCPģʽ��Socket3�Ͽ����Ӵ����Լ���Ӵ��� */
		{
			Write_W5100(W5100_S3_CR, S_CR_CLOSE);		/* �رն˿ڣ��ȴ����´����� */
//			Write_W5100(W5100_S3_CR, S_CR_LISTEN);	  			
			S3_State=0;
		}
		if(j&S_IR_SENDOK)				/* Socket3���ݷ�����ɣ������ٴ�����S_tx_process()������������ */
		{
			S3_Data|=S_TRANSMITOK;
		}
		if(j&S_IR_RECV)		/* Socket3���յ����ݣ���������S_rx_process()���� */
		{
			S3_Data|=S_RECEIVE;
//			g_UART1Cnt = S_rx_process(3);			
		    OSSemPost(g_JG3flag);
		}
		if(j&S_IR_TIMEOUT)	/* Socket3���ӻ����ݴ��䳬ʱ���� */
		{
			Write_W5100(W5100_S3_CR, S_CR_CLOSE);		/*�رն˿ڣ��ȴ����´����� */
			S3_State=0;
		}
	}

	i=Read_W5100(W5100_IR);
	if(i)
	{
		u32Interrupt_Num++;
		goto LOOP;
	}

}
