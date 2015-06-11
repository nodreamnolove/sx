/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			W5100App.c
** Last modified Date:  20110622
** Last Version:		1.0
** Descriptions:		5100��������
**
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
** Version:
** Descriptions:
**
*********************************************************************************************************/
#define	__W5100APP_C
#include "W5100App.h"
#include "WT_Task.h"
#include "WT_Task.h"
#include "RD_data.h"
#include "Timer0.h"
int G[1100];
int G1[1100];

static	NetInfo		m_sniLocal;		//����������Ϣ
uint32 tcount=1;
uint32 tcount1=0;
uint32 deng=0;
uint32 deng1=0;
uint32 miss=0;
int	Ttest1;
int	Ttest2;

//static	uint8		m_au8NetBuf[W5100BUFSIZE];

#define W5100_Disable 	( P3_OUTP_CLR = ( 1 << 11 ) )			//W5100��ʹ��
#define W5100_Enable	( P3_OUTP_SET = ( 1 << 11 ) )				//W5100ʹ��
#define W5100_Reset()	W5100_Disable;W5100_Disable;W5100_Enable;	//W5100��λ


/**********************************************************************
������: InitNetSetting
����: m_sniLocal	�����������
���: ��
����: ��
˵�������������������
**********************************************************************/
void SetNetSetting(NetInfo *p_sniLocal)
{
	memcpy(&m_sniLocal,p_sniLocal,sizeof(m_sniLocal));
}

/**********************************************************************
������: InitNetSetting
����: m_sniLocal	�����������
���: ��
����: ��
˵������ʼ��������� ���˿ںţ�IP��
**********************************************************************/
void InitNetSetting(void)
{
	unsigned char i;

	S0_State = 0x00;
	S1_State = 0x00;
	S2_State = 0x00;
	S3_State = 0x00;
					
	for(i=0; i<4; i++)
	{
		IP_Addr[i]		= m_sniLocal.au8IPAddr[4-i-1];			// ����IP��ַ  	  ++
		Sub_Mask[i]		= m_sniLocal.au8SubMask[4-i-1];			// ������������	  ++
		Gateway_IP[i]	= m_sniLocal.au8GatewayIP[4-i-1];		// �������ز���	  ++
	}

	/* ���������ַ */
	for(i=0; i<6; i++)
		Phy_Addr[i] = m_sniLocal.au8MACAddr[i];

	//�˿�0
	S0_Port[0] = (m_sniLocal.u32LocalPortNO >> 8) & 0xFF;		//�˿ں�	  ++
	S0_Port[1] = m_sniLocal.u32LocalPortNO & 0xFF; 	

#ifndef SIM_SOFTWARE						 //�Ƿ���
	for(i=0; i<4; i++)
	{
		S0_DIP[i] = m_sniLocal.au8ServerIP1[4-i-1];		// Ŀ��IP��ַ	
	}
	S0_DPort[0] = (m_sniLocal.u32ServerPortNO1 >> 8) & 0xFF;	 	//Ŀ�Ķ˿ں�
	S0_DPort[1] = m_sniLocal.u32ServerPortNO1 & 0xFF;

#else										//����	 
	S0_DIP[0] = 192;
	S0_DIP[1] = 168;
	S0_DIP[2] = 0;
	S0_DIP[3] = 20;
	S0_DPort[0] = (8081 >> 8) & 0xFF;	 	//Ŀ�Ķ˿ں�
	S0_DPort[1] = (8081) & 0xFF;	
#endif
	S0_Mode = TCP_CLIENT;							//���ö˿�0Ĭ�ϵĹ�����ʽΪUDP

	//�˿�1
	S1_Port[0] = ((m_sniLocal.u32LocalPortNO+1) >> 8) & 0xFF;		//�˿ں� ++
	S1_Port[1] = ((m_sniLocal.u32LocalPortNO+1) & 0xFF);	   	

	for(i=0; i<4; i++)
	{
		S1_DIP[i] = m_sniLocal.au8ServerIP2[4-i-1];		// Ŀ��IP��ַ2	
	}
	S1_DPort[0] = (m_sniLocal.u32ServerPortNO2 >> 8) & 0xFF;	 	//Ŀ�Ķ˿ں�
	S1_DPort[1] = m_sniLocal.u32ServerPortNO2 & 0xFF;
	S1_Mode = TCP_CLIENT;							//���ö˿�1Ĭ�ϵĹ�����ʽΪtcp 

		//�˿�2
	S2_Port[0] = ((m_sniLocal.u32LocalPortNO+2) >> 8) & 0xFF;		//�˿ں� ++
	S2_Port[1] = ((m_sniLocal.u32LocalPortNO+2) & 0xFF);	   	
#ifndef SIM_SOFTWARE	  
	for(i=0; i<4; i++)
	{
		S2_DIP[i] = m_sniLocal.au8ServerIP3[4-i-1];		// Ŀ��IP��ַ2	
	}
	S2_DPort[0] = (m_sniLocal.u32ServerPortNO3 >> 8) & 0xFF;	 	//Ŀ�Ķ˿ں�
	S2_DPort[1] = m_sniLocal.u32ServerPortNO3 & 0xFF;
#else 
   	S2_DIP[0] = 192;
	S2_DIP[1] = 168;
	S2_DIP[2] = 0;
	S2_DIP[3] = 20;
	S2_DPort[0] = (8082 >> 8) & 0xFF;	 	//Ŀ�Ķ˿ں�
	S2_DPort[1] = (8082) & 0xFF;

#endif
	S2_Mode = TCP_CLIENT;							//���ö˿�1Ĭ�ϵĹ�����ʽΪUDP 
			
			//�˿�3
	S3_Port[0] = ((m_sniLocal.u32LocalPortNO+3) >> 8) & 0xFF;		//�˿ں� ++
	S3_Port[1] = ((m_sniLocal.u32LocalPortNO+3) & 0xFF);	   	

	for(i=0; i<4; i++)
	{
		S3_DIP[i] = m_sniLocal.au8ServerIP4[4-i-1];		// Ŀ��IP��ַ2	
	}
	S3_DPort[0] = (m_sniLocal.u32ServerPortNO4 >> 8) & 0xFF;	 	//Ŀ�Ķ˿ں�
	S3_DPort[1] = m_sniLocal.u32ServerPortNO4 & 0xFF;
	S3_Mode = TCP_CLIENT;	

//===============================================================================================
	//�˿�2
//	S2_Port[0] = ((m_sniLocal.u32LocalPortNO+2) >> 8) & 0xFF;		//�˿ں�
//	S2_Port[1] = ((m_sniLocal.u32LocalPortNO+2) & 0xFF );
	
//	for(i=0; i<4; i++)
//		S2_DIP[i]		= m_sniLocal.au8ServerIP[i];		// Ŀ��IP��ַ

//#ifdef SIM_SOFTWARE
//	S2_DIP[0] =	192;
//	S2_DIP[1] =	168;
//	S2_DIP[2] =	0;
//	S2_DIP[3] =	19;  
//	S2_DPort[0] = (8080 >> 8) & 0xFF;	 	//Ŀ�Ķ˿ں�
//	S2_DPort[1] = 8080 & 0xFF;	 
//	S2_Mode = TCP_CLIENT;					//���ö˿�2Ĭ�ϵĹ�����ʽΪTCP������ 
//#else
//	S2_Mode = TCP_SERVER;
//#endif
	//�˿�3
//	S3_Port[0] = ((m_sniLocal.u32LocalPortNO+3) >> 8) & 0xFF;		//�˿ں�
//	S3_Port[1] = ((m_sniLocal.u32LocalPortNO+3) & 0xFF);
	
//	for(i=0; i<4; i++)
//		S3_DIP[i]		= m_sniLocal.Server_IP[4-i-1];		// Ŀ��IP��ַ 
//	
//	S3_DPort[0] = (m_sniLocal.Server_Port >> 8) & 0xFF;	 	//Ŀ�Ķ˿ں�
//	S3_DPort[1] = m_sniLocal.Server_Port & 0xFF; 
//	S3_Mode = TCP_CLIENT;						//���ö˿�3Ĭ�ϵĹ�����ʽΪTCP������ 
//	S3_Mode = TCP_SERVER;
}

/*****************************************************************
������: SetSocket
����: ��
���: ��
����: ��
˵�����ֱ�����4���˿ڣ����ݶ˿ڹ���ģʽ�����˿�����TCP��������TCP�ͻ���
      ��UDPģʽ��
      �Ӷ˿�״̬�ֽ�Socket_State�����ж϶˿ڵĹ������
*****************************************************************/
void SetSocket(void)
 {
	/* �˿� 0 */
	if(S0_State==0)
	{
		if(S0_Mode==TCP_SERVER)			/* TCP������ģʽ */
		{
			if(Socket_Listen(0)==TRUE)
				S0_State=S_INIT;
			else
				S0_State=0;
		}
		else if(S0_Mode==TCP_CLIENT) 	/* TCP�ͻ���ģʽ */
		{
			if(Socket_Connect(0)==TRUE)
				S0_State=S_INIT;
			else
				S0_State=0;
		}
		else							/* UDPģʽ */
		{
			if(Socket_UDP(0)==TRUE)
				S0_State=S_INIT|S_CONN;
			else
				S0_State=0;
		}
	}

	/* �˿� 1 */
	if(S1_State==0)
	{
		if(S1_Mode==TCP_SERVER)			/* TCP������ģʽ */
		{
			if(Socket_Listen(1)==TRUE)
				S1_State=S_INIT;
			else
				S1_State=0;
		}
		else if(S1_Mode==TCP_CLIENT)	/* TCP�ͻ���ģʽ */
		{
			if(Socket_Connect(1)==TRUE)
				S1_State=S_INIT;
			else
				S1_State=0;
		}
		else							/* UDPģʽ */
		{
			if(Socket_UDP(1)==TRUE)
				S1_State=S_INIT|S_CONN;
			else
				S1_State=0;
		}
	}

	/* �˿� 2 */
	if(S2_State==0)
	{
		if(S2_Mode==TCP_SERVER)			/* TCP������ģʽ */
		{
			if(Socket_Listen(2)==TRUE)
				S2_State=S_INIT;
			else
				S2_State=0;
		}
		else if(S2_Mode==TCP_CLIENT) 	/* TCP�ͻ���ģʽ */
		{
			if(Socket_Connect(2)==TRUE)
				S2_State=S_INIT;
			else
				S2_State=0;
		}
		else							/* UDPģʽ */
		{
			if(Socket_UDP(2)==TRUE)
				S2_State=S_INIT|S_CONN;
			else
				S2_State=0;
		}
	}

	/* �˿� 3 */
	if(S3_State==0)
	{
		if(S3_Mode==TCP_SERVER)			/* TCP������ģʽ */
		{
			if(Socket_Listen(3)==TRUE)
				S3_State=S_INIT;
			else
				S3_State=0;
		}
		else if(S3_Mode==TCP_CLIENT) 	/* TCP�ͻ���ģʽ */
		{
			if(Socket_Connect(3)==TRUE)
				S3_State=S_INIT;
			else
				S3_State=0;
		}
		else							/* UDPģʽ */
		{
			if(Socket_UDP(3)==TRUE)
				S3_State=S_INIT|S_CONN;
			else
				S3_State=0;
		}
	}
}

/*****************************************************************
������: InitializeW5100
����: ��
���: ��
����: ��
˵�����ȶ�W5100��ʼ����Ȼ�������أ����ֱ��ʼ��4���˿�
*****************************************************************/
void InitializeW5100(NetInfo *p_sniLocal)
{
	//��������
	P0_MUX_CLR = (1 << 1) | ( 1 << 0 );			//P0.0 P0.1 ����ΪP0.0 P0.1
	P0_DIR_CLR = (1 << 1) | ( 1 << 0 );			//P0.0 P0.1 ����Ϊ����

	P2_MUX_CLR = (1 << 0);						//GPIO_02 ����Ϊ GPIO_02
	P2_DIR_CLR = (1 << 27 );					//GPIO_02 ����Ϊ����

	sic2IrqFuncSet(2	, 0	,(unsigned int)IRQ_W5100);      //�ж�����GPIO_02��ΪW5100�ж��ź�  �½��ش����ж�
	

	W5100_Reset();
	
	SetNetSetting(p_sniLocal);
		
	InitNetSetting();

	W5100_Init();

	/* ������ط����� */
//	Detect_Gateway();

	/* �˿�0 */
	Socket_Init(0);

	/* �˿�1 */
	Socket_Init(1);

	/* �˿�2 */
	Socket_Init(2);

	/* �˿�3 */
	Socket_Init(3);

	//��ʼ����
	SetSocket();

//	SIC2_ER					|= (1 << 2);		 //??
	
//	sic2IrqFuncSet(2	, 0	,(unsigned int)IRQ_W5100);      //�ж�����GPIO_02��ΪW5100�ж��ź�  �½��ش����ж�

}
/*********************************************************************************************************
** Function name:		SendDataNet
** Descriptions:		�����ڷ�������
** input parameters:	p_u8SendBuf		��������ָ�� 
** 						p_u32Len		�������ݳ���
** output parameters:	��������
** Created by:			ZHANG Ye		  
** Created Date:		20110622	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		
** Modified date:	
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/

uint32 SendDataNet(SOCKET s,uint8 * p_u8SendBuf, uint32 p_u32Len)
{
	uint32	l_u32SendSize;
//	l_u32SendSize = (p_u32Len > W5100BUFSIZE) ? W5100BUFSIZE : p_u32Len;
//		
//	memcpy(Tx_Buffer, p_u8SendBuf, l_u32SendSize);
	if(Read_W5100(W5100_S0_SSR+s*0x100) == S_SSR_ESTABLISHED)
		l_u32SendSize = S_tx_process(s, p_u8SendBuf,p_u32Len);

	return l_u32SendSize;
}

//W5100�жϴ���
uint32 W5100_T0TC[4]={0};
void IRQ_W5100(void)		 //��Ҫʱ��TOTC  15��Լ15/1625 * 2 =0.018ms
{	 
	W5100_Interrupt = 1;

	W5100_T0TC[0] = T0TC;
	W5100_T0TC[2] = t0_count2;
	W5100_Interrupt_Process();				//�жϷ�ʽ

	W5100_T0TC[1] = T0TC;
	W5100_T0TC[3] = t0_count2;	   
}

