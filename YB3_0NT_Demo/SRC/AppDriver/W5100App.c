/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			W5100App.c
** Last modified Date:  20110622
** Last Version:		1.0
** Descriptions:		5100顶层驱动
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

static	NetInfo		m_sniLocal;		//本机网络信息
uint32 tcount=1;
uint32 tcount1=0;
uint32 deng=0;
uint32 deng1=0;
uint32 miss=0;
int	Ttest1;
int	Ttest2;

//static	uint8		m_au8NetBuf[W5100BUFSIZE];

#define W5100_Disable 	( P3_OUTP_CLR = ( 1 << 11 ) )			//W5100不使能
#define W5100_Enable	( P3_OUTP_SET = ( 1 << 11 ) )				//W5100使能
#define W5100_Reset()	W5100_Disable;W5100_Disable;W5100_Enable;	//W5100复位


/**********************************************************************
程序名: InitNetSetting
输入: m_sniLocal	本地网络参数
输出: 无
返回: 无
说明：设置网络参数设置
**********************************************************************/
void SetNetSetting(NetInfo *p_sniLocal)
{
	memcpy(&m_sniLocal,p_sniLocal,sizeof(m_sniLocal));
}

/**********************************************************************
程序名: InitNetSetting
输入: m_sniLocal	本地网络参数
输出: 无
返回: 无
说明：初始化网络参数 ，端口号，IP等
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
		IP_Addr[i]		= m_sniLocal.au8IPAddr[4-i-1];			// 加载IP地址  	  ++
		Sub_Mask[i]		= m_sniLocal.au8SubMask[4-i-1];			// 加载子网掩码	  ++
		Gateway_IP[i]	= m_sniLocal.au8GatewayIP[4-i-1];		// 加载网关参数	  ++
	}

	/* 加载物理地址 */
	for(i=0; i<6; i++)
		Phy_Addr[i] = m_sniLocal.au8MACAddr[i];

	//端口0
	S0_Port[0] = (m_sniLocal.u32LocalPortNO >> 8) & 0xFF;		//端口号	  ++
	S0_Port[1] = m_sniLocal.u32LocalPortNO & 0xFF; 	

#ifndef SIM_SOFTWARE						 //非仿真
	for(i=0; i<4; i++)
	{
		S0_DIP[i] = m_sniLocal.au8ServerIP1[4-i-1];		// 目的IP地址	
	}
	S0_DPort[0] = (m_sniLocal.u32ServerPortNO1 >> 8) & 0xFF;	 	//目的端口号
	S0_DPort[1] = m_sniLocal.u32ServerPortNO1 & 0xFF;

#else										//仿真	 
	S0_DIP[0] = 192;
	S0_DIP[1] = 168;
	S0_DIP[2] = 0;
	S0_DIP[3] = 20;
	S0_DPort[0] = (8081 >> 8) & 0xFF;	 	//目的端口号
	S0_DPort[1] = (8081) & 0xFF;	
#endif
	S0_Mode = TCP_CLIENT;							//设置端口0默认的工作方式为UDP

	//端口1
	S1_Port[0] = ((m_sniLocal.u32LocalPortNO+1) >> 8) & 0xFF;		//端口号 ++
	S1_Port[1] = ((m_sniLocal.u32LocalPortNO+1) & 0xFF);	   	

	for(i=0; i<4; i++)
	{
		S1_DIP[i] = m_sniLocal.au8ServerIP2[4-i-1];		// 目的IP地址2	
	}
	S1_DPort[0] = (m_sniLocal.u32ServerPortNO2 >> 8) & 0xFF;	 	//目的端口号
	S1_DPort[1] = m_sniLocal.u32ServerPortNO2 & 0xFF;
	S1_Mode = TCP_CLIENT;							//设置端口1默认的工作方式为tcp 

		//端口2
	S2_Port[0] = ((m_sniLocal.u32LocalPortNO+2) >> 8) & 0xFF;		//端口号 ++
	S2_Port[1] = ((m_sniLocal.u32LocalPortNO+2) & 0xFF);	   	
#ifndef SIM_SOFTWARE	  
	for(i=0; i<4; i++)
	{
		S2_DIP[i] = m_sniLocal.au8ServerIP3[4-i-1];		// 目的IP地址2	
	}
	S2_DPort[0] = (m_sniLocal.u32ServerPortNO3 >> 8) & 0xFF;	 	//目的端口号
	S2_DPort[1] = m_sniLocal.u32ServerPortNO3 & 0xFF;
#else 
   	S2_DIP[0] = 192;
	S2_DIP[1] = 168;
	S2_DIP[2] = 0;
	S2_DIP[3] = 20;
	S2_DPort[0] = (8082 >> 8) & 0xFF;	 	//目的端口号
	S2_DPort[1] = (8082) & 0xFF;

#endif
	S2_Mode = TCP_CLIENT;							//设置端口1默认的工作方式为UDP 
			
			//端口3
	S3_Port[0] = ((m_sniLocal.u32LocalPortNO+3) >> 8) & 0xFF;		//端口号 ++
	S3_Port[1] = ((m_sniLocal.u32LocalPortNO+3) & 0xFF);	   	

	for(i=0; i<4; i++)
	{
		S3_DIP[i] = m_sniLocal.au8ServerIP4[4-i-1];		// 目的IP地址2	
	}
	S3_DPort[0] = (m_sniLocal.u32ServerPortNO4 >> 8) & 0xFF;	 	//目的端口号
	S3_DPort[1] = m_sniLocal.u32ServerPortNO4 & 0xFF;
	S3_Mode = TCP_CLIENT;	

//===============================================================================================
	//端口2
//	S2_Port[0] = ((m_sniLocal.u32LocalPortNO+2) >> 8) & 0xFF;		//端口号
//	S2_Port[1] = ((m_sniLocal.u32LocalPortNO+2) & 0xFF );
	
//	for(i=0; i<4; i++)
//		S2_DIP[i]		= m_sniLocal.au8ServerIP[i];		// 目的IP地址

//#ifdef SIM_SOFTWARE
//	S2_DIP[0] =	192;
//	S2_DIP[1] =	168;
//	S2_DIP[2] =	0;
//	S2_DIP[3] =	19;  
//	S2_DPort[0] = (8080 >> 8) & 0xFF;	 	//目的端口号
//	S2_DPort[1] = 8080 & 0xFF;	 
//	S2_Mode = TCP_CLIENT;					//设置端口2默认的工作方式为TCP服务器 
//#else
//	S2_Mode = TCP_SERVER;
//#endif
	//端口3
//	S3_Port[0] = ((m_sniLocal.u32LocalPortNO+3) >> 8) & 0xFF;		//端口号
//	S3_Port[1] = ((m_sniLocal.u32LocalPortNO+3) & 0xFF);
	
//	for(i=0; i<4; i++)
//		S3_DIP[i]		= m_sniLocal.Server_IP[4-i-1];		// 目的IP地址 
//	
//	S3_DPort[0] = (m_sniLocal.Server_Port >> 8) & 0xFF;	 	//目的端口号
//	S3_DPort[1] = m_sniLocal.Server_Port & 0xFF; 
//	S3_Mode = TCP_CLIENT;						//设置端口3默认的工作方式为TCP服务器 
//	S3_Mode = TCP_SERVER;
}

/*****************************************************************
程序名: SetSocket
输入: 无
输出: 无
返回: 无
说明：分别设置4个端口，根据端口工作模式，将端口置于TCP服务器、TCP客户端
      或UDP模式。
      从端口状态字节Socket_State可以判断端口的工作情况
*****************************************************************/
void SetSocket(void)
 {
	/* 端口 0 */
	if(S0_State==0)
	{
		if(S0_Mode==TCP_SERVER)			/* TCP服务器模式 */
		{
			if(Socket_Listen(0)==TRUE)
				S0_State=S_INIT;
			else
				S0_State=0;
		}
		else if(S0_Mode==TCP_CLIENT) 	/* TCP客户端模式 */
		{
			if(Socket_Connect(0)==TRUE)
				S0_State=S_INIT;
			else
				S0_State=0;
		}
		else							/* UDP模式 */
		{
			if(Socket_UDP(0)==TRUE)
				S0_State=S_INIT|S_CONN;
			else
				S0_State=0;
		}
	}

	/* 端口 1 */
	if(S1_State==0)
	{
		if(S1_Mode==TCP_SERVER)			/* TCP服务器模式 */
		{
			if(Socket_Listen(1)==TRUE)
				S1_State=S_INIT;
			else
				S1_State=0;
		}
		else if(S1_Mode==TCP_CLIENT)	/* TCP客户端模式 */
		{
			if(Socket_Connect(1)==TRUE)
				S1_State=S_INIT;
			else
				S1_State=0;
		}
		else							/* UDP模式 */
		{
			if(Socket_UDP(1)==TRUE)
				S1_State=S_INIT|S_CONN;
			else
				S1_State=0;
		}
	}

	/* 端口 2 */
	if(S2_State==0)
	{
		if(S2_Mode==TCP_SERVER)			/* TCP服务器模式 */
		{
			if(Socket_Listen(2)==TRUE)
				S2_State=S_INIT;
			else
				S2_State=0;
		}
		else if(S2_Mode==TCP_CLIENT) 	/* TCP客户端模式 */
		{
			if(Socket_Connect(2)==TRUE)
				S2_State=S_INIT;
			else
				S2_State=0;
		}
		else							/* UDP模式 */
		{
			if(Socket_UDP(2)==TRUE)
				S2_State=S_INIT|S_CONN;
			else
				S2_State=0;
		}
	}

	/* 端口 3 */
	if(S3_State==0)
	{
		if(S3_Mode==TCP_SERVER)			/* TCP服务器模式 */
		{
			if(Socket_Listen(3)==TRUE)
				S3_State=S_INIT;
			else
				S3_State=0;
		}
		else if(S3_Mode==TCP_CLIENT) 	/* TCP客户端模式 */
		{
			if(Socket_Connect(3)==TRUE)
				S3_State=S_INIT;
			else
				S3_State=0;
		}
		else							/* UDP模式 */
		{
			if(Socket_UDP(3)==TRUE)
				S3_State=S_INIT|S_CONN;
			else
				S3_State=0;
		}
	}
}

/*****************************************************************
程序名: InitializeW5100
输入: 无
输出: 无
返回: 无
说明：先对W5100初始化，然后检查网关，最后分别初始化4个端口
*****************************************************************/
void InitializeW5100(NetInfo *p_sniLocal)
{
	//设置引脚
	P0_MUX_CLR = (1 << 1) | ( 1 << 0 );			//P0.0 P0.1 设置为P0.0 P0.1
	P0_DIR_CLR = (1 << 1) | ( 1 << 0 );			//P0.0 P0.1 设置为输入

	P2_MUX_CLR = (1 << 0);						//GPIO_02 配置为 GPIO_02
	P2_DIR_CLR = (1 << 27 );					//GPIO_02 设置为输入

	sic2IrqFuncSet(2	, 0	,(unsigned int)IRQ_W5100);      //中断引脚GPIO_02作为W5100中断信号  下降沿触发中断
	

	W5100_Reset();
	
	SetNetSetting(p_sniLocal);
		
	InitNetSetting();

	W5100_Init();

	/* 检查网关服务器 */
//	Detect_Gateway();

	/* 端口0 */
	Socket_Init(0);

	/* 端口1 */
	Socket_Init(1);

	/* 端口2 */
	Socket_Init(2);

	/* 端口3 */
	Socket_Init(3);

	//开始监听
	SetSocket();

//	SIC2_ER					|= (1 << 2);		 //??
	
//	sic2IrqFuncSet(2	, 0	,(unsigned int)IRQ_W5100);      //中断引脚GPIO_02作为W5100中断信号  下降沿触发中断

}
/*********************************************************************************************************
** Function name:		SendDataNet
** Descriptions:		向网口发送数据
** input parameters:	p_u8SendBuf		发送数据指针 
** 						p_u32Len		发送数据长度
** output parameters:	发送数量
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

//W5100中断处理
uint32 W5100_T0TC[4]={0};
void IRQ_W5100(void)		 //需要时间TOTC  15；约15/1625 * 2 =0.018ms
{	 
	W5100_Interrupt = 1;

	W5100_T0TC[0] = T0TC;
	W5100_T0TC[2] = t0_count2;
	W5100_Interrupt_Process();				//中断方式

	W5100_T0TC[1] = T0TC;
	W5100_T0TC[3] = t0_count2;	   
}

