/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			Uart3.c
** Last modified Date:  2011-04-12
** Last Version:		1.0
** Descriptions:		串口3相关函数，用于数字接线盒
**
**--------------------------------------------------------------------------------------------------------
** Created by:			ZHANG Ye
** Created date:		2011-04-12
** Version:				1.0
** Descriptions:		Uart3
**
**--------------------------------------------------------------------------------------------------------
** Modified by:			ZHANG Ye			
** Modified date:		20110518		
** Version:
** Descriptions:
**
*********************************************************************************************************/
#define	__UART3_C
#include "Uart3.h"

//static	uint32	m_u32Ch;

void IRQ_UART3(void);		//中断处理

/*********************************************************************************************************
** Function name:		U3SendBytes
** Descriptions:		向串口3发送一个字节数据
** input parameters:	p_u8SendBuf		发送数据指针 
** 						p_u32Len		发送数据长度
**
** Created by:			ZHANG Ye		  
** Created Date:		20110331	  
**-------------------------------------------------------------------------------------------------------
** Modified by:			ZHANG Ye	
** Modified date:		20110517  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void U3SendBytes(uint8 * p_u8SendBuf, uint32 p_u32Len)
{
	uint32	l_u32Tmp;

	//等待发送寄存器将当前数据发送完毕
	for (l_u32Tmp = 0; l_u32Tmp < p_u32Len; l_u32Tmp ++)
	{
		while ((U3LSR & 0x40) == 0);	
		
		U3THR = *(p_u8SendBuf+l_u32Tmp);
	}
}

/*********************************************************************************************************
** Function name:     IRQ_Uart3
** Descriptions:      串口5接收中断服务程序
** input parameters:  none        
** output parameters: none
**
** Created by:		  
** Created Date:	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void IRQ_Uart3(void)
{
//	static int U32_ad[CHANNELNUM];
//	uint32 td;
	uint32 i = 0;
	
	/* 			
	*  UART3接收中断处理 			
	*/
	i = (uint32)(U3IIR&0x0f);                                  
	if (i!= 0x01) 
	{		                  					/* 若尚有中断待处理             */
//		switch (i)
//		{
//		    case 0x04:		
//		    case 0x0c:
//				if (U3LSR & 0x04)		//校验不通过，第一个字节
//				{
//					m_u32Ch	= 0;
//				}
//				else				   	//非第一个字节
//				{
//					m_u32Ch	++;	 
//				}
//						
//				td	= m_u32Ch >> 1;
//		
//				if((m_u32Ch & 1)==0)
//					U32_ad[td]	= U3RBR << 8;
//				else
//					U32_ad[td]	+= U3RBR;
//				
//				if(m_u32Ch == (CHANNELNUM<<1)-1)
//				{
//					g_an32SPIADBuf[0][g_u8ADIn]	= U32_ad[0];
//					g_an32SPIADBuf[1][g_u8ADIn]	= U32_ad[1];
//#if CHANNELNUM > 2
//					g_an32SPIADBuf[2][g_u8ADIn]	= U32_ad[2];
//					g_an32SPIADBuf[3][g_u8ADIn]	= U32_ad[3];
//#endif
//					//
//					g_u8ADIn++;
//					g_u8ADcount++;
//		
//				}
//				break;
//				                                             
//	        default:
//		        break;
//		}
	}
	

}

/*********************************************************************************************************
** Function name:     UART3Init
** Descriptions:      令串口3完成全部初始化工作,Baund=Fpcld/[16*(DLM:DLL)]*X/Y
** input parameters:  p_iBaudRate	波特率        
** output parameters: none
**
** Created by:		  
** Created Date:	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void UART3Init(int p_iBaudRate)
{	
	/*  
    *  设置波特率
    */
	U3LCR		= 0x83;					/* 使能波特率分频器锁存的访问   */
	U3DLM		= UARTDLM;				/* 给分频器赋值，这里赋值为1    */ 
	U3DLL		= UARTDLL; 
	U3LCR		= 0x3B;					//强制奇偶校验为0
	U3LCR		= 0x03;	//不使用奇偶校验

	//switch (p_iBaudRate)
	switch (57600)
	{	
	case UBR_4800:
		U3CLK        = (1 << 8) | (169 << 0);
		break;
	
	case UBR_9600:
		U3CLK        = (3 << 8) | (254 << 0);
		break;
	
	case UBR_19200:
		U3CLK        = (3 << 8) | (127 << 0);
		break;

	case UBR_38400:
		U3CLK        = (6 << 8) | (127 << 0);
		break;

	case UBR_57600:
		U3CLK        = (9 << 8) | (127 << 0);
		break;

	case UBR_115200:
		U3CLK        = (19 << 8) | (134 << 0);
		break;

	case UBR_230400:
		U3CLK        = (19 << 8) | (67 << 0);
		break;

	case UBR_460800:
		U3CLK        = (38 << 8) | (67 << 0);
		break;

	default:
		U3CLK        = (19 << 8) | (134 << 0);
		break;
  	}

	/*
    *  使能UART3中断，开启波特率时钟
    */	 
	U3FCR  	     = 0x00;				//禁止FIFO
	U3FCR  	     = 0x30;
	U3IER        = 0x01;				/* 使能RDA                      */
	UART_CLKMODE |= 0x0020;				/* 选择UART3的时钟模式          */
	 
	/* 
    *  初始化UART VIC 中断接口，上升沿触发 
    */
	micIrqFuncSet(7, 1, (unsigned int)IRQ_Uart3 );             
	
}
