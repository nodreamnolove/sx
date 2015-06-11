/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			Uart5.c
** Last modified Date:  2011-04-12
** Last Version:		1.0
** Descriptions:		串口5相关函数
**
**--------------------------------------------------------------------------------------------------------
** Created by:			ZHANG Ye
** Created date:		2011-04-12
** Version:				1.0
** Descriptions:		Uart5
**
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
** Version:
** Descriptions:
**
*********************************************************************************************************/
#define	__UART5_C
#include "UART5.h"

#define		UARTRCVBUFSIZE		2048

static	uint8	m_u8UART5RcvBuf[UARTRCVBUFSIZE];			//串口5接收缓存
static	uint16	m_u16UART5SavIndex;							//串口5保存位置
static	uint16	m_u16UART5ReadIndex;						//串口5读取位置
//static	uint8	Rec_Buf[2048];
uint8	Rec_N;
//uint32  g_Uart5_Count=0;
//uint32  Uart_IRQ_count=0;
OS_EVENT *g_Uart5_Rec;

#define		BUFDATANUM		((m_u16UART5SavIndex + UARTRCVBUFSIZE - m_u16UART5ReadIndex) % UARTRCVBUFSIZE)

void IRQ_UART5(void);		//中断处理

/*********************************************************************************************************
** Function name:		U5SendBytes
** Descriptions:		向串口5发送一个字节数据
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
void U5SendBytes(uint8 * p_u8SendBuf, uint32 p_u32Len)
{
	uint32	l_u32Tmp;

	for (l_u32Tmp = 0; l_u32Tmp < p_u32Len; l_u32Tmp ++)
	{
		//等待发送寄存器将当前数据发送完毕
		while ((U5LSR & 0x40) == 0);	
		
		U5THR = *(p_u8SendBuf+l_u32Tmp);
	}
}

/*********************************************************************************************************
** Function name:		U5ReciveByte
** Descriptions:		向串口5发送一个字节数据
** input parameters:	*p_pu8RcvDataBuf	接收到的指针					 
** output parameters:	p_u16RcvNum			接收数量(该值为0时，将所有数据传出)
** Return Value:		实际接收数量：
**						当该值小于需要接收数量时，并不接收数据，而是返回Buf中有效字节的数量
**
** Created by:			ZHANG Ye		  
** Created Date:		20110331	  
**-------------------------------------------------------------------------------------------------------
** Modified by:			ZHANG Ye	
** Modified date:		20110518	
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
uint16 U5ReciveByte(uint8 *p_pu8RcvDataBuf, uint8 p_u16RcvNum)
{
	uint16	u16Tmp;
	uint16	u16Tmp2;
	u16Tmp	= BUFDATANUM;		//可接收数量
	if ((u16Tmp >= p_u16RcvNum) || (p_u16RcvNum == 0))		//可接收数量满足要求
	{
		if (p_u16RcvNum > 0)
			u16Tmp	= p_u16RcvNum;
		
		for(u16Tmp2 = 0; u16Tmp2 < u16Tmp; u16Tmp2++)
		{
			*(p_pu8RcvDataBuf + u16Tmp2) = m_u8UART5RcvBuf[m_u16UART5ReadIndex++];
			m_u16UART5ReadIndex	%= UARTRCVBUFSIZE;	
		}
	}
	return	u16Tmp;
}

/*********************************************************************************************************
** Function name:     IRQ_UART5
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
void IRQ_UART5(void)
{
	uint32 i = 0;
	
	/* 			
	*  UART5接收中断处理 			
	*/
	i = (uint32)(U5IIR&0x0f);                                  

	if (i!= 0x01) 
	{		                  					/* 若尚有中断待处理             */
		switch (i)
		{
		    case 0x04:		
		    case 0x0c:		
			    i	= (uint32)U5RXLEV;				/* 在中断里保存了i个有效数据*/
				for (;i>0;i--) 
				{
					m_u8UART5RcvBuf[m_u16UART5SavIndex++]	= (uint8)U5RBR;	 //hong
					m_u16UART5SavIndex	%= UARTRCVBUFSIZE;					   //hong
						//填满2048buff再从头开始；
				}
				OSSemPost(g_Uart5_Rec);	
				break;	                                                    
	        default:
		        break;
		}
	
	}
	
#if NOTDEBUG				//非调试状态
	WDTIM_COUNTER	= 0;									/* 喂狗							*/
#endif
}

/////////////
uint8 Uart5RecByte(void)
{
	uint8 temp;
		uint8 err;

	OSSemPend(g_Uart5_Rec,0,&err);
	temp = (uint8)U5RBR;
	return temp;
	

}
/////////////
/*********************************************************************************************************
** Function name:     UART5Init
** Descriptions:      令串口5完成全部初始化工作,Baund=Fpcld/[16*(DLM:DLL)]*X/Y
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
void UART5Init(int p_iBaudRate)
{
	/*  
    *  设置波特率
    */
	U5LCR        = 0x83;                                                /* 使能波特率分频器锁存的访问   */
	U5DLM        = UARTDLM;                                                 /* 给分频器赋值，这里赋值为1    */ 
	U5DLL        = UARTDLL; 
	U5LCR        = 0x03;                                                /* 使能THR,RBR,IER寄存器        */
	
	
	switch (p_iBaudRate)
	{	
	case UBR_4800:
		U5CLK        = (1 << 8) | (169 << 0);
		break;
	
	case UBR_9600:
		U5CLK        = (3 << 8) | (254 << 0);
		break;
	
	case UBR_19200:
		U5CLK        = (3 << 8) | (127 << 0);
		break;

	case UBR_38400:
		U5CLK        = (6 << 8) | (127 << 0);
		break;

	case UBR_57600:
		U5CLK        = (9 << 8) | (127 << 0);
		break;

	case UBR_115200:
		U5CLK        = (19 << 8) | (134 << 0);
		break;

	case UBR_230400:
		U5CLK        = (19 << 8) | (67 << 0);
		break;

	case UBR_460800:
		U5CLK        = (38 << 8) | (67 << 0);
		break;

	default:
		U5CLK        = (19 << 8) | (134 << 0);
		break;
  	}
	

	/*
    *  使能UART5中断，开启波特率时钟
    */	 
	U5FCR  	     = 0x3f;                   					            /* 使能FIFO，并设置FIFO触发深度 */ //16
	U5FCR  	     = 0xff;
	U5IER        = 0x01;		 					                    /* 使能RDA                      */
	UART_CLKMODE |= 0x0200;                                              /* 选择UART5的时钟模式          */ 
    
	m_u16UART5SavIndex		= 0;
	m_u16UART5ReadIndex		= 0;
	/* 
    *  初始化UART VIC 中断接口，上升沿触发 
    */
	micIrqFuncSet(9, 1, (unsigned int)IRQ_UART5 );             
	
}
