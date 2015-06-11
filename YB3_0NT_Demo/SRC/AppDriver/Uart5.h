/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			Uart5.h
** Last modified Date:  2011-04-12
** Last Version:		1.0
** Descriptions:		Uart5初始化、中断处理
**
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
** Version:
** Descriptions:
**
*********************************************************************************************************/
#ifndef	__UART5_H
#define	__UART5_H

#include "config.h"

#ifdef	__UART5_C
#define	U5_EXT
#else
#define	U5_EXT	extern
#endif


//初始化
U5_EXT	void	UART5Init(int p_iBaudRate);				
/*
#define		UBR_9600	9600		//串口波特率
#define		UBR_19200	19200		//串口波特率
#define		UBR_57600	57600		//串口波特率
#define		UBR_115200	115200		//串口波特率
#define		UBR_230400	230400		//串口波特率
#define		UBR_460800	460800		//串口波特率
*/
//接收指定数量的字节
U5_EXT	uint16 	U5ReciveByte(uint8 *p_pu8RcvDataBuf, uint8 p_u16RcvNum);

//发送指定数量的数据
U5_EXT	void	U5SendBytes(uint8 * p_u8SendBuf, uint32 p_u32Len);

U5_EXT	uint8	Rec_N;
U5_EXT	uint8	Rec_Buf[2048];
U5_EXT  uint32  g_Uart5_Count;
extern  OS_EVENT *g_Uart5_Rec;
extern uint8 Uart5RecByte(void);

#endif		//__UART5_H
