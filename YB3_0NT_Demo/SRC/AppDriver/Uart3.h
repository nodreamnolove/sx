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
#ifndef	__UART3_H
#define	__UART3_H

#include "config.h"

#ifdef	__UART3_C
#define	U3_EXT
#else
#define	U3_EXT	extern
#endif

//初始化
U3_EXT	void UART3Init(int p_iBaudRate);				
/*
#define		UBR_9600	9600		//串口波特率
#define		UBR_19200	19200		//串口波特率
#define		UBR_57600	57600		//串口波特率
#define		UBR_115200	115200		//串口波特率
#define		UBR_230400	230400		//串口波特率
#define		UBR_460800	460800		//串口波特率
*/

//串口发数
U3_EXT	void U3SendBytes(uint8 * p_u8SendBuf, uint32 p_u32Len);

#endif		//__UART3_H
