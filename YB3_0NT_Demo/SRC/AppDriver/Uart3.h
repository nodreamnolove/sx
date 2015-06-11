/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			Uart5.h
** Last modified Date:  2011-04-12
** Last Version:		1.0
** Descriptions:		Uart5��ʼ�����жϴ���
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

//��ʼ��
U3_EXT	void UART3Init(int p_iBaudRate);				
/*
#define		UBR_9600	9600		//���ڲ�����
#define		UBR_19200	19200		//���ڲ�����
#define		UBR_57600	57600		//���ڲ�����
#define		UBR_115200	115200		//���ڲ�����
#define		UBR_230400	230400		//���ڲ�����
#define		UBR_460800	460800		//���ڲ�����
*/

//���ڷ���
U3_EXT	void U3SendBytes(uint8 * p_u8SendBuf, uint32 p_u32Len);

#endif		//__UART3_H
