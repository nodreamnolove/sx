/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			Uart2.h
** Last modified Date:  20110518
** Last Version:		1.0
** Descriptions:		Uart2��ʼ�����жϴ���
**
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
** Version:
** Descriptions:
**
*********************************************************************************************************/
#ifndef	__UART2_H
#define	__UART2_H

#include "config.h"

#ifdef	__UART2_C
#define	U2_EXT
#else
#define	U2_EXT	extern
#endif


//��ʼ��
U2_EXT	void	UART2Init(int p_iBaudRate);				
/*
#define		UBR_9600	9600		//���ڲ�����
#define		UBR_19200	19200		//���ڲ�����
#define		UBR_57600	57600		//���ڲ�����
#define		UBR_115200	115200		//���ڲ�����
#define		UBR_230400	230400		//���ڲ�����
#define		UBR_460800	460800		//���ڲ�����
*/
U2_EXT	void	U2SendBytes(uint8 * p_u8SendBuf, uint32 p_u32Len);	//��������

#endif		//__UART2_H
