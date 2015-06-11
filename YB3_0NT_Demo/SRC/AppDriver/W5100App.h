/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			W5100App.h
** Last modified Date:  20110622
** Last Version:		1.0
** Descriptions:		5100Ӧ��
**
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
** Version:
** Descriptions:
**
*********************************************************************************************************/
#ifndef	__W5100APP_H
#define	__W5100APP_H

#include "config.h"	
#include "W5100.h"

#ifdef	__W5100APP_C
#define	W5A_EXT
#include "Common.h"

#else
#define	W5A_EXT	extern
#endif

W5A_EXT	void	SetNetSetting(NetInfo *p_sniLocal);		//��������������ã��˿ںţ�IP��
W5A_EXT	void	InitNetSetting(void);					//��ʼ�������������
W5A_EXT	void	InitializeW5100(NetInfo *p_sniLocal);	//��ʼ��5100оƬ
W5A_EXT	void	SetSocket(void);
W5A_EXT	void	IRQ_W5100(void);						//�����жϴ���
W5A_EXT	uint32	SendDataNet(SOCKET s,uint8 * p_u8SendBuf, uint32 p_u32Len);		//������������
W5A_EXT	int G[1100];
W5A_EXT	int G1[1100];



#endif	// __W5100APP_H
