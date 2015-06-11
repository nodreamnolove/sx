/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			W5100App.h
** Last modified Date:  20110622
** Last Version:		1.0
** Descriptions:		5100应用
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

W5A_EXT	void	SetNetSetting(NetInfo *p_sniLocal);		//设置网络参数设置，端口号，IP等
W5A_EXT	void	InitNetSetting(void);					//初始化网络参数设置
W5A_EXT	void	InitializeW5100(NetInfo *p_sniLocal);	//初始化5100芯片
W5A_EXT	void	SetSocket(void);
W5A_EXT	void	IRQ_W5100(void);						//网口中断处理
W5A_EXT	uint32	SendDataNet(SOCKET s,uint8 * p_u8SendBuf, uint32 p_u32Len);		//发送网络数据
W5A_EXT	int G[1100];
W5A_EXT	int G1[1100];



#endif	// __W5100APP_H
