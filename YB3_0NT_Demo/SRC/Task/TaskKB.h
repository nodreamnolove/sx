/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			TaskKB.h
** Last modified Date:  20110511
** Last Version:		1.0
** Descriptions:		键盘任务
**
**--------------------------------------------------------------------------------------------------------
** Created by:			ZHANG Ye
** Created date:		20110511
** Version:				1.0
** Descriptions:		
**
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
** Version:
** Descriptions:
**
*********************************************************************************************************/
#include "WT_Task.h"
#include "Common.h"
#ifndef	__TASKKB_H
#define	__TASKKB_H
		
#include "Config.h"

#ifdef	__TASKKB_C
#define	TKB_EXT		

#if	YBVERSION >= 30		//3.0仪表功能
#include "Keyboard.h"
#endif

#else
#define	TKB_EXT	extern
#endif

#include "KBMacro.h"

TKB_EXT	OS_STK	TaskRec3Stk[TASKSTACKSIZE];	//键盘任务
TKB_EXT	void	TaskRec3(void *pdata);


#endif
