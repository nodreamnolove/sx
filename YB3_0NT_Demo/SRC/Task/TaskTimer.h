/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			TaskTimer.h
** Last modified Date:  20110512
** Last Version:		1.0
** Descriptions:		键盘任务
**
**--------------------------------------------------------------------------------------------------------
** Created by:			ZHANG Ye
** Created date:		20110512
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

#ifndef	__TASKTIMER_H
#define	__TASKTIMER_H
		
#include "Config.h"

#ifdef	__TASKTIMER_C
#define	TMR_EXT	

#if	YBVERSION >= 30		//3.0仪表功能
#include "PCF8563.h"  
#include "DS18B20.h"
#else					//2.2仪表功能									
#include "RTC.h"
#endif

#else
#define	TMR_EXT	extern
#endif

TMR_EXT	void	TaskRec2(void *pdata);
			
TMR_EXT	OS_STK	TaskRec2Stk[TASKSTACKSIZE];

#endif
