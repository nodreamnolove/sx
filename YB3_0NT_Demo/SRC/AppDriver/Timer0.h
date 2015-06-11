/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			Timer0.h
** Last modified Date:  2011-04-12
** Last Version:		1.0
** Descriptions:		Timer0初始化、中断处理
**
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
** Version:
** Descriptions:
**
*********************************************************************************************************/
#ifndef	__TIMER0_H
#define	__TIMER0_H

#include "config.h"

#ifdef	__TIMER0_C
#define	T0_EXT
#include "WDT.h"
#else
#define	T0_EXT	extern
#endif

//初始化
T0_EXT	void Time0Init(void);
extern	uint32 t0_count1;
extern	uint32 t0_count2;
extern uint32 Timer_cou_JG0;
extern uint32 Timer_cou_JG1;

#endif		//__TIMER0_H
