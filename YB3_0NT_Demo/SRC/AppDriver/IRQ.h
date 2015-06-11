/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			IRQ.h
** Last modified Date:  2011-03-26
** Last Version:		1.0
** Descriptions:		硬中断初始化、处理函数、应用函数
**
**--------------------------------------------------------------------------------------------------------
** Created by:			ZHANG Ye
** Created date:		2011-03-31
** Version:				1.0
** Descriptions:		The original version
**
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
** Version:
** Descriptions:
**
*********************************************************************************************************/
#ifndef	__IRQ_H
#define	__IRQ_H

#include "Timer0.h"
#include "WDT.h"
#include "Uart5.h"
#include "Uart3.h" 
#include "Uart2.h"
#include "StorageApp.h"
#include "I2C1.h" 
#include "I2C2.h"
#include "DS2460.h"
#include "PCF8563.h"
#include "Keyboard.h"
#include "LCDApp.h"
#include "W5100App.h"
#include "DS18B20.h"


#ifdef IRQ_C
#define IRQ_EXT
#else
#define IRQ_EXT  extern
#endif

IRQ_EXT	void	InitAllIRQ(void);
IRQ_EXT	void	InitAllSP(uint32 p_u32U2, uint32 p_u32U3, uint32 p_u32U5);

#endif		//__IRQ_H
