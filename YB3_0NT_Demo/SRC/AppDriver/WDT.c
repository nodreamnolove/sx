/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			WDT.c
** Last modified Date:  2011-04-12
** Last Version:		1.0
** Descriptions:		看门狗相关函数
**
**--------------------------------------------------------------------------------------------------------
** Created by:			ZHANG Ye
** Created date:		2011-04-12
** Version:				1.0
** Descriptions:		WDT
**
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
** Version:
** Descriptions:
**
*********************************************************************************************************/
#define	__WDT_C
#include "WDT.h"

/*********************************************************************************************************
** Function name:		WDTInit
**
** Descriptions:		WDT初始化函数
**
** input parameters:	None
** output parameters:	None
** Returned value:		None
**
** Created by:			Tang Libing
** Created Date:		2009-3-11
**--------------------------------------------------------------------------------------------------------
** Modified by:			ZHANG Ye
** Modified date:		20110331
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void WDTInit(uint8 start)
{
    TIMCLK_CTRL		|= 0x01;                       				/* 使能看门狗定时器时钟         */

    WDTIM_CTRL		= 0x00;                       				/* 停止看门狗定时器计数         */
    WDTIM_CTRL		= 0x02;                           			/* 复位看门狗定时器计数         */

    WDTIM_COUNTER	= 0;                                		/* 清零看门狗定时器计数器       */
    WDTIM_MCTRL		= 0x08;                          			/* 匹配产生内部器件复位         */
	WDTIM_MATCH0	= 0x10000000;  								/* 给看门狗匹配寄存器赋值       */
    WDTIM_PULSE		= 0x01;   									/* 设置看门狗复位脉冲数为6个Fpc */
                                        						/* lk                           */
    WDTIM_EMR		= 0x20;                        				/* 外部匹配寄存器设置为匹配时输 */

   
    WDTIM_CTRL		= start;                       				/* 启动看门狗定时器             */
	g_u8WDTFlag		= 0;
}
