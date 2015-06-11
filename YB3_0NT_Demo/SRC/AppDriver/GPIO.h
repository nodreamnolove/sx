/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			GPIO.h
** Last modified Date:  2011-03-26
** Last Version:		1.0
** Descriptions:		通用接口函数，如振铃等，包括LCD
**
**--------------------------------------------------------------------------------------------------------
** Created by:			ZHANG Ye
** Created date:		2011-03-26
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
#ifndef	__GPIO_H
#define	__GPIO_H

#include "Config.h"

#ifdef __GPIO_C
#define		GPIO_EXT
#else
#define		GPIO_EXT	extern
#endif

#define		BEEP				1 << 6					/* GPO_07控制BEEP，低电平蜂鸣   */

#define		BeepOFF()	        P3_OUTP_CLR = BEEP		//振铃响
#define		BeepON()			P3_OUTP_SET = BEEP		//振铃不响

#define		LEDBIT				(1<<9)					//GPO_9

#define		LEDON()				P3_OUTP_CLR = LEDBIT	//LED亮
#define		LEDOFF()			P3_OUTP_SET = LEDBIT	//LED灭


#define		BDMODE				(P3_INP_STATE & (1<<2))		//拨码开关，是否进入标定状态

#define		LCM_LEDCON			(1<<14)					//控制背光开关  GPO14
#define		LCDRST				(1<<1)					//LCD复位引脚   GPO01

#define		DELAY_TIME			100						/* 基本延迟时间					*/
#define		TICKS_PER_SEC		100						/* 每秒钟Tick数,与OS_CFG.H一致	*/

//#define		GSTriggerPIN		(((P3_INP_STATE >> 28) & 1) & g_u32SoftGS )		//遮挡光栅信号：判断是否遮挡光栅	0:遮挡  1:松开
#define		LPTriggerPIN		((P3_INP_STATE >> 16) & 1)		//触发线圈信号
#define		GSErrPIN			((P3_INP_STATE >> 5) & 1)		//光栅故障信号
#define		LPErrPIN			((P3_INP_STATE >> 9) & 1)		//线圈故障信号

#define		OUTBGON				((P3_INP_STATE >> 4) & 1)		//外部点亮显示屏跳线

#define     GPO_02              (1<<2)
#define     GPO_03              (1<<3)
#define     GPO_07              (1<<7)
#define     GPO_08              (1<<8)

#define     GPI_19              (1<<19)
#define     GPI_22              (1<<22)

GPIO_EXT	void	Delay(uint32 p_u32TickNumber);				/* 延时函数，参数为延时数		*/


#endif		//__GPIO_C
