/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			DEFConfig.h
** Last modified Date:  20110718
** Last Version:		1.0
** Descriptions:		配置宏定义
**
**--------------------------------------------------------------------------------------------------------
** Created by:			ZHANG Ye
** Created date:		20110718
** Version:				1.0
** Descriptions:		The original version
**
**--------------------------------------------------------------------------------------------------------
** Modified by:			Wang ZiFeng
** Modified date:		20130318
** Version:				2.0
** Descriptions:
**
*********************************************************************************************************/
#ifndef	__DEFCONFIG_H
#define	__DEFCONFIG_H

#define		JGJD_VERSION		"S1-20140828"	   //程序版本号

//#define		PROGRAMVERSION		"BC B1.2.3.1"

#define		SHOWVEHPASSDEBUG 	0			//显示动态过车状态


#if	YBVERSION >= 30	 
#define		TASKSTACKSIZE		10240	//任务堆栈大小
#else								
#define		TASKSTACKSIZE		512
#endif

#define		POINTRATE			1000		//采样率   	
#define		CHANNELNUM			4			//通道数   


#define		DEBUG_NT			0			//调试状态:1 调试；	0 非调试   

#endif	//#ifndef	__DEFCONFIG_H
