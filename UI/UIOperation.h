/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			UIOperation.h
** Last modified Date:  20110505
** Last Version:		1.0
** Descriptions:		程序所有界面操作函数
**
**--------------------------------------------------------------------------------------------------------
** Created by:			ZHANG Ye
** Created date:		20110505
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
#ifndef	__UIOPERATION_H
#define	__UIOPERATION_H

#include "Config.h"

#ifdef	__UIOPERATION_C

#define		UIO_EXT

#include "Common.h"
#include "TDC256.h"

#if	YBVERSION >= 30		//3.0仪表功能
#include "PCF8563.h"
#include "StorageApp.h"
#else	//2.2
#include "IAP.h"
#include "RTC.h"
#endif	//#if	YBVERSION >= 30		//3.0仪表功能

#else
#define		UIO_EXT		extern
#endif

UIO_EXT	void	SaveParams(void);			//保存参数设置
UIO_EXT void 	SaveThreshold(void);		//保存阈值 
UIO_EXT void 	SaveVehBuf(void);			//保存车辆缓存信息
#if	YBVERSION >= 30		//3.0仪表功能
UIO_EXT	void	SaveNetInfo(void);			//保存网络信息
#endif

UIO_EXT	void	InitSystem(void);			//初始化所有系统参数
UIO_EXT	void	InitNonWeight(void);		//初始化非称重设置
UIO_EXT	void	InitMotionModify(void);		//初始化动态修正参数
UIO_EXT	void	InitVehModify(void);		//初始化车型修正参数
UIO_EXT	void	InitStaticModify(void);		//初始化静态修正参数
UIO_EXT	void	InitThreshold(void);		//初始化阈值参数
UIO_EXT	void	InitVehBufIndex(void);		//初始化铁电缓存车辆信息
#if	YBVERSION >= 30		//3.0仪表功能														  
UIO_EXT	void 	InitNetParam(void);			//初始化网口信息
#endif

UIO_EXT	void	SaveTime(SystemTime p_stSet);				//保存时间

UIO_EXT	uint8	RecoverToLast(void);		//恢复为最近标定参数
UIO_EXT	uint8	RecoverToHistory(void);		//恢复为历史标定参数

UIO_EXT	void	ClearStartupCnt(void);		//启动次数清零
UIO_EXT	void	AddStartupCnt(void);		//启动次数加1

UIO_EXT	void	InitRestart(void);			//初始化后自动重启

#if	YBVERSION >= 30		//3.0仪表功能
UIO_EXT	uint16	SavePassVehInfo(VehicleRecord * p_pvrPassInfo);						//保存过车数据
UIO_EXT	void	ReadPassVehInfo(VehicleRecord * p_pvrPassInfo, uint16 p_u16Addr);	//读取过车数据
#endif	//#if	YBVERSION >= 30		//3.0仪表功能

#endif		//__UIOPERATION_H
