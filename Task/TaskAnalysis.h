/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			TaskAnalysis.h
** Last modified Date:  20110512
** Last Version:		1.0
** Descriptions:		数据分析任务，包括分析行车方向(任务4)和计算重量(任务6)
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

#ifndef	__TASKANALYSIS_H
#define	__TASKANALYSIS_H

#ifdef	__TASKANALYSIS_C
#define	TAN_EXT	 
#include "no_Axle.h"
#include "Common.h"
#include "UIOperation.h"
#else
#define	TAN_EXT	extern
#endif

#include "Config.h"

TAN_EXT	void	TaskRec4(void *pdata);
TAN_EXT	void	TaskRec6(void *pdata);

TAN_EXT	OS_STK	TaskRec4Stk[TASKSTACKSIZE];	//收尾后任务
TAN_EXT	OS_STK	TaskRec6Stk[TASKSTACKSIZE];	//计算轴重任务

TAN_EXT	void  	YuChuli(void);
TAN_EXT	void  	DefaultVehType(VehicleInfo * p_psviVehicle);					//默认车型函数
TAN_EXT	void  	CalVehAxleGrpInfo(VehicleInfo *pstutVehicle,uint8 *ptaixing);	//用于计算车辆轴组类型
TAN_EXT	uint8	WuLunZhouChangeVehType(int32 n32WLZVeh,VehicleInfo *pstutVehicle);	//用于无轮轴程序修改轴组等
TAN_EXT	void  	CalVehAxleGrpWeight(VehicleInfo *pstutVehicle);
#endif
