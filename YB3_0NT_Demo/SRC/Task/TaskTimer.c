/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			TaskTimer.C
** Last modified Date:  20110512
** Last Version:		1.0
** Descriptions:		时钟任务
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
#define	__TASKTIMER_C
#include "TaskTimer.h"

#define TASKTIMERDLY	50	  

#define		SETUPALIAS				g_sspSetup			//设置参数结构	 
/*********************************************************************************************************
** Function name:		TaskRec2
** Descriptions:		任务2，时钟任务
** input parameters:	None 
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110512	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void  TaskRec2(void *tdata)						
{
   	
#if	YBVERSION >= 30		//3.0仪表功能
	uint32	l_au32TemperatureBuf[4];
	uint8	l_u8TemperatureIndex;
	uint32	l_u32TemperatureAll;
	uint32	l_u32TemperatureTmp;

#endif	//#if	YBVERSION >= 30		//3.0仪表功能
	tdata = tdata; 
	OSTimeDly(2);

#if	YBVERSION >= 30		//3.0仪表功能	
	RTC8563Init();	//时间设置；

	//设置启动时间
	GetRTCTime(&g_sstStartTime);
	g_u32Temprature		= 0;		
#else		//2.2仪表功能	
	RTCInit();

	//设置启动时间
	g_sstStartTime.u16Year	= g_sstTempTime.u16Year;
	g_sstStartTime.u8Month	= g_sstTempTime.u8Month;
	g_sstStartTime.u8Day	= g_sstTempTime.u8Day;
	g_sstStartTime.u8Hour	= g_sstTempTime.u8Hour;
	g_sstStartTime.u8Minute	= g_sstTempTime.u8Minute;
	g_sstStartTime.u8Second	= g_sstTempTime.u8Second;	  
#endif

	////////////////////////////////////////////////////////////////////////////
			
	while (1)
	{ 
		GetRTCTime(&g_sstCurTime);					

#if	YBVERSION >= 30		//3.0仪表功能		
		if (g_sstCurTime.u8Second & 0x01)
			LEDON();
		else
			LEDOFF();
			
		l_u32TemperatureTmp	= ReadTemperature();	//读取温度，当温度小于70度时记录
		if (l_u32TemperatureTmp < 70)
		{
			l_u32TemperatureAll	= l_u32TemperatureAll + l_u32TemperatureTmp - l_au32TemperatureBuf[l_u8TemperatureIndex];
			l_au32TemperatureBuf[l_u8TemperatureIndex]	= l_u32TemperatureTmp;
			g_u32Temprature	= l_u32TemperatureAll >> 2;

			l_u8TemperatureIndex	++;
			if (l_u8TemperatureIndex >= 4)
				l_u8TemperatureIndex	%= 4;
		}
		if (g_u32Temprature > 100) 
			g_u32Temprature	= 99;
	
#endif		 					  
		OSSemAccept(g_psemScreenRefresh);
		OSSemPost(g_psemScreenRefresh);

#if  SHOWVEHPASSDEBUG > 0	//显示过车调试代码
		if (g_u16VehDebugIndex > 1000)
			g_u16VehDebugIndex = 0;
#endif
		OSTimeDly(TASKTIMERDLY); 		
	}

}
