/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			WT_Task.h
** Last modified Date:  20110511
** Last Version:		1.0
** Descriptions:		
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

#ifndef	__WT_TASK_H
#define	__WT_TASK_H

#ifdef	__WT_TASK_C
#define	WT_EXT	 
#include "TaskKB.h"
#include "TaskTimer.h"
//#include "TaskAnalysis.h"
//#include "UIOperation.h"
//#include "Send_Task.h"	
#include "sdconfig.h"
#include "sdcommon.h"
#include "sddriver.h"
#include "Common.h"
					   
#if	YBVERSION >= 30		//3.0仪表功能
#include "IRQ.h"	   				 
#include "Uart5.h"
#define	UARTSENDDATA(a,b)	U5SendBytes(a,b) 
//#include "TaskSD.h"
#else	//2.2
#include "AllIRQ.h"					
#include "Uart0.h"	 
#define	UARTSENDDATA(a,b)	UART0SendByte(a,b) 
#endif	//#if	YBVERSION >= 30		//3.0仪表功能

//#include "UI.h"

#else
#define	WT_EXT	extern
#endif

#include "Config.h"


WT_EXT 	OS_STK	TaskStartStk[TASKSTACKSIZE];					
WT_EXT 	OS_STK	TaskChecknet[TASKSTACKSIZE];	
WT_EXT 	OS_STK  TaskSendStk[TASKSTACKSIZE];
WT_EXT 	OS_STK  TaskTiPo[TASKSTACKSIZE];
WT_EXT 	OS_STK  Send08Task[TASKSTACKSIZE];
WT_EXT 	OS_STK  TaskUart5[TASKSTACKSIZE];
WT_EXT 	OS_STK  TaskJG0[TASKSTACKSIZE];
WT_EXT 	OS_STK  TaskJG1[TASKSTACKSIZE];
WT_EXT 	OS_STK  TaskJG2[TASKSTACKSIZE];
WT_EXT 	OS_STK  TaskJG3[TASKSTACKSIZE];
WT_EXT 	OS_STK  TaskDataJG[TASKSTACKSIZE];
//WT_EXT 	OS_STK  TaskSD[TASKSTACKSIZE];
WT_EXT 	OS_STK  TaskUart5Senddata[TASKSTACKSIZE];
WT_EXT 	OS_STK  TaskTest[TASKSTACKSIZE];
WT_EXT 	OS_STK  TaskSendUart1[TASKSTACKSIZE];
WT_EXT 	OS_STK  TaskNetSend[TASKSTACKSIZE];
WT_EXT 	OS_STK  TaskMatchSend[TASKSTACKSIZE];

extern	void	Task_TiPo(void *pdata);
WT_EXT	void	TaskStart(void *pdata); 
WT_EXT	void	Task_Checknet(void *pdata);
WT_EXT	void	Task_JG0(void *tdata);
WT_EXT	void	Task_JG1(void *tdata);	 
WT_EXT	void	Task_JG2(void *tdata);
WT_EXT	void	Task_JG3(void *tdata);	
WT_EXT	void 	Task_Data_JG(void *tdata);
WT_EXT	void	Task_Uart5(void *tdata);
WT_EXT 	void 	Task_Uart5_Senddata(void *tdata);
WT_EXT 	void 	Task_Uart1_Senddata(void *tdata);
//WT_EXT 	void 	Task_SD(void *tdata);
WT_EXT 	void 	Task_SendUart1(void *tdata);
WT_EXT 	void 	Task_Net_Send(void *tdata);
WT_EXT	void 	Task_Match_Send(void *tdata);

WT_EXT	void 	Task_Test(void *tdata);

WT_EXT	void	RunStartTask(void);		//调用启动任务	 
WT_EXT	void	JZInit(void);			//启动OS前初始化
WT_EXT  uint8   JGCheck_Para(void);		//检查激光的特定参数
WT_EXT	void WeightAdd(uint8 l_u8AddMode);

WT_EXT	uint8 Flag_NetConnect;
WT_EXT  uint8 Flag_NetToPC;   //有线网络连接标志  1表示网络连接成功，0表示网络中断
WT_EXT	uint8  S_08[73];
WT_EXT	uint8 RdToLMSSendData[24];
WT_EXT	uint32 L_Data_Pro(uint8 *p); 

extern uint8  Send_data02[18];
extern uint8  Send_data08[65]; 
extern uint8  Send_data01_temp[500];
extern uint8  g_u8Flag_wireless;
extern uint32 g_u32JG1_Timermiss_count;
//extern uint8 SD_Buff_Send_VehInfo_Uart1[10][55];
//extern uint32 	SD_store_count;
extern uint32  SD_pro_count  ;
extern uint8   Flag_SD_Init_err;//SD卡初始化失败，
extern uint8 g_au8JG_4_Buff[4][832];  //激光数据缓存 4个；
extern uint8 g_u8Jg_4_Buff_Count;
extern uint8 g_au8Two_Buff[5000][1662];
extern uint32 g_u32Two_Buff_cout;
extern uint8 Read256_full(uint16 p_u16Addr, uint8 * p_pu8WriteBuf, uint16 p_u16Len);
extern uint8 Write256_full(uint16 p_u16Addr, uint8 * p_pu8WriteBuf, uint16 p_u16Len);
#endif	//__WT_TASK_H
