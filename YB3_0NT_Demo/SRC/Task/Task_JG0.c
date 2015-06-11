/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			Task_JG0.C
** Last modified Date:  20120718
** Last Version:		1.0
** Descriptions:		垂直激光接收任务（网络端口0接收）
**
**--------------------------------------------------------------------------------------------------------
** Created by:			Hong XiangYuan
** Created date:		20120718
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
#include "Task_JG0.h"
#include "W5100.h"
#include "WT_Task.h"
#include "Timer0.h"
#include "JZStructure.h"
#include "JZGlobal.h"
#include "math.h"	
#include "Task_Data_JG.h"
#include "JG.h"


#define		SETUPALIAS				g_sspSetup
uint32 g_u32JG0_count=0;	 //JG0接收帧数计数；
//uint32 g_u32JG0_miss_count=0;	 //JG0连续丢帧计数；

uint32 JG0_time_T0TC[4]={0};
uint32 JG0_time_counter[4]={0};
uint32 jg0=0;
extern uint32 g_au32Tempa[5000][4];
uint32 g_au32count=0;

/********JG0数据读和处理记录变量***********/
uint8 	g_u8JG0_RBuff_Count=0; //两激光缓存计数；
uint8 	g_u8JG0_PBuff_Count=0; //两激光缓存计数；
uint8 	g_au8JG0_3Buff[3][831];  //激光0数据缓存3个；
int32   LMS_data_0[3][362]={0};
uint8   JG0_CurBuff[831]={0};
uint8   g_au8recvBuff[1460];
int32   l_32tmp2,l_tmp1,TempVaule1;
extern int32   LMS_data_1[3][362];
extern uint8 	g_u8JG1_PBuff_Count;
/******************************************/
void Task_JG0(void *tdata)
{
	uint8 err;
	uint32	i;
	uint32  j;
	uint16  l_u16index,l_u16tmp;
	uint16  l_leftXpt, l_rightXpt;   //左右X距离对应的点数
	int32   Len_0=0;
	uint32 l_u32JG0_err_count=0;  //JG0接收错误计数
	tdata=tdata;  	
	while(1)
	{
		OSSemPend(g_JG0flag,0,&err);
		if(OS_NO_ERR == err)  //调用成功
		{
			JG0_time_T0TC[0] = T0TC;
			JG0_time_counter[0] = t0_count2;
			S0_Data&=~S_RECEIVE;
			i=S_rx_process(0);
			g_u32JG0_count++;
#ifndef SIM_SOFTWARE	
			if(i != 831)
			{
//				l_u32JG0_err_count = l_u32JG0_err_count+1;
//				SETUPALIAS.u32Net1_InvalidRecNum++;
			}
			else   //存数据
			{	  
				memcpy(g_au8JG0_3Buff[g_u8JG0_RBuff_Count],Rx_Buffer,Max_Size);
				g_u8JG0_RBuff_Count=g_u8JG0_RBuff_Count+1;
				g_u8JG0_RBuff_Count=g_u8JG0_RBuff_Count%3;
			}	
#else 
		    if(i!=1460)
			{
			
			}
			else
			{
				memcpy(g_au8recvBuff , Rx_Buffer,i);
				g_u8JG0_PBuff_Count  = 1;
				g_u8JG1_PBuff_Count  = 1;
				for(i=85,j=0;i<806&&j<POINT_SUM;i=i+2,j++)
				{		
					LMS_data_0[g_u8JG0_PBuff_Count][j]=	(g_au8recvBuff[i-71]<<8)+g_au8recvBuff[i-70];
					LMS_data_1[g_u8JG1_PBuff_Count][j]=	(g_au8recvBuff[i+651]<<8)+g_au8recvBuff[i+652];	
				}  
				LMS_data_0[g_u8JG0_PBuff_Count][361] = ((g_au8recvBuff[8]<<24)+(g_au8recvBuff[9]<<16)+(g_au8recvBuff[10]<<8)+g_au8recvBuff[11])/1000;				
				LMS_data_1[g_u8JG1_PBuff_Count][361] = ((g_au8recvBuff[8]<<24)+(g_au8recvBuff[9]<<16)+(g_au8recvBuff[10]<<8)+g_au8recvBuff[11])/1000;
				OSSemPost(g_JG_Pro); //仿真抛信号
			}

#endif
	   }//if(os_no_err)	
	   	/********************每激光器每帧实时处理***************************/
#ifndef SIM_SOFTWARE
		if(g_u8JG0_RBuff_Count!=g_u8JG0_PBuff_Count)
		 { 	
			memcpy(JG0_CurBuff,g_au8JG0_3Buff[g_u8JG0_PBuff_Count],831);	
		 	Len_0= (JG0_CurBuff[83]<<8)+JG0_CurBuff[84];
			if(Len_0 == POINT_SUM)
			{ 
				for(i=85,j=0;i<807 && j < POINT_SUM; i=i+2,j++)	//20130426 修改，去掉偏移量	
				{
					LMS_data_0[g_u8JG0_PBuff_Count][j]=	(JG0_CurBuff[i]<<8)+JG0_CurBuff[i+1];	 //每点座标
					if(LMS_data_0[g_u8JG0_PBuff_Count][j] < 0)									 //每点座标
					LMS_data_0[g_u8JG0_PBuff_Count][j] = 0;										 //每点座标
				}	
				LMS_data_0[g_u8JG0_PBuff_Count][361] = ((JG0_CurBuff[42]<<24)+(JG0_CurBuff[43]<<16)+(JG0_CurBuff[44]<<8)+JG0_CurBuff[45])/1000;
			//添加时间信息
			LMS_data_0[g_u8JG0_PBuff_Count][362]=t0_count2;
			OSSemPost(g_JG_Pro);  //实际抛信号
			}
		}
#endif
	}
}





