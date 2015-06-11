/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			Task_JG1.C
** Last modified Date:  20120718
** Last Version:		1.0
** Descriptions:		倾斜激光接收任务（网络端口1接收）
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
#include "Task_JG1.h"
#include "Task_JG0.h"
#include "W5100.h"
#include "WT_Task.h"
#include "common.h"
#include "CMD.h"
#include "Uart5.h"
#include "Uart1.h"
#include "crc.h"
#include "Timer0.h"
#include "Task_Data_JG.h"
#include "JG.h"

#define		SETUPALIAS				g_sspSetup
uint32 g_u32JG0_reconnect_count=0;    //因JG0引起的重连计数

//uint32 JG1_time_T0TC[4]={0};
//uint32 JG1_time_counter[4]={0};
 uint32 jg1=0;
  extern uint32   g_au32Tempa[5000][4];
/***************JG0入口系统参数************************/
/* JG1起点        g_sspSetup.u16StartPtNum1; 
/* JG1止点	      g_sspSetup.u16EndPtNum1
/* JG1零点	      g_sspSetup.u16VerticalZeroPos1
/* JG1高度		  g_sspSetup.HeightLaser1
/* JG0/1距离	  g_sspSetup.LaserDistance
/**************************************************/
/********定义JG0数据读和处理记录变量***************/
uint8 	g_u8JG1_RBuff_Count=0; //两激光缓存计数；
uint8 	g_u8JG1_PBuff_Count=0; //两激光缓存计数；
uint8 	g_au8JG1_3Buff[3][831];  //激光0数据缓存3个；
int32   LMS_data_1[3][362]={0};
uint8   JG1_CurBuff[831]={0};

/**************************************************/
void Task_JG1(void *tdata)
{
	uint8 err;
	uint16  l_leftXpt, l_rightXpt;   //左右X距离对应的点数
	int32   l_32tmp2,l_tmp1,TempVaule1;
	uint32	i=0; 
	uint32	j=0; 
	uint16 Len_1;
	tdata=tdata; 
//	uint8 *l_JG0; //JG0数据指针
//	uint8 l_au8JG1temp[4][3]={0};	 //两激光帧匹配数组
//	uint32 l_u32JG1_err_count=0;
//初始化；
//	l_au8JG1temp[0][0] = 3;
//	l_au8JG1temp[0][1] = 2;
//	l_au8JG1temp[0][2] = 1;
//	l_au8JG1temp[1][0] = 0;
//	l_au8JG1temp[1][1] = 3;
//	l_au8JG1temp[1][2] = 2;
//	l_au8JG1temp[2][0] = 1;
//	l_au8JG1temp[2][1] = 0;
//	l_au8JG1temp[2][2] = 3;
//	l_au8JG1temp[3][0] = 2;
//	l_au8JG1temp[3][1] = 1;
//	l_au8JG1temp[3][2] = 0;
//	tdata=tdata;
//	Send_VehInfo_Uart1[9]=0x31;
//	Send_VehInfo_Uart1[15]=0x12;
//	Send_VehInfo_Uart1[16]=MONTH;
//	Send_VehInfo_Uart1[17]=DAY;
//	Send_VehInfo_Uart1[18]=HOUR;
//	Send_VehInfo_Uart1[19]=MIN;
//	Send_VehInfo_Uart1[20]=SEC;
//	Send_VehInfo_Uart1[21]=0x03;// 车型
//	{0xff,0x00,0x09,0x00,0x37,0x00,0x03,0x56,0x4c,0x58,0x00,0x00,0x00,0x00,0x00,0x12,0x05,0x14,0x13,0x20,0x05,0x02,0x00,
//	crc_create(Send_VehInfo_Uart1,52);
//	UART1_SendBuf(Send_VehInfo_Uart1,55); 
	while(1)
	{
		OSSemPend(g_JG1flag,10,&err);
		if(OS_NO_ERR == err)  //调用成功
		{
			S1_Data&=~S_RECEIVE;
			i = S_rx_process(1); 
		    if(i != 831)
		    {

		    }
		    else
		    {
		        jg1++;
			    memcpy(g_au8JG1_3Buff[g_u8JG1_RBuff_Count],Rx_Buffer+Max_Size,Max_Size);
				g_u8JG1_RBuff_Count=g_u8JG1_RBuff_Count+1;
				g_u8JG1_RBuff_Count=g_u8JG1_RBuff_Count%3;
			}
		}
		if(g_u8JG1_RBuff_Count!=g_u8JG1_PBuff_Count)
		{
			memcpy(JG1_CurBuff,g_au8JG1_3Buff[g_u8JG1_PBuff_Count],831);	
		 	Len_1= (JG1_CurBuff[83]<<8)+JG1_CurBuff[84];
			if(Len_1 == POINT_SUM)
			{ 
				for(i=85, j=0;i<807 && j < POINT_SUM; i=i+2,j++)	//20130426 修改，去掉偏移量	
				{
						LMS_data_1[g_u8JG1_PBuff_Count][j]=	(JG1_CurBuff[i]<<8)+JG1_CurBuff[i+1];	 //每点座标
						if(LMS_data_1[g_u8JG1_PBuff_Count][j] < 0)									 //每点座标
						LMS_data_1[g_u8JG1_PBuff_Count][j] = 0;										 //每点座标
				}	
			   LMS_data_1[g_u8JG1_PBuff_Count][361] = ((JG1_CurBuff[42]<<24)+(JG1_CurBuff[43]<<16)+(JG1_CurBuff[44]<<8)+JG1_CurBuff[45])/1000;
		       LMS_data_1[g_u8JG1_PBuff_Count][362]=t0_count2;	
			   OSSemPost(g_JG_Pro);	 //
			} 		
		}
 
	}
}
