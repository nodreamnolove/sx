/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			Task_TiPo.C
** Last modified Date:  20120718
** Last Version:		1.0
** Descriptions:		时间刷新任务
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

#include "Task_TiPo.h"
#include "WT_Task.h"
#include "common.h"	
#include "PCF8563.h"  
#include "rd_data.h"
#include "FW.h"
#include "TaskMatchSend.h"

uint8 stat_tosend_flag = 0;
uint8 stat_tocontinue_flag = 0;	//续传标志变量
extern uint32 head_VehSendInfo;
extern uint32 tail_VehSendInfo;
extern _VehSendInfo VehSendInfo[30];
extern uint32 CycleQue_Cnt_VehSendInfo;
extern unsigned char Que_VehSendInfo[32];
extern uint8 Stru_Is_Empty(_VehSendInfo *p_VehSendInfo);
//void FlushVehSendInfo(V16_TIME*ptime)
//{
//	uint8 l_sec;
//	uint8 year,month,day;
//	uint32 save_add = 0;
//	uint32 day_temp,time_num,year_temp;
//	uint8 ReadFWtemp[12];
//	uint8 WriteFWtemp[12];
//	uint8 add_buf[512];
//	_Cycle_Que_Continue cycle_que_conti;
//	V_TIME time_temp;
//	int i;
//	for(i=0;i<30;i++)
//	{
//		if(1 == Stru_Is_Empty(&VehSendInfo[i]))
//		{
//			if(VehSendInfo[i].u16Year != ptime->year || VehSendInfo[i].u8Month != ptime->month || VehSendInfo[i].u8Day != ptime->day
//					|| VehSendInfo[i].u8Hour != ptime->hour || VehSendInfo[i].u8Minute != ptime->minute)
//			{
//				l_sec = (60 - VehSendInfo[i].u8Second) + ptime->second;		
//			}
//			else
//			{
//				l_sec = ptime->second - VehSendInfo[i].u8Second;
//			}	
//			if(l_sec > 10)	//如果时间差大于10s
//			{
//				if(CycleQue_Cnt_VehSendInfo == 30)	  //队列满
//				{
//					return;		
//				}
//				else
//				{
//					Que_VehSendInfo[tail_VehSendInfo] = i;
//					tail_VehSendInfo = (tail_VehSendInfo+1)%30;
//					CycleQue_Cnt_VehSendInfo++;
//					 year_temp = VehSendInfo[i].u16Year - 2013;
//					 time_temp.year = year_temp;
//					 time_temp.month = VehSendInfo[i].u8Month;
//					 time_temp.day = VehSendInfo[i].u8Day;
//					 day_temp = year_temp * 372 + (time_temp.month - 1) * 31 + time_temp.day;
//					 time_num = VehSendInfo[i].u32Veh_Index;	//数据序列号
//					 full_read_sd(SD_STAT1_ADD_START + day_temp,add_buf);
//					 if((add_buf[0] == SD_BLOCK_HEAD[0]) && (add_buf[1] == SD_BLOCK_HEAD[1]) &&
//					 		(add_buf[2] == SD_BLOCK_HEAD[2]) && (add_buf[3] == SD_BLOCK_HEAD[3]))	//SD卡头对了
//					 	save_add = (time_num-1)/24 + (add_buf[4] & 0xFF) + ((add_buf[5] & 0xFF) << 8) + ((add_buf[6] & 0xFF) << 16) + ((add_buf[7] & 0xFF) << 24);
//					 else 
//					 	return ;
//					 if(save_add != 0)
//					 {
////					 	Set_Que_Cycle_Continue(save_add,time_num); 
//					 }
//				}
//				memset((void*)&VehSendInfo[i],0,sizeof(_VehSendInfo));
//			}
//		}
//	}	
//}
void Task_TiPo(void *tdata)
{
}

