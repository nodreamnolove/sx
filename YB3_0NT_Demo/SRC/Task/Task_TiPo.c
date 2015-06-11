/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			Task_TiPo.C
** Last modified Date:  20120718
** Last Version:		1.0
** Descriptions:		ʱ��ˢ������
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
#include "Task_Sv_Continue.h"

uint8 stat_tosend_flag = 0;
uint8 stat_tocontinue_flag = 0;	//������־����
extern uint32 head_VehSendInfo;
extern uint32 tail_VehSendInfo;
extern _VehSendInfo VehSendInfo[30];
extern uint32 CycleQue_Cnt_VehSendInfo;
extern unsigned char Que_VehSendInfo[32];
extern uint8 Stru_Is_Empty(_VehSendInfo *p_VehSendInfo);
void FlushVehSendInfo(V16_TIME*ptime)
{
	uint8 l_sec;
	uint8 year,month,day;
	uint32 save_add = 0;
	uint32 day_temp,time_num,year_temp;
	uint8 ReadFWtemp[12];
	uint8 WriteFWtemp[12];
	uint8 add_buf[512];
	_Cycle_Que_Continue cycle_que_conti;
	V_TIME time_temp;
	int i;
	for(i=0;i<30;i++)
	{
		if(1 == Stru_Is_Empty(&VehSendInfo[i]))
		{
			if(VehSendInfo[i].u16Year != ptime->year || VehSendInfo[i].u8Month != ptime->month || VehSendInfo[i].u8Day != ptime->day
					|| VehSendInfo[i].u8Hour != ptime->hour || VehSendInfo[i].u8Minute != ptime->minute)
			{
				l_sec = (60 - VehSendInfo[i].u8Second) + ptime->second;		
			}
			else
			{
				l_sec = ptime->second - VehSendInfo[i].u8Second;
			}	
			if(l_sec > 10)	//���ʱ������10s
			{
				if(CycleQue_Cnt_VehSendInfo == 30)	  //������
				{
					return;		
				}
				else
				{
					Que_VehSendInfo[tail_VehSendInfo] = i;
					tail_VehSendInfo = (tail_VehSendInfo+1)%30;
					CycleQue_Cnt_VehSendInfo++;
					 year_temp = VehSendInfo[i].u16Year - 2013;
					 time_temp.year = year_temp;
					 time_temp.month = VehSendInfo[i].u8Month;
					 time_temp.day = VehSendInfo[i].u8Day;
					 day_temp = year_temp * 372 + (time_temp.month - 1) * 31 + time_temp.day;
					 time_num = VehSendInfo[i].u32Veh_Index;	//�������к�
					 full_read_sd(SD_STAT1_ADD_START + day_temp,add_buf);
					 if((add_buf[0] == SD_BLOCK_HEAD[0]) && (add_buf[1] == SD_BLOCK_HEAD[1]) &&
					 		(add_buf[2] == SD_BLOCK_HEAD[2]) && (add_buf[3] == SD_BLOCK_HEAD[3]))	//SD��ͷ����
					 	save_add = (time_num-1)/24 + (add_buf[4] & 0xFF) + ((add_buf[5] & 0xFF) << 8) + ((add_buf[6] & 0xFF) << 16) + ((add_buf[7] & 0xFF) << 24);
					 else 
					 	return ;
					 if(save_add != 0)
					 {
					 	Set_Que_Cycle_Continue(save_add,time_num); 
					 }
				}
				memset((void*)&VehSendInfo[i],0,sizeof(_VehSendInfo));
			}
		}
	}	
}
void Task_TiPo(void *tdata)
{//ʱ��ˢ�����񣬰�����ι�����ɼ��¶ȡ����SD�����롢ÿСʱ��ʱ�����ȣ�
//	uint32	stat;
//	uint32 	i=0;
//	uint32	j=0;
//	uint8   name[11]="H1234.txt";
//	uint32  filesize=102400;
	uint8 l_u8temp=0;
	uint32 count =0;
	uint8 last_day;
	uint8 wtmp[4];
//	uint8 l_u8HOUR_Pre=0;
//	uint32 test_num[4] = {0};
//	uint8 Pre_HOUR=0;
	uint8 last_sec;
	V_TIME time_temp;
	V16_TIME time16_temp;

	tdata = tdata; 
	OSTimeDly(2);
	RTC8563Init();	//ʱ�����ã�
	GetRTCTime(&g_sstCurTime);
	YEAR 	=	g_sstCurTime.u16Year;	//��
	YEAR_uint8 = (YEAR-2000);
	MONTH	=	g_sstCurTime.u8Month;			//��
	DAY		=	g_sstCurTime.u8Day;			//��
	WEEK	=	g_sstCurTime.u8Week;
	HOUR	=	g_sstCurTime.u8Hour;			//ʱ
	MIN		=	g_sstCurTime.u8Minute;		//��
	SEC		=	g_sstCurTime.u8Second;		//��
	
	time_temp.year = YEAR - 2013;
	time_temp.month = g_sstCurTime.u8Month;
	time_temp.day = g_sstCurTime.u8Day;
	time_temp.hour = g_sstCurTime.u8Hour;
	time_temp.minute = g_sstCurTime.u8Minute;
	time_temp.second = g_sstCurTime.u8Second;
//	
	time16_temp.year = YEAR;
	time16_temp.month = g_sstCurTime.u8Month;
	time16_temp.day = g_sstCurTime.u8Day;
	time16_temp.hour = g_sstCurTime.u8Hour;
	time16_temp.minute = g_sstCurTime.u8Minute;
	time16_temp.second = g_sstCurTime.u8Second;
	Read256_full(PARA_ADD+0x100+100,wtmp,4);
	if(wtmp[0]!=g_sstCurTime.u8Day)			  //gv_index ����ÿһ��ĳ����������
	{										  //�ϵ�ʱ����
		gv_index=1;
		wtmp[0]=g_sstCurTime.u8Day;
		wtmp[1]=gv_index&0xFF;
		wtmp[2]=(gv_index>>8)&0xFF;
		wtmp[3]=(gv_index>>16)&0xFF;
		Write256_full(PARA_ADD+0x100+100,wtmp,4);
	}
	else
	{
		gv_index = wtmp[1] + (wtmp[2]<<8) + (wtmp[3]<<16) ;
		sv_count = (gv_index-1)%24;
	}
	last_day = DAY;	//��ʼ��last_day	����whileѭ������ĵ�һ��if��֧һ��������
	while(1)
	{
		count++;
		if(count>40)
		{	 
			count = 0; 		
		}
		GetRTCTime(&g_sstCurTime);
		YEAR 	=	g_sstCurTime.u16Year;	//��
		YEAR_uint8 = (YEAR-2000);
		MONTH	=	g_sstCurTime.u8Month;			//��
		DAY		=	g_sstCurTime.u8Day;			//��
		WEEK	=	g_sstCurTime.u8Week;
		HOUR	=	g_sstCurTime.u8Hour;			//ʱ
		MIN		=	g_sstCurTime.u8Minute;		//��
		SEC		=	g_sstCurTime.u8Second;		//��

		time_temp.year = YEAR -2013;
		time_temp.month = g_sstCurTime.u8Month;
		time_temp.day = g_sstCurTime.u8Day;
		time_temp.hour = g_sstCurTime.u8Hour;
		time_temp.minute = g_sstCurTime.u8Minute;
		time_temp.second = g_sstCurTime.u8Second;

		time16_temp.year = YEAR;
		time16_temp.month = g_sstCurTime.u8Month;
		time16_temp.day = g_sstCurTime.u8Day;
		time16_temp.hour = g_sstCurTime.u8Hour;
		time16_temp.minute = g_sstCurTime.u8Minute;
		time16_temp.second = g_sstCurTime.u8Second;
		if(last_day!=DAY)									  //gv_index ����ÿһ��ĳ����������
		{
			last_day = DAY;
			gv_index=1;
			wtmp[0]=DAY;
			wtmp[1]=gv_index&0xFF;
			wtmp[2]=(gv_index>>8)&0xFF;
			wtmp[3]=(gv_index>>16)&0xFF;
			Write256_full(PARA_ADD+0x100+100,wtmp,4);	
		}
		if(last_sec!=SEC)
		{
			last_sec=SEC;
			if(CycleQue_Cnt_VehSendInfo == 0)
			{
			}
			else
			{
				FlushVehSendInfo(&time16_temp);
			}
			if(SEC==59)		//ÿ��1min������������
			{
				stat_tocontinue_flag = 1;	//��λ������־����					
			}
			if((SEC==59)&&((MIN+1)%ProCycle==0))
			{
				stat_tosend_flag = 1;					
				//��ʱ����ͳ�ư�����ʱ�� 20131216
				TEMP_YEAR = YEAR;				//��
				TEMP_MONTH = MONTH;				//��
				TEMP_DAY = DAY;				//��
				TEMP_WEEK = WEEK;				//���ڼ�
				TEMP_HOUR = HOUR;				//ʱ
				TEMP_MIN = MIN;				//��
				TEMP_SEC = SEC;				//��
			}
		}

		if(l_u8temp == 0)
		{
			g_ai32Pre_Veh_Info_1_Lane[5]=0x12;  //����1����ǰһ��������Ϣ��
			g_ai32Pre_Veh_Info_1_Lane[6]=MONTH; 
			g_ai32Pre_Veh_Info_1_Lane[7]=DAY; 
			g_ai32Pre_Veh_Info_1_Lane[8]=HOUR; 
			g_ai32Pre_Veh_Info_1_Lane[9]=MIN; 
			g_ai32Pre_Veh_Info_1_Lane[10]=SEC;
			g_ai32Pre_Veh_Info_2_Lane[5]=0x12;  //����1����ǰһ��������Ϣ��
			g_ai32Pre_Veh_Info_2_Lane[6]=MONTH; 
			g_ai32Pre_Veh_Info_2_Lane[7]=DAY; 
			g_ai32Pre_Veh_Info_2_Lane[8]=HOUR; 
			g_ai32Pre_Veh_Info_2_Lane[9]=MIN; 
			g_ai32Pre_Veh_Info_2_Lane[10]=SEC;
			g_ai32Pre_Veh_Info_3_Lane[5]=0x12;  //����1����ǰһ��������Ϣ��
			g_ai32Pre_Veh_Info_3_Lane[6]=MONTH; 
			g_ai32Pre_Veh_Info_3_Lane[7]=DAY; 
			g_ai32Pre_Veh_Info_3_Lane[8]=HOUR; 
			g_ai32Pre_Veh_Info_3_Lane[9]=MIN; 
			g_ai32Pre_Veh_Info_3_Lane[10]=SEC;
			g_ai32Pre_Veh_Info_4_Lane[5]=0x12;  //����1����ǰһ��������Ϣ��
			g_ai32Pre_Veh_Info_4_Lane[6]=MONTH; 
			g_ai32Pre_Veh_Info_4_Lane[7]=DAY; 
			g_ai32Pre_Veh_Info_4_Lane[8]=HOUR; 
			g_ai32Pre_Veh_Info_4_Lane[9]=MIN; 
			g_ai32Pre_Veh_Info_4_Lane[10]=SEC; 
			g_ai32Pre_Veh_Info_5_Lane[5]=0x12;  //����1����ǰһ��������Ϣ��
			g_ai32Pre_Veh_Info_5_Lane[6]=MONTH; 
			g_ai32Pre_Veh_Info_5_Lane[7]=DAY; 
			g_ai32Pre_Veh_Info_5_Lane[8]=HOUR; 
			g_ai32Pre_Veh_Info_5_Lane[9]=MIN; 
			g_ai32Pre_Veh_Info_5_Lane[10]=SEC; 
			g_ai32Pre_Veh_Info_6_Lane[5]=0x12;  //����1����ǰһ��������Ϣ��
			g_ai32Pre_Veh_Info_6_Lane[6]=MONTH; 
			g_ai32Pre_Veh_Info_6_Lane[7]=DAY; 
			g_ai32Pre_Veh_Info_6_Lane[8]=HOUR; 
			g_ai32Pre_Veh_Info_6_Lane[9]=MIN; 
			g_ai32Pre_Veh_Info_6_Lane[10]=SEC; 
		}
		l_u8temp = 1;

		last_sec=SEC;

//		if(l_u8HOUR_Pre != HOUR)  //������
//		{
//			l_u8HOUR_Pre = HOUR;	
//			if(1 == Flag_Change_RD_Num)	 // ���޸�վ���ţ�
//			{
//				Flag_Change_RD_Num = 0;
//				memcpy(RDNum,NewRDNum,sizeof(NewRDNum));
//			}
//			if(1 == Flag_Change_DSC_Ip)	 ////IP�޸�
//			{
//				Flag_Change_DSC_Ip = 0;
//			}
//			if(1 == Flag_Change_InvContents)   //�޸ĵ�������
//			{
//				Flag_Change_InvContents = 0;
//			    InvContents = NewInvContents;
//			}
//			if(1 == Flag_Change_ProCycle)	  //(��������)
//			{
//				 Flag_Change_ProCycle = 0;
//				 ProCycle = NewProCycle;
//			}
//			if(1 == Flag_Change_DisTime)   //�����ٷֱȼ���ʱ��
//			{
//				 Flag_Change_DisTime = 0;
//				 DisTime = NewDisTime;
//			}
//		}
/***********************************************************************/
////		if(Pre_HOUR != HOUR)
//		{
//			BeepON();
////			Test_Record();
//			OSTimeDly(50);
//			BeepOFF();	
//		}
//		Pre_HOUR = HOUR;
		OSTimeDly(250);
	}
}

