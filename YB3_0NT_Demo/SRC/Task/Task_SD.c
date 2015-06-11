/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			Task_SD.C
** Last modified Date:  20120718
** Last Version:		1.0
** Descriptions:		SD卡存储任务
**
**--------------------------------------------------------------------------------------------------------
** Created by:			Hong XiangYuan
** Created date:		20120718
** Version:				1.0
** Descriptions:SD卡：共0-0xECDFFF块；		
**
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
** Version:
** Descriptions:
**
*********************************************************************************************************/
#include "Task_SD.h"
#include "WT_Task.h"
#include "CMD.h"
#include "sdcommon.h"
#include "sddriver.h"
#include "Uart1.h"
#include "sdhal.h"
#include "Timer0.h"
#include "FW.h"
#include "rd_data.h"
#include "Task_SendUart1.h"
#include "TDC256.h"

#define  SETUPALIAS        g_sspSetup

uint32 g_u32SD_err_count=0;
uint32 g_u32SD_write_count=0;
//uint8 SD_test[1048]={0};
//uint32 test_MCIerr_count=0;
uint32 SD_time[4]={0};
uint8  l_u8SD_write_temp[512]={0};

//0x11包单车数据缓存
uint8 frame11_buf[57] = {0};

extern uint32	sv_write_sd_add;


uint8  SD_BLOCK_HEAD[4] = {0x30, 0x24, 0x85, 0x36};

OS_EVENT *FW_flag;	 //铁电的资源标志
OS_EVENT *SD_flag;	 //SD卡的资源标志
OS_EVENT *NET_flag;	 //网络的资源标志
OS_EVENT *W_R_FW_flag;	 //读写铁电的资源标志

V_TIME sv_time_lock;
uint32 gv_index=1;

STAT_SEND_INFO stat_send_fifo[100];
int stat_send_fifohead=0;
int stat_send_fifotail=0;

SV_SEND_INFO sv_send_fifo[100];
int sv_send_fifohead=0;
int sv_send_fifotail=0;

void Task_SD(void *tdata)
{
	uint32 stat;
//	uint32 i=0;
//	uint32 CMP_stat=0;
//	INT8U  err=0;

	uint32 SD_address=0;
	uint32 SD_Base_address;
	uint32 temp_SD_address=0;	//每次加1；
//	uint8  temp_uart[55]={0};
//	uint32 j=0;

	SD_Base_address = 0xf0000;

	tdata = tdata;
	if(0 == Flag_SD_Init_err)
	{
		SDCardInit();
		if(0 == Flag_SD_Init_err)
		{
			 SDCardInit();
		}
	}
//	crc_create(Send_VehInfo_Uart1,52);
//	U5SendBytes(Send_VehInfo_Uart1,55);
//	for(stat = 0;stat<20;stat ++)
//	{
//		FW_Read_test(0x120+stat* 4);	
//	}

	while(1)
	{	
/****************************************************************/		
/****************************************************************/		
		 if(Flag_NetConnect==1)
		 {
//		 SD_time[0] = T0TC;
//		 SD_time[1] = t0_count2;
//		 OSSemPend(g_SD_single_Veh,0,&err);	  //等待新车信号量
		 P3_OUTP_CLR = (1 << 9);	 //亮
		 OSTimeDly(1);
		 SD_address = SD_Base_address + temp_SD_address;
		 temp_SD_address = temp_SD_address +1;
//		 MCI_count = 0;
		 stat	= WriteSDCardBlock(SD_address,l_u8SD_write_temp);	  //DMA
		 g_u32SD_write_count = g_u32SD_write_count +1;		
		 P3_OUTP_SET = (1 << 9);	 //灭
//	     test_MCIerr_count = 0;
//		 SD_time[2] = T0TC;
//		 SD_time[3] = t0_count2;
		if(stat != 0)
		{	//写入不成功			  			  
//			g_u32SD_err_count = g_u32SD_err_count + 1;		 //yzb0906
//			BeepON();
//			Storage_Write_TDC256(SD_TEST_WRITE);
//			Delay(500);	
//			BeepOFF();
		}
		}
		OSTimeDly(60);
	}
}


uint8 full_read_sd(uint32 sd_add, uint8 *buf)   //存储SD卡，包含有独享SD卡的那些动作
{	
	uint8 err;
	uint8 stat;

	OSSemPend(SD_flag, 5000, &err);
	if (err == 0)
	{
		sic2IrqDisable(2);
		stat = ReadSDCardBlock(sd_add, buf);
		sic2IrqEnable(2);
		OSSemPost(SD_flag);

		if (stat==0)
		{
			return 0;
		}
		else
		{
			return 1;
		}
	}
	else
	{
		return 1;
	}
}

uint8 full_write_sd(uint32 sd_add, uint8 *buf)	    //取出SD卡，包含有独享SD卡的那些动作
{
	uint8 err;
	uint8 stat;
	
	OSSemPend(SD_flag,5000,&err);
	if(err==0)				
	{				
		sic2IrqDisable(2);
		stat = WriteSDCardBlock(sd_add,buf);		 
		sic2IrqEnable(2);
		OSSemPost(SD_flag);

		if (stat == 0)
		{
			return 0;
		}
		else
		{
			return 1;
		}
	}
	else
	{
		return 1;
	}	
}

/***************************************************************
//函数名:save11_to_sd
//功能说明:保存单车数据到SD卡中
// 返回值: 单车存Sd卡扇区号 
// add by mjh 2014-6-11
***************************************************************/
/*
uint32 save11_to_sd(uint8 *buf,int32 len)	//将11包单车数据存储到SD卡中
{
	 uint32 save_add;
	 uint8  save_buf[512] = {0};
	 uint8  add_buf[512] = {0};
	 uint16 len_temp;
	 uint16 day_temp;
	 uint16 year_temp;
	 uint32 time_num;	//数据序列号
	 V_TIME time_temp;
	 
	 int i;

	 len_temp = len;
	 year_temp = (buf[22] & 0xFF) + ((buf[23] & 0xFF) << 8) - 2013;
	 time_temp.year = year_temp;
	 time_temp.month = buf[24];
	 time_temp.day = buf[25];

	 day_temp = year_temp * 372 + (time_temp.month - 1) * 31 + time_temp.day;

	 time_num = (buf[31] & 0xFF) + ((buf[32] & 0xFF) << 8) + ((buf[33] & 0xFF) << 16);	//数据序列号

	 full_read_sd(SD_STAT1_ADD_START + day_temp,add_buf);
	 if((add_buf[0] == SD_BLOCK_HEAD[0]) && (add_buf[1] == SD_BLOCK_HEAD[1]) &&
	 		(add_buf[2] == SD_BLOCK_HEAD[2]) && (add_buf[3] == SD_BLOCK_HEAD[3]))	//SD卡头对了
	 	save_add = time_num + (add_buf[4] & 0xFF) + ((add_buf[5] & 0xFF) << 8) + ((add_buf[6] & 0xFF) << 16) + ((add_buf[7] & 0xFF) << 24);
	 else 
	 	return 0;

	 for (i = 0; i < 4; i++)
	 {
		save_buf[i] = SD_BLOCK_HEAD[i];
     }

	 if((buf == NULL) || (len > 512) || (len <0))
	 {
	 	return 0;
	 }
	 for (i = 0; i < len; i++)
	 {
		save_buf[4+i] = buf[i];
 	 }
	 
	 if (full_write_sd(save_add, save_buf))	 //将数据写入SD卡中	  没有写入成功
	 {
		BeepON();
		OSTimeDly(50);	
		BeepOFF();
	 }
	 return save_add; 
}	*/
uint32 save11_to_sd(uint8 *buf,int32 len)	//将11包单车数据存储到SD卡中
{
	 uint32 save_add;
	 uint8  save_buf[512] = {0};
	 uint8  add_buf[512] = {0};
	 uint8  last_add_buf[21]={0};
	 uint16 crc16;
	 uint16 len_temp;
	 uint16 day_temp;
	 uint16 year_temp;
	 uint32 time_num;	//数据序列号
	 V_TIME time_temp;
	 uint32 i;
	 if((buf == NULL) || (len > 512) || (len <0))
	 {
	 	return 0;
	 }
	 memcpy(last_add_buf,&buf[len-21],21);
	 len_temp = len;
	 year_temp = (last_add_buf[0] & 0xFF) + ((last_add_buf[1] & 0xFF) << 8) - 2013;
	 time_temp.year = year_temp;
	 time_temp.month = last_add_buf[2];
	 time_temp.day = last_add_buf[3];

	 day_temp = year_temp * 372 + (time_temp.month - 1) * 31 + time_temp.day;

	 time_num = (last_add_buf[9] & 0xFF) + ((last_add_buf[10] & 0xFF) << 8) + ((last_add_buf[11] & 0xFF) << 16);	//数据序列号

	 full_read_sd(SD_STAT1_ADD_START + day_temp,add_buf);
	 if((add_buf[0] == SD_BLOCK_HEAD[0]) && (add_buf[1] == SD_BLOCK_HEAD[1]) &&
	 		(add_buf[2] == SD_BLOCK_HEAD[2]) && (add_buf[3] == SD_BLOCK_HEAD[3]))	//SD卡头对了
	 	save_add = (time_num - 1)/24 + (add_buf[4] & 0xFF) + ((add_buf[5] & 0xFF) << 8) + ((add_buf[6] & 0xFF) << 16) + ((add_buf[7] & 0xFF) << 24);
	 else 
	 	return 0;

	 for (i = 0; i < 4; i++)
	 {
		save_buf[i] = SD_BLOCK_HEAD[i];
     }


	 for (i = 0; i < len; i++)
	 {
		save_buf[4+i] = buf[i];
 	 }
	 crc16 = CRC16(buf,len+4);
	 save_buf[len+4] = (crc16 & 0xFF);
	 save_buf[len+5] = ((crc16 >> 8) & 0xFF);
	 if (full_write_sd(save_add, save_buf))	 //将数据写入SD卡中	  没有写入成功
	 {
		BeepON();
		OSTimeDly(50);	
		BeepOFF();
	 }
	 else
	 {
	 	sv_write_sd_add =  save_add + 1;
		Write256_full(SVWRITESDADD,(uint8 *)&sv_write_sd_add,4);
	 }
	 return save_add; 
}
void save01_to_sd(uint8 *buf, int32 len)   //将01包存储到SD卡中
{
	V_TIME time_temp;
	int16  year_temp;
	uint16 time_num;
	uint32 save_add;
	uint8  save_buf[512];
	uint16 len_temp;

	int i;

	//20140217 增加
	if (buf == NULL || len > 512 || len < 0)
	{
		return;
	}

	len_temp = len;
	year_temp = buf[34] + (buf[35]<<8) - 2013;

	time_temp.year   = year_temp;   //一年按照372天计算
	time_temp.month  = buf[36];     //一个月按照31天计算
	time_temp.day    = buf[37];
	time_num         = buf[39] + (buf[40]<<8);   //时间序号，一天按照时间序号1-1440计算

	//20140217 增加对参数判断
	if (buf == NULL || len < 0 || len > 512)
	{
		return;
	}
	
	//取得写入的地址
	save_add = SD_STAT_START + ((time_temp.year * 372 * 288) + ((time_temp.month-1) * 31 * 288)
			 + ((time_temp.day-1) * 288) + time_num - 1) % SD_STAT_BLOCK_NUM;
	
	for (i = 0; i < 4; i++)
	{
		save_buf[i] = SD_BLOCK_HEAD[i];
	} 
	save_buf[4]  = time_temp.year;
	save_buf[5]  = time_temp.month;
	save_buf[6]  = time_temp.day;
	save_buf[7]  = time_num & 0xFF;
	save_buf[8]  = time_num>>8;
	save_buf[9]  = len_temp & 0xFF;
	save_buf[10] = len_temp>>8;
	save_buf[11] = 0;

	for (i = 0; i < len; i++)
	{
		save_buf[12+i] = buf[i];
	}							

	if (full_write_sd(save_add, save_buf))	 //将数据写入SD卡中	  没有写入成功
	{
		BeepON();
		OSTimeDly(50);	
		BeepOFF();
	} 
}

uint16 get01_from_sd(uint8 *buf,V_TIME *ptime,uint16 time_num)	 //将01包从SD卡中取出来,放到buf中，并返回长度
{
	V_TIME time_temp;
	uint32 get_add;
	uint8  get_buf[512];
	uint16 len;
	uint16 time_num_temp;
	int    i;

	time_temp = *ptime;
	//获取01包在SD卡中的地址
	get_add   = SD_STAT_START + ((time_temp.year * 372 * 288) + ((time_temp.month-1) * 31 * 288) 
	          + ((time_temp.day-1) * 288) + time_num -1 )%	SD_STAT_BLOCK_NUM;

	full_read_sd(get_add, get_buf);   //读取SD卡中的01包数据

	len = 0;
	if ( (get_buf[0] == SD_BLOCK_HEAD[0]) && (get_buf[1] == SD_BLOCK_HEAD[1]) &&
	     (get_buf[2] == SD_BLOCK_HEAD[2]) && (get_buf[3] == SD_BLOCK_HEAD[3]) )
	{
		time_num_temp = get_buf[7] + (get_buf[8]<<8);
		if( (get_buf[4] == time_temp.year) && (get_buf[5]==	time_temp.month) && 
			(get_buf[6] == time_temp.day)  && (time_num_temp = time_num) )
		{
			len = get_buf[9] + (get_buf[10]<<8);
			for (i = 0; i < len; i++)
			{
				buf[i] = get_buf[12+i];
			}
		}
	}

	return len;

}

//void add_to_net_send_stat(uint16 year, uint8 month, uint8 day, uint32 start_num, uint32 end_num)
//{
//	uint32 start_add;
//	uint32 end_add;
//
//	if( (stat_send_fifotail+1)%100 == stat_send_fifohead ) //FiFO满
//	{
//		return;
//	}
//
//	//存储到数组中
//	stat_send_fifo[stat_send_fifotail].time_info.year  = year - 2013;
//	stat_send_fifo[stat_send_fifotail].time_info.month = month;
//	stat_send_fifo[stat_send_fifotail].time_info.day   = day;
//
//	stat_send_fifo[stat_send_fifotail].start_num       = start_num;
//	stat_send_fifo[stat_send_fifotail].end_num         = end_num;
//	stat_send_fifotail = (stat_send_fifotail+1)%100;
//
//}

//构造一个新任务专门处理大量数据请求的网络输出,（若直接在响应请求时发送网络数据包，则会出现任务阻塞的情况）
//void Task_Net_Send(void *tdata)
//{
//	STAT_SEND_INFO *stat_send_info_buf;
//	SV_SEND_INFO *sv_send_info_buf;
//	int i;
//	V_TIME temp_time;
//	uint32 temp_time_num;
//	uint8 get_buf[512];
//	uint16 get_len;
////	uint32 s_minute_indexfrom2013,e_minute_indexfrom2013,temp_minute_indexfrom2013;
////	uint8 sv_tbuf[512];
//
//	tdata = tdata;
//
//	OSTimeDly(500);
//	while(1)
//	{
//		if(stat_send_fifotail != stat_send_fifohead)  //FIFO有数据
//		{
//			stat_send_info_buf = &stat_send_fifo[stat_send_fifohead];
//			stat_send_fifohead = (stat_send_fifohead+1)%100;
//			OSTimeDly(1);
//			temp_time_num = stat_send_info_buf->start_num;
//			temp_time     = stat_send_info_buf->time_info;
//
//			for(i = 0; i < 1440; i++)
//			{
//				if(temp_time_num <= stat_send_info_buf->end_num)
//				{
//					get_len = get01_from_sd(get_buf, &temp_time, temp_time_num);
//					if(get_len > 0)
//					{
//						Send1Data(get_buf, get_len);   //发送存储在SD卡中的01包数据
//					}
//					temp_time_num++;
//				}
//				else
//				{
//					break;
//				}
//				OSTimeDly(5);
//			}
//		}	
//		OSTimeDly(10);
//
//
//	}
//}
uint8 get_lane(uint8 olane) //车道映射
{
	uint8 ret;
	switch(olane)
	{
		case  1	:
			ret = 11;
			break;
		case  3 :
			ret = 12;
			break;
		case  2	:
			ret = 31;
			break;
		case  4	:
			ret = 32;
			break;
		default :
			ret = 11;
	}
	return ret;
}

/*
void gen_sv_fram(uint8 *buf)   //组帧 0x11包单车数据 共49个字节	不包括帧头、帧尾
{
	int i;
	uint16 year_temp;
	uint16 crc16;
	
	year_temp = sv_time_lock.year;

	frame11_buf[0] = 0xAA;
	frame11_buf[1] = 0xAA;

	frame11_buf[2] = 0x31;			//长度 小端
	frame11_buf[3] = 0x00;

	frame11_buf[4] = 0x11;
	for(i=0;i<16;i++)
	{
		frame11_buf[5+i] = RDid[i];
	}
	frame11_buf[21]=0;
	frame11_buf[22]=year_temp&0xFF;
	frame11_buf[23]=(year_temp>>8)&0xFF;
	frame11_buf[24]=sv_time_lock.month;
	frame11_buf[25]=sv_time_lock.day;
	frame11_buf[26]=sv_time_lock.hour;
	frame11_buf[27]=sv_time_lock.minute;
	frame11_buf[28]=sv_time_lock.second;
	frame11_buf[29]=0;
	frame11_buf[30]=0;
	frame11_buf[31]=gv_index&0xFF;
	frame11_buf[32]=(gv_index>>8)&0xFF;
	frame11_buf[33]=(gv_index>>16)&0xFF;
	frame11_buf[34] = get_lane(buf[38]); 
	frame11_buf[35] = buf[21];
	frame11_buf[36] = buf[9];

//	for(i=0;i<16;i++)
//		frame11_buf[37+i]=0;
	//车长
	frame11_buf[37] =  buf[7];
	frame11_buf[38] =  buf[8];
	//车宽
	frame11_buf[39] =  buf[43];
	frame11_buf[40] =  buf[44];
	//车高
	frame11_buf[41] =  buf[41];
	frame11_buf[42] =  buf[42];

	frame11_buf[43] = 0;
	frame11_buf[44] = 0;
	frame11_buf[45] = 0;
	frame11_buf[46] = 0;
	frame11_buf[47] = 0;
	frame11_buf[48] = 0;

	frame11_buf[49] = 0;
	frame11_buf[50] = 0;

	frame11_buf[51] = 0;
	frame11_buf[52] = 0;

	crc16 = crc_16(frame11_buf,53);
	frame11_buf[53] = (crc16 & 0xFF);
	frame11_buf[54] = ((crc16 >> 8) & 0xFF);

	frame11_buf[55] = 0xEE;
	frame11_buf[56] = 0xEE;
}
*/
void gen_sv_fram(uint8 *buf)   //组帧 0x11包单车数据 共49个字节	不包括帧头、帧尾
{
	int i;
	uint16 year_temp;
	uint16 crc16;
	
	year_temp = YEAR;

	frame11_buf[0] = 0xAA;
	frame11_buf[1] = 0xAA;

	frame11_buf[2] = 0x31;			//长度 小端
	frame11_buf[3] = 0x00;

	frame11_buf[4] = 0x11;
	for(i=0;i<16;i++)
	{
		frame11_buf[5+i] = RDid[i];
	}
	frame11_buf[21]=0;
	frame11_buf[22]=year_temp&0xFF;
	frame11_buf[23]=(year_temp>>8)&0xFF;
	frame11_buf[24]=MONTH;
	frame11_buf[25]=DAY;
	frame11_buf[26]=HOUR;
	frame11_buf[27]=MIN;
	frame11_buf[28]=SEC;
	frame11_buf[29]=0;
	frame11_buf[30]=0;
	frame11_buf[31]=gv_index&0xFF;
	frame11_buf[32]=(gv_index>>8)&0xFF;
	frame11_buf[33]=(gv_index>>16)&0xFF;
	frame11_buf[34] = get_lane(buf[38]); 
	frame11_buf[35] = buf[21];
	frame11_buf[36] = buf[9];

	//车长
	frame11_buf[37] =  buf[8];
	frame11_buf[38] =  buf[7];
	//车宽
	frame11_buf[39] =  buf[44];
	frame11_buf[40] =  buf[43];
	//车高
	frame11_buf[41] =  buf[42];
	frame11_buf[42] =  buf[41];

	frame11_buf[43] = 0;
	frame11_buf[44] = 0;
	frame11_buf[45] = 0;
	frame11_buf[46] = 0;
	frame11_buf[47] = 0;
	frame11_buf[48] = 0;

	frame11_buf[49] = 0;
	frame11_buf[50] = 0;

	frame11_buf[51] = 0;
	frame11_buf[52] = 0;

	crc16 = CRC16(&frame11_buf[2],51);
	frame11_buf[53] = (crc16 & 0xFF);
	frame11_buf[54] = ((crc16 >> 8) & 0xFF);

	frame11_buf[55] = 0xEE;
	frame11_buf[56] = 0xEE;

	sv_sd_frame[0]=YEAR&0xFF;
	sv_sd_frame[1]=(YEAR>>8)&0xFF;
	sv_sd_frame[2]=MONTH;
	sv_sd_frame[3]=DAY;
	sv_sd_frame[4]=HOUR;
	sv_sd_frame[5]=MIN;
	sv_sd_frame[6]=SEC;
	sv_sd_frame[7]=0;
	sv_sd_frame[8]=0;
	sv_sd_frame[9]=gv_index&0xFF;
	sv_sd_frame[10]=(gv_index>>8)&0xFF;
	sv_sd_frame[11]=(gv_index>>16)&0xFF;
	sv_sd_frame[12] = get_lane(buf[38]); 
	sv_sd_frame[13] = buf[21];
	sv_sd_frame[14] = buf[9];
	//车长
	sv_sd_frame[15] =  buf[8];
	sv_sd_frame[16] =  buf[7];
	//车宽
	sv_sd_frame[17] =  buf[44];
	sv_sd_frame[18] =  buf[43];
	//车高
	sv_sd_frame[19] =  buf[42];
	sv_sd_frame[20] =  buf[41];

}
uint8 Write256_full(uint16 p_u16Addr, uint8 * p_pu8WriteBuf, uint16 p_u16Len)
{
	uint8 err;
	uint8 ret; 
	OSSemPend(FW_flag,5000,&err);
	if(err==0)
	{
		ret = WriteC256(p_u16Addr,p_pu8WriteBuf,p_u16Len);
		OSSemPost(FW_flag);
	}
	return ret;
}
uint8 Read256_full(uint16 p_u16Addr, uint8 * p_pu8WriteBuf, uint16 p_u16Len)
{
	uint8 err;
	uint8 ret;
	OSSemPend(FW_flag,5000,&err);
	if(err==0)
	{
		ret = ReadC256(p_u16Addr,p_pu8WriteBuf,p_u16Len);
		OSSemPost(FW_flag);
	}
	return ret;
}










