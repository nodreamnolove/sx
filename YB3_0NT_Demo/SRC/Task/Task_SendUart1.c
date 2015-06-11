/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			Task_SendUart1.C
** Last modified Date:  20120721
** Last Version:		1.0
** Descriptions:		网络连接任务
**
**--------------------------------------------------------------------------------------------------------
** Created by:			Hong XiangYuan
** Created date:		20120721
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
#include "Task_SendUart1.h"
#include "Uart1.h"
#include "Common.h"
#include "CMD.h"
#include "Timer0.h"
#include "RD_data.h"
#include "WT_Task.h"
#include "TDC256.h"
#include "W5100.h"
#include "Task_SD.h"
#include "Task_TiPo.h"


#define		SETUPALIAS				g_sspSetup
#define     MAXPACKETNUM            5000

uint32 Uart1_TOTC[4]={0};
uint8 Send1DataBuff[512]={0};
uint32 uart1_send_count=0;
uint32 OSRec0A1_count = 0;
extern uint8 stat_tosend_flag;

uint8  MONTH_DAY[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

/****************************UART1出单车外部变量声明********************************************/
extern unsigned char Que_VehSendInfo[30];
extern uint32 tail_VehSendInfo;
extern uint32 head_VehSendInfo;
extern _VehSendInfo VehSendInfo[30];
extern uint32 CycleQue_Cnt_VehSendInfo;
/****************************UART1出单车外部变量声明********************************************/

uint8 Send1Data(uint8 *p,uint16 len)
{
	uint8 data[1460];
	uint16 uchCRC;
	uint32 tmp_len;

	//20140217 增加对参数判断
	if (p == NULL || len > 504 )
	{
		return 1;
	}
	//帧头
	data[0] = 0xAA;				   
	data[1] = 0xAA;
	//帧长
	data[2] = len & 0xFF;
	data[3] = (len>>8) & 0xFF;
	memcpy(data+4, p, len);
	//CRC校验
	uchCRC = CRC16(data+2,len+2);
	data[len+4]	= uchCRC & 0xFF;
	data[len+5]	= (uchCRC>>8) & 0xFF;
	//帧尾
	data[len+6]	= 0xEE;
	data[len+7]	= 0xEE;
	memcpy(Send1DataBuff, data, len+8);
	tmp_len = len+8; //TCP数据发送长度
	
//	if(!Transport_Way)			//Transport_Way;// 数据传输方式 0：有线网络传输 1：无线网络传输
	//启动发送
//	if(0)//===待定
//	{
//		S_tx_process(0, TcpSend_len);  //直接发送通过0端口
//	}
//	else
//	{
		UART1_SendBuf_full(Send1DataBuff,tmp_len);
//	}
	return 1;		
}

void SD_DataAdd_Save(uint8 *data)	   //存储数据地址
{
	uint8 i;
	uint8 ReadFWtemp[8];
	uint8 WriteFWtemp[8];
	uint32 head,tail;		  //队列头：出队，队列尾：入队。
	uint16 year_temp;
	V_TIME 	time_temp;
	uint16 time_num;
	uint32 save_add;

	//20140217 增加对参数的判断
	if (data == NULL)
	{
		return;
	}

	Read256_full(BUF1ADDR,ReadFWtemp,8);	  //读取控制寄存器中的内容
	head=ReadFWtemp[0]+(ReadFWtemp[1]<<8)+(ReadFWtemp[2]<<16)+(ReadFWtemp[3]<<24);
	tail=ReadFWtemp[4]+(ReadFWtemp[5]<<8) + (ReadFWtemp[6]<<16) + (ReadFWtemp[7]<<24);

	year_temp= data[34] + (data[35]<<8) - 2013 ;

	time_temp.year = year_temp;	  //一年按照372天计算
	time_temp.month = data[36];			  //一个月按照31天计算
	time_temp.day = data[37];
	time_num = data[39] + (data[40]<<8);    //时间序号，一天按照时间序号1-1440计算

	//取得写入的地址
	save_add = SD_STAT_START + ((time_temp.year * 372 * 288) + ((time_temp.month-1) * 31 * 288) + 
				((time_temp.day-1) * 288) + time_num -1 )%	SD_STAT_BLOCK_NUM;

	for (i = 0; i < 4; i++)
	{
		WriteFWtemp[i] = SD_BLOCK_HEAD[i];
	} 
	WriteFWtemp[4] = save_add&0xFF;
	WriteFWtemp[5] = (save_add>>8)&0xFF;
	WriteFWtemp[6] = (save_add>>16)&0xFF;
	WriteFWtemp[7] = (save_add>>24)&0xFF;

	if((tail+1)%SD_STAT_ADD_NUM==head)	   //队列满
	{
		return;
	}
	else		              //队列未满
	{
		if (full_write_sd(tail+SD_STAT_ADD_START, WriteFWtemp))   //地址写入SD卡	没有写入成功
		{
			BeepON();
			OSTimeDly(50);	
			BeepOFF();
		}
		tail= (tail+1)%SD_STAT_ADD_NUM;
		WriteFWtemp[0]=tail&0xFF;
		WriteFWtemp[1]=(tail>>8)&0xFF;
		WriteFWtemp[2] = (tail>>16)&0xFF;
		WriteFWtemp[3] = (tail>>24)&0xFF;
		Write256_full(BUF1ADDR+0x04,WriteFWtemp,4);											 //索引写入队列指针
	}	
}

uint8 SD_DataAdd_Read(uint8 *data)
{
	uint8 ReadFWtemp[512];
	uint8 WriteFWtemp[8];
	uint32 head,tail;		  //队列头：出队，队列尾：入队。
	uint32 read_add;
	int i;
	uint16 len;

	//20140217 增加对参数的判断
	if (data == NULL)
	{
		return 2;
	}

	Read256_full(BUF1ADDR,ReadFWtemp,8);	  //读取控制寄存器中的内容
	head=ReadFWtemp[0]+(ReadFWtemp[1]<<8) + (ReadFWtemp[2]<<16) + (ReadFWtemp[3]<<24);
	tail=ReadFWtemp[4]+(ReadFWtemp[5]<<8) + (ReadFWtemp[6]<<16) + (ReadFWtemp[7]<<24);

	if(tail==head)   //队列空
	{
		return 1;
	}
	else
	{
//		ReadC256(BUF1ADDR+0x200+head*4,ReadFWtemp,4);		//出队操作
		if (full_read_sd(head+SD_STAT_ADD_START, ReadFWtemp))	//从SD卡中读取数据地址	  没有读取成功
		{
			BeepON();
			OSTimeDly(50);	
			BeepOFF();
		}         
		if( (ReadFWtemp[0]==SD_BLOCK_HEAD[0])&&(ReadFWtemp[1]==SD_BLOCK_HEAD[1])&&
			(ReadFWtemp[2]==SD_BLOCK_HEAD[2])&&(ReadFWtemp[3]==SD_BLOCK_HEAD[3]) )   //block头对了
		{
			read_add = ReadFWtemp[4]+(ReadFWtemp[5]<<8)+(ReadFWtemp[6]<<16)+(ReadFWtemp[7]<<24);
		}
		head=(head+1)%SD_STAT_ADD_NUM;
		WriteFWtemp[0]=head&0xFF;
		WriteFWtemp[1]=(head>>8)&0xFF;
		WriteFWtemp[2]=(head>>16)&0xFF;
		WriteFWtemp[3]=(head>>24)&0xFF;
		Write256_full(BUF1ADDR,WriteFWtemp,4);

		if (full_read_sd(read_add,ReadFWtemp))
		{
			BeepON();
			OSTimeDly(50);	
			BeepOFF();
		}
		len=0;															
		if( (ReadFWtemp[0]==SD_BLOCK_HEAD[0])&&(ReadFWtemp[1]==SD_BLOCK_HEAD[1])&&
			(ReadFWtemp[2]==SD_BLOCK_HEAD[2])&&(ReadFWtemp[3]==SD_BLOCK_HEAD[3]) )   //block头对了
		{
			len = ReadFWtemp[9] + (ReadFWtemp[10]<<8);
			for(i=0;i<len;i++)
			{
				data[i]=ReadFWtemp[12+i];
			}
			return 0;
		}
		else
		{
			return 2;
		}

	}
}
/****************************更新VehSendInfo结构体数组*****************************/
//
/****************************更新VehSendInfo结构体数组*****************************/

uint8 Stru_Is_Empty(_VehSendInfo *p_VehSendInfo)
{
	 if((p_VehSendInfo->u16Year == 0) && (p_VehSendInfo->u8Month == 0) && (p_VehSendInfo->u8Day == 0) &&
	 		(p_VehSendInfo->u8Hour == 0) && (p_VehSendInfo->u8Minute == 0) && (p_VehSendInfo->u8Second == 0) &&
				(p_VehSendInfo->u32Veh_Index == 0))
		return 0;
	else
		return 1;
}
void RenewVehSendInfo(struct _arg_RenewVehSendInfo*pstru)
{
	int i;

	for(i=0;i<30;i++)
	{
		if(0 == Stru_Is_Empty(&VehSendInfo[i]))	//结构体为空	 	
		{
			
		}
		else if((VehSendInfo[i].u16Year == pstru->u16Year) && (VehSendInfo[i].u8Month == pstru->u8Month) && 
					(VehSendInfo[i].u8Day == pstru->u8Day) && (VehSendInfo[i].u32Veh_Index == pstru->u32VehIndex))
		{
			memset((void*)&VehSendInfo[i],0,sizeof(_VehSendInfo));
			if(CycleQue_Cnt_VehSendInfo == 30)	  //队列满
			{
				return;		
			}
			else
			{
				Que_VehSendInfo[tail_VehSendInfo] = i;
				tail_VehSendInfo = (tail_VehSendInfo+1)%30;
				CycleQue_Cnt_VehSendInfo++;
			}

		}
	}	
}
uint16 count_rec = 0;
void 	Task_SendUart1(void *tdata)
{	
	INT8U err;

	uint16 len,uchCRC;
	uint8 wireless_Rec[2048]={0},data[500]={0},uart1_rec[20]={0};
	uint8 div_uart1_data = 0,i = 0;
	uint32 l_pro_Uart5_count=0;
	uint8	QualType = 0,test11[10]={1,1,1,1,2,2,2,2,3,3};
	uint32 l_u32datacount=0;
	struct _arg_RenewVehSendInfo arg_RenewVehSendInfo;

	tdata=tdata;
		
	  while(1)
		{
			OSSemPend(g_Uart1_send,0,&err);	 		  
			g_Uart1_send->OSEventCnt = 0;
			len = 0;	
	//		l_pro_Uart5_count=0;
			l_u32datacount = g_UART1Cnt;
			l_u32datacount = l_u32datacount%2048;	

			memcpy(wireless_Rec,Rcv_Buf,l_u32datacount);
//			U5SendBytes(test11,10);
//			U5SendBytes(wireless_Rec,l_u32datacount);
//			U5SendBytes(test11,10);						

			if(wireless_Rec[0]==0xFF)						  // zyj
			{
				if(wireless_Rec[1]==0xAA)
				{
					len = wireless_Rec[3];
					if (0 == Check_CRCSum(&wireless_Rec[4], len))//校验成功
					{
						 QualType = wireless_Rec[2];
						 JudgeQual(QualType, &wireless_Rec[4]);
					}
					else	 //校验失败
					{
					
					}
				}

			}
			else if(wireless_Rec[0]==0xAA)
			{
			 	if(wireless_Rec[1]==0xAA)
				{
					len = (wireless_Rec[3]<<8) + wireless_Rec[2];
				}
				if(len>=200)
				{
					continue;
				}
				else
				{
					TcpData_Pro( wireless_Rec, l_u32datacount);
				}

			}  //end else if
			
		} //end while 	
}

void Task_Uart1_Senddata(void *tdata)
{
	uint8 err;
	uint8 l_flag_send01=0;
	uint8 wtemp[8]={0};
	uint16 ReadLen=0;
	uint8 i=0;
	uint16 len=0;
	uint8 R_Buf[64]={0};
	uint16  l_u16PackageSeq = 0;

    tdata =tdata;
	Update_data02(); 
    Update_data08();

	WDTIM_COUNTER	= 1;									/* 喂狗							*/

	if (SETUPALIAS.resetCnt == 1)
	{
		Write256_full(BUF1ADDR,wtemp,8);	//铁电中的头尾队列号清零FIFO
	}

	while(1)
	{
		OSTimeDly(5);
			
		 //发送设备实时交通数据包：01数据包
		 //SendData();//	数据发送函数，可以自动添加帧头、帧尾、CRC校验、数据长度
//		 if(Flag_NetConnect == 1)
//		 {
		 	if(stat_tosend_flag==1)			 //5分钟
			{  
			   stat_tosend_flag=0;
			   
			   ReadLen = (uint16)(HSU1_LEVEL & 0xff);
               if(ReadLen ==0x40)
			   {			                                                                    /* 保存的数据在中断中处理       */          
		           for (i=0; i < ReadLen; i++) 
				   {
		            R_Buf[i] = (uint8)(HSU1_RX & 0xff);                       /* 接收数据存入接收缓冲区       */
		           }
//				   HSU1_IIR = 0x00 ;
			   }
				//添加异常波形处理 20140314
				if (Flag_NetConnect)//数据波形正常且网络连接正常
				{
					len = Save_data_01_process(); //存
				}
				
				//20140217 增加
				save01_to_sd(Send_data01_temp,len);//存储01包数据到SD卡
				
				Update_data02();
			 	Send1Data( Send_data02,18);//发送02包；
			 	EVENT_02Rev->OSEventCnt = 0;
			 	OSSemPend(EVENT_02Rev,5000,&err);	// 等待02包返回。 10s;

			 	if(err == OS_NO_ERR)
			  	{
			  	   g_u8Flag_wireless = 1;
				   Update_data08();
				   Send1Data(Send_data08,65);   //发送08包
				   OSTimeDly(800);
			  	}
				else
				{
					Update_data02();
			 		Send1Data( Send_data02,18);//发送02包；
			 		EVENT_02Rev->OSEventCnt = 0;
			 		OSSemPend(EVENT_02Rev,5000,&err);	// 等待02包返回。 10s;

			 		if(err == OS_NO_ERR)
			  		{
			  		   	g_u8Flag_wireless = 1;
				 		Update_data08();
					   	Send1Data(Send_data08,65);   //发送08包
				   		OSTimeDly(800);
			  		}
					else
					{
						g_u8Flag_wireless = 0;	
					}
				}
				
				if(g_u8Flag_wireless == 0)
				{
					SD_DataAdd_Save(Send_data01_temp);		 //存储当前统计帧
				}
				else
				{
					Send1Data(Send_data01_temp,len);
					EVENT_0ARev->OSEventCnt = 0;       //20130617 防止单点多传多重接收出现问题
					OSSemPend(EVENT_0ARev,5000,&err);

					if(err == OS_NO_ERR)
					{	
					 	l_flag_send01 = 1;//确认发送成功；
					
					}
					else
					{
						Send1Data(Send_data01_temp,len);
						EVENT_0ARev->OSEventCnt = 0;       //20130617 防止单点多传多重接收出现问题
						OSSemPend(EVENT_0ARev,5000,&err);

						if(err == OS_NO_ERR)
						{	
					 		l_flag_send01 = 1;//确认发送成功；
						}
						else
						{
						 	l_flag_send01 = 0;//两次均未成功；
						}
					}
					if(l_flag_send01==1)
					{
						while(0==SD_DataAdd_Read(Send_data01_temp))	 //取出铁电数据	返回：0：正确取出，1：铁电已空，2：超时无响应
						{
							Send1Data(Send_data01_temp,len);		//发送
			  				uart1_send_count = uart1_send_count +1;
			  				//发送01数据包；		  //会收到回复的0A数据包。
							EVENT_0ARev->OSEventCnt = 0;       //20130617 防止单点多传多重接收出现问题
			  				OSSemPend(EVENT_0ARev,5000,&err);	//等待0A包返回。2500--5s; 10000--20s;

							if(err != OS_NO_ERR)  //没有发送成功
							{
								SD_DataAdd_Save(Send_data01_temp); //存回去
								OSRec0A1_count = OSRec0A1_count + 1;
								break;
							}
						}	
					 }
					 else
					 {
					 	SD_DataAdd_Save(Send_data01_temp);		 //连上了但是发送存储当前统计帧	
					 }
				}	
				
			}
			else 
			{
				//new char;
				OSTimeDly(5);  //10s
			}
//		}
	}
}
// 	   INT8U err;
//		   
//	   tdata = tdata;	   
//
//	   while(1)
//	   {
//	 	  	OSSemPend(g_Uart1_send,0,&err);
//			if(g_Uart1_send->OSEventCnt>0)
//			{
//				g_Uart1_send->OSEventCnt = 0;
//			}
//			Uart1_TOTC[0] = T0TC;
//			Uart1_TOTC[2] = t0_count2;
//			UART1_SendBuf(Send_VehInfo_Uart1,55);	   //TOTC  = 32
//			Uart1_TOTC[3] = t0_count2;
//			Uart1_TOTC[1] = T0TC;
//			OSTimeDly(10);
//	   }
/****************************************************
/**** 20140314
/****函数功能：用于从SD卡中读取数据
/****输入uint8* const data, uint16* const pDataLen, 
/****const uint16 u16PackageSeq  包序号	 const uint8 u8TimeFlag 标识信息
/****输出 0 表示读取成功，其他表示读取失败
/****************************************************/
uint8 ReadData_FromSD(uint8* const data, uint16* const pDataLen, const uint16 u16PackageSeq, const uint8 u8TimeFlag)
{
	uint8 ReadFWtemp[512]; 	
	uint16 len;
	uint32 read_add;
	int i;

	if (data == NULL || pDataLen == NULL)
	{
		return 2;
	}
	//取上个相同星期相同时段的数据 先计算地址
	read_add = GetData_SDAdd(u16PackageSeq, u8TimeFlag);
	if (!read_add)
	{	//没有取得地址  返回异常
		return 2;
	}
	if (full_read_sd(read_add,ReadFWtemp))
	{
		BeepON();
		OSTimeDly(50);	
		BeepOFF();
	}
	len=0;															
	if( (ReadFWtemp[0]==SD_BLOCK_HEAD[0])&&(ReadFWtemp[1]==SD_BLOCK_HEAD[1])&&
		(ReadFWtemp[2]==SD_BLOCK_HEAD[2])&&(ReadFWtemp[3]==SD_BLOCK_HEAD[3]) )   //block头对了
	{
		len = ReadFWtemp[9] + (ReadFWtemp[10]<<8);
		if (len >= (500-12))
		{
			return 2;
		} 
		for(i=0;i<len;i++)
		{
			data[i]=ReadFWtemp[12+i];
		}
		*pDataLen = len;
		return 0;
	}
	else
	{
		return 1;
	}

}

/****************************************************************
/**** 20140314
/**** ProcDataPackage从SD读出的处理进行处理,改变包的数据信息
/**** 输入 uint8* const pData, const uint16 u16PackageSeq, const uint16 u16DataLen
/****输出 0表示成功，其他失败
/***************************************************************/
uint8  ProcDataPackage(uint8* const pData, const uint16 u16PackageSeq, const uint16 u16DataLen)
{
	uint8  l_u8I        = 0;
	uint8  l_u8J        = 0;
	uint8  lane_num     = 0;
	uint16 l_u16DataLen = 0;
	uint32 l_u32Temp    = 0;

	if (pData == NULL || u16DataLen > 500 || u16PackageSeq > 288 || g_sspSetup.u8LaneNum > 6)
		return 1;


	lane_num = g_sspSetup.u8LaneNum;

	//修改数据包中部分数据
	pData[34]   = TEMP_YEAR;
	pData[35]   = (TEMP_YEAR>>8);
	pData[36]   = TEMP_MONTH;
	pData[37]   = TEMP_DAY;
	//包序号 低位在前
	pData[39] = u16PackageSeq&0xff;
	pData[40] = (u16PackageSeq>>8)&0xff;
	//修改车辆信息
	l_u16DataLen = 42;
	for(l_u8I=0;l_u8I<lane_num;l_u8I++)
	{	
		// 将车道号转换成国标车道号
// 	  	frame[len++] = ChangetoBG_chedao_num(l_u8I);
		l_u16DataLen++;  
		
	
		//其它情况数据不变
		l_u16DataLen += 4;

		//uint32 g_total_veh[4][9]={0}; 
		//九车型总数；小客车	小货车	大客车	中型货车	大型货车	特大型货车	集装箱车	拖拉机	摩托车
		//uint32 g_speed_veh_sum[4][9]={50,50,50,50,50,50,50,50,50};
		//中小客交通量
		for(l_u8J=0;l_u8J<9;l_u8J++)
		{
			l_u32Temp    = (pData[l_u16DataLen] + (pData[l_u16DataLen+1]<<8));
			pData[l_u16DataLen++] = l_u32Temp & 0xff;
			pData[l_u16DataLen++] = (l_u32Temp>>8) & 0xff;

			l_u16DataLen++;
		} //9*3=27
	}

	if (u16DataLen == l_u16DataLen)
		return 0;
	else
		return 1;
	
}

/******************************************************************
/****  20140314
/****GetData_SDAdd 根据当前时间，计算上个相同星期相同时段，数据存储到SD卡上的地址
/****输入：const uint16 u16PackageSeq 包序号 const uint8 u8TimeFlag 标识信息
/****输出：数据在SD卡中的地址
/****************************************************************/
uint32 GetData_SDAdd(const uint16 u16PackageSeq, const uint8 u8TimeFlag)
{
	uint8  l_u8TimeFlag = u8TimeFlag;
	uint32 RetAdd = 0;
	V_TIME 	timeStrt;

	if (l_u8TimeFlag > 1 || u16PackageSeq > 288)  //检查参数合法性
	{
		return 0;
	}
	timeStrt.year   = TEMP_YEAR - 2013;
	timeStrt.month  = TEMP_MONTH;
	timeStrt.day    = TEMP_DAY;
	//判断当前年是否是润年
	if (TEMP_YEAR % 400 == 0 || (TEMP_YEAR%4==0 && TEMP_YEAR%100!= 0))
	{ //润年
		MONTH_DAY[1] = 29;  //修改2月天数 		
	}
	if (!u8TimeFlag) //0 表示上个星期相同包
	{
		if (timeStrt.day <= 7) //日期小于或等7
		{
			if (timeStrt.month <= 1) //月份1
			{
				RetAdd = SD_STAT_START + (((timeStrt.year-1) * 372 * 288) + ((12-1) * 31 * 288)
					 + ((timeStrt.day+MONTH_DAY[11]-1-7) * 288) + u16PackageSeq - 1) % SD_STAT_BLOCK_NUM;
			}
			else
			{
			    //月份大于1
				RetAdd = SD_STAT_START + ((timeStrt.year * 372 * 288) + ((timeStrt.month-1-1) * 31 * 288)
					     + ((timeStrt.day+MONTH_DAY[timeStrt.month-1]-1-7) * 288) + u16PackageSeq - 1) % SD_STAT_BLOCK_NUM;
			}
		}
		else
		{
			//日期大于7
			RetAdd = SD_STAT_START + ((timeStrt.year * 372 * 288) + ((timeStrt.month-1) * 31 * 288)
					 + ((timeStrt.day-1-7) * 288) + u16PackageSeq - 1) % SD_STAT_BLOCK_NUM;
		}
	}
	else //1表示前一包
	{
		if (u16PackageSeq <= 1) //第1包
		{
			if (timeStrt.day <= 1) //日期不大于1
			{
				if (timeStrt.month <= 1) //月份是1月
				{
					RetAdd = SD_STAT_START + (((timeStrt.year-1) * 372 * 288) + ((12-1) * 31 * 288)
							 + ((timeStrt.day+MONTH_DAY[11]-1-1) * 288) + 288-1) % SD_STAT_BLOCK_NUM;
				}
				else
				{
				 	//月份大于1
					RetAdd = SD_STAT_START + ((timeStrt.year * 372 * 288) + ((timeStrt.month-1) * 31 * 288)
							 + ((timeStrt.day+MONTH_DAY[timeStrt.month-1]-1-1) * 288) + 288-1) % SD_STAT_BLOCK_NUM;
				}
			}
			else
			{
				//日期大于1
				RetAdd = SD_STAT_START + ((timeStrt.year * 372 * 288) + ((timeStrt.month-1) * 31 * 288)
						 + ((timeStrt.day-1-1) * 288) + 288-1) % SD_STAT_BLOCK_NUM;
			}
		}
		else
		{
			RetAdd = SD_STAT_START + ((timeStrt.year * 372 * 288) + ((timeStrt.month-1) * 31 * 288)
					 + ((timeStrt.day-1) * 288) + u16PackageSeq - 2) % SD_STAT_BLOCK_NUM;
		}
	}

   	return RetAdd;
}   