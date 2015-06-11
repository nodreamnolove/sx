/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			Task_SendUart1.C
** Last modified Date:  20120721
** Last Version:		1.0
** Descriptions:		������������
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

/****************************UART1�������ⲿ��������********************************************/
extern unsigned char Que_VehSendInfo[30];
extern uint32 tail_VehSendInfo;
extern uint32 head_VehSendInfo;
extern _VehSendInfo VehSendInfo[30];
extern uint32 CycleQue_Cnt_VehSendInfo;
/****************************UART1�������ⲿ��������********************************************/

uint8 Send1Data(uint8 *p,uint16 len)
{
	uint8 data[1460];
	uint16 uchCRC;
	uint32 tmp_len;

	//20140217 ���ӶԲ����ж�
	if (p == NULL || len > 504 )
	{
		return 1;
	}
	//֡ͷ
	data[0] = 0xAA;				   
	data[1] = 0xAA;
	//֡��
	data[2] = len & 0xFF;
	data[3] = (len>>8) & 0xFF;
	memcpy(data+4, p, len);
	//CRCУ��
	uchCRC = CRC16(data+2,len+2);
	data[len+4]	= uchCRC & 0xFF;
	data[len+5]	= (uchCRC>>8) & 0xFF;
	//֡β
	data[len+6]	= 0xEE;
	data[len+7]	= 0xEE;
	memcpy(Send1DataBuff, data, len+8);
	tmp_len = len+8; //TCP���ݷ��ͳ���
	
//	if(!Transport_Way)			//Transport_Way;// ���ݴ��䷽ʽ 0���������紫�� 1���������紫��
	//��������
//	if(0)//===����
//	{
//		S_tx_process(0, TcpSend_len);  //ֱ�ӷ���ͨ��0�˿�
//	}
//	else
//	{
		UART1_SendBuf_full(Send1DataBuff,tmp_len);
//	}
	return 1;		
}

void SD_DataAdd_Save(uint8 *data)	   //�洢���ݵ�ַ
{
	uint8 i;
	uint8 ReadFWtemp[8];
	uint8 WriteFWtemp[8];
	uint32 head,tail;		  //����ͷ�����ӣ�����β����ӡ�
	uint16 year_temp;
	V_TIME 	time_temp;
	uint16 time_num;
	uint32 save_add;

	//20140217 ���ӶԲ������ж�
	if (data == NULL)
	{
		return;
	}

	Read256_full(BUF1ADDR,ReadFWtemp,8);	  //��ȡ���ƼĴ����е�����
	head=ReadFWtemp[0]+(ReadFWtemp[1]<<8)+(ReadFWtemp[2]<<16)+(ReadFWtemp[3]<<24);
	tail=ReadFWtemp[4]+(ReadFWtemp[5]<<8) + (ReadFWtemp[6]<<16) + (ReadFWtemp[7]<<24);

	year_temp= data[34] + (data[35]<<8) - 2013 ;

	time_temp.year = year_temp;	  //һ�갴��372�����
	time_temp.month = data[36];			  //һ���°���31�����
	time_temp.day = data[37];
	time_num = data[39] + (data[40]<<8);    //ʱ����ţ�һ�찴��ʱ�����1-1440����

	//ȡ��д��ĵ�ַ
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

	if((tail+1)%SD_STAT_ADD_NUM==head)	   //������
	{
		return;
	}
	else		              //����δ��
	{
		if (full_write_sd(tail+SD_STAT_ADD_START, WriteFWtemp))   //��ַд��SD��	û��д��ɹ�
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
		Write256_full(BUF1ADDR+0x04,WriteFWtemp,4);											 //����д�����ָ��
	}	
}

uint8 SD_DataAdd_Read(uint8 *data)
{
	uint8 ReadFWtemp[512];
	uint8 WriteFWtemp[8];
	uint32 head,tail;		  //����ͷ�����ӣ�����β����ӡ�
	uint32 read_add;
	int i;
	uint16 len;

	//20140217 ���ӶԲ������ж�
	if (data == NULL)
	{
		return 2;
	}

	Read256_full(BUF1ADDR,ReadFWtemp,8);	  //��ȡ���ƼĴ����е�����
	head=ReadFWtemp[0]+(ReadFWtemp[1]<<8) + (ReadFWtemp[2]<<16) + (ReadFWtemp[3]<<24);
	tail=ReadFWtemp[4]+(ReadFWtemp[5]<<8) + (ReadFWtemp[6]<<16) + (ReadFWtemp[7]<<24);

	if(tail==head)   //���п�
	{
		return 1;
	}
	else
	{
//		ReadC256(BUF1ADDR+0x200+head*4,ReadFWtemp,4);		//���Ӳ���
		if (full_read_sd(head+SD_STAT_ADD_START, ReadFWtemp))	//��SD���ж�ȡ���ݵ�ַ	  û�ж�ȡ�ɹ�
		{
			BeepON();
			OSTimeDly(50);	
			BeepOFF();
		}         
		if( (ReadFWtemp[0]==SD_BLOCK_HEAD[0])&&(ReadFWtemp[1]==SD_BLOCK_HEAD[1])&&
			(ReadFWtemp[2]==SD_BLOCK_HEAD[2])&&(ReadFWtemp[3]==SD_BLOCK_HEAD[3]) )   //blockͷ����
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
			(ReadFWtemp[2]==SD_BLOCK_HEAD[2])&&(ReadFWtemp[3]==SD_BLOCK_HEAD[3]) )   //blockͷ����
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
/****************************����VehSendInfo�ṹ������*****************************/
//
/****************************����VehSendInfo�ṹ������*****************************/

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
		if(0 == Stru_Is_Empty(&VehSendInfo[i]))	//�ṹ��Ϊ��	 	
		{
			
		}
		else if((VehSendInfo[i].u16Year == pstru->u16Year) && (VehSendInfo[i].u8Month == pstru->u8Month) && 
					(VehSendInfo[i].u8Day == pstru->u8Day) && (VehSendInfo[i].u32Veh_Index == pstru->u32VehIndex))
		{
			memset((void*)&VehSendInfo[i],0,sizeof(_VehSendInfo));
			if(CycleQue_Cnt_VehSendInfo == 30)	  //������
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
					if (0 == Check_CRCSum(&wireless_Rec[4], len))//У��ɹ�
					{
						 QualType = wireless_Rec[2];
						 JudgeQual(QualType, &wireless_Rec[4]);
					}
					else	 //У��ʧ��
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

	WDTIM_COUNTER	= 1;									/* ι��							*/

	if (SETUPALIAS.resetCnt == 1)
	{
		Write256_full(BUF1ADDR,wtemp,8);	//�����е�ͷβ���к�����FIFO
	}

	while(1)
	{
		OSTimeDly(5);
			
		 //�����豸ʵʱ��ͨ���ݰ���01���ݰ�
		 //SendData();//	���ݷ��ͺ����������Զ����֡ͷ��֡β��CRCУ�顢���ݳ���
//		 if(Flag_NetConnect == 1)
//		 {
		 	if(stat_tosend_flag==1)			 //5����
			{  
			   stat_tosend_flag=0;
			   
			   ReadLen = (uint16)(HSU1_LEVEL & 0xff);
               if(ReadLen ==0x40)
			   {			                                                                    /* ������������ж��д���       */          
		           for (i=0; i < ReadLen; i++) 
				   {
		            R_Buf[i] = (uint8)(HSU1_RX & 0xff);                       /* �������ݴ�����ջ�����       */
		           }
//				   HSU1_IIR = 0x00 ;
			   }
				//����쳣���δ��� 20140314
				if (Flag_NetConnect)//���ݲ���������������������
				{
					len = Save_data_01_process(); //��
				}
				
				//20140217 ����
				save01_to_sd(Send_data01_temp,len);//�洢01�����ݵ�SD��
				
				Update_data02();
			 	Send1Data( Send_data02,18);//����02����
			 	EVENT_02Rev->OSEventCnt = 0;
			 	OSSemPend(EVENT_02Rev,5000,&err);	// �ȴ�02�����ء� 10s;

			 	if(err == OS_NO_ERR)
			  	{
			  	   g_u8Flag_wireless = 1;
				   Update_data08();
				   Send1Data(Send_data08,65);   //����08��
				   OSTimeDly(800);
			  	}
				else
				{
					Update_data02();
			 		Send1Data( Send_data02,18);//����02����
			 		EVENT_02Rev->OSEventCnt = 0;
			 		OSSemPend(EVENT_02Rev,5000,&err);	// �ȴ�02�����ء� 10s;

			 		if(err == OS_NO_ERR)
			  		{
			  		   	g_u8Flag_wireless = 1;
				 		Update_data08();
					   	Send1Data(Send_data08,65);   //����08��
				   		OSTimeDly(800);
			  		}
					else
					{
						g_u8Flag_wireless = 0;	
					}
				}
				
				if(g_u8Flag_wireless == 0)
				{
					SD_DataAdd_Save(Send_data01_temp);		 //�洢��ǰͳ��֡
				}
				else
				{
					Send1Data(Send_data01_temp,len);
					EVENT_0ARev->OSEventCnt = 0;       //20130617 ��ֹ����ഫ���ؽ��ճ�������
					OSSemPend(EVENT_0ARev,5000,&err);

					if(err == OS_NO_ERR)
					{	
					 	l_flag_send01 = 1;//ȷ�Ϸ��ͳɹ���
					
					}
					else
					{
						Send1Data(Send_data01_temp,len);
						EVENT_0ARev->OSEventCnt = 0;       //20130617 ��ֹ����ഫ���ؽ��ճ�������
						OSSemPend(EVENT_0ARev,5000,&err);

						if(err == OS_NO_ERR)
						{	
					 		l_flag_send01 = 1;//ȷ�Ϸ��ͳɹ���
						}
						else
						{
						 	l_flag_send01 = 0;//���ξ�δ�ɹ���
						}
					}
					if(l_flag_send01==1)
					{
						while(0==SD_DataAdd_Read(Send_data01_temp))	 //ȡ����������	���أ�0����ȷȡ����1�������ѿգ�2����ʱ����Ӧ
						{
							Send1Data(Send_data01_temp,len);		//����
			  				uart1_send_count = uart1_send_count +1;
			  				//����01���ݰ���		  //���յ��ظ���0A���ݰ���
							EVENT_0ARev->OSEventCnt = 0;       //20130617 ��ֹ����ഫ���ؽ��ճ�������
			  				OSSemPend(EVENT_0ARev,5000,&err);	//�ȴ�0A�����ء�2500--5s; 10000--20s;

							if(err != OS_NO_ERR)  //û�з��ͳɹ�
							{
								SD_DataAdd_Save(Send_data01_temp); //���ȥ
								OSRec0A1_count = OSRec0A1_count + 1;
								break;
							}
						}	
					 }
					 else
					 {
					 	SD_DataAdd_Save(Send_data01_temp);		 //�����˵��Ƿ��ʹ洢��ǰͳ��֡	
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
/****�������ܣ����ڴ�SD���ж�ȡ����
/****����uint8* const data, uint16* const pDataLen, 
/****const uint16 u16PackageSeq  �����	 const uint8 u8TimeFlag ��ʶ��Ϣ
/****��� 0 ��ʾ��ȡ�ɹ���������ʾ��ȡʧ��
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
	//ȡ�ϸ���ͬ������ͬʱ�ε����� �ȼ����ַ
	read_add = GetData_SDAdd(u16PackageSeq, u8TimeFlag);
	if (!read_add)
	{	//û��ȡ�õ�ַ  �����쳣
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
		(ReadFWtemp[2]==SD_BLOCK_HEAD[2])&&(ReadFWtemp[3]==SD_BLOCK_HEAD[3]) )   //blockͷ����
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
/**** ProcDataPackage��SD�����Ĵ�����д���,�ı����������Ϣ
/**** ���� uint8* const pData, const uint16 u16PackageSeq, const uint16 u16DataLen
/****��� 0��ʾ�ɹ�������ʧ��
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

	//�޸����ݰ��в�������
	pData[34]   = TEMP_YEAR;
	pData[35]   = (TEMP_YEAR>>8);
	pData[36]   = TEMP_MONTH;
	pData[37]   = TEMP_DAY;
	//����� ��λ��ǰ
	pData[39] = u16PackageSeq&0xff;
	pData[40] = (u16PackageSeq>>8)&0xff;
	//�޸ĳ�����Ϣ
	l_u16DataLen = 42;
	for(l_u8I=0;l_u8I<lane_num;l_u8I++)
	{	
		// ��������ת���ɹ��공����
// 	  	frame[len++] = ChangetoBG_chedao_num(l_u8I);
		l_u16DataLen++;  
		
	
		//����������ݲ���
		l_u16DataLen += 4;

		//uint32 g_total_veh[4][9]={0}; 
		//�ų���������С�ͳ�	С����	��ͳ�	���ͻ���	���ͻ���	�ش��ͻ���	��װ�䳵	������	Ħ�г�
		//uint32 g_speed_veh_sum[4][9]={50,50,50,50,50,50,50,50,50};
		//��С�ͽ�ͨ��
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
/****GetData_SDAdd ���ݵ�ǰʱ�䣬�����ϸ���ͬ������ͬʱ�Σ����ݴ洢��SD���ϵĵ�ַ
/****���룺const uint16 u16PackageSeq ����� const uint8 u8TimeFlag ��ʶ��Ϣ
/****�����������SD���еĵ�ַ
/****************************************************************/
uint32 GetData_SDAdd(const uint16 u16PackageSeq, const uint8 u8TimeFlag)
{
	uint8  l_u8TimeFlag = u8TimeFlag;
	uint32 RetAdd = 0;
	V_TIME 	timeStrt;

	if (l_u8TimeFlag > 1 || u16PackageSeq > 288)  //�������Ϸ���
	{
		return 0;
	}
	timeStrt.year   = TEMP_YEAR - 2013;
	timeStrt.month  = TEMP_MONTH;
	timeStrt.day    = TEMP_DAY;
	//�жϵ�ǰ���Ƿ�������
	if (TEMP_YEAR % 400 == 0 || (TEMP_YEAR%4==0 && TEMP_YEAR%100!= 0))
	{ //����
		MONTH_DAY[1] = 29;  //�޸�2������ 		
	}
	if (!u8TimeFlag) //0 ��ʾ�ϸ�������ͬ��
	{
		if (timeStrt.day <= 7) //����С�ڻ��7
		{
			if (timeStrt.month <= 1) //�·�1
			{
				RetAdd = SD_STAT_START + (((timeStrt.year-1) * 372 * 288) + ((12-1) * 31 * 288)
					 + ((timeStrt.day+MONTH_DAY[11]-1-7) * 288) + u16PackageSeq - 1) % SD_STAT_BLOCK_NUM;
			}
			else
			{
			    //�·ݴ���1
				RetAdd = SD_STAT_START + ((timeStrt.year * 372 * 288) + ((timeStrt.month-1-1) * 31 * 288)
					     + ((timeStrt.day+MONTH_DAY[timeStrt.month-1]-1-7) * 288) + u16PackageSeq - 1) % SD_STAT_BLOCK_NUM;
			}
		}
		else
		{
			//���ڴ���7
			RetAdd = SD_STAT_START + ((timeStrt.year * 372 * 288) + ((timeStrt.month-1) * 31 * 288)
					 + ((timeStrt.day-1-7) * 288) + u16PackageSeq - 1) % SD_STAT_BLOCK_NUM;
		}
	}
	else //1��ʾǰһ��
	{
		if (u16PackageSeq <= 1) //��1��
		{
			if (timeStrt.day <= 1) //���ڲ�����1
			{
				if (timeStrt.month <= 1) //�·���1��
				{
					RetAdd = SD_STAT_START + (((timeStrt.year-1) * 372 * 288) + ((12-1) * 31 * 288)
							 + ((timeStrt.day+MONTH_DAY[11]-1-1) * 288) + 288-1) % SD_STAT_BLOCK_NUM;
				}
				else
				{
				 	//�·ݴ���1
					RetAdd = SD_STAT_START + ((timeStrt.year * 372 * 288) + ((timeStrt.month-1) * 31 * 288)
							 + ((timeStrt.day+MONTH_DAY[timeStrt.month-1]-1-1) * 288) + 288-1) % SD_STAT_BLOCK_NUM;
				}
			}
			else
			{
				//���ڴ���1
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