/*************hongxy*************
2012.5.4
发送数据任务；
任务：Task_SendData
任务：Send08_Task
任务; Data_Acquisition_Task(void *Id)发送上位机要求的数据包


************************************/
#include "Send_Task.h"
#include "rd_data.h"
#include "wt_task.h"
#include "w5100.h"
#include "common.h"
//uint8  TimeNeedSendCnt;
//uint8  TimeSendSuccessCnt;
//
//void Task_SendData(void *tdata)
//{
////	uint8 Send_buf[1000];
////	uint8 err;
////	uint16 len;
////	uint8 ta[10]={1,2,3,4,5,6,7,8,9,0};
//	uint8 i;
//	FP64 Raddress;
//	FP64 ResendAddress;
//	uint8 sd_buf[512];
//	uint8 buf[400];
//	uint16 len;
//	uint16 tcplen;
//	uint8 ResendCnt;
//	uint8  err = 0;
//	tdata=tdata;//??
//
//	while(1)
//	{	OSTimeDly(10);
//		if(1)//if(!Transport_Way)判断传输方式；
//		{
//			while(Flag_NetConnect)
//			{
//				if(DataResendCnt)	//需要重发数据；
//				{
//					//sd_buf写
//					//OSSemPend(Flag_SendOK,800,&err);等待1000ms发送
//					//01包数据来自计算结果，存储在SD卡中；发送时从SD卡读出；
////					memcpy(Tcpadd1,sd_buf,tcplen);
////					SendData(Tcpadd1,tcplen);		//发送01数据包
//					if(err == OS_NO_ERR)				//01包已经发送成功
//					{
//						DataResendBgnNum++;
//						DataResendCnt--;
//					}
//
//				}
//				else
//				{
//				//存储区域需重发计数清零	
//				}
//
//				if(TimeNeedSendCnt > TimeSendSuccessCnt)
//				{
//					
//				}
//				else
//				{}
//			}
//		}
////		OSSemPend(g_sendtest,0,&err);
////		U5SendBytes(ta,16);		
////		OSTimeDly(2);
//		//发送数据至上位机；
//	}
//}
//
//
////==================================
//void Send08_Task(void *Id)
//{
//	uint8 err;
//	uint8 buf[100];
//	 Id=Id;
//	RD_Int08();
//	RD_Int02();
//	while(1)
//	{
//		if(Flag_NetConnect && Flag_08_NotReturn)//Flag_08_NotReturn在RD_DataPro(),和05格式数据包处理函数Change_Time（）中修改；
//		{
//				//OSSemPend(Flag_SendOK,0,&err);
//				if(err == OS_NO_ERR)				  		//发送成功
//				{
//					//更新08包时间
//					ASK08_data_Modify.ASK08_Modify.yearL 	= ASK08_data.ASK08.yearL = (YEAR+2000) & 0xFF;
//					ASK08_data_Modify.ASK08_Modify.yearH 	= ASK08_data.ASK08.yearH = ((YEAR+2000) >> 8) & 0xFF;
//					ASK08_data_Modify.ASK08_Modify.month 	= ASK08_data.ASK08.month = MONTH;
//					ASK08_data_Modify.ASK08_Modify.day 		= ASK08_data.ASK08.day   = DAY;
//					ASK08_data_Modify.ASK08_Modify.hour 	= ASK08_data.ASK08.hour  = HOUR; 
//					ASK08_data_Modify.ASK08_Modify.min 		= ASK08_data.ASK08.min   = MIN;
//					ASK08_data_Modify.ASK08_Modify.sec 		= ASK08_data.ASK08.sec   = SEC;
//					SendData( ASK08_data.ASK08_65,65);	  	//发送08包
//					Flag_08_NotReturn = 0;
//				}		
//			}
//			if(Flag_08_ReturnChangePara && Flag_NetConnect)//Flag_08_ReturnChangePara在03、04、06、07、0B格式数据包处理函数中改变；
//			{
//				//OSSemPend(Flag_SendOK,0,&err);
//				if(err == OS_NO_ERR)				  		//发送成功
//				{
//					//更新08包时间
//					ASK08_data_Modify.ASK08_Modify.yearL 	= ASK08_data.ASK08.yearL = (YEAR+2000) & 0xFF;
//					ASK08_data_Modify.ASK08_Modify.yearH 	= ASK08_data.ASK08.yearH = ((YEAR+2000)>> 8) & 0xFF;
//					ASK08_data_Modify.ASK08_Modify.month 	= ASK08_data.ASK08.month = MONTH;
//					ASK08_data_Modify.ASK08_Modify.day 		= ASK08_data.ASK08.day   = DAY;
//					ASK08_data_Modify.ASK08_Modify.hour 	= ASK08_data.ASK08.hour  = HOUR; 
//					ASK08_data_Modify.ASK08_Modify.min 		= ASK08_data.ASK08.min   = MIN;
//					ASK08_data_Modify.ASK08_Modify.sec 		= ASK08_data.ASK08.sec   = SEC;
//					SendData( ASK08_data_Modify.ASK08_Modify_65,65);//发送08包,未生效的参数
//					Flag_08_ReturnChangePara = 0;
//				}
//			}
//			OSTimeDly(10);
////			if((S2_Data & S_TRANSMITOK) == S_TRANSMITOK) //发送完成
////				{
////					S2_Data&=~S_TRANSMITOK;
////					memcpy(Tx_Buffer+Max_Size*2, S_08, 73);
////			        S_tx_process(2,73);	
////					Flag_08_NotReturn=0;				
////				}
//		OSTimeDly(10);
//
//	}
//
//}
////========================================
////发送上位机要求的数据包
//void Data_Acquisition_Task(void *Id)
//{
//	  while(1)
//	  {
//	  	 OSTimeDly(10);
//	  }
//}

