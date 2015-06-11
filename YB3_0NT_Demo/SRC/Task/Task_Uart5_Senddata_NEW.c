					   /****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			Task_Uart5_Senddata_NEW.C
** Last modified Date:  20120814
** Last Version:		1.0
** Descriptions:		无线发送任务
**
**--------------------------------------------------------------------------------------------------------
** Created by:			Hong XiangYuan
** Created date:		20120814
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
#include "Task_Uart5_Senddata_NEW.h"
#include "WT_Task.h"
#include "RD_data.h"
#include "Common.h"
#include "rd_data.h"

//uint32 uart_send_count=0;
//uint32 OSRec0A_count = 0;
uint32 Send_01_NUM;//保存当前需要发送的01包NUM
uint32 Save_01_NUM;//保存已经产生的01包NUM;

void Send_01(void);
void Task_Uart5_Senddata_NEW(void *tdata)
{
	uint8 err;

    tdata =tdata;
	Update_data02(); 
    Update_data08();
	while(1)
	{
//		while(1)
//		{
//			OSTimeDly(5000);
//		}
		 //发送设备实时交通数据包：01数据包
		 //SendData();//	数据发送函数，可以自动添加帧头、帧尾、CRC校验、数据长度
		 while(g_u8Flag_wireless == 0)//无线传输
		 {
		 	  OSTimeDly(200);
			  Update_data02();
			  SendData( Send_data02,18);//发送02包；
			  EVENT_02Rev->OSEventCnt = 0;
			  OSSemPend(EVENT_02Rev,10000,&err);	// 等待02包返回。 20s;
			  if(err == OS_NO_ERR)
			  {
			  	   g_u8Flag_wireless = 1;
				   Update_data08();
				   SendData(Send_data08,65);   //发送08包
				   OSTimeDly(800);
			  }
			  else
			  {			  	
			  }
		 }
		 if(g_u8Flag_wireless == 1)
		 {
		 	 Send_01();
			 OSTimeDly(2000);
		 }
	}	
}
void Send_01(void)
{
	uint8 send_data[512];
	uint8 stat;
	uint8 err;
	uint16 data_len=0;
	uint8 return_value=0;
	uint32 SD_address_Send;
	
	 if(Send_01_NUM <= Save_01_NUM)
	 {
	 	 SD_address_Send = SD_Base_address_01 + Send_01_NUM;
		 stat = ReadSDCardBlock(SD_address_Send,send_data);
		 if(stat == 0)
		 {	   //读正确;
		 	  data_len = (send_data[510]<<8) + send_data[511];
			  //需要检查数据是否正确；
			  stat = SendData(send_data,data_len);	
			  if(stat == 1)
			  {
			  	   OSSemPend(EVENT_0ARev,10000,&err);
				   if(err == 0)//发送成功；
				   {
				   	   Send_01_NUM = Send_01_NUM + 1;
					   return_value = 1;
				   }
				   else
				   {
				   	   return_value = 0;
				   }
			  }
			  else
			  {
			  	  return_value = 0;
			  }		  
		 }
		 else
		 {	//读错误
		 	
		 }
	 }
	 else
	 {
	 	   return_value = 0;
	 }
	 
}
