/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			Task_Uart5.C
** Last modified Date:  20120718
** Last Version:		1.0
** Descriptions:		无线串口接收任务
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
#include "Task_Uart5.h"
#include "Uart5.h"
#include "Common.h"
#include "RD_data.h"
#include "WT_Task.h"


uint8 g_au8Uart5_buf[200];	 //串口数据接收缓存
//串口接收数据处理；
void Task_Uart5(void *tdata)
{	
//	uint8 Uart5_buf[2048];	 
//	uint8 Uart5_count=0;
	INT8U err	;

	uint16 len;
	uint8 wireless_Rec[200]={0};
	uint32 wire_Len=0;
	uint32 l_pro_Uart5_count=0;
	tdata=tdata;
	while(1)
	{		
	  while(1)
		{
			OSSemPend(g_Uart5_Rec,0,&err);
			OSTimeDly(30);
			g_Uart5_Rec->OSEventCnt = 0;
			wire_Len = 0;
			while(l_pro_Uart5_count != g_Uart5_Count)
			{
				 
				 l_pro_Uart5_count = l_pro_Uart5_count %2048;
				 
				 wireless_Rec[wire_Len++] = Rec_Buf[l_pro_Uart5_count];

				 l_pro_Uart5_count = l_pro_Uart5_count + 1;				
			}						
			OSTimeDly(20);
			if(wireless_Rec[0]==0xAA)
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
					if ((wireless_Rec[len+2+4]==0xEE)&&(wireless_Rec[len+2+5]==0xEE))
					{
					//	Rx_FTCdata_cnt++; 为什么？？
					// 数据处理	
					  	TcpData_Pro( wireless_Rec, wire_Len);
						if(1 == Flag_08_ReturnChangePara)
						{
							Update_data08();
							SendData(Send_data08,65);   //发送08包
							Flag_08_ReturnChangePara = 0;
						}

					}
				}
			}
		}  
	}
}
