/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			Task_Uart5_Senddata.C
** Last modified Date:  20120718
** Last Version:		1.0
** Descriptions:		无线发送任务
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
#include "Task_Uart5_Senddata.h"
#include "WT_Task.h"
#include "RD_data.h"
#include "Common.h"
#include "TDC256.h"

uint32 uart_send_count=0;
uint32 OSRec0A_count = 0;
void FW_fram_save(uint8 *data);
uint8 FW_fram_read(uint8 *data);

void Task_Uart5_Senddata(void *tdata)
{
	uint8 err;
	uint8 l_flag_send01=0;
	uint8 wtemp[2]={0,0};

    tdata =tdata;
	Update_data02(); 
    Update_data08();

	WDTIM_COUNTER	= 1;									/* 喂狗							*/

	WriteC256(BUF1ADDR,wtemp,2);	//清零FIFO

	while(1)
	{
		WDTIM_COUNTER	= 1;									/* 喂狗							*/
		OSTimeDly(5000);
		WDTIM_COUNTER	= 1;									/* 喂狗							*/
			
		 //发送设备实时交通数据包：01数据包
		 //SendData();//	数据发送函数，可以自动添加帧头、帧尾、CRC校验、数据长度
		 if(Flag_NetConnect == 1)
		 {
		 	if(MIN%5==0)			 //5分钟
			{
				Save_data_01_process(); //存

				Update_data02();
			 	SendData( Send_data02,18);//发送02包；
			 	EVENT_02Rev->OSEventCnt = 0;
			 	OSSemPend(EVENT_02Rev,5000,&err);	// 等待02包返回。 10s;

				WDTIM_COUNTER	= 1;									/* 喂狗							*/

			 	if(err == OS_NO_ERR)
			  	{
			  	   g_u8Flag_wireless = 1;
				   Update_data08();
				   SendData(Send_data08,65);   //发送08包
				   OSTimeDly(800);
			  	}
				else
				{
					Update_data02();
			 		SendData( Send_data02,18);//发送02包；
			 		EVENT_02Rev->OSEventCnt = 0;
			 		OSSemPend(EVENT_02Rev,5000,&err);	// 等待02包返回。 10s;

					WDTIM_COUNTER	= 1;									/* 喂狗							*/

			 		if(err == OS_NO_ERR)
			  		{
			  		   	g_u8Flag_wireless = 1;
				 		Update_data08();
					   	SendData(Send_data08,65);   //发送08包
				   		OSTimeDly(800);
			  		}
					else
					{
						g_u8Flag_wireless = 0;	
					}
				}
				
				if(g_u8Flag_wireless == 0)
				{
					FW_fram_save(Send_data01_temp);		 //存储当前统计帧
				}
				else
				{
					SendData(Send_data01_temp,170);
					OSSemPend(EVENT_0ARev,5000,&err);

					WDTIM_COUNTER	= 1;									/* 喂狗							*/

					if(err == OS_NO_ERR)
					{	
					 	l_flag_send01 = 1;//确认发送成功；
					
					}
					else
					{
						SendData(Send_data01_temp,170);
						OSSemPend(EVENT_0ARev,5000,&err);

						WDTIM_COUNTER	= 1;									/* 喂狗							*/

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
						while(0==FW_fram_read(Send_data01_temp))	 //取出铁电数据	返回：0：正确取出，1：铁电已空，2：超时无响应
						{
							SendData(Send_data01_temp,170);		//发送
			  				uart_send_count = uart_send_count +1;
			  				//发送01数据包；		  //会收到回复的0A数据包。
			  				OSSemPend(EVENT_0ARev,5000,&err);	//等待0A包返回。2500--5s; 10000--20s;

							WDTIM_COUNTER	= 1;									/* 喂狗							*/

							if(err != OS_NO_ERR)  //没有发送成功
							{
								FW_fram_save(Send_data01_temp); //存回去
								OSRec0A_count = OSRec0A_count + 1;
								break;
							}
						}	
					 }
					 else
					 {
					 	FW_fram_save(Send_data01_temp);		 //连上了但是发送存储当前统计帧	
					 }
				}
				WDTIM_COUNTER	= 1;									/* 喂狗							*/	
				OSTimeDly(5000);
				WDTIM_COUNTER	= 1;									/* 喂狗							*/	
				OSTimeDly(5000);
				WDTIM_COUNTER	= 1;									/* 喂狗							*/	
				OSTimeDly(5000);
				WDTIM_COUNTER	= 1;									/* 喂狗							*/	
				OSTimeDly(5000);
				WDTIM_COUNTER	= 1;									/* 喂狗							*/	
				OSTimeDly(5000);
				WDTIM_COUNTER	= 1;									/* 喂狗							*/	
				OSTimeDly(5000);
				WDTIM_COUNTER	= 1;									/* 喂狗							*/	
				OSTimeDly(5000);
				WDTIM_COUNTER	= 1;									/* 喂狗							*/	
				
			}
			else 
			{
				OSTimeDly(5000);  //10s
			}
		}
	}
}

void FW_fram_save(uint8 *data)
{
	uint8 ReadFWtemp[170];
	uint8 WriteFWtemp[170];
	uint8 head,tail;		  //队列头：出队，队列尾：入队。

	ReadC256(BUF1ADDR,ReadFWtemp,2);	  //读取控制寄存器中的内容
	head=ReadFWtemp[0];
	tail=ReadFWtemp[1];

	if((tail+1)%100==head)	   //队列满
	{
		return;
	}
	else		              //队列未满
	{
		memcpy(WriteFWtemp,data,170);
		WriteC256(BUF1ADDR+0x200+tail*170,WriteFWtemp,170);							 //写入铁电
		tail=(tail+1)%100;
		WriteC256(BUF1ADDR+0x01,&tail,1);											 //写入队列指针
	}
}
uint8 FW_fram_read(uint8 *data)
{
	uint8 ReadFWtemp[170];
//	uint8 WriteFWtemp[170];
	uint8 head,tail;		  //队列头：出队，队列尾：入队。

	ReadC256(BUF1ADDR,ReadFWtemp,2);	  //读取控制寄存器中的内容
	head=ReadFWtemp[0];
	tail=ReadFWtemp[1];

	if(tail==head)   //队列空
	{
		return 1;
	}
	else
	{
		ReadC256(BUF1ADDR+0x200+head*170,ReadFWtemp,170);		//出队操作
		head=(head+1)%100;
		WriteC256(BUF1ADDR,&head,1);
		memcpy(data,ReadFWtemp,170);
		return 0;
	}
}
