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


void Task_Uart5_Senddata_NEW(void *tdata)
{
	uint8 err;

    tdata =tdata;
	while(1)
	{

	}	
}

