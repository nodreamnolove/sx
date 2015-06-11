/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			Timer0.c
** Last modified Date:  2011-04-12
** Last Version:		1.0
** Descriptions:		计时器0相关函数
**
**--------------------------------------------------------------------------------------------------------
** Created by:			ZHANG Ye
** Created date:		2011-04-12
** Version:				1.0
** Descriptions:		Timer0
**
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
** Version:
** Descriptions:
**
*********************************************************************************************************/
#define	__TIMER0_C
#include "Timer0.h"
#include "WT_Task.h"
#include "CRC.h"
#include "common.h"
//#include "send_task.h"
#include "UART5.h"
#include "rd_data.h"
#include "W5100.h"
#include "Task_JG2.h"


static	void IRQ_Timer0(void);		//中断处理
uint32 t0_count1=0;
uint32 t0_count2=0;
//uint32 Timer_cou_JG0=0;
//uint32 Timer_cou_JG1=0;
  extern  uint32 g_au32count;
/*********************************************************************************************************
** 函数名称:  IRQ_Timer0
** 函数功能:  定时器0中断服务程序。
** 入口参数:  无
** 出口参数:  无
** 函数说明:
*********************************************************************************************************/
extern uint32 g_au32Tempa[5000][4] ;
void IRQ_Timer0(void)
{ 	
	T0IR			= 0x01;					                /* 清除中断标志	                */
 
//	OS_ENTER_CRITICAL();
	OSTimeTick(); 
	t0_count1++;
	t0_count2++;    
#ifndef 	 SIM_SOFTWARE	  
	 if(Flag_NetConnect==1)
	 {		
	 
	 	if(t0_count1==1)			
		{
			//g_au32Tempa[g_au32count][1] = t0_count2;
		   //g_au32count=(g_au32count+1)%500;
			S0_Data&=~S_TRANSMITOK;	
			S_tx_process(0,RdToLMSSendData, 24);
		 }
		 else if(t0_count1==3)
		 {
			S1_Data&=~S_TRANSMITOK;	
	        S_tx_process(1,RdToLMSSendData, 24); 
		 }
		 else if(t0_count1==5)
		 {	
			S2_Data&=~S_TRANSMITOK;	
	        S_tx_process(2,RdToLMSSendData, 24);  
		 }
		 else if(t0_count1==7)
		 {	
			S3_Data&=~S_TRANSMITOK;	
	        S_tx_process(3,RdToLMSSendData, 24);									
		 }
		 else if(t0_count1>9)
			t0_count1=0;
	}  
	
#else	 //仿真
	if(Flag_NetConnect==1)
	{
	  	if(t0_count1 == 1)
		{					
			S0_Data&=~S_TRANSMITOK;
			S_tx_process(0,RdToLMSSendData, 24);

		
		}
		else
		if(t0_count1 == 3)
		{
			S2_Data&=~S_TRANSMITOK;
			S_tx_process(2,RdToLMSSendData, 24);
		}
		else if(t0_count1 > 9)
		{  		  
			t0_count1 = 0;
		}
	}


#endif
//	OS_EXIT_CRITICAL();
}
/***************************************************************************************
**
**   Time0初始化函数
**
****************************************************************************************/
void Time0Init(void)
{
	MIC_ER			|= (1<<16);									//使能Timer0中断
	TIMCLK_CTRL1	= 0x04;                             		/* 使能定时器0的时钟            */
	T0TCR			= 0x02;                           			/* 复位并禁能定时器0            */
	T0IR			= 0xFF;                           			/* 清除所有中断                 */ 
	T0TC			= 0x00000000;								/* 定时器设置为0                */										
	T0PR			= 0x0000000F;						 		/* 时钟16分频                   */											
	T0PC			= 0x00000000;
	T0MCR			= 0x0003;						       		/* 设置T0MR0匹配后复位T0TC，    */
//	T0MCR			= 0x0001;//  更改 	                                                    		/* 并产生中断标志               */
	T0CTCR			= 0x00;                                                                    				
	T0MR0			= Fpclk / (16*OS_TICKS_PER_SEC);			/* 1秒钟定时                    */
	//10ms
	T0TCR			= 0x01;						   				/* 启动定时器0                  */										    
	micIrqFuncSet(16, 0, (unsigned int)IRQ_Timer0);				/* 加入中断向量表				*/
}

