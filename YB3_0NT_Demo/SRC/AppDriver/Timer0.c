/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			Timer0.c
** Last modified Date:  2011-04-12
** Last Version:		1.0
** Descriptions:		��ʱ��0��غ���
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


static	void IRQ_Timer0(void);		//�жϴ���
uint32 t0_count1=0;
uint32 t0_count2=0;
//uint32 Timer_cou_JG0=0;
//uint32 Timer_cou_JG1=0;
  extern  uint32 g_au32count;
/*********************************************************************************************************
** ��������:  IRQ_Timer0
** ��������:  ��ʱ��0�жϷ������
** ��ڲ���:  ��
** ���ڲ���:  ��
** ����˵��:
*********************************************************************************************************/
extern uint32 g_au32Tempa[5000][4] ;
void IRQ_Timer0(void)
{ 	
	T0IR			= 0x01;					                /* ����жϱ�־	                */
 
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
	
#else	 //����
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
**   Time0��ʼ������
**
****************************************************************************************/
void Time0Init(void)
{
	MIC_ER			|= (1<<16);									//ʹ��Timer0�ж�
	TIMCLK_CTRL1	= 0x04;                             		/* ʹ�ܶ�ʱ��0��ʱ��            */
	T0TCR			= 0x02;                           			/* ��λ�����ܶ�ʱ��0            */
	T0IR			= 0xFF;                           			/* ��������ж�                 */ 
	T0TC			= 0x00000000;								/* ��ʱ������Ϊ0                */										
	T0PR			= 0x0000000F;						 		/* ʱ��16��Ƶ                   */											
	T0PC			= 0x00000000;
	T0MCR			= 0x0003;						       		/* ����T0MR0ƥ���λT0TC��    */
//	T0MCR			= 0x0001;//  ���� 	                                                    		/* �������жϱ�־               */
	T0CTCR			= 0x00;                                                                    				
	T0MR0			= Fpclk / (16*OS_TICKS_PER_SEC);			/* 1���Ӷ�ʱ                    */
	//10ms
	T0TCR			= 0x01;						   				/* ������ʱ��0                  */										    
	micIrqFuncSet(16, 0, (unsigned int)IRQ_Timer0);				/* �����ж�������				*/
}

