/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			TaskAnalysis.C
** Last modified Date:  20110512
** Last Version:		1.0
** Descriptions:		���ݷ������񣬰��������г�����(����4)�ͼ�������(����6)
**
**--------------------------------------------------------------------------------------------------------
** Created by:			ZHANG Ye
** Created date:		20110512
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
#define	__TASKANALYSIS_C
#include "TaskAnalysis.h"
																			 
#define		THRESHOLDALIAS			g_sudtThreshold		//��ֵ
#define		SETUPALIAS				g_sspSetup			//���ò����ṹ

/*********************************************************************************************************
** Function name:		TaskRec4
** Descriptions:		�ж��г�����
** input parameters:	None 
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110512	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void TaskRec4(void *tdata)
{
	tdata = tdata;

	OSTimeDly(10);
	while(1)
	{
		
		OSSemAccept(g_psemScreenRefresh);
		OSSemPost(g_psemScreenRefresh);
		

	}
}

/*********************************************************************************************************
** Function name:		TaskRec6
** Descriptions:		��������
** input parameters:	None 
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110512	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  	ZHANG Ye	
** Modified date:	  	20111026
**						�޸�����������˳���Լ���ȡ���������ʱ��
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void TaskRec6(void *tdata)					    
{
	AxleQue *	l_psaqTempInfo;
	uint8	l_u8Err;

//	uint16		l_u16RcvbufAddNum = 0;
//	int16 * 	l_pn16RecBuf;

	tdata = tdata; 
	OSTimeDly(10);
	while(1)
	{
		l_psaqTempInfo	= OSQPend(g_pqueAxleCalc, 0, &l_u8Err);		//�ȴ������ź���
		if(l_u8Err == OS_NO_ERR)
		{
			if (l_psaqTempInfo != (void *)0)
				l_psaqTempInfo	= (void *)0;
		}
	}
}
