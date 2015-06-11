/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			UIOperation.h
** Last modified Date:  20110505
** Last Version:		1.0
** Descriptions:		�������н����������
**
**--------------------------------------------------------------------------------------------------------
** Created by:			ZHANG Ye
** Created date:		20110505
** Version:				1.0
** Descriptions:		The original version
**
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
** Version:
** Descriptions:
**
*********************************************************************************************************/
#ifndef	__UIOPERATION_H
#define	__UIOPERATION_H

#include "Config.h"

#ifdef	__UIOPERATION_C

#define		UIO_EXT

#include "Common.h"
#include "TDC256.h"

#if	YBVERSION >= 30		//3.0�Ǳ���
#include "PCF8563.h"
#include "StorageApp.h"
#else	//2.2
#include "IAP.h"
#include "RTC.h"
#endif	//#if	YBVERSION >= 30		//3.0�Ǳ���

#else
#define		UIO_EXT		extern
#endif

UIO_EXT	void	SaveParams(void);			//�����������
UIO_EXT void 	SaveThreshold(void);		//������ֵ 
UIO_EXT void 	SaveVehBuf(void);			//���泵��������Ϣ
#if	YBVERSION >= 30		//3.0�Ǳ���
UIO_EXT	void	SaveNetInfo(void);			//����������Ϣ
#endif

UIO_EXT	void	InitSystem(void);			//��ʼ������ϵͳ����
UIO_EXT	void	InitNonWeight(void);		//��ʼ���ǳ�������
UIO_EXT	void	InitMotionModify(void);		//��ʼ����̬��������
UIO_EXT	void	InitVehModify(void);		//��ʼ��������������
UIO_EXT	void	InitStaticModify(void);		//��ʼ����̬��������
UIO_EXT	void	InitThreshold(void);		//��ʼ����ֵ����
UIO_EXT	void	InitVehBufIndex(void);		//��ʼ�����绺�泵����Ϣ
#if	YBVERSION >= 30		//3.0�Ǳ���														  
UIO_EXT	void 	InitNetParam(void);			//��ʼ��������Ϣ
#endif

UIO_EXT	void	SaveTime(SystemTime p_stSet);				//����ʱ��

UIO_EXT	uint8	RecoverToLast(void);		//�ָ�Ϊ����궨����
UIO_EXT	uint8	RecoverToHistory(void);		//�ָ�Ϊ��ʷ�궨����

UIO_EXT	void	ClearStartupCnt(void);		//������������
UIO_EXT	void	AddStartupCnt(void);		//����������1

UIO_EXT	void	InitRestart(void);			//��ʼ�����Զ�����

#if	YBVERSION >= 30		//3.0�Ǳ���
UIO_EXT	uint16	SavePassVehInfo(VehicleRecord * p_pvrPassInfo);						//�����������
UIO_EXT	void	ReadPassVehInfo(VehicleRecord * p_pvrPassInfo, uint16 p_u16Addr);	//��ȡ��������
#endif	//#if	YBVERSION >= 30		//3.0�Ǳ���

#endif		//__UIOPERATION_H
