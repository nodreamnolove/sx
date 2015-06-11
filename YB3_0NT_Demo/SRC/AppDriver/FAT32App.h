/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			FAT32App.h
** Last modified Date:  2011-05-04
** Last Version:		1.0
** Descriptions:		Sd��FAT32��ʼ����Ӧ��
**
**--------------------------------------------------------------------------------------------------------
**
** Created by:			HE NA
** Created date:		2011-05-04
** Version:				1.0
** Descriptions:		SDCard FAT32
**
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
** Version:
** Descriptions:
**
*********************************************************************************************************/
#ifndef	__FAT32APP_H__
#define	__FAT32APP_H__

#ifdef	__FAT32APP_C
#define	FAT32A_EXT	

#include "sdconfig.h"
#include "FAT32.h"	
#else
#define	FAT32A_EXT	extern
#endif

#define	DIRSECNUM		128				//Ŀ¼����ռ������Ŀ�������Դ���ļ���=16*DIRSECNUM  ��16*128=2048�� FAT32.c�ļ����õ�
#define	DIRCLUSNUM		(DIRSECNUM/8)	//Ŀ¼����ռ�ص���Ŀ�����ڳ�ʼ��FAT��

FAT32A_EXT	void	FAT32Init(void);				//FAT32��ʼ�� 
FAT32A_EXT	uint8	FAT32CreateFile(uint8  p_au8Name[11],  uint32  p_ulSize);	//����һ�����ļ�
FAT32A_EXT	uint8	FAT32ReadFile(uint8  p_au8Name[11],  uint32  p_ulStart,  uint32  p_ulLen,  uint8  *p_au8Buffer);	//��ָ���ļ�
FAT32A_EXT	uint8	FAT32WriteFile(uint8 p_au8Name[11],  uint32  p_ulStart,  uint32  p_ulLen,  uint8  *p_au8Buffer);	//дָ���ļ�
FAT32A_EXT	uint8	FAT32EreaseFile(uint8  p_au8Name[11]);	//ɾ��ָ���ļ�

#endif

