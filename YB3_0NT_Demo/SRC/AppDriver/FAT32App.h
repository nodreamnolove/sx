/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			FAT32App.h
** Last modified Date:  2011-05-04
** Last Version:		1.0
** Descriptions:		Sd卡FAT32初始化、应用
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

#define	DIRSECNUM		128				//目录项所占扇区数目，即可以存的文件数=16*DIRSECNUM  （16*128=2048） FAT32.c文件中用到
#define	DIRCLUSNUM		(DIRSECNUM/8)	//目录项所占簇的数目，用于初始化FAT表

FAT32A_EXT	void	FAT32Init(void);				//FAT32初始化 
FAT32A_EXT	uint8	FAT32CreateFile(uint8  p_au8Name[11],  uint32  p_ulSize);	//创建一个空文件
FAT32A_EXT	uint8	FAT32ReadFile(uint8  p_au8Name[11],  uint32  p_ulStart,  uint32  p_ulLen,  uint8  *p_au8Buffer);	//读指定文件
FAT32A_EXT	uint8	FAT32WriteFile(uint8 p_au8Name[11],  uint32  p_ulStart,  uint32  p_ulLen,  uint8  *p_au8Buffer);	//写指定文件
FAT32A_EXT	uint8	FAT32EreaseFile(uint8  p_au8Name[11]);	//删除指定文件

#endif

