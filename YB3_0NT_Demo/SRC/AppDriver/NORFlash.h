/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			NORFlash.h
** Last modified Date:  20110419
** Last Version:		1.0
** Descriptions:		NORFlash驱动
**
**--------------------------------------------------------------------------------------------------------
** Created by:			ZHANG Ye
** Created date:		20110419
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
#ifndef	__NORFLASH_H
#define	__NORFLASH_H

#include "config.h"

#ifdef	__NORFLASH_C
#define	NOR_EXT
#else
#define	NOR_EXT	extern
#endif

#define		IRAMADDR		0x08020000			//IRAM中缓存2K数据的首地址

#define		NORSTARTREC		0x1E2000			//程序启动记录地址，总共8K:0x1E2000~0x1E3FFF
#define		VEHBUFREC		0x190000			//过车数据记录，64B*2048= 256K： 0x190000~0x1CFFFF

//基本功能
NOR_EXT	uint8	EraseNORFlash(void);					//擦除所有NORFlash数据
NOR_EXT	uint8	EraseBlock(uint32 p_u32BlockID);		//擦除Block数据
NOR_EXT	uint8	EraseSector(uint32 p_u32SectorAddr);	//擦除Sector数据
NOR_EXT uint8	WordProgram(uint32 p_u32Address, uint16 p_u16Data);		//写WORD 
//
////扩展功能
////读写Flash中指定位置数据
NOR_EXT	uint8	ReadNORFlash(uint32 p_u32Address, uint32 p_u32ReadBytes, uint8 * p_pu8Data);
NOR_EXT	uint8	WriteNORFlash(uint32 p_u32Address, uint32 p_u32WriteNumber, uint8 * p_pu8Data);

#endif		//__NORFLASH_H
