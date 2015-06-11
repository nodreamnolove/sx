/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			NANDFlash.h
** Last modified Date:  20110419
** Last Version:		1.0
** Descriptions:		NANDFlash驱动,只在周立功开发板上使用
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
#ifndef	__NANDFlash_H
#define	__NANDFlash_H

#include "config.h"

#ifdef	__NANDFlash_C
#define	NAND_EXT
#else
#define	NAND_EXT	extern
#endif


NAND_EXT	void	NandInit(void);									//初始化
NAND_EXT	uint8	EraseNandBlock(uint32 p_u32BlockAddress);		//擦除指定Block数据
NAND_EXT	uint8	ReadNandFlashID(uint32 * p_pu32ID);				//读取NandFlash的ID

//读写NandPage
NAND_EXT	uint8	ReadNandPage(uint32 p_u32Address, uint32 p_u32ReadBytes, uint8 * p_pu8Data);
NAND_EXT	uint8	ProgramNandPage(uint32 p_u32Address, uint32 p_u32WriteBytes, uint8 * p_pu8Data);

#endif		//__NANDFlash_H
