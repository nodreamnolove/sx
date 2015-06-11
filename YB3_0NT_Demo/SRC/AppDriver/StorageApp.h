/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			StorageApp.h
** Last modified Date:  20110419
** Last Version:		1.0
** Descriptions:		存储设备调用集合,包括SD卡，NandFlash，Norflash
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

#ifndef	__STORAGEAPP_H
#define	__STORAGEAPP_H

#include "config.h"
#include "TDC256.h"

#include "NORFlash.h"

#ifdef	__STRG_SD
#include "SDApp.h"
#include "FAT32App.h"
#endif

#ifdef	__STRG_NAND		//NAND设备
#include "NANDFlash.h"
#endif		//__STRG_NAND

#ifdef	__STORAGEAPP_C
#define	STA_EXT
#else
#define	STA_EXT		extern
#endif

//初始化存储设备，包括Flash、铁电、SD卡
STA_EXT	void	StorageInit(void);										//初始化

STA_EXT	void	SaveRamToFlash(uint8 * p_pu8Data);	//将设置数据写入Flash
STA_EXT	void	LoadFlashToRam(uint8 * p_pu8Data);	//从Flash载入设置数据

/*
TD_EXT	uint8	ReadC256(uint16 p_u16Addr, uint8 * p_pu8ReadBuf, uint16 p_u16Len);
TD_EXT	uint8	WriteC256(uint16 p_u16Addr, uint8 * p_pu8WriteBuf, uint16 p_u16Len);
*/

#ifdef	__STRG_SD		//是否包含SD卡设备
STA_EXT	void	SaveDataToSD(uint32 p_u32Address, uint32 p_u32Bytes, uint8 * p_pu8Data);		//保存数据到SD卡
STA_EXT	void	LoadSDToData(uint32 p_u32Address, uint32 p_u32Bytes, uint8 * p_pu8Data);		//加载SD卡数据

#endif
#endif		//__STORAGEAPP_H

