/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			StorageApp.h
** Last modified Date:  20110419
** Last Version:		1.0
** Descriptions:		�洢�豸���ü���,����SD����NandFlash��Norflash
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

#ifdef	__STRG_NAND		//NAND�豸
#include "NANDFlash.h"
#endif		//__STRG_NAND

#ifdef	__STORAGEAPP_C
#define	STA_EXT
#else
#define	STA_EXT		extern
#endif

//��ʼ���洢�豸������Flash�����硢SD��
STA_EXT	void	StorageInit(void);										//��ʼ��

STA_EXT	void	SaveRamToFlash(uint8 * p_pu8Data);	//����������д��Flash
STA_EXT	void	LoadFlashToRam(uint8 * p_pu8Data);	//��Flash������������

/*
TD_EXT	uint8	ReadC256(uint16 p_u16Addr, uint8 * p_pu8ReadBuf, uint16 p_u16Len);
TD_EXT	uint8	WriteC256(uint16 p_u16Addr, uint8 * p_pu8WriteBuf, uint16 p_u16Len);
*/

#ifdef	__STRG_SD		//�Ƿ����SD���豸
STA_EXT	void	SaveDataToSD(uint32 p_u32Address, uint32 p_u32Bytes, uint8 * p_pu8Data);		//�������ݵ�SD��
STA_EXT	void	LoadSDToData(uint32 p_u32Address, uint32 p_u32Bytes, uint8 * p_pu8Data);		//����SD������

#endif
#endif		//__STORAGEAPP_H

