/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			Storage.c
** Last modified Date:  20110419
** Last Version:		1.0
** Descriptions:		存储设备驱动集合,包括SD卡，NandFlash，Norflash
**
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
** Version:
** Descriptions:
**
*********************************************************************************************************/
#define	__STORAGEAPP_C
#include "StorageApp.h"

#define		FLASH_ADDR		0xe0000000			//NOR Flash 接在BANK0上
#define		NORDATA			0x1D0000			//NOR中保存参数设置的位置
#define		FLHSIZE			1024				//默认2K数据


/*********************************************************************************************************
** Function name:		StorageInit
**
** Descriptions:		初始化SD卡，NandFlash，NorFlash，铁电函数
**
** input parameters:	None
** output parameters:	None
** Returned value:		None
**
** Created by:			ZHANG Ye
** Created Date:		20110331
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void StorageInit(void)
{
	//NORFlash,不需要初始化

	//SD卡
#ifdef	__STRG_SD
//	SDCardInit();
//	FAT32Init();
#endif

	//NandFlash
#ifdef	__STRG_NAND
	NandInit();
#endif

}

/*********************************************************************************************************
** Function name:		SaveRamToFlash
**
** Descriptions:		将设置数据写入Flash，目前只使用了2K的数据
**
** input parameters:	p_pu8Data	数据指针
** output parameters:	None
** Returned value:		None
**
** Created by:			ZHANG Ye
** Created Date:		20110331
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void SaveRamToFlash(uint8 * p_pu8Data)
{
	uint32		i;
	uint16 *	l_pu16Data;
	
	OS_ENTER_CRITICAL();
	
	l_pu16Data	= (uint16 *)p_pu8Data;

	EraseSector(NORDATA);		//擦除一个扇区，否则不能对这个空间进行写操作，空间为2K	

	for (i = 0; i < FLHSIZE; i += 2)
	{
		while(FALSE == WordProgram(NORDATA + i, *(l_pu16Data + (i >> 1))));		//将数据写入NORFlash
	}

	OS_EXIT_CRITICAL();
}

/*********************************************************************************************************
** Function name:		LoadFlashToRam
**
** Descriptions:		LoadFlashToRam
**
** input parameters:	p_pu8Data	数据指针
** output parameters:	None
** Returned value:		None
**
** Created by:			ZHANG Ye
** Created Date:		20110331
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void LoadFlashToRam(uint8 * p_pu8Data)
{
	uint32		i;
	uint32 *	l_pu32Data;

	l_pu32Data	= (uint32 *)p_pu8Data;

	for (i = 0; i < FLHSIZE; i += 4)
	{
		//读NORFlash
		*(l_pu32Data ++)	= *(uint32 *)(FLASH_ADDR|((NORDATA+i) & 0x1FFFFF));
	}
	l_pu32Data	= 0;
}
