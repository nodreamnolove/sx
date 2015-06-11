/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			Storage.c
** Last modified Date:  20110419
** Last Version:		1.0
** Descriptions:		�洢�豸��������,����SD����NandFlash��Norflash
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

#define		FLASH_ADDR		0xe0000000			//NOR Flash ����BANK0��
#define		NORDATA			0x1D0000			//NOR�б���������õ�λ��
#define		FLHSIZE			1024				//Ĭ��2K����


/*********************************************************************************************************
** Function name:		StorageInit
**
** Descriptions:		��ʼ��SD����NandFlash��NorFlash�����纯��
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
	//NORFlash,����Ҫ��ʼ��

	//SD��
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
** Descriptions:		����������д��Flash��Ŀǰֻʹ����2K������
**
** input parameters:	p_pu8Data	����ָ��
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

	EraseSector(NORDATA);		//����һ�������������ܶ�����ռ����д�������ռ�Ϊ2K	

	for (i = 0; i < FLHSIZE; i += 2)
	{
		while(FALSE == WordProgram(NORDATA + i, *(l_pu16Data + (i >> 1))));		//������д��NORFlash
	}

	OS_EXIT_CRITICAL();
}

/*********************************************************************************************************
** Function name:		LoadFlashToRam
**
** Descriptions:		LoadFlashToRam
**
** input parameters:	p_pu8Data	����ָ��
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
		//��NORFlash
		*(l_pu32Data ++)	= *(uint32 *)(FLASH_ADDR|((NORDATA+i) & 0x1FFFFF));
	}
	l_pu32Data	= 0;
}
