/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			FAT32App.c
** Last modified Date:  2011-05-04
** Last Version:		1.0
** Descriptions:		SD卡FAT32相关函数--创建文件、读文件、写文件、删除文件
**
**--------------------------------------------------------------------------------------------------------
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
#include "sdconfig.h"
#include "FAT32App.h"
#include "FAT32.h"

//static	sd_struct	sd_info;					//SD卡信息
/*****************************************
延时函数
*****************************************/
void Delay_fat(uint32 i)
{
    uint32 k;
	for(;i!=0;i--)
	   for (k = 0; k < 0xFFFFF; k++)
	       ;
}
/*****************************************
FAT32文件系统初始化
*****************************************/
void FAT32Init(void)	  
{
	uint32 i = 0;
	uint32 l_ulStatus 	= 0;
	uint8  l_au8Sdbuf[512];
    uint32 l_ulDirStart;
	
	for (i = 0; i < 512; i++ ) 
	{				
        l_au8Sdbuf[i] = 0x00;	  							    /* 初始化目录项用数据*/
    }
    /*启动*/                                               
//    l_ulStatus = WriteSDCardBlock(0x7657, l_au8Sdbuf);         /* 必须有这一步！ 将sdbuf缓冲区的数据写入实际应用时写入第0x7657扇区，FAT2最后的扇区-不影响其他操作
//	                                                              试验时用的是第0x7658扇区，SD卡根目录开始扇区-但影响其他操作 */
//	if (l_ulStatus != SD_NO_ERR) 
//	{
//		P3_OUTP_CLR = (1<<9);
//		Delay_fat(2);
//		P3_OUTP_SET = (1<<9);
//		Delay_fat(2);
//		P3_OUTP_CLR = (1<<9);	
//	}
															  
	i=ReadBPB();											    /* 读启动区数据 */
	if(i==SD_FAIL)
	{
		P3_OUTP_CLR = (1<<9);
		Delay_fat(2);
		P3_OUTP_SET = (1<<9);
		Delay_fat(2);
		P3_OUTP_CLR = (1<<9);	
	}    

}

/*********************************************************************************************************
** Function name:		FAT32CreateFile
** Descriptions:		FAT32文件系统  创建文件
**
** input parameters:	None
** output parameters:	None
** Returned value:		操作是否成功	0：成功
**									    1：失败
** Created by:			HE NA
** Created Date:		20110504
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
uint8  FAT32CreateFile(uint8 p_au8Name[11], uint32  p_ulSize)
{
    return CreateFile(p_au8Name, p_ulSize);
}

/*********************************************************************************************************
** Function name:		FAT32ReadFile
** Descriptions:		FAT32文件系统  读文件
**
** input parameters:	None
** output parameters:	None
** Returned value:		操作是否成功	0：成功
**									    1：失败
** Created by:			HE NA
** Created Date:		20110504
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
uint8  FAT32ReadFile(uint8 p_au8Name[11], uint32 p_ulStart, uint32 p_ulLen, uint8 *p_au8Buffer)	
{
	return ReadFile(p_au8Name, p_ulStart, p_ulLen, p_au8Buffer);
}

/*********************************************************************************************************
** Function name:		FAT32WriteFile
** Descriptions:		FAT32文件系统  写文件
**
** input parameters:	None
** output parameters:	None
** Returned value:		操作是否成功	0：成功
**									    1：失败
** Created by:			HE NA
** Created Date:		20110504
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
uint8  FAT32WriteFile(uint8 p_au8Name[11], uint32 p_ulStart, uint32 p_ulLen, uint8 *p_au8Buffer)
{
	return WriteFile(p_au8Name, p_ulStart, p_ulLen, p_au8Buffer);
}

/*********************************************************************************************************
** Function name:		FAT32EreaseFile
** Descriptions:		FAT32文件系统  删除文件
**
** input parameters:	None
** output parameters:	None
** Returned value:		操作是否成功	0：成功
**									    1：失败
** Created by:			HE NA
** Created Date:		20110504
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
uint8  FAT32EreaseFile(uint8  p_au8Name[11])
{
    return 	EreaseFile(p_au8Name);
}



