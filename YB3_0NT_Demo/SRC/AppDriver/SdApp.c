/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			SdApp.c
** Last modified Date:  2011-04-13
** Last Version:		1.0
** Descriptions:		SD卡相关函数
**
**--------------------------------------------------------------------------------------------------------
** Created by:			ZHANG Ye
** Created date:		2011-04-13
** Version:				1.0
** Descriptions:		SDCard
**
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
** Version:
** Descriptions:
**
*********************************************************************************************************/

#ifdef	__STRG_SD		//是否包含SD卡设备
#define	__SDAPP_C
#include "SdApp.h"
#include "sdcommon.h"

//static	sd_struct	sd_info;					//SD卡信息

/*********************************************************************************************************
** Function name:		SDCardInit
**
** Descriptions:		SD卡操作初始化函数
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
void SDCardInit(void)
{
	uint8	l_u8Status;
	P2_DIR_CLR = (1<<25);
	
	l_u8Status	= SD_Initialize(&sd_info);                                            /* 初始化SD卡                   */
    
    if (l_u8Status != SD_NO_ERR) 
    {
       //初始化失败，响铃三声提示
		BeepON();
		Delay(500);	
		BeepOFF();
		Delay(1000);
		BeepON();
		Delay(500);	
		BeepOFF();
		Delay(1000);
		BeepON();
		Delay(500);
		BeepOFF();
    }
	
    micIrqFuncSet(13, 3, (uint32)MCI_Handler);						/* 设置SD/MMC卡控制器的中断向量 */

	if (GetSDInsertStatus != 0)			//检测卡插入状态
		g_u8SDInsert=0;						/* 未完全插入 */	
	else
		g_u8SDInsert=1;						/* 完全插入 */	
}

/*******************************************************************************************************************
** 函数名称: ReadSDCardBlock()				
**
** 功能描述: 从SD/MMC卡中读出一个数据块
**
** 输　  入: INT32U blockaddr: 以块为单位的块地址, 例如, 卡开始的0 ~ 511字节为块地址0, 512 ~ 1023字节的块地址为1
**	         
** 输 　 出: INT8U *recbuf   : 接收缓冲区,长度固定为 512 字节	
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
********************************************************************************************************************/
INT8U ReadSDCardBlock(INT32U blockaddr, INT8U *recbuf)
{
	return	SD_ReadBlock(&sd_info, blockaddr, recbuf);
}

/*******************************************************************************************************************
** 函数名称: WriteSDCardBlock()				
**
** 功能描述: 向SD/MMC卡中写入一个块	
**
** 输　  入: INT32U blockaddr: 以块为单位的块地址, 例如, 卡开始的0 ~ 511字节为块地址0, 512 ~ 1023字节的块地址为1
**           INT8U *sendbuf  : 发送缓冲区,长度固定为 512 字节	
**	         
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
********************************************************************************************************************/
INT8U WriteSDCardBlock(INT32U blockaddr, INT8U *sendbuf)
{
	return	SD_WriteBlock(&sd_info, blockaddr, sendbuf);
}

#endif		//__STRG_SD
