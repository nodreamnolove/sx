/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			SdApp.h
** Last modified Date:  2011-04-13
** Last Version:		1.0
** Descriptions:		Sd卡初始化、应用
**
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
** Version:
** Descriptions:
**
*********************************************************************************************************/
#ifdef	__STRG_SD		//是否包含SD卡设备

#ifndef	__SDAPP_H
#define	__SDAPP_H

#include "config.h"
#include "sdconfig.h"


#ifdef	__SDAPP_C
#define	SDA_EXT
#else
#define	SDA_EXT	extern
#endif

#define	ROOTADDR	"SD:\"				//根目录路径

#define	GetSDInsertStatus		(P3_INP_STATE & SD_INSERT_BIT)		//SD卡插入状态

//初始化
SDA_EXT	void	SDCardInit(void);

SDA_EXT	INT8U ReadSDCardBlock(INT32U blockaddr, INT8U *recbuf);
SDA_EXT	INT8U WriteSDCardBlock(INT32U blockaddr, INT8U *sendbuf);

/*******************************************************************************************************************
** 函数名称:	ReadSDCardFile()				
**
** 功能描述: 	读取SD卡中的文件
** 输　  入: 	p_pcFolderPath:		文件夹路径
**	         	p_pcFileName:		文件名
**	         
** 输 　 出: 	p_pu8DataOutBuf:	数据指针
**				p_pu32DataSize:		数据长度（字节数）
**
** 返 回 值:	0:   正确    >0:   错误码
********************************************************************************************************************/
SDA_EXT	uint8	ReadSDCardFile(char * p_pcFolderPath,char * p_pcFileName, uint8 * p_pu8DataOutBuf, uint32 *p_pu32DataSize);

/*******************************************************************************************************************
** 函数名称:	WriteSDCardFile()				
**
** 功能描述: 	向SD卡中写文件
** 输　  入: 	p_pcFolderPath:		文件夹路径
**	         	p_pcFileName:		文件名
**	         	p_pu8DataInBuf:		数据指针
**				p_pu32DataSize:		数据长度（字节数）
** 输 　 出: 	None
** 返 回 值:	0:   正确    >0:   错误码
********************************************************************************************************************/
SDA_EXT	uint8	WriteSDCardFile(char * p_pcFolderPath,char * p_pcFileName, uint8 * p_pu8DataInBuf, uint32 *p_pu32DataSize);

/*******************************************************************************************************************
** 函数名称:	CheckFileNum()				
**
** 功能描述: 	查询指定文件夹下中的文件数和文件夹数
** 输　  入: 	p_pcFolderPath:		查询的文件夹
** 输 　 出: 	p_pu32FileNum: 		直接文件数
**				p_pu32AllFileNum: 	总文件数
**				p_pu32FolderNum: 	直接文件夹数
** 返 回 值:	0:   正确    >0:   错误码
********************************************************************************************************************/
SDA_EXT	uint8	CheckFileNum(char * p_pcFolderPath, uint32 *p_pu32FileNum, uint32 *p_pu32AllFileNum, uint32 *p_pu32FolderNum);

/*******************************************************************************************************************
** 函数名称:	CheckSDCardCapacity()				
**
** 功能描述: 	查询SD卡中的容量信息(以MB为单位)
** 输　  入: 	None
** 输 　 出: 	p_pu32AllCap: 		总容量
**				p_pu32RemainCap:	剩余容量
** 返 回 值:	0:   正确    >0:   错误码
********************************************************************************************************************/
SDA_EXT	uint8 	CheckSDCardCapacity(uint32 *p_pu32AllCap,uint32 *p_pu32RemainCap);

/*******************************************************************************************************************
** 函数名称:	CreateSDFolder()				
**
** 功能描述: 	在SD卡指定路径下创建文件夹
** 输　  入: 	p_pcFolderPath:		文件夹路径
**	         	p_pcFileName:		文件夹名
** 输 　 出: 	None
** 返 回 值:	0:   正确    >0:   错误码
********************************************************************************************************************/
SDA_EXT	uint8	CreateSDFolder(char * p_pcFolderPath, char * p_pcFolderName);

/*******************************************************************************************************************
** 函数名称:	DeleteSDFolder()				
**
** 功能描述: 	删除SD卡指定路径下文件夹
** 输　  入: 	p_pcFolderPath:		文件夹路径
**	         	p_pcFileName:		文件夹名
** 输 　 出: 	None
** 返 回 值:	0:   正确    >0:   错误码
********************************************************************************************************************/
SDA_EXT	uint8	DeleteSDFolder(char * p_pcFolderPath, char * p_pcFolderName);

/*******************************************************************************************************************
** 函数名称:	RenameSDFolder()				
**
** 功能描述: 	重命名SD卡指定路径下文件夹
** 输　  入: 	p_pcFolderPath:		文件夹路径
**	         	p_pcFileName:		文件夹名
**	         	p_pcNewFolderName:	新文件夹名
** 输 　 出: 	None
** 返 回 值:	0:   正确    >0:   错误码
********************************************************************************************************************/
SDA_EXT	uint8	RenameSDFolder(char * p_pcFolderPath, char * p_pcFolderName, char * p_pcNewFolderName);

#endif	//__SDAPP_H
#endif		//__STRG_SD
