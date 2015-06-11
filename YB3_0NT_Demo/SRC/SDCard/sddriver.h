/****************************************Copyright (c)**************************************************
**                               Guangzhou ZLG-MCU Development Co.,LTD.
**                                      graduate school
**                                 http://www.zlgmcu.com
**
**--------------File Info-------------------------------------------------------------------------------
** File name:			sddriver.h
** Last modified Date:	2007-5-19
** Last Version:		V2.0
** Descriptions:		sddriver.c 的头文件
**
**
**------------------------------------------------------------------------------------------------------
** Created by:			Ming Yuan Zheng
** Created date:		2005-1-6
** Version:				V1.0
** Descriptions:		The original version
**
**------------------------------------------------------------------------------------------------------
** Modified by:			Ming Yuan Zheng
** Modified date:		2005-3-10
** Version:				V2.0
** Descriptions:		增加了对MMC卡的支持,增加了对UCOS-II的支持,使该模块不仅能运行于前后台系统,还可运行
**						于UCOS-II上
**------------------------------------------------------------------------------------------------------
** Modified by:         Ming Yuan Zheng
** Modified date:       2007-5-19
** Version:	            V3.0
** Descriptions:        1. 增加了对SD/MMC卡 SD 总线模式的支持(不同模式下API函数名相同), 各API函数的入口增加了sd_struct结构体, 用于支持多卡读写
**
**                      2. SPI模式下和SD模式下共用 SDMMC_GetCardInfo() 和 SDMMC_CalTimeout()函数
**
**                      3. sd_struct结构体增加了成员多个成员变量, 访问卡的时钟及访问卡的最大时钟改为可动态获取
**
**                      4. SPI 模式下的命令函数使用SpiCmd_ 开头, SD 模式下的命令使用 SdCmd_ 开头
**
**                      5. SPI 模式下与硬件相关的函数使用 SdSpiHal_ 开头, SD 模式下与硬件相关的函数使用 SdHal_ 开头
********************************************************************************************************/

#ifndef __SDDRIVER_H__
#define __SDDRIVER_H__


    	/*
		*************************************************************

				用户API函数:	读, 写, 擦 SD卡  API函数及错误码
 			
		*************************************************************
		*/
		
/* 错误码 error code */
#define   SD_NO_ERR			     	0x00			// 函数执行成功
#define   SD_ERR_NO_CARD		 	0x01			// 卡没有完全插入到卡座中
#define   SD_ERR_USER_PARAM      	0x02			// 用户使用API函数时，入口参数有错误
#define   SD_ERR_CARD_PARAM		 	0x03			// 卡中参数有错误（与本模块不兼容）
#define	  SD_ERR_VOL_NOTSUSP        0x04			// 卡不支持3.3V供电
#define   SD_ERR_OVER_CARDRANGE		0x05			// 操作超出卡容量范围
#define   SD_ERR_UNKNOWN_CARD       0x06			// 无法识别卡型

#define   SD_ERR_CURRENT_STATE      0x07            // 卡当前状态不正确(只适用于SD模式)
#define   SD_ERR_READY_FOR_DATA     0x08            // 卡数据缓冲区未就绪


/* SD命令可能返回的错误码 */
#define   SD_ERR_CMD_RESPTYPE	 	0x10			// 命令类型错误
#define   SD_ERR_CMD_TIMEOUT     	0x11			// 卡命令响应超时
#define   SD_ERR_CMD_RESP		 	0x12			// 卡命令响应错误
#define   SD_ERR_CMD_NOTSUSP		0x13			// 不支持该命令
#define   SD_ERR_CMD_RESPCRC        0x14			// 命令CRC响应错误


/* 数据流错误码 */
#define   SD_ERR_DATA_CRC16      	0x20			// 数据流CRC16校验不通过
#define   SD_ERR_DATA_START_TOK		0x21			// 读单块或多块时，数据开始令牌不正确
#define	  SD_ERR_DATA_RESP		 	0x22			// 写单块或多块时，卡数据响应令牌不正确
#define   SD_ERR_DATA_STARTBIT      0x23            // 写单块或多块时, 数据起始位不正确


/* 等待错误码 */
#define   SD_ERR_TIMEOUT_WAIT    	0x30			// 写或擦操作时，发生超时错误
#define   SD_ERR_TIMEOUT_READ    	0x31			// 读操作超时错误
#define	  SD_ERR_TIMEOUT_WRITE	 	0x32			// 写操作超时错误
#define   SD_ERR_TIMEOUT_ERASE   	0x33			// 擦除操作超时错误
#define	  SD_ERR_TIMEOUT_WAITIDLE 	0x34			// 初始化卡时，等待卡退出空闲状态超时错误


/* 写操作可能返回的错误码 */
#define	  SD_ERR_WRITE_BLK			0x40			// 写块数据错误
#define	  SD_ERR_WRITE_BLKNUMS      0x41			// 写多块时，想要写入的块与正确写入的块数不一致
#define   SD_ERR_WRITE_PROTECT		0x42			// 卡外壳的写保护开关打在写保护位置


/* 使用UCOS-II时可能出现的错误 */
#define   SD_ERR_CREATE_SEMSD		0xA0			// 创建SD/MMC卡访问信号量失败
#define   SD_ERR_WAIT_WRSEM         0xA1            // 等待读写信号量失败


/* SD_STRUCT结构体可能出现的错误 */
#define   SD_ERR_SD_STRUCT          0xB0            // SD_STRUCT结构体成员值存在错误


/* 其它可能的错误 */
#define   SD_ERR_HARDWARE           0xE0            // 硬件出错        


/**************************************************
	
			其它宏定义

***************************************************/

#define   CARDTYPE_UNKNOWN			0x00
#define	  CARDTYPE_SD				0x01			// 卡型为SD  卡
#define   CARDTYPE_MMC				0x02			// 卡型为MMC 卡

/* for determin version, like card type   */
#define SD_V1     0x00                                        /* version 1                              */
#define SD_V2     0x01                                        /* version 2                              */
#define SD_NA     0x02                                        /* unkown                                 */
#define   CARDTYPE_SDHC              0x03                     /* SDHC card                              */

#define   SD_READREG_TIMEOUT		8				// 读寄存器的超时时间
	

extern INT8U SD_Initialize(sd_struct *sds);
extern INT8U SD_WriteBlock(sd_struct *sds, INT32U blockaddr, INT8U *sendbuf);
extern INT8U SD_ReadBlock(sd_struct *sds, INT32U blockaddr, INT8U *recbuf);

extern INT8U SD_ReadMultiBlock(sd_struct *sds, INT32U blockaddr, INT32U blocknum, INT8U *recbuf);
extern INT8U SD_WriteMultiBlock(sd_struct *sds, INT32U blockaddr, INT32U blocknum, INT8U *sendbuf);
extern INT8U SD_EraseBlock(sd_struct *sds, INT32U startaddr, INT32U blocknum);
extern INT8U SD_Ver2ActiveInit(sd_struct *sds);

	    /*
	    *************************************************************
	    
	     			下面为软件包用到的与SD卡相关的宏定义
	     	
	    *************************************************************
	    */
	    	    	    
/* Mask off the bits in the OCR corresponding to voltage range 3.2V to
 * 3.4V, OCR bits 20 and 21 */
#define MSK_OCR_33			 	0x00100000

/* 定义在初始化阶段,等待SD卡退出空闲状态的重试次数 */
/* Number of tries to wait for the card to go idle during initialization */
#define SD_IDLE_WAIT_MAX     	8000

/* SD卡命令超时时间(单位 8clock)*/
/* timeout of command */
#define SD_CMD_TIMEOUT   	 	100


#endif
