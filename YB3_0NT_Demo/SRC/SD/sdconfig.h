/****************************************Copyright (c)**************************************************
**                               Guangzhou ZLG-MCU Development Co.,LTD.
**                                      graduate school
**                                 http://www.zlgmcu.com
**
**--------------File Info-------------------------------------------------------------------------------
** File name:			sdconfig.h
** Last modified Date:	2005-3-11
** Last Version:		V2.0
** Descriptions:		SD/MMC 卡读写模块: 配置文件
**						Soft Packet of SD/MMC Card: configuration header file
**
**------------------------------------------------------------------------------------------------------
** Created by:			Ming Yuan Zheng
** Created date:		2005-1-6
** Version:				V1.0
** Descriptions:		初始版本 The original version
**
**------------------------------------------------------------------------------------------------------
** Modified by:			Ming Yuan Zheng
** Modified date:		2005-3-11
** Version:				V2.0
** Descriptions:		增加了LPC22xx的硬件配置 	
**						add LPC22xx hardware configuration
**------------------------------------------------------------------------------------------------------
** Modified by: 
** Modified date:
** Version:	
** Descriptions: 
**
********************************************************************************************************/

#ifndef __SDCONFIG_H__
#define __SDCONFIG_H__


typedef unsigned char  INT8U;                   /* 无符号8位整型变量                        */
typedef signed   char  INT8;                    /* 有符号8位整型变量                        */
typedef unsigned short INT16U;                  /* 无符号16位整型变量                       */
typedef signed   short INT16;                   /* 有符号16位整型变量                       */
typedef unsigned int   INT32U;                  /* 无符号32位整型变量                       */
typedef signed   int   INT32;                   /* 有符号32位整型变量                       */
typedef float          FP32;                    /* 单精度浮点数（32位长度）                 */
typedef double         FP64;                    /* 双精度浮点数（64位长度）                 */

/* ZLG/SD 软件包的参数配置 */
#define SD_UCOSII_EN			1			/* 是(1)否(0)在UCOS-II上运行本软件包 */

#define SDBUS_MODE_EN           1           /* 是(1)否(0)使能访问卡的SD总线模式 */

#define SD_WIDTHBUS_EN          1           /* 是(1)否(0)使能SD卡的宽总线模式(仅SD总线模式) */

#define SD_CRC_EN		   		1			/* 是(1)否(0)在数据传输时使用CRC校验 */


/* 下列函数, 如果用户不需要, 可置为 0 裁剪指定函数 */

#define SD_ReadMultiBlock_EN    0			/* 是(1)否(0)使能读多块函数 */

#define SD_WriteMultiBlock_EN   0			/* 是(1)否(0)使能写多块函数 */

#define SD_EraseBlock_EN		1			/* 是(1)否(0)使能擦卡函数 */


/* 包含本软件包的所有头文件 */
#include "config.h"
#include "sdcommon.h"

#if SDBUS_MODE_EN
#include "sdhal.h"
#else
#include "sdspihal.h"
#endif

#include "sdcrc.h"
#include "sdcmd.h"
#include "sddriver.h"

#endif
