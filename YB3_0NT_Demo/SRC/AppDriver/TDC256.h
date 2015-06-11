/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			TDC256.h
** Last modified Date:  20110517
** Last Version:		1.0
** Descriptions:		铁电
**
**--------------------------------------------------------------------------------------------------------
** Created by:			ZHANG Ye
** Created date:		20110517
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
#ifndef __TDC256_H

#define __TDC256_H

#ifdef __TDC256_C
#define	TD_EXT
#include "I2C1.h"
#else
#define	TD_EXT	extern
#endif

#include "config.h"

#define		BUF0ADDR			0x0000			//命令模式0的缓存子地址
#define		BUF0ADDR_BK			0x0200
#define		BUF1ADDR			0x3000			//命令模式1的缓存子地址
#define     PARA_ADD            0x2000

//#define		LASTBDADDR			0x4000			//最近标定参数子地址
//#define		HISTORYBDADDR		0x4800			//历史标定参数子地址
//#define		STARTTIMESADDR		0x5000			//启动次数				
//#define		VEHBUFINDEXADDR		0x5004			//缓存车数信息			
//#define		THRESHOLDADDR		0x5010			//缓存车数信息
//
//#define		UPDATERECINDEXADDR	0x5007			//刷程序记录序号
//#define		STARTRECINDEXADDR	0x5008			//程序启动记录序号	 
//#define		NETINFOADDR			0x5020			//网口信息数据
//#define		RECBUFBASEADDR		0x5040			//过车数据记录基地址 0x5040~0x507F,共32*2B=64B
//#define		NEXTRECBUFADDR		0x500a			//过车数据记录下一存储位置序号 2B

#define		RESETINFOADDR		0x1500			//20130701hyw复位信息存储地址
#define     DEVICECODEADDR      0x1400          //设备识别码子地址	 
#define     STATIONNUMADDR		0x1450
#define		SVWRITESDADD		0x7800				//用于存储uart1出单车时，SD卡中写的位置
#define     LANEDIRADDR         0x1470          //车道上下行

TD_EXT	uint8	ReadC256(uint16 p_u16Addr, uint8 * p_pu8ReadBuf, uint16 p_u16Len);
TD_EXT	uint8	WriteC256(uint16 p_u16Addr, uint8 * p_pu8WriteBuf, uint16 p_u16Len);

#endif		//__TDC256_H
