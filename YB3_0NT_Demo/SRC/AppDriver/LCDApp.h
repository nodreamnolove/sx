/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			LCDApp.h
** Last modified Date:  20110418
** Last Version:		1.0
** Descriptions:		LCD应用程序
**
**--------------------------------------------------------------------------------------------------------
** Created by:			ZHANG Ye
** Created date:		20110418
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
#ifndef	__LCDAPP_H
#define	__LCDAPP_H

#ifdef	__LCDAPP_C
#define	LCDAPP_EXT	
#else
#define	LCDAPP_EXT	extern
#endif

#include "Config.h"

///////////////////////////////////////
#include "LCDconfig.h" 

LCDAPP_EXT	void	BackGroundON(void);				//点亮背景
LCDAPP_EXT	void	BackGroundOFF(void);			//关闭背景
LCDAPP_EXT	void	BackGroundReverse(void);		//反转背景灯
LCDAPP_EXT	void	BackGroundSave(void);			//保存背光灯状态
LCDAPP_EXT	void	BackGroundRecover(void);		//恢复背光灯状态

LCDAPP_EXT	void	CheckOutBGStatus(void);			//检查外部背景灯控制

LCDAPP_EXT	void	Enter(void);							//显示屏输入点坐标跳至下一行开始位置
LCDAPP_EXT	void	GotoXY(uint8 p_u8X, uint8 p_u8Y);		//显示屏输入点坐标跳至指定位置

//在当前坐标点输出字符
LCDAPP_EXT	void	PrintChar(char * p_cStr);				
//在指定坐标点输出字符
LCDAPP_EXT	void	PrintCharXY(uint8 p_u8X, uint8 p_u8Y, char * p_cStr);		
//在当前坐标点按照一定格式输出数字
LCDAPP_EXT	void	PrintFormatData(char * p_cFormat, uint32 p_u32Data);					
//在指定坐标点输出字符
LCDAPP_EXT	void	PrintFormatDataXY(uint8 p_u8X, uint8 p_u8Y, char * p_cFormat, uint32 p_u32Data);

//在指定坐标点输出字符绘制线修图像
LCDAPP_EXT	void	DrawLineModify(uint16 *p_pModifyParam);


LCDAPP_EXT	void	DrawPic(void);			//显示图片

LCDAPP_EXT	void	LCDInit(void);			//初始化
		
#endif
