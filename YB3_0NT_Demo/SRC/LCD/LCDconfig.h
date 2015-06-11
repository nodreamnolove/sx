/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			LCDconfig.h
** Last modified Date:  20110418
** Last Version:		1.0
** Descriptions:		标准显示屏LCD各种头文件
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
#define	__LCDCONFIG_H

#ifdef	__LCDDRIVE_C
#define	LCDCFG_EXT	
#else
#define	LCDCFG_EXT	extern
#endif


#include	"config.h"

#include    "GUI_CONFIG.H"
#include    "LCDDRIVE.H"
#include    "GUI_BASIC.H"
#include    "GUI_STOCKC.H"
#include    "FONT_MACRO.H"
#include    "FONT5_7.H"
#include    "FONT8_8.H"
#include    "FONT24_32.H"
#include    "LOADBIT.H"
#include    "WINDOWS.H"
#include    "MENU.H"
#include    "SPLINE.H"

LCDCFG_EXT	uint8	g_u8PosX;
LCDCFG_EXT	uint8	g_u8PosY;
