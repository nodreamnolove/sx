/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			Keyboard.h
** Last modified Date:  20110414
** Last Version:		1.0
** Descriptions:		¼üÅÌ²Ù×÷Çý¶¯
**
**--------------------------------------------------------------------------------------------------------
** Created by:			ZHANG Ye
** Created date:		20110414
** Version:				1.0
** Descriptions:		¼üÅÌ
**
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
** Version:
** Descriptions:
**
*********************************************************************************************************/
#ifndef	__KEYBOARD_H
#define	__KEYBOARD_H

#ifdef __KEYBOARD_C
#define	KB_EXT
#else
#define	KB_EXT	extern
#endif

#include "config.h"

KB_EXT	void	KeyboardInit(void);
		
#endif		//__KEYBOARD_H
