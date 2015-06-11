/****************************************Copyright (c)****************************************************
**                                     BEIJING  WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			Keyboard.c
** Last modified Date:  20110414
** Last Version:		1.0
** Descriptions:		键盘操作驱动
**
**--------------------------------------------------------------------------------------------------------
** Created by:			ZHANG Ye
** Created date:		20110414
** Version:				1.0
** Descriptions:		键盘
**
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
** Version:
** Descriptions:
**
*********************************************************************************************************/
#define	__KEYBOARD_C
#include "Keyboard.h"

#define		KEYD			(1<<11)

#define		GPIO_01			(1<<26)			//GPIO_01  P3口	  数据
#define		GPIO_03			(1<<28)			//GPIO_03  P3口	  时钟
#define		KB_PURSEREG		P3_INP_STATE	//脉冲寄存器,使用该寄存器的KEYD位接收键盘脉冲

static	void	IRQ_Keyboard(void);			//键盘中断处理函数


/*********************************************************************************************************
** Function name:		KeyboardInit
** Descriptions:		键盘初始化
**
** input parameters:	None
** output parameters:	None
** Returned value:		None
**
** Created by:			ZHANG Ye
** Created Date:		20110414
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void KeyboardInit(void)
{
	//按键初始化
	g_u8KeyValueMapped		= 0;
	g_u32KeyValueOri		= 0;
	g_u32KeyCnt				= 0;
	


	P2_MUX_CLR				= 0x02;					//设置GPIO_03为GPIO模式
	P2_DIR_CLR				= GPIO_03 | GPIO_01;	//将引脚设置为输入引脚
	SIC2_ER					|= (1 << 3);
	sic2IrqFuncSet(3	, 0	, (unsigned int)IRQ_Keyboard);		//中断引脚GPIO_03	下降沿触发中断
}

/*********************************************************************************************************
** Function name:		IRQ_Keyboard
** Descriptions:		键盘中断处理
**
** input parameters:	None
** output parameters:	None
** Returned value:		None
**
** Created by:			ZHANG Ye
** Created Date:		20110414
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void IRQ_Keyboard(void)
{
	if((g_u32KeyCnt > 0)&&(g_u32KeyCnt < 9))		//至采集脉冲1~8
	{
		if((KB_PURSEREG & KEYD)!=0)
		{
			g_u32KeyValueOri	+= (1<<(g_u32KeyCnt-1));
		}	
	}
	g_u32KeyCnt	++;



	if (g_u32KeyCnt == 0x0B)
	{	
		if (g_u32KeyValueOri == 0xE0)
		{
			g_u32KeyValueOri	= g_u32KeyValueOri << 8;
			g_u32KeyCnt	= 0;
		}
		else
		{
//			OSSemPost(g_psemKey);
			OSIntNesting++;
	 	    OSIntExit();
		}
	}
}
