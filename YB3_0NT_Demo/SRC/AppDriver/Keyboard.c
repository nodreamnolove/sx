/****************************************Copyright (c)****************************************************
**                                     BEIJING  WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			Keyboard.c
** Last modified Date:  20110414
** Last Version:		1.0
** Descriptions:		���̲�������
**
**--------------------------------------------------------------------------------------------------------
** Created by:			ZHANG Ye
** Created date:		20110414
** Version:				1.0
** Descriptions:		����
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

#define		GPIO_01			(1<<26)			//GPIO_01  P3��	  ����
#define		GPIO_03			(1<<28)			//GPIO_03  P3��	  ʱ��
#define		KB_PURSEREG		P3_INP_STATE	//����Ĵ���,ʹ�øüĴ�����KEYDλ���ռ�������

static	void	IRQ_Keyboard(void);			//�����жϴ�����


/*********************************************************************************************************
** Function name:		KeyboardInit
** Descriptions:		���̳�ʼ��
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
	//������ʼ��
	g_u8KeyValueMapped		= 0;
	g_u32KeyValueOri		= 0;
	g_u32KeyCnt				= 0;
	


	P2_MUX_CLR				= 0x02;					//����GPIO_03ΪGPIOģʽ
	P2_DIR_CLR				= GPIO_03 | GPIO_01;	//����������Ϊ��������
	SIC2_ER					|= (1 << 3);
	sic2IrqFuncSet(3	, 0	, (unsigned int)IRQ_Keyboard);		//�ж�����GPIO_03	�½��ش����ж�
}

/*********************************************************************************************************
** Function name:		IRQ_Keyboard
** Descriptions:		�����жϴ���
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
	if((g_u32KeyCnt > 0)&&(g_u32KeyCnt < 9))		//���ɼ�����1~8
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
