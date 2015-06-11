/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			LCDApp.h
** Last modified Date:  20110418
** Last Version:		1.0
** Descriptions:		LCDӦ�ó���
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

LCDAPP_EXT	void	BackGroundON(void);				//��������
LCDAPP_EXT	void	BackGroundOFF(void);			//�رձ���
LCDAPP_EXT	void	BackGroundReverse(void);		//��ת������
LCDAPP_EXT	void	BackGroundSave(void);			//���汳���״̬
LCDAPP_EXT	void	BackGroundRecover(void);		//�ָ������״̬

LCDAPP_EXT	void	CheckOutBGStatus(void);			//����ⲿ�����ƿ���

LCDAPP_EXT	void	Enter(void);							//��ʾ�����������������һ�п�ʼλ��
LCDAPP_EXT	void	GotoXY(uint8 p_u8X, uint8 p_u8Y);		//��ʾ���������������ָ��λ��

//�ڵ�ǰ���������ַ�
LCDAPP_EXT	void	PrintChar(char * p_cStr);				
//��ָ�����������ַ�
LCDAPP_EXT	void	PrintCharXY(uint8 p_u8X, uint8 p_u8Y, char * p_cStr);		
//�ڵ�ǰ����㰴��һ����ʽ�������
LCDAPP_EXT	void	PrintFormatData(char * p_cFormat, uint32 p_u32Data);					
//��ָ�����������ַ�
LCDAPP_EXT	void	PrintFormatDataXY(uint8 p_u8X, uint8 p_u8Y, char * p_cFormat, uint32 p_u32Data);

//��ָ�����������ַ���������ͼ��
LCDAPP_EXT	void	DrawLineModify(uint16 *p_pModifyParam);


LCDAPP_EXT	void	DrawPic(void);			//��ʾͼƬ

LCDAPP_EXT	void	LCDInit(void);			//��ʼ��
		
#endif
