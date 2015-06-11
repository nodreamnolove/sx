/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			TDC256.h
** Last modified Date:  20110517
** Last Version:		1.0
** Descriptions:		����
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

#define		BUF0ADDR			0x0000			//����ģʽ0�Ļ����ӵ�ַ
#define		BUF0ADDR_BK			0x0200
#define		BUF1ADDR			0x3000			//����ģʽ1�Ļ����ӵ�ַ
#define     PARA_ADD            0x2000

//#define		LASTBDADDR			0x4000			//����궨�����ӵ�ַ
//#define		HISTORYBDADDR		0x4800			//��ʷ�궨�����ӵ�ַ
//#define		STARTTIMESADDR		0x5000			//��������				
//#define		VEHBUFINDEXADDR		0x5004			//���泵����Ϣ			
//#define		THRESHOLDADDR		0x5010			//���泵����Ϣ
//
//#define		UPDATERECINDEXADDR	0x5007			//ˢ�����¼���
//#define		STARTRECINDEXADDR	0x5008			//����������¼���	 
//#define		NETINFOADDR			0x5020			//������Ϣ����
//#define		RECBUFBASEADDR		0x5040			//�������ݼ�¼����ַ 0x5040~0x507F,��32*2B=64B
//#define		NEXTRECBUFADDR		0x500a			//�������ݼ�¼��һ�洢λ����� 2B

#define		RESETINFOADDR		0x1500			//20130701hyw��λ��Ϣ�洢��ַ
#define     DEVICECODEADDR      0x1400          //�豸ʶ�����ӵ�ַ	 
#define     STATIONNUMADDR		0x1450
#define		SVWRITESDADD		0x7800				//���ڴ洢uart1������ʱ��SD����д��λ��
#define     LANEDIRADDR         0x1470          //����������

TD_EXT	uint8	ReadC256(uint16 p_u16Addr, uint8 * p_pu8ReadBuf, uint16 p_u16Len);
TD_EXT	uint8	WriteC256(uint16 p_u16Addr, uint8 * p_pu8WriteBuf, uint16 p_u16Len);

#endif		//__TDC256_H
