/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			AdjustMacro.h
** Last modified Date:  20110531
** Last Version:		1.0
** Descriptions:		��УЭ������궨��
**
**--------------------------------------------------------------------------------------------------------
** Created by:			ZHANG Ye
** Created date:		20110531
** Version:				1.0
** Descriptions:		��УЭ������궨��
**
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
** Version:
** Descriptions:
**
*********************************************************************************************************/
#ifndef	__ADJUSTMACRO_H
#define	__ADJUSTMACRO_H

//����֡
//	0xFF 00 [LEN] [COMM] [PARAM1] [PARAM2] [PARAM3] [VALUE1] [VALUE2] [VALUE3] [VALUE4] [CRC1] [CRC2]

//����궨��	COMM
#define		ADJ_SAVE				0xA1
#define		ADJ_GETALLPARAM			0xA2
#define		ADJ_MODIFYPARAM			0xA3	//��ʱȡ��������
#define		ADJ_GETNONEWEIGHT		0xA4		//��ȡ���з�������������
#define		ADJ_UPDATENONEWEIGHT	0xA5		//�������з�������������
  
#define		ADJ_GETSTATICWEIGHT		0xA6		//��ȡ���о�̬��������
#define		ADJ_UPDATESTATICWEIGHT	0xA7		//�������о�̬��������

#define		ADJ_GETMOTIONWEIGHT		0xA8		//��ȡ���ж�̬��������
#define		ADJ_UPDATEMOTIONWEIGHT	0xA9		//�������ж�̬��������

#define		ADJ_UPDATEALL		0xA0		//��������SETUP

#define		ADJ_READVEHNUM		0x04		//�����泵��
#define		ADJ_DEVICESTATUS	0x05		//�豸״̬
#define		ADJ_DELETEFIRST		0x03		//ɾ�׳�
#define		ADJ_DELETELAST		0x06		//ɾβ��	
#define		ADJ_DELETENO		0x12		//ɾ��ָ������
#define		ADJ_SYNCHRONY		0x07		//ͬ��
#define		ADJ_RESEND			0x0A		//�ط�
#define		ADJ_CAPTURE			0x13		//ץ��
#define		ADJ_ManualShouwei	0x14		//ǿ����β
#define		ADJ_LOOPTRIGGER		0x21		//��Ȧ����

#define		ADJ_TIME			0x3A		//ϵͳУʱ
#define		ADJ_INITWITHCACHE	0x55		//���������ʼ��	
#define		ADJ_INITNOCACHE		0x54		//�����������ʼ��

#define		ADJ_SENDAXLE		0x01		//����������
#define		ADJ_SENDAXLEGRP		0x00		//������������

#define		ADJ_SLAVESTART		0x09		//�ӻ��ϵ�	 
#define		ADJ_QINGLING		0x32		//����

//�޸Ĳ���ʱ������1�Ĳ�������

#define		PRM_ZERO			0x01		//���
#define		PRM_GAIN			0x02		//����
#define		PRM_STATICMODIFY	0x03		//��̬����
#define		PRM_SPEEDMODIFY		0x04		//�ٶ�����
	
#define		PRM_PWD				0x05		//����
#define		PRM_FULL			0x06		//����
#define		PRM_PODU			0x07		//�¶� 
#define		PRM_STATICSCALE		0x08		//��̬�ֶ�
#define		PRM_MOTIONSCALE		0x09		//��̬�ֶ�
#define		PRM_GENZONG			0x0A		//������
#define		PRM_COMMODE			0x0B		//����ģʽ
#define		PRM_BAUDRATE		0x0C		//������
#define		PRM_DOG				0x0D		//���Ź�	

//һ�㲻�޸�ʱ��
#define		PRM_YEAR			0x0E		//�޸�ʱ�䣬��
#define		PRM_MONTH			0x0F		//�޸�ʱ�䣬��
#define		PRM_DAY				0x10		//�޸�ʱ�䣬��

#define		PRM_PLATWIDTH		0x11		//̨����
#define		PRM_FANGXIANG		0x12		//����ʹ��
#define		PRM_DIAODIAN		0x13		//���籣����ʶ	
#define		PRM_CAPTURE			0x14		//ץ��ʹ��
#define		PRM_LOOP			0x15		//��Ȧ����ʹ��

#define		PRM_VEHTOTALM		0x16		//������������
#define		PRM_VEHSPEEDM		0x17		//�����ٶ�����
#define		PRM_PROTOCOL		0x18		//Э��
													   
#define		PRM_BUFSIZE			0x19		//���泵��
#define		PRM_LUNZHOUERR		0x1A		//���ᱨ��ʹ��
#define		PRM_LUNZHOUPROG		0x1B		//���������ʹ�� 
#define		PRM_SENDWAVE		0x1C		//������ʹ��


#endif	//__ADJUSTMACRO_H
