/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			UI.h
** Last modified Date:  20110505
** Last Version:		1.0
** Descriptions:		�������н��溯��
**
**--------------------------------------------------------------------------------------------------------
** Created by:			ZHANG Ye
** Created date:		20110505
** Version:				1.0
** Descriptions:		The original version
**
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
** Version:
** Descriptions:
**
*********************************************************************************************************/
#ifndef	__UI_H
#define	__UI_H

#include "Config.h"

#ifdef	__UI_C
#define		UI_EXT
#include "Common.h"
#include "TDC256.h"

#include "KBMacro.h"
#include "UIOperation.h"
#include "LCDApp.h"

#if	YBVERSION >= 30		//3.0�Ǳ���				  
#include "StorageApp.h"
#else	//2.2
#include "ISP.h"
#endif	//#if	YBVERSION >= 30		//3.0�Ǳ���

#else		//__UI_C
#define		UI_EXT		extern
#endif

UI_EXT	void	SaveParams(void);			//�����������

UI_EXT	void	UIStartUp(void);			//��������

UI_EXT	void	UIGeneral(void);			//ͨ�ù�������
UI_EXT	void	UIValidate(void);			//ˢ�������֤����
UI_EXT	void	UISN(void);					//SN�����

UI_EXT	void	UIBDRoot(void);				//�궨������

UI_EXT	void	UIBDMain(uint8 p_u8Motion);	//����̬�궨����
//UI_EXT	void	UIBDStatic(void);			//��̬�궨����
UI_EXT	void	UISystemInit(void);			//ϵͳ��ʼ������

UI_EXT	void	UIBDWanBanChoose(uint8 p_u8Motion);				//���궨ͨ��ѡ�����
UI_EXT	void	UIBDWanBan(uint8 p_u8CID, uint8 p_u8Motion);	//���궨����
UI_EXT	void	UIBDChengTai(uint8 p_u8CID, uint8 p_u8Motion);	//��̨�궨����	   
UI_EXT	void	UIBDChengTaiAll(uint8 p_u8Motion);				//��̨�궨����:����̨��Ӻ�ֵ��̨���ѡ��0
UI_EXT	void	UIBDChengtaiChoose(uint8 p_u8Motion);			//��̨̨��ѡ����� 
UI_EXT	void	UIBDScale(uint8 p_u8Motion);					//���÷ֶ�

UI_EXT	void	UIBDGenZong(void);			//������ʹ�ܽ���
UI_EXT	void	UIBDPoDu(void);				//�¶���������
UI_EXT	void	UIBDLunZhou(void);			//���������ʹ�ܽ���
#if	SENDWAVEENABLE > 0		//ʹ�ܷ�����
UI_EXT	void	UIBDSendWave(void);			//���Ͳ���ʹ�ܽ���
#endif
UI_EXT	void	UIBDFullRange(void);		//����������� 
UI_EXT	void	UIBDStaticModify(void);		//���þ�̬�������棬���մ�������������
UI_EXT	void	UIBDChooseMotion(void);		//��̬����ѡ����棬����or����
UI_EXT	void	UIBDLineModify(void);		//������������
UI_EXT	void	UIBDPointModify(void);		//����������
UI_EXT	void	UIBDChooseVehPos(void);		//�����ٶ�����λ��ѡ��
//UI_EXT	void	UIBDVehSpeedModifyAB(void);	//�����ٶ�����AB��
//UI_EXT	void	UIBDVehSpeedModifyBC(void);	//�����ٶ�����BC��
//UI_EXT	void	UIBDVehSpeedModifyGap(void);//�����ٶ�����ѹ�� 
//UI_EXT	void	UIBDVehModify(void);		//������������
//UI_EXT	void	UIBDVehGap(void);			//��������ѹ������
UI_EXT	void	UIBDChooseVeh(uint8 p_u8Pos);	//ѡ���ͽ��棬����ѹ��λ�ò�ͬ���鿴�����ݲ�ͬ
UI_EXT	void	UIBDVSModifyParam(uint8 p_u8Pos, uint8 p_u8Veh);	//����ֵ���棬����ѹ��λ�úͳ���


UI_EXT	void	UICommonSet(void);			//��ͨ���ý��棬111
UI_EXT	void	UIViewSetting(void);		//�鿴���ò������棬222
UI_EXT	void	UIViewModify(void);			//�鿴�ٶ��������棬333
UI_EXT	void	UIViewAuthor(void);			//�鿴������Ϣ���棬888
UI_EXT	void	UIViewThreshold(void);		//��ֵ������8968
#if	YBVERSION >= 30		//3.0�Ǳ���
UI_EXT	void	UIViewIPInfo(void);			//�鿴IP��Ϣ��444
UI_EXT	void	UIViewStartUpTime(void);	//�鿴����ʱ����Ϣ��8494
#endif													
		
UI_EXT	void	UIF3Code(void);				//�鿴F3�������
UI_EXT	void	UIF4Code(void);				//�鿴F4�������
UI_EXT	void	UIF5Code(void);				//�鿴F5�������

UI_EXT	void	UISetBaudRate(void);		//���ò����ʽ���
UI_EXT	void	UISetTime(void);			//����ʱ�����
UI_EXT	void	UISetCommandMode(void);		//��������ģʽ����
UI_EXT	void	UISetForwardEnable(void);	//���÷���ʹ�ܽ���
UI_EXT	void	UISetProtocol(void);		//Э�����
UI_EXT	void	UISetLoop(void);			//��Ȧʹ�ܽ���
//UI_EXT	void	UISetPassword(void);		//�޸��������
UI_EXT	void	UISetCapture(void);			//����ץ��ʹ�ܽ���
UI_EXT	void	UISetPlat(void);			//����̨���Ƚ���
UI_EXT	void	UISetVehicleCache(void);	//���ó�������
UI_EXT	void	UISetDiaodian(void);		//���õ��籣������
UI_EXT	void	UISetDog(void);				//���ÿ��Ź�����
UI_EXT	void	UISetLunZhouEnable(void);	//�������������ʾʹ�ܽ���
#if	YBVERSION >= 30		//3.0�Ǳ���
UI_EXT	void	UISetValueParamIP(char * p_pcName, uint8 * p_pu8IP);	//����IP
#endif


//UI_EXT	void	UIBDStaticWanBan(uint8	p_u8CID);	//��徲̬�궨���棬���ھ�̬����
//UI_EXT	void	UIBDStaticChengTai(uint8 p_u8CID);	//��̨��̬�궨���棬���ھ�̬����  
//UI_EXT	void	UIBDStaticChengtaiChoose(void);
//UI_EXT	void	UIBDStaticScale(void);				//���þ�̬�ֶ� 
//UI_EXT	void	UIBDStaticWanBanChoose(void);		//���궨ͨ��ѡ�����

#endif		//__UI_C
