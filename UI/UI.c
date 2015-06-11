/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			UI.c
** Last modified Date:  20110506
** Last Version:		1.0
** Descriptions:		���н��溯��
**
**--------------------------------------------------------------------------------------------------------
** Created by:			ZHANG Ye
** Created date:		20110506
** Version:				1.0
** Descriptions:		
**
**--------------------------------------------------------------------------------------------------------
** Modified by:			ZHANG Ye			
** Modified date:		20110510
** Version:				1.1
** Descriptions:		��ͨ˫���
**
*********************************************************************************************************/
#define	__UI_C
#include "UI.h"

static	uint8	m_u8Err;
char	m_acTmp[30];		//��ʾ��һ��

//ȫ�ֱ�������
#define		SETUPALIAS				g_sspSetup			//���ò����ṹ
#define		SNALIAS					g_u32SN				//SNֵ
#define		TMPTIMEALIAS			g_sstTempTime		//ʱ��ṹ��ʱ����
#if	YBVERSION >= 30		//3.0�Ǳ���
#define		IPINFOALIAS				g_sniLocal			//IP��Ϣ����
#endif
#define		CURTIMEALIAS			g_sstCurTime		//��ǰʱ��
#define		THRESHOLDALIAS			g_sudtThreshold		//��ֵ
#define		LZSIGNAL				g_u32LunZhouSignal	//�����ź�	
#define		LZSIGNALLast			g_u32LunZhouSignalLast	//��һ�������ź�
#define		ADAvg					g_an32AvgAD				//ƽ��ADֵ
#define		PASSOVERINFO			g_sviVehicleInfo		//����������Ϣ
#define		ERRALIAS				g_u8DeviceERR			//���ϴ���

#define		YBVERALIAS				YBVERSION			//�Ǳ�汾��
#define		VERSIONALIAS			PROGRAMVERSION		//�汾��

#define		LCDPosX					X					//��ʾ��X����
#define		LCDPosY					Y					//��ʾ��Y����

#define		LCDPRINTC				PrintCharXY			//��ʾ�ַ���
#define		LCDPRINTFC				PrintFormatDataXY	//��ʾһ����ʽ���ַ���

#define		BGON					BackGroundON		//��������
#define		BGOFF					BackGroundOFF		//��������
#define		BGConvert				BackGroundReverse	//�����Ʒ�ת

#define		GUANGSHANStatus			GSTriggerPIN		//��դ״̬
#define		LOOPStatus				LPTriggerPIN		//��Ȧ״̬
#define		GUANGSHANErr			GSErrPIN			//��դ����
#define		LOOPErr					LPErrPIN			//��Ȧ����

////////////////////////////////////
//��ѯʱ��(��λ10ms)
#define		LOOPTIME				1

#define		WTD_CT					0x01				//��̨
#define		WTD_2WB					0x02				//2���
#define		WTD_3WB					0x06				//3���

#define		WTDEVICE				WTD_2WB
		
#define		IF3WB					WTDEVICE & 0x04		

//�����ֵ
#define		KeyValue				g_u8KeyValueMapped
#define		ClearKeyValue()			KeyValue	= 0xff

#define		ClearLCDLine(X)			LCDPRINTC(0		, X	, "                              ")

//����ָ��:0x01:�ػ���0x02���˳�					
#define		ResetControlCode(ControlCode)	ControlCode = 0x00

#define		IfReDraw(ControlCode)	ControlCode & 0x01
#define		ToReDraw(ControlCode)	ControlCode |= 0x01
#define		NotReDraw(ControlCode)	ControlCode &= ~0x01

#define		IfBreak(ControlCode)	ControlCode & 0x02
#define		ToBreak(ControlCode)	ControlCode |= 0x02
#define		NotBreak(ControlCode)	ControlCode &= ~0x02

//�ȴ���Ļˢ���ź�
#define		WAITSCREENREFRESH()		OSSemPend(g_psemScreenRefresh,0, &m_u8Err)

//ѹ��λ��
#define		POS_ABS		1			//AB�ٶ�
#define		POS_BCS		2			//BC�ٶ�
#define		POS_ABCS	3			//ѹ���ٶ�
#define		POS_VEH		4			//��������
#define		POS_GAP		5			//ѹ������
#define		POS_2WB		6			//��������

//����̬
#define		UI_STATIC	0
#define		UI_MOTION	1

//����ʹ�ܲ�����uint8����
static	void UISetEnableParam(char * p_pcName, uint8 * p_pu8Param);

//uint8,uint16,uint32��������
static	void UISetValueParamU8(char * p_pcName, uint8 * p_pu8Param, uint32 p_u32Min, uint32 p_u32Max);
static	void UISetValueParamU16(char * p_pcName, uint16 * p_pu16Param, uint32 p_u32Min, uint32 p_u32Max);
#if	YBVERSION >= 30		//3.0�Ǳ���
static	void UISetValueParamU32(char * p_pcName, uint32 * p_pu32Param, uint32 p_u32Min, uint32 p_u32Max);
#endif	//#if	YBVERSION >= 30		//3.0�Ǳ���

/*********************************************************************************************************
** Function name:		UIStartUp
** Descriptions:		�������棬����Ҫѭ����ʾ�����һ��ʱ���ر�
** input parameters:	None 
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110506	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void UIStartUp(void)
{
	GUI_ClearSCR();								//����
	DrawPic();									//��ʾͼƬ

	LCDPRINTC(130	, 72	, VERSIONALIAS	);	//��ʾ�汾��Ϣ

//	BeepON();			//��������
//	
//	OSTimeDly(10);	
//	
//	BeepOFF();			//��������	      
	
	LEDON();
}

/*********************************************************************************************************
** Function name:		UIGeneral
** Descriptions:		��׼���̽��棬һֱ��ʾ
** input parameters:	None 
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110506	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void UIGeneral(void)
{
	uint8	l_u8ControlCode;		//����ָ��
	int32	l_n32TempWeight;		//��ʱ����
	uint32	l_u32Temp1;				//��ʱ����1
	int32	l_n32TotalWeight;		//����
	uint32	l_u32Temp2,l_u32Temp3;	//��ʱ����2,3
	uint8	l_u8k;					//ѭ���ۼӱ���
	uint8	l_u8FindFirstData;
	uint8	l_u8EraseStatus;		//����״̬ 0��������Ϊ�գ� 1��������ȫ���� 2����ʾ����Ϣ; 3:ֻ��һ��
	char 	l_cTaiXing[20];
	uint8	l_u8TaiXingIndex;
	int32	l_n32TmpStaticWeight[CHANNELNUM];
	int32	l_n32StaticXiuIndex;
	uint32	l_u32TimeCnt;
	
	uint8	l_u8ReShowVeh;			//������ʾ״̬: 0x10��������ʾ(��������ʾ)��0x11������ʾ��0x00ȡ��������ʾ

	void *	l_pTmp;
	AxleShow	l_asTmp;
	
	AddStartupCnt();	//��¼��������
	
	l_u8ReShowVeh	= 0x00;
	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();
	memset(m_acTmp, 0, 30);

	while(1)
	{
		l_u32TimeCnt	= 0;
		l_u8EraseStatus	= 0;

		//������
		GUI_ClearSCR();
		LCDPRINTC(0		, 0		, "����  "	);
		LCDPRINTC(56	, 0		, "������"	);
		LCDPRINTC(128	, 0		, "����  "	); 
		LCDPRINTC(192	, 0		, "����  "	);

		LCDPRINTC(80	, 115	, "C"	);
#if	YBVERSION < 30		//2.2�Ǳ���
		LCDPRINTC(64	, 115	, "00"	);
#endif	//#if	YBVERSION < 30		//2.2�Ǳ���
		LCDPRINTC(96	, 115	, "E"	);
		LCDPRINTC(128	, 115	, "BUF"	);	
		LCDPRINTC(192	, 115	, ":"	);
		LCDPRINTC(216	, 115	, ":"	);

		GUI_Line(0		, 16	, 239	, 16	, 1	);
		GUI_Line(0		, 114	, 239	, 114	, 1	);

		while (1)		//�жϰ�����ˢ������
		{
			//�жϰ���
			switch(KeyValue)
			{
				case KB_ESC:
					ToReDraw(l_u8ControlCode); 
					l_u8ReShowVeh	= 0x00;		//��ֹ������ʾ
					UISN();
					break;

				case KB_F1:		//����
					NotReDraw(l_u8ControlCode);
					CLEARZERO();
					break;	
				
				case KB_F2:		//����̬�л�
					NotReDraw(l_u8ControlCode);
					g_u8StaticMotionswitch	= (g_u8StaticMotionswitch + 1) & 0x01;
					break;	
				
				case KB_F3:		//������Ϣ
					ToReDraw(l_u8ControlCode);	  
					l_u8ReShowVeh	|= 0x01;	//����������ʾ
					UIF3Code();
					break;	
				
				case KB_F4:		//��������
					ToReDraw(l_u8ControlCode);	  
					l_u8ReShowVeh	|= 0x01;	//����������ʾ
					UIF4Code();
					break;	
				
				case KB_F5:		//����״̬
					ToReDraw(l_u8ControlCode);	  
					l_u8ReShowVeh	|= 0x01;	//����������ʾ
					UIF5Code();
					break;	
				
				case KB_F6:		//��ʾ�������ƿ���
					NotReDraw(l_u8ControlCode);
					BGConvert();	
					break;	
				
				default:
					NotReDraw(l_u8ControlCode);
					break;
			}	//switch(KeyValue)
			ClearKeyValue();
					
			//�Ƿ��ػ�
			if (IfReDraw(l_u8ControlCode))
				break;

			//ˢ����ʾ
			if (OSSemAccept(g_psemSystemTime)>0)		//���ź��������벢��ʾ����
			{
				//E/BUF
#if	YBVERSION >= 30		//3.0�Ǳ���				
				LCDPRINTFC(64	, 115	, "%02D"	, g_u32Temprature);
#endif
				LCDPRINTFC(104	, 115	, "%02X "	, ERRALIAS);
				LCDPRINTFC(152	, 115	, "%02d "	, g_u8CurVehNum);
				
				//ʱ��
				LCDPRINTFC(176	, 115	, "%0.2d"	, CURTIMEALIAS.u8Hour);
				LCDPRINTFC(200	, 115	, "%0.2d"	, CURTIMEALIAS.u8Minute);		
				LCDPRINTFC(224	, 115	, "%0.2d"	, CURTIMEALIAS.u8Second);
				
				//��դ����Ȧ״̬
				if(GUANGSHANStatus==0)				
					LCDPRINTC(232	, 0		, "*");
				else 
					LCDPRINTC(232	, 0		, " ");
				if(LOOPStatus==0)
					LCDPRINTC(224	, 0		, "@");
				else 
					LCDPRINTC(224	, 0		, " ");

				//��ʾ˲ʱֵ,�Դ�������̬����
				l_n32TempWeight	= 0;
				for(l_u8k = 0;l_u8k < CHANNELNUM; l_u8k++)
				{
					l_n32TmpStaticWeight[l_u8k] = (ADAvg[l_u8k] - g_an32MotionZero[l_u8k]) * SETUPALIAS.an32AxGain[l_u8k]/10000;
					l_n32StaticXiuIndex = StaticXiuZhengIndex(l_n32TmpStaticWeight[l_u8k]);
					l_n32TmpStaticWeight[l_u8k]	= l_n32TmpStaticWeight[l_u8k] * (int32)SETUPALIAS.au16StaticModify[l_u8k][l_n32StaticXiuIndex]/10000;
				
					l_n32TempWeight	+= l_n32TmpStaticWeight[l_u8k]; 
				}				
			
				if(g_u8StaticMotionswitch == 1)		//��̬����
				{
					if (l_u32TimeCnt > 100)
					{
						if(l_n32TempWeight < (int32)SETUPALIAS.u8MotionScale)	   			//С�ڶ�̬�ֶ�ֵ
					    	CLEARZERO();
						l_u32TimeCnt	= 0;
					}
					l_n32TempWeight	= ScaleUp(l_n32TempWeight, SETUPALIAS.u8MotionScale);
					LCDPRINTFC(0		, 115	, "D%-6d"	, l_n32TempWeight);
				}
				else		//��̬����
				{
					if (SETUPALIAS.u8Genzong > 0)
					{
						if (l_u32TimeCnt > 100)
						{
							l_u32TimeCnt	= 0;
							if(l_n32TempWeight < (int32)SETUPALIAS.u8StaticScale)	   			//С�ھ�̬�ֶ�ֵ
						    	CLEARZERO();
						}	
					}
					l_n32TempWeight	= ScaleUp(l_n32TempWeight, SETUPALIAS.u8StaticScale);
				   	LCDPRINTFC(0	, 115	, "S%-6d"	, l_n32TempWeight);					
				}
				
				if(l_n32TempWeight > (int32)SETUPALIAS.u32Full) 
				{
					LCDPRINTC(0		, 115	, "Over   ");
				}
			}
			
			l_pTmp	= OSQAccept(g_pqueAxleShow);		//�������̣�����Ϣ
			if (l_pTmp!= (void *)0)		//��Ϣ����������Ϣ�����벢��ʾ����
			{
				switch (l_u8EraseStatus)
				{
					case 1:	//ȫ������Ҫȫ������
						ClearLCDLine(18);
						ClearLCDLine(34);
						ClearLCDLine(50);
						ClearLCDLine(66);
						ClearLCDLine(81);
						ClearLCDLine(97);
						break;

					case 2:	//������Ϣ
						ClearLCDLine(18);
						ClearLCDLine(34);
						ClearLCDLine(50);
						break;

					case 3:	//������Ϣ
						ClearLCDLine(18);
						break;

					default:
						break;
				}
				l_u8EraseStatus	= 0;	  
				l_u8ReShowVeh	= 0x00;	//�������ᣬֹͣ������ʾ��������

				memcpy(&l_asTmp, (AxleShow *)l_pTmp,sizeof(AxleShow));

				//��ʾ
				switch((l_asTmp.u8Direct) & 0x7f)
				{
					case 0:
							LCDPRINTFC(0		, 18	, "%d.  ", l_asTmp.u8AxleID);	//1.
							LCDPRINTC(24		, 18	, "+");	//+	��ʾ����
#if ISDEBUG
							if (l_asTmp.n32AxleWeight != 0)
							{
								LCDPRINTFC(24		, 34	, "WEIGHT: %d.  "	, l_asTmp.n32AxleWeight);		//����
								LCDPRINTFC(24		, 50	, "MAX   : %d.  "	, l_asTmp.n32AxleMaxWeight);	//���ֵ
								LCDPRINTFC(144		, 50	, "AT : %d.  "		, l_asTmp.u16ArchiveTimes);		//���ֵ
							}
#endif
							l_u8EraseStatus	= 2;
							break;
					case 1:
							LCDPRINTFC(0		, 18	, "%d.  ", l_asTmp.u8AxleID);	//1.
							LCDPRINTC(24		, 18	, "-");	//- ��ʾ����
							l_u8EraseStatus	= 3;
							break;
					case 2:
							LCDPRINTC(24		, 18	, ">");	//> �����ϡ�Ȼ�󵹻�
							l_u8EraseStatus	= 3;
							break;
					case 3:
							LCDPRINTC(24		, 18	, "<");	//< ���򵹡�Ȼ��ǰ���˳�
							l_u8EraseStatus	= 3;
							break;
					default:
							break;
				}

				//����ר��
				if (l_asTmp.u8Direct >> 7)		//2̨��
					LCDPRINTC(40		, 18	, "B");	//B
				else							//1̨��
					LCDPRINTC(40		, 18	, "A");	//A
			}
			
//			if (OSSemAccept(g_psemVehicleInfo)>0)		//��ʾ������������
			if ((OSSemAccept(g_psemVehicleInfo)>0)||(l_u8ReShowVeh==0x11))		//��ʾ������������
			{						   
				l_u8ReShowVeh	= 0x10;

				switch (l_u8EraseStatus)
				{
					case 1:	//ȫ������Ҫȫ������
						ClearLCDLine(18);
						ClearLCDLine(34);
						ClearLCDLine(50);
						ClearLCDLine(66);
						ClearLCDLine(81);
						ClearLCDLine(97);
						break;

					case 2:	//������Ϣ
						ClearLCDLine(18);
						ClearLCDLine(34);
						ClearLCDLine(50);
						break;

					case 3:	//������Ϣ
						ClearLCDLine(18);
						break;

					default:
						break;
				}
				l_u8EraseStatus		= 0;

				l_n32TotalWeight	= 0;
				for (l_u32Temp1 = 0; l_u32Temp1 < PASSOVERINFO.u8AxleTotalNum; l_u32Temp1 ++)
				{
					LCDPRINTFC(0		, l_u32Temp1*13+18	, "%d."		,l_u32Temp1 + 1);	
					LCDPRINTFC(120		, l_u32Temp1*13+18	, "%dkg"	,PASSOVERINFO.an32AxleWeight[l_u32Temp1]*10);
					LCDPRINTFC(192		, l_u32Temp1*13+18	, "%dkm/h"	,PASSOVERINFO.au16AxleSpeed[l_u32Temp1]);
					
					if (l_u32Temp1 == 5)	//���ֻ��ʾ6��
						break;
				}

				for (l_u32Temp1 = 0; l_u32Temp1 < PASSOVERINFO.u8AxleGrpTotalNum; l_u32Temp1 ++)
				{
					l_n32TotalWeight	+= PASSOVERINFO.an32AxleGrpWeight[l_u32Temp1]*10;
					
					//�޸ģ���6�����˳���Ϊ6��������ֻ����������ʾ
					if (l_u32Temp1 < 6)	//���ֻ��ʾ6��
						LCDPRINTFC(56		, l_u32Temp1*13+18	, "%dkg"	,PASSOVERINFO.an32AxleGrpWeight[l_u32Temp1]*10);
					
				}
				
				//����
				LCDPRINTC(0		, 97	, "����: "	);
				LCDPRINTFC(40	, 97	, "%dkg "	,l_n32TotalWeight);
				
				//����	
				LCDPRINTC(112	, 97	, "����:");

				if(PASSOVERINFO.u8AxleGrpTotalNum < 7) 
				{
					l_u8TaiXingIndex = 0;
					for(l_u32Temp1=0;l_u32Temp1<PASSOVERINFO.u8AxleGrpTotalNum;l_u32Temp1++)
					{
						l_u32Temp2 = (PASSOVERINFO.au8AxleTAIType[l_u32Temp1] & 0xF0)>>4;	//ȡ��λ��������λ;
						l_u8FindFirstData = 0;
						l_u32Temp3 = 0x08;
						for(l_u8k = 0;l_u8k < 4; l_u8k++)
						{
							if((l_u32Temp2&l_u32Temp3)>0 && (l_u8FindFirstData ==0))
							{
								l_u8FindFirstData = 1;	
							}
							if(l_u8FindFirstData == 1)
							{
								if((l_u32Temp2&l_u32Temp3)>0)
								{
									l_cTaiXing[l_u8TaiXingIndex++] ='2'	;
								}
								else
								{
									l_cTaiXing[l_u8TaiXingIndex++] ='1'	;
								}
							}
							l_u32Temp3 = l_u32Temp3 >> 1;
							
						}
						if(l_u8FindFirstData == 0)
						{
							l_cTaiXing[l_u8TaiXingIndex++] ='1'	;	
							l_u8FindFirstData = 0;
						}
						
							
						if(l_u32Temp1 < (PASSOVERINFO.u8AxleGrpTotalNum-1)) 
						l_cTaiXing[l_u8TaiXingIndex++] = '+';
					}
					l_cTaiXing[l_u8TaiXingIndex++] = '\0';
					LCDPRINTC(150	, 97	, (char*)l_cTaiXing);	
				}
				else	//���ᴦ��ֻ��ʾ�м����ᣬ����
				{
					sprintf(m_acTmp, "%d��,%d����", PASSOVERINFO.u8AxleTotalNum, PASSOVERINFO.u8AxleGrpTotalNum);
					LCDPRINTC(150	, 97	, (char *)m_acTmp);
				}
				l_u8EraseStatus	= 1;
				OSQAccept(g_pqueAxleShow);				
			}
		
			l_u32TimeCnt	++;

			WAITSCREENREFRESH();		//�ȴ�ˢ���ź���
		}
		//�Ƿ��˳�
		if (IfBreak(l_u8ControlCode))
			break;
	}
}

/*********************************************************************************************************
** Function name:		UISN
** Descriptions:		SN����
** input parameters:	None 
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110506	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void UISN(void)
{
	uint8	l_u8ControlCode;//����ָ��
	uint32	l_u32Value;		//��������
	uint32	l_u32SN;		//SNֵ
	uint8	l_u8DigitCnt;	//���ָ���
	uint8	l_u8Tmp1;		//��ʱ����
	uint8	l_u8Key;					  
	uint32	l_u32SNTimer;	//��Timer���ɵ�SN����

	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();
	
	BackGroundSave();
	BackGroundON();

	l_u32SN	= 0;
	g_u8EnableClearZero	= 0;	//����Ҫ����
	while(1)
	{
		l_u32SNTimer	= SNALIAS;
		l_u32Value	= 0;
		l_u8DigitCnt= 0;
		
		//������
		GUI_ClearSCR();
		LCDPRINTFC(16		, 32	, "SN:         %d", l_u32SNTimer);
		LCDPRINTC(16		, 64	, "���������: ");
		
		while (1)		//�жϰ�����ˢ������
		{
			//�жϰ���
			l_u8Key	= KeyValue;
			switch(l_u8Key)
			{
				case KB_ESC:
					ToBreak(l_u8ControlCode);
					break;

				case KB_1:		
				case KB_2:		
				case KB_3:		
				case KB_4:		
				case KB_5:		
				case KB_6:		
				case KB_7:		
				case KB_8:		
				case KB_9:		
				case KB_0:
					NotReDraw(l_u8ControlCode);
					l_u8DigitCnt++;
					l_u32Value	*= 10;
					l_u32Value	+= l_u8Key;
					break;	
				
				case KB_BKSP:		//��ɾһ������
					NotReDraw(l_u8ControlCode);
					if (l_u8DigitCnt>0)
						l_u8DigitCnt	--;
					l_u32Value	/= 10;
					break;	
				
				case KB_ENTER:		//ϵͳ��ʼ��
					l_u32SN=bcd(l_u32SNTimer,6)*bcd(l_u32SNTimer,1)*10000+bcd(l_u32SNTimer,5)*bcd(l_u32SNTimer,2)*100+bcd(l_u32SNTimer,4)*bcd(l_u32SNTimer,3) ;
					l_u32SN=(bcd(l_u32SN,1)<<20)+(bcd(l_u32SN,2)<<16)+(bcd(l_u32SN,3)<<12)+(bcd(l_u32SN,4)<<8)+(bcd(l_u32SN,5)<<4)+bcd(l_u32SN,6);
					
					if (l_u32SN == l_u32Value || l_u32Value == SUPERPWD)
						UIBDRoot();
					
					switch(l_u32Value)
					{
						case 111:
							UICommonSet();
							break;
						
						case 222:
							UIViewSetting();
							break;
								
						case 333:
							UIViewModify();
							break;
						
						case 888:
							UIViewAuthor();
							break;
#if	YBVERSION >= 30		//3.0�Ǳ���						
						case 444:
							UIViewIPInfo();
							break;
						
						case 8494:
							UIViewStartUpTime();
							break;
#endif	//#if	YBVERSION >= 30		//3.0�Ǳ���
						case 8968:
							UIViewThreshold();
							break;
																		 
#if	YBVERSION < 30		//2.2�Ǳ���
						case 5885:	GUI_ClearSCR();
							LCDPRINTC(16	, 32	, "����ISP״̬......");
							ISPinit();
							bootloader_entry();
							break;				
#endif	//#if	YBVERSION < 30		//2.2�Ǳ���

						default:
							break;	
					}
					l_u32Value	= 0;
					ToBreak(l_u8ControlCode);
					break;	
				
				default:
					NotReDraw(l_u8ControlCode);
					break;
			}	//switch(KeyValue)
			ClearKeyValue();
					
			//�Ƿ��ػ�
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;

			//ˢ����Ļ
			LCDPRINTC(112 + l_u8DigitCnt<<3 	, 64	, "  ");

			for (l_u8Tmp1 = 0; l_u8Tmp1 < l_u8DigitCnt; l_u8Tmp1 ++)
				LCDPRINTC(112 + l_u8Tmp1<<3 	, 64	, "*");

			WAITSCREENREFRESH();		//�ȴ�ˢ���ź���
		}

		//�Ƿ��˳�
		if (IfBreak(l_u8ControlCode))
			break;
	}
	g_u8EnableClearZero	= 1;		//�ָ��Զ�����	 
	BackGroundRecover();
}

/*********************************************************************************************************
** Function name:		UIBDRoot
** Descriptions:		�궨������
** input parameters:	None 
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110506	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void UIBDRoot(void)
{
	uint8	l_u8ControlCode;//����ָ��
	
	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();

	while(1)
	{
		//������
		GUI_ClearSCR();
		LCDPRINTC(0		, 0		, " ϵͳӦ��ѡ��");
		LCDPRINTC(176	, 0		, "Esc �˳�");
		
		LCDPRINTC(8		, 24	, "1. ��̬�ӱ궨");
		LCDPRINTC(8		, 44	, "2. ��̬�ӱ궨");
		LCDPRINTC(8		, 64	, "3. ϵͳ��ʼ��");
		
		GUI_Line(0		, 18	, 239	, 18	, 1);
		GUI_Line(0		, 124	, 239	, 124	, 1);
		GUI_Line(0		, 127	, 239	, 127	, 1);
		
		while (1)		//�жϰ�����ˢ������
		{
			//�жϰ���
			switch(KeyValue)
			{
				case KB_ESC:
					ToBreak(l_u8ControlCode);
					break;

				case KB_1:		//��̬�궨
					ToReDraw(l_u8ControlCode);
					UIBDMain(UI_MOTION);
					break;	
				
				case KB_2:		//��̬�궨
					ToReDraw(l_u8ControlCode);
					UIBDMain(UI_STATIC);
					break;	
				
				case KB_3:		//ϵͳ��ʼ��
					ToReDraw(l_u8ControlCode);
					UISystemInit();
					break;	
				
				default:
					NotReDraw(l_u8ControlCode);
					break;
			}	//switch(KeyValue)
			ClearKeyValue();
					
			//�Ƿ��ػ�
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;

			WAITSCREENREFRESH();		//�ȴ�ˢ���ź�
		}

		//�Ƿ��˳�
		if (IfBreak(l_u8ControlCode))
			break;
	}
}

/*********************************************************************************************************
** Function name:		UIBDMain
** Descriptions:		��̬�궨����
** input parameters:	p_u8Motion ����̬��־ 0����̬��1����̬ 
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110506	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void UIBDMain(uint8 p_u8Motion)
{
	uint8	l_u8ControlCode;//����ָ��
	
	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();

	while(1)
	{
		//������
		GUI_ClearSCR();
		if (p_u8Motion == UI_MOTION)		//��̬
			LCDPRINTC(0		, 0		, " ��̬�궨");
		else 				//��̬
			LCDPRINTC(0		, 0		, " ��̬�궨");

		LCDPRINTC(112	, 0		, "0 ����  Esc �˳�" );
		
		LCDPRINTC(8		, 24	, "1. ���궨");
		LCDPRINTC(8		, 44	, "2. ��̨�궨");
		LCDPRINTC(8		, 64	, "3. �ֶ��趨");
		if (p_u8Motion == UI_MOTION)
		{
			LCDPRINTC(8		, 84	, "4. ��̬����");
			LCDPRINTC(8		, 104	, "5. ��̬����");
			LCDPRINTC(128	, 24	, "6. �����趨");
			LCDPRINTC(128	, 44	, "7. ������");
			LCDPRINTC(128	, 64	, "8. �¶�����");
			LCDPRINTC(128	, 84	, "9. ��������");
			LCDPRINTC(128	, 104	, "F1 ������ʹ��");
		}
		GUI_Line(0		, 18	, 239	, 18	, 1);
		GUI_Line(0		, 124	, 239	, 124	, 1);
		GUI_Line(0		, 127	, 239	, 127	, 1);

		while (1)		//�жϰ�����ˢ������
		{
			//�жϰ���
			NotReDraw(l_u8ControlCode);
			switch(KeyValue)
			{
				case KB_ESC:
					ToBreak(l_u8ControlCode);
					break;

				case KB_1:		//���궨
					ToReDraw(l_u8ControlCode);
					UIBDWanBanChoose(p_u8Motion);
					break;	
				
				case KB_2:		//��̨�궨
					ToReDraw(l_u8ControlCode);
					UIBDChengtaiChoose(p_u8Motion);
					break;	
				
				case KB_3:		//�ֶ��趨
					ToReDraw(l_u8ControlCode);
					UIBDScale(p_u8Motion);
					break;	
				
				case KB_4:		//��̬����
					if (p_u8Motion == UI_MOTION)
					{
						ToReDraw(l_u8ControlCode);
						UIBDStaticModify();
					}
					break;	
				
				case KB_5:		//��̬����
					if (p_u8Motion == UI_MOTION)
					{
						ToReDraw(l_u8ControlCode);
						UIBDChooseMotion();
					}
					break;	
				
				case KB_6:		//�����趨
					if (p_u8Motion == UI_MOTION)
					{
						ToReDraw(l_u8ControlCode);
						UIBDFullRange();
					}
					break;	
				
				case KB_7:		//������
					if (p_u8Motion == UI_MOTION)
					{
						ToReDraw(l_u8ControlCode);
						UIBDGenZong();
					}
					break;	
				
				case KB_8:		//�¶�����
					if (p_u8Motion == UI_MOTION)
					{
						ToReDraw(l_u8ControlCode);
						UIBDPoDu();
					}
					break;	
				
				case KB_9:		//��������
					if (p_u8Motion == UI_MOTION)
					{
						ToReDraw(l_u8ControlCode);
						UIBDChooseVehPos();
					}
					break;	
				
				case KB_0:		//�����趨
					ToReDraw(l_u8ControlCode);
					SaveParams();
					break;	
				
				case KB_F1:		//���������
					if (p_u8Motion == UI_MOTION)
					{
						ToReDraw(l_u8ControlCode);
						UIBDLunZhou();
					}
					break;	

#if	SENDWAVEENABLE > 0		//ʹ�ܷ�����				
				case KB_F2:		//������ʹ��
					if (p_u8Motion == UI_MOTION)
					{
						ToReDraw(l_u8ControlCode);
						UIBDSendWave();
					}
					break;	
#endif	//#if	SENDWAVEENABLE > 0		//ʹ�ܷ�����
				
				default:
					NotReDraw(l_u8ControlCode);
					break;
			}	//switch(KeyValue)
			ClearKeyValue();
					
			//�Ƿ��ػ�
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;

			WAITSCREENREFRESH();		//�ȴ�ˢ���ź�
		}

		//�Ƿ��˳�
		if (IfBreak(l_u8ControlCode))
			break;
	}
}

/*********************************************************************************************************
** Function name:		UIBDWanBanChoose
** Descriptions:		��嶯̬�궨ͨ��ѡ�����
** input parameters:	p_u8Motion	����̬��־ 0��ʾ��̬��1��ʾ��̬ 
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110510	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  	ZHANG Ye	
** Modified date:	  	20110801
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void UIBDWanBanChoose(uint8 p_u8Motion)
{
	uint8	l_u8ControlCode;//����ָ��
	uint8	l_u8Key;
	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();

	while(1)
	{
		//������
		GUI_ClearSCR();
		GUI_Line(0		, 17	, 239	, 17	, 1);				
		GUI_Line(0		, 108	, 239	, 108	, 1);				
		
		if (p_u8Motion == UI_MOTION)		//��̬
			LCDPRINTC(0		, 0		, "1. ��嶯̬�궨       Esc-����");
		else 				//��̬
			LCDPRINTC(0		, 0		, "1. ��徲̬�궨       Esc-����");

		LCDPRINTC(0		, 19	, "������ͨ����:");
		
		while (1)		//�жϰ�����ˢ������
		{
			//�жϰ���
			l_u8Key	= KeyValue;
			switch(l_u8Key)
			{
				case KB_ESC:
					ToBreak(l_u8ControlCode);
					break;

				case KB_1:		
				case KB_2:
				case KB_3:		
				case KB_4:
					ToReDraw(l_u8ControlCode);
					UIBDWanBan(l_u8Key, p_u8Motion);
					break;

//				case 0xff:
				default:
					NotReDraw(l_u8ControlCode);
					break;
			}	//switch(KeyValue)
			ClearKeyValue();
					
			//�Ƿ��ػ�
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;

			WAITSCREENREFRESH();		//�ȴ�ˢ���ź�
		}

		//�Ƿ��˳�
		if (IfBreak(l_u8ControlCode))
			break;
	}
}

/*********************************************************************************************************
** Function name:		UIBDWanBan
** Descriptions:		��嶯̬�궨ͨ������
** input parameters:	p_u8CID		ͨ���� 
**						p_u8Motion	����̬��־ 0��ʾ��̬��1��ʾ��̬ 
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110507	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void UIBDWanBan(uint8 p_u8CID, uint8 p_u8Motion)
{
	uint8	l_u8ControlCode;//����ָ��
	uint32	l_u32Value;		//���������
	uint8	l_u8InputStatus;	//����״̬��1��
	uint8	l_u8Key;
	int32	l_n32ADTmp;

	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();

	while(1)
	{
		l_u8InputStatus	= 0;
		l_u32Value	= 0;
		
		//������
		GUI_ClearSCR();
		GUI_Line(0		, 17	, 239	, 17	, 1);				
		GUI_Line(0		, 108	, 239	, 108	, 1);				
		
		LCDPRINTC(0		, 0		, "1. ���궨           Esc-����");
		
		LCDPRINTFC(8	, 19	, " %uͨ��   "	, p_u8CID);
		
		LCDPRINTC(16	, 40	, "��ʼ��λ:");
		LCDPRINTFC(96	, 40	, "%u    "	, SETUPALIAS.an32Zero[p_u8CID-1]);
		LCDPRINTC(16	, 56	, "��ǰ����:");
		LCDPRINTC(16	, 72	, "��ǰ����:");
		LCDPRINTC(16	, 88	, "����ϵ��:");
		LCDPRINTFC(96	, 88	, "%u    "	, SETUPALIAS.an32AxGain[p_u8CID-1]);
		if (p_u8Motion == UI_STATIC)
			LCDPRINTC(0			, 112	, "F1����  F2�궨  F3����  F4ȥƤ");
		else
			LCDPRINTC(0			, 112	, "F1-����  F2-�궨  F3-����ϵ��");

		while (1)		//�жϰ�����ˢ������
		{
			//�жϰ���
			l_u8Key	= KeyValue;
			switch(l_u8Key)
			{
				case KB_ESC:
					ToBreak(l_u8ControlCode);
					break;

				case KB_1:		
				case KB_2:		
				case KB_3:		
				case KB_4:		
				case KB_5:		
				case KB_6:		
				case KB_7:		
				case KB_8:		
				case KB_9:		
				case KB_0:
					if (l_u8InputStatus != 0)
					{ 
						l_u32Value	*= 10;
						l_u32Value	+= l_u8Key;
					}
					NotReDraw(l_u8ControlCode);
					break;	
				
				case KB_BKSP:		//��ɾһ������
					if (l_u8InputStatus != 0)
					{ 
						l_u32Value	/= 10;
					}
					NotReDraw(l_u8ControlCode);
					break;	
				
				case KB_ENTER:		//ȷ������
					if (l_u8InputStatus == KB_F3)	//��������ϵ��
					{
						if (l_u32Value != 0)
							SETUPALIAS.an32AxGain[p_u8CID-1]	= l_u32Value;
					}
					else if (l_u8InputStatus == KB_F2)		//����궨����
					{
						if (l_u32Value != 0)
							SETUPALIAS.an32AxGain[p_u8CID-1]	= l_u32Value * 10000 / (ADAvg[p_u8CID-1] - SETUPALIAS.an32Zero[p_u8CID-1]);
					}
					ToReDraw(l_u8ControlCode);
					l_u8InputStatus	= 0;
					break;
				
				case KB_F1:			//F1������
					if (l_u8InputStatus == 0)
					{
						SETUPALIAS.an32Zero[p_u8CID-1]	= ADAvg[p_u8CID-1];
						ToReDraw(l_u8ControlCode);
						
						//���¶�̬���
						g_an32MotionZero[p_u8CID-1]	= SETUPALIAS.an32Zero[p_u8CID-1];
					}
					break;

				case KB_F4:			//F4����̬ȥƤ��
					if ((l_u8InputStatus == 0) && (p_u8Motion == UI_STATIC))
					{
						SETUPALIAS.an32Zero[p_u8CID-1]	= ADAvg[p_u8CID-1];
						ToReDraw(l_u8ControlCode);
					}
					else
						NotReDraw(l_u8ControlCode);
					break;

				case KB_F2:			//F2���궨
					if (l_u8InputStatus == 0)
					{
						l_u8InputStatus	= l_u8Key;
					}
					NotReDraw(l_u8ControlCode);
					break;
				
				case KB_F3:			//F3������ϵ��
					if (l_u8InputStatus == 0)
					{
						l_u8InputStatus	= l_u8Key;
					}
					NotReDraw(l_u8ControlCode);
					break;

				default:
					NotReDraw(l_u8ControlCode);
					break;
			}	//switch(KeyValue)
			ClearKeyValue();
					
			//�Ƿ��ػ�
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
			{
				break;
			}

			//ˢ����ʾ
			l_n32ADTmp	= ADAvg[p_u8CID-1];
			
			LCDPRINTFC(96		, 56	, "%u    "	, l_n32ADTmp);				//����
			
			if (l_u8InputStatus != KB_F2)		//��������궨����
			{
					LCDPRINTFC(96		, 72	, "%d kg    ", (l_n32ADTmp-SETUPALIAS.an32Zero[p_u8CID-1])*SETUPALIAS.an32AxGain[p_u8CID-1]/10000);		//����
			}
			else		//����궨����
			{
				if (l_u32Value == 0)
					LCDPRINTC(96		, 72	, "         ");
				else
					LCDPRINTFC(96		, 72	, "%u      ", l_u32Value);
			}
			
			if (l_u8InputStatus == KB_F3)		//��������
			{
				if (l_u32Value == 0)
					LCDPRINTC(96		, 88	, "         ");
				else
					LCDPRINTFC(96		, 88	, "%u      ", l_u32Value);
			}

			WAITSCREENREFRESH();		//�ȴ�ˢ���ź���

		}

		//�Ƿ��˳�
		if (IfBreak(l_u8ControlCode))
			break;
	}
}

/*********************************************************************************************************
** Function name:		UISystemInit
** Descriptions:		ϵͳ��ʼ������
** input parameters:	None 
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110507	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void UISystemInit(void)
{
	uint8	l_u8ControlCode;//����ָ��
	uint8	l_u8IfPwd;		//�Ƿ�Ϊ�ڶ������������666666
	uint32	l_u32Value;		//��������
	uint8	l_u8Operation;	//��ʼ����������
	uint8	l_u8Tmp1;		//��ʱ������ѭ����
	uint8	l_u8Key;
	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();
	
	while(1)
	{
		l_u8IfPwd	= 0;
		l_u32Value	= 0;
		l_u8Operation	= 0;
		//������
		GUI_ClearSCR();
		
		LCDPRINTC(0		, 0		, "1. ��ʼ��ΪϵͳĬ�ϲ���");
		LCDPRINTC(0		, 16	, "2. ������У��������ʼ��");
		LCDPRINTC(0		, 32	, "3. ��̬������ʼ��");
		LCDPRINTC(0		, 48	, "4. ����������ʼ��");
		LCDPRINTC(0		, 64	, "5. ��̬������ʼ��");
		LCDPRINTC(0		, 80	, "6. �ָ�������궨����");
		LCDPRINTC(0		, 96	, "7. �ָ�����ʷ�궨����");
		
		while (1)		//�жϰ�����ˢ������
		{
			//�жϰ���
			l_u8Key	= KeyValue;
			switch(l_u8Key)
			{
				case KB_ESC:
					ToBreak(l_u8ControlCode);
					break;

				case KB_1:		//��ʼ��ΪϵͳĬ�ϲ���
				case KB_2:		//������У��������ʼ��
				case KB_3:		//��̬������ʼ��
				case KB_4:		//����������ʼ��
				case KB_5:		//��̬������ʼ��
				case KB_6:		//�ָ�������궨����
				case KB_7:		//�ָ�����ʷ�궨����
					if (l_u8IfPwd == 0)
					{
						l_u8Operation	= l_u8Key;
						for (l_u8Tmp1 = 0; l_u8Tmp1 < 8; l_u8Tmp1 ++)
						{
							if (l_u8Tmp1 != l_u8Operation-1)
								ClearLCDLine(l_u8Tmp1<<4);
						}
						LCDPRINTC(16 	, 112	, "���������:");
					}
					else
					{
						l_u32Value	*= 10;
						l_u32Value	+= l_u8Key;
						LCDPRINTC(112 + l_u8IfPwd<<3 	, 112	, "*");
					}
					l_u8IfPwd	++;		
					break;	
				
				case KB_8:
				case KB_9:
				case KB_0:
					if (l_u8IfPwd == 0)
					{
						ToBreak(l_u8ControlCode);
					}
					else
					{
						l_u32Value	*= 10;
						l_u32Value	+= l_u8Key;
						LCDPRINTC(112 + l_u8IfPwd<<3 	, 112	, "*");
						l_u8IfPwd ++;
					}					
					break;	
				
				case KB_ENTER:			//ȷ��
					NotReDraw(l_u8ControlCode);
					l_u8IfPwd	= 0xf0;	//ȷ�Ϻ��ж������Ƿ���ȷ
					break;

				case 0xff:
					NotReDraw(l_u8ControlCode);
					break;

				default:		//�����������˳�
					ToBreak(l_u8ControlCode);
					break;
			}	//switch(KeyValue)
			ClearKeyValue();
					
			//�Ƿ��ػ�
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;
			
			//ˢ��
			if(l_u8IfPwd == 20)	//����20���ַ�����,���˳�
			{
				ToBreak(l_u8ControlCode);
				break;
			}
			
			if (l_u8IfPwd == 0xf0)	//�����������
			{
				if (l_u32Value == 666666)		//������ȷ
				{
					SETUPALIAS.u8DiaoDianTag	= 0;		//ȡ�����籣�����
	
					switch (l_u8Operation)
					{
						case 1:
							InitSystem();
							break;

						case 2:
							InitNonWeight();
							break;
						case 3:
							InitMotionModify();
							break;

						case 4:
							InitVehModify();
							break;

						case 5:
							InitStaticModify();
							break;

						case 6:
							if (RecoverToLast())	//�ɹ�
							{
								GUI_ClearSCR();
								LCDPRINTC(40 	, 56	, "��������ָ��ɹ�!");
								OSTimeDly(100);
							}
							else
							{
								GUI_ClearSCR();
								LCDPRINTC(40 	, 56	, "��������ָ�ʧ��!");
								OSTimeDly(100);
							}
							break;

						case 7:
							if (RecoverToHistory())	//�ɹ�
							{
								GUI_ClearSCR();
								LCDPRINTC(40 	, 56	, "��ʷ�����ָ��ɹ�!");
								OSTimeDly(100);
							}
							else
							{
								GUI_ClearSCR();
								LCDPRINTC(40 	, 56	, "��ʷ�����ָ�ʧ��!");
								OSTimeDly(100);
							}
							break;

						default:
							break;
					}
					GUI_ClearSCR();
					LCDPRINTC(40 	, 56	, "��ʼ�����, ��������...  ");
					InitRestart();
					OSTimeDly(100);
				}
				ToBreak(l_u8ControlCode);
				break;	
			}
			
			WAITSCREENREFRESH();		//�ȴ�ˢ���ź�
		}

		//�Ƿ��˳�
		if (IfBreak(l_u8ControlCode))
			break;
	}
}

/*********************************************************************************************************
** Function name:		UIBDChengtaiChoose
** Descriptions:		��̨����̬�궨ͨ��ѡ�����
** input parameters:	p_u8Motion	����̬��־ 0��ʾ��̬��1��ʾ��̬
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110531	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void UIBDChengtaiChoose(uint8 p_u8Motion)
{
	uint8	l_u8ControlCode;//����ָ��
	uint8	l_u8Key;
	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();

	while(1)
	{
		//������
		GUI_ClearSCR();
		GUI_Line(0		, 17	, 239	, 17	, 1);				
		GUI_Line(0		, 108	, 239	, 108	, 1);				
		
		if (p_u8Motion == UI_MOTION)		//��̬
			LCDPRINTC(0		, 0		, "2. ��̨��̬�궨       Esc-����");
		else 				//��̬
			LCDPRINTC(0		, 0		, "2. ��̨��̬�궨       Esc-����");

		LCDPRINTC(0		, 19	, "������̨���:");
		
		while (1)		//�жϰ�����ˢ������
		{
			//�жϰ���
			l_u8Key	= KeyValue;
			switch(l_u8Key)
			{
				case KB_ESC:
					ToBreak(l_u8ControlCode);
					break;

				case KB_1:		
				case KB_2:
					ToReDraw(l_u8ControlCode);
					UIBDChengTai(l_u8Key, p_u8Motion);
					break;

				case KB_0:
					ToReDraw(l_u8ControlCode);
					UIBDChengTaiAll(p_u8Motion);
					break;

				default:
					NotReDraw(l_u8ControlCode);
					break;
			}	//switch(KeyValue)
			ClearKeyValue();
					
			//�Ƿ��ػ�
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;

			WAITSCREENREFRESH();		//�ȴ�ˢ���ź�
		}

		//�Ƿ��˳�
		if (IfBreak(l_u8ControlCode))
			break;
	}
}

/*********************************************************************************************************
** Function name:		UIBDChengTaiAll
** Descriptions:		��̨��̬�궨����
** input parameters:	p_u8Motion	����̬��0��ʾ��̬��1��ʾ��̬
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110507	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void UIBDChengTaiAll(uint8 p_u8Motion)
{
	uint8	l_u8ControlCode;//����ָ��
	uint32	l_u32Value;		//���������
	uint8	l_u8InputStatus;	//����״̬��1��
	uint8	l_u8Key;
	int32	l_an32ADTmp[CHANNELNUM];
	int32	l_n32WholeAD;
	int32	l_n32WholeWeight;
	int32	l_n32WholeZero;
	uint8	l_u8TmpI;

	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();

	while(1)
	{
		l_u8InputStatus	= 0;
		l_u32Value		= 0;
		l_n32WholeZero	= 0;

		for (l_u8TmpI = 0; l_u8TmpI < CHANNELNUM; l_u8TmpI ++)
		{
			l_n32WholeZero	+= SETUPALIAS.an32Zero[l_u8TmpI];
		}

		//������
		GUI_ClearSCR();
		GUI_Line(0		, 17	, 239	, 17	, 1);				
		GUI_Line(0		, 108	, 239	, 108	, 1);				
		
		LCDPRINTC(0		, 0		, "2. ��̨�궨           Esc-����");
		LCDPRINTC(8		, 19	, " ̨��: ");
		for (l_u8TmpI = 0; l_u8TmpI < PLATNUM; l_u8TmpI ++)
		{
			LCDPRINTFC(64 + (l_u8TmpI<<5)	, 19	, "%d"	, l_u8TmpI+1);
			if (l_u8TmpI < PLATNUM -1)
			{
				LCDPRINTC(80 + (l_u8TmpI<<5)	, 19	, "+");
			}
		}
		LCDPRINTC(16		, 40	, "��ʼ��λ:");
		LCDPRINTFC(96		, 40	, "%u  "	, l_n32WholeZero);
		LCDPRINTC(16		, 56	, "��ǰ����:");
		LCDPRINTC(16		, 72	, "��ǰ����:");
		LCDPRINTC(16		, 88	, "����ϵ��:");
		LCDPRINTFC(96		, 88	, "%u  "	, SETUPALIAS.an32AxGain[0]);
		if (p_u8Motion == UI_STATIC)
			LCDPRINTC(0			, 112	, "F1����  F2�궨  F3����  F4ȥƤ");
		else
			LCDPRINTC(0			, 112	, "F1-����  F2-�궨  F3-����ϵ��");

		while (1)		//�жϰ�����ˢ������
		{
			//�жϰ���
			l_u8Key	= KeyValue;
			switch(l_u8Key)
			{
				case KB_ESC:
					ToBreak(l_u8ControlCode);
					break;

				case KB_1:		
				case KB_2:		
				case KB_3:		
				case KB_4:		
				case KB_5:		
				case KB_6:		
				case KB_7:		
				case KB_8:		
				case KB_9:		
				case KB_0:
					if (l_u8InputStatus != 0)
					{ 
						l_u32Value	*= 10;
						l_u32Value	+= l_u8Key;
					}
					NotReDraw(l_u8ControlCode);
					break;	
				
				case KB_BKSP:		//��ɾһ������
					if (l_u8InputStatus != 0)
					{ 
						l_u32Value	/= 10;
					}
					NotReDraw(l_u8ControlCode);
					break;	
				
				case KB_ENTER:		//ȷ������
					if (l_u8InputStatus == KB_F3)	//��������ϵ��
					{
						if (l_u32Value != 0)
						{
							for (l_u8TmpI = 0; l_u8TmpI < CHANNELNUM; l_u8TmpI ++)
							{
								SETUPALIAS.an32AxGain[l_u8TmpI]	= l_u32Value;
							}
						}
					}
					else if (l_u8InputStatus == KB_F2)		//����궨����
					{
						if (l_u32Value != 0)
						{
							l_n32WholeAD	= 0;
							for (l_u8TmpI = 0; l_u8TmpI < CHANNELNUM; l_u8TmpI ++)
							{
								l_n32WholeAD	+= l_an32ADTmp[l_u8TmpI];
							}
							SETUPALIAS.an32AxGain[0]	= l_u32Value * 10000 / (l_n32WholeAD - l_n32WholeZero);
							for (l_u8TmpI = 1; l_u8TmpI < CHANNELNUM; l_u8TmpI ++)
							{
								SETUPALIAS.an32AxGain[l_u8TmpI]	= SETUPALIAS.an32AxGain[0];
							}
						}
					}
					ToReDraw(l_u8ControlCode);
					l_u8InputStatus	= 0;
					break;
				
				case KB_F1:			//F1������
					if (l_u8InputStatus == 0)
					{
						for (l_u8TmpI = 0; l_u8TmpI < CHANNELNUM; l_u8TmpI ++)
						{
							SETUPALIAS.an32Zero[l_u8TmpI]	= ADAvg[l_u8TmpI];
						}
						ToReDraw(l_u8ControlCode);
					}
					break;

				case KB_F4:			//F4����̬ȥƤ��
					if ((l_u8InputStatus == 0) && (p_u8Motion == UI_STATIC))	//��̬����Ч
					{
						for (l_u8TmpI = 0; l_u8TmpI < CHANNELNUM; l_u8TmpI ++)
						{
							SETUPALIAS.an32Zero[l_u8TmpI]	= ADAvg[l_u8TmpI];
						}
						ToReDraw(l_u8ControlCode);
					}
					else
						NotReDraw(l_u8ControlCode);
					break;

				case KB_F2:			//F2���궨
					if (l_u8InputStatus == 0)
					{
						l_u8InputStatus	= l_u8Key;
					}
					NotReDraw(l_u8ControlCode);
					break;
				
				case KB_F3:			//F3������ϵ��
					if (l_u8InputStatus == 0)
					{
						l_u8InputStatus	= l_u8Key;
					}
					NotReDraw(l_u8ControlCode);
					break;

				default:
					NotReDraw(l_u8ControlCode);
					break;
			}	//switch(KeyValue)
			ClearKeyValue();
					
			//�Ƿ��ػ�
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
			{
				break;
			}

			//ˢ����ʾ
			l_n32WholeAD	= 0;
			l_n32WholeWeight	= 0;
			for (l_u8TmpI = 0; l_u8TmpI < CHANNELNUM; l_u8TmpI ++)
			{
				l_an32ADTmp[l_u8TmpI]	= ADAvg[l_u8TmpI];
				l_n32WholeAD	+= l_an32ADTmp[l_u8TmpI];
				l_n32WholeWeight	+= (l_an32ADTmp[l_u8TmpI] - SETUPALIAS.an32Zero[l_u8TmpI])* SETUPALIAS.an32AxGain[l_u8TmpI] / 10000;
			}

			LCDPRINTFC(96		, 56	, "%u    "	, l_n32WholeAD);				//����
			
			if (l_u8InputStatus != KB_F2)		//��������궨����
			{
				LCDPRINTFC(96		, 72	, "%d kg      ", l_n32WholeWeight);		//����
			}
			else		//����궨����
			{
				if (l_u32Value == 0)
					LCDPRINTC(96		, 72	, "         ");
				else
					LCDPRINTFC(96		, 72	, "%u      "	, l_u32Value);
			}
			
			if (l_u8InputStatus == KB_F3)		//��������
			{
				if (l_u32Value == 0)
					LCDPRINTC(96		, 88	, "         ");
				else
					LCDPRINTFC(96		, 88	, "%u    "	, l_u32Value);
			}

			WAITSCREENREFRESH();		//�ȴ�ˢ���ź�
		}

		//�Ƿ��˳�
		if (IfBreak(l_u8ControlCode))
			break;
	}
}

/*********************************************************************************************************
** Function name:		UIBDChengTai
** Descriptions:		��̨��̬�궨����
** input parameters:	p_u8CID		̨���
**						p_u8Motion	����̬��0��ʾ��̬��1��ʾ��̬
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110507	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void UIBDChengTai(uint8	p_u8CID, uint8 p_u8Motion)
{
	uint8	l_u8ControlCode;//����ָ��
	uint32	l_u32Value;		//���������
	uint8	l_u8InputStatus;	//����״̬��1��
	uint8	l_u8Key;
	int32	l_an32ADTmp[2];

	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();

	while(1)
	{
		l_u8InputStatus	= 0;
		l_u32Value	= 0;
		
		//������
		GUI_ClearSCR();
		GUI_Line(0		, 17	, 239	, 17	, 1);				
		GUI_Line(0		, 108	, 239	, 108	, 1);				
		
		LCDPRINTC(0		, 0		, "2. ��̨�궨           Esc-����");
		//LCDPRINTC(0		, 19	, "������ͨ����:");
		LCDPRINTFC(8		, 19	, " %ų��"	, p_u8CID);
		
		LCDPRINTC(16		, 40	, "��ʼ��λ:");
		LCDPRINTFC(96		, 40	, "%u  "	, SETUPALIAS.an32Zero[2*(p_u8CID-1)]+SETUPALIAS.an32Zero[2*(p_u8CID-1)+1]);
		LCDPRINTC(16		, 56	, "��ǰ����:");
		LCDPRINTC(16		, 72	, "��ǰ����:");
		LCDPRINTC(16		, 88	, "����ϵ��:");
		LCDPRINTFC(96		, 88	, "%u  "	, SETUPALIAS.an32AxGain[2*(p_u8CID-1)]);
		if (p_u8Motion == UI_STATIC)
			LCDPRINTC(0			, 112	, "F1����  F2�궨  F3����  F4ȥƤ");
		else
			LCDPRINTC(0			, 112	, "F1-����  F2-�궨  F3-����ϵ��");

		while (1)		//�жϰ�����ˢ������
		{
			//�жϰ���
			l_u8Key	= KeyValue;
			switch(l_u8Key)
			{
				case KB_ESC:
					ToBreak(l_u8ControlCode);
					break;

				case KB_1:		
				case KB_2:		
				case KB_3:		
				case KB_4:		
				case KB_5:		
				case KB_6:		
				case KB_7:		
				case KB_8:		
				case KB_9:		
				case KB_0:
					if (l_u8InputStatus != 0)
					{ 
						l_u32Value	*= 10;
						l_u32Value	+= l_u8Key;
					}
					NotReDraw(l_u8ControlCode);
					break;	
				
				case KB_BKSP:		//��ɾһ������
					if (l_u8InputStatus != 0)
					{ 
						l_u32Value	/= 10;
					}
					NotReDraw(l_u8ControlCode);
					break;	
				
				case KB_ENTER:		//ȷ������
					if (l_u8InputStatus == KB_F3)	//��������ϵ��
					{
						if (l_u32Value != 0)
						{
							SETUPALIAS.an32AxGain[2*(p_u8CID-1)]	= l_u32Value;
							SETUPALIAS.an32AxGain[2*(p_u8CID-1)+1]	= l_u32Value;
						}
					}
					else if (l_u8InputStatus == KB_F2)		//����궨����
					{
						if (l_u32Value != 0)
						{
							SETUPALIAS.an32AxGain[2*(p_u8CID-1)]	= l_u32Value * 10000 / (ADAvg[2*(p_u8CID-1)] + ADAvg[2*(p_u8CID-1)+1] - SETUPALIAS.an32Zero[2*(p_u8CID-1)] - SETUPALIAS.an32Zero[2*(p_u8CID-1)+1]);
							SETUPALIAS.an32AxGain[2*(p_u8CID-1)+1]	= SETUPALIAS.an32AxGain[2*(p_u8CID-1)];
						}
					}
					ToReDraw(l_u8ControlCode);
					l_u8InputStatus	= 0;
					break;
				
				case KB_F1:			//F1������
					if (l_u8InputStatus == 0)
					{
						SETUPALIAS.an32Zero[2*(p_u8CID-1)]		= ADAvg[2*(p_u8CID-1)];
						SETUPALIAS.an32Zero[2*(p_u8CID-1)+1]	= ADAvg[2*(p_u8CID-1)+1];
						ToReDraw(l_u8ControlCode);
					}
					break;

				case KB_F4:			//F4����̬ȥƤ��
					if ((l_u8InputStatus == 0) && (p_u8Motion == UI_STATIC))	//��̬����Ч
					{
						SETUPALIAS.an32Zero[2*(p_u8CID-1)]		= ADAvg[2*(p_u8CID-1)];
						SETUPALIAS.an32Zero[2*(p_u8CID-1)+1]	= ADAvg[2*(p_u8CID-1)+1];
						ToReDraw(l_u8ControlCode);
					}
					else
						NotReDraw(l_u8ControlCode);
					break;

				case KB_F2:			//F2���궨
					if (l_u8InputStatus == 0)
					{
						l_u8InputStatus	= l_u8Key;
					}
					NotReDraw(l_u8ControlCode);
					break;
				
				case KB_F3:			//F3������ϵ��
					if (l_u8InputStatus == 0)
					{
						l_u8InputStatus	= l_u8Key;
					}
					NotReDraw(l_u8ControlCode);
					break;

				default:
					NotReDraw(l_u8ControlCode);
					break;
			}	//switch(KeyValue)
			ClearKeyValue();
					
			//�Ƿ��ػ�
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
			{
				break;
			}

			//ˢ����ʾ
			l_an32ADTmp[0]	= ADAvg[2*(p_u8CID-1)];
			l_an32ADTmp[1]	= ADAvg[2*(p_u8CID-1)+1];

			LCDPRINTFC(96		, 56	, "%u    "	, l_an32ADTmp[0]+l_an32ADTmp[1]);				//����
			
			if (l_u8InputStatus != KB_F2)		//��������궨����
			{
				LCDPRINTFC(96		, 72	, "%d kg    ", (l_an32ADTmp[0]-SETUPALIAS.an32Zero[2*(p_u8CID-1)])*SETUPALIAS.an32AxGain[2*(p_u8CID-1)]/10000+(l_an32ADTmp[1]-SETUPALIAS.an32Zero[2*(p_u8CID-1)+1])*SETUPALIAS.an32AxGain[2*(p_u8CID-1)+1]/10000);		//����
			}
			else		//����궨����
			{
				if (l_u32Value == 0)
					LCDPRINTC(96		, 72	, "         ");
				else
					LCDPRINTFC(96		, 72	, "%u    "	, l_u32Value);
			}
			
			if (l_u8InputStatus == KB_F3)		//��������
			{
				if (l_u32Value == 0)
					LCDPRINTC(96		, 88	, "         ");
				else
					LCDPRINTFC(96		, 88	, "%u    "	, l_u32Value);
			}

			WAITSCREENREFRESH();		//�ȴ�ˢ���ź�
		}

		//�Ƿ��˳�
		if (IfBreak(l_u8ControlCode))
			break;
	}
}

/*********************************************************************************************************
** Function name:		UIBDGenZong
** Descriptions:		������ʹ�ܽ���
** input parameters:	None 
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110507	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void UIBDGenZong(void)
{
	UISetEnableParam(" ������ʹ��" , &(SETUPALIAS.u8Genzong));
}

/*********************************************************************************************************
** Function name:		UIBDPoDu
** Descriptions:		�¶���������
** input parameters:	None 
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110507	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void UIBDPoDu(void)
{
	UISetValueParamU16("�¶�����", &(SETUPALIAS.u16Podu), 9000, 11000);
}

/*********************************************************************************************************
** Function name:		UIBDLunZhou
** Descriptions:		���������ʹ�ܽ���
** input parameters:	None 
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110507	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void UIBDLunZhou(void)
{
	uint8	l_u8ControlCode;//����ָ��

	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();

	while(1)
	{
		//������
		GUI_ClearSCR();
		LCDPRINTC(0		, 0		, " ���������ʹ��");
		LCDPRINTC(176	, 0		, "Esc ����");
		
		LCDPRINTC(8		, 24	, "�Ƿ�ʹ��:");
		LCDPRINTC(8		, 44	, "1.ʹ��  2.��ֹ");
		
		GUI_Line(0		, 18	, 239	, 18	, 1);
		GUI_Line(0		, 124	, 239	, 124	, 1);
		GUI_Line(0		, 127	, 239	, 127	, 1);
		
		while (1)		//�жϰ�����ˢ������
		{
			//�жϰ���
			switch(KeyValue)
			{
				case KB_ESC:
				case KB_ENTER:
					ToBreak(l_u8ControlCode);
					break;
					
				case KB_1:		//ʹ��
					NotReDraw(l_u8ControlCode);
					SETUPALIAS.u8LunZhouERR	|= 0x02;
					break;	
						
				case KB_2:		//��ֹ		
					NotReDraw(l_u8ControlCode);
					SETUPALIAS.u8LunZhouERR	&= ~0x02;
					break;	
					
				default:
					NotReDraw(l_u8ControlCode);
					break;
			}	//switch(KeyValue)
			ClearKeyValue();
					
			//�Ƿ��ػ�
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;

			//ˢ����Ļ
			if (SETUPALIAS.u8LunZhouERR & 0x02)		//Ϊ1ʱ��ʹ��
				LCDPRINTC(120		, 24	, "ʹ��  ");
			else
				LCDPRINTC(120		, 24	, "��ֹ  ");
			
			WAITSCREENREFRESH();		//�ȴ�ˢ���ź�
		}

		//�Ƿ��˳�
		if (IfBreak(l_u8ControlCode))
			break;

	}
}

/*********************************************************************************************************
** Function name:		UISetLunZhouEnable
** Descriptions:		����������ϱ���ʹ�ܽ���
** input parameters:	None 
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110507	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void UISetLunZhouEnable(void)
{
	uint8	l_u8ControlCode;//����ָ��

	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();

	while(1)
	{
		//������
		GUI_ClearSCR();
		LCDPRINTC(0		, 0		, " ������ϱ���ʹ��");
		LCDPRINTC(176	, 0		, "Esc ����");
		
		LCDPRINTC(8		, 24	, "�Ƿ񱨴�:");
		LCDPRINTC(8		, 44	, "1.����  2.������");
		
		GUI_Line(0		, 18	, 239	, 18	, 1);
		GUI_Line(0		, 124	, 239	, 124	, 1);
		GUI_Line(0		, 127	, 239	, 127	, 1);
		
		while (1)		//�жϰ�����ˢ������
		{
			//�жϰ���
			switch(KeyValue)
			{
				case KB_ESC:
				case KB_ENTER:
					ToBreak(l_u8ControlCode);
					ClearKeyValue();
					break;
					
				case KB_1:		//����
					NotReDraw(l_u8ControlCode);
					SETUPALIAS.u8LunZhouERR	|= 0x01;
					ClearKeyValue();
					break;	
						
				case KB_2:		//������		
					NotReDraw(l_u8ControlCode);
					SETUPALIAS.u8LunZhouERR	&= ~0x01;
					ClearKeyValue();
					break;	
					
				default:
					NotReDraw(l_u8ControlCode);
					ClearKeyValue();
					break;
			}	//switch(KeyValue)

			//�Ƿ��ػ�
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;

			//ˢ����Ļ
			if (SETUPALIAS.u8LunZhouERR & 0x01)		//Ϊ1ʱ������
				LCDPRINTC(120		, 24	, "����    ");
			else
				LCDPRINTC(120		, 24	, "������  ");
			
			WAITSCREENREFRESH();		//�ȴ�ˢ���ź�
		}

		//�Ƿ��˳�
		if (IfBreak(l_u8ControlCode))
			break;

	}
}
  
#if	SENDWAVEENABLE > 0		//ʹ�ܷ�����
/*********************************************************************************************************
** Function name:		UIBDSendWave
** Descriptions:		���Ͳ���ʹ�ܽ���
** input parameters:	None 
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110507	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void UIBDSendWave(void)
{
	uint8	l_u8Status	= 0;
	l_u8Status	|= ((SETUPALIAS.u8SendWaveEnable	& 0x0f)<<4);
	UISetEnableParam(" ���Ͳ���ʹ��" , &(SETUPALIAS.u8SendWaveEnable));
	
	l_u8Status	|= SETUPALIAS.u8SendWaveEnable	& 0x0f;

	if (l_u8Status == 0x01)
		OSTaskResume(TASKWAVEPRIO);
	else
		if (l_u8Status == 0x10)
			OSTaskSuspend(TASKWAVEPRIO);
}									  
#endif

/*********************************************************************************************************
** Function name:		UISetDog
** Descriptions:		���Ź����ý���
** input parameters:	None 
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110507	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void UISetDog(void)
{
	UISetEnableParam(" ���Ź�ʹ��" , &(SETUPALIAS.u8DOG));
}

/*********************************************************************************************************
** Function name:		UISetCapture
** Descriptions:		����ץ��ʹ�ܽ���
** input parameters:	None 
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110507	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void UISetCapture(void)
{
	UISetEnableParam(" �ϳ�ץ��ʹ��" , &(SETUPALIAS.u8ZhuapaiEnable));
}

/*********************************************************************************************************
** Function name:		UISetLoop
** Descriptions:		��Ȧ����ʹ�ܽ���
** input parameters:	None 
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110507	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void UISetLoop(void)
{
	UISetEnableParam(" ��Ȧ����ʹ��" , &(SETUPALIAS.u8LoopTriggerEnable));
}
   
/*********************************************************************************************************
** Function name:		UISetForwardEnable
** Descriptions:		����ʹ�ܽ���
** input parameters:	None 
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110507	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void UISetForwardEnable(void)
{
	UISetEnableParam(" ����ʹ��" , &(SETUPALIAS.u8FangxiangEnable));
}

/*********************************************************************************************************
** Function name:		UIViewAuthor
** Descriptions:		�鿴������Ϣ����
** input parameters:	None 
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110507	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void UIViewAuthor(void)
{
	//������
	GUI_ClearSCR();
	ClearKeyValue();

	LCDPRINTC(16		, 32	, "����: ������");
	LCDPRINTC(16		, 64	, "�绰: 13801298463");
	LCDPRINTC(16		, 96	, "tly001@vip.sina.com");
	
	GUI_Line(0		, 18	, 239	, 18	, 1);
	GUI_Line(0		, 124	, 239	, 124	, 1);
	GUI_Line(0		, 127	, 239	, 127	, 1);
		
	while (KeyValue == 0xff)		//�а���
	{
		WAITSCREENREFRESH();		//�ȴ�ˢ���ź�
	}
}

/*********************************************************************************************************
** Function name:		UISetProtocol
** Descriptions:		����Э�����
** input parameters:	None 
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110507	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void UISetProtocol(void)
{
	uint8	l_u8ControlCode;//����ָ��
	uint8	l_u8Key;
	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();

	while(1)
	{
		//������
		GUI_ClearSCR();
		LCDPRINTC(64	, 0		, "ͨѶЭ��:");

		LCDPRINTC(8		, 24	, "0--����");
		LCDPRINTC(8		, 40	, "1--��");
		LCDPRINTC(8		, 56	, "2--����");
		LCDPRINTC(8		, 72	, "3--����");
		LCDPRINTC(8		, 88	, "4--����");
		LCDPRINTC(8		, 104	, "5--�Ĵ�");
		LCDPRINTC(128	, 24	, "6--����");
		LCDPRINTC(128	, 40	, "7--����");
		LCDPRINTC(128	, 56	, "8--����");
		LCDPRINTC(128	, 72	, "9--����");

		GUI_Line(0		, 18	, 239	, 18	, 1);
		GUI_Line(0		, 124	, 239	, 124	, 1);
		GUI_Line(0		, 127	, 239	, 127	, 1);
		
		while (1)		//�жϰ�����ˢ������
		{
			//�жϰ���
			l_u8Key	= KeyValue;
			switch(l_u8Key)
			{
				case KB_ESC:
				case KB_ENTER:
					ToBreak(l_u8ControlCode);
					break;
					
				case KB_0:
				case KB_1:
				case KB_2:
				case KB_3:
				case KB_4:
				case KB_5:
				case KB_6:
				case KB_7:
				case KB_8:
				case KB_9:
					NotReDraw(l_u8ControlCode);
					SETUPALIAS.u8Protocol	= l_u8Key;
					break;	
					
				case KB_J:
					NotReDraw(l_u8ControlCode);
					SETUPALIAS.u8Protocol	= 10;
					break;	
					
				default:
					NotReDraw(l_u8ControlCode);
					break;
			}	//switch(KeyValue)
			ClearKeyValue();
					
			//�Ƿ��ػ�
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;

			//ˢ����Ļ
			LCDPRINTFC(160	, 0		, "%d  ",SETUPALIAS.u8Protocol);

			WAITSCREENREFRESH();		//�ȴ�ˢ���ź�
		}

		//�Ƿ��˳�
		if (IfBreak(l_u8ControlCode))
			break;
	}
}

/*********************************************************************************************************
** Function name:		UISetBaudRate
** Descriptions:		���ò����ʽ���
** input parameters:	None 
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110507	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void UISetBaudRate(void)
{
	uint8	l_u8ControlCode;//����ָ��
	uint8	l_u8Key;
	uint32	l_au32BR[6] = {4800, 4800, 9600, 38400, 57600, 115200};
	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();

	while(1)
	{
		//������
		GUI_ClearSCR();
		LCDPRINTC(0		, 0		, "2. �������趨       Esc-����");
		
		LCDPRINTC(0		, 19	, "��ѡ������:");
		LCDPRINTC(64	, 40	, "1.  4800");
		LCDPRINTC(64	, 56	, "2.  9600");
		LCDPRINTC(64	, 72	, "3.  38400");
		LCDPRINTC(64	, 88	, "4.  57600");
		LCDPRINTC(64	, 104	, "5.  115200");
				
		GUI_Line(0		, 17	, 239	, 17	, 1);
		GUI_Line(0		, 124	, 239	, 124	, 1);
		GUI_Line(0		, 127	, 239	, 127	, 1);
				
		while (1)		//�жϰ�����ˢ������
		{
			//�жϰ���
			l_u8Key	= KeyValue;
			switch(l_u8Key)
			{
				case KB_ESC:
				case KB_ENTER:
					ToBreak(l_u8ControlCode);
					break;
					
				case KB_1:
				case KB_2:
				case KB_3:
				case KB_4:
				case KB_5:
					NotReDraw(l_u8ControlCode);
					SETUPALIAS.u8BaudRate	= l_u8Key;
					break;	
					
				default:
					NotReDraw(l_u8ControlCode);
					break;
			}	//switch(KeyValue)
			ClearKeyValue();
					
			//�Ƿ��ػ�
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;

			//ˢ����Ļ
			LCDPRINTFC(112	, 19	, "%u    "	, l_au32BR[SETUPALIAS.u8BaudRate]);

			WAITSCREENREFRESH();		//�ȴ�ˢ���ź�
		}

		//�Ƿ��˳�
		if (IfBreak(l_u8ControlCode))
			break;
	}
}

/*********************************************************************************************************
** Function name:		UISetCommandMode
** Descriptions:		��������ģʽ����
** input parameters:	None 
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110507	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void UISetCommandMode(void)
{
	uint8	l_u8ControlCode;//����ָ��
	uint8	l_u8Key;
	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();

	while(1)
	{
		//������
		GUI_ClearSCR();
		LCDPRINTC(64		, 56	, "���ʽ: ");
					
		while (1)		//�жϰ�����ˢ������
		{
			//�жϰ���
			l_u8Key	= KeyValue;
			switch(l_u8Key)
			{
				case KB_ESC:
				case KB_ENTER:
					ToBreak(l_u8ControlCode);
					break;
					
				case KB_0:
				case KB_1:
					NotReDraw(l_u8ControlCode);
					SETUPALIAS.u8ComMode	= l_u8Key;
					break;	
					
				default:
					NotReDraw(l_u8ControlCode);
					break;
			}	//switch(KeyValue)
			ClearKeyValue();
					
			//�Ƿ��ػ�
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;

			//ˢ����Ļ
			LCDPRINTFC(144	, 56	, "%2d"	, SETUPALIAS.u8ComMode);
		
			WAITSCREENREFRESH();		//�ȴ�ˢ���ź�
		}

		//�Ƿ��˳�
		if (IfBreak(l_u8ControlCode))
			break;
	}
}

/*********************************************************************************************************
** Function name:		UISetPlat
** Descriptions:		����̨���Ƚ���
** input parameters:	None 
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110509	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void UISetPlat(void)
{
	UISetValueParamU8("̨����", &(SETUPALIAS.u8PlatWidth), 1, 127);  
	g_u16PlatWidth			= (SETUPALIAS.u8PlatWidth+30)*36*POINTRATE/1000;		//cw	
}

/*********************************************************************************************************
** Function name:		UISetVehicleCache
** Descriptions:		���ó����������(1~10)��Ĭ��10
** input parameters:	None 
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110509	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void UISetVehicleCache(void)
{
	UISetValueParamU8("��������", &(SETUPALIAS.u8VehicleBufSize), 0, MAXBUFNUM);
}
   
/*********************************************************************************************************
** Function name:		UISetDiaodian
** Descriptions:		���籣��ʹ�ܽ���
** input parameters:	None 
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110509	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void UISetDiaodian(void)
{
	UISetEnableParam(" ���籣��ʹ��" , &(SETUPALIAS.u8DiaoDianTag));
}

/*********************************************************************************************************
** Function name:		UISetEnableParam
** Descriptions:		ͨ��ʹ�ܲ������ý���
** input parameters:	p_pcName		������
**						p_pu8Param 		����ָ��
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110509	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void UISetEnableParam(char * p_pcName, uint8 * p_pu8Param)
{
	uint8	l_u8ControlCode;//����ָ��

	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();

	while(1)
	{
		//������
		GUI_ClearSCR();
		LCDPRINTC(0		, 0		, p_pcName);
		LCDPRINTC(176	, 0		, "Esc ����");
		
		LCDPRINTC(8		, 24	, "�Ƿ�ʹ��:");
		LCDPRINTC(8		, 44	, "1.ʹ��  2.��ֹ");
		GUI_Line(0		, 18	, 239	, 18	, 1);
		GUI_Line(0		, 124	, 239	, 124	, 1);
		GUI_Line(0		, 127	, 239	, 127	, 1);
		
		while (1)		//�жϰ�����ˢ������
		{
			//�жϰ���
			switch(KeyValue)
			{
				case KB_ESC:
				case KB_ENTER:
					ToBreak(l_u8ControlCode);
					break;
					
				case KB_1:		//ʹ��
					NotReDraw(l_u8ControlCode);
					* p_pu8Param	= 1;
					break;	
						
				case KB_2:		//��ֹ		
					NotReDraw(l_u8ControlCode);
					* p_pu8Param	= 0;
					break;	
					
				default:
					NotReDraw(l_u8ControlCode);
					break;
			}	//switch(KeyValue)
			ClearKeyValue();

			//�Ƿ��ػ�
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;

			//ˢ����Ļ
			if (* p_pu8Param)		//Ϊ1ʱ��ʹ��
				LCDPRINTC(120		, 24	, "ʹ��  ");
			else
				LCDPRINTC(120		, 24	, "��ֹ  ");
			
			WAITSCREENREFRESH();		//�ȴ�ˢ���ź�
		}

		//�Ƿ��˳�
		if (IfBreak(l_u8ControlCode))
			break;

	}
}

/*********************************************************************************************************
** Function name:		UISetValueParamU8
** Descriptions:		ͨ�ò������ý���,����U8����
** input parameters:	p_pcName		������
**						p_pu8Param 		����ָ��
**						p_u32Max		�������� 
**						p_u32Min		��������
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110509	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void UISetValueParamU8(char * p_pcName, uint8 * p_pu8Param, uint32 p_u32Min, uint32 p_u32Max)
{
	uint8	l_u8ControlCode;//����ָ��
	uint32	l_u32Value	= 0;		//��������
	uint8	l_u8Status;		//���ܼ�״̬
	uint8	l_u8Key;		//����
	char	l_acTmp[30];

	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();

	while(1)
	{
		l_u32Value	= 0;
		l_u8Status	= 0;

		//������
		GUI_ClearSCR();
		memset(l_acTmp, 0, 30);
		sprintf(l_acTmp,"%s: %u",p_pcName,*p_pu8Param);
		LCDPRINTC(0		, 0		, l_acTmp);
		LCDPRINTC(0		, 32	, " Esc�˳�, F1���޸�");
		
		GUI_Line(0		, 18	, 239	, 18	, 1);
		GUI_Line(0		, 124	, 239	, 124	, 1);
		GUI_Line(0		, 127	, 239	, 127	, 1);
		
		while (1)		//�жϰ�����ˢ������
		{
			//�жϰ���
			l_u8Key	= KeyValue;
			switch(l_u8Key)
			{
				case KB_ESC:
					ToBreak(l_u8ControlCode);
					break;

				case KB_1:		
				case KB_2:		
				case KB_3:		
				case KB_4:		
				case KB_5:		
				case KB_6:		
				case KB_7:		
				case KB_8:		
				case KB_9:		
				case KB_0:
					if (l_u8Status != 0)
					{
						l_u32Value	*= 10;
						l_u32Value	+= l_u8Key;
					}
					NotReDraw(l_u8ControlCode);
					break;	
				
				case KB_BKSP:		//��ɾһ������
					NotReDraw(l_u8ControlCode);
					l_u32Value	/= 10;
					break;	
				
				case KB_F1:		//�޸�״̬����ʼ��������
					if (l_u8Status == 0)
					{
						l_u8Status	= KB_F1;
						l_u32Value	= 0;
						memset(l_acTmp, 0, 30);
						sprintf(l_acTmp," ��������ֵ(%u-%u):",p_u32Min,p_u32Max);
						LCDPRINTC(16		, 56	, l_acTmp);
					}
					NotReDraw(l_u8ControlCode);
					break;	
				
				case KB_ENTER:		//ȷ������
					if (l_u8Status	== KB_F1)
					{
						if ((p_u32Max == 0 && p_u32Min == 0) || (l_u32Value >= p_u32Min && l_u32Value <= p_u32Max))
						{
							*p_pu8Param	= l_u32Value & 0xff;
						}
						l_u32Value	= 0;
						l_u8Status	= 0;
						ToReDraw(l_u8ControlCode);
					}
					else
						NotReDraw(l_u8ControlCode);
					break;	
				
				default:
					NotReDraw(l_u8ControlCode);
					break;
			}	//switch(KeyValue)
			ClearKeyValue();

			//�Ƿ��ػ�
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;

			//ˢ����Ļ
			if (l_u8Status == KB_F1)		//����״̬
			{
				if (l_u32Value == 0)
					LCDPRINTC(56		, 80	, "         ");
				else
					LCDPRINTFC(56		, 80	, "%u      ", l_u32Value);
			}
	
			WAITSCREENREFRESH();		//�ȴ�ˢ���ź�
		}

		//�Ƿ��˳�
		if (IfBreak(l_u8ControlCode))
			break;
	}
}

/*********************************************************************************************************
** Function name:		UISetValueParamU16
** Descriptions:		ͨ�ò������ý���,����U16����
** input parameters:	p_pcName		������
**						p_pu16Param 	����ָ��
**						p_u32Max		�������� 
**						p_u32Min		��������
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110509	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void UISetValueParamU16(char * p_pcName, uint16 * p_pu16Param, uint32 p_u32Min, uint32 p_u32Max)
{
	uint8	l_u8ControlCode;//����ָ��
	uint32	l_u32Value	= 0;		//��������
	uint8	l_u8Status;		//���ܼ�״̬
	char	l_acTmp[30];
	uint8	l_u8Key;
	
	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();

	while(1)
	{
		l_u32Value	= 0;
		l_u8Status	= 0;

		//������
		GUI_ClearSCR();
		memset(l_acTmp, 0, 30);
		sprintf(l_acTmp,"%s: %u",p_pcName,*p_pu16Param);
		LCDPRINTC(0		, 0		, l_acTmp);
		LCDPRINTC(0		, 32	, " Esc�˳�, F1���޸�");
		
		GUI_Line(0		, 18	, 239	, 18	, 1);
		GUI_Line(0		, 124	, 239	, 124	, 1);
		GUI_Line(0		, 127	, 239	, 127	, 1);
		
		while (1)		//�жϰ�����ˢ������
		{
			//�жϰ���
			l_u8Key	= KeyValue;
			switch(l_u8Key)
			{
				case KB_ESC:
					ToBreak(l_u8ControlCode);
					break;

				case KB_1:		
				case KB_2:		
				case KB_3:		
				case KB_4:		
				case KB_5:		
				case KB_6:		
				case KB_7:		
				case KB_8:		
				case KB_9:		
				case KB_0:
					if (l_u8Status != 0)
					{
						l_u32Value	*= 10;
						l_u32Value	+= l_u8Key;
					}
					NotReDraw(l_u8ControlCode);
					break;	
				
				case KB_BKSP:		//��ɾһ������
					NotReDraw(l_u8ControlCode);
					l_u32Value	/= 10;
					break;	
				
				case KB_F1:		//�޸�״̬����ʼ��������
					if (l_u8Status == 0)
					{
						l_u8Status	= KB_F1;
						l_u32Value	= 0;
						memset(l_acTmp, 0, 30);
						sprintf(l_acTmp," ��������ֵ(%u-%u):",p_u32Min,p_u32Max);
						LCDPRINTC(16		, 56	, l_acTmp);
					}
					NotReDraw(l_u8ControlCode);
					break;	
				
				case KB_ENTER:		//ȷ������
					if (l_u8Status	== KB_F1)
					{
						if ((p_u32Max == 0 && p_u32Min == 0) || (l_u32Value >= p_u32Min && l_u32Value <= p_u32Max))
						{
							*p_pu16Param	= l_u32Value & 0xffff;
						}
						l_u32Value	= 0;
						l_u8Status	= 0;
						ToReDraw(l_u8ControlCode);
					}
					else
						NotReDraw(l_u8ControlCode);
					
					break;	
				
				default:
					NotReDraw(l_u8ControlCode);
					break;
			}	//switch(KeyValue)
			ClearKeyValue();
					
			//�Ƿ��ػ�
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;

			//ˢ����Ļ
			if (l_u8Status == KB_F1)		//����״̬
			{
				if (l_u32Value == 0)
					LCDPRINTC(56		, 80	, "         ");
				else
					LCDPRINTFC(56		, 80	, "%u      "	, l_u32Value);
			}

			WAITSCREENREFRESH();		//�ȴ�ˢ���ź�
		}

		//�Ƿ��˳�
		if (IfBreak(l_u8ControlCode))
			break;
	}
}

/*********************************************************************************************************
** Function name:		UIValidate
** Descriptions:		��֤����
** input parameters:	None 
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110614	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void UIValidate(void)
{
	uint8	l_u8ControlCode;//����ָ��
	uint32	l_u32Value;		//��������
	uint32	l_u32SN;		//SNֵ
	uint8	l_u8DigitCnt;	//���ָ���
	uint8	l_u8Tmp1;		//��ʱ����
	uint8	l_u8Key;
	uint32	l_u32SNTimer;	//��Timer���ɵ�SN����
	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();
	l_u32SN	= 0;

	while(1)
	{
		l_u32SNTimer	= SNALIAS;
		l_u32Value	= 0;
		l_u8DigitCnt= 0;
		
		//������
		GUI_ClearSCR();
		LCDPRINTFC(16		, 32	, "SN:         %d", l_u32SNTimer);
		LCDPRINTC(16		, 64	, "���������: ");
		
		while (1)		//�жϰ�����ˢ������
		{
			//�жϰ���
			l_u8Key	= KeyValue;
			switch(l_u8Key)
			{
				case KB_ESC:
					ToBreak(l_u8ControlCode);
					break;

				case KB_1:		
				case KB_2:		
				case KB_3:		
				case KB_4:		
				case KB_5:		
				case KB_6:		
				case KB_7:		
				case KB_8:		
				case KB_9:		
				case KB_0:
					NotReDraw(l_u8ControlCode);
					l_u8DigitCnt++;
					l_u32Value	*= 10;
					l_u32Value	+= l_u8Key;
					if (l_u8DigitCnt > 12)
						return;
					break;	
				
				case KB_BKSP:		//��ɾһ������
					NotReDraw(l_u8ControlCode);
					if (l_u8DigitCnt>0)
						l_u8DigitCnt	--;
					l_u32Value	/= 10;
					break;	
				
				case KB_ENTER:		//ϵͳ��ʼ��
					l_u32SN=bcd(l_u32SNTimer,6)*bcd(l_u32SNTimer,1)*10000+bcd(l_u32SNTimer,5)*bcd(l_u32SNTimer,2)*100+bcd(l_u32SNTimer,4)*bcd(l_u32SNTimer,3) ;
					l_u32SN=(bcd(l_u32SN,1)<<20)+(bcd(l_u32SN,2)<<16)+(bcd(l_u32SN,3)<<12)+(bcd(l_u32SN,4)<<8)+(bcd(l_u32SN,5)<<4)+bcd(l_u32SN,6);
					
					if (l_u32SN == l_u32Value || l_u32Value == SUPERPWD)
						UISystemInit();
					
					l_u32Value	= 0;
					ToBreak(l_u8ControlCode);
					break;	
				
				default:
					NotReDraw(l_u8ControlCode);
					break;
			}	//switch(KeyValue)
			ClearKeyValue();
					
			//�Ƿ��ػ�
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;

			//ˢ����Ļ
			LCDPRINTC(112 + l_u8DigitCnt<<3 	, 64	, "  ");

			for (l_u8Tmp1 = 0; l_u8Tmp1 < l_u8DigitCnt; l_u8Tmp1 ++)
			{
				LCDPRINTC(112 + l_u8Tmp1<<3 	, 64	, "*");
			}

			WAITSCREENREFRESH();		//�ȴ�ˢ���ź���
		}

		//�Ƿ��˳�
		if (IfBreak(l_u8ControlCode))
			break;

	}
}

							  
#if	YBVERSION >= 30		//3.0�Ǳ���
/*********************************************************************************************************
** Function name:		UIViewStartUpTime
** Descriptions:		�鿴ϵͳ����ʱ���¼
** input parameters:	None 
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110530	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void UIViewStartUpTime(void)
{
	uint16	l_u16Tmp,l_u16Tmp2;
	uint16	l_u16RecIndex;
	uint32	l_u32StartCnt;
	uint8	l_au8RecData[8];
	uint8	l_u8Status;
	uint8	l_u8Row;
	
	l_u8Status	= 0x01;
			
	//������
	GUI_ClearSCR();
	ClearKeyValue();

	ReadC256(STARTRECINDEXADDR,(uint8 *)&l_u16RecIndex,2);		//�ҵ���Ч��¼��
	l_u16RecIndex	&= 0x3ff;
	l_u16Tmp		= l_u16RecIndex;
	l_u16RecIndex	++;
	l_u16RecIndex	&= 0x3ff;
	l_u32StartCnt	= g_u32StartupTime;

	l_u16Tmp2	= 0;		//����ʱ����������ڷ�����ʾ
	while ((l_u32StartCnt > 0)&&(l_u16Tmp != l_u16RecIndex))
	{
		//��ȡ����ʱ��8B����
		ReadNORFlash(NORSTARTREC	+ (l_u16Tmp << 3), 8, &l_au8RecData[0]);
		
		//�ж��Ƿ�Ϊ��Ч����
		if (l_au8RecData[7] != 0xaa)	//��Ч
		{
			break;	
		}

		//��ʾ����ʱ��
		g_sstTempTime.u16Year	= (l_au8RecData[0]	+ (l_au8RecData[1] <<8));
		g_sstTempTime.u8Month	= l_au8RecData[2];
		g_sstTempTime.u8Day		= l_au8RecData[3];
		g_sstTempTime.u8Hour	= l_au8RecData[4];
		g_sstTempTime.u8Minute	= l_au8RecData[5];
		g_sstTempTime.u8Second	= l_au8RecData[6];

		l_u8Row	= (l_u16Tmp2 & 0x07)<<4;
		l_u16Tmp2	++;
		l_u16Tmp2	&= 0x3ff;
		LCDPRINTFC(0	, l_u8Row	, "%04d "	,l_u16Tmp2);
		
		sprintf(m_acTmp, ": %04d-%02d-%02d %02d:%02d:%02d %04d", g_sstTempTime.u16Year, g_sstTempTime.u8Month, 
			g_sstTempTime.u8Day, g_sstTempTime.u8Hour, g_sstTempTime.u8Minute, g_sstTempTime.u8Second,l_u16Tmp);

		LCDPRINTC(32	, l_u8Row	, m_acTmp);
		
		l_u16Tmp	--;
		l_u16Tmp	&= 0x3ff;

		//��ͣ 		
		l_u32StartCnt	--;
		ClearKeyValue();
		if (l_u16Tmp2 % 8 == 0)
		{
			while (KeyValue == 0xff)		//�а���
			{
				WAITSCREENREFRESH();		//�ȴ�ˢ���ź���
			}
			if (KeyValue == KB_ESC)
			{
				l_u8Status	&= ~0x10;
				return;
			}
			ClearKeyValue();
			GUI_ClearSCR();
			if (l_u32StartCnt == 0)
				return;
		}
	}
	
	if (l_u8Status & 0x01)		//�����ֹ�˳�����ȴ�����
	{							
		ClearKeyValue();
		while (KeyValue == 0xff)		//�а���
		{
			WAITSCREENREFRESH();		//�ȴ�ˢ���ź���
			//OSTimeDly(LOOPTIME);		//��ʱ����ѯʱ��
		}
		if (KeyValue == KB_ESC)
			return;
		ClearKeyValue();
		GUI_ClearSCR();
	}
}

/*********************************************************************************************************
** Function name:		UIViewIPInfo
** Descriptions:		IP������444
** input parameters:	None 
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110623	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void UIViewIPInfo(void)
{
	uint8	l_u8ControlCode;//����ָ��

	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();

	while(1)
	{
		//������
		GUI_ClearSCR();
		
		GUI_Line(0		, 16	, 239	, 16	, 1);
		GUI_Line(0		, 128	, 239	, 128	, 1);
		
		LCDPRINTC(0		, 0		, " �������ò���          0-���� ");
		
		LCDPRINTC(0		, 17	, "1.MAC��ַ");
//		memset(m_acTmp,0, sizeof(m_acTmp));
		sprintf(m_acTmp, "%2x-%2x-%2x-%2x-%2x-%2x", IPINFOALIAS.au8MACAddr[0], IPINFOALIAS.au8MACAddr[1], IPINFOALIAS.au8MACAddr[2], IPINFOALIAS.au8MACAddr[3], IPINFOALIAS.au8MACAddr[4], IPINFOALIAS.au8MACAddr[5]); 
		LCDPRINTC(96	, 17	, m_acTmp);

		LCDPRINTC(0		, 33	, "2.����IP");
//		memset(m_acTmp,0, sizeof(m_acTmp));
		sprintf(m_acTmp, "%3d.%3d.%3d.%3d", IPINFOALIAS.au8IPAddr[0], IPINFOALIAS.au8IPAddr[1], IPINFOALIAS.au8IPAddr[2], IPINFOALIAS.au8IPAddr[3]); 
		LCDPRINTC(96	, 33	, m_acTmp);

		LCDPRINTC(0		, 48	, "3.���ض˿�");
//		memset(m_acTmp,0, sizeof(m_acTmp));
		sprintf(m_acTmp, "%-d", IPINFOALIAS.u32LocalPortNO); 
		LCDPRINTC(96	, 48	, m_acTmp);

		LCDPRINTC(0		, 64	, "4.��������");
//		memset(m_acTmp,0, sizeof(m_acTmp));
		sprintf(m_acTmp, "%3d.%3d.%3d.%3d", IPINFOALIAS.au8SubMask[0], IPINFOALIAS.au8SubMask[1], IPINFOALIAS.au8SubMask[2], IPINFOALIAS.au8SubMask[3]); 
		LCDPRINTC(96	, 64	, m_acTmp);
		
		LCDPRINTC(0		, 79	, "5.Ĭ������");
//		memset(m_acTmp,0, sizeof(m_acTmp));
		sprintf(m_acTmp, "%3d.%3d.%3d.%3d", IPINFOALIAS.au8GatewayIP[0], IPINFOALIAS.au8GatewayIP[1], IPINFOALIAS.au8GatewayIP[2], IPINFOALIAS.au8GatewayIP[3]); 
		LCDPRINTC(96	, 79	, m_acTmp);

		LCDPRINTC(0		, 95	, "6.Զ��IP");
//		memset(m_acTmp,0, sizeof(m_acTmp));
		sprintf(m_acTmp, "%3d.%3d.%3d.%3d", IPINFOALIAS.au8ServerIP[0], IPINFOALIAS.au8ServerIP[1], IPINFOALIAS.au8ServerIP[2], IPINFOALIAS.au8ServerIP[3]); 
		LCDPRINTC(96	, 95	, m_acTmp);

		LCDPRINTC(0		, 111	, "7.Զ�̶˿�");
//		memset(m_acTmp,0, sizeof(m_acTmp));
		sprintf(m_acTmp, "%-d", IPINFOALIAS.u32ServerPortNO); 
		LCDPRINTC(96	, 111	, m_acTmp);

		while (1)		//�жϰ�����ˢ������
		{
			ToReDraw(l_u8ControlCode);
					
			//�жϰ���
			switch(KeyValue)
			{
				case KB_ESC:
				case KB_ENTER:
					ToBreak(l_u8ControlCode);
					break;
					
				case KB_1:		//MAC�����޸�
					NotReDraw(l_u8ControlCode);
					break;

				case KB_2:		//����IP
					UISetValueParamIP("����IP", IPINFOALIAS.au8IPAddr);
					break;	
					
				case KB_3:		//���ض˿�
					UISetValueParamU32("���ض˿�", &(IPINFOALIAS.u32LocalPortNO), 0, 0);
					break;	
					
				case KB_4:		//��������
					UISetValueParamIP("��������", IPINFOALIAS.au8SubMask);
					break;	
					
				case KB_5:		//Ĭ������
					UISetValueParamIP("Ĭ������", IPINFOALIAS.au8GatewayIP);
					break;	
					
				case KB_6:		//Զ��IP
					UISetValueParamIP("Զ��IP", IPINFOALIAS.au8ServerIP);
					break;	
					
				case KB_7:		//Զ�̶˿�
					UISetValueParamU32("Զ�̶˿�", &(IPINFOALIAS.u32ServerPortNO), 0, 0);
					break;	
					
				case KB_0:		//����
					SaveNetInfo();
					ToReDraw(l_u8ControlCode);
					break;
					
				case KB_I:		//��ʼ��
					InitNetParam();
					ToReDraw(l_u8ControlCode);
					break;
					
				default:
					NotReDraw(l_u8ControlCode);
					
					break;
			}	//switch(KeyValue)
			ClearKeyValue();

			//�Ƿ��ػ�
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;

			WAITSCREENREFRESH();		//�ȴ�ˢ���ź���
			//OSTimeDly(LOOPTIME);		//��ʱ����ѯʱ��
		}

		//�Ƿ��˳�
		if (IfBreak(l_u8ControlCode))
			break;
	}
}

/*********************************************************************************************************
** Function name:		UISetValueParamIP
** Descriptions:		�޸�IP����
** input parameters:	p_pcName		������
**						p_pu8IP		 	����ָ��
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110509	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void UISetValueParamIP(char * p_pcName, uint8 * p_pu8IP)
{
	uint8	l_u8ControlCode;		//����ָ��
	uint32	l_u32Value	= 0;		//��������
	uint8	l_u8Status;				//���ܼ�״̬
	uint8	l_u8Key;
	uint8	l_au8TmpIP[4];
	char	l_acTmp[30];

	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();

	while(1)
	{
		l_u32Value	= 0;
		l_u8Status	= 0;

		//������
		GUI_ClearSCR();
		memset(l_acTmp, 0, 30);
		sprintf(l_acTmp,"%s",p_pcName);
		LCDPRINTC(0		, 0		, l_acTmp);		   
		memset(l_acTmp	, 0		, 30);
		sprintf(l_acTmp	,"%3d.%3d.%3d.%3d"	, *p_pu8IP, *(p_pu8IP+1), *(p_pu8IP+2), *(p_pu8IP+3));
		LCDPRINTC(24	, 20	, l_acTmp);
		LCDPRINTC(0		, 36	, " Esc�˳�, F1���޸�");
		
		GUI_Line(0		, 18	, 239	, 18	, 1);
		GUI_Line(0		, 124	, 239	, 124	, 1);
		GUI_Line(0		, 127	, 239	, 127	, 1);
		
		while (1)		//�жϰ�����ˢ������
		{
			//�жϰ���
			l_u8Key	= KeyValue;
			switch(l_u8Key)
			{
				case KB_ESC:
					ToBreak(l_u8ControlCode);
					break;

				case KB_1:		
				case KB_2:		
				case KB_3:		
				case KB_4:		
				case KB_5:		
				case KB_6:		
				case KB_7:		
				case KB_8:		
				case KB_9:		
				case KB_0:
					if (l_u8Status != 0)
					{
						l_u32Value	*= 10;
						l_u32Value	+= l_u8Key;
					}
					NotReDraw(l_u8ControlCode);
					break;	
				
				case KB_BKSP:		//��ɾһ������
					NotReDraw(l_u8ControlCode);
					l_u32Value	/= 10;
					break;	
				
				case KB_F1:		//�޸�״̬����ʼ��������
					if (l_u8Status == 0)
					{
						l_u8Status	= 1;
						l_u32Value	= 0;
						LCDPRINTC(16		, 56	,"��������ֵ: ");
					}
					NotReDraw(l_u8ControlCode);
					break;	
				
				case KB_ENTER:		//ȷ������
					NotReDraw(l_u8ControlCode);
					switch (l_u8Status)
					{
						case 1:
						case 2:
						case 3:
						case 4:
							if (l_u32Value <256)
							{	
								l_au8TmpIP[l_u8Status-1]	= l_u32Value & 0xff;
								LCDPRINTFC((l_u8Status<<5) - 8	, 76	, "%3d"	, l_au8TmpIP[l_u8Status-1]);
								if (l_u8Status == 4)
								{
									LCDPRINTC(32		, 92	, "�������. ");	
									LCDPRINTC(32		, 108	, "Enter:���� Esc:ȡ��");
								}
								else
								{											
									LCDPRINTC((l_u8Status<<5) + 16	, 76	, ".");
								}
							}
							else
								ToReDraw(l_u8ControlCode);
									
							break;
						
						case 5:		//ȷ�ϱ���
							//����
							*p_pu8IP		= l_au8TmpIP[0];
							*(p_pu8IP+1)	= l_au8TmpIP[1];
							*(p_pu8IP+2)	= l_au8TmpIP[2];
							*(p_pu8IP+3)	= l_au8TmpIP[3];

							ToReDraw(l_u8ControlCode);
							break;

						default:
							ToReDraw(l_u8ControlCode);
							break;							
					}
					l_u8Status	++;

					l_u32Value	= 0;
					break;	
				
				default:
					NotReDraw(l_u8ControlCode);
					break;
			}	//switch(KeyValue)
			ClearKeyValue();

			//�Ƿ��ػ�
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;

			if (l_u8Status	> 0)
			{
				if (l_u32Value != 0)
					LCDPRINTFC((l_u8Status<<5) - 8	, 76	, "%u    "	, l_u32Value);
				else
					LCDPRINTC((l_u8Status<<5) - 8	, 76	, "      ");		
			}

			WAITSCREENREFRESH();		//�ȴ�ˢ���ź���
		}

		//�Ƿ��˳�
		if (IfBreak(l_u8ControlCode))
			break;
	}
}

/*********************************************************************************************************
** Function name:		UISetValueParamU32
** Descriptions:		ͨ�ò������ý���,����U32����
** input parameters:	p_pcName		������
**						p_pu32Param 	����ָ��
**						p_u32Max		�������� 
**						p_u32Min		��������
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110509	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void UISetValueParamU32(char * p_pcName, uint32 * p_pu32Param, uint32 p_u32Min, uint32 p_u32Max)
{
	uint8	l_u8ControlCode;//����ָ��
	uint32	l_u32Value	= 0;		//��������
	uint8	l_u8Status;		//���ܼ�״̬
	uint8	l_u8Key;
	char	l_acTmp[30];

	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();

	while(1)
	{
		l_u32Value	= 0;
		l_u8Status	= 0;

		//������
		GUI_ClearSCR();
		memset(l_acTmp, 0, 30);
		sprintf(l_acTmp,"%s: %u",p_pcName,*p_pu32Param);
		LCDPRINTC(0		, 0		, l_acTmp);
		LCDPRINTC(0		, 32	, " Esc�˳�, F1���޸�");
		
		GUI_Line(0		, 18	, 239	, 18	, 1);
		GUI_Line(0		, 124	, 239	, 124	, 1);
		GUI_Line(0		, 127	, 239	, 127	, 1);
		
		while (1)		//�жϰ�����ˢ������
		{
			//�жϰ���
			l_u8Key	= KeyValue;
			switch(l_u8Key)
			{
				case KB_ESC:
					ToBreak(l_u8ControlCode);
					break;

				case KB_1:		
				case KB_2:		
				case KB_3:		
				case KB_4:		
				case KB_5:		
				case KB_6:		
				case KB_7:		
				case KB_8:		
				case KB_9:		
				case KB_0:
					if (l_u8Status != 0)
					{
						l_u32Value	*= 10;
						l_u32Value	+= l_u8Key;
					}
					NotReDraw(l_u8ControlCode);
					break;	
				
				case KB_BKSP:		//��ɾһ������
					NotReDraw(l_u8ControlCode);
					l_u32Value	/= 10;
					break;	
				
				case KB_F1:		//�޸�״̬����ʼ��������
					if (l_u8Status == 0)
					{
						l_u8Status	= KB_F1;
						l_u32Value	= 0;
						memset(l_acTmp, 0, 30);
						sprintf(l_acTmp," ��������ֵ(%u-%u):",p_u32Min,p_u32Max);
						LCDPRINTC(16		, 56	, l_acTmp);
					}
					NotReDraw(l_u8ControlCode);
					break;	
				
				case KB_ENTER:		//ȷ������
					if (l_u8Status	== KB_F1)
					{
						if ((p_u32Max == 0 && p_u32Min == 0) || (l_u32Value >= p_u32Min && l_u32Value <= p_u32Max))
						{
							*p_pu32Param	= l_u32Value;
						}
						l_u32Value	= 0;
						l_u8Status	= 0;
						ToReDraw(l_u8ControlCode);
					}
					else
						NotReDraw(l_u8ControlCode);
					break;	
				
				default:
					NotReDraw(l_u8ControlCode);
					break;
			}	//switch(KeyValue)
			ClearKeyValue();

			//�Ƿ��ػ�
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;

			//ˢ����Ļ
			if (l_u8Status == KB_F1)		//����״̬
			{
				if (l_u32Value == 0)
					LCDPRINTC(56		, 80	, "         ");
				else
					LCDPRINTFC(56		, 80	, "%u      "	, l_u32Value);
			}

			WAITSCREENREFRESH();		//�ȴ�ˢ���ź�
		}

		//�Ƿ��˳�
		if (IfBreak(l_u8ControlCode))
			break;
	}
}

#endif	//#if	YBVERSION >= 30		//3.0�Ǳ���

/*********************************************************************************************************
** Function name:		UISetTime
** Descriptions:		�趨ϵͳʱ��
** input parameters:	None 
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110509	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void UISetTime(void)
{
	uint8	l_u8ControlCode;//����ָ��
	uint32	l_u32Value;		//��������
	uint8	l_u8Status;		//���ܼ�״̬
	uint16	l_u16X;			//����X
	uint8	l_u8Key;

	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();

	while(1)
	{
		l_u32Value	= 0;
		l_u8Status	= 0;

		//������
		GUI_ClearSCR();
		LCDPRINTC(0		, 0		, "ʱ���趨:    Esc�˳�, F1���޸�");
		
		LCDPRINTC(16	, 20	, "��ǰʱ��:");
		LCDPRINTC(64	, 36	, "-");
		LCDPRINTC(88	, 36	, "-");
		LCDPRINTC(144	, 36	, ":");
		LCDPRINTC(168	, 36	, ":");
		
		GUI_Line(0		, 18	, 239	, 18	, 1);
		GUI_Line(0		, 125	, 239	, 125	, 1);
		GUI_Line(0		, 127	, 239	, 127	, 1);
			
		while (1)		//�жϰ�����ˢ������
		{
			//�жϰ���
			l_u8Key	= KeyValue;
			switch(l_u8Key)
			{
				case KB_ESC:
					ToBreak(l_u8ControlCode);
					break;

				case KB_1:		
				case KB_2:		
				case KB_3:		
				case KB_4:		
				case KB_5:		
				case KB_6:		
				case KB_7:		
				case KB_8:		
				case KB_9:		
				case KB_0:
					if (l_u8Status != 0)
					{
						l_u32Value	*= 10;
						l_u32Value	+= l_u8Key;
					}
					NotReDraw(l_u8ControlCode);
					break;	
				
				case KB_BKSP:		//��ɾһ������
					NotReDraw(l_u8ControlCode);
					l_u32Value	/= 10;
					break;	
				
				case KB_F1:		//�޸�״̬����ʼ��������
					if (l_u8Status == 0)
					{
						l_u8Status	= 1;
						l_u32Value	= 0;
						LCDPRINTC(16		, 60	,"�趨ʱ��:");
					}
					NotReDraw(l_u8ControlCode);
					break;	
				
				case KB_ENTER:		//ȷ������
					NotReDraw(l_u8ControlCode);
					switch (l_u8Status)
					{
						case 1:		//��
							if (l_u32Value > 1999 && l_u32Value < 2100)
							{	
								TMPTIMEALIAS.u16Year	= l_u32Value;
								LCDPRINTFC(32		, 76	, "%04d"	, l_u32Value);
								LCDPRINTC(64		, 76	, "-");
							}
							else
								ToReDraw(l_u8ControlCode);
									
							break;
						
						case 2:		//��
							if (l_u32Value > 0 && l_u32Value < 13)
							{
								TMPTIMEALIAS.u8Month	= l_u32Value & 0xff;
								LCDPRINTFC(72		, 76	, "%02d"	, l_u32Value);
								LCDPRINTC(88		, 76	, "-");
			
							}
							else
								ToReDraw(l_u8ControlCode);
									
							break;

						case 3:		//��
							if (l_u32Value > 0 && l_u32Value < 32)
							{	
								TMPTIMEALIAS.u8Day	= l_u32Value & 0xff;
								LCDPRINTFC(96		, 76	, "%02d"	, l_u32Value);
							}
							else
								ToReDraw(l_u8ControlCode);
									
							break;
						
						case 4:		//ʱ:0~23
							if (l_u32Value < 24)
							{	
								TMPTIMEALIAS.u8Hour	= l_u32Value & 0xff;
								LCDPRINTFC(128	, 76	, "%02d"	, l_u32Value);
								LCDPRINTC(144	, 76	, ":");
							}
							else
								ToReDraw(l_u8ControlCode);	
							break;
						
						case 5:		//��:0~59
							if (l_u32Value < 60)
							{	
								TMPTIMEALIAS.u8Minute	= l_u32Value & 0xff;
								LCDPRINTFC(152	, 76	, "%02d"	, l_u32Value);	
								LCDPRINTC(168	, 76	, ":");			
							}
							else
								ToReDraw(l_u8ControlCode);	
							break;
						
						case 6:		//��:0~59
							if (l_u32Value < 60)
							{
								TMPTIMEALIAS.u8Second	= l_u32Value & 0xff;
								LCDPRINTFC(176	, 76	, "%02d"	, l_u32Value);
								LCDPRINTC(32		, 92	, "�������. ");	
								LCDPRINTC(32		, 108	, "Enter:���� Esc:ȡ��");
							}
							else
								ToReDraw(l_u8ControlCode);	
							break;

						case 7:		//ȷ�ϱ���
							//����
							SaveTime(TMPTIMEALIAS);
							ToReDraw(l_u8ControlCode);
							break;

						default:
							ToReDraw(l_u8ControlCode);
							break;							
					}
					l_u8Status	++;

					l_u32Value	= 0;
					break;	
				
//				case 0xff:
				default:
					NotReDraw(l_u8ControlCode);
					break;
			}	//switch(KeyValue)
			ClearKeyValue();

			//�Ƿ��ػ�
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;

			//ˢ����Ļ
			LCDPRINTFC(32		, 36	, "%04d"	, CURTIMEALIAS.u16Year);
			LCDPRINTFC(72		, 36	, "%02d"	, CURTIMEALIAS.u8Month);
			LCDPRINTFC(96		, 36	, "%02d"	, CURTIMEALIAS.u8Day);

			LCDPRINTFC(128		, 36	, "%02d"	, CURTIMEALIAS.u8Hour);
			LCDPRINTFC(152		, 36	, "%02d"	, CURTIMEALIAS.u8Minute);	
			LCDPRINTFC(176		, 36	, "%02d"	, CURTIMEALIAS.u8Second);

			switch (l_u8Status)		//��ʾ���������
			{
				case 1:		//��
					l_u16X	= 32;		
					break;
				
				case 2:		//��
					l_u16X	= 72;		
					break;

				case 3:		//��
					l_u16X	= 96;		
					break;
				
				case 4:		//ʱ:0~23
					l_u16X	= 128;		
					break;
				
				case 5:		//��:0~59
					l_u16X	= 152;		
					break;
				
				case 6:		//��:0~59
					l_u16X	= 176;		
					break;

				default:
					l_u16X	= 0xff;		
					break;							
			}
			
			if (l_u16X	!= 0xff)
			{
				if (l_u32Value != 0)
					LCDPRINTFC(l_u16X	, 76	, "%u    "	, l_u32Value);
				else
					LCDPRINTC(l_u16X	, 76	, "      ");		
			}
			WAITSCREENREFRESH();		//�ȴ�ˢ���ź�
		}

		//�Ƿ��˳�
		if (IfBreak(l_u8ControlCode))
			break;

	}
}

/*********************************************************************************************************
** Function name:		UIF5Code
** Descriptions:		F5�������
** input parameters:	None 
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110509	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void UIF5Code(void)
{
	uint8	l_u8ControlCode;//����ָ��
	uint8	l_u8Tmp1;
	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();
	
	BackGroundSave();
	BackGroundON();		
	while(1)
	{
		//������
		GUI_ClearSCR();
		for(l_u8Tmp1 = 0; l_u8Tmp1 < 16; l_u8Tmp1++)
		{
			LCDPRINTFC(l_u8Tmp1*8+48	, 32	, "%d"	, (l_u8Tmp1+1)%10);
		}
	   	LCDPRINTC(0		, 0		, "̥��״̬");
	   	LCDPRINTC(0		, 32	, "���:");
	   	LCDPRINTC(0		, 48	, "״̬:");
	   	LCDPRINTC(0		, 64	, "����:");

		while (1)		//�жϰ�����ˢ������
		{
			//�жϰ���
			switch(KeyValue)
			{
				case KB_ESC:
				case KB_ENTER:
				case KB_F5:
					ToBreak(l_u8ControlCode);
					break;
					
				default:
					NotReDraw(l_u8ControlCode);
					break;
			}	//switch(KeyValue)
			ClearKeyValue();
					
			//�Ƿ��ػ�
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;

			//ˢ����Ļ
			for(l_u8Tmp1 = 0; l_u8Tmp1 < 16; l_u8Tmp1++)
			{
				if (LZSIGNAL & (0x01<< l_u8Tmp1))
					LCDPRINTC(l_u8Tmp1*8+48	, 64	, "*");
				else 
					LCDPRINTC(l_u8Tmp1*8+48	, 64	, "O");

				if (LZSIGNALLast & (0x01<< l_u8Tmp1))
					LCDPRINTC(l_u8Tmp1*8+48	, 48	, "*");
				else 
					LCDPRINTC(l_u8Tmp1*8+48	, 48	, "O");
			}
			OSTimeDly(LOOPTIME*2);		//��ʱ20ms����ѯʱ��
		}

		//�Ƿ��˳�
		if (IfBreak(l_u8ControlCode))
			break;
	}
	BackGroundRecover();
}

/*********************************************************************************************************
** Function name:		UIF4Code
** Descriptions:		�鿴F4��Ϣ����
** input parameters:	None 
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110507	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void UIF4Code(void)
{
	ClearKeyValue();
	
	BackGroundSave();
	BackGroundON();
	//������
	GUI_ClearSCR();
	
	LCDPRINTC(0		, 0		, "F4����:");
	
#if  SHOWVEHPASSDEBUG > 0	//��ʾ�������Դ���
 	LCDPRINTC(0		, 16	, (char *)g_chVehPassDebug);
#else
	LCDPRINTC(0		, 16	, "F4���빦��δ����");
#endif
		
	while (KeyValue == 0xff)		//�а���
	{	
#if	SENDWAVEENABLE > 0		//ʹ�ܷ�����		
		LCDPRINTFC(0	, 97	, "READ:%5u "	, g_u16WeightBufReadIndex);
		LCDPRINTFC(120	, 97	, "SAVE:%5u "	, g_u16WeightBufSavIndex);
#endif	//#if	SENDWAVEENABLE > 0		//ʹ�ܷ�����	

		WAITSCREENREFRESH();		//�ȴ�ˢ���ź�
	}
	ClearKeyValue();   
	BackGroundRecover();
}

/*********************************************************************************************************
** Function name:		UIF3Code
** Descriptions:		�鿴F3��Ϣ����
** input parameters:	None 
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110507	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void UIF3Code(void)
{
	ClearKeyValue();
	BackGroundSave();
	BackGroundON();
	//������
	GUI_ClearSCR();
	
	LCDPRINTFC(0	, 0		, "�汾��: %2dNT"	, YBVERALIAS);
 	LCDPRINTC(104	, 0		, VERSIONALIAS);
	
	//����ʱ��
	sprintf(m_acTmp, "����ʱ��: %.4d-%.2d-%.2d %.2d:%.2d:%.2d", g_sstStartTime.u16Year, g_sstStartTime.u8Month, g_sstStartTime.u8Day, g_sstStartTime.u8Hour, g_sstStartTime.u8Minute, g_sstStartTime.u8Second);
	LCDPRINTC(0		, 18	, (char *)m_acTmp);
	
	sprintf(m_acTmp, "��������: %-5d ������: %-5d ", g_u32StartupTime	, POINTRATE);
	LCDPRINTC(0		, 40	, (char *)m_acTmp);
								  
#if ISDEBUG				//����״̬
	LCDPRINTC(232	, 0		, "*");	

	
#if	SENDWAVEENABLE > 0		//ʹ�ܷ�����

#if	SENDWAVEBYSP > 0	
	LCDPRINTC(216	, 0		, "��");
#endif								  
#if	SENDWAVEBYNET > 0
	LCDPRINTC(200	, 0		, "��");
#endif

#endif	//#if	SENDWAVEENABLE > 0		//ʹ�ܷ�����		

#endif
	while (KeyValue == 0xff)		//�а���
	{
		WAITSCREENREFRESH();		//�ȴ�ˢ���ź�
	}
	if (KeyValue == KB_F1)
		ClearStartupCnt();

	ClearKeyValue(); 
	BackGroundRecover();
}

/*********************************************************************************************************
** Function name:		UICommonSet
** Descriptions:		��ͨ���ý��棬111
** input parameters:	None 
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110509	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void UICommonSet(void)
{
	uint8	l_u8ControlCode;//����ָ��
	
	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();

	while(1)
	{
		//������
		GUI_ClearSCR();
		LCDPRINTC(0		, 0		, " ϵͳ����");
		LCDPRINTC(112	, 0		, "0 ����  Esc �˳�" );
		LCDPRINTC(8		, 24	, "1. ʱ������");
		LCDPRINTC(8		, 40	, "2. ����������");
		LCDPRINTC(8		, 56	, "3. ����ģʽ");
		LCDPRINTC(8		, 72	, "4. ����ʹ��");
		LCDPRINTC(8		, 88	, "5. ͨѶЭ��");
		LCDPRINTC(8		, 104	, "F5 ���泵��");
		
		LCDPRINTC(128	, 24	, "6. ̨����");
		LCDPRINTC(128	, 40	, "7. ��Ȧץ��");
		LCDPRINTC(128	, 56	, "8. ���ᱨ��");
		LCDPRINTC(128	, 72	, "9. ץ��ʹ��");
		LCDPRINTC(128	, 88	, "F1 ���Ź�����");
		LCDPRINTC(128	, 104	, "F4 ���籣��");

		GUI_Line(0		, 18	, 239	, 18	, 1);
		GUI_Line(0		, 124	, 239	, 124	, 1);
		GUI_Line(0		, 127	, 239	, 127	, 1);
		
		while (1)		//�жϰ�����ˢ������
		{
			//�жϰ���
			switch(KeyValue)
			{
				case KB_ESC:
					ToBreak(l_u8ControlCode);
					break;

				case KB_1:		//ʱ������
					ToReDraw(l_u8ControlCode);
					UISetTime();
					break;	
				
				case KB_2:		//����������
					ToReDraw(l_u8ControlCode);
					UISetBaudRate();
					break;	
				
				case KB_3:		//����ģʽ
					ToReDraw(l_u8ControlCode);
					UISetCommandMode();
					break;	
				
				case KB_4:		//����ʹ��
					ToReDraw(l_u8ControlCode);
					UISetForwardEnable();
					break;	
				
				case KB_5:		//ͨѶЭ��
					ToReDraw(l_u8ControlCode);
					UISetProtocol();
					break;	
				
				case KB_6:		//̨����
					ToReDraw(l_u8ControlCode);
					UISetPlat();
					break;	
				
				case KB_7:		//��Ȧץ��
					ToReDraw(l_u8ControlCode);
					UISetLoop();
					break;	
				
				case KB_8:		//���Ŀ���
					ToReDraw(l_u8ControlCode);
					UISetLunZhouEnable();
					break;	
				
				case KB_9:		//ץ��ʹ��
					ToReDraw(l_u8ControlCode);
					UISetCapture();
					break;	
				
				case KB_0:		//��������
					ToReDraw(l_u8ControlCode);
					SaveParams();
					break;	
				
				case KB_F1:		//���Ź�
					ToReDraw(l_u8ControlCode);
					UISetDog();
					break;	
				
				case KB_F4:		//���籣��
					ToReDraw(l_u8ControlCode);
					UISetDiaodian();
					break;	
				
				case KB_F5:		//���泵��
					ToReDraw(l_u8ControlCode);
					UISetVehicleCache();
					break;	
				
				default:
					NotReDraw(l_u8ControlCode);
					break;
			}	//switch(KeyValue)
			ClearKeyValue();

			//�Ƿ��ػ�
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;

			WAITSCREENREFRESH();		//�ȴ�ˢ���ź�
		}

		//�Ƿ��˳�
		if (IfBreak(l_u8ControlCode))
			break;
	}
}

/*********************************************************************************************************
** Function name:		UIViewSetting
** Descriptions:		�鿴���ò������棬222
** input parameters:	None 
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110509	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void UIViewSetting(void)
{
	SetupParam l_ssTmp;
	GUI_ClearSCR();
	ClearKeyValue();

	//������
	ReadC256(LASTBDADDR,(uint8 *)&l_ssTmp, 1024);	

	GUI_Line(0		, 18	, 239	, 18	, 1);
	GUI_Line(0		, 124	, 239	, 124	, 1);
	GUI_Line(0		, 127	, 239	, 127	, 1);
		
	LCDPRINTC(0		, 0		, VERSIONALIAS);
	LCDPRINTC(176	, 0		, "��ǰ����");
//	memset(m_acTmp	, 0		, 30); 
	sprintf(m_acTmp	, "��  λ:%d+%d=%d",l_ssTmp.an32Zero[0],l_ssTmp.an32Zero[1],l_ssTmp.an32Zero[0]+l_ssTmp.an32Zero[1]);
	LCDPRINTC(0		, 20	, m_acTmp);
	
	LCDPRINTFC(0		, 36	, "��  ��:%5d,"	, l_ssTmp.an32AxGain[0]);
	LCDPRINTFC(112		, 36	, "%d  "	, l_ssTmp.an32AxGain[1]);
		  
	LCDPRINTFC(0		, 52	, "�ֶ�ֵ:%dkg"	, l_ssTmp.u8MotionScale);
	  
	LCDPRINTFC(0		, 68	, "������:%ukg"	, l_ssTmp.u32Full);
	LCDPRINTFC(0		, 84	, "��  ��:%u"	, l_ssTmp.u8Genzong);

	sprintf(m_acTmp	, "�޸�����:%04d-%02d-%02d",2000+l_ssTmp.u8Year,l_ssTmp.u8Month,l_ssTmp.u8Day);
	LCDPRINTC(0		, 104	, m_acTmp); 
 	 
	while (KeyValue == 0xff)		//�а���
	{
		WAITSCREENREFRESH();		//�ȴ�ˢ���ź�
	}
	ClearKeyValue();

	memset((uint8 *)&l_ssTmp, 0, 1024);
	
	ReadC256(HISTORYBDADDR,(uint8 *)&l_ssTmp, 1024);
	GUI_ClearSCR();
	GUI_Line(0		, 18	, 239	, 18	, 1);
	GUI_Line(0		, 124	, 239	, 124	, 1);
	GUI_Line(0		, 127	, 239	, 127	, 1);
	
	LCDPRINTC(0		, 0		, VERSIONALIAS);
	LCDPRINTC(176	, 0		, "��ʷ����");

	sprintf(m_acTmp	, "��  λ:%d+%d=%d",l_ssTmp.an32Zero[0],l_ssTmp.an32Zero[1],l_ssTmp.an32Zero[0]+l_ssTmp.an32Zero[1]);
	LCDPRINTC(0		, 20	, m_acTmp);
	
	LCDPRINTFC(0	, 36	, "��  ��:%5d,"	, l_ssTmp.an32AxGain[0]);
	LCDPRINTFC(112	, 36	, "%5d  "	, l_ssTmp.an32AxGain[1]);
		  
	LCDPRINTFC(0	, 52	, "�ֶ�ֵ:%dkg"	, l_ssTmp.u8MotionScale);
	  
	LCDPRINTFC(0	, 68	, "������:%ukg"	, l_ssTmp.u32Full);
	LCDPRINTFC(0	, 84	, "��  ��:%u"	, l_ssTmp.u8Genzong);

//	memset(m_acTmp, 0, 30);
	sprintf(m_acTmp	, "�޸�����:%04u-%02u-%02u",2000+l_ssTmp.u8Year,l_ssTmp.u8Month,l_ssTmp.u8Day);
	LCDPRINTC(0		, 104	, m_acTmp); 
 	
	while (KeyValue == 0xff)		//�ް���
	{
		WAITSCREENREFRESH();		//�ȴ�ˢ���ź�
	}
	ClearKeyValue();
}

/*********************************************************************************************************
** Function name:		UIViewModify
** Descriptions:		�鿴�ٶ��������棬333������̬�������ٶ�������
** input parameters:	None 
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110509	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void UIViewModify(void)
{
	uint8	l_u8Tmp1,l_u8Tmp2;
	
	//������
	GUI_ClearSCR();
	ClearKeyValue();
	l_u8Tmp2	= 0;
	for(l_u8Tmp1 = 0; l_u8Tmp1 < 16; l_u8Tmp1 ++)
	{
		sprintf(m_acTmp, "%2d: 0- %d", l_u8Tmp1, SETUPALIAS.au16StaticModify[0][l_u8Tmp1]);
		LCDPRINTC(0		, l_u8Tmp2	, m_acTmp);
		LCDPRINTFC(112	, l_u8Tmp2	, "1- %d",SETUPALIAS.au16StaticModify[1][l_u8Tmp1]);
		
		l_u8Tmp2	= (l_u8Tmp2 + 16) & 0x7f;
		
		if (l_u8Tmp1 % 8 == 7)
		{
			while (KeyValue == 0xff)		//�а���
			{
				WAITSCREENREFRESH();		//�ȴ�ˢ���ź�
			}
			if (KeyValue == KB_ESC)
				return;
			ClearKeyValue();
			GUI_ClearSCR();
		}
	}
	
	l_u8Tmp2	= 0;
	for(l_u8Tmp1 = 0; l_u8Tmp1 < 32; l_u8Tmp1 ++)
	{
		sprintf(m_acTmp,"%2dkm %d",l_u8Tmp1,SETUPALIAS.au16Speedmodify[l_u8Tmp1]);
		LCDPRINTC(0		, l_u8Tmp2	, m_acTmp);
		
		l_u8Tmp2	= (l_u8Tmp2 + 16) & 0x7f;
		if (l_u8Tmp1 % 8 == 7)
		{
			while (KeyValue == 0xff)		//�а���
			{
				WAITSCREENREFRESH();		//�ȴ�ˢ���ź�
			}

			if (KeyValue == KB_ESC)
				return;

			ClearKeyValue();
			GUI_ClearSCR();
		}
	}
}

/*********************************************************************************************************
** Function name:		UIViewThreshold
** Descriptions:		��ֵ������8968
** input parameters:	None 
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110509	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void UIViewThreshold(void)
{
	uint8	l_u8ControlCode;//����ָ��

	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();

	while(1)
	{
		//������
		GUI_ClearSCR();
		
		GUI_Line(0		, 18	, 239	, 18	, 1);
		GUI_Line(0		, 124	, 239	, 124	, 1);
		GUI_Line(0		, 127	, 239	, 127	, 1);
		
		LCDPRINTC(0		, 0		, " THRESHOLD");
		
		LCDPRINTFC(0		, 20	, "1.UP:  %d        ", THRESHOLDALIAS.u16UpValue);
		LCDPRINTFC(0		, 36	, "2.DN:  %d        ", THRESHOLDALIAS.u16DownValue);
		LCDPRINTFC(0		, 52	, "3.FD:  %d        ", THRESHOLDALIAS.u16ForwardWidth);
		LCDPRINTFC(0		, 68	, "4.RIF: %d        ", THRESHOLDALIAS.u16FilterLevel);
		LCDPRINTFC(0		, 84	, "5.WT:  %d        ", THRESHOLDALIAS.u16AxleWidth);
	
		LCDPRINTC(0		, 104	, "0.����");
				
		while (1)		//�жϰ�����ˢ������
		{
			ToReDraw(l_u8ControlCode);
					
			//�жϰ���
			switch(KeyValue)
			{
				case KB_ESC:
				case KB_ENTER:
					ToBreak(l_u8ControlCode);
					break;
					
				case KB_1:		//�ϳ�
					UISetValueParamU16("UP", &(THRESHOLDALIAS.u16UpValue), 100, 500);
					break;

				case KB_2:		//�³�
					UISetValueParamU16("DN", &(THRESHOLDALIAS.u16DownValue), 60, 300);
					break;	
					
				case KB_3:		//�����б���ֵ
					ClearKeyValue();
					UISetValueParamU16("FD", &(THRESHOLDALIAS.u16ForwardWidth), 60, 500);
					break;	
					
				case KB_4:		//�˲�����
					UISetValueParamU16("RIF", &(THRESHOLDALIAS.u16FilterLevel), 3, 128);
					break;	
					
				case KB_5:		//������С���
					UISetValueParamU16("WT", &(THRESHOLDALIAS.u16AxleWidth), 50, 500);
					break;	
					
				case KB_0:		//����
					SaveThreshold();
					break;
					
				default:
					NotReDraw(l_u8ControlCode);
					
					break;
			}	//switch(KeyValue)
			ClearKeyValue();

			//�Ƿ��ػ�
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;

			WAITSCREENREFRESH();		//�ȴ�ˢ���ź�
		}

		//�Ƿ��˳�
		if (IfBreak(l_u8ControlCode))
			break;
	}
}

/*********************************************************************************************************
** Function name:		UIBDScale
** Descriptions:		���ö���̬�ֶȽ���
** input parameters:	p_u8Motion ����̬��־��0��ʾ��̬��1��ʾ��̬ 
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110509	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  	ZHANG Ye	
** Modified date:	  	20110801
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void UIBDScale(uint8 p_u8Motion)
{
	uint8	l_u8ControlCode;//����ָ��
	uint8 *	l_pu8Scale;
	switch (p_u8Motion)
	{
		case UI_STATIC:	 //��̬
			l_pu8Scale = &SETUPALIAS.u8StaticScale;
			break;
			
		case UI_MOTION:	 //��̬
			l_pu8Scale = &SETUPALIAS.u8MotionScale;
			break;

		default:
			return;
	}
	
	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();

	while(1)
	{
		//������
		GUI_ClearSCR();
		if (p_u8Motion == UI_MOTION)		//��̬
			LCDPRINTC(0		, 0		, "��̬�ֶ��趨         Esc-����");
		else 				//��̬
			LCDPRINTC(0		, 0		, "��̬�ֶ��趨         Esc-����");

		LCDPRINTC(0		, 19	, "��ѡ��ֶ�ֵ:");
		
		LCDPRINTC(64	, 40	, "1--1   kg");
		
		LCDPRINTC(64	, 56	, "2--10  kg");
		LCDPRINTC(64	, 72	, "3--20  kg");
		LCDPRINTC(64	, 88	, "4--50  kg");
		LCDPRINTC(64	, 104	, "5--100 kg");
		
		GUI_Line(0		, 18	, 239	, 18	, 1);
		GUI_Line(0		, 124	, 239	, 124	, 1);
		GUI_Line(0		, 127	, 239	, 127	, 1);
	
		while (1)		//�жϰ�����ˢ������
		{
			//�жϰ���
			switch(KeyValue)
			{
				case KB_ESC:
				case KB_ENTER:
					ToBreak(l_u8ControlCode);
					break;
					
				case KB_1:
					*l_pu8Scale		= 1;
					break;

				case KB_2:
					*l_pu8Scale		= 10;
					break;

				case KB_3:
					*l_pu8Scale		= 20;
					break;

				case KB_4:
					*l_pu8Scale		= 50;
					break;

				case KB_5:
					*l_pu8Scale		= 100;
					break;
	
				default:
					NotReDraw(l_u8ControlCode);
					break;
			}	//switch(KeyValue)
			ClearKeyValue();

			//�Ƿ��ػ�
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;

			//ˢ����Ļ
			if (*l_pu8Scale == 0)
				*l_pu8Scale		= 50;

			LCDPRINTFC(112	, 19	, "%ukg    "	, *l_pu8Scale);

			WAITSCREENREFRESH();		//�ȴ�ˢ���ź�
		}

		//�Ƿ��˳�
		if (IfBreak(l_u8ControlCode))
			break;
	}
}

/*********************************************************************************************************
** Function name:		UIBDStaticModify
** Descriptions:		���þ�̬�������棬���մ�������������
** input parameters:	None 
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110509	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void UIBDStaticModify(void)
{
	uint8	l_u8ControlCode;//����ָ��
	uint8	l_u8Status;		//״̬
	uint8	l_u8Tmp1;		//��ʱ����
	uint8	l_u8Tmp2;
	uint8	l_u8Tmp3;

	ResetControlCode(l_u8ControlCode);
	l_u8Status	= 0;
	ClearKeyValue();

	while(1)
	{
		//������
		GUI_ClearSCR();
		l_u8Tmp2	= (l_u8Status >> 3)>0 ?	8 : 0;
		for (l_u8Tmp1 = l_u8Tmp2; l_u8Tmp1 <= l_u8Status; l_u8Tmp1 ++)
		{
			l_u8Tmp3	= (l_u8Tmp1 - l_u8Tmp2)<<4;
			LCDPRINTFC(0		, l_u8Tmp3	, "%d:", l_u8Tmp1);
			LCDPRINTFC(32		, l_u8Tmp3	, "0- %d"	, SETUPALIAS.au16StaticModify[0][l_u8Tmp1]);
			LCDPRINTFC(112		, l_u8Tmp3	, "1- %d"	, SETUPALIAS.au16StaticModify[1][l_u8Tmp1]);
		}
	
		while (1)		//�жϰ�����ˢ������
		{
			//�жϰ���
			switch(KeyValue)
			{
				case KB_ESC:
					ToBreak(l_u8ControlCode);
					break;
				
				case 0xf1:		//�༭
					sprintf(m_acTmp, "��̬����%dkm-0", l_u8Status);
					UISetValueParamU16(m_acTmp, &(SETUPALIAS.au16StaticModify[0][l_u8Status]), 9500, 10500);
					sprintf(m_acTmp, "��̬����%dkm-1", l_u8Status);
					UISetValueParamU16(m_acTmp, &(SETUPALIAS.au16StaticModify[1][l_u8Status]), 9500, 10500);

					ToReDraw(l_u8ControlCode);
					break;

				case 0xff:
					NotReDraw(l_u8ControlCode);
					break;

				default:
					l_u8Status	++;
					if (l_u8Status == 8)
						GUI_ClearSCR();
					else if (l_u8Status == 16)
					{	
						ToBreak(l_u8ControlCode);
						break;
					}

					//�κΰ�������ʾһ���µ�����
					l_u8Tmp2	= (l_u8Status&0x07) << 4;
					LCDPRINTFC(0		, l_u8Tmp2	, "%d:", l_u8Status);
					LCDPRINTFC(32		, l_u8Tmp2	, "0- %d"	, SETUPALIAS.au16StaticModify[0][l_u8Status]);
					LCDPRINTFC(112		, l_u8Tmp2	, "1- %d"	, SETUPALIAS.au16StaticModify[1][l_u8Status]);
					NotReDraw(l_u8ControlCode);
					break;
			}	//switch(KeyValue)
			ClearKeyValue();

			//�Ƿ��ػ�
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;
			
			WAITSCREENREFRESH();		//�ȴ�ˢ���ź�
		}

		//�Ƿ��˳�
		if (IfBreak(l_u8ControlCode))
			break;
	}
}

/*********************************************************************************************************
** Function name:		UIBDFullRange
** Descriptions:		����������� 
** input parameters:	None 
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110509	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void UIBDFullRange(void)
{
	uint8	l_u8ControlCode;//����ָ��

	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();

	while(1)
	{
		//������
		GUI_ClearSCR();
		LCDPRINTC(0		, 0		, "6. �����趨           Esc-����");
		
		LCDPRINTC(0		, 19	, "��ѡ���������:");

		LCDPRINTC(64	, 40	, "1--10000 kg");
		LCDPRINTC(64	, 56	, "2--15000 kg");
		LCDPRINTC(64	, 72	, "3--20000 kg");
		LCDPRINTC(64	, 88	, "4--30000 kg");
		LCDPRINTC(64	, 104	, "5--35000 kg");
		
		GUI_Line(0		, 18	, 239	, 18	, 1);
		GUI_Line(0		, 124	, 239	, 124	, 1);
		GUI_Line(0		, 127	, 239	, 127	, 1);

		while (1)		//�жϰ�����ˢ������
		{
			//�жϰ���
			switch(KeyValue)
			{
				case KB_ESC:
				case KB_ENTER:
					ToBreak(l_u8ControlCode);
					break;
					
				case KB_1:
					SETUPALIAS.u32Full	= 10000;
					break;

				case KB_2:
					SETUPALIAS.u32Full	= 15000;
					break;

				case KB_3:
					SETUPALIAS.u32Full	= 20000;
					break;

				case KB_4:
					SETUPALIAS.u32Full	= 30000;
					break;

				case KB_5:
					SETUPALIAS.u32Full	= 35000;
					break;
	
				default:
					NotReDraw(l_u8ControlCode);
					break;
			}	//switch(KeyValue)
			ClearKeyValue();

			//�Ƿ��ػ�
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;

			//ˢ����Ļ
			if (SETUPALIAS.u32Full == 0)
				SETUPALIAS.u32Full	= 35000;
			
			LCDPRINTFC(128	, 19	, "%ukg      "	, SETUPALIAS.u32Full);

			WAITSCREENREFRESH();		//�ȴ�ˢ���ź�
		}

		//�Ƿ��˳�
		if (IfBreak(l_u8ControlCode))
			break;
	}
}

/*********************************************************************************************************
** Function name:		UIBDChooseMotion
** Descriptions:		��̬����ѡ����棬����or����
** input parameters:	None 
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110509	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void UIBDChooseMotion(void)
{
	uint8	l_u8ControlCode;//����ָ��
	
	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();

	while(1)
	{
		//������
		GUI_ClearSCR();
		LCDPRINTC(0		, 0		, " ��̬����");
		
		LCDPRINTC(176	, 0		, "Esc �˳�");
		
		LCDPRINTC(8		, 24	, "1. ������");
		LCDPRINTC(8		, 44	, "2. ������");
	
		GUI_Line(0		, 18	, 239	, 18	, 1);
		GUI_Line(0		, 124	, 239	, 124	, 1);
		GUI_Line(0		, 127	, 239	, 127	, 1);

		while (1)		//�жϰ�����ˢ������
		{
			//�жϰ���
			switch(KeyValue)
			{
				case KB_ESC:
					ToBreak(l_u8ControlCode);
					break;

				case KB_1:		//������
					ToReDraw(l_u8ControlCode);
					UIBDLineModify();
					break;	
				
				case KB_2:		//������
					ToReDraw(l_u8ControlCode);
					UIBDPointModify();
					break;	
				
				default:
					NotReDraw(l_u8ControlCode);
					break;
			}	//switch(KeyValue)
			ClearKeyValue();

			//�Ƿ��ػ�
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;

			WAITSCREENREFRESH();		//�ȴ�ˢ���ź�
		}

		//�Ƿ��˳�
		if (IfBreak(l_u8ControlCode))
			break;
	}
}



/*********************************************************************************************************
** Function name:		UIBDLineModify
** Descriptions:		������������
** input parameters:	None 
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110509	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void UIBDLineModify(void)
{
	uint8	l_u8ControlCode;//����ָ��
	uint8	l_u8Status;		//���ܼ�״̬
	uint8	l_u8Key;		//����
	uint8	l_u8Tmp1;
	
	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();

	while(1)
	{
		l_u8Status	= 0;

		//������
		GUI_ClearSCR();
		LCDPRINTC(0		, 0		, "�������ٶȶ�n: 1-8  (n-1)*4km");
		
		DrawLineModify(g_sspSetup.au16Speedmodify);
		
		while (1)		//�жϰ�����ˢ������
		{
			//�жϰ���
			l_u8Key	= KeyValue;
			switch(l_u8Key)
			{
				case KB_ESC:
					if (l_u8Status == 0)
						ToBreak(l_u8ControlCode);
					else
					{
						ToReDraw(l_u8ControlCode);
						l_u8Status	= 0;
					}
					break;

				case KB_1:		
				case KB_2:		
				case KB_3:		
				case KB_4:		
				case KB_5:		
				case KB_6:		
				case KB_7:		
				case KB_8:		
					if (l_u8Status == 0)		//��ʾ��Ӧ�ٶȶ�
					{
						l_u8Tmp1	= (l_u8Key-1)<<2;
					
//						memset(m_acTmp, 0, 30);
						sprintf(m_acTmp,"%dkm-%dkm ��ǰֵ: %d",l_u8Tmp1,l_u8Tmp1+3, SETUPALIAS.au16Speedmodify[l_u8Tmp1]);
						LCDPRINTC(0		, 16	, m_acTmp);
						
						LCDPRINTC(0,32,"Esc�˳�, F1���޸�");
						l_u8Status	= l_u8Key;	
					}
					NotReDraw(l_u8ControlCode);
					break;	
				
				case KB_F1:		//�޸�״̬����ʼ��������
					l_u8Tmp1	= (l_u8Status-1)<<2;
					if (l_u8Status != 0)
					{
//						memset(m_acTmp, 0, 30);
						sprintf(m_acTmp,"�������ٶ�%dkm",l_u8Tmp1);
						UISetValueParamU16(m_acTmp, &(SETUPALIAS.au16Speedmodify[l_u8Tmp1]), 8000, 12000);
					}
					if (l_u8Status>1)	//������ߵĵ�
					{
						SETUPALIAS.au16Speedmodify[l_u8Tmp1-2]	= (SETUPALIAS.au16Speedmodify[l_u8Tmp1] + SETUPALIAS.au16Speedmodify[l_u8Tmp1-4])>>1;
						SETUPALIAS.au16Speedmodify[l_u8Tmp1-1]	= (SETUPALIAS.au16Speedmodify[l_u8Tmp1] + SETUPALIAS.au16Speedmodify[l_u8Tmp1-2])>>1;
						SETUPALIAS.au16Speedmodify[l_u8Tmp1-3]	= (SETUPALIAS.au16Speedmodify[l_u8Tmp1-2] + SETUPALIAS.au16Speedmodify[l_u8Tmp1-4])>>1;
					}
					if (l_u8Status<8)	//�����ұߵĵ�
					{
						SETUPALIAS.au16Speedmodify[l_u8Tmp1+2]	= (SETUPALIAS.au16Speedmodify[l_u8Tmp1] + SETUPALIAS.au16Speedmodify[l_u8Tmp1+4])>>1;
						SETUPALIAS.au16Speedmodify[l_u8Tmp1+1]	= (SETUPALIAS.au16Speedmodify[l_u8Tmp1] + SETUPALIAS.au16Speedmodify[l_u8Tmp1+2])>>1;
						SETUPALIAS.au16Speedmodify[l_u8Tmp1+3]	= (SETUPALIAS.au16Speedmodify[l_u8Tmp1+2] + SETUPALIAS.au16Speedmodify[l_u8Tmp1+4])>>1;
					}
					ToReDraw(l_u8ControlCode);
					l_u8Status	= 0;
					break;	
				
				default:
					NotReDraw(l_u8ControlCode);
					break;
			}	//switch(KeyValue)
			ClearKeyValue();

			//�Ƿ��ػ�
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;

			WAITSCREENREFRESH();		//�ȴ�ˢ���ź�
		}

		//�Ƿ��˳�
		if (IfBreak(l_u8ControlCode))
			break;
	}
}

/*********************************************************************************************************
** Function name:		UIBDPointModify
** Descriptions:		����������
** input parameters:	None 
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110509	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void UIBDPointModify(void)
{
	uint8	l_u8ControlCode;//����ָ��
	uint8	l_u8Key;		//����
	uint32	l_u32Value;
	
	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();
	l_u32Value	= 0;
	while(1)
	{	
		//������
		GUI_ClearSCR();
		LCDPRINTC(0		, 0		, "�������ٶ� 1-31:");
		
		while (1)		//�жϰ�����ˢ������
		{
			//�жϰ���
			l_u8Key	= KeyValue;
			switch(l_u8Key)
			{
				case KB_ESC:
					ToBreak(l_u8ControlCode);
					break;

				case KB_1:		
				case KB_2:		
				case KB_3:		
				case KB_4:		
				case KB_5:		
				case KB_6:		
				case KB_7:		
				case KB_8:		
				case KB_9:		
				case KB_0:
					l_u32Value	*= 10;
					l_u32Value	+= l_u8Key;
					NotReDraw(l_u8ControlCode);
					break;	
				
				case KB_BKSP:		//��ɾһ������
					NotReDraw(l_u8ControlCode);
					l_u32Value	/= 10;
					break;	
				
				case KB_ENTER:		//ȷ������
					if ((l_u32Value >0 && l_u32Value < 32))
					{
						sprintf(m_acTmp,"������ %d km:",l_u32Value);
						UISetValueParamU16(m_acTmp, &(SETUPALIAS.au16Speedmodify[l_u32Value]), 8000, 12000);
					}
					l_u32Value	= 0;
					ToReDraw(l_u8ControlCode);
					break;	
				
				default:
					NotReDraw(l_u8ControlCode);
					break;
			}	//switch(KeyValue)
			ClearKeyValue();

			//�Ƿ��ػ�
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;
			
			//ˢ����Ļ
			if (l_u32Value == 0)
				LCDPRINTC(144	, 0		, "         ");
			else
				LCDPRINTFC(144	, 0		, "%u    "	, l_u32Value);
			
			WAITSCREENREFRESH();		//�ȴ�ˢ���ź�
		}

		//�Ƿ��˳�
		if (IfBreak(l_u8ControlCode))
			break;
	}
}


/*********************************************************************************************************
** Function name:		UIBDChooseVehPos
** Descriptions:		�����ٶ�����λ��ѡ��
** input parameters:	None 
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110510	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void UIBDChooseVehPos(void)
{
	uint8	l_u8ControlCode;//����ָ��
	
	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();

	while(1)
	{
		//������
		GUI_ClearSCR();
		LCDPRINTC(0		, 0		, " ��������");
		
		LCDPRINTC(176	, 0		, "Esc �˳�");
		LCDPRINTC(8		, 24	, "1. ������������");
#if	IF3WB		
		LCDPRINTC(8		, 40	, "2. AB�峵���ٶȵ�����");
		LCDPRINTC(8		, 56	, "3. BC�峵���ٶȵ�����");	 
		LCDPRINTC(8		, 72	, "4. ����ѹ������");
		LCDPRINTC(8		, 88	, "5. ѹ�쳵���ٶȵ�����");
#else
		LCDPRINTC(8		, 40	, "2. �����ٶȵ�����");
#endif
		
		GUI_Line(0		, 18	, 239	, 18	, 1);
		GUI_Line(0		, 124	, 239	, 124	, 1);
		GUI_Line(0		, 127	, 239	, 127	, 1);

		while (1)		//�жϰ�����ˢ������
		{
			//�жϰ���
			switch(KeyValue)
			{
				case KB_ESC:
					ToBreak(l_u8ControlCode);
					break;
				
				case KB_1:		//������������
					ToReDraw(l_u8ControlCode);
					UIBDChooseVeh(POS_VEH);
					break;	
				
				case KB_2:		//AB�峵���ٶ�����
					ToReDraw(l_u8ControlCode);
#if	IF3WB
					UIBDChooseVeh(POS_ABS);
#else
					UIBDChooseVeh(POS_2WB);
#endif
					break;
						
#if	IF3WB			
				case KB_3:		//BC�峵���ٶ�����
					ToReDraw(l_u8ControlCode);
					UIBDChooseVeh(POS_BCS);
					break;	
				
				case KB_4:		//ѹ�쳵���ٶ�����
					ToReDraw(l_u8ControlCode);
					UIBDChooseVeh(POS_ABCS);
					break;	
				
				case KB_5:		//����ѹ������
					ToReDraw(l_u8ControlCode);
					UIBDChooseVeh(POS_GAP);
					break;	
#endif				
				default:
					NotReDraw(l_u8ControlCode);
					break;
			}	//switch(KeyValue)
			ClearKeyValue();

			//�Ƿ��ػ�
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;

			WAITSCREENREFRESH();		//�ȴ�ˢ���ź�
		}

		//�Ƿ��˳�
		if (IfBreak(l_u8ControlCode))
			break;
	}
}

/*********************************************************************************************************
** Function name:		UIBDChooseVeh
** Descriptions:		ѡ���ͽ��棬����ѹ��λ�ò�ͬ���鿴�����ݲ�ͬ
** input parameters:	None 
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110510	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void UIBDChooseVeh(uint8 p_u8Pos)
{
	uint8	l_u8ControlCode;//����ָ��
	uint8	l_u8Key;
	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();

	while(1)
	{
		//������
		GUI_ClearSCR();
		switch(p_u8Pos)
		{
#if	IF3WB
		case POS_ABS:	//AB
			LCDPRINTC(0		, 0		, "AB����ʻ�����ٶȵ�����");
			break;

		case POS_BCS:	//BC
			LCDPRINTC(0		, 0		, "BC����ʻ�����ٶȵ�����");
			break;

		case POS_ABCS:	//ѹ��
			LCDPRINTC(0		, 0		, "ѹ����ʻ�����ٶȵ�����");
			break;
		
		case POS_GAP:	//ѹ������
			LCDPRINTC(0		, 0		, "����ѹ����������");
			break;
#endif
		case POS_VEH:	//��������
			LCDPRINTC(0		, 0		, "������������");
			break;

		case POS_2WB:	//�����
			LCDPRINTC(0		, 0		, "�����ٶ�����");
			break;

		default:
			ToBreak(l_u8ControlCode);
			break;
		}		
		LCDPRINTC(8		, 24	, "1.11/12");
		LCDPRINTC(8		, 44	, "2.13/14/15");
		LCDPRINTC(8		, 64	, "3.112/122");
		LCDPRINTC(8		, 84	, "4.113/115");
		LCDPRINTC(8		, 104	, "5.123/124/125");
		LCDPRINTC(124	, 24	, "6.126/127");
		LCDPRINTC(124	, 44	, "7.155/135/153");
		LCDPRINTC(124	, 64	, "8.157/156/1127");
		//LCDPRINTC(124	, 84	, "9.��������");
		LCDPRINTC(124	, 104	, "Esc. �˳����� ");
	
		GUI_Line(0		, 18	, 239	, 18	, 1);
		GUI_Line(0		, 124	, 239	, 124	, 1);
		GUI_Line(0		, 127	, 239	, 127	, 1);

		while (1)		//�жϰ�����ˢ������
		{
			//�жϰ���
			l_u8Key	= KeyValue;
			switch(l_u8Key)
			{
				case KB_ESC:
					ToBreak(l_u8ControlCode);
					break;

				case KB_1:		//"1. 11/12"
				case KB_2:		//"2. 13/14/15"
				case KB_3:		//"3. 112/122"
				case KB_4:		//"4. 113/115"
				case KB_5:		//"5. 123/124/125"
				case KB_6:		//"6. 126/127"
				case KB_7:		//"7. 155/135/153"
				case KB_8:		//"8. 157/156/1127"
					ToReDraw(l_u8ControlCode);
					switch(p_u8Pos)
					{

#if	IF3WB
					case POS_ABS:	//AB
					case POS_BCS:	//BC
					case POS_ABCS:	//ѹ��
#endif
					case POS_2WB:	//�����
						UIBDVSModifyParam(p_u8Pos, l_u8Key);
						break;
			
					case POS_VEH:	//��������
						sprintf(m_acTmp, "��������-����%d",l_u8Key);  
						UISetValueParamU16(&m_acTmp[0], &(SETUPALIAS.au16VehTotalModify[l_u8Key-1]), 8000, 12000);
						break;
#if	IF3WB			
					case POS_GAP:	//ѹ������
						sprintf(m_acTmp, "ѹ������-����%d",l_u8Key);  
						UISetValueParamU16(m_acTmp, &(SETUPALIAS.au16GapModify[l_u8Key-1]), 8000, 12000);
						break;
#endif			
					default:
						ToBreak(l_u8ControlCode);
						break;
					}
					break;	
				
				default:
					NotReDraw(l_u8ControlCode);
					break;
			}	//switch(KeyValue)
			ClearKeyValue();
					
			//�Ƿ��ػ�
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;

			WAITSCREENREFRESH();		//�ȴ�ˢ���ź�
		}

		//�Ƿ��˳�
		if (IfBreak(l_u8ControlCode))
			break;
	}
}

/*********************************************************************************************************
** Function name:		UIBDVSModifyParam
** Descriptions:		����ֵ���棬����ѹ��λ�úͳ���
** input parameters:	p_u8Pos		λ��
**						p_u8Veh		���� 
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110510	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void UIBDVSModifyParam(uint8 p_u8Pos, uint8 p_u8Veh)
{
	uint8	l_u8ControlCode;		//����ָ��
	uint32	l_u32Value	= 0;		//��������
	uint8	l_u8Key;
	uint8	l_u8Status;
	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();
	
	while(1)
	{
		//������
		GUI_ClearSCR();
		switch(p_u8Pos)
		{
#if	IF3WB
		case POS_ABS:	//AB
			LCDPRINTFC(0		, 0		, "AB�峵���ٶ�����-����%d"	, p_u8Veh);
			break;

		case POS_BCS:	//BC
			LCDPRINTFC(0		, 0		, "BC�峵���ٶ�����-����%d"	, p_u8Veh);
			break;

		case POS_ABCS:	//ѹ��
			LCDPRINTFC(0		, 0		, "ѹ�쳵���ٶ�����-����%d"	, p_u8Veh);
			break;
#endif
		case POS_2WB:	//�����
			LCDPRINTFC(0		, 0		, "�����ٶ�����-����%d"		, p_u8Veh);
			break;

		default:
			ToBreak(l_u8ControlCode);
			break;
		}		
#if	IF3WB
		LCDPRINTC(0		, 32	, "�������ٶ�(0-19): ");//20�������ڣ��༭��Ӧ����
#else
		LCDPRINTC(0		, 32	, "�������ٶ�(0-31): ");//32�������ڣ��༭��Ӧ����
#endif
		
		GUI_Line(0		, 18	, 239	, 18	, 1);
		GUI_Line(0		, 124	, 239	, 124	, 1);
		GUI_Line(0		, 127	, 239	, 127	, 1);
		l_u8Status	= 0xff;
		while (1)		//�жϰ�����ˢ������
		{
			//�жϰ���
			//�жϰ���
			l_u8Key	= KeyValue;
			switch(l_u8Key)
			{
				case KB_ESC:
					ToBreak(l_u8ControlCode);
					break;

				case KB_1:		
				case KB_2:		
				case KB_3:		
				case KB_4:		
				case KB_5:		
				case KB_6:		
				case KB_7:		
				case KB_8:		
				case KB_9:		
				case KB_0:
					l_u32Value	*= 10;
					l_u32Value	+= l_u8Key;
					NotReDraw(l_u8ControlCode);
					l_u8Status	= 0;
					break;	
				
				case KB_BKSP:		//��ɾһ������
					NotReDraw(l_u8ControlCode);
					l_u32Value	/= 10;
					if (l_u32Value == 0)
						l_u8Status	= 0xff;
					break;	
				
				case KB_ENTER:		//ȷ������
					if (l_u8Status != 0xff)
					{
#if	IF3WB
						if (l_u32Value < 20)	//20�������ڣ��༭��Ӧ����
#else
						if (l_u32Value < 32)	//32�������ڣ��༭��Ӧ����
#endif
						{
							switch(p_u8Pos)
							{
#if	IF3WB		
							case POS_ABS:	//AB
//								memset(m_acTmp, 0, 30);
								sprintf(m_acTmp, "AB-����%d-�ٶ�%dkm",p_u8Veh,l_u32Value);  
								UISetValueParamU16(m_acTmp, &(SETUPALIAS.au16VehSpeedModify[p_u8Veh-1][l_u32Value]), (VSMSTANDARD*8)/10, (VSMSTANDARD*12)/10);
								break;
					
							case POS_BCS:	//BC
//								memset(m_acTmp, 0, 30);
								sprintf(m_acTmp, "BC-����%d-�ٶ�%dkm",p_u8Veh,l_u32Value);  
								UISetValueParamU16(m_acTmp, &(SETUPALIAS.au16VehSpeedModify[p_u8Veh-1][l_u32Value]), (VSMSTANDARD*8)/10, (VSMSTANDARD*12)/10);
								break;
					
							case POS_ABCS:	//ѹ��
//								memset(m_acTmp, 0, 30);
								sprintf(m_acTmp, "ѹ��-����%d-�ٶ�%dkm",p_u8Veh,l_u32Value);  
								UISetValueParamU16(m_acTmp, &(SETUPALIAS.au16VehSpeedModify[p_u8Veh-1][l_u32Value]), (VSMSTANDARD*8)/10, (VSMSTANDARD*12)/10);
								break;
#else				
							case POS_2WB:	//2���
//								memset(m_acTmp, 0, 30);
								sprintf(m_acTmp, "����%d-�ٶ�%dkm",p_u8Veh,l_u32Value);  
								UISetValueParamU16(m_acTmp, &(SETUPALIAS.au16VehSpeedModify[p_u8Veh-1][l_u32Value]), (VSMSTANDARD*8)/10, (VSMSTANDARD*12)/10);
								break;
#endif				
							default:
								ToBreak(l_u8ControlCode);
								break;
							}
						}
						l_u32Value	= 0;
						ToReDraw(l_u8ControlCode);
					}
					else
						NotReDraw(l_u8ControlCode);
					break;	
				
				default:
					NotReDraw(l_u8ControlCode);
					break;
			}	//switch(KeyValue)
			ClearKeyValue();
					
			//�Ƿ��ػ�
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;

			//ˢ����Ļ
			if (l_u8Status == 0xff)
				LCDPRINTC(144	, 32	, "         ");
			else
				LCDPRINTFC(144	, 32	, "%u       "	, l_u32Value);
			
			WAITSCREENREFRESH();		//�ȴ�ˢ���ź�
		}

		//�Ƿ��˳�
		if (IfBreak(l_u8ControlCode))
			break;
	}
}
