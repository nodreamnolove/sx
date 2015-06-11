/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			UI.c
** Last modified Date:  20110506
** Last Version:		1.0
** Descriptions:		所有界面函数
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
** Descriptions:		普通双弯板
**
*********************************************************************************************************/
#define	__UI_C
#include "UI.h"

static	uint8	m_u8Err;
char	m_acTmp[30];		//显示屏一行

//全局变量别名
#define		SETUPALIAS				g_sspSetup			//设置参数结构
#define		SNALIAS					g_u32SN				//SN值
#define		TMPTIMEALIAS			g_sstTempTime		//时间结构临时变量
#if	YBVERSION >= 30		//3.0仪表功能
#define		IPINFOALIAS				g_sniLocal			//IP信息别名
#endif
#define		CURTIMEALIAS			g_sstCurTime		//当前时间
#define		THRESHOLDALIAS			g_sudtThreshold		//阈值
#define		LZSIGNAL				g_u32LunZhouSignal	//轮轴信号	
#define		LZSIGNALLast			g_u32LunZhouSignalLast	//上一个轮轴信号
#define		ADAvg					g_an32AvgAD				//平均AD值
#define		PASSOVERINFO			g_sviVehicleInfo		//完整过车信息
#define		ERRALIAS				g_u8DeviceERR			//故障代码

#define		YBVERALIAS				YBVERSION			//仪表版本号
#define		VERSIONALIAS			PROGRAMVERSION		//版本号

#define		LCDPosX					X					//显示屏X坐标
#define		LCDPosY					Y					//显示屏Y坐标

#define		LCDPRINTC				PrintCharXY			//显示字符串
#define		LCDPRINTFC				PrintFormatDataXY	//显示一定格式的字符串

#define		BGON					BackGroundON		//背景灯亮
#define		BGOFF					BackGroundOFF		//背景灯灭
#define		BGConvert				BackGroundReverse	//背景灯反转

#define		GUANGSHANStatus			GSTriggerPIN		//光栅状态
#define		LOOPStatus				LPTriggerPIN		//线圈状态
#define		GUANGSHANErr			GSErrPIN			//光栅故障
#define		LOOPErr					LPErrPIN			//线圈故障

////////////////////////////////////
//轮询时间(单位10ms)
#define		LOOPTIME				1

#define		WTD_CT					0x01				//称台
#define		WTD_2WB					0x02				//2弯板
#define		WTD_3WB					0x06				//3弯板

#define		WTDEVICE				WTD_2WB
		
#define		IF3WB					WTDEVICE & 0x04		

//清除键值
#define		KeyValue				g_u8KeyValueMapped
#define		ClearKeyValue()			KeyValue	= 0xff

#define		ClearLCDLine(X)			LCDPRINTC(0		, X	, "                              ")

//控制指令:0x01:重画；0x02：退出					
#define		ResetControlCode(ControlCode)	ControlCode = 0x00

#define		IfReDraw(ControlCode)	ControlCode & 0x01
#define		ToReDraw(ControlCode)	ControlCode |= 0x01
#define		NotReDraw(ControlCode)	ControlCode &= ~0x01

#define		IfBreak(ControlCode)	ControlCode & 0x02
#define		ToBreak(ControlCode)	ControlCode |= 0x02
#define		NotBreak(ControlCode)	ControlCode &= ~0x02

//等待屏幕刷新信号
#define		WAITSCREENREFRESH()		OSSemPend(g_psemScreenRefresh,0, &m_u8Err)

//压缝位置
#define		POS_ABS		1			//AB速度
#define		POS_BCS		2			//BC速度
#define		POS_ABCS	3			//压缝速度
#define		POS_VEH		4			//车型整体
#define		POS_GAP		5			//压缝整体
#define		POS_2WB		6			//两弯板情况

//动静态
#define		UI_STATIC	0
#define		UI_MOTION	1

//设置使能参数，uint8类型
static	void UISetEnableParam(char * p_pcName, uint8 * p_pu8Param);

//uint8,uint16,uint32参数设置
static	void UISetValueParamU8(char * p_pcName, uint8 * p_pu8Param, uint32 p_u32Min, uint32 p_u32Max);
static	void UISetValueParamU16(char * p_pcName, uint16 * p_pu16Param, uint32 p_u32Min, uint32 p_u32Max);
#if	YBVERSION >= 30		//3.0仪表功能
static	void UISetValueParamU32(char * p_pcName, uint32 * p_pu32Param, uint32 p_u32Min, uint32 p_u32Max);
#endif	//#if	YBVERSION >= 30		//3.0仪表功能

/*********************************************************************************************************
** Function name:		UIStartUp
** Descriptions:		启动画面，不需要循环显示，间隔一段时间后关闭
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
	GUI_ClearSCR();								//清屏
	DrawPic();									//显示图片

	LCDPRINTC(130	, 72	, VERSIONALIAS	);	//显示版本信息

//	BeepON();			//峰鸣器开
//	
//	OSTimeDly(10);	
//	
//	BeepOFF();			//峰鸣器关	      
	
	LEDON();
}

/*********************************************************************************************************
** Function name:		UIGeneral
** Descriptions:		标准过程界面，一直显示
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
	uint8	l_u8ControlCode;		//控制指令
	int32	l_n32TempWeight;		//临时重量
	uint32	l_u32Temp1;				//临时变量1
	int32	l_n32TotalWeight;		//总重
	uint32	l_u32Temp2,l_u32Temp3;	//临时变量2,3
	uint8	l_u8k;					//循环累加变量
	uint8	l_u8FindFirstData;
	uint8	l_u8EraseStatus;		//擦除状态 0：数据区为空； 1：数据区全满； 2：显示轴信息; 3:只有一行
	char 	l_cTaiXing[20];
	uint8	l_u8TaiXingIndex;
	int32	l_n32TmpStaticWeight[CHANNELNUM];
	int32	l_n32StaticXiuIndex;
	uint32	l_u32TimeCnt;
	
	uint8	l_u8ReShowVeh;			//重新显示状态: 0x10不重新显示(可重新显示)；0x11重新显示；0x00取消重新显示

	void *	l_pTmp;
	AxleShow	l_asTmp;
	
	AddStartupCnt();	//记录启动次数
	
	l_u8ReShowVeh	= 0x00;
	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();
	memset(m_acTmp, 0, 30);

	while(1)
	{
		l_u32TimeCnt	= 0;
		l_u8EraseStatus	= 0;

		//画界面
		GUI_ClearSCR();
		LCDPRINTC(0		, 0		, "轴序  "	);
		LCDPRINTC(56	, 0		, "轴组重"	);
		LCDPRINTC(128	, 0		, "轴重  "	); 
		LCDPRINTC(192	, 0		, "车速  "	);

		LCDPRINTC(80	, 115	, "C"	);
#if	YBVERSION < 30		//2.2仪表功能
		LCDPRINTC(64	, 115	, "00"	);
#endif	//#if	YBVERSION < 30		//2.2仪表功能
		LCDPRINTC(96	, 115	, "E"	);
		LCDPRINTC(128	, 115	, "BUF"	);	
		LCDPRINTC(192	, 115	, ":"	);
		LCDPRINTC(216	, 115	, ":"	);

		GUI_Line(0		, 16	, 239	, 16	, 1	);
		GUI_Line(0		, 114	, 239	, 114	, 1	);

		while (1)		//判断按键并刷新数据
		{
			//判断按键
			switch(KeyValue)
			{
				case KB_ESC:
					ToReDraw(l_u8ControlCode); 
					l_u8ReShowVeh	= 0x00;		//禁止重新显示
					UISN();
					break;

				case KB_F1:		//清零
					NotReDraw(l_u8ControlCode);
					CLEARZERO();
					break;	
				
				case KB_F2:		//动静态切换
					NotReDraw(l_u8ControlCode);
					g_u8StaticMotionswitch	= (g_u8StaticMotionswitch + 1) & 0x01;
					break;	
				
				case KB_F3:		//程序信息
					ToReDraw(l_u8ControlCode);	  
					l_u8ReShowVeh	|= 0x01;	//允许重新显示
					UIF3Code();
					break;	
				
				case KB_F4:		//过车代码
					ToReDraw(l_u8ControlCode);	  
					l_u8ReShowVeh	|= 0x01;	//允许重新显示
					UIF4Code();
					break;	
				
				case KB_F5:		//轮轴状态
					ToReDraw(l_u8ControlCode);	  
					l_u8ReShowVeh	|= 0x01;	//允许重新显示
					UIF5Code();
					break;	
				
				case KB_F6:		//显示屏背景灯开关
					NotReDraw(l_u8ControlCode);
					BGConvert();	
					break;	
				
				default:
					NotReDraw(l_u8ControlCode);
					break;
			}	//switch(KeyValue)
			ClearKeyValue();
					
			//是否重画
			if (IfReDraw(l_u8ControlCode))
				break;

			//刷新显示
			if (OSSemAccept(g_psemSystemTime)>0)		//有信号量，申请并显示数据
			{
				//E/BUF
#if	YBVERSION >= 30		//3.0仪表功能				
				LCDPRINTFC(64	, 115	, "%02D"	, g_u32Temprature);
#endif
				LCDPRINTFC(104	, 115	, "%02X "	, ERRALIAS);
				LCDPRINTFC(152	, 115	, "%02d "	, g_u8CurVehNum);
				
				//时间
				LCDPRINTFC(176	, 115	, "%0.2d"	, CURTIMEALIAS.u8Hour);
				LCDPRINTFC(200	, 115	, "%0.2d"	, CURTIMEALIAS.u8Minute);		
				LCDPRINTFC(224	, 115	, "%0.2d"	, CURTIMEALIAS.u8Second);
				
				//光栅和线圈状态
				if(GUANGSHANStatus==0)				
					LCDPRINTC(232	, 0		, "*");
				else 
					LCDPRINTC(232	, 0		, " ");
				if(LOOPStatus==0)
					LCDPRINTC(224	, 0		, "@");
				else 
					LCDPRINTC(224	, 0		, " ");

				//显示瞬时值,对传感器静态修正
				l_n32TempWeight	= 0;
				for(l_u8k = 0;l_u8k < CHANNELNUM; l_u8k++)
				{
					l_n32TmpStaticWeight[l_u8k] = (ADAvg[l_u8k] - g_an32MotionZero[l_u8k]) * SETUPALIAS.an32AxGain[l_u8k]/10000;
					l_n32StaticXiuIndex = StaticXiuZhengIndex(l_n32TmpStaticWeight[l_u8k]);
					l_n32TmpStaticWeight[l_u8k]	= l_n32TmpStaticWeight[l_u8k] * (int32)SETUPALIAS.au16StaticModify[l_u8k][l_n32StaticXiuIndex]/10000;
				
					l_n32TempWeight	+= l_n32TmpStaticWeight[l_u8k]; 
				}				
			
				if(g_u8StaticMotionswitch == 1)		//动态重量
				{
					if (l_u32TimeCnt > 100)
					{
						if(l_n32TempWeight < (int32)SETUPALIAS.u8MotionScale)	   			//小于动态分度值
					    	CLEARZERO();
						l_u32TimeCnt	= 0;
					}
					l_n32TempWeight	= ScaleUp(l_n32TempWeight, SETUPALIAS.u8MotionScale);
					LCDPRINTFC(0		, 115	, "D%-6d"	, l_n32TempWeight);
				}
				else		//静态重量
				{
					if (SETUPALIAS.u8Genzong > 0)
					{
						if (l_u32TimeCnt > 100)
						{
							l_u32TimeCnt	= 0;
							if(l_n32TempWeight < (int32)SETUPALIAS.u8StaticScale)	   			//小于静态分度值
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
			
			l_pTmp	= OSQAccept(g_pqueAxleShow);		//过车过程，轴信息
			if (l_pTmp!= (void *)0)		//消息队列中有消息，申请并显示数据
			{
				switch (l_u8EraseStatus)
				{
					case 1:	//全满，需要全部擦除
						ClearLCDLine(18);
						ClearLCDLine(34);
						ClearLCDLine(50);
						ClearLCDLine(66);
						ClearLCDLine(81);
						ClearLCDLine(97);
						break;

					case 2:	//过轴信息
						ClearLCDLine(18);
						ClearLCDLine(34);
						ClearLCDLine(50);
						break;

					case 3:	//单行信息
						ClearLCDLine(18);
						break;

					default:
						break;
				}
				l_u8EraseStatus	= 0;	  
				l_u8ReShowVeh	= 0x00;	//接收新轴，停止重新显示过车数据

				memcpy(&l_asTmp, (AxleShow *)l_pTmp,sizeof(AxleShow));

				//显示
				switch((l_asTmp.u8Direct) & 0x7f)
				{
					case 0:
							LCDPRINTFC(0		, 18	, "%d.  ", l_asTmp.u8AxleID);	//1.
							LCDPRINTC(24		, 18	, "+");	//+	表示正轴
#if ISDEBUG
							if (l_asTmp.n32AxleWeight != 0)
							{
								LCDPRINTFC(24		, 34	, "WEIGHT: %d.  "	, l_asTmp.n32AxleWeight);		//重量
								LCDPRINTFC(24		, 50	, "MAX   : %d.  "	, l_asTmp.n32AxleMaxWeight);	//最大值
								LCDPRINTFC(144		, 50	, "AT : %d.  "		, l_asTmp.u16ArchiveTimes);		//最大值
							}
#endif
							l_u8EraseStatus	= 2;
							break;
					case 1:
							LCDPRINTFC(0		, 18	, "%d.  ", l_asTmp.u8AxleID);	//1.
							LCDPRINTC(24		, 18	, "-");	//- 表示倒轴
							l_u8EraseStatus	= 3;
							break;
					case 2:
							LCDPRINTC(24		, 18	, ">");	//> 正向上、然后倒回
							l_u8EraseStatus	= 3;
							break;
					case 3:
							LCDPRINTC(24		, 18	, "<");	//< 反向倒、然后前进退出
							l_u8EraseStatus	= 3;
							break;
					default:
							break;
				}

				//薄称专用
				if (l_asTmp.u8Direct >> 7)		//2台板
					LCDPRINTC(40		, 18	, "B");	//B
				else							//1台板
					LCDPRINTC(40		, 18	, "A");	//A
			}
			
//			if (OSSemAccept(g_psemVehicleInfo)>0)		//显示完整过车数据
			if ((OSSemAccept(g_psemVehicleInfo)>0)||(l_u8ReShowVeh==0x11))		//显示完整过车数据
			{						   
				l_u8ReShowVeh	= 0x10;

				switch (l_u8EraseStatus)
				{
					case 1:	//全满，需要全部擦除
						ClearLCDLine(18);
						ClearLCDLine(34);
						ClearLCDLine(50);
						ClearLCDLine(66);
						ClearLCDLine(81);
						ClearLCDLine(97);
						break;

					case 2:	//过轴信息
						ClearLCDLine(18);
						ClearLCDLine(34);
						ClearLCDLine(50);
						break;

					case 3:	//单行信息
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
					
					if (l_u32Temp1 == 5)	//最多只显示6行
						break;
				}

				for (l_u32Temp1 = 0; l_u32Temp1 < PASSOVERINFO.u8AxleGrpTotalNum; l_u32Temp1 ++)
				{
					l_n32TotalWeight	+= PASSOVERINFO.an32AxleGrpWeight[l_u32Temp1]*10;
					
					//修改，将6轴组退出改为6轴组以上只计数，不显示
					if (l_u32Temp1 < 6)	//最多只显示6行
						LCDPRINTFC(56		, l_u32Temp1*13+18	, "%dkg"	,PASSOVERINFO.an32AxleGrpWeight[l_u32Temp1]*10);
					
				}
				
				//总重
				LCDPRINTC(0		, 97	, "总重: "	);
				LCDPRINTFC(40	, 97	, "%dkg "	,l_n32TotalWeight);
				
				//轴型	
				LCDPRINTC(112	, 97	, "轴型:");

				if(PASSOVERINFO.u8AxleGrpTotalNum < 7) 
				{
					l_u8TaiXingIndex = 0;
					for(l_u32Temp1=0;l_u32Temp1<PASSOVERINFO.u8AxleGrpTotalNum;l_u32Temp1++)
					{
						l_u32Temp2 = (PASSOVERINFO.au8AxleTAIType[l_u32Temp1] & 0xF0)>>4;	//取高位并且移四位;
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
				else	//多轴处理，只显示有几个轴，轴组
				{
					sprintf(m_acTmp, "%d轴,%d轴组", PASSOVERINFO.u8AxleTotalNum, PASSOVERINFO.u8AxleGrpTotalNum);
					LCDPRINTC(150	, 97	, (char *)m_acTmp);
				}
				l_u8EraseStatus	= 1;
				OSQAccept(g_pqueAxleShow);				
			}
		
			l_u32TimeCnt	++;

			WAITSCREENREFRESH();		//等待刷新信号量
		}
		//是否退出
		if (IfBreak(l_u8ControlCode))
			break;
	}
}

/*********************************************************************************************************
** Function name:		UISN
** Descriptions:		SN界面
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
	uint8	l_u8ControlCode;//控制指令
	uint32	l_u32Value;		//输入数字
	uint32	l_u32SN;		//SN值
	uint8	l_u8DigitCnt;	//数字个数
	uint8	l_u8Tmp1;		//临时变量
	uint8	l_u8Key;					  
	uint32	l_u32SNTimer;	//由Timer生成的SN明码

	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();
	
	BackGroundSave();
	BackGroundON();

	l_u32SN	= 0;
	g_u8EnableClearZero	= 0;	//不需要清零
	while(1)
	{
		l_u32SNTimer	= SNALIAS;
		l_u32Value	= 0;
		l_u8DigitCnt= 0;
		
		//画界面
		GUI_ClearSCR();
		LCDPRINTFC(16		, 32	, "SN:         %d", l_u32SNTimer);
		LCDPRINTC(16		, 64	, "请输入口令: ");
		
		while (1)		//判断按键并刷新数据
		{
			//判断按键
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
				
				case KB_BKSP:		//回删一个数字
					NotReDraw(l_u8ControlCode);
					if (l_u8DigitCnt>0)
						l_u8DigitCnt	--;
					l_u32Value	/= 10;
					break;	
				
				case KB_ENTER:		//系统初始化
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
#if	YBVERSION >= 30		//3.0仪表功能						
						case 444:
							UIViewIPInfo();
							break;
						
						case 8494:
							UIViewStartUpTime();
							break;
#endif	//#if	YBVERSION >= 30		//3.0仪表功能
						case 8968:
							UIViewThreshold();
							break;
																		 
#if	YBVERSION < 30		//2.2仪表功能
						case 5885:	GUI_ClearSCR();
							LCDPRINTC(16	, 32	, "进入ISP状态......");
							ISPinit();
							bootloader_entry();
							break;				
#endif	//#if	YBVERSION < 30		//2.2仪表功能

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
					
			//是否重画
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;

			//刷新屏幕
			LCDPRINTC(112 + l_u8DigitCnt<<3 	, 64	, "  ");

			for (l_u8Tmp1 = 0; l_u8Tmp1 < l_u8DigitCnt; l_u8Tmp1 ++)
				LCDPRINTC(112 + l_u8Tmp1<<3 	, 64	, "*");

			WAITSCREENREFRESH();		//等待刷新信号量
		}

		//是否退出
		if (IfBreak(l_u8ControlCode))
			break;
	}
	g_u8EnableClearZero	= 1;		//恢复自动清零	 
	BackGroundRecover();
}

/*********************************************************************************************************
** Function name:		UIBDRoot
** Descriptions:		标定根界面
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
	uint8	l_u8ControlCode;//控制指令
	
	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();

	while(1)
	{
		//画界面
		GUI_ClearSCR();
		LCDPRINTC(0		, 0		, " 系统应用选项");
		LCDPRINTC(176	, 0		, "Esc 退出");
		
		LCDPRINTC(8		, 24	, "1. 动态秤标定");
		LCDPRINTC(8		, 44	, "2. 静态秤标定");
		LCDPRINTC(8		, 64	, "3. 系统初始化");
		
		GUI_Line(0		, 18	, 239	, 18	, 1);
		GUI_Line(0		, 124	, 239	, 124	, 1);
		GUI_Line(0		, 127	, 239	, 127	, 1);
		
		while (1)		//判断按键并刷新数据
		{
			//判断按键
			switch(KeyValue)
			{
				case KB_ESC:
					ToBreak(l_u8ControlCode);
					break;

				case KB_1:		//动态标定
					ToReDraw(l_u8ControlCode);
					UIBDMain(UI_MOTION);
					break;	
				
				case KB_2:		//静态标定
					ToReDraw(l_u8ControlCode);
					UIBDMain(UI_STATIC);
					break;	
				
				case KB_3:		//系统初始化
					ToReDraw(l_u8ControlCode);
					UISystemInit();
					break;	
				
				default:
					NotReDraw(l_u8ControlCode);
					break;
			}	//switch(KeyValue)
			ClearKeyValue();
					
			//是否重画
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;

			WAITSCREENREFRESH();		//等待刷新信号
		}

		//是否退出
		if (IfBreak(l_u8ControlCode))
			break;
	}
}

/*********************************************************************************************************
** Function name:		UIBDMain
** Descriptions:		动态标定界面
** input parameters:	p_u8Motion 动静态标志 0，静态，1，动态 
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
	uint8	l_u8ControlCode;//控制指令
	
	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();

	while(1)
	{
		//画界面
		GUI_ClearSCR();
		if (p_u8Motion == UI_MOTION)		//动态
			LCDPRINTC(0		, 0		, " 动态标定");
		else 				//静态
			LCDPRINTC(0		, 0		, " 静态标定");

		LCDPRINTC(112	, 0		, "0 保存  Esc 退出" );
		
		LCDPRINTC(8		, 24	, "1. 弯板标定");
		LCDPRINTC(8		, 44	, "2. 秤台标定");
		LCDPRINTC(8		, 64	, "3. 分度设定");
		if (p_u8Motion == UI_MOTION)
		{
			LCDPRINTC(8		, 84	, "4. 静态修正");
			LCDPRINTC(8		, 104	, "5. 动态修正");
			LCDPRINTC(128	, 24	, "6. 量程设定");
			LCDPRINTC(128	, 44	, "7. 零点跟踪");
			LCDPRINTC(128	, 64	, "8. 坡度修正");
			LCDPRINTC(128	, 84	, "9. 车型修正");
			LCDPRINTC(128	, 104	, "F1 无轮轴使能");
		}
		GUI_Line(0		, 18	, 239	, 18	, 1);
		GUI_Line(0		, 124	, 239	, 124	, 1);
		GUI_Line(0		, 127	, 239	, 127	, 1);

		while (1)		//判断按键并刷新数据
		{
			//判断按键
			NotReDraw(l_u8ControlCode);
			switch(KeyValue)
			{
				case KB_ESC:
					ToBreak(l_u8ControlCode);
					break;

				case KB_1:		//弯板标定
					ToReDraw(l_u8ControlCode);
					UIBDWanBanChoose(p_u8Motion);
					break;	
				
				case KB_2:		//称台标定
					ToReDraw(l_u8ControlCode);
					UIBDChengtaiChoose(p_u8Motion);
					break;	
				
				case KB_3:		//分度设定
					ToReDraw(l_u8ControlCode);
					UIBDScale(p_u8Motion);
					break;	
				
				case KB_4:		//静态修正
					if (p_u8Motion == UI_MOTION)
					{
						ToReDraw(l_u8ControlCode);
						UIBDStaticModify();
					}
					break;	
				
				case KB_5:		//动态修正
					if (p_u8Motion == UI_MOTION)
					{
						ToReDraw(l_u8ControlCode);
						UIBDChooseMotion();
					}
					break;	
				
				case KB_6:		//量程设定
					if (p_u8Motion == UI_MOTION)
					{
						ToReDraw(l_u8ControlCode);
						UIBDFullRange();
					}
					break;	
				
				case KB_7:		//零点跟踪
					if (p_u8Motion == UI_MOTION)
					{
						ToReDraw(l_u8ControlCode);
						UIBDGenZong();
					}
					break;	
				
				case KB_8:		//坡度修正
					if (p_u8Motion == UI_MOTION)
					{
						ToReDraw(l_u8ControlCode);
						UIBDPoDu();
					}
					break;	
				
				case KB_9:		//车型修正
					if (p_u8Motion == UI_MOTION)
					{
						ToReDraw(l_u8ControlCode);
						UIBDChooseVehPos();
					}
					break;	
				
				case KB_0:		//保存设定
					ToReDraw(l_u8ControlCode);
					SaveParams();
					break;	
				
				case KB_F1:		//无轮轴程序
					if (p_u8Motion == UI_MOTION)
					{
						ToReDraw(l_u8ControlCode);
						UIBDLunZhou();
					}
					break;	

#if	SENDWAVEENABLE > 0		//使能发波形				
				case KB_F2:		//发波形使能
					if (p_u8Motion == UI_MOTION)
					{
						ToReDraw(l_u8ControlCode);
						UIBDSendWave();
					}
					break;	
#endif	//#if	SENDWAVEENABLE > 0		//使能发波形
				
				default:
					NotReDraw(l_u8ControlCode);
					break;
			}	//switch(KeyValue)
			ClearKeyValue();
					
			//是否重画
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;

			WAITSCREENREFRESH();		//等待刷新信号
		}

		//是否退出
		if (IfBreak(l_u8ControlCode))
			break;
	}
}

/*********************************************************************************************************
** Function name:		UIBDWanBanChoose
** Descriptions:		弯板动态标定通道选择界面
** input parameters:	p_u8Motion	动静态标志 0表示静态，1表示动态 
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
	uint8	l_u8ControlCode;//控制指令
	uint8	l_u8Key;
	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();

	while(1)
	{
		//画界面
		GUI_ClearSCR();
		GUI_Line(0		, 17	, 239	, 17	, 1);				
		GUI_Line(0		, 108	, 239	, 108	, 1);				
		
		if (p_u8Motion == UI_MOTION)		//动态
			LCDPRINTC(0		, 0		, "1. 弯板动态标定       Esc-返回");
		else 				//静态
			LCDPRINTC(0		, 0		, "1. 弯板静态标定       Esc-返回");

		LCDPRINTC(0		, 19	, "请输入通道号:");
		
		while (1)		//判断按键并刷新数据
		{
			//判断按键
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
					
			//是否重画
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;

			WAITSCREENREFRESH();		//等待刷新信号
		}

		//是否退出
		if (IfBreak(l_u8ControlCode))
			break;
	}
}

/*********************************************************************************************************
** Function name:		UIBDWanBan
** Descriptions:		弯板动态标定通道界面
** input parameters:	p_u8CID		通道号 
**						p_u8Motion	动静态标志 0表示静态，1表示动态 
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
	uint8	l_u8ControlCode;//控制指令
	uint32	l_u32Value;		//输入的数字
	uint8	l_u8InputStatus;	//输入状态；1：
	uint8	l_u8Key;
	int32	l_n32ADTmp;

	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();

	while(1)
	{
		l_u8InputStatus	= 0;
		l_u32Value	= 0;
		
		//画界面
		GUI_ClearSCR();
		GUI_Line(0		, 17	, 239	, 17	, 1);				
		GUI_Line(0		, 108	, 239	, 108	, 1);				
		
		LCDPRINTC(0		, 0		, "1. 弯板标定           Esc-返回");
		
		LCDPRINTFC(8	, 19	, " %u通道   "	, p_u8CID);
		
		LCDPRINTC(16	, 40	, "初始零位:");
		LCDPRINTFC(96	, 40	, "%u    "	, SETUPALIAS.an32Zero[p_u8CID-1]);
		LCDPRINTC(16	, 56	, "当前内码:");
		LCDPRINTC(16	, 72	, "当前重量:");
		LCDPRINTC(16	, 88	, "增益系数:");
		LCDPRINTFC(96	, 88	, "%u    "	, SETUPALIAS.an32AxGain[p_u8CID-1]);
		if (p_u8Motion == UI_STATIC)
			LCDPRINTC(0			, 112	, "F1清零  F2标定  F3增益  F4去皮");
		else
			LCDPRINTC(0			, 112	, "F1-清零  F2-标定  F3-输入系数");

		while (1)		//判断按键并刷新数据
		{
			//判断按键
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
				
				case KB_BKSP:		//回删一个数字
					if (l_u8InputStatus != 0)
					{ 
						l_u32Value	/= 10;
					}
					NotReDraw(l_u8ControlCode);
					break;	
				
				case KB_ENTER:		//确认输入
					if (l_u8InputStatus == KB_F3)	//输入增益系数
					{
						if (l_u32Value != 0)
							SETUPALIAS.an32AxGain[p_u8CID-1]	= l_u32Value;
					}
					else if (l_u8InputStatus == KB_F2)		//输入标定重量
					{
						if (l_u32Value != 0)
							SETUPALIAS.an32AxGain[p_u8CID-1]	= l_u32Value * 10000 / (ADAvg[p_u8CID-1] - SETUPALIAS.an32Zero[p_u8CID-1]);
					}
					ToReDraw(l_u8ControlCode);
					l_u8InputStatus	= 0;
					break;
				
				case KB_F1:			//F1，清零
					if (l_u8InputStatus == 0)
					{
						SETUPALIAS.an32Zero[p_u8CID-1]	= ADAvg[p_u8CID-1];
						ToReDraw(l_u8ControlCode);
						
						//更新动态零点
						g_an32MotionZero[p_u8CID-1]	= SETUPALIAS.an32Zero[p_u8CID-1];
					}
					break;

				case KB_F4:			//F4，静态去皮重
					if ((l_u8InputStatus == 0) && (p_u8Motion == UI_STATIC))
					{
						SETUPALIAS.an32Zero[p_u8CID-1]	= ADAvg[p_u8CID-1];
						ToReDraw(l_u8ControlCode);
					}
					else
						NotReDraw(l_u8ControlCode);
					break;

				case KB_F2:			//F2，标定
					if (l_u8InputStatus == 0)
					{
						l_u8InputStatus	= l_u8Key;
					}
					NotReDraw(l_u8ControlCode);
					break;
				
				case KB_F3:			//F3，输入系数
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
					
			//是否重画
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
			{
				break;
			}

			//刷新显示
			l_n32ADTmp	= ADAvg[p_u8CID-1];
			
			LCDPRINTFC(96		, 56	, "%u    "	, l_n32ADTmp);				//内码
			
			if (l_u8InputStatus != KB_F2)		//不是输入标定重量
			{
					LCDPRINTFC(96		, 72	, "%d kg    ", (l_n32ADTmp-SETUPALIAS.an32Zero[p_u8CID-1])*SETUPALIAS.an32AxGain[p_u8CID-1]/10000);		//重量
			}
			else		//输入标定重量
			{
				if (l_u32Value == 0)
					LCDPRINTC(96		, 72	, "         ");
				else
					LCDPRINTFC(96		, 72	, "%u      ", l_u32Value);
			}
			
			if (l_u8InputStatus == KB_F3)		//输入增益
			{
				if (l_u32Value == 0)
					LCDPRINTC(96		, 88	, "         ");
				else
					LCDPRINTFC(96		, 88	, "%u      ", l_u32Value);
			}

			WAITSCREENREFRESH();		//等待刷新信号量

		}

		//是否退出
		if (IfBreak(l_u8ControlCode))
			break;
	}
}

/*********************************************************************************************************
** Function name:		UISystemInit
** Descriptions:		系统初始化界面
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
	uint8	l_u8ControlCode;//控制指令
	uint8	l_u8IfPwd;		//是否为第二次输入的密码666666
	uint32	l_u32Value;		//输入数字
	uint8	l_u8Operation;	//初始化操作类型
	uint8	l_u8Tmp1;		//临时变量，循环用
	uint8	l_u8Key;
	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();
	
	while(1)
	{
		l_u8IfPwd	= 0;
		l_u32Value	= 0;
		l_u8Operation	= 0;
		//画界面
		GUI_ClearSCR();
		
		LCDPRINTC(0		, 0		, "1. 初始化为系统默认参数");
		LCDPRINTC(0		, 16	, "2. 保留调校参数并初始化");
		LCDPRINTC(0		, 32	, "3. 动态修正初始化");
		LCDPRINTC(0		, 48	, "4. 车型修正初始化");
		LCDPRINTC(0		, 64	, "5. 静态修正初始化");
		LCDPRINTC(0		, 80	, "6. 恢复至最近标定参数");
		LCDPRINTC(0		, 96	, "7. 恢复至历史标定参数");
		
		while (1)		//判断按键并刷新数据
		{
			//判断按键
			l_u8Key	= KeyValue;
			switch(l_u8Key)
			{
				case KB_ESC:
					ToBreak(l_u8ControlCode);
					break;

				case KB_1:		//初始化为系统默认参数
				case KB_2:		//保留调校参数并初始化
				case KB_3:		//动态修正初始化
				case KB_4:		//车型修正初始化
				case KB_5:		//静态修正初始化
				case KB_6:		//恢复至最近标定参数
				case KB_7:		//恢复至历史标定参数
					if (l_u8IfPwd == 0)
					{
						l_u8Operation	= l_u8Key;
						for (l_u8Tmp1 = 0; l_u8Tmp1 < 8; l_u8Tmp1 ++)
						{
							if (l_u8Tmp1 != l_u8Operation-1)
								ClearLCDLine(l_u8Tmp1<<4);
						}
						LCDPRINTC(16 	, 112	, "请输入口令:");
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
				
				case KB_ENTER:			//确认
					NotReDraw(l_u8ControlCode);
					l_u8IfPwd	= 0xf0;	//确认后，判断密码是否正确
					break;

				case 0xff:
					NotReDraw(l_u8ControlCode);
					break;

				default:		//其他按键，退出
					ToBreak(l_u8ControlCode);
					break;
			}	//switch(KeyValue)
			ClearKeyValue();
					
			//是否重画
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;
			
			//刷新
			if(l_u8IfPwd == 20)	//输入20个字符密码,则退出
			{
				ToBreak(l_u8ControlCode);
				break;
			}
			
			if (l_u8IfPwd == 0xf0)	//输入密码完毕
			{
				if (l_u32Value == 666666)		//密码正确
				{
					SETUPALIAS.u8DiaoDianTag	= 0;		//取消掉电保护标记
	
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
							if (RecoverToLast())	//成功
							{
								GUI_ClearSCR();
								LCDPRINTC(40 	, 56	, "最近参数恢复成功!");
								OSTimeDly(100);
							}
							else
							{
								GUI_ClearSCR();
								LCDPRINTC(40 	, 56	, "最近参数恢复失败!");
								OSTimeDly(100);
							}
							break;

						case 7:
							if (RecoverToHistory())	//成功
							{
								GUI_ClearSCR();
								LCDPRINTC(40 	, 56	, "历史参数恢复成功!");
								OSTimeDly(100);
							}
							else
							{
								GUI_ClearSCR();
								LCDPRINTC(40 	, 56	, "历史参数恢复失败!");
								OSTimeDly(100);
							}
							break;

						default:
							break;
					}
					GUI_ClearSCR();
					LCDPRINTC(40 	, 56	, "初始化完成, 重新启动...  ");
					InitRestart();
					OSTimeDly(100);
				}
				ToBreak(l_u8ControlCode);
				break;	
			}
			
			WAITSCREENREFRESH();		//等待刷新信号
		}

		//是否退出
		if (IfBreak(l_u8ControlCode))
			break;
	}
}

/*********************************************************************************************************
** Function name:		UIBDChengtaiChoose
** Descriptions:		称台动静态标定通道选择界面
** input parameters:	p_u8Motion	动静态标志 0表示静态，1表示动态
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
	uint8	l_u8ControlCode;//控制指令
	uint8	l_u8Key;
	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();

	while(1)
	{
		//画界面
		GUI_ClearSCR();
		GUI_Line(0		, 17	, 239	, 17	, 1);				
		GUI_Line(0		, 108	, 239	, 108	, 1);				
		
		if (p_u8Motion == UI_MOTION)		//动态
			LCDPRINTC(0		, 0		, "2. 称台动态标定       Esc-返回");
		else 				//静态
			LCDPRINTC(0		, 0		, "2. 称台静态标定       Esc-返回");

		LCDPRINTC(0		, 19	, "请输入台板号:");
		
		while (1)		//判断按键并刷新数据
		{
			//判断按键
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
					
			//是否重画
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;

			WAITSCREENREFRESH();		//等待刷新信号
		}

		//是否退出
		if (IfBreak(l_u8ControlCode))
			break;
	}
}

/*********************************************************************************************************
** Function name:		UIBDChengTaiAll
** Descriptions:		称台动态标定界面
** input parameters:	p_u8Motion	动静态，0表示静态，1表示动态
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
	uint8	l_u8ControlCode;//控制指令
	uint32	l_u32Value;		//输入的数字
	uint8	l_u8InputStatus;	//输入状态；1：
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

		//画界面
		GUI_ClearSCR();
		GUI_Line(0		, 17	, 239	, 17	, 1);				
		GUI_Line(0		, 108	, 239	, 108	, 1);				
		
		LCDPRINTC(0		, 0		, "2. 称台标定           Esc-返回");
		LCDPRINTC(8		, 19	, " 台板: ");
		for (l_u8TmpI = 0; l_u8TmpI < PLATNUM; l_u8TmpI ++)
		{
			LCDPRINTFC(64 + (l_u8TmpI<<5)	, 19	, "%d"	, l_u8TmpI+1);
			if (l_u8TmpI < PLATNUM -1)
			{
				LCDPRINTC(80 + (l_u8TmpI<<5)	, 19	, "+");
			}
		}
		LCDPRINTC(16		, 40	, "初始零位:");
		LCDPRINTFC(96		, 40	, "%u  "	, l_n32WholeZero);
		LCDPRINTC(16		, 56	, "当前内码:");
		LCDPRINTC(16		, 72	, "当前重量:");
		LCDPRINTC(16		, 88	, "增益系数:");
		LCDPRINTFC(96		, 88	, "%u  "	, SETUPALIAS.an32AxGain[0]);
		if (p_u8Motion == UI_STATIC)
			LCDPRINTC(0			, 112	, "F1清零  F2标定  F3增益  F4去皮");
		else
			LCDPRINTC(0			, 112	, "F1-清零  F2-标定  F3-输入系数");

		while (1)		//判断按键并刷新数据
		{
			//判断按键
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
				
				case KB_BKSP:		//回删一个数字
					if (l_u8InputStatus != 0)
					{ 
						l_u32Value	/= 10;
					}
					NotReDraw(l_u8ControlCode);
					break;	
				
				case KB_ENTER:		//确认输入
					if (l_u8InputStatus == KB_F3)	//输入增益系数
					{
						if (l_u32Value != 0)
						{
							for (l_u8TmpI = 0; l_u8TmpI < CHANNELNUM; l_u8TmpI ++)
							{
								SETUPALIAS.an32AxGain[l_u8TmpI]	= l_u32Value;
							}
						}
					}
					else if (l_u8InputStatus == KB_F2)		//输入标定重量
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
				
				case KB_F1:			//F1，清零
					if (l_u8InputStatus == 0)
					{
						for (l_u8TmpI = 0; l_u8TmpI < CHANNELNUM; l_u8TmpI ++)
						{
							SETUPALIAS.an32Zero[l_u8TmpI]	= ADAvg[l_u8TmpI];
						}
						ToReDraw(l_u8ControlCode);
					}
					break;

				case KB_F4:			//F4，静态去皮重
					if ((l_u8InputStatus == 0) && (p_u8Motion == UI_STATIC))	//静态才有效
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

				case KB_F2:			//F2，标定
					if (l_u8InputStatus == 0)
					{
						l_u8InputStatus	= l_u8Key;
					}
					NotReDraw(l_u8ControlCode);
					break;
				
				case KB_F3:			//F3，输入系数
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
					
			//是否重画
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
			{
				break;
			}

			//刷新显示
			l_n32WholeAD	= 0;
			l_n32WholeWeight	= 0;
			for (l_u8TmpI = 0; l_u8TmpI < CHANNELNUM; l_u8TmpI ++)
			{
				l_an32ADTmp[l_u8TmpI]	= ADAvg[l_u8TmpI];
				l_n32WholeAD	+= l_an32ADTmp[l_u8TmpI];
				l_n32WholeWeight	+= (l_an32ADTmp[l_u8TmpI] - SETUPALIAS.an32Zero[l_u8TmpI])* SETUPALIAS.an32AxGain[l_u8TmpI] / 10000;
			}

			LCDPRINTFC(96		, 56	, "%u    "	, l_n32WholeAD);				//内码
			
			if (l_u8InputStatus != KB_F2)		//不是输入标定重量
			{
				LCDPRINTFC(96		, 72	, "%d kg      ", l_n32WholeWeight);		//重量
			}
			else		//输入标定重量
			{
				if (l_u32Value == 0)
					LCDPRINTC(96		, 72	, "         ");
				else
					LCDPRINTFC(96		, 72	, "%u      "	, l_u32Value);
			}
			
			if (l_u8InputStatus == KB_F3)		//输入增益
			{
				if (l_u32Value == 0)
					LCDPRINTC(96		, 88	, "         ");
				else
					LCDPRINTFC(96		, 88	, "%u    "	, l_u32Value);
			}

			WAITSCREENREFRESH();		//等待刷新信号
		}

		//是否退出
		if (IfBreak(l_u8ControlCode))
			break;
	}
}

/*********************************************************************************************************
** Function name:		UIBDChengTai
** Descriptions:		称台动态标定界面
** input parameters:	p_u8CID		台板号
**						p_u8Motion	动静态，0表示静态，1表示动态
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
	uint8	l_u8ControlCode;//控制指令
	uint32	l_u32Value;		//输入的数字
	uint8	l_u8InputStatus;	//输入状态；1：
	uint8	l_u8Key;
	int32	l_an32ADTmp[2];

	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();

	while(1)
	{
		l_u8InputStatus	= 0;
		l_u32Value	= 0;
		
		//画界面
		GUI_ClearSCR();
		GUI_Line(0		, 17	, 239	, 17	, 1);				
		GUI_Line(0		, 108	, 239	, 108	, 1);				
		
		LCDPRINTC(0		, 0		, "2. 称台标定           Esc-返回");
		//LCDPRINTC(0		, 19	, "请输入通道号:");
		LCDPRINTFC(8		, 19	, " %u台板"	, p_u8CID);
		
		LCDPRINTC(16		, 40	, "初始零位:");
		LCDPRINTFC(96		, 40	, "%u  "	, SETUPALIAS.an32Zero[2*(p_u8CID-1)]+SETUPALIAS.an32Zero[2*(p_u8CID-1)+1]);
		LCDPRINTC(16		, 56	, "当前内码:");
		LCDPRINTC(16		, 72	, "当前重量:");
		LCDPRINTC(16		, 88	, "增益系数:");
		LCDPRINTFC(96		, 88	, "%u  "	, SETUPALIAS.an32AxGain[2*(p_u8CID-1)]);
		if (p_u8Motion == UI_STATIC)
			LCDPRINTC(0			, 112	, "F1清零  F2标定  F3增益  F4去皮");
		else
			LCDPRINTC(0			, 112	, "F1-清零  F2-标定  F3-输入系数");

		while (1)		//判断按键并刷新数据
		{
			//判断按键
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
				
				case KB_BKSP:		//回删一个数字
					if (l_u8InputStatus != 0)
					{ 
						l_u32Value	/= 10;
					}
					NotReDraw(l_u8ControlCode);
					break;	
				
				case KB_ENTER:		//确认输入
					if (l_u8InputStatus == KB_F3)	//输入增益系数
					{
						if (l_u32Value != 0)
						{
							SETUPALIAS.an32AxGain[2*(p_u8CID-1)]	= l_u32Value;
							SETUPALIAS.an32AxGain[2*(p_u8CID-1)+1]	= l_u32Value;
						}
					}
					else if (l_u8InputStatus == KB_F2)		//输入标定重量
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
				
				case KB_F1:			//F1，清零
					if (l_u8InputStatus == 0)
					{
						SETUPALIAS.an32Zero[2*(p_u8CID-1)]		= ADAvg[2*(p_u8CID-1)];
						SETUPALIAS.an32Zero[2*(p_u8CID-1)+1]	= ADAvg[2*(p_u8CID-1)+1];
						ToReDraw(l_u8ControlCode);
					}
					break;

				case KB_F4:			//F4，静态去皮重
					if ((l_u8InputStatus == 0) && (p_u8Motion == UI_STATIC))	//静态才有效
					{
						SETUPALIAS.an32Zero[2*(p_u8CID-1)]		= ADAvg[2*(p_u8CID-1)];
						SETUPALIAS.an32Zero[2*(p_u8CID-1)+1]	= ADAvg[2*(p_u8CID-1)+1];
						ToReDraw(l_u8ControlCode);
					}
					else
						NotReDraw(l_u8ControlCode);
					break;

				case KB_F2:			//F2，标定
					if (l_u8InputStatus == 0)
					{
						l_u8InputStatus	= l_u8Key;
					}
					NotReDraw(l_u8ControlCode);
					break;
				
				case KB_F3:			//F3，输入系数
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
					
			//是否重画
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
			{
				break;
			}

			//刷新显示
			l_an32ADTmp[0]	= ADAvg[2*(p_u8CID-1)];
			l_an32ADTmp[1]	= ADAvg[2*(p_u8CID-1)+1];

			LCDPRINTFC(96		, 56	, "%u    "	, l_an32ADTmp[0]+l_an32ADTmp[1]);				//内码
			
			if (l_u8InputStatus != KB_F2)		//不是输入标定重量
			{
				LCDPRINTFC(96		, 72	, "%d kg    ", (l_an32ADTmp[0]-SETUPALIAS.an32Zero[2*(p_u8CID-1)])*SETUPALIAS.an32AxGain[2*(p_u8CID-1)]/10000+(l_an32ADTmp[1]-SETUPALIAS.an32Zero[2*(p_u8CID-1)+1])*SETUPALIAS.an32AxGain[2*(p_u8CID-1)+1]/10000);		//重量
			}
			else		//输入标定重量
			{
				if (l_u32Value == 0)
					LCDPRINTC(96		, 72	, "         ");
				else
					LCDPRINTFC(96		, 72	, "%u    "	, l_u32Value);
			}
			
			if (l_u8InputStatus == KB_F3)		//输入增益
			{
				if (l_u32Value == 0)
					LCDPRINTC(96		, 88	, "         ");
				else
					LCDPRINTFC(96		, 88	, "%u    "	, l_u32Value);
			}

			WAITSCREENREFRESH();		//等待刷新信号
		}

		//是否退出
		if (IfBreak(l_u8ControlCode))
			break;
	}
}

/*********************************************************************************************************
** Function name:		UIBDGenZong
** Descriptions:		零点跟踪使能界面
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
	UISetEnableParam(" 零点跟踪使能" , &(SETUPALIAS.u8Genzong));
}

/*********************************************************************************************************
** Function name:		UIBDPoDu
** Descriptions:		坡度修正界面
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
	UISetValueParamU16("坡度修正", &(SETUPALIAS.u16Podu), 9000, 11000);
}

/*********************************************************************************************************
** Function name:		UIBDLunZhou
** Descriptions:		无轮轴程序使能界面
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
	uint8	l_u8ControlCode;//控制指令

	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();

	while(1)
	{
		//画界面
		GUI_ClearSCR();
		LCDPRINTC(0		, 0		, " 无轮轴程序使能");
		LCDPRINTC(176	, 0		, "Esc 返回");
		
		LCDPRINTC(8		, 24	, "是否使能:");
		LCDPRINTC(8		, 44	, "1.使能  2.禁止");
		
		GUI_Line(0		, 18	, 239	, 18	, 1);
		GUI_Line(0		, 124	, 239	, 124	, 1);
		GUI_Line(0		, 127	, 239	, 127	, 1);
		
		while (1)		//判断按键并刷新数据
		{
			//判断按键
			switch(KeyValue)
			{
				case KB_ESC:
				case KB_ENTER:
					ToBreak(l_u8ControlCode);
					break;
					
				case KB_1:		//使能
					NotReDraw(l_u8ControlCode);
					SETUPALIAS.u8LunZhouERR	|= 0x02;
					break;	
						
				case KB_2:		//禁止		
					NotReDraw(l_u8ControlCode);
					SETUPALIAS.u8LunZhouERR	&= ~0x02;
					break;	
					
				default:
					NotReDraw(l_u8ControlCode);
					break;
			}	//switch(KeyValue)
			ClearKeyValue();
					
			//是否重画
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;

			//刷新屏幕
			if (SETUPALIAS.u8LunZhouERR & 0x02)		//为1时，使能
				LCDPRINTC(120		, 24	, "使能  ");
			else
				LCDPRINTC(120		, 24	, "禁止  ");
			
			WAITSCREENREFRESH();		//等待刷新信号
		}

		//是否退出
		if (IfBreak(l_u8ControlCode))
			break;

	}
}

/*********************************************************************************************************
** Function name:		UISetLunZhouEnable
** Descriptions:		设置轮轴故障报错使能界面
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
	uint8	l_u8ControlCode;//控制指令

	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();

	while(1)
	{
		//画界面
		GUI_ClearSCR();
		LCDPRINTC(0		, 0		, " 轮轴故障报错使能");
		LCDPRINTC(176	, 0		, "Esc 返回");
		
		LCDPRINTC(8		, 24	, "是否报错:");
		LCDPRINTC(8		, 44	, "1.报错  2.不报错");
		
		GUI_Line(0		, 18	, 239	, 18	, 1);
		GUI_Line(0		, 124	, 239	, 124	, 1);
		GUI_Line(0		, 127	, 239	, 127	, 1);
		
		while (1)		//判断按键并刷新数据
		{
			//判断按键
			switch(KeyValue)
			{
				case KB_ESC:
				case KB_ENTER:
					ToBreak(l_u8ControlCode);
					ClearKeyValue();
					break;
					
				case KB_1:		//报错
					NotReDraw(l_u8ControlCode);
					SETUPALIAS.u8LunZhouERR	|= 0x01;
					ClearKeyValue();
					break;	
						
				case KB_2:		//不报错		
					NotReDraw(l_u8ControlCode);
					SETUPALIAS.u8LunZhouERR	&= ~0x01;
					ClearKeyValue();
					break;	
					
				default:
					NotReDraw(l_u8ControlCode);
					ClearKeyValue();
					break;
			}	//switch(KeyValue)

			//是否重画
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;

			//刷新屏幕
			if (SETUPALIAS.u8LunZhouERR & 0x01)		//为1时，报错
				LCDPRINTC(120		, 24	, "报错    ");
			else
				LCDPRINTC(120		, 24	, "不报错  ");
			
			WAITSCREENREFRESH();		//等待刷新信号
		}

		//是否退出
		if (IfBreak(l_u8ControlCode))
			break;

	}
}
  
#if	SENDWAVEENABLE > 0		//使能发波形
/*********************************************************************************************************
** Function name:		UIBDSendWave
** Descriptions:		发送波形使能界面
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
	UISetEnableParam(" 发送波形使能" , &(SETUPALIAS.u8SendWaveEnable));
	
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
** Descriptions:		看门狗设置界面
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
	UISetEnableParam(" 看门狗使能" , &(SETUPALIAS.u8DOG));
}

/*********************************************************************************************************
** Function name:		UISetCapture
** Descriptions:		设置抓拍使能界面
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
	UISetEnableParam(" 上称抓拍使能" , &(SETUPALIAS.u8ZhuapaiEnable));
}

/*********************************************************************************************************
** Function name:		UISetLoop
** Descriptions:		线圈触发使能界面
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
	UISetEnableParam(" 线圈触发使能" , &(SETUPALIAS.u8LoopTriggerEnable));
}
   
/*********************************************************************************************************
** Function name:		UISetForwardEnable
** Descriptions:		方向使能界面
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
	UISetEnableParam(" 方向使能" , &(SETUPALIAS.u8FangxiangEnable));
}

/*********************************************************************************************************
** Function name:		UIViewAuthor
** Descriptions:		查看作者信息界面
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
	//画界面
	GUI_ClearSCR();
	ClearKeyValue();

	LCDPRINTC(16		, 32	, "作者: 田林岩");
	LCDPRINTC(16		, 64	, "电话: 13801298463");
	LCDPRINTC(16		, 96	, "tly001@vip.sina.com");
	
	GUI_Line(0		, 18	, 239	, 18	, 1);
	GUI_Line(0		, 124	, 239	, 124	, 1);
	GUI_Line(0		, 127	, 239	, 127	, 1);
		
	while (KeyValue == 0xff)		//有按键
	{
		WAITSCREENREFRESH();		//等待刷新信号
	}
}

/*********************************************************************************************************
** Function name:		UISetProtocol
** Descriptions:		设置协议界面
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
	uint8	l_u8ControlCode;//控制指令
	uint8	l_u8Key;
	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();

	while(1)
	{
		//画界面
		GUI_ClearSCR();
		LCDPRINTC(64	, 0		, "通讯协议:");

		LCDPRINTC(8		, 24	, "0--江苏");
		LCDPRINTC(8		, 40	, "1--万集");
		LCDPRINTC(8		, 56	, "2--智运");
		LCDPRINTC(8		, 72	, "3--安徽");
		LCDPRINTC(8		, 88	, "4--福建");
		LCDPRINTC(8		, 104	, "5--四川");
		LCDPRINTC(128	, 24	, "6--贵州");
		LCDPRINTC(128	, 40	, "7--陕西");
		LCDPRINTC(128	, 56	, "8--辽宁");
		LCDPRINTC(128	, 72	, "9--河南");

		GUI_Line(0		, 18	, 239	, 18	, 1);
		GUI_Line(0		, 124	, 239	, 124	, 1);
		GUI_Line(0		, 127	, 239	, 127	, 1);
		
		while (1)		//判断按键并刷新数据
		{
			//判断按键
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
					
			//是否重画
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;

			//刷新屏幕
			LCDPRINTFC(160	, 0		, "%d  ",SETUPALIAS.u8Protocol);

			WAITSCREENREFRESH();		//等待刷新信号
		}

		//是否退出
		if (IfBreak(l_u8ControlCode))
			break;
	}
}

/*********************************************************************************************************
** Function name:		UISetBaudRate
** Descriptions:		设置波特率界面
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
	uint8	l_u8ControlCode;//控制指令
	uint8	l_u8Key;
	uint32	l_au32BR[6] = {4800, 4800, 9600, 38400, 57600, 115200};
	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();

	while(1)
	{
		//画界面
		GUI_ClearSCR();
		LCDPRINTC(0		, 0		, "2. 波特率设定       Esc-返回");
		
		LCDPRINTC(0		, 19	, "请选择波特率:");
		LCDPRINTC(64	, 40	, "1.  4800");
		LCDPRINTC(64	, 56	, "2.  9600");
		LCDPRINTC(64	, 72	, "3.  38400");
		LCDPRINTC(64	, 88	, "4.  57600");
		LCDPRINTC(64	, 104	, "5.  115200");
				
		GUI_Line(0		, 17	, 239	, 17	, 1);
		GUI_Line(0		, 124	, 239	, 124	, 1);
		GUI_Line(0		, 127	, 239	, 127	, 1);
				
		while (1)		//判断按键并刷新数据
		{
			//判断按键
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
					
			//是否重画
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;

			//刷新屏幕
			LCDPRINTFC(112	, 19	, "%u    "	, l_au32BR[SETUPALIAS.u8BaudRate]);

			WAITSCREENREFRESH();		//等待刷新信号
		}

		//是否退出
		if (IfBreak(l_u8ControlCode))
			break;
	}
}

/*********************************************************************************************************
** Function name:		UISetCommandMode
** Descriptions:		设置命令模式界面
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
	uint8	l_u8ControlCode;//控制指令
	uint8	l_u8Key;
	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();

	while(1)
	{
		//画界面
		GUI_ClearSCR();
		LCDPRINTC(64		, 56	, "命令方式: ");
					
		while (1)		//判断按键并刷新数据
		{
			//判断按键
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
					
			//是否重画
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;

			//刷新屏幕
			LCDPRINTFC(144	, 56	, "%2d"	, SETUPALIAS.u8ComMode);
		
			WAITSCREENREFRESH();		//等待刷新信号
		}

		//是否退出
		if (IfBreak(l_u8ControlCode))
			break;
	}
}

/*********************************************************************************************************
** Function name:		UISetPlat
** Descriptions:		设置台面宽度界面
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
	UISetValueParamU8("台面宽度", &(SETUPALIAS.u8PlatWidth), 1, 127);  
	g_u16PlatWidth			= (SETUPALIAS.u8PlatWidth+30)*36*POINTRATE/1000;		//cw	
}

/*********************************************************************************************************
** Function name:		UISetVehicleCache
** Descriptions:		设置车辆缓存界面(1~10)，默认10
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
	UISetValueParamU8("车辆缓存", &(SETUPALIAS.u8VehicleBufSize), 0, MAXBUFNUM);
}
   
/*********************************************************************************************************
** Function name:		UISetDiaodian
** Descriptions:		掉电保护使能界面
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
	UISetEnableParam(" 掉电保护使能" , &(SETUPALIAS.u8DiaoDianTag));
}

/*********************************************************************************************************
** Function name:		UISetEnableParam
** Descriptions:		通用使能参数设置界面
** input parameters:	p_pcName		参数名
**						p_pu8Param 		变量指针
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
	uint8	l_u8ControlCode;//控制指令

	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();

	while(1)
	{
		//画界面
		GUI_ClearSCR();
		LCDPRINTC(0		, 0		, p_pcName);
		LCDPRINTC(176	, 0		, "Esc 返回");
		
		LCDPRINTC(8		, 24	, "是否使能:");
		LCDPRINTC(8		, 44	, "1.使能  2.禁止");
		GUI_Line(0		, 18	, 239	, 18	, 1);
		GUI_Line(0		, 124	, 239	, 124	, 1);
		GUI_Line(0		, 127	, 239	, 127	, 1);
		
		while (1)		//判断按键并刷新数据
		{
			//判断按键
			switch(KeyValue)
			{
				case KB_ESC:
				case KB_ENTER:
					ToBreak(l_u8ControlCode);
					break;
					
				case KB_1:		//使能
					NotReDraw(l_u8ControlCode);
					* p_pu8Param	= 1;
					break;	
						
				case KB_2:		//禁止		
					NotReDraw(l_u8ControlCode);
					* p_pu8Param	= 0;
					break;	
					
				default:
					NotReDraw(l_u8ControlCode);
					break;
			}	//switch(KeyValue)
			ClearKeyValue();

			//是否重画
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;

			//刷新屏幕
			if (* p_pu8Param)		//为1时，使能
				LCDPRINTC(120		, 24	, "使能  ");
			else
				LCDPRINTC(120		, 24	, "禁止  ");
			
			WAITSCREENREFRESH();		//等待刷新信号
		}

		//是否退出
		if (IfBreak(l_u8ControlCode))
			break;

	}
}

/*********************************************************************************************************
** Function name:		UISetValueParamU8
** Descriptions:		通用参数设置界面,设置U8参数
** input parameters:	p_pcName		参数名
**						p_pu8Param 		变量指针
**						p_u32Max		参数上限 
**						p_u32Min		参数下限
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
	uint8	l_u8ControlCode;//控制指令
	uint32	l_u32Value	= 0;		//输入数字
	uint8	l_u8Status;		//功能键状态
	uint8	l_u8Key;		//按键
	char	l_acTmp[30];

	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();

	while(1)
	{
		l_u32Value	= 0;
		l_u8Status	= 0;

		//画界面
		GUI_ClearSCR();
		memset(l_acTmp, 0, 30);
		sprintf(l_acTmp,"%s: %u",p_pcName,*p_pu8Param);
		LCDPRINTC(0		, 0		, l_acTmp);
		LCDPRINTC(0		, 32	, " Esc退出, F1键修改");
		
		GUI_Line(0		, 18	, 239	, 18	, 1);
		GUI_Line(0		, 124	, 239	, 124	, 1);
		GUI_Line(0		, 127	, 239	, 127	, 1);
		
		while (1)		//判断按键并刷新数据
		{
			//判断按键
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
				
				case KB_BKSP:		//回删一个数字
					NotReDraw(l_u8ControlCode);
					l_u32Value	/= 10;
					break;	
				
				case KB_F1:		//修改状态，开始输入数字
					if (l_u8Status == 0)
					{
						l_u8Status	= KB_F1;
						l_u32Value	= 0;
						memset(l_acTmp, 0, 30);
						sprintf(l_acTmp," 请输入新值(%u-%u):",p_u32Min,p_u32Max);
						LCDPRINTC(16		, 56	, l_acTmp);
					}
					NotReDraw(l_u8ControlCode);
					break;	
				
				case KB_ENTER:		//确认输入
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

			//是否重画
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;

			//刷新屏幕
			if (l_u8Status == KB_F1)		//输入状态
			{
				if (l_u32Value == 0)
					LCDPRINTC(56		, 80	, "         ");
				else
					LCDPRINTFC(56		, 80	, "%u      ", l_u32Value);
			}
	
			WAITSCREENREFRESH();		//等待刷新信号
		}

		//是否退出
		if (IfBreak(l_u8ControlCode))
			break;
	}
}

/*********************************************************************************************************
** Function name:		UISetValueParamU16
** Descriptions:		通用参数设置界面,设置U16参数
** input parameters:	p_pcName		参数名
**						p_pu16Param 	变量指针
**						p_u32Max		参数上限 
**						p_u32Min		参数下限
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
	uint8	l_u8ControlCode;//控制指令
	uint32	l_u32Value	= 0;		//输入数字
	uint8	l_u8Status;		//功能键状态
	char	l_acTmp[30];
	uint8	l_u8Key;
	
	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();

	while(1)
	{
		l_u32Value	= 0;
		l_u8Status	= 0;

		//画界面
		GUI_ClearSCR();
		memset(l_acTmp, 0, 30);
		sprintf(l_acTmp,"%s: %u",p_pcName,*p_pu16Param);
		LCDPRINTC(0		, 0		, l_acTmp);
		LCDPRINTC(0		, 32	, " Esc退出, F1键修改");
		
		GUI_Line(0		, 18	, 239	, 18	, 1);
		GUI_Line(0		, 124	, 239	, 124	, 1);
		GUI_Line(0		, 127	, 239	, 127	, 1);
		
		while (1)		//判断按键并刷新数据
		{
			//判断按键
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
				
				case KB_BKSP:		//回删一个数字
					NotReDraw(l_u8ControlCode);
					l_u32Value	/= 10;
					break;	
				
				case KB_F1:		//修改状态，开始输入数字
					if (l_u8Status == 0)
					{
						l_u8Status	= KB_F1;
						l_u32Value	= 0;
						memset(l_acTmp, 0, 30);
						sprintf(l_acTmp," 请输入新值(%u-%u):",p_u32Min,p_u32Max);
						LCDPRINTC(16		, 56	, l_acTmp);
					}
					NotReDraw(l_u8ControlCode);
					break;	
				
				case KB_ENTER:		//确认输入
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
					
			//是否重画
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;

			//刷新屏幕
			if (l_u8Status == KB_F1)		//输入状态
			{
				if (l_u32Value == 0)
					LCDPRINTC(56		, 80	, "         ");
				else
					LCDPRINTFC(56		, 80	, "%u      "	, l_u32Value);
			}

			WAITSCREENREFRESH();		//等待刷新信号
		}

		//是否退出
		if (IfBreak(l_u8ControlCode))
			break;
	}
}

/*********************************************************************************************************
** Function name:		UIValidate
** Descriptions:		验证界面
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
	uint8	l_u8ControlCode;//控制指令
	uint32	l_u32Value;		//输入数字
	uint32	l_u32SN;		//SN值
	uint8	l_u8DigitCnt;	//数字个数
	uint8	l_u8Tmp1;		//临时变量
	uint8	l_u8Key;
	uint32	l_u32SNTimer;	//由Timer生成的SN明码
	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();
	l_u32SN	= 0;

	while(1)
	{
		l_u32SNTimer	= SNALIAS;
		l_u32Value	= 0;
		l_u8DigitCnt= 0;
		
		//画界面
		GUI_ClearSCR();
		LCDPRINTFC(16		, 32	, "SN:         %d", l_u32SNTimer);
		LCDPRINTC(16		, 64	, "请输入口令: ");
		
		while (1)		//判断按键并刷新数据
		{
			//判断按键
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
				
				case KB_BKSP:		//回删一个数字
					NotReDraw(l_u8ControlCode);
					if (l_u8DigitCnt>0)
						l_u8DigitCnt	--;
					l_u32Value	/= 10;
					break;	
				
				case KB_ENTER:		//系统初始化
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
					
			//是否重画
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;

			//刷新屏幕
			LCDPRINTC(112 + l_u8DigitCnt<<3 	, 64	, "  ");

			for (l_u8Tmp1 = 0; l_u8Tmp1 < l_u8DigitCnt; l_u8Tmp1 ++)
			{
				LCDPRINTC(112 + l_u8Tmp1<<3 	, 64	, "*");
			}

			WAITSCREENREFRESH();		//等待刷新信号量
		}

		//是否退出
		if (IfBreak(l_u8ControlCode))
			break;

	}
}

							  
#if	YBVERSION >= 30		//3.0仪表功能
/*********************************************************************************************************
** Function name:		UIViewStartUpTime
** Descriptions:		查看系统启动时间记录
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
			
	//画界面
	GUI_ClearSCR();
	ClearKeyValue();

	ReadC256(STARTRECINDEXADDR,(uint8 *)&l_u16RecIndex,2);		//找到有效记录号
	l_u16RecIndex	&= 0x3ff;
	l_u16Tmp		= l_u16RecIndex;
	l_u16RecIndex	++;
	l_u16RecIndex	&= 0x3ff;
	l_u32StartCnt	= g_u32StartupTime;

	l_u16Tmp2	= 0;		//启动时间计数，由于分屏显示
	while ((l_u32StartCnt > 0)&&(l_u16Tmp != l_u16RecIndex))
	{
		//读取启动时间8B数据
		ReadNORFlash(NORSTARTREC	+ (l_u16Tmp << 3), 8, &l_au8RecData[0]);
		
		//判断是否为有效数据
		if (l_au8RecData[7] != 0xaa)	//无效
		{
			break;	
		}

		//显示启动时间
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

		//暂停 		
		l_u32StartCnt	--;
		ClearKeyValue();
		if (l_u16Tmp2 % 8 == 0)
		{
			while (KeyValue == 0xff)		//有按键
			{
				WAITSCREENREFRESH();		//等待刷新信号量
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
	
	if (l_u8Status & 0x01)		//如果终止退出，则等待按键
	{							
		ClearKeyValue();
		while (KeyValue == 0xff)		//有按键
		{
			WAITSCREENREFRESH();		//等待刷新信号量
			//OSTimeDly(LOOPTIME);		//延时，轮询时间
		}
		if (KeyValue == KB_ESC)
			return;
		ClearKeyValue();
		GUI_ClearSCR();
	}
}

/*********************************************************************************************************
** Function name:		UIViewIPInfo
** Descriptions:		IP参数，444
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
	uint8	l_u8ControlCode;//控制指令

	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();

	while(1)
	{
		//画界面
		GUI_ClearSCR();
		
		GUI_Line(0		, 16	, 239	, 16	, 1);
		GUI_Line(0		, 128	, 239	, 128	, 1);
		
		LCDPRINTC(0		, 0		, " 网络配置参数          0-保存 ");
		
		LCDPRINTC(0		, 17	, "1.MAC地址");
//		memset(m_acTmp,0, sizeof(m_acTmp));
		sprintf(m_acTmp, "%2x-%2x-%2x-%2x-%2x-%2x", IPINFOALIAS.au8MACAddr[0], IPINFOALIAS.au8MACAddr[1], IPINFOALIAS.au8MACAddr[2], IPINFOALIAS.au8MACAddr[3], IPINFOALIAS.au8MACAddr[4], IPINFOALIAS.au8MACAddr[5]); 
		LCDPRINTC(96	, 17	, m_acTmp);

		LCDPRINTC(0		, 33	, "2.本地IP");
//		memset(m_acTmp,0, sizeof(m_acTmp));
		sprintf(m_acTmp, "%3d.%3d.%3d.%3d", IPINFOALIAS.au8IPAddr[0], IPINFOALIAS.au8IPAddr[1], IPINFOALIAS.au8IPAddr[2], IPINFOALIAS.au8IPAddr[3]); 
		LCDPRINTC(96	, 33	, m_acTmp);

		LCDPRINTC(0		, 48	, "3.本地端口");
//		memset(m_acTmp,0, sizeof(m_acTmp));
		sprintf(m_acTmp, "%-d", IPINFOALIAS.u32LocalPortNO); 
		LCDPRINTC(96	, 48	, m_acTmp);

		LCDPRINTC(0		, 64	, "4.子网掩码");
//		memset(m_acTmp,0, sizeof(m_acTmp));
		sprintf(m_acTmp, "%3d.%3d.%3d.%3d", IPINFOALIAS.au8SubMask[0], IPINFOALIAS.au8SubMask[1], IPINFOALIAS.au8SubMask[2], IPINFOALIAS.au8SubMask[3]); 
		LCDPRINTC(96	, 64	, m_acTmp);
		
		LCDPRINTC(0		, 79	, "5.默认网关");
//		memset(m_acTmp,0, sizeof(m_acTmp));
		sprintf(m_acTmp, "%3d.%3d.%3d.%3d", IPINFOALIAS.au8GatewayIP[0], IPINFOALIAS.au8GatewayIP[1], IPINFOALIAS.au8GatewayIP[2], IPINFOALIAS.au8GatewayIP[3]); 
		LCDPRINTC(96	, 79	, m_acTmp);

		LCDPRINTC(0		, 95	, "6.远程IP");
//		memset(m_acTmp,0, sizeof(m_acTmp));
		sprintf(m_acTmp, "%3d.%3d.%3d.%3d", IPINFOALIAS.au8ServerIP[0], IPINFOALIAS.au8ServerIP[1], IPINFOALIAS.au8ServerIP[2], IPINFOALIAS.au8ServerIP[3]); 
		LCDPRINTC(96	, 95	, m_acTmp);

		LCDPRINTC(0		, 111	, "7.远程端口");
//		memset(m_acTmp,0, sizeof(m_acTmp));
		sprintf(m_acTmp, "%-d", IPINFOALIAS.u32ServerPortNO); 
		LCDPRINTC(96	, 111	, m_acTmp);

		while (1)		//判断按键并刷新数据
		{
			ToReDraw(l_u8ControlCode);
					
			//判断按键
			switch(KeyValue)
			{
				case KB_ESC:
				case KB_ENTER:
					ToBreak(l_u8ControlCode);
					break;
					
				case KB_1:		//MAC不可修改
					NotReDraw(l_u8ControlCode);
					break;

				case KB_2:		//本地IP
					UISetValueParamIP("本地IP", IPINFOALIAS.au8IPAddr);
					break;	
					
				case KB_3:		//本地端口
					UISetValueParamU32("本地端口", &(IPINFOALIAS.u32LocalPortNO), 0, 0);
					break;	
					
				case KB_4:		//子网掩码
					UISetValueParamIP("子网掩码", IPINFOALIAS.au8SubMask);
					break;	
					
				case KB_5:		//默认网关
					UISetValueParamIP("默认网关", IPINFOALIAS.au8GatewayIP);
					break;	
					
				case KB_6:		//远程IP
					UISetValueParamIP("远程IP", IPINFOALIAS.au8ServerIP);
					break;	
					
				case KB_7:		//远程端口
					UISetValueParamU32("远程端口", &(IPINFOALIAS.u32ServerPortNO), 0, 0);
					break;	
					
				case KB_0:		//保存
					SaveNetInfo();
					ToReDraw(l_u8ControlCode);
					break;
					
				case KB_I:		//初始化
					InitNetParam();
					ToReDraw(l_u8ControlCode);
					break;
					
				default:
					NotReDraw(l_u8ControlCode);
					
					break;
			}	//switch(KeyValue)
			ClearKeyValue();

			//是否重画
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;

			WAITSCREENREFRESH();		//等待刷新信号量
			//OSTimeDly(LOOPTIME);		//延时，轮询时间
		}

		//是否退出
		if (IfBreak(l_u8ControlCode))
			break;
	}
}

/*********************************************************************************************************
** Function name:		UISetValueParamIP
** Descriptions:		修改IP参数
** input parameters:	p_pcName		参数名
**						p_pu8IP		 	变量指针
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
	uint8	l_u8ControlCode;		//控制指令
	uint32	l_u32Value	= 0;		//输入数字
	uint8	l_u8Status;				//功能键状态
	uint8	l_u8Key;
	uint8	l_au8TmpIP[4];
	char	l_acTmp[30];

	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();

	while(1)
	{
		l_u32Value	= 0;
		l_u8Status	= 0;

		//画界面
		GUI_ClearSCR();
		memset(l_acTmp, 0, 30);
		sprintf(l_acTmp,"%s",p_pcName);
		LCDPRINTC(0		, 0		, l_acTmp);		   
		memset(l_acTmp	, 0		, 30);
		sprintf(l_acTmp	,"%3d.%3d.%3d.%3d"	, *p_pu8IP, *(p_pu8IP+1), *(p_pu8IP+2), *(p_pu8IP+3));
		LCDPRINTC(24	, 20	, l_acTmp);
		LCDPRINTC(0		, 36	, " Esc退出, F1键修改");
		
		GUI_Line(0		, 18	, 239	, 18	, 1);
		GUI_Line(0		, 124	, 239	, 124	, 1);
		GUI_Line(0		, 127	, 239	, 127	, 1);
		
		while (1)		//判断按键并刷新数据
		{
			//判断按键
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
				
				case KB_BKSP:		//回删一个数字
					NotReDraw(l_u8ControlCode);
					l_u32Value	/= 10;
					break;	
				
				case KB_F1:		//修改状态，开始输入数字
					if (l_u8Status == 0)
					{
						l_u8Status	= 1;
						l_u32Value	= 0;
						LCDPRINTC(16		, 56	,"请输入新值: ");
					}
					NotReDraw(l_u8ControlCode);
					break;	
				
				case KB_ENTER:		//确认输入
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
									LCDPRINTC(32		, 92	, "输入完毕. ");	
									LCDPRINTC(32		, 108	, "Enter:保存 Esc:取消");
								}
								else
								{											
									LCDPRINTC((l_u8Status<<5) + 16	, 76	, ".");
								}
							}
							else
								ToReDraw(l_u8ControlCode);
									
							break;
						
						case 5:		//确认保存
							//保存
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

			//是否重画
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;

			if (l_u8Status	> 0)
			{
				if (l_u32Value != 0)
					LCDPRINTFC((l_u8Status<<5) - 8	, 76	, "%u    "	, l_u32Value);
				else
					LCDPRINTC((l_u8Status<<5) - 8	, 76	, "      ");		
			}

			WAITSCREENREFRESH();		//等待刷新信号量
		}

		//是否退出
		if (IfBreak(l_u8ControlCode))
			break;
	}
}

/*********************************************************************************************************
** Function name:		UISetValueParamU32
** Descriptions:		通用参数设置界面,设置U32参数
** input parameters:	p_pcName		参数名
**						p_pu32Param 	变量指针
**						p_u32Max		参数上限 
**						p_u32Min		参数下限
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
	uint8	l_u8ControlCode;//控制指令
	uint32	l_u32Value	= 0;		//输入数字
	uint8	l_u8Status;		//功能键状态
	uint8	l_u8Key;
	char	l_acTmp[30];

	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();

	while(1)
	{
		l_u32Value	= 0;
		l_u8Status	= 0;

		//画界面
		GUI_ClearSCR();
		memset(l_acTmp, 0, 30);
		sprintf(l_acTmp,"%s: %u",p_pcName,*p_pu32Param);
		LCDPRINTC(0		, 0		, l_acTmp);
		LCDPRINTC(0		, 32	, " Esc退出, F1键修改");
		
		GUI_Line(0		, 18	, 239	, 18	, 1);
		GUI_Line(0		, 124	, 239	, 124	, 1);
		GUI_Line(0		, 127	, 239	, 127	, 1);
		
		while (1)		//判断按键并刷新数据
		{
			//判断按键
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
				
				case KB_BKSP:		//回删一个数字
					NotReDraw(l_u8ControlCode);
					l_u32Value	/= 10;
					break;	
				
				case KB_F1:		//修改状态，开始输入数字
					if (l_u8Status == 0)
					{
						l_u8Status	= KB_F1;
						l_u32Value	= 0;
						memset(l_acTmp, 0, 30);
						sprintf(l_acTmp," 请输入新值(%u-%u):",p_u32Min,p_u32Max);
						LCDPRINTC(16		, 56	, l_acTmp);
					}
					NotReDraw(l_u8ControlCode);
					break;	
				
				case KB_ENTER:		//确认输入
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

			//是否重画
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;

			//刷新屏幕
			if (l_u8Status == KB_F1)		//输入状态
			{
				if (l_u32Value == 0)
					LCDPRINTC(56		, 80	, "         ");
				else
					LCDPRINTFC(56		, 80	, "%u      "	, l_u32Value);
			}

			WAITSCREENREFRESH();		//等待刷新信号
		}

		//是否退出
		if (IfBreak(l_u8ControlCode))
			break;
	}
}

#endif	//#if	YBVERSION >= 30		//3.0仪表功能

/*********************************************************************************************************
** Function name:		UISetTime
** Descriptions:		设定系统时间
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
	uint8	l_u8ControlCode;//控制指令
	uint32	l_u32Value;		//输入数字
	uint8	l_u8Status;		//功能键状态
	uint16	l_u16X;			//坐标X
	uint8	l_u8Key;

	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();

	while(1)
	{
		l_u32Value	= 0;
		l_u8Status	= 0;

		//画界面
		GUI_ClearSCR();
		LCDPRINTC(0		, 0		, "时间设定:    Esc退出, F1键修改");
		
		LCDPRINTC(16	, 20	, "当前时间:");
		LCDPRINTC(64	, 36	, "-");
		LCDPRINTC(88	, 36	, "-");
		LCDPRINTC(144	, 36	, ":");
		LCDPRINTC(168	, 36	, ":");
		
		GUI_Line(0		, 18	, 239	, 18	, 1);
		GUI_Line(0		, 125	, 239	, 125	, 1);
		GUI_Line(0		, 127	, 239	, 127	, 1);
			
		while (1)		//判断按键并刷新数据
		{
			//判断按键
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
				
				case KB_BKSP:		//回删一个数字
					NotReDraw(l_u8ControlCode);
					l_u32Value	/= 10;
					break;	
				
				case KB_F1:		//修改状态，开始输入数字
					if (l_u8Status == 0)
					{
						l_u8Status	= 1;
						l_u32Value	= 0;
						LCDPRINTC(16		, 60	,"设定时间:");
					}
					NotReDraw(l_u8ControlCode);
					break;	
				
				case KB_ENTER:		//确认输入
					NotReDraw(l_u8ControlCode);
					switch (l_u8Status)
					{
						case 1:		//年
							if (l_u32Value > 1999 && l_u32Value < 2100)
							{	
								TMPTIMEALIAS.u16Year	= l_u32Value;
								LCDPRINTFC(32		, 76	, "%04d"	, l_u32Value);
								LCDPRINTC(64		, 76	, "-");
							}
							else
								ToReDraw(l_u8ControlCode);
									
							break;
						
						case 2:		//月
							if (l_u32Value > 0 && l_u32Value < 13)
							{
								TMPTIMEALIAS.u8Month	= l_u32Value & 0xff;
								LCDPRINTFC(72		, 76	, "%02d"	, l_u32Value);
								LCDPRINTC(88		, 76	, "-");
			
							}
							else
								ToReDraw(l_u8ControlCode);
									
							break;

						case 3:		//日
							if (l_u32Value > 0 && l_u32Value < 32)
							{	
								TMPTIMEALIAS.u8Day	= l_u32Value & 0xff;
								LCDPRINTFC(96		, 76	, "%02d"	, l_u32Value);
							}
							else
								ToReDraw(l_u8ControlCode);
									
							break;
						
						case 4:		//时:0~23
							if (l_u32Value < 24)
							{	
								TMPTIMEALIAS.u8Hour	= l_u32Value & 0xff;
								LCDPRINTFC(128	, 76	, "%02d"	, l_u32Value);
								LCDPRINTC(144	, 76	, ":");
							}
							else
								ToReDraw(l_u8ControlCode);	
							break;
						
						case 5:		//分:0~59
							if (l_u32Value < 60)
							{	
								TMPTIMEALIAS.u8Minute	= l_u32Value & 0xff;
								LCDPRINTFC(152	, 76	, "%02d"	, l_u32Value);	
								LCDPRINTC(168	, 76	, ":");			
							}
							else
								ToReDraw(l_u8ControlCode);	
							break;
						
						case 6:		//秒:0~59
							if (l_u32Value < 60)
							{
								TMPTIMEALIAS.u8Second	= l_u32Value & 0xff;
								LCDPRINTFC(176	, 76	, "%02d"	, l_u32Value);
								LCDPRINTC(32		, 92	, "输入完毕. ");	
								LCDPRINTC(32		, 108	, "Enter:保存 Esc:取消");
							}
							else
								ToReDraw(l_u8ControlCode);	
							break;

						case 7:		//确认保存
							//保存
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

			//是否重画
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;

			//刷新屏幕
			LCDPRINTFC(32		, 36	, "%04d"	, CURTIMEALIAS.u16Year);
			LCDPRINTFC(72		, 36	, "%02d"	, CURTIMEALIAS.u8Month);
			LCDPRINTFC(96		, 36	, "%02d"	, CURTIMEALIAS.u8Day);

			LCDPRINTFC(128		, 36	, "%02d"	, CURTIMEALIAS.u8Hour);
			LCDPRINTFC(152		, 36	, "%02d"	, CURTIMEALIAS.u8Minute);	
			LCDPRINTFC(176		, 36	, "%02d"	, CURTIMEALIAS.u8Second);

			switch (l_u8Status)		//显示输入的数字
			{
				case 1:		//年
					l_u16X	= 32;		
					break;
				
				case 2:		//月
					l_u16X	= 72;		
					break;

				case 3:		//日
					l_u16X	= 96;		
					break;
				
				case 4:		//时:0~23
					l_u16X	= 128;		
					break;
				
				case 5:		//分:0~59
					l_u16X	= 152;		
					break;
				
				case 6:		//秒:0~59
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
			WAITSCREENREFRESH();		//等待刷新信号
		}

		//是否退出
		if (IfBreak(l_u8ControlCode))
			break;

	}
}

/*********************************************************************************************************
** Function name:		UIF5Code
** Descriptions:		F5代码界面
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
	uint8	l_u8ControlCode;//控制指令
	uint8	l_u8Tmp1;
	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();
	
	BackGroundSave();
	BackGroundON();		
	while(1)
	{
		//画界面
		GUI_ClearSCR();
		for(l_u8Tmp1 = 0; l_u8Tmp1 < 16; l_u8Tmp1++)
		{
			LCDPRINTFC(l_u8Tmp1*8+48	, 32	, "%d"	, (l_u8Tmp1+1)%10);
		}
	   	LCDPRINTC(0		, 0		, "胎型状态");
	   	LCDPRINTC(0		, 32	, "序号:");
	   	LCDPRINTC(0		, 48	, "状态:");
	   	LCDPRINTC(0		, 64	, "触发:");

		while (1)		//判断按键并刷新数据
		{
			//判断按键
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
					
			//是否重画
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;

			//刷新屏幕
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
			OSTimeDly(LOOPTIME*2);		//延时20ms，轮询时间
		}

		//是否退出
		if (IfBreak(l_u8ControlCode))
			break;
	}
	BackGroundRecover();
}

/*********************************************************************************************************
** Function name:		UIF4Code
** Descriptions:		查看F4信息界面
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
	//画界面
	GUI_ClearSCR();
	
	LCDPRINTC(0		, 0		, "F4代码:");
	
#if  SHOWVEHPASSDEBUG > 0	//显示过车调试代码
 	LCDPRINTC(0		, 16	, (char *)g_chVehPassDebug);
#else
	LCDPRINTC(0		, 16	, "F4代码功能未开启");
#endif
		
	while (KeyValue == 0xff)		//有按键
	{	
#if	SENDWAVEENABLE > 0		//使能发波形		
		LCDPRINTFC(0	, 97	, "READ:%5u "	, g_u16WeightBufReadIndex);
		LCDPRINTFC(120	, 97	, "SAVE:%5u "	, g_u16WeightBufSavIndex);
#endif	//#if	SENDWAVEENABLE > 0		//使能发波形	

		WAITSCREENREFRESH();		//等待刷新信号
	}
	ClearKeyValue();   
	BackGroundRecover();
}

/*********************************************************************************************************
** Function name:		UIF3Code
** Descriptions:		查看F3信息界面
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
	//画界面
	GUI_ClearSCR();
	
	LCDPRINTFC(0	, 0		, "版本号: %2dNT"	, YBVERALIAS);
 	LCDPRINTC(104	, 0		, VERSIONALIAS);
	
	//启动时间
	sprintf(m_acTmp, "启动时间: %.4d-%.2d-%.2d %.2d:%.2d:%.2d", g_sstStartTime.u16Year, g_sstStartTime.u8Month, g_sstStartTime.u8Day, g_sstStartTime.u8Hour, g_sstStartTime.u8Minute, g_sstStartTime.u8Second);
	LCDPRINTC(0		, 18	, (char *)m_acTmp);
	
	sprintf(m_acTmp, "启动次数: %-5d 波特率: %-5d ", g_u32StartupTime	, POINTRATE);
	LCDPRINTC(0		, 40	, (char *)m_acTmp);
								  
#if ISDEBUG				//调试状态
	LCDPRINTC(232	, 0		, "*");	

	
#if	SENDWAVEENABLE > 0		//使能发波形

#if	SENDWAVEBYSP > 0	
	LCDPRINTC(216	, 0		, "串");
#endif								  
#if	SENDWAVEBYNET > 0
	LCDPRINTC(200	, 0		, "网");
#endif

#endif	//#if	SENDWAVEENABLE > 0		//使能发波形		

#endif
	while (KeyValue == 0xff)		//有按键
	{
		WAITSCREENREFRESH();		//等待刷新信号
	}
	if (KeyValue == KB_F1)
		ClearStartupCnt();

	ClearKeyValue(); 
	BackGroundRecover();
}

/*********************************************************************************************************
** Function name:		UICommonSet
** Descriptions:		普通设置界面，111
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
	uint8	l_u8ControlCode;//控制指令
	
	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();

	while(1)
	{
		//画界面
		GUI_ClearSCR();
		LCDPRINTC(0		, 0		, " 系统设置");
		LCDPRINTC(112	, 0		, "0 保存  Esc 退出" );
		LCDPRINTC(8		, 24	, "1. 时钟设置");
		LCDPRINTC(8		, 40	, "2. 波特率设置");
		LCDPRINTC(8		, 56	, "3. 命令模式");
		LCDPRINTC(8		, 72	, "4. 方向使能");
		LCDPRINTC(8		, 88	, "5. 通讯协议");
		LCDPRINTC(8		, 104	, "F5 缓存车数");
		
		LCDPRINTC(128	, 24	, "6. 台面宽度");
		LCDPRINTC(128	, 40	, "7. 线圈抓拍");
		LCDPRINTC(128	, 56	, "8. 轮轴报错");
		LCDPRINTC(128	, 72	, "9. 抓拍使能");
		LCDPRINTC(128	, 88	, "F1 看门狗设置");
		LCDPRINTC(128	, 104	, "F4 掉电保护");

		GUI_Line(0		, 18	, 239	, 18	, 1);
		GUI_Line(0		, 124	, 239	, 124	, 1);
		GUI_Line(0		, 127	, 239	, 127	, 1);
		
		while (1)		//判断按键并刷新数据
		{
			//判断按键
			switch(KeyValue)
			{
				case KB_ESC:
					ToBreak(l_u8ControlCode);
					break;

				case KB_1:		//时钟设置
					ToReDraw(l_u8ControlCode);
					UISetTime();
					break;	
				
				case KB_2:		//波特率设置
					ToReDraw(l_u8ControlCode);
					UISetBaudRate();
					break;	
				
				case KB_3:		//命令模式
					ToReDraw(l_u8ControlCode);
					UISetCommandMode();
					break;	
				
				case KB_4:		//方向使能
					ToReDraw(l_u8ControlCode);
					UISetForwardEnable();
					break;	
				
				case KB_5:		//通讯协议
					ToReDraw(l_u8ControlCode);
					UISetProtocol();
					break;	
				
				case KB_6:		//台面宽度
					ToReDraw(l_u8ControlCode);
					UISetPlat();
					break;	
				
				case KB_7:		//线圈抓拍
					ToReDraw(l_u8ControlCode);
					UISetLoop();
					break;	
				
				case KB_8:		//更改口令
					ToReDraw(l_u8ControlCode);
					UISetLunZhouEnable();
					break;	
				
				case KB_9:		//抓拍使能
					ToReDraw(l_u8ControlCode);
					UISetCapture();
					break;	
				
				case KB_0:		//保存设置
					ToReDraw(l_u8ControlCode);
					SaveParams();
					break;	
				
				case KB_F1:		//看门狗
					ToReDraw(l_u8ControlCode);
					UISetDog();
					break;	
				
				case KB_F4:		//掉电保护
					ToReDraw(l_u8ControlCode);
					UISetDiaodian();
					break;	
				
				case KB_F5:		//缓存车数
					ToReDraw(l_u8ControlCode);
					UISetVehicleCache();
					break;	
				
				default:
					NotReDraw(l_u8ControlCode);
					break;
			}	//switch(KeyValue)
			ClearKeyValue();

			//是否重画
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;

			WAITSCREENREFRESH();		//等待刷新信号
		}

		//是否退出
		if (IfBreak(l_u8ControlCode))
			break;
	}
}

/*********************************************************************************************************
** Function name:		UIViewSetting
** Descriptions:		查看设置参数界面，222
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

	//画界面
	ReadC256(LASTBDADDR,(uint8 *)&l_ssTmp, 1024);	

	GUI_Line(0		, 18	, 239	, 18	, 1);
	GUI_Line(0		, 124	, 239	, 124	, 1);
	GUI_Line(0		, 127	, 239	, 127	, 1);
		
	LCDPRINTC(0		, 0		, VERSIONALIAS);
	LCDPRINTC(176	, 0		, "当前设置");
//	memset(m_acTmp	, 0		, 30); 
	sprintf(m_acTmp	, "零  位:%d+%d=%d",l_ssTmp.an32Zero[0],l_ssTmp.an32Zero[1],l_ssTmp.an32Zero[0]+l_ssTmp.an32Zero[1]);
	LCDPRINTC(0		, 20	, m_acTmp);
	
	LCDPRINTFC(0		, 36	, "增  益:%5d,"	, l_ssTmp.an32AxGain[0]);
	LCDPRINTFC(112		, 36	, "%d  "	, l_ssTmp.an32AxGain[1]);
		  
	LCDPRINTFC(0		, 52	, "分度值:%dkg"	, l_ssTmp.u8MotionScale);
	  
	LCDPRINTFC(0		, 68	, "满量程:%ukg"	, l_ssTmp.u32Full);
	LCDPRINTFC(0		, 84	, "跟  踪:%u"	, l_ssTmp.u8Genzong);

	sprintf(m_acTmp	, "修改日期:%04d-%02d-%02d",2000+l_ssTmp.u8Year,l_ssTmp.u8Month,l_ssTmp.u8Day);
	LCDPRINTC(0		, 104	, m_acTmp); 
 	 
	while (KeyValue == 0xff)		//有按键
	{
		WAITSCREENREFRESH();		//等待刷新信号
	}
	ClearKeyValue();

	memset((uint8 *)&l_ssTmp, 0, 1024);
	
	ReadC256(HISTORYBDADDR,(uint8 *)&l_ssTmp, 1024);
	GUI_ClearSCR();
	GUI_Line(0		, 18	, 239	, 18	, 1);
	GUI_Line(0		, 124	, 239	, 124	, 1);
	GUI_Line(0		, 127	, 239	, 127	, 1);
	
	LCDPRINTC(0		, 0		, VERSIONALIAS);
	LCDPRINTC(176	, 0		, "历史设置");

	sprintf(m_acTmp	, "零  位:%d+%d=%d",l_ssTmp.an32Zero[0],l_ssTmp.an32Zero[1],l_ssTmp.an32Zero[0]+l_ssTmp.an32Zero[1]);
	LCDPRINTC(0		, 20	, m_acTmp);
	
	LCDPRINTFC(0	, 36	, "增  益:%5d,"	, l_ssTmp.an32AxGain[0]);
	LCDPRINTFC(112	, 36	, "%5d  "	, l_ssTmp.an32AxGain[1]);
		  
	LCDPRINTFC(0	, 52	, "分度值:%dkg"	, l_ssTmp.u8MotionScale);
	  
	LCDPRINTFC(0	, 68	, "满量程:%ukg"	, l_ssTmp.u32Full);
	LCDPRINTFC(0	, 84	, "跟  踪:%u"	, l_ssTmp.u8Genzong);

//	memset(m_acTmp, 0, 30);
	sprintf(m_acTmp	, "修改日期:%04u-%02u-%02u",2000+l_ssTmp.u8Year,l_ssTmp.u8Month,l_ssTmp.u8Day);
	LCDPRINTC(0		, 104	, m_acTmp); 
 	
	while (KeyValue == 0xff)		//无按键
	{
		WAITSCREENREFRESH();		//等待刷新信号
	}
	ClearKeyValue();
}

/*********************************************************************************************************
** Function name:		UIViewModify
** Descriptions:		查看速度修正界面，333，（静态修正和速度修正）
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
	
	//画界面
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
			while (KeyValue == 0xff)		//有按键
			{
				WAITSCREENREFRESH();		//等待刷新信号
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
			while (KeyValue == 0xff)		//有按键
			{
				WAITSCREENREFRESH();		//等待刷新信号
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
** Descriptions:		阈值参数，8968
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
	uint8	l_u8ControlCode;//控制指令

	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();

	while(1)
	{
		//画界面
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
	
		LCDPRINTC(0		, 104	, "0.保存");
				
		while (1)		//判断按键并刷新数据
		{
			ToReDraw(l_u8ControlCode);
					
			//判断按键
			switch(KeyValue)
			{
				case KB_ESC:
				case KB_ENTER:
					ToBreak(l_u8ControlCode);
					break;
					
				case KB_1:		//上称
					UISetValueParamU16("UP", &(THRESHOLDALIAS.u16UpValue), 100, 500);
					break;

				case KB_2:		//下称
					UISetValueParamU16("DN", &(THRESHOLDALIAS.u16DownValue), 60, 300);
					break;	
					
				case KB_3:		//正向判别阈值
					ClearKeyValue();
					UISetValueParamU16("FD", &(THRESHOLDALIAS.u16ForwardWidth), 60, 500);
					break;	
					
				case KB_4:		//滤波级别
					UISetValueParamU16("RIF", &(THRESHOLDALIAS.u16FilterLevel), 3, 128);
					break;	
					
				case KB_5:		//波形最小宽度
					UISetValueParamU16("WT", &(THRESHOLDALIAS.u16AxleWidth), 50, 500);
					break;	
					
				case KB_0:		//保存
					SaveThreshold();
					break;
					
				default:
					NotReDraw(l_u8ControlCode);
					
					break;
			}	//switch(KeyValue)
			ClearKeyValue();

			//是否重画
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;

			WAITSCREENREFRESH();		//等待刷新信号
		}

		//是否退出
		if (IfBreak(l_u8ControlCode))
			break;
	}
}

/*********************************************************************************************************
** Function name:		UIBDScale
** Descriptions:		设置动静态分度界面
** input parameters:	p_u8Motion 动静态标志，0表示静态，1表示动态 
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
	uint8	l_u8ControlCode;//控制指令
	uint8 *	l_pu8Scale;
	switch (p_u8Motion)
	{
		case UI_STATIC:	 //静态
			l_pu8Scale = &SETUPALIAS.u8StaticScale;
			break;
			
		case UI_MOTION:	 //动态
			l_pu8Scale = &SETUPALIAS.u8MotionScale;
			break;

		default:
			return;
	}
	
	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();

	while(1)
	{
		//画界面
		GUI_ClearSCR();
		if (p_u8Motion == UI_MOTION)		//动态
			LCDPRINTC(0		, 0		, "动态分度设定         Esc-返回");
		else 				//静态
			LCDPRINTC(0		, 0		, "静态分度设定         Esc-返回");

		LCDPRINTC(0		, 19	, "请选择分度值:");
		
		LCDPRINTC(64	, 40	, "1--1   kg");
		
		LCDPRINTC(64	, 56	, "2--10  kg");
		LCDPRINTC(64	, 72	, "3--20  kg");
		LCDPRINTC(64	, 88	, "4--50  kg");
		LCDPRINTC(64	, 104	, "5--100 kg");
		
		GUI_Line(0		, 18	, 239	, 18	, 1);
		GUI_Line(0		, 124	, 239	, 124	, 1);
		GUI_Line(0		, 127	, 239	, 127	, 1);
	
		while (1)		//判断按键并刷新数据
		{
			//判断按键
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

			//是否重画
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;

			//刷新屏幕
			if (*l_pu8Scale == 0)
				*l_pu8Scale		= 50;

			LCDPRINTFC(112	, 19	, "%ukg    "	, *l_pu8Scale);

			WAITSCREENREFRESH();		//等待刷新信号
		}

		//是否退出
		if (IfBreak(l_u8ControlCode))
			break;
	}
}

/*********************************************************************************************************
** Function name:		UIBDStaticModify
** Descriptions:		设置静态修正界面，按照传感器重量修正
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
	uint8	l_u8ControlCode;//控制指令
	uint8	l_u8Status;		//状态
	uint8	l_u8Tmp1;		//临时变量
	uint8	l_u8Tmp2;
	uint8	l_u8Tmp3;

	ResetControlCode(l_u8ControlCode);
	l_u8Status	= 0;
	ClearKeyValue();

	while(1)
	{
		//画界面
		GUI_ClearSCR();
		l_u8Tmp2	= (l_u8Status >> 3)>0 ?	8 : 0;
		for (l_u8Tmp1 = l_u8Tmp2; l_u8Tmp1 <= l_u8Status; l_u8Tmp1 ++)
		{
			l_u8Tmp3	= (l_u8Tmp1 - l_u8Tmp2)<<4;
			LCDPRINTFC(0		, l_u8Tmp3	, "%d:", l_u8Tmp1);
			LCDPRINTFC(32		, l_u8Tmp3	, "0- %d"	, SETUPALIAS.au16StaticModify[0][l_u8Tmp1]);
			LCDPRINTFC(112		, l_u8Tmp3	, "1- %d"	, SETUPALIAS.au16StaticModify[1][l_u8Tmp1]);
		}
	
		while (1)		//判断按键并刷新数据
		{
			//判断按键
			switch(KeyValue)
			{
				case KB_ESC:
					ToBreak(l_u8ControlCode);
					break;
				
				case 0xf1:		//编辑
					sprintf(m_acTmp, "静态修正%dkm-0", l_u8Status);
					UISetValueParamU16(m_acTmp, &(SETUPALIAS.au16StaticModify[0][l_u8Status]), 9500, 10500);
					sprintf(m_acTmp, "静态修正%dkm-1", l_u8Status);
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

					//任何按键都显示一条新的数据
					l_u8Tmp2	= (l_u8Status&0x07) << 4;
					LCDPRINTFC(0		, l_u8Tmp2	, "%d:", l_u8Status);
					LCDPRINTFC(32		, l_u8Tmp2	, "0- %d"	, SETUPALIAS.au16StaticModify[0][l_u8Status]);
					LCDPRINTFC(112		, l_u8Tmp2	, "1- %d"	, SETUPALIAS.au16StaticModify[1][l_u8Status]);
					NotReDraw(l_u8ControlCode);
					break;
			}	//switch(KeyValue)
			ClearKeyValue();

			//是否重画
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;
			
			WAITSCREENREFRESH();		//等待刷新信号
		}

		//是否退出
		if (IfBreak(l_u8ControlCode))
			break;
	}
}

/*********************************************************************************************************
** Function name:		UIBDFullRange
** Descriptions:		设置最大量程 
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
	uint8	l_u8ControlCode;//控制指令

	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();

	while(1)
	{
		//画界面
		GUI_ClearSCR();
		LCDPRINTC(0		, 0		, "6. 量程设定           Esc-返回");
		
		LCDPRINTC(0		, 19	, "请选择最大量程:");

		LCDPRINTC(64	, 40	, "1--10000 kg");
		LCDPRINTC(64	, 56	, "2--15000 kg");
		LCDPRINTC(64	, 72	, "3--20000 kg");
		LCDPRINTC(64	, 88	, "4--30000 kg");
		LCDPRINTC(64	, 104	, "5--35000 kg");
		
		GUI_Line(0		, 18	, 239	, 18	, 1);
		GUI_Line(0		, 124	, 239	, 124	, 1);
		GUI_Line(0		, 127	, 239	, 127	, 1);

		while (1)		//判断按键并刷新数据
		{
			//判断按键
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

			//是否重画
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;

			//刷新屏幕
			if (SETUPALIAS.u32Full == 0)
				SETUPALIAS.u32Full	= 35000;
			
			LCDPRINTFC(128	, 19	, "%ukg      "	, SETUPALIAS.u32Full);

			WAITSCREENREFRESH();		//等待刷新信号
		}

		//是否退出
		if (IfBreak(l_u8ControlCode))
			break;
	}
}

/*********************************************************************************************************
** Function name:		UIBDChooseMotion
** Descriptions:		动态修正选择界面，点修or线修
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
	uint8	l_u8ControlCode;//控制指令
	
	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();

	while(1)
	{
		//画界面
		GUI_ClearSCR();
		LCDPRINTC(0		, 0		, " 动态修正");
		
		LCDPRINTC(176	, 0		, "Esc 退出");
		
		LCDPRINTC(8		, 24	, "1. 线修正");
		LCDPRINTC(8		, 44	, "2. 点修正");
	
		GUI_Line(0		, 18	, 239	, 18	, 1);
		GUI_Line(0		, 124	, 239	, 124	, 1);
		GUI_Line(0		, 127	, 239	, 127	, 1);

		while (1)		//判断按键并刷新数据
		{
			//判断按键
			switch(KeyValue)
			{
				case KB_ESC:
					ToBreak(l_u8ControlCode);
					break;

				case KB_1:		//线修正
					ToReDraw(l_u8ControlCode);
					UIBDLineModify();
					break;	
				
				case KB_2:		//点修正
					ToReDraw(l_u8ControlCode);
					UIBDPointModify();
					break;	
				
				default:
					NotReDraw(l_u8ControlCode);
					break;
			}	//switch(KeyValue)
			ClearKeyValue();

			//是否重画
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;

			WAITSCREENREFRESH();		//等待刷新信号
		}

		//是否退出
		if (IfBreak(l_u8ControlCode))
			break;
	}
}



/*********************************************************************************************************
** Function name:		UIBDLineModify
** Descriptions:		线性修正界面
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
	uint8	l_u8ControlCode;//控制指令
	uint8	l_u8Status;		//功能键状态
	uint8	l_u8Key;		//按键
	uint8	l_u8Tmp1;
	
	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();

	while(1)
	{
		l_u8Status	= 0;

		//画界面
		GUI_ClearSCR();
		LCDPRINTC(0		, 0		, "请输入速度段n: 1-8  (n-1)*4km");
		
		DrawLineModify(g_sspSetup.au16Speedmodify);
		
		while (1)		//判断按键并刷新数据
		{
			//判断按键
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
					if (l_u8Status == 0)		//显示对应速度段
					{
						l_u8Tmp1	= (l_u8Key-1)<<2;
					
//						memset(m_acTmp, 0, 30);
						sprintf(m_acTmp,"%dkm-%dkm 当前值: %d",l_u8Tmp1,l_u8Tmp1+3, SETUPALIAS.au16Speedmodify[l_u8Tmp1]);
						LCDPRINTC(0		, 16	, m_acTmp);
						
						LCDPRINTC(0,32,"Esc退出, F1键修改");
						l_u8Status	= l_u8Key;	
					}
					NotReDraw(l_u8ControlCode);
					break;	
				
				case KB_F1:		//修改状态，开始输入数字
					l_u8Tmp1	= (l_u8Status-1)<<2;
					if (l_u8Status != 0)
					{
//						memset(m_acTmp, 0, 30);
						sprintf(m_acTmp,"线修正速度%dkm",l_u8Tmp1);
						UISetValueParamU16(m_acTmp, &(SETUPALIAS.au16Speedmodify[l_u8Tmp1]), 8000, 12000);
					}
					if (l_u8Status>1)	//修正左边的点
					{
						SETUPALIAS.au16Speedmodify[l_u8Tmp1-2]	= (SETUPALIAS.au16Speedmodify[l_u8Tmp1] + SETUPALIAS.au16Speedmodify[l_u8Tmp1-4])>>1;
						SETUPALIAS.au16Speedmodify[l_u8Tmp1-1]	= (SETUPALIAS.au16Speedmodify[l_u8Tmp1] + SETUPALIAS.au16Speedmodify[l_u8Tmp1-2])>>1;
						SETUPALIAS.au16Speedmodify[l_u8Tmp1-3]	= (SETUPALIAS.au16Speedmodify[l_u8Tmp1-2] + SETUPALIAS.au16Speedmodify[l_u8Tmp1-4])>>1;
					}
					if (l_u8Status<8)	//修正右边的点
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

			//是否重画
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;

			WAITSCREENREFRESH();		//等待刷新信号
		}

		//是否退出
		if (IfBreak(l_u8ControlCode))
			break;
	}
}

/*********************************************************************************************************
** Function name:		UIBDPointModify
** Descriptions:		点修正界面
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
	uint8	l_u8ControlCode;//控制指令
	uint8	l_u8Key;		//按键
	uint32	l_u32Value;
	
	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();
	l_u32Value	= 0;
	while(1)
	{	
		//画界面
		GUI_ClearSCR();
		LCDPRINTC(0		, 0		, "请输入速度 1-31:");
		
		while (1)		//判断按键并刷新数据
		{
			//判断按键
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
				
				case KB_BKSP:		//回删一个数字
					NotReDraw(l_u8ControlCode);
					l_u32Value	/= 10;
					break;	
				
				case KB_ENTER:		//确认输入
					if ((l_u32Value >0 && l_u32Value < 32))
					{
						sprintf(m_acTmp,"点修正 %d km:",l_u32Value);
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

			//是否重画
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;
			
			//刷新屏幕
			if (l_u32Value == 0)
				LCDPRINTC(144	, 0		, "         ");
			else
				LCDPRINTFC(144	, 0		, "%u    "	, l_u32Value);
			
			WAITSCREENREFRESH();		//等待刷新信号
		}

		//是否退出
		if (IfBreak(l_u8ControlCode))
			break;
	}
}


/*********************************************************************************************************
** Function name:		UIBDChooseVehPos
** Descriptions:		车型速度修正位置选择
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
	uint8	l_u8ControlCode;//控制指令
	
	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();

	while(1)
	{
		//画界面
		GUI_ClearSCR();
		LCDPRINTC(0		, 0		, " 车型修正");
		
		LCDPRINTC(176	, 0		, "Esc 退出");
		LCDPRINTC(8		, 24	, "1. 车型整体修正");
#if	IF3WB		
		LCDPRINTC(8		, 40	, "2. AB板车型速度点修正");
		LCDPRINTC(8		, 56	, "3. BC板车型速度点修正");	 
		LCDPRINTC(8		, 72	, "4. 车型压缝修正");
		LCDPRINTC(8		, 88	, "5. 压缝车型速度点修正");
#else
		LCDPRINTC(8		, 40	, "2. 车型速度点修正");
#endif
		
		GUI_Line(0		, 18	, 239	, 18	, 1);
		GUI_Line(0		, 124	, 239	, 124	, 1);
		GUI_Line(0		, 127	, 239	, 127	, 1);

		while (1)		//判断按键并刷新数据
		{
			//判断按键
			switch(KeyValue)
			{
				case KB_ESC:
					ToBreak(l_u8ControlCode);
					break;
				
				case KB_1:		//车型整体修正
					ToReDraw(l_u8ControlCode);
					UIBDChooseVeh(POS_VEH);
					break;	
				
				case KB_2:		//AB板车型速度修正
					ToReDraw(l_u8ControlCode);
#if	IF3WB
					UIBDChooseVeh(POS_ABS);
#else
					UIBDChooseVeh(POS_2WB);
#endif
					break;
						
#if	IF3WB			
				case KB_3:		//BC板车型速度修正
					ToReDraw(l_u8ControlCode);
					UIBDChooseVeh(POS_BCS);
					break;	
				
				case KB_4:		//压缝车型速度修正
					ToReDraw(l_u8ControlCode);
					UIBDChooseVeh(POS_ABCS);
					break;	
				
				case KB_5:		//车型压缝修正
					ToReDraw(l_u8ControlCode);
					UIBDChooseVeh(POS_GAP);
					break;	
#endif				
				default:
					NotReDraw(l_u8ControlCode);
					break;
			}	//switch(KeyValue)
			ClearKeyValue();

			//是否重画
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;

			WAITSCREENREFRESH();		//等待刷新信号
		}

		//是否退出
		if (IfBreak(l_u8ControlCode))
			break;
	}
}

/*********************************************************************************************************
** Function name:		UIBDChooseVeh
** Descriptions:		选择车型界面，根据压缝位置不同，查看的数据不同
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
	uint8	l_u8ControlCode;//控制指令
	uint8	l_u8Key;
	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();

	while(1)
	{
		//画界面
		GUI_ClearSCR();
		switch(p_u8Pos)
		{
#if	IF3WB
		case POS_ABS:	//AB
			LCDPRINTC(0		, 0		, "AB板行驶车型速度点修正");
			break;

		case POS_BCS:	//BC
			LCDPRINTC(0		, 0		, "BC板行驶车型速度点修正");
			break;

		case POS_ABCS:	//压缝
			LCDPRINTC(0		, 0		, "压缝行驶车型速度点修正");
			break;
		
		case POS_GAP:	//压缝整体
			LCDPRINTC(0		, 0		, "车型压缝增益修正");
			break;
#endif
		case POS_VEH:	//车型整体
			LCDPRINTC(0		, 0		, "车型整体修正");
			break;

		case POS_2WB:	//两弯板
			LCDPRINTC(0		, 0		, "车型速度修正");
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
		//LCDPRINTC(124	, 84	, "9.车型修正");
		LCDPRINTC(124	, 104	, "Esc. 退出设置 ");
	
		GUI_Line(0		, 18	, 239	, 18	, 1);
		GUI_Line(0		, 124	, 239	, 124	, 1);
		GUI_Line(0		, 127	, 239	, 127	, 1);

		while (1)		//判断按键并刷新数据
		{
			//判断按键
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
					case POS_ABCS:	//压缝
#endif
					case POS_2WB:	//两弯板
						UIBDVSModifyParam(p_u8Pos, l_u8Key);
						break;
			
					case POS_VEH:	//车型整体
						sprintf(m_acTmp, "车型整体-车型%d",l_u8Key);  
						UISetValueParamU16(&m_acTmp[0], &(SETUPALIAS.au16VehTotalModify[l_u8Key-1]), 8000, 12000);
						break;
#if	IF3WB			
					case POS_GAP:	//压缝整体
						sprintf(m_acTmp, "压缝整体-车型%d",l_u8Key);  
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
					
			//是否重画
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;

			WAITSCREENREFRESH();		//等待刷新信号
		}

		//是否退出
		if (IfBreak(l_u8ControlCode))
			break;
	}
}

/*********************************************************************************************************
** Function name:		UIBDVSModifyParam
** Descriptions:		参数值界面，根据压缝位置和车型
** input parameters:	p_u8Pos		位置
**						p_u8Veh		车型 
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
	uint8	l_u8ControlCode;		//控制指令
	uint32	l_u32Value	= 0;		//输入数字
	uint8	l_u8Key;
	uint8	l_u8Status;
	ResetControlCode(l_u8ControlCode);
	ClearKeyValue();
	
	while(1)
	{
		//画界面
		GUI_ClearSCR();
		switch(p_u8Pos)
		{
#if	IF3WB
		case POS_ABS:	//AB
			LCDPRINTFC(0		, 0		, "AB板车型速度修正-车型%d"	, p_u8Veh);
			break;

		case POS_BCS:	//BC
			LCDPRINTFC(0		, 0		, "BC板车型速度修正-车型%d"	, p_u8Veh);
			break;

		case POS_ABCS:	//压缝
			LCDPRINTFC(0		, 0		, "压缝车型速度修正-车型%d"	, p_u8Veh);
			break;
#endif
		case POS_2WB:	//两弯板
			LCDPRINTFC(0		, 0		, "车型速度修正-车型%d"		, p_u8Veh);
			break;

		default:
			ToBreak(l_u8ControlCode);
			break;
		}		
#if	IF3WB
		LCDPRINTC(0		, 32	, "请输入速度(0-19): ");//20公里以内，编辑对应参数
#else
		LCDPRINTC(0		, 32	, "请输入速度(0-31): ");//32公里以内，编辑对应参数
#endif
		
		GUI_Line(0		, 18	, 239	, 18	, 1);
		GUI_Line(0		, 124	, 239	, 124	, 1);
		GUI_Line(0		, 127	, 239	, 127	, 1);
		l_u8Status	= 0xff;
		while (1)		//判断按键并刷新数据
		{
			//判断按键
			//判断按键
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
				
				case KB_BKSP:		//回删一个数字
					NotReDraw(l_u8ControlCode);
					l_u32Value	/= 10;
					if (l_u32Value == 0)
						l_u8Status	= 0xff;
					break;	
				
				case KB_ENTER:		//确认输入
					if (l_u8Status != 0xff)
					{
#if	IF3WB
						if (l_u32Value < 20)	//20公里以内，编辑对应参数
#else
						if (l_u32Value < 32)	//32公里以内，编辑对应参数
#endif
						{
							switch(p_u8Pos)
							{
#if	IF3WB		
							case POS_ABS:	//AB
//								memset(m_acTmp, 0, 30);
								sprintf(m_acTmp, "AB-车型%d-速度%dkm",p_u8Veh,l_u32Value);  
								UISetValueParamU16(m_acTmp, &(SETUPALIAS.au16VehSpeedModify[p_u8Veh-1][l_u32Value]), (VSMSTANDARD*8)/10, (VSMSTANDARD*12)/10);
								break;
					
							case POS_BCS:	//BC
//								memset(m_acTmp, 0, 30);
								sprintf(m_acTmp, "BC-车型%d-速度%dkm",p_u8Veh,l_u32Value);  
								UISetValueParamU16(m_acTmp, &(SETUPALIAS.au16VehSpeedModify[p_u8Veh-1][l_u32Value]), (VSMSTANDARD*8)/10, (VSMSTANDARD*12)/10);
								break;
					
							case POS_ABCS:	//压缝
//								memset(m_acTmp, 0, 30);
								sprintf(m_acTmp, "压缝-车型%d-速度%dkm",p_u8Veh,l_u32Value);  
								UISetValueParamU16(m_acTmp, &(SETUPALIAS.au16VehSpeedModify[p_u8Veh-1][l_u32Value]), (VSMSTANDARD*8)/10, (VSMSTANDARD*12)/10);
								break;
#else				
							case POS_2WB:	//2弯板
//								memset(m_acTmp, 0, 30);
								sprintf(m_acTmp, "车型%d-速度%dkm",p_u8Veh,l_u32Value);  
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
					
			//是否重画
			if (IfReDraw(l_u8ControlCode)||IfBreak(l_u8ControlCode))
				break;

			//刷新屏幕
			if (l_u8Status == 0xff)
				LCDPRINTC(144	, 32	, "         ");
			else
				LCDPRINTFC(144	, 32	, "%u       "	, l_u32Value);
			
			WAITSCREENREFRESH();		//等待刷新信号
		}

		//是否退出
		if (IfBreak(l_u8ControlCode))
			break;
	}
}
