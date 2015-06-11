/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			UIOperation.c
** Last modified Date:  20110507
** Last Version:		1.0
** Descriptions:		所有界面需要调用的操作，如初始化，保存参数等
**
**--------------------------------------------------------------------------------------------------------
** Created by:			ZHANG Ye
** Created date:		20110507
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
#define	__UIOPERATION_C
#include "UIOperation.h"
#include "norflash.h"

//全局变量别名
#define		SETUPALIAS				g_sspSetup			//设置参数结构体
#define		THRESHOLDALIAS			g_sudtThreshold		//阈值参数
#define		SNALIAS					g_u32SN				//SN值
#define		CRCFunc					CheckCrc			//CRC校验函数名	 
#define		CRCFunc16				AddCrc16			//CRC校验函数名,带赋值

#define		UWValue					400	//UPWEIGHT				//设置参数结构体
#define		DWValue					300	//DNWEIGH				//设置参数结构体

#if	YBVERSION >= 30		//3.0仪表功能
#define		IPINFOALIAS				g_sniLocal			//网络参数
#define		RamToFlash()			SaveRamToFlash((uint8 *)&SETUPALIAS)
#endif

/*********************************************************************************************************
** Function name:		SaveParams
** Descriptions:		保存参数设置
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
void SaveParams(void)
{
	uint8 l_au8Tmp[1024];
	
#if	YBVERSION >= 30		//3.0仪表功能
	SETUPALIAS.u8Year		= g_sstCurTime.u16Year-2000;	// 取得年值
	SETUPALIAS.u8Month		= g_sstCurTime.u8Month;			// 取得月值
	SETUPALIAS.u8Day		= g_sstCurTime.u8Day;			// 取得日值
#else	//2.2		
	SETUPALIAS.u8Year		= ((CTIME1>>16) & 0xFFF);		// 取得年值
	SETUPALIAS.u8Month		=  (CTIME1>>8)  & 0x0F;			// 取得月值
	SETUPALIAS.u8Day		=   CTIME1      & 0x1F;			// 取得日值
#endif	//#if	YBVERSION >= 30		//3.0仪表功能
	  	
	ReadC256(LASTBDADDR, l_au8Tmp, 1024);	// 在0x00地址处读出320字节数据
	WriteC256(HISTORYBDADDR, l_au8Tmp, 1024);	// 在0x00地址处写入32字节数据
	
	CRCFunc16((uint8 *)&SETUPALIAS,1022);
	RamToFlash();

	WriteC256(LASTBDADDR,(uint8 *)&SETUPALIAS,1024);	// 在0x00地址处写入32字节数据
}

/*********************************************************************************************************
** Function name:		InitVehBufIndex
** Descriptions:		保存参数设置
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
void InitVehBufIndex(void)
{
	g_u8CurVehNum	= 0;
	g_u8IndexLast	= 0;
	g_u8IndexFirst	= 0;

	SaveVehBuf();
}

#if	YBVERSION >= 30		//3.0仪表功能
/*********************************************************************************************************
** Function name:           ReadPassVehInfo
**
** Descriptions:            读取过车数据
**
** input parameters:        VehicleRecord * p_pvrPassInfo	:带保存的过车数据指针
** output parameters:       uint16		读取的位置
** Returned value:          none
**
** Created by:              ZHANG Ye
** Created Date:            20110704
**--------------------------------------------------------------------------------------------------------
** Modified by:
** Modified date:
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void ReadPassVehInfo(VehicleRecord * p_pvrPassInfo, uint16 p_u16Addr)
{
	//保存信息至Flash
	ReadNORFlash(VEHBUFREC + (p_u16Addr<<7), sizeof(VehicleRecord), (uint8 *)p_pvrPassInfo);

}

/*********************************************************************************************************
** Function name:           SavePassVehInfo
**
** Descriptions:            保存过车数据
**
** input parameters:        VehicleRecord * p_pvrPassInfo	:带保存的过车数据指针
** output parameters:       none
** Returned value:          uint16		写入的位置
**
** Created by:              ZHANG Ye
** Created Date:            20110701
**--------------------------------------------------------------------------------------------------------
** Modified by:
** Modified date:
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
uint16 SavePassVehInfo(VehicleRecord * p_pvrPassInfo)
{
	uint16	l_u16RecPos;
	uint16	l_u16Result;

	//读取记录位置
	ReadC256(NEXTRECBUFADDR, (uint8 *)&l_u16RecPos, 2);
	
	if (l_u16RecPos % 32 == 0)
		EraseSector(VEHBUFREC + (l_u16RecPos<<7));

	//保存信息至Flash
	WriteNORFlash(VEHBUFREC + (l_u16RecPos<<7), sizeof(VehicleRecord), (uint8 *)p_pvrPassInfo);

	//更新记录位置
	l_u16Result	= l_u16RecPos ++;
	l_u16RecPos	%= 2048;
	
	WriteC256(NEXTRECBUFADDR, (uint8 *)&l_u16RecPos, 2);

	return l_u16Result;
}
#endif	//#if	YBVERSION >= 30		//3.0仪表功能
/*********************************************************************************************************
** Function name:		SaveVehBuf
** Descriptions:		保存车辆缓存信息
** input parameters:	None 
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110531	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void SaveVehBuf(void)
{
	WriteC256(VEHBUFINDEXADDR		, &g_u8CurVehNum	, 1);
	WriteC256(VEHBUFINDEXADDR + 1	, &g_u8IndexLast	, 1);
	WriteC256(VEHBUFINDEXADDR + 2	, &g_u8IndexFirst	, 1);
}

/*********************************************************************************************************
** Function name:		ClearStartupCnt
** Descriptions:		启动次数清零
** input parameters:	None 
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110517	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void ClearStartupCnt(void)
{
	g_u32StartupTime	= 0;
	WriteC256(STARTTIMESADDR,(uint8 *)&g_u32StartupTime, 4);
}

/*********************************************************************************************************
** Function name:		AddStartupCnt
** Descriptions:		启动次数累加
** input parameters:	None 
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110517	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void AddStartupCnt(void)
{
#if	YBVERSION >= 30		//3.0仪表功能
	uint16	l_u16RecIndex;
	uint8	l_au8RecordData[8];

	//加1
	ReadC256(STARTTIMESADDR		, (uint8 *)&g_u32StartupTime	, 4);
	g_u32StartupTime	++;
	WriteC256(STARTTIMESADDR	, (uint8 *)&g_u32StartupTime	, 4);

	//记录时间
	ReadC256(STARTRECINDEXADDR,(uint8 *)&l_u16RecIndex,2);		//找到有效记录号
	l_u16RecIndex	+= 1;
	l_u16RecIndex	&= 0x3ff;		//0~1023
	WriteC256(STARTRECINDEXADDR,(uint8 *)&l_u16RecIndex,2);		//更新记录号
	
	if ((l_u16RecIndex & 0x1ff) == 0)		//写满一个Sector
	{
		EraseSector(NORSTARTREC	+ (l_u16RecIndex << 3));
	}
	
	//生成记录
	l_au8RecordData[0]	= g_sstCurTime.u16Year & 0xff;	//年
	l_au8RecordData[1]	= g_sstCurTime.u16Year >> 8;
	l_au8RecordData[2]	= g_sstCurTime.u8Month;			//月
	l_au8RecordData[3]	= g_sstCurTime.u8Day;			//日
	l_au8RecordData[4]	= g_sstCurTime.u8Hour;			//时
	l_au8RecordData[5]	= g_sstCurTime.u8Minute;		//分
	l_au8RecordData[6]	= g_sstCurTime.u8Second;		//秒
	l_au8RecordData[7]	= 0xaa;
	WriteNORFlash(NORSTARTREC + (l_u16RecIndex<<3), 8, &l_au8RecordData[0]);	

#else	//2.2
	ReadC256(STARTTIMESADDR	, (uint8 *)&g_u32StartupTime	, 4);
	g_u32StartupTime	++;
	WriteC256(STARTTIMESADDR,(uint8 *)&g_u32StartupTime, 4);
#endif	//#if	YBVERSION >= 30		//3.0仪表功能
}

/*********************************************************************************************************
** Function name:		SaveTime
** Descriptions:		保存时间设置
** input parameters:	None 
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110517	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void SaveTime(SystemTime p_stSet)
{
	SetRTCTime(&p_stSet);
}

/*********************************************************************************************************
** Function name:		InitRestart
** Descriptions:		初始化后重启
** input parameters:	None 
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110516	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void InitRestart(void)
{
	InitVehBufIndex();

	//重启
#if	YBVERSION >= 30		//3.0仪表功能
#if NOTDEBUG				//非调试状态
	WDTIM_COUNTER	= 0x100000;									/* 喂狗							*/
#endif
#else	//2.2
	T1TCR	= 0;
#endif	//#if	YBVERSION >= 30		//3.0仪表功能
}

/*********************************************************************************************************
** Function name:		InitThreshold
** Descriptions:		初始化阈值参数
** input parameters:	None 
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110513	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void InitThreshold(void)
{
	THRESHOLDALIAS.u16UpValue		= UWValue;
	THRESHOLDALIAS.u16DownValue		= DWValue;
	THRESHOLDALIAS.u16ForwardWidth	= 200;
    THRESHOLDALIAS.u16FilterLevel	= 8;
	THRESHOLDALIAS.u16AxleWidth		= 100;
	THRESHOLDALIAS.au16Reserved[0]	= 0;
	THRESHOLDALIAS.au16Reserved[1]	= 0;
	
	CRCFunc16((uint8 *)&THRESHOLDALIAS, sizeof(THRESHOLDALIAS)-2);
	WriteC256(THRESHOLDADDR,(uint8 *)&THRESHOLDALIAS, sizeof(THRESHOLDALIAS));
}

/*********************************************************************************************************
** Function name:		SaveThreshold
** Descriptions:		保存阈值参数
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
void SaveThreshold(void)
{
	CRCFunc16((uint8 *)&THRESHOLDALIAS, sizeof(THRESHOLDALIAS)-2);
	WriteC256(THRESHOLDADDR,(uint8 *)&THRESHOLDALIAS, sizeof(THRESHOLDALIAS));
}

#if	YBVERSION >= 30		//3.0仪表功能
/*********************************************************************************************************
** Function name:		InitNetParam
** Descriptions:		初始化网络参数
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
void InitNetParam(void)
{
	IPINFOALIAS.au8SubMask[0]		= 0xff;
	IPINFOALIAS.au8SubMask[1]		= 0xff;
	IPINFOALIAS.au8SubMask[2]		= 0xff;
	IPINFOALIAS.au8SubMask[3]		= 0x00;
	
	IPINFOALIAS.au8GatewayIP[0]		= 192;
	IPINFOALIAS.au8GatewayIP[1]		= 168;
	IPINFOALIAS.au8GatewayIP[2]		= 0;
	IPINFOALIAS.au8GatewayIP[3]		= 1;   

	IPINFOALIAS.au8IPAddr[0]		= 192;
	IPINFOALIAS.au8IPAddr[1]		= 168;
	IPINFOALIAS.au8IPAddr[2]		= 0;
	IPINFOALIAS.au8IPAddr[3]		= 113;

	IPINFOALIAS.u32LocalPortNO		= 5000;	 

	IPINFOALIAS.au8ServerIP[0]		= 192;
	IPINFOALIAS.au8ServerIP[1]		= 168;
	IPINFOALIAS.au8ServerIP[2]		= 0;
	IPINFOALIAS.au8ServerIP[3]		= 111;
								   
	IPINFOALIAS.u32ServerPortNO		= 4000;
	
	IPINFOALIAS.au8MACAddr[0]		= 0x52;
	IPINFOALIAS.au8MACAddr[1]		= 0x54;
	IPINFOALIAS.au8MACAddr[2]		= 0x4c;
	IPINFOALIAS.au8MACAddr[3]		= 0x19;
	IPINFOALIAS.au8MACAddr[4]		= 0xf7;
	IPINFOALIAS.au8MACAddr[5]		= 0x48;
	
	SaveNetInfo();
}

/*********************************************************************************************************
** Function name:		SaveNetInfo
** Descriptions:		保存网络参数
** input parameters:	None 
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110624	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void SaveNetInfo(void)
{
	CRCFunc16((uint8 *)&IPINFOALIAS, sizeof(IPINFOALIAS)-2);
	WriteC256(NETINFOADDR,(uint8 *)&IPINFOALIAS, sizeof(IPINFOALIAS));
}
#endif	//#if	YBVERSION >= 30		//3.0仪表功能

/*********************************************************************************************************
** Function name:		InitSystem
** Descriptions:		系统初始化
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
void InitSystem(void)
{
	uint16		l_u16Tmp;
	uint16 *	l_pu16Tmp;

	for(l_u16Tmp = 0; l_u16Tmp < 16; l_u16Tmp++)
	{
		SETUPALIAS.au16StaticModify[0][l_u16Tmp]	= 10000;
		SETUPALIAS.au16StaticModify[1][l_u16Tmp]	= 10000; 
		SETUPALIAS.au16StaticModify[2][l_u16Tmp]	= 10000;
		SETUPALIAS.au16StaticModify[3][l_u16Tmp]	= 10000;
	}
	for(l_u16Tmp = 0; l_u16Tmp < 32; l_u16Tmp++)
	{
	  SETUPALIAS.au16Speedmodify[l_u16Tmp]	= 10000;
	}

	SETUPALIAS.an32Zero[0]			= 0;
	SETUPALIAS.an32Zero[1]			= 0;
	SETUPALIAS.an32Zero[2]			= 0;
	SETUPALIAS.an32Zero[3]			= 0;
	SETUPALIAS.an32AxGain[0]		= 10000;
	SETUPALIAS.an32AxGain[1]		= 10000;
	SETUPALIAS.an32AxGain[2]		= 10000;
	SETUPALIAS.an32AxGain[3]		= 10000;

	SETUPALIAS.u32Password			= SUPERPWD;
	SETUPALIAS.u32Full				= 35000;
	SETUPALIAS.u16Podu				= 10000;
	SETUPALIAS.u8StaticScale		= 20;
	SETUPALIAS.u8MotionScale		= 20;
	SETUPALIAS.u8Genzong			= 1;
	SETUPALIAS.u8ComMode			= 1;
	SETUPALIAS.u8BaudRate			= 2;
	SETUPALIAS.u8DOG				= 1; 
	SETUPALIAS.u8Year				= 0;
	SETUPALIAS.u8Month				= 0;
	SETUPALIAS.u8Day				= 0;
	SETUPALIAS.u8PlatWidth			= 62;
	
	SETUPALIAS.u8FangxiangEnable	= 1;
	SETUPALIAS.u8DiaoDianTag		= 0;
	SETUPALIAS.u8ZhuapaiEnable		= 0;
	SETUPALIAS.u8LoopTriggerEnable	= 0;
	SETUPALIAS.u8Protocol			= 1;
	SETUPALIAS.u8VehicleBufSize		= 10;
	
	l_pu16Tmp	= &(SETUPALIAS.au16VehTotalModify[0]);
	for (l_u16Tmp = 0; l_u16Tmp < 10; l_u16Tmp ++)
		*(l_pu16Tmp++)	= 10000;

	l_pu16Tmp	= &(SETUPALIAS.au16VehSpeedModify[0][0]);
	for (l_u16Tmp = 0; l_u16Tmp < 320; l_u16Tmp ++)
		*(l_pu16Tmp++)	= VSMSTANDARD;

	SETUPALIAS.u8LunZhouERR			= 0x03;
	SETUPALIAS.u8SendWaveEnable		= 0;
	
	CRCFunc16((uint8 *)&SETUPALIAS, 1022);
#if	YBVERSION >= 30		//3.0仪表功能
	l_u16Tmp	= 0;
	WriteC256(NEXTRECBUFADDR,(uint8 *)&l_u16Tmp, 2);
	InitNetParam();
	ClearStartupCnt();
#endif	//#if	YBVERSION >= 30		//3.0仪表功能

	RamToFlash();
	InitThreshold();

}

/*********************************************************************************************************
** Function name:		InitNonWeight
** Descriptions:		初始化非称重设置
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
void InitNonWeight(void)
{
	SETUPALIAS.u32Password			= SUPERPWD;
	SETUPALIAS.u32Full				= 35000;
	SETUPALIAS.u16Podu				= 10000;
	SETUPALIAS.u8StaticScale		= 20;
	SETUPALIAS.u8MotionScale		= 20;
	SETUPALIAS.u8Genzong			= 1;
	SETUPALIAS.u8ComMode			= 1;
	SETUPALIAS.u8BaudRate			= 2;
	SETUPALIAS.u8DOG				= 0; 
	SETUPALIAS.u8Year				= 0;
	SETUPALIAS.u8Month				= 0;
	SETUPALIAS.u8Day				= 0;
	SETUPALIAS.u8PlatWidth			= 75;
	
	SETUPALIAS.u8FangxiangEnable	= 1;
	SETUPALIAS.u8DiaoDianTag		= 0;
	SETUPALIAS.u8ZhuapaiEnable		= 0;
	SETUPALIAS.u8LoopTriggerEnable	= 0;
	SETUPALIAS.u8Protocol			= 1;
	SETUPALIAS.u8VehicleBufSize		= 10;

	SETUPALIAS.u8LunZhouERR			= 0x03;
	SETUPALIAS.u8SendWaveEnable		= 0;
	
	CRCFunc16((uint8 *)&SETUPALIAS, 1022);

	RamToFlash();
	
	InitThreshold();
}

/*********************************************************************************************************
** Function name:		InitMotionModify
** Descriptions:		初始化动态修正参数
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
void InitMotionModify(void)
{
	uint16		l_u16Tmp;

	for(l_u16Tmp = 0; l_u16Tmp < 32; l_u16Tmp++)
	{
	  SETUPALIAS.au16Speedmodify[l_u16Tmp]	= 10000;
	}
	
	CRCFunc16((uint8 *)&SETUPALIAS, 1022);

	RamToFlash();
	
}

/*********************************************************************************************************
** Function name:		InitVehModify
** Descriptions:		初始化车型修正参数
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
void InitVehModify(void)
{
	uint16		l_u16Tmp;
	uint16 *	l_pu16Tmp;

	l_pu16Tmp	= &(SETUPALIAS.au16VehTotalModify[0]);
	for (l_u16Tmp = 0; l_u16Tmp < 10; l_u16Tmp ++)
		*(l_pu16Tmp++)	= 10000;

	l_pu16Tmp	= &(SETUPALIAS.au16VehSpeedModify[0][0]);
	for (l_u16Tmp = 0; l_u16Tmp < 320; l_u16Tmp ++)
		*(l_pu16Tmp++)	= 10000;

	CRCFunc16((uint8 *)&SETUPALIAS, 1022);

	RamToFlash();

}

/*********************************************************************************************************
** Function name:		InitStaticModify
** Descriptions:		初始化静态修正参数
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
void InitStaticModify(void)
{
	uint16		l_u16Tmp;

	for(l_u16Tmp = 0; l_u16Tmp < 16; l_u16Tmp++)
	{
		SETUPALIAS.au16StaticModify[0][l_u16Tmp]	= 10000;
		SETUPALIAS.au16StaticModify[1][l_u16Tmp]	= 10000; 
#if CHANNELNUM > 2
		SETUPALIAS.au16StaticModify[2][l_u16Tmp]	= 10000;
		SETUPALIAS.au16StaticModify[3][l_u16Tmp]	= 10000;
#endif
	}
	
	CRCFunc16((uint8 *)&SETUPALIAS, 1022);

	RamToFlash();
}

/*********************************************************************************************************
** Function name:		RecoverToLast
** Descriptions:		恢复为最近标定参数
** input parameters:	None 
** output parameters:	none
** Return Value:		uint8	是否成功：1，成功     0，失败
** Created by:			ZHANG Ye		  
** Created Date:		20110510	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
uint8 RecoverToLast(void)
{
	ReadC256(LASTBDADDR,(uint8 *)&SETUPALIAS,1024);	
	if(CRCFunc((uint8 *)&SETUPALIAS,1022)==0)		//失败
	{
		return	0;
	}
	else	//成功
	{
		RamToFlash();
		return	1;
	}
}

/*********************************************************************************************************
** Function name:		RecoverToLast
** Descriptions:		恢复为历史标定参数
** input parameters:	None 
** output parameters:	none
** Return Value:		uint8	是否成功：1，成功     0，失败
** Created by:			ZHANG Ye		  
** Created Date:		20110510	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
uint8 RecoverToHistory(void)
{
	ReadC256(HISTORYBDADDR,(uint8 *)&SETUPALIAS,1024);	
	if(CRCFunc((uint8 *)&SETUPALIAS,1022)==0)		//失败
	{
		return	0;
	}
	else	//成功
	{
		RamToFlash();
		return	1;
	}
}
