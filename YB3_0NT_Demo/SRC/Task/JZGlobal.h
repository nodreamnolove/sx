/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			JZGlobal.h
** Last modified Date:  2011511
** Last Version:		1.0
** Descriptions:		计重程序全局变量
**
**--------------------------------------------------------------------------------------------------------
** Created by:			ZHANG Ye
** Created date:		2011511
** Version:				1.0
** Descriptions:		
**
**--------------------------------------------------------------------------------------------------------
** Modified by:			Wang ZiFeng
** Modified date:		2013-03-11
** Version:				2.0
** Descriptions:
**
*********************************************************************************************************/
#ifndef	__JZGLOBAL_H
#define	__JZGLOBAL_H


#ifdef	__JZGLOBAL
#define	JZG_EXT	
#else
#define	JZG_EXT	extern
#endif
														
 
#define		NOTDEBUG			(DEBUG_NT == 0)	//不是调试

#include	"JZStructure.h"

JZG_EXT uint16 limit_height;
JZG_EXT uint16 limit_length;
JZG_EXT uint16 limit_width;
JZG_EXT	uint8  if_send_flag;	//用于判断是否将车辆信息通过DTU传输
JZG_EXT	uint8 sv_frame_data[512];
JZG_EXT	uint8 sv_count;
JZG_EXT	uint16 IF_SAME_DAY;
JZG_EXT	uint8 sv_sd_frame[21];
//信号量  
JZG_EXT OS_EVENT *g_JG0flag;
JZG_EXT	OS_EVENT *g_JG1flag;
JZG_EXT	OS_EVENT *g_JG2flag;
JZG_EXT	OS_EVENT *g_JG3flag;
JZG_EXT	OS_EVENT *g_JG_Pro;
JZG_EXT	OS_EVENT  *g_Uart1_send;

//Uart1出车中用于保存一帧数据的车

//Uart1出车中用于统计未被保存的出车数，当出车数满足24时，将会一起写进SD卡中


//用于保存往SD卡中保存车辆的局部信息
//JZG_EXT	OS_EVENT *		g_psemKey;				//键盘信号量


	 	  
//JZG_EXT	OS_EVENT *		g_psemScreenRefresh;	//屏幕刷新信号量  
//变量
JZG_EXT	NetInfo			g_sniLocal;
JZG_EXT uint8           g_u8LaneDir;           

JZG_EXT	SetupParam		g_sspSetup;				//设置参数	1K， 原Setup   
//键盘
JZG_EXT	volatile uint8	g_u8KeyValueMapped;
JZG_EXT	volatile uint32	g_u32KeyValueOri;	
JZG_EXT	volatile uint32	g_u32KeyCnt;  	 

JZG_EXT	SystemTime		g_sstTempTime;			//临时时间
JZG_EXT	SystemTime		g_sstCurTime;			//当前时间
JZG_EXT	SystemTime		g_sstStartTime;			//启动时间，用于记录系统重启时间  

#define POINT_SUM	361

//JZG_EXT int32   JishuNum;
//JZG_EXT	int32   LMS_data_1Copy[POINT_SUM+1];
//JZG_EXT	int32	LMS_data_2[POINT_SUM+1];

JZG_EXT	 uint32 g_u32count_Pro;	//数据处理计数
JZG_EXT	float Sin_Angle12;		//float Angle12		=	48.432;	
//cos(fabs(Angle12-90)*pi/180);
JZG_EXT	float Cos_Angle12;
JZG_EXT	uint32 g_u32cout_Pro_Two_Buff; //处理计数       //500循环
JZG_EXT	uint8  g_u8flag_veh;//本次处理有车
//JZG_EXT	int32  g_Base_data0_Value[POINT_SUM];
//JZG_EXT	int8  g_Base_data0_Cnt[POINT_SUM];
//JZG_EXT	int32  g_Base_data1_Value[POINT_SUM];
//JZG_EXT	int8  g_Base_data1_Cnt[POINT_SUM];

JZG_EXT uint32 g_total_count_Veh;

JZG_EXT	uint8 	g_u8Jg_4_Buff_Count; //两激光缓存计数；
JZG_EXT	uint8 	g_au8JG_4_Buff[4][832];  //激光数据缓存 4个；	
JZG_EXT	uint8 	g_au8Two_Buff[5000][1662];  //将两激光数据合并存储buff
JZG_EXT	uint32 	g_u32Two_Buff_cout;		//接收	JG2 计数 //500循环

JZG_EXT int32 	 ThresVehHigh;       //%车辆高度上线：5米
JZG_EXT	int32 	 ThresVehLow;       //%车辆高度下线：20cm
JZG_EXT	int32 	 ThresVehParallelWide;  // %并车宽度上线：3.5m
JZG_EXT	int32 	 ThresVehSingleWide;//%    并车情况下单车宽度阈值，2.2m	//修改为2000，hong.7.11
JZG_EXT	int32 	 ThresOrigineDataHigh ;  //%激光器扫描距离上线：20m
JZG_EXT	int32	 ThresOrigineDataLow;      //%激光器扫描距离下线：0.03m


JZG_EXT	int32	 g_UART1Cnt;      //%激光器扫描距离下线：0.03m
#if  SHOWVEHPASSDEBUG > 0	//显示过车调试代码	
JZG_EXT	char			g_chVehPassDebug[1024];	//调试过车界面用记录F4代码
JZG_EXT	uint16			g_u16VehDebugIndex;	  
#endif 

#if	YBVERSION >= 30		//3.0仪表功能
//SD卡
JZG_EXT uint8			g_u8SDInsert; 
JZG_EXT	uint8			g_u8SDEndFlag;
JZG_EXT	uint8			g_u8SDDownFlag;

#define   SMALL_AREA              300
#define   MIN_PTNUM               3
#define   SMOOTH_BOUNDLIMIT	    500	
#define   VEHICHLE_DISTANT_GAP	1000	 //两车间距	
#define   BEISHU                1.414
#define   BIGANGSMALLTHR      6000 

//车型标志
#define ZHONGXIAOKE    1
#define DAKECHE        3
#define XIAOHUOCHE     2
#define ZHONGHUO       4
#define DAHUO          5
#define TEDAHUO        6
#define JIZHUANGXIANG  7
#define MOTUOCHE       9
#define TUOLAJI        8
									 
//温度	  
JZG_EXT	uint32			g_u32Temprature;
#else	//2.2仪表功能
JZG_EXT	void	(*bootloader_entry)(void);
#endif	//#if	YBVERSION >= 30		//3.0仪表功能

#define VEHICLE_MASK	0x0F
#define VEHICLE_MAX		0x10

JZG_EXT uint8             g_u8JGFXFlag;   //激光器旋转方向，点数增加与x轴值增加是否一致，1表示一致

JZG_EXT uint16            g_NearMinWide;
JZG_EXT uint16            g_NearMaxWide;            //近激光器的最大宽度（即激光器到靠近激光器一侧隔离带的宽度）
JZG_EXT uint16            g_FarMinWide;             //激光器到远离激光器一侧的隔离带的宽度
JZG_EXT uint16            g_FarMaxWide;             //激光器到远离激光器一侧的有效最远距离
JZG_EXT uint16            g_MedianLeftWide;         //左边隔离带宽  用于正装方式
JZG_EXT uint16            g_MedianRightWide;        //右边隔离带宽  用于正装方式
JZG_EXT uint16            g_MaxLeftWide;            //左边最大有效宽度 用于正装方式
JZG_EXT uint16            g_MaxRightWide;           //右边最大有效宽度 用于正装方式
JZG_EXT uint16            g_LaneWide;               //车道宽，g_sspSetup.LaneWide*BEISHU
JZG_EXT uint32            g_VerToIncDistance;       //垂直激光器与倾斜激光器扫描在道路上的间隔距离
/********************侧装垂直激光器JG0和JG1用*******************/
JZG_EXT	uint16			  g_VehicleSetIndex[VEHICLE_MAX];	
JZG_EXT uint8			  g_totalVehicle;	 //当前正在处理的车数
JZG_EXT uint32            g_VerToIncDistance;       //垂直激光器与倾斜激光器扫描在道路上的间隔距离
JZG_EXT int32			  g_XdistanceV[POINT_SUM];	 //垂直时使用
JZG_EXT int32			  g_ZdistanceV[POINT_SUM];	 //垂直Z轴使用
JZG_EXT	VehicleStruct	g_VehicleSet[VEHICLE_MAX];
JZG_EXT int32			  g_XdistanceV1[POINT_SUM];	 //JG1X用
JZG_EXT int32			  g_ZdistanceV1[POINT_SUM];	 //JG1Z用
/***************************************************************/
/********************顺车道激光JG2用****************************/
JZG_EXT	VehIncSt		g_VehIncSet[VEHICLE_MAX];
JZG_EXT	uint16			  g_VehIncSetIndex[VEHICLE_MAX];
JZG_EXT uint8			  g_VehIncTotal;	 //当前正在处理的顺扫车数
JZG_EXT int32			  g_ZdistanceI[POINT_SUM];	 //倾斜Z轴使用
JZG_EXT int32			  g_YdistanceI[POINT_SUM];	 //倾斜Y轴使用
/****************************************************************/
/*******************顺车道激光JG3用******************************/
JZG_EXT	VehIncSt		  g_VehIncSet3[VEHICLE_MAX];
JZG_EXT	uint16			  g_VehIncSetIndex3[VEHICLE_MAX];
JZG_EXT uint8			  g_VehIncTotal3;	 //当前正在处理的顺扫车数
JZG_EXT int32			  g_ZdistanceI3[POINT_SUM];	 //倾斜Z轴使用
JZG_EXT int32			  g_YdistanceI3[POINT_SUM];	 //倾斜Y轴使用
#endif
