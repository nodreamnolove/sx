/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			JZGlobal.h
** Last modified Date:  2011511
** Last Version:		1.0
** Descriptions:		���س���ȫ�ֱ���
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
														
 
#define		NOTDEBUG			(DEBUG_NT == 0)	//���ǵ���

#include	"JZStructure.h"

JZG_EXT uint16 limit_height;
JZG_EXT uint16 limit_length;
JZG_EXT uint16 limit_width;
JZG_EXT	uint8  if_send_flag;	//�����ж��Ƿ񽫳�����Ϣͨ��DTU����
JZG_EXT	uint8 sv_frame_data[512];
JZG_EXT	uint8 sv_count;
JZG_EXT	uint16 IF_SAME_DAY;
JZG_EXT	uint8 sv_sd_frame[21];
//�ź���  
JZG_EXT OS_EVENT *g_JG0flag;
JZG_EXT	OS_EVENT *g_JG1flag;
JZG_EXT	OS_EVENT *g_JG2flag;
JZG_EXT	OS_EVENT *g_JG3flag;
JZG_EXT	OS_EVENT *g_JG_Pro;
JZG_EXT	OS_EVENT  *g_Uart1_send;

//Uart1���������ڱ���һ֡���ݵĳ�

//Uart1����������ͳ��δ������ĳ�������������������24ʱ������һ��д��SD����


//���ڱ�����SD���б��泵���ľֲ���Ϣ
//JZG_EXT	OS_EVENT *		g_psemKey;				//�����ź���


	 	  
//JZG_EXT	OS_EVENT *		g_psemScreenRefresh;	//��Ļˢ���ź���  
//����
JZG_EXT	NetInfo			g_sniLocal;
JZG_EXT uint8           g_u8LaneDir;           

JZG_EXT	SetupParam		g_sspSetup;				//���ò���	1K�� ԭSetup   
//����
JZG_EXT	volatile uint8	g_u8KeyValueMapped;
JZG_EXT	volatile uint32	g_u32KeyValueOri;	
JZG_EXT	volatile uint32	g_u32KeyCnt;  	 

JZG_EXT	SystemTime		g_sstTempTime;			//��ʱʱ��
JZG_EXT	SystemTime		g_sstCurTime;			//��ǰʱ��
JZG_EXT	SystemTime		g_sstStartTime;			//����ʱ�䣬���ڼ�¼ϵͳ����ʱ��  

#define POINT_SUM	361

//JZG_EXT int32   JishuNum;
//JZG_EXT	int32   LMS_data_1Copy[POINT_SUM+1];
//JZG_EXT	int32	LMS_data_2[POINT_SUM+1];

JZG_EXT	 uint32 g_u32count_Pro;	//���ݴ������
JZG_EXT	float Sin_Angle12;		//float Angle12		=	48.432;	
//cos(fabs(Angle12-90)*pi/180);
JZG_EXT	float Cos_Angle12;
JZG_EXT	uint32 g_u32cout_Pro_Two_Buff; //�������       //500ѭ��
JZG_EXT	uint8  g_u8flag_veh;//���δ����г�
//JZG_EXT	int32  g_Base_data0_Value[POINT_SUM];
//JZG_EXT	int8  g_Base_data0_Cnt[POINT_SUM];
//JZG_EXT	int32  g_Base_data1_Value[POINT_SUM];
//JZG_EXT	int8  g_Base_data1_Cnt[POINT_SUM];

JZG_EXT uint32 g_total_count_Veh;

JZG_EXT	uint8 	g_u8Jg_4_Buff_Count; //�����⻺�������
JZG_EXT	uint8 	g_au8JG_4_Buff[4][832];  //�������ݻ��� 4����	
JZG_EXT	uint8 	g_au8Two_Buff[5000][1662];  //�����������ݺϲ��洢buff
JZG_EXT	uint32 	g_u32Two_Buff_cout;		//����	JG2 ���� //500ѭ��

JZG_EXT int32 	 ThresVehHigh;       //%�����߶����ߣ�5��
JZG_EXT	int32 	 ThresVehLow;       //%�����߶����ߣ�20cm
JZG_EXT	int32 	 ThresVehParallelWide;  // %����������ߣ�3.5m
JZG_EXT	int32 	 ThresVehSingleWide;//%    ��������µ��������ֵ��2.2m	//�޸�Ϊ2000��hong.7.11
JZG_EXT	int32 	 ThresOrigineDataHigh ;  //%������ɨ��������ߣ�20m
JZG_EXT	int32	 ThresOrigineDataLow;      //%������ɨ��������ߣ�0.03m


JZG_EXT	int32	 g_UART1Cnt;      //%������ɨ��������ߣ�0.03m
#if  SHOWVEHPASSDEBUG > 0	//��ʾ�������Դ���	
JZG_EXT	char			g_chVehPassDebug[1024];	//���Թ��������ü�¼F4����
JZG_EXT	uint16			g_u16VehDebugIndex;	  
#endif 

#if	YBVERSION >= 30		//3.0�Ǳ���
//SD��
JZG_EXT uint8			g_u8SDInsert; 
JZG_EXT	uint8			g_u8SDEndFlag;
JZG_EXT	uint8			g_u8SDDownFlag;

#define   SMALL_AREA              300
#define   MIN_PTNUM               3
#define   SMOOTH_BOUNDLIMIT	    500	
#define   VEHICHLE_DISTANT_GAP	1000	 //�������	
#define   BEISHU                1.414
#define   BIGANGSMALLTHR      6000 

//���ͱ�־
#define ZHONGXIAOKE    1
#define DAKECHE        3
#define XIAOHUOCHE     2
#define ZHONGHUO       4
#define DAHUO          5
#define TEDAHUO        6
#define JIZHUANGXIANG  7
#define MOTUOCHE       9
#define TUOLAJI        8
									 
//�¶�	  
JZG_EXT	uint32			g_u32Temprature;
#else	//2.2�Ǳ���
JZG_EXT	void	(*bootloader_entry)(void);
#endif	//#if	YBVERSION >= 30		//3.0�Ǳ���

#define VEHICLE_MASK	0x0F
#define VEHICLE_MAX		0x10

JZG_EXT uint8             g_u8JGFXFlag;   //��������ת���򣬵���������x��ֵ�����Ƿ�һ�£�1��ʾһ��

JZG_EXT uint16            g_NearMinWide;
JZG_EXT uint16            g_NearMaxWide;            //��������������ȣ���������������������һ�������Ŀ�ȣ�
JZG_EXT uint16            g_FarMinWide;             //��������Զ�뼤����һ��ĸ�����Ŀ��
JZG_EXT uint16            g_FarMaxWide;             //��������Զ�뼤����һ�����Ч��Զ����
JZG_EXT uint16            g_MedianLeftWide;         //��߸������  ������װ��ʽ
JZG_EXT uint16            g_MedianRightWide;        //�ұ߸������  ������װ��ʽ
JZG_EXT uint16            g_MaxLeftWide;            //��������Ч��� ������װ��ʽ
JZG_EXT uint16            g_MaxRightWide;           //�ұ������Ч��� ������װ��ʽ
JZG_EXT uint16            g_LaneWide;               //������g_sspSetup.LaneWide*BEISHU
JZG_EXT uint32            g_VerToIncDistance;       //��ֱ����������б������ɨ���ڵ�·�ϵļ������
/********************��װ��ֱ������JG0��JG1��*******************/
JZG_EXT	uint16			  g_VehicleSetIndex[VEHICLE_MAX];	
JZG_EXT uint8			  g_totalVehicle;	 //��ǰ���ڴ���ĳ���
JZG_EXT uint32            g_VerToIncDistance;       //��ֱ����������б������ɨ���ڵ�·�ϵļ������
JZG_EXT int32			  g_XdistanceV[POINT_SUM];	 //��ֱʱʹ��
JZG_EXT int32			  g_ZdistanceV[POINT_SUM];	 //��ֱZ��ʹ��
JZG_EXT	VehicleStruct	g_VehicleSet[VEHICLE_MAX];
JZG_EXT int32			  g_XdistanceV1[POINT_SUM];	 //JG1X��
JZG_EXT int32			  g_ZdistanceV1[POINT_SUM];	 //JG1Z��
/***************************************************************/
/********************˳��������JG2��****************************/
JZG_EXT	VehIncSt		g_VehIncSet[VEHICLE_MAX];
JZG_EXT	uint16			  g_VehIncSetIndex[VEHICLE_MAX];
JZG_EXT uint8			  g_VehIncTotal;	 //��ǰ���ڴ����˳ɨ����
JZG_EXT int32			  g_ZdistanceI[POINT_SUM];	 //��бZ��ʹ��
JZG_EXT int32			  g_YdistanceI[POINT_SUM];	 //��бY��ʹ��
/****************************************************************/
/*******************˳��������JG3��******************************/
JZG_EXT	VehIncSt		  g_VehIncSet3[VEHICLE_MAX];
JZG_EXT	uint16			  g_VehIncSetIndex3[VEHICLE_MAX];
JZG_EXT uint8			  g_VehIncTotal3;	 //��ǰ���ڴ����˳ɨ����
JZG_EXT int32			  g_ZdistanceI3[POINT_SUM];	 //��бZ��ʹ��
JZG_EXT int32			  g_YdistanceI3[POINT_SUM];	 //��бY��ʹ��
#endif
