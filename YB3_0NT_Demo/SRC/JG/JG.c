#include "crc.h"
#include "JG.h"
#include "math.h"	
#include "config.h"
#include "Timer0.h"
#include "common.h"
#include "cmd.h"
#include "RD_data.h"
#include "WT_Task.h"
#include "Uart5.h"
#include "Uart1.h"
#include "LCDApp.h"
#include "Task_Data_JG.h"
#include "Task_SendUart1.h"
#include "JZStructure.h"
#include "W5100.h" 
#include "W5100App.h"
#include "task_sd.h"
#include "tdc256.h"
#include "TaskMatchSend.h"
#define pi 3.14

#include<stdio.h>
#include<stdlib.h>
#include<string.h>

extern uint32	sv_write_sd_add;
_Sveh_Que Sveh_Que;
_VehSendInfo VehSendInfo[30];
_Cycle_Que_Continue cycle_que_continue;
//������Ϣ�ṹ������ά������   ������SDRAM�У��ϵ綪ʧ
uint32 tail_VehSendInfo = 0;
uint32 head_VehSendInfo = 0;
uint32 CycleQue_Cnt_VehSendInfo = 30;	  
unsigned char Que_VehSendInfo[30] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29};	//��ʼ��
uint32 head_continue,tail_continue;	  //��������	��������
#define    ERRORVALUE       0xFFFF
#define    MAXDAFEIPTNUM    4

#define    MINWIDE_THRESHOLD     100   //��ǰ��������С�ڸ�ֵ��������ƥ������
#define		POINTNUM	360
#define		TOTALCAR	10			//20140213

#define		VEH_HEAD_RES	2600	  //��ͷ��
#define		VEH_BODY_H_RES	3200	  //����߶���ֵ
	 

/**************�׳��ñ���---��ɾ��************************/
							   //�ĳ����ֿ���
uint32 g_total_veh[6][9]={0}; //�ų���������С�ͳ�	С����	��ͳ�	���ͻ���	���ͻ���	�ش��ͻ���	��װ�䳵	������	Ħ�г�
uint32 g_total_veh_temp[6][9]={0}; //G4Э�����ڳ�������
uint32 g_speed_veh_sum[6][9]={0}; //�ų����ٶ�(ͳ���������ۼ�)��С�ͳ�	С����	��ͳ�	���ͻ���	���ͻ���	�ش��ͻ���	��װ�䳵	������	Ħ�г�
uint32 g_total_Lane[6]={0};// �ĳ������Գ���������
uint32 g_average_shiju[6]={0}; //��������ͷʱ������
uint32 g_sum_shiju[6] = {0};   //��������ͷʱ��sum ��
uint32 g_sum_shijian_share[6] = {0};   //�ĳ���ʱ��ռ���ʣ�
uint32 g_sum_shijian[6] = {0};		//�ĳ�������ռ��ʱ���ܺͣ�
uint32 g_total_genche[6]={0};//����������
uint32 g_percent_genche[6]={0};//�����ٷֱȣ�
uint32 g_average_jianju[6]={0}; //ƽ����������ͷ���


int32  g_ai32Pre_Veh_Info_1_Lane[26]={0};  //����1����ǰһ��������Ϣ�����һλ����ǰһ�����ĳ�ͷʱ��
int32  g_ai32Pre_Veh_Info_2_Lane[26]={0};  //����2����ǰһ��������Ϣ��
int32  g_ai32Pre_Veh_Info_3_Lane[26]={0};  //����3����ǰһ��������Ϣ��
int32  g_ai32Pre_Veh_Info_4_Lane[26]={0};  //����4����ǰһ��������Ϣ��
int32  g_ai32Pre_Veh_Info_5_Lane[26]={0};  //����5����ǰһ��������Ϣ��
int32  g_ai32Pre_Veh_Info_6_Lane[26]={0};  //����6����ǰһ��������Ϣ��


int32 Veh_Info[25]={0};   //��ǰ��������Ϣ


int32 ManualforWide = 0;
int32 ManualforHeight = 0;


/******************��ȡ���ĳ�����Ϣ****************************/
uint8  VehMod=0;
uint16 VehLength=0;	           //����
uint16 VehSpeed=0;             //����
uint8  VehTime=0;              //ʱ��:�����ա�ʱ����

uint16 VehHeight=0;            //����
uint8  VehHeadFlag=0;          //�Ƿ����ֳ���ͷ��	1�����ֳ���ͷ��  0��û�����ֳ���ͷ
uint16 VehHeadLength=0;        //��ͷ����
uint8  VehTopPlaneness=0;      //������ƽ����		1��ƽ���� 0��ƽ��
uint8  VehBackPlaneness=0;     //����ƽ����		1��ƽ���� 0��ƽ��




//�������ȼ������
int Length_S1=0;
int Length_S2=0;
int StartTime=0;
int EndTime=0;
int LengthFlag1=0;
int LengthFlag2=0;



/***************************************************************/



uint8 VehInfo_Buffer[19];	 //��������λ�����ͳ����ٶ�

uint8 DivideSigBuffer[8];	 //���ڷ��ͷֳ��ź�
extern uint16  g_u16DataFrame;
extern uint16  g_u16QufaFrame;
extern uint8 send_flag;
extern uint8 send_count;
uint8   l_u8VehHeight[20];
uint8 g_data=0;
uint8 g_match = 1;			 //20140404


/*����ȫ�ֱ���*/ 
#define   ANGLEDEVIATION_V     18      //��ֱ����Ƕ�ƫ��9��
#define   ANGLEDEVIATION_I    	20       //��б����Ƕ�ƫ��7��
#define   SMALL_CAR            600

uint8  g_u8TimeHour;     //����ʱ������ 20130416 
uint8  g_u8TimeMin;
uint8  g_u8TimeSec;


uint16  g_u16InclineStartAnglePt  = 0;   //��б����������ʼ����
uint16  g_u16InclineEndAnglePt    = 0;   //��б����������ֹ����


int32  PastPoint[10]={0};         //���÷�ʱ�ֳ��㷨�м�¼��ʷ�ֳ���


int32   ThresVehLengthLow=1000;     // ��������������ޣ�����ʶ����



#define ERRPTTHRESHOLD       100  //   20130510 ��ɵĵ����ֵ

/***********�����***************/
#define FENCHEFLAG      0x22	  //�ֳ������
#define VEHINFOR	   0x10       //����ţ�����λ�����ͳ�����Ϣ


/*ʱ�����*********************************/
uint32 JG_T0TC[20]={0};
uint32 JG_counter2[20]={0};
uint32 g_u32Test = 0;
/**********************************/

/////////////////////////////
/*
*/

#define max(x,y)	(x > y ? x:y)
#define min(x,y)	(x > y ? y:x)
#define MAX_U(x,y)	(abs(x) > abs(y) ? x:y)
#define MIN_U(x,y)	(abs(x) > abs(y) ? y:x)

#define IS_INSIDE(x,y,dx,dy)	( (min(dx,dy) >= max(x,y)+500 || max(dx,dy) <= min(x,y))-500 ? 0:1 )   //x,y��dx,dyһ�෵��0


uint8 TCP_ScanSendCmd[16] = {0x73,0x53,0x4E,0x20,0x4C,0x4D,0x44,0x73,0x63,0x61,0x6E,0x64,0x61,0x74,0x61,0x20}; //"sSN LMDscandata"

int32 GetMaxValue(const int32 *pData,uint32 start,uint32 end);
uint16 GetPosFromXDistance(int32 *xDistant,PointStruct *pPtStruct, PointStruct *pPtData, uint8 vehPosFlag);
uint8 ISVehicle(VehicleDataStruct* pVdata);
uint8 ISVehRegion(const uint16 u16RegionWide, const PointStruct *pPtStruct, const int32* pZdistance);
uint16 RegionMatch(int32 n32LeftX, int32 n32RightX, VehicleStruct *pVehicle, uint16 u16Index);
uint8 RegionMatch_Point(PointStruct *pVILocateX, PointStruct *pLocateX, PointStruct *pPtData, uint8 u8Flag);
void RegionMerging(PointSet *pFrameInfo);
void RegionMergingEx(PointSet *pFrameInfo);
int Myrand(int start, int end);
//
//int32 Average(const int32 *a,uint8 num)
//{
//   int32 aver=0;
//	uint8 i;
//
//	//20140217 �����ж�
//	if (a == NULL )
//	{
//		return 0;
//	}
//
//	for(i=0;i<num;i++)
//	{
//	    aver+=a[i];
//	}
//	aver=aver/num;
//	return aver;
//	
//}
//

//void clearVehicleErr()
//{
//	uint16 i,j,l_u16tmp;
// 	if (g_totalVehicle > VEHICLE_MAX)
//	{
//		g_totalVehicle = 0;
//		for (j = 0; j < VEHICLE_MAX; j++)
//		{
//			if (g_VehicleSet[j].u8Vstate != NO_USED)
//				g_totalVehicle++;
//		}
//	}
//
//	 for(j = 0;j < g_totalVehicle;j++)
//	 {
//		   i = (g_VehicleSetIndex[j] - 1) & VEHICLE_MASK;
//		   if (g_VehicleSet[i].u8Vstate == PASSED_USED)  
//		   {
//				if (g_VehicleSet[i].Vdata.u16FrameCnt< NORMAL_MAX_EMPTYFRAME)
//				{
//				   memset(&g_VehicleSet[i],0,sizeof(VehicleStruct));
//				   g_VehicleSet[i].u8Vstate = NO_USED;  //�󴥷��ĳ� 
//				   for(l_u16tmp = j;l_u16tmp < g_totalVehicle - 1;l_u16tmp++)
//					   g_VehicleSetIndex[l_u16tmp] = g_VehicleSetIndex[l_u16tmp+1];
//				   
//				   g_VehicleSetIndex[g_totalVehicle - 1] = 0;
//				   g_totalVehicle--;
//				}
//		   }
//	}
//   }
//
///**************����д********************/
//void sendTmpData(int *data,int len)
//{  
//	uint16  Index = 0,i;
//	memset(Tx_Buffer,0,W5100BUFSIZE);
//	Tx_Buffer[Index++] = 0x02;
//	Tx_Buffer[Index++] = 0x02;
//	Tx_Buffer[Index++] = 0x02;
//	Tx_Buffer[Index++] = 0x02;
//
//	Tx_Buffer[Index++] = 0x01;
//
//	Tx_Buffer[Index++] = 0x00;
//	Tx_Buffer[Index++] = 0x00;
//	Tx_Buffer[Index++] = 0x00;
//
//	memcpy(Tx_Buffer+Index, TCP_ScanSendCmd, 16);  
//
//	Tx_Buffer[83] = ((len)>>8)& 0xFF;	   //ȡ��8λ
//	Tx_Buffer[84] = (len) & 0xFF;	       //ɨ������*2����2�����ʾһ�����룬ȡ��8λ	
//	Index =85;
//	for(i=0; i<len; i++)
//	{
//		 Tx_Buffer[Index++] = (data[i]>>8)& 0xFF;
//		 Tx_Buffer[Index++] = data[i] & 0xFF; 
//	}
//
//  	SendDataNet(3,Tx_Buffer,(len<<1)+85);
//}
////��ԭʼ���ݽ���Ԥ���� ��Դ�ɵĵ���0ֵ����Ĵ��� 20130422
//void YuChuLiData_Zero(void)
//{
////	//ֻ����Ƕȷ�Χ�ڵ�����
////	uint16  index = 0;
////	uint16  u16ValidData = 800;  //�����������ֵʱ���Ĺ̶�����ֵ����ʾ�õ��г�
////	uint16  u16TmpIndex = 0;
////	uint16  u16Tmp = 0;
////	uint16  u16TmpHeight;    //�߶Ȳ�(��׼ֵ�뵱ǰ��ֵ�Ĳ
////	uint8   u8TmpFlag = 0;   //Ҫ�ı�0ֵ�ı�ʶ��0��ʾ���ı䣬1��ʾ�ı�
////
////	//��ֱ����������
////	index = g_u16VerticalStartAnglePt-1;
////	u16TmpIndex = index + 1;
////	while(index < g_u16VerticalEndAnglePt-2)
////	{  //20130506  ����0��Ϊ10 ��ΪSICK��������ɵĵ�ֵ��3
////		u16TmpHeight = 	(g_Base_data0_Value[index-1] - LMS_data_1[index-1] > u16ValidData) ?  (g_Base_data0_Value[index-1] - LMS_data_1[index-1]) : u16ValidData;
////		if ( g_Base_data0_Value[index]-LMS_data_1[index] > ThresVehLow && LMS_data_1[index] <=ERRPTTHRESHOLD )	//ԭʼ�����д�ɵĵ�Ϊ0ֵ
////		{
////			if (g_Base_data0_Value[index-1]-LMS_data_1[index-1] > ThresVehLow && LMS_data_1[index-1] > ERRPTTHRESHOLD)	 //ǰһ��ֵ�����Ҹ߶Ȳ����350
////			{
////				u16TmpHeight = 	(g_Base_data0_Value[index-1] - LMS_data_1[index-1] > u16ValidData) ?  (g_Base_data0_Value[index-1] - LMS_data_1[index-1]) : u16ValidData;
////				LMS_data_1[index] =  g_Base_data0_Value[index-1] - u16TmpHeight;
////				u16TmpIndex = index;
////			}
////			else if (g_Base_data0_Value[index+1]-LMS_data_1[index+1] > ThresVehLow && LMS_data_1[index+1] > ERRPTTHRESHOLD)  //��һ��ֵ�����Ҹ߶Ȳ����350
////			{
////				u16TmpHeight = 	(g_Base_data0_Value[index+1] - LMS_data_1[index+1] > u16ValidData) ?  (g_Base_data0_Value[index+1] - LMS_data_1[index+1]) : u16ValidData;
////				for (u16Tmp = u16TmpIndex; u16Tmp <= index; u16Tmp++)
////				{
////					LMS_data_1[u16Tmp] =  g_Base_data0_Value[u16Tmp] - u16TmpHeight;
////				}
////				u16TmpIndex = index;
////				u8TmpFlag = 0;	
////			}
////			else if (g_Base_data0_Value[index+1]-LMS_data_1[index+1] <= ThresVehLow && LMS_data_1[index+1] > ERRPTTHRESHOLD)	 //��һ����������߶Ȳ�С��350
////			{
////			    //20130508  �����ɽû�г�ʱ��4�����ɴ���
////				if (index - u16TmpIndex < 5 && index-u16TmpIndex>1)
////				{
////					for (u16Tmp = u16TmpIndex; u16Tmp <= index; u16Tmp++)
////					{
////						LMS_data_1[u16Tmp] =  g_Base_data0_Value[u16Tmp];
////					}				
////				}
////				else if (index-u16TmpIndex == 1)
////				{
////					LMS_data_1[index] =  g_Base_data0_Value[index];
////				}
////				else
////				{
////					for (u16Tmp = u16TmpIndex; u16Tmp <= index; u16Tmp++)
////					{
////						LMS_data_1[u16Tmp] =  g_Base_data0_Value[u16Tmp] - u16ValidData;	   //��ֵΪĬ�ϸ߶Ȳ�
////					}
////				}		
////				u16TmpIndex = index;
////				u8TmpFlag = 0;
////			}
////			else if (g_Base_data0_Value[index+1]-LMS_data_1[index+1] > ThresVehLow && LMS_data_1[index+1] <= ERRPTTHRESHOLD && (!u8TmpFlag))//��һ��Ҳ�Ǵ�ɵĵ�
////			{
////				u16TmpIndex = index;
////				u8TmpFlag = 1;
////			}
////		}
////		else
////		{
////			u16TmpIndex = index;
////		}
////		index++;
////	}
//}
//
///*********************************************************************************************************
//** ��������:  GetVehHeight2
//** ��������:  ������ƽ����Ϊ�����߶�
//** ��ڲ���:  Z��������ָ�룬��ʼλ�ã�����λ��
//** ���ڲ���:  �߶�ƽ��ֵ
//** ����˵��:
//*********************************************************************************************************/
int GetVehHeight2(int *pg_ZdistanceI, uint16 u16StartPt, uint16 u16EndPt)
{
	int    Tmpi = 0;
	int    Tmpj = 0;
	int    RetHeight = 0;
	int    ThdHeight = 0;
	int    SecHeight = 0;
	int    MaxHeight = 0;
	int    NewHeight = 0;
	uint8  u8PtNum = 0;
	u8PtNum = u16EndPt - u16StartPt + 1;
	if (u8PtNum == 0)
	{
		return 0;
	}
	if (u8PtNum == 1)
	{
		RetHeight = pg_ZdistanceI[u16StartPt];
		return RetHeight;
	}

	//�ȼ���Ƿ����쳣�� �����2�����쳣
	for (Tmpi = u16StartPt; Tmpi <= u16EndPt; Tmpi++)
	{
		if (MaxHeight < pg_ZdistanceI[Tmpi])
		{
			ThdHeight = SecHeight;
			SecHeight = MaxHeight;
			MaxHeight = pg_ZdistanceI[Tmpi];
		}
		else if (SecHeight < pg_ZdistanceI[Tmpi] && 
				(MaxHeight!=pg_ZdistanceI[Tmpi]))
		{
			ThdHeight = SecHeight;
			SecHeight = pg_ZdistanceI[Tmpi];			
		}
		else if (ThdHeight < pg_ZdistanceI[Tmpi] 
			&& SecHeight!=pg_ZdistanceI[Tmpi] 
			&& MaxHeight!=pg_ZdistanceI[Tmpi])
		{
		    ThdHeight = pg_ZdistanceI[Tmpi];
		}	
	}

	if (ThdHeight && SecHeight > ThdHeight + 600 && SecHeight > 2500)  //2�����쳣
	{
		for (Tmpi = u16StartPt; Tmpi <= u16EndPt; Tmpi++)
		{
			if (SecHeight <= pg_ZdistanceI[Tmpi] && Tmpi > u16StartPt)   // erro2 '=' -> '=='
			{
				pg_ZdistanceI[Tmpi] = pg_ZdistanceI[Tmpi-1];
			}
			else if (SecHeight <= pg_ZdistanceI[Tmpi])
			{
				pg_ZdistanceI[Tmpi] = 800;
			}
		    //�����Ҹ߶�
			if (NewHeight < pg_ZdistanceI[Tmpi])
			{
				NewHeight = pg_ZdistanceI[Tmpi];
			}					
		}
	}
	else if (SecHeight && MaxHeight > SecHeight + 600 && MaxHeight > 2500) //��һ�����쳣�߶�
	{
		for (Tmpi = u16StartPt; Tmpi <= u16EndPt; Tmpi++)
		{
			if (MaxHeight == pg_ZdistanceI[Tmpi] && Tmpi > u16StartPt)   // erro2 '=' -> '=='
			{
				pg_ZdistanceI[Tmpi] = pg_ZdistanceI[Tmpi-1];
			}
			else if (MaxHeight == pg_ZdistanceI[Tmpi])
			{
				pg_ZdistanceI[Tmpi] = 800;
			}
		    //�����Ҹ߶�
			if (NewHeight < pg_ZdistanceI[Tmpi])
			{
				NewHeight = pg_ZdistanceI[Tmpi];			
			}
	
		}		
	}
	if (NewHeight > 2600)
	{
		for(Tmpi = u16StartPt; Tmpi <= u16EndPt; Tmpi++)
		{
			if (NewHeight - pg_ZdistanceI[Tmpi] < 200 && Tmpj <5)	//���5����	  ��500��Ϊ200
			{
				RetHeight += pg_ZdistanceI[Tmpi];
				Tmpj++;
			}
		}
	}
	else if (NewHeight>0)
	{
		for(Tmpi = u16StartPt; Tmpi <= u16EndPt; Tmpi++)
		{
			if (NewHeight - pg_ZdistanceI[Tmpi] < 350)
			{
				RetHeight += pg_ZdistanceI[Tmpi];
				Tmpj++;
			}
		}
	}


	if (Tmpj < 1)
	{
		if (NewHeight>0)
		{
			RetHeight = NewHeight;		
		}
		else
		{
			RetHeight = (MaxHeight+SecHeight+ThdHeight)/3;
		}

	}
	else
	{
		RetHeight = RetHeight/Tmpj;    //������ĵ�2�ξ�ֵ
	}

	return  RetHeight;	
	
	  
}

///************************************/
// //����ÿ֡���ݵĶ��ξ�ֵ��Ϊ��֡�ĳ���
int GetVehHeight(VehicleDataStruct *pdata, uint16 u16FrameNum)
{
	int    Tmpi = 0;
	int    Tmpj = 0;
	int    RetHeight = 0;
	int    ThdHeight = 0;
	int    SecHeight = 0;
	int    MaxHeight = 0;
	int    NewHeight = 0;
	uint8  l_u8AfreshFlag = 0;   //���¼����һ�ξ�ֵ�ı�ʶ��1��ʾ���¼���
	uint8  l_u8AfreshPtNum = 0;   //���¼����һ�ξ�ֵ�ĵ���
	uint8  u8PtNum = 0;
	u8PtNum = pdata->zdata[u16FrameNum][0];   //����
	if (u8PtNum == 0)
	{
		return 0;
	}
	if (u8PtNum == 1)
	{
		RetHeight = pdata->zMax[u16FrameNum];
		return RetHeight;
	}

	//�ȼ���Ƿ����쳣�� �����2�����쳣
	for (Tmpi = 1; Tmpi <= pdata->zdata[u16FrameNum][0]; Tmpi++)
	{
		if (MaxHeight < pdata->zdata[u16FrameNum][Tmpi])
		{
			ThdHeight = SecHeight;
			SecHeight = MaxHeight;
			MaxHeight = pdata->zdata[u16FrameNum][Tmpi];
		}
		else if (SecHeight < pdata->zdata[u16FrameNum][Tmpi] && 
				(MaxHeight>pdata->zdata[u16FrameNum][Tmpi]))
		{
			ThdHeight = SecHeight;
			SecHeight = pdata->zdata[u16FrameNum][Tmpi];			
		}
		else if	(ThdHeight < pdata->zdata[u16FrameNum][Tmpi] && 
				(SecHeight>pdata->zdata[u16FrameNum][Tmpi]))
		{
		   ThdHeight = pdata->zdata[u16FrameNum][Tmpi];
		}
	}

	if (ThdHeight && SecHeight > ThdHeight + 600 && SecHeight > 2500)  //2�����쳣
	{
		for (Tmpi = 1; Tmpi <= pdata->zdata[u16FrameNum][0]; Tmpi++)
		{
			if (SecHeight <= pdata->zdata[u16FrameNum][Tmpi] && Tmpi > 1)   // erro2 '=' -> '=='
			{
				pdata->zdata[u16FrameNum][Tmpi] = pdata->zdata[u16FrameNum][Tmpi-1];
			}
			else if (SecHeight <= pdata->zdata[u16FrameNum][Tmpi])
			{
				pdata->zdata[u16FrameNum][Tmpi] = 800;
			}
		    //�����Ҹ߶�
			if (NewHeight < pdata->zdata[u16FrameNum][Tmpi])
				NewHeight = pdata->zdata[u16FrameNum][Tmpi];	
		}
	}
	else if (SecHeight && MaxHeight > SecHeight + 600 && MaxHeight > 2500) //��һ�����쳣�߶�
	{
		for (Tmpi = 1; Tmpi <= pdata->zdata[u16FrameNum][0]; Tmpi++)
		{
			if (MaxHeight == pdata->zdata[u16FrameNum][Tmpi] && Tmpi > 1)   // erro2 '=' -> '=='
			{
				pdata->zdata[u16FrameNum][Tmpi] = pdata->zdata[u16FrameNum][Tmpi-1];
			}
			else if (MaxHeight == pdata->zdata[u16FrameNum][Tmpi])
			{
				pdata->zdata[u16FrameNum][Tmpi] = 800;
			}
		    //�����Ҹ߶�
			if (NewHeight < pdata->zdata[u16FrameNum][Tmpi])
				NewHeight = pdata->zdata[u16FrameNum][Tmpi];	
		}		
	}
	if (NewHeight > 0)
		pdata->zMax[u16FrameNum] = NewHeight;
	else
		pdata->zMax[u16FrameNum] = MaxHeight;

	if (pdata->zMax[u16FrameNum] > 2600)
	{
		for(Tmpi = 1; Tmpi <= u8PtNum; Tmpi++)
		{
			if (pdata->zMax[u16FrameNum] - pdata->zdata[u16FrameNum][Tmpi] < 200 && Tmpj <5)	//���5����	  ��500��Ϊ200
			{
				RetHeight += pdata->zdata[u16FrameNum][Tmpi];
				Tmpj++;
			}
		}
	}
	else
	{
		for(Tmpi = 1; Tmpi <= u8PtNum; Tmpi++)
		{
			if (pdata->zMax[u16FrameNum] - pdata->zdata[u16FrameNum][Tmpi] < 350)
			{
				RetHeight += pdata->zdata[u16FrameNum][Tmpi];
				Tmpj++;
			}
		}
	}

	if (Tmpj < 1)
	{
		RetHeight = pdata->zMax[u16FrameNum];
	}
	else
	{
		RetHeight = RetHeight/Tmpj;    //������ĵ�2�ξ�ֵ
	}

	return  RetHeight;
}

///***********************************************************/
int GetVHeight(VehicleDataStruct *pdata, uint16 u16FrameNum)
{
   	int    Tmpi = 0;
	int    MaxHeight = 0;
   	int	   TempHeight = 0;
	for(Tmpi = 0;Tmpi < u16FrameNum;Tmpi++)
	{
	   TempHeight=GetVehHeight(pdata, Tmpi);
	   if(MaxHeight<=TempHeight)
			MaxHeight = TempHeight;
	}
	return  MaxHeight;
}
///***********************************************************/
//
///*********************************************************************************************************
//** ��������:  GetVehLength
//** ��������:  �����ĳ���
//** ��ڲ���:  �������k����ƽ���ĸ�����
//** ���ڲ���:  ��������ƽ��ֵ
//** ����˵��:  ��Ϊ��������ʻ���м�λ��ʱ����õĳ���������׼ȷ��ȡ�м��һ����֡����ƽ����Ϊ�����ĳ���
//*********************************************************************************************************/
//int GetVehLength(int k,uint8 AveNum)
//{
//
////int Tmpi=0;
////int RealAveNum=0;
////int MidNum_Flag=0;
////int CarLength=0;
////
////int MidNum=0;
////int MinXSum=abs(Lane_Vertical[k][1][3]+Lane_Vertical[k][1][4]); //����һ֡��ͷX�ͳ�βX������͵ľ���ֵ����MinXSum
////
////  
/////************************************���������ҵ���С�ĳ�ͷ�ͳ�βX������͵ľ���ֵ***************************/
////   for(Tmpi=2;Tmpi<Lane_Vertical[k][0][0];Tmpi++)
////    {
////     if((Lane_Vertical[k][Tmpi][1]!=0)&&(Lane_Vertical[k][Tmpi][3]<0)&&(Lane_Vertical[k][Tmpi][4]>0)&&(abs(Lane_Vertical[k][Tmpi][3]+Lane_Vertical[k][Tmpi][4])<MinXSum))
////   	  {
////	  MinXSum=abs(Lane_Vertical[k][Tmpi][3]+Lane_Vertical[k][Tmpi][4]);
////	  MidNum=Tmpi;
////	  MidNum_Flag=1;
////	  } 
////	}
/////***********************************************************************************************/
////
/////******************������ʻ���м�ʱ�����ĳ���ƽ��ֵ*****************************/ 
////if(MidNum_Flag)
//// {
////  if(MidNum>(AveNum/2))
////    {
////	if((Lane_Vertical[k][0][0]-MidNum)>=(AveNum/2))		     //ǰ����AveNum/2����
////	 {
////	 for(Tmpi=MidNum-(AveNum/2);Tmpi<=MidNum+(AveNum/2)-1;Tmpi++)
////	   {
////	   CarLength=CarLength+(Lane_Vertical[k][Tmpi][4]-Lane_Vertical[k][Tmpi][3]);		//��ͷX�����ȥ��βX����
////	   RealAveNum++;
////	   }
////	 }
////	else								                   //ǰ��AveNum/2����������AveNum/2����
////	  {
////	  for(Tmpi=MidNum-(AveNum/2);Tmpi<=Lane_Vertical[k][0][0]-1;Tmpi++)
////	   {
////	   CarLength=CarLength+(Lane_Vertical[k][Tmpi][4]-Lane_Vertical[k][Tmpi][3]);		//��ͷX�����ȥ��βX����
////	   RealAveNum++;
////	   }
////	 
////	  }
////	}
////
////  else 																		  
////        if((Lane_Vertical[k][0][0]-MidNum)>=(AveNum/2))	    //ǰ��AveNum/2����������AveNum/2����
////          {
////		  for(Tmpi=1;Tmpi<=MidNum+(AveNum/2)-1;Tmpi++)
////	        {
////	        
////	        }
////		  }
////		else	                                            //ǰ��AveNum/2����������AveNum/2����
////		   {
////		    for(Tmpi=1;Tmpi<=Lane_Vertical[k][0][0]-1;Tmpi++)
////	        {
////	        CarLength=CarLength+(Lane_Vertical[k][Tmpi][4]-Lane_Vertical[k][Tmpi][3]);		//��ͷX�����ȥ��βX����
////	        RealAveNum++;
////	        }
////		   }
////	
////	 CarLength=CarLength/RealAveNum;
//// }
//// else
////  {
////  MinXSum=abs(Lane_Vertical[k][1][3]);
////  MidNum=1;
////  for(Tmpi=2;Tmpi<Lane_Vertical[k][0][0];Tmpi++)
////	  {
////	  if(abs(Lane_Vertical[k][Tmpi][3])<MinXSum)
////	   {
////	   MinXSum=abs(Lane_Vertical[k][Tmpi][3]);
////	   MidNum=Tmpi;
////	   }
////	  }
////  CarLength=Lane_Vertical[k][MidNum][1];
////
////  }
////     return CarLength; 
//
//	return 0;
// } 
//
// /***********************��ȡ�����������ݽ���************************************/
////u8StarttEndFlag 0��ʾ��ʼ�㣬1��ʾ�����㣻u8VIFlag 0��ʾ��ֱ��1��ʾ��б
uint16 GetStartEndPt(const int* const pdata, const uint16 startPt, const uint8 u8StartEndFlag, const uint8 u8JGIndx)  
{
	uint16 Ret = startPt;
	uint16 index = 0;
	uint16 minHeight = 4000;   //��Ѱ����ʼ�㡢������ʱ����С�߶�
	uint16 u16TmpZeroPos = 180;
	uint16 u16StartPtNum=0;
	uint16 u16EndPtNum=0;
	uint16 u8InstallFlag=0;

	//���ӶԲ������ж�
	if (pdata == NULL || startPt >= POINT_SUM)  //Ϊ�� ���ش���ֵ  20140214
	{
		return ERRORVALUE;
	}

	if(u8JGIndx==0)
	{
		u16TmpZeroPos = g_sspSetup.u16J0ZeroPos;
		u16StartPtNum= g_sspSetup.u16J0StartPos;
		u16EndPtNum=g_sspSetup.u16J0EndPos;
		u8InstallFlag=0;	
	}
	else if(u8JGIndx==1)
	{
	 	u16TmpZeroPos = g_sspSetup.u16J1ZeroPos;
		u16StartPtNum= g_sspSetup.u16J1StartPos;
		u16EndPtNum=g_sspSetup.u16J1EndPos;
		u8InstallFlag=0;
	}
	else if(u8JGIndx==2)
	{
	 	u16TmpZeroPos = g_sspSetup.u16J2ZeroPos;
		u16StartPtNum= g_sspSetup.u16J2StartPos;
		u16EndPtNum=g_sspSetup.u16J2EndPos;
		u8InstallFlag=1;
	}
	else if(u8JGIndx==3)
	{
	 	u16TmpZeroPos = g_sspSetup.u16J3ZeroPos;
		u16StartPtNum= g_sspSetup.u16J3StartPos;
		u16EndPtNum=g_sspSetup.u16J3EndPos;
		u8InstallFlag=1;
	}

	if (u16TmpZeroPos >= POINT_SUM)
		return ERRORVALUE;

	if (u8InstallFlag)  //��װ��ʽ
	{
		if (startPt <= u16TmpZeroPos)	 //��ʼ��	С����������ĵ�
		{
			for (index = u16StartPtNum; index < u16TmpZeroPos; index++)	
			{
				if (pdata[index] > minHeight)	
				{
					Ret = index;
					break;
				}
			}
			if(index>= u16TmpZeroPos)	   //��ʼ��û�ҵ�	  20140217 �޸�
			{
				Ret = u16StartPtNum;
			}
		}
		else
		{
			for (index = u16StartPtNum; index > u16TmpZeroPos; index--)	 //��ֹ��   
			{
				if (pdata[index] > minHeight)
				{
					Ret = index;
					break;
				}
			}
			if(index <= u16TmpZeroPos)	   //��ʼ��û�ҵ�	  20140217 �޸�
			{
				Ret = u16EndPtNum;
			}
		}
	}
	else  //��װ��ʽ
	{
		//��װʱ����ʼ��ͽ������������ͬ��	����ʼ����С�ڽ�����
		if (u8StartEndFlag)	  //Ѱ�ҽ�����
		{
			for (index = startPt; index > u16StartPtNum; index--)	 
			{
				if (pdata[index] > minHeight)
				{
					Ret = index;
					break;
				}
			}
			if(index <= u16StartPtNum)	   //��ʼ��û�ҵ�	 20140217 �޸�
			{
				Ret = u16EndPtNum;
			}
		}
		else  //Ѱ�ҿ�ʼ��
		{
			for (index = startPt; index < u16EndPtNum; index++)  
			{
				if (pdata[index] > minHeight)
				{
					Ret = index;
					break;
				}
			}
			if(index >= u16EndPtNum)	   //��ʼ��û�ҵ�	  20140217 �޸�
			{
				Ret = u16StartPtNum;
			}
		}
	}
	return Ret;
}
///*******************�������ִ��ͳ�����Ҫ�Ǵ�ͳ�3�����ͻ���4,���ͻ���5���ش��ͻ���6����װ��7)********************/
//
//uint8 GetLargeVehPattern(VehicleStruct *pVeh)
//{
////	VehicleStruct *pVehicle = pVeh;
////	uint8   l_u8DafeiFrm    = 0;  //���֡��־ ��1��ʾ��֡�ж�Ϊ��ɵģ� 0����
////	uint8   RetPattern       = 0;
////	uint8   i,j,k;
////	uint8   l_u8DaFeiFrameCnt = 0;  //��¼�������д�ɵ�֡��
////	uint8   l_u8DaFeiFrameCnt2 = 0;
////	uint8   l_u8EqualNum      = 0;   //��¼ÿ֡���ж��ٸ��൱�ĸ߶�ֵ�����ڴ��֡���ж�
////	uint8   l_u8TmpVehNum     = 0;
////	uint8   l_u8QianShiJing   = 0;   //����ǰ�Ӿ��ϵ�֡��־�� 1��ʾҪ�޳���֡
////	uint8   l_u8DownThresVehLow = 0;  //��¼��ֵ���µĵ�ĸ���
////
////	uint8   Toupos           = 0;    //��ͷ�복��ֽ�λ��
////	uint8   Shenpos          = 0;    //����ʼλ�ã���Ҫ���ڼ�װ�䣩
////	uint8   CheDingStartPos  = 0;    //������ʼλ��
////	uint8   IStou            = 0;
////	uint8 StartDing=0;	
////
////	uint8 l_u32index,l_u32index2,l_u32index3;
////	uint8 l_u8CheTouStart;
////	uint8 l_u8CheTouEnd=0;
////	int32 l_nCheTouHeight=0;
////	uint8 l_u8ShuiNiGuanFlag=0;
////
////	uint8   Veh_Num          = 0;
////	int32   VehLength        = 0;
////	int32   VehHeight        = 0;
////	int32   VehWide          = 0;
////	int32 l_u32Height1,l_u32Height2,l_u32Height3;
////	uint32 l_u32Wide1,l_u32Wide2,l_u32Wide3;
////
////	uint8  dakeche     = 0;  //��ͳ���ʶ��1��ʾ�Ǵ�ͳ�
////	uint8  Not_keche = 0;  //���Ǵ�ͳ���ʶ ��1��ʾ����
////	uint8  u8Lane      = 0;	//����
////
////	uint8  l_u8ChetouDafeiPt = 0;
////
////	uint8  jizhuangxiang = 0;
////	uint8  jizhuangxiang_count = 0;
////	uint8  jizhuangxiang_diffcnt = 0;   //��¼�жϼ�װ��ʱ����֡�ĸ߶Ȳ����
////	uint8  u16Index            = 0;
////	uint8  tmpFlag     = 0;
////	uint16 l_u16HeightThresh = 0;
////	uint16 tmp1   = 0;
////	uint16 tmp2   = 0;
////	uint16  l_u8Pos  = 0;
////	uint32	TouHeight =0;	  //��ʽ������ͷ��
////	uint32	TouWide =0;		  //��ʽ������ͷ��
////	uint32	TouGao =0;		  //�����������ͷ��
////	uint32	TouKuan =0;		  //�����������ͷ��
////	uint8   l_u8HighCount = 0;
////	uint8   u8Flag          = 1;
////	uint8   l_u8StartFrame  = 0;	
//// 	int32   ENDRatio=750;
////	int32   l_n32DakeHeightThr = 4000;   //��ͳ��߶���ֵ						 
////	int32   l_n32AllSDThr    = 500;     //ȫ������ֵ
////	int32   l_n32MultiSDThr  = 720;   //��㳵������ֵ
////	int32   allSDcha         =0;    //ȫ����
////	int32   MultiCheshenSD   =0;    //��㳵����
////	int32   SigleCheshenSD=0;
////    int     l_nDaKeHeightThr = 3950;
////	int     l_nDaKeThreshhold11 = 500; 
////    int     l_nDaKeThreshhold12 = 700;
////	int     l_nJZXThreshhold11 = 150;  //��50��Ϊ150   �����ĵ��㳵����Ƚ�
////	int     l_nJZXThreshhold12 = 245;   //��50��Ϊ245   �����Ķ�㳵����Ƚ�
////	int     l_nDaKeThreshhold21 = 400;
////	int     l_nDaKeThreshhold22 = 500;
////	int     DakecheFrameCnt = 12;
////
////	static int32 Z[FRAME_MAXCNT][FRAME_BUFLEN]={0};	 //
////    static int32 Z3[FRAME_MAXCNT][40]={0};
////    static int32 Heightfangcha[FRAME_MAXCNT]={0}; 
////	static int32 duodianduicha[FRAME_MAXCNT]  ={0};	   //�����㷽����м�ֵ
////	static int32 Height[FRAME_MAXCNT] = {0};   //���ڼ���ʱʹ�õĳ���
////	int32   tmpHeight        = 0;
////	int32   Widefangcha      = 0;   //����������ͳ��ĳ�����
////
////	uint8	VehEnd_NumIndex	= 0;
////	uint8   l_u8TotalVehNum     = 0;
////	int		X1,X2;
////
////	memset(Z, 0, sizeof(Z));
////	memset(Z3, 0, sizeof(Z3));
////	memset(Heightfangcha, 0, sizeof(Heightfangcha));
////	memset(duodianduicha, 0, sizeof(duodianduicha));
////	memset(Height, 0, sizeof(Height));
////
////	//20140217  ���ӶԲ����ж�
////	if (pVeh == NULL)
////	{
////		return ZHONGXIAOKE;
////	}
////
////	Veh_Num          = pVehicle->Vdata.u16FrameCnt;
////	if (Veh_Num > FRAME_MAXCNT || Veh_Num < 1)
////	{
////		Veh_Num = 0;  //֡���������֡��ֵ��֡����ֵΪ0
////		pVehicle->Vdata.u16FrameCnt = 1;
////	}
////	VehLength        = pVehicle->yLen;
////	VehHeight        = pVehicle->zLen;
////	VehWide          = pVehicle->xLen;
////
////
////	l_u8TotalVehNum = Veh_Num;
////	if(g_sspSetup.u8RoadType)
////	{
////	    DakecheFrameCnt = 10;
////	}
////	else
////	{
////		DakecheFrameCnt = 15;				
////	}
////
////	//���޳���Щ�н϶��ɵ��֡
////	for (i = 0; i < Veh_Num; i++)
////	{	
////		for (j = 1; j <= pVehicle->Vdata.zdata[i][0]; j++)  //erro3   2.24 ������ѭ��,���޸�
////		{
////			//�޳���ǰ�Ӿ��Գ��ͼ����Ӱ��
////			if (((pVehicle->Vdata.zdata[i][0] <= 6) && (pVehicle->Vdata.zMax[i] > 2000) && (i < 4))
////				|| (pVehicle->Vdata.zMax[i] < 1200 && i < 4)) //���߸߶�С��1200
////			{
////				//ֻ���ǰ4֡������С��6���߶Ƚϸ�,ֱ���޳���֡
////				l_u8QianShiJing = 1;
////				break;
////			} 
////			else if ((pVehicle->Vdata.zdata[i][j] <= ThresVehLow) &&
////			         (pVehicle->Vdata.zMax[i] > 2000) && (i < 4))  // ֻ���ǰ4֡���߶Ƚϸߣ���¼��߶�����ֵ���µĸ���
////			{
////				l_u8DownThresVehLow++;
////			}
////			else
////			{
////			}
////			//Ѱ�Ҵ�ɵ�
////			if (((pVehicle->Vdata.zdata[i][j] == pVehicle->Vdata.zdata[i][j+1]) ||  //ǰ��2�������
////				(l_u8EqualNum>=3 && (0==pVehicle->Vdata.zdata[i][j+1])) 
////				||(l_u8EqualNum>=3 && (pVehicle->Vdata.zdata[i][j]!=pVehicle->Vdata.zdata[i][j+1])&& (pVehicle->Vdata.zdata[i][j+2]==pVehicle->Vdata.zdata[i][j+1]))
////				||(l_u8EqualNum<1&&(pVehicle->Vdata.zdata[i][j] == 0 && pVehicle->Vdata.zdata[i][j+1] !=0))) && (j <pVehicle->Vdata.zdata[i][0])) //20140320��������ȵĵ���ڣ��ҽ����и߶���0ֵ��
////			{
////				if (i < Veh_Num/4 && pVehicle->Vdata.zdata[i][j] >= pVehicle->Vdata.zMax[i])	//����ͳ���ͷ������Ǹ�֡�����߶ȴ�
////				{
////					l_u8ChetouDafeiPt++;			
////				}
////				l_u8EqualNum++;
////			}
////			else
////			{
////				if ((pVehicle->Vdata.zdata[i][0] > 15 && l_u8EqualNum+1 >= pVehicle->Vdata.zdata[i][0]/3) ||
////					(pVehicle->Vdata.zdata[i][0] <=15 && l_u8EqualNum>=4)  //	15�������� �г���4�������Ϊ���
////					) //����8����ֱ����Ϊ��֡�����Ǵ��	  || (l_u8EqualNum >= 8)
////				{ //��Ϊ��֡���Զ���,ֱ������
////					l_u8DafeiFrm = 1;
////					l_u8DaFeiFrameCnt++;
////					break;	
////				}
////				
////				if (l_u8ChetouDafeiPt >= 4)	
////				{
////					l_u8DafeiFrm = 1;
////					l_u8DaFeiFrameCnt++;
////					break;	
////				}
////				l_u8EqualNum = 0;
////				l_u8ChetouDafeiPt = 0;	
////			}
////		
////		}
////		if (l_u8DownThresVehLow>5)
////		{   //��֡С����ֵ�ĵ�������5
////			l_u8QianShiJing = 1;
////		}
////
////		if ((l_u8QianShiJing) || (l_u8DafeiFrm)||l_u8ChetouDafeiPt)//��Ϊ��֡���Զ���
////		{	
////		}
////		else
////		{
////			Height[l_u8TmpVehNum] = pVehicle->Vdata.zMax[i];
////			memcpy(&Z[l_u8TmpVehNum++][0], &(pVehicle->Vdata.zdata[i][0]), sizeof(int32)*(pVehicle->Vdata.zdata[i][0]+1));
////		}
////		l_u8DafeiFrm        = 0;
////		l_u8QianShiJing     = 0;
////		l_u8DownThresVehLow = 0;
////		l_u8EqualNum        = 0;
////		l_u8ChetouDafeiPt   = 0;
////	}
////	if (Veh_Num != l_u8TmpVehNum)  //����ȣ������޳���֡  �����Ҹ�
////	{
////		VehHeight = 0;
////		for (i = 0; i < l_u8TmpVehNum; i++)
////		{
////			if (VehHeight < Height[i])
////			{
////				VehHeight = Height[i];
////			}
////		}
////	}
////	Veh_Num = l_u8TmpVehNum;
////	//�޳����
////
//// //���㳵��
////	l_u32Height1 = 0;
////	l_u32Height2 = 0;
////	if (l_u8TotalVehNum>10 && l_u8TmpVehNum>6 && l_u8TmpVehNum>l_u8TotalVehNum/4)
////	{
////		for (i=0; i<l_u8TmpVehNum-1; i++)
////		{
////			if (i<=l_u8TmpVehNum/3)
////			{
////				l_u32Height1 = l_u32Height1 + Height[i];
////			}
////			else if (i>=l_u8TmpVehNum*2/3)
////			{
////				l_u32Height2 = l_u32Height2 + Height[i];
////			}
////		}
////		l_u32Height1 /=(l_u8TmpVehNum/3+1);
////		l_u32Height2 /=(l_u8TmpVehNum-1-l_u8TmpVehNum*2/3); 	
////	}
////
////    //Ϊ�˷�ֹ��ͳ���ɵ�֡���ܶ�����ֱ���жϴ�ͳ��ķ���
//// 	if (((l_u32Height2>0 && l_u32Height1>0 && abs(l_u32Height2-l_u32Height1)<800)|| (0 == l_u32Height2 && 0 == l_u32Height1))
////		&& Veh_Num < pVehicle->Vdata.u16FrameCnt/2 && pVehicle->Vdata.u16FrameCnt > 12 && pVehicle->zLen > 2500 )
////	{
////		RetPattern = DAKECHE;
////	}
////	else if (Veh_Num <=5 && pVehicle->zLen > 2700)
////	{
////		RetPattern = ZHONGXIAOKE;
////	}
//// 	else if (Veh_Num <= 5) //5֡������ֱ������С�ͳ�
////	{
////		RetPattern = ZHONGXIAOKE;
////	}
////	else
////	{
////	     k=0; 
////		 if(pVehicle->locateX.u16Rightpt < g_sspSetup.u16VerticalZeroPos)
////		 {
////             for(i=0;i<Veh_Num;i++)
////	         {	
////		         k=0;
////				 l_u16HeightThresh = (Height[i]*4/5 > 1500)	? (Height[i]*4/5) : 1500;
////	             for(j=0;j<Z[i][0]*(1000-ENDRatio)/1000;j++)
////	             {
////		            if (Z[i][j] > l_u16HeightThresh && k < 39)
////				    {
////				        Z3[i][k]= Z[i][j];//��ÿ֡�к�����֮һ�����Z3�У�0λ�ô泵�����һ����,�쳣��ȥ��
////					    k++;
////					    Z3[i][39]= k;
////			        }
////				    if(k >= 39)
////				    {
////				        break;
////				    }
////		         }
////	         }
////		 }
////		 else
////		 {
////		     for(i=0;i<Veh_Num;i++)
////	         {	
////		         k=0;
////				 l_u16HeightThresh = (Height[i]*4/5 > 1500)	? (Height[i]*4/5) : 1500;
////	             for(j=Z[i][0];j>Z[i][0]*ENDRatio/1000;j--)
////	             {
////		            if (Z[i][j] > l_u16HeightThresh && k < 39)
////				    {
////				        Z3[i][k]= Z[i][j];//��ÿ֡�к�����֮һ�����Z3�У�0λ�ô泵�����һ����,�쳣��ȥ��
////					    k++;
////					    Z3[i][39]= k;
////			        }
////				    if(k >= 39)
////				    {
////				        break;
////				    }
////		         }
////	         }
////		 }
////	   //���㳵ͷ�ĸ߶Ⱦ�ֵ
////	   l_u8CheTouStart = 0;
////	   for ( i = 1; i < Veh_Num/3; i++)
////	   {
////	   		if (abs(Height[i]-Height[i-1])< 100 && abs(Height[i]-Height[i+1])< 100 && Height[i-1] > 1800)
////			{
////				l_u8CheTouStart = i-1;	  //��ͷ��ʼ��
////				break;
////			}
////	   }
////	   for ( i = l_u8CheTouStart; i < Veh_Num/3; i++)	  //Ѱ�ҳ�ͷ�Ľ�����
////	   {
////	   		if (abs(Height[i]-Height[i+1]) > 200)
////			{
////				l_u8CheTouEnd = i;
////				if (l_u8CheTouEnd - l_u8CheTouStart > 4)
////				{
////					l_u8CheTouEnd = l_u8CheTouStart + 4;
////				}
////				break;
////			}
////	   }
////	   //���㳵ͷ�ľ�ֵ�߶�
////	   if (l_u8CheTouEnd - l_u8CheTouStart + 1 >= 2 && l_u8CheTouEnd >= 1)
////	   {
////	   		if (l_u8CheTouStart >= 10)
////			{
////				l_nCheTouHeight =  Height[l_u8CheTouStart];
////			}
////			else
////			{
////				for(i=l_u8CheTouStart;i<=l_u8CheTouEnd;i++)
////				{
////					
////					if(i > 10)
////					{
////						break;
////					}
////					l_nCheTouHeight += Height[i];
////				}
////				l_nCheTouHeight = l_nCheTouHeight / (i - l_u8CheTouStart);     //��i>10ʱ����l_u8CheTouEnd��ĸ߶Ȳ�׼ȷ
////			}
////	   }
////
////	   //Ѱ�ҳ�����ʼ��λ��
////	   for (i = 1; i < Veh_Num/2; i++)		  //����i<    20131216
////	   {
////	   		if (abs(Height[i]-Height[i-1]) < 100 &&	abs(Height[i]-Height[i+1]) < 100 && Height[i-1] > 2000)
////			{
////				StartDing = i-1;
////				break;
////			}
////	   }
////	   if (g_sspSetup.u8RoadType && StartDing < 2)	 //����·��С��2�Ĵ�2��ʼ
////	   {
////	   		StartDing = 2;
////	   }
////
////	   //Ѱ�ҳ�ͷ�복��ķֽ��
////	   tmp1 = 0;
////	   if (Veh_Num/2 > 4)
////	   {
////	   		for (i = 4; i < Veh_Num/2; i++)
////			{
////				if (Height[i] < 2000)
////				{
////					IStou = 1;
////					Toupos = i;
////					break;
////				}
////			}
////			for (i = 1; i < Veh_Num/2; i++)
////			{
////				if (Height[i]-Height[i+1] > 400)
////				{
////					IStou = 1;
////					Toupos = i;
////					break;
////				}
////				else if (i>=3 && Height[i]-Height[i+1] < -300 )
////				{
////					IStou  = 1;
////					Toupos = i;
////					break;
////				}
////
////				if (abs(Height[i]-Height[i+1])<100)//20140811
////				{
////					tmp1++;
////				}
////				else if (Height[i]-Height[i+1]<-300 && tmp1>0)
////				{
////					IStou = 1;
////					Toupos = i;
////					break;
////				}
////			}
////	   }
////		if (IStou == 1)
////		{
////			for(i = 1; i<=Toupos; i++)
////			{
////				TouHeight += Height[i];
////				TouWide += pVehicle->Vdata.xMax[i];
////			}
////			TouHeight =  TouHeight/Toupos;			   //���㳵ͷ�ߺͳ�ͷ��Ϊʶ����ʽ���� 
////			TouWide = TouWide/Toupos;		
////			for(i=Toupos;i<Veh_Num/2;i++)
////			{
////				if (abs(Height[i]-Height[i-1])<200 && abs(Height[i]-Height[i+1])<200&& Height[i-1]>3000)
////				{
////					Shenpos=i-1;	 //����λ�ã���Ҫ��Ϊ��װ�����
////					break;
////				}
////			}
////		}
////
////		//���㳵��ĵ��㷽��
////		if (Shenpos>0 && Shenpos < Veh_Num-1)
////		{
////			j=0;
////			for (i=Shenpos;i<Veh_Num-1;i++)
////			{
////				j=j+1;
////				Heightfangcha[i-Shenpos]=(Height[i]-VehHeight)*(Height[i]-VehHeight);
////			}
////			SigleCheshenSD=Average(Heightfangcha,j);
////			SigleCheshenSD=SigleCheshenSD/100;
////		}
////		else
////		{
////			j=0;
////			for (i=Veh_Num/3;i<Veh_Num-1;i++)
////			{
////				Heightfangcha[i-Veh_Num/3]=(Height[i]-VehHeight)*(Height[i]-VehHeight);
////				j++;
////			}
////			SigleCheshenSD=Average(Heightfangcha,j);
////			SigleCheshenSD=SigleCheshenSD/100;
////		}
////		//�����㳵����
////		j=0;
////		k=0;
////		for (i=0;i<Veh_Num;i++)
////		{
////			for (j=0;j<Z3[i][39];j++)
////			{
////				duodianduicha[i]=duodianduicha[i] + (Z3[i][j]-VehHeight)*(Z3[i][j]-VehHeight);
////			}
////			if (Z3[i][39] > 0)
////			{
////				duodianduicha[i]=duodianduicha[i]/Z3[i][39];
////			}
////			else
////			{
////				duodianduicha[i] = 0;
////			}
////		}
////		if (Shenpos > 0)
////		{
////			MultiCheshenSD = Average(&duodianduicha[Shenpos],Veh_Num-Shenpos-1);
////		}
////		else
////		{
////			MultiCheshenSD = Average(&duodianduicha[Veh_Num/3],Veh_Num-Veh_Num/3-1);
////		}
////		MultiCheshenSD=MultiCheshenSD/100;
////		//����ȫ��ķ���
////		allSDcha=0;
////		if (StartDing>0)
////		{
////			for (i=StartDing;i<Veh_Num-1;i++)//StartDing
////			{
////				allSDcha=allSDcha+((Height[i]-VehHeight)*(Height[i]-VehHeight));
////			}
////			allSDcha=allSDcha/((Veh_Num-StartDing-1)*100);
////		}
////		else
////		{
////			for (i=1;i<Veh_Num-1;i++)
////			{
////				allSDcha=allSDcha+((Height[i]-VehHeight)*(Height[i]-VehHeight));
////			}
////			allSDcha=allSDcha/((Veh_Num-2)*100);
////		}
////	    //����ǰ����֮һ��ȡ��߶�ƽ����������֮����ȸ߶�ƽ�����Լ�����ƽ����
////		//ǰ����֮һ��ȸ߶�ƽ��
////		l_u32Wide1 = 0;
////		l_u32Height1 = 0;
////		l_u32index2 = 0;
////		l_u32index3 = 0;
////		for(l_u32index = 1;l_u32index < Veh_Num/3;l_u32index++)
////		{
////		    if(pVehicle->Vdata.xMax[l_u32index] < 3000)
////			{
////			    l_u32Wide1 += pVehicle->Vdata.xMax[l_u32index];
////				l_u32index2++;
////			}
////			if(Height[l_u32index] < 5000)
////			{
////			    if(l_u32index <= 2 && Height[l_u32index] > 700)	  //ljj�޸� л��@20130809
////				{
////					l_u32Height1 += Height[l_u32index];
////					l_u32index3++;				
////				}
////				else if(l_u32index >= 3)
////				{
////					l_u32Height1 += Height[l_u32index];
////					l_u32index3++;
////				}
////			} 	    
////		}
////		if(l_u32index2 > 0)
////		{
////		    l_u32Wide1 = l_u32Wide1/l_u32index2;
////		}
////		if(l_u32index3 > 0)
////		{
////		    l_u32Height1 = l_u32Height1/l_u32index3;
////		}
////	
////		//������֮����ȸ߶�ƽ��
////		l_u32Wide2 = 0;
////		l_u32Height2 = 0;
////		l_u32index2 = 0;
////		l_u32index3 = 0;
////		for(l_u32index = Veh_Num/3;l_u32index < Veh_Num - 1;l_u32index++)
////		{
////		    if(pVehicle->Vdata.xMax[l_u32index] < 3000)
////			{
////			    l_u32Wide2 += pVehicle->Vdata.xMax[l_u32index];
////				l_u32index2++;
////			}
////			if(Height[l_u32index] < 5000)
////			{
////			    	l_u32Height2 += Height[l_u32index];
////					l_u32index3++;
////			} 	    
////		}
////		if(l_u32index2 > 0)
////		{
////		    l_u32Wide2 = l_u32Wide2/l_u32index2;
////		}
////		if(l_u32index3 > 0)
////		{
////		    l_u32Height2 = l_u32Height2/l_u32index3;
////		}
////		//�����ȸ߶�ƽ��
////		l_u32Wide3 = 0;
////		l_u32Height3 = 0;
////		l_u32index2 = 0;
////		l_u32index3 = 0;
////		for(l_u32index = 1;l_u32index < Veh_Num - 1;l_u32index++)
////		{
////		    if(pVehicle->Vdata.xMax[l_u32index] < 3000)
////			{
////			    l_u32Wide3 += pVehicle->Vdata.xMax[l_u32index];
////				l_u32index2++;
////			}
////			if(Height[l_u32index] < 5000)
////			{
////			    l_u32Height3 += Height[l_u32index];
////				l_u32index3++;
////			} 	    
////		}
////		if(l_u32index2 > 0)
////		{
////		    l_u32Wide3 = l_u32Wide3/l_u32index2;
////		}
////		if(l_u32index3 > 0)
////		{
////		    l_u32Height3 = l_u32Height3/l_u32index3;
////		} 
////
////		//�����ȷ���	 ���ڴ������
////		l_u32index2 = 0;
////		Widefangcha = 0;
////		if (g_sspSetup.u8RoadType) //����·�����ٽϿ�
////		{
////			if (pVehicle->xLen >= 2000)//������2000������
////			{
////				for (l_u32index = 2; l_u32index < Veh_Num /2 ; l_u32index ++)
////				{	
////					if( pVehicle->Vdata.xMax[l_u32index] < 2800)	  //�������³���Ƚϴ�
////					{
////					   l_u32index2++ ;
////					   Widefangcha += (pVehicle->Vdata.xMax[l_u32index]-l_u32Wide2)*(pVehicle->Vdata.xMax[l_u32index]-l_u32Wide2);
////					}
////					
////				}
////			}
////		}
////		else   //��ͨ·
////		{
////
////			for (l_u32index = 2; l_u32index < Veh_Num /2 ; l_u32index ++)
////			{	
////				if( pVehicle->Vdata.xMax[l_u32index] < 2800)	  //�������³���Ƚϴ�
////				{
////				   l_u32index2++ ;
////				   Widefangcha += (pVehicle->Vdata.xMax[l_u32index]-l_u32Wide2)*(pVehicle->Vdata.xMax[l_u32index]-l_u32Wide2);
////				}
////				
////			}
////			
////		}
////		if(l_u32index2 > 0)
////		{
////		   Widefangcha = Widefangcha / l_u32index2 /100;
////		}
////		else
////			Widefangcha	 = 1000;
////
//////�ҳ��������ɵ���ȷ����ͳ�
//////��֡�Ľ���		
////		VehEnd_NumIndex = 0;
////		if (IStou!=1 && allSDcha>l_nDaKeThreshhold11 && MultiCheshenSD>l_nDaKeThreshhold12)
////		{
////			for	(i=l_u8TotalVehNum*2/3; i< l_u8TotalVehNum; i++)
////			{
////				if (VehHeight>2000 && (2*(pVehicle->Vdata.zdata[i-1][0]+2) >= 3*pVehicle->Vdata.zdata[i][0]) 
////					&& pVehicle->Vdata.zMax[i-1]-pVehicle->Vdata.zMax[i]>1000)
////				{
////					if (i>=l_u8TotalVehNum-2)
////					{
////						VehEnd_NumIndex = i;
////						break;					
////					}					
////					
////				}							
////			}
////		
////		}
////		else		//β֡�е������� 20140809
////		{
////			tmp1 = 0;
////			if (VehHeight>2000)
////			{
////				for (i=0; i< l_u8TotalVehNum; i++)
////				{
////					if (pVehicle->Vdata.zdata[i][0]>tmp1)
////					{
////						tmp1 = 	pVehicle->Vdata.zdata[i][0];
////					}					
////				}
////				for	(i=l_u8TotalVehNum/3; i< l_u8TotalVehNum; i++)
////				{
////					if (pVehicle->Vdata.zdata[i][0]<=tmp1/2 && pVehicle->Vdata.zMax[i] + 1000 <VehHeight)
////					{
////						VehEnd_NumIndex = i;
////						break;
////					}
////				}			
////			}
////		}
////
////		VehEnd_NumIndex = (VehEnd_NumIndex!=0) ? VehEnd_NumIndex : l_u8TotalVehNum;
////		if (IStou==0 && VehHeight>2500)
////		{
////			l_u8DaFeiFrameCnt = 0;
////			l_u8DaFeiFrameCnt2 = 0;
////			for	(i=l_u8TotalVehNum/3; i< VehEnd_NumIndex; i++)
////			{
////				l_u32index2 = 0;
////				l_u32index3 = 0;
////				l_u8EqualNum = 0;
////				X1 = abs(pVehicle->Vdata.xdata[i][pVehicle->Vdata.zdata[i][0]]);
////				X2 = abs(pVehicle->Vdata.xdata[i][1]);
////				l_u32index = (X1<X2) ? pVehicle->Vdata.zdata[i][0] : 1;
////				if (l_u32index != 1)
////				{
////					for (j=l_u32index; j>=1; j--)
////					{
////						if (pVehicle->Vdata.zdata[i][j] >500 && pVehicle->Vdata.zdata[i][j] > pVehicle->Vdata.zMax[i]*2/3)
////						{							
////							break;
////						}
////						l_u32index2++;
////
////					}
////					if (l_u32index2>5)
////					{
////						tmp1 = 0;
////						tmp2 = 0;
////						for(j=l_u32index; j>=1; j--)
////						{
////						   if (pVehicle->Vdata.zdata[i][j]>=pVehicle->Vdata.zMax[i]/2 && pVehicle->Vdata.zdata[i][j]<=pVehicle->Vdata.zMax[i]*9/10)
////						   {
////								if (((pVehicle->Vdata.zdata[i][j] == pVehicle->Vdata.zdata[i][j-1])   //ǰ��2������� 
////									||(l_u8EqualNum>=1 && (pVehicle->Vdata.zdata[i][j]!=pVehicle->Vdata.zdata[i][j-1])&& (pVehicle->Vdata.zdata[i][j-2]==pVehicle->Vdata.zdata[i][j-1]))
////									||(l_u8EqualNum<1&&(pVehicle->Vdata.zdata[i][j] == 0 && pVehicle->Vdata.zdata[i][j-1] !=0))) && (j <pVehicle->Vdata.zdata[i][0]-1)) //20140320��������ȵĵ���ڣ��ҽ����и߶���0ֵ��
////								{
////									l_u8EqualNum++;
////								}
////								else
////								{
////									if (l_u8EqualNum<4)
////									{
////										if (++tmp2>=2)
////										{
////											l_u8EqualNum = 0;
////										}									
////									}
////
////								}													   
////						   }
////						   else if (pVehicle->Vdata.zdata[i][j]>=pVehicle->Vdata.zMax[i]*9/10)
////						   {
////						   		if (++tmp1>=3)
////								{
////									break;								
////								}
////
////						   }
////						   else
////						   {
////						   		continue;
////						   }
////						   l_u32index3++;
////						}
////						if (l_u8EqualNum>=4 && l_u8EqualNum>=(l_u32index3-tmp1)/2)
////						{
////						   l_u8DaFeiFrameCnt++;
////						}
////						else if (l_u8EqualNum>=2)
////						{
////						   l_u8DaFeiFrameCnt2++;
////						}				
////					}				
////								
////				}
////				else
////				{
////					for (j=l_u32index; j<=pVehicle->Vdata.zdata[i][0]; j++)
////					{
////						if (pVehicle->Vdata.zdata[i][j] >500 && pVehicle->Vdata.zdata[i][j] > pVehicle->Vdata.zMax[i]*2/3)
////						{						
////							break;
////						}
////						l_u32index2++;
////					}
////					if (l_u32index2>5)
////					{
////						tmp1 = 0;
////						for(j=l_u32index; j<=pVehicle->Vdata.zdata[i][0]; j++)
////						{
////						   if (pVehicle->Vdata.zdata[i][j]>=pVehicle->Vdata.zMax[i]/2 && pVehicle->Vdata.zdata[i][j]>=pVehicle->Vdata.zMax[i]*9/10)
////						   {
////								if (((pVehicle->Vdata.zdata[i][j] == pVehicle->Vdata.zdata[i][j+1])   //ǰ��2������� 
////									||(l_u8EqualNum>=1 && (pVehicle->Vdata.zdata[i][j]!=pVehicle->Vdata.zdata[i][j+1])&& (pVehicle->Vdata.zdata[i][j+2]==pVehicle->Vdata.zdata[i][j+1]))
////									||(l_u8EqualNum<1&&(pVehicle->Vdata.zdata[i][j] == 0 && pVehicle->Vdata.zdata[i][j+1] !=0))) && (j <pVehicle->Vdata.zdata[i][0]-1)) //20140320��������ȵĵ���ڣ��ҽ����и߶���0ֵ��
////								{
////									l_u8EqualNum++;
////								}
////								else
////								{
////									if (l_u8EqualNum<4)
////									{
////										if (++tmp2>=2)
////										{
////											l_u8EqualNum = 0;
////										}									
////									}
////								}					   
////						   }
////						   else if (pVehicle->Vdata.zdata[i][j]>=pVehicle->Vdata.zMax[i]*9/10)
////						   {
////						   		if (++tmp1 >=3)
////								{
////									break;								
////								}
////
////						   }
////						   else
////						   {
////						   		continue;
////						   }
////						   l_u32index3++;
////						}
////						if (l_u8EqualNum>=4 && l_u8EqualNum>=(l_u32index3-tmp1)/2)
////						{
////						   l_u8DaFeiFrameCnt++;
////						}
////						else if (l_u8EqualNum>=2)
////						{
////						   l_u8DaFeiFrameCnt2++;
////						}				
////					}								
////								
////				}			
////			}		
////			if (l_u8DaFeiFrameCnt>3 && ((l_u8DaFeiFrameCnt >= (VehEnd_NumIndex-l_u8TotalVehNum/3)/3) 
////				|| (l_u8DaFeiFrameCnt2>0 && l_u8DaFeiFrameCnt+l_u8DaFeiFrameCnt2>=(VehEnd_NumIndex-l_u8TotalVehNum/3)/2)))
////			{
////				dakeche = 1;
////			}						
////		}
////
////		if (0 == dakeche && VehHeight>3400 && l_u32Height2>3400)
////		{
////			tmpHeight = 0;
////			for (i=l_u8TotalVehNum*2/3; i<VehEnd_NumIndex; i++)
////			{
////				tmpHeight = tmpHeight + abs(pVehicle->Vdata.zMax[i] - l_u32Height2);
////			}
////			tmpHeight /= (VehEnd_NumIndex-l_u8TotalVehNum*2/3);
////		}
////
////
////		//���ͻ���
////		if (VehHeight < 1600 && pVehicle->Vdata.u16FrameCnt <= 20)	//�������ش������Ϊ��С�ͳ�
////		{
////			RetPattern = ZHONGXIAOKE;
////		}  
////		else if (VehLength >= BIGANGSMALLTHR && VehLength < 6500) //6000����6500
////		{
////			if(VehWide < 1700 && VehHeight < 2100)
////			{
////				RetPattern=XIAOHUOCHE;//С�ͻ���
////			}
////			else if (IStou!=1 && (!(l_u32Height2 - l_u32Height1 > 180 && l_u32Height1 > 0 && l_u32Height2 > 0 && l_u32Height2 > l_u32Height1)))
////			{
////				if (((allSDcha < l_nDaKeThreshhold11 && MultiCheshenSD < l_nDaKeThreshhold12 && MultiCheshenSD > 20))
////				&& VehHeight>2800 && VehHeight<l_nDaKeHeightThr && VehLength >= BIGANGSMALLTHR && pVehicle->Vdata.u16FrameCnt > DakecheFrameCnt)		
////				{
////					RetPattern = DAKECHE;	//��ͳ�
////				}
////				else if((allSDcha < 100 && MultiCheshenSD < 150)
////				&& VehLength <= 7500 && VehHeight > 2350 && VehHeight <= 2700) ////�жϿ�˹�س�Ϊ��ͳ�
////				{
////					RetPattern = DAKECHE;
////				}
////				else
////				{
////					RetPattern = XIAOHUOCHE;//С�ͻ���
////				}				
////			}
////			else
////			{
////				RetPattern= ZHONGHUO;//���ͻ���  20131223��С�ͻ���Ϊ���ͻ���  XIAOHUOCHE
////			}
////			if(RetPattern == XIAOHUOCHE && (VehHeight > 3400 ||(VehWide > 2500 && VehHeight > 3000)))
////			{
////				RetPattern=ZHONGHUO;//���ͻ���
////			}	
////		}
////		else if (VehLength >= 6500 && VehLength <12000)	   //6500����12000
////		{
////			if (VehHeight < 2200 )
////			{
////				RetPattern = ZHONGHUO;   //�л���
////			}
////			else if (IStou==1 || (l_u32Height2 - l_u32Height1 > 200 && l_u32Height1 > 0 && l_u32Height2 > 0 ) )
////			{
////				if(VehLength <= 7500 || VehHeight <= 2900)
////				{
////					RetPattern = ZHONGHUO;//���ͻ���
////				}		
////				else if(l_nCheTouHeight < 2800 && l_nCheTouHeight > 1000)
////				{
////					RetPattern = ZHONGHUO;//���ͻ���
////				}
////				else if(VehLength > 9000)
////				{
////					RetPattern = DAHUO;//���ͻ���
////				}
////				else
////				{
////					RetPattern = ZHONGHUO;//���ͻ���
////				}
////			}
////			else if (VehHeight>3400 && l_u32Height2>3400 && l_u32Height2>l_u32Height1+50
////				&& tmpHeight>0 && tmpHeight<40) //20140809
////			{
////				RetPattern = DAHUO;	//���ͻ���
////			}
////			else if ((allSDcha < l_nDaKeThreshhold11 && MultiCheshenSD < l_nDaKeThreshhold12 && MultiCheshenSD > 20)//20130704,�����㷽������ޣ���ֹС��������
////			&& VehHeight>2800 && VehHeight<l_nDaKeHeightThr && VehLength >= BIGANGSMALLTHR  && pVehicle->Vdata.u16FrameCnt > DakecheFrameCnt)  	 //		֡������
////			{
////				RetPattern = DAKECHE;	//��ͳ�
////			}
////			else if((allSDcha < l_nDaKeThreshhold11 && MultiCheshenSD < l_nDaKeThreshhold12)
////			&& VehLength < 7500 && VehHeight > 2500 && VehHeight < 2750)   //��2600��Ϊ2500
////			{
////				RetPattern = DAKECHE;	//��ͳ���19�����Ͽͳ���25�����£�
////			}
////			else if((allSDcha < 100 && MultiCheshenSD < 150&&VehLength < 7500)
////			 && VehHeight > 2350 && VehHeight <= 2500) ////�жϿ�˹�س�Ϊ��ͳ�
////			{
////				RetPattern = DAKECHE;
////			}
////			else if (1 == dakeche && VehHeight>=2500)		   //20140808
////			{
////				RetPattern = DAKECHE;
////			}
////			else
////			{
////				if(VehLength <= 7500 || VehHeight <= 2900)
////				{
////					RetPattern = ZHONGHUO;//���ͻ���
////				}		
////				else if(l_nCheTouHeight < 2800 && l_nCheTouHeight > 1000)
////				{
////					RetPattern = ZHONGHUO;//���ͻ���
////				}
////				else if(VehLength > 9000)
////				{
////					RetPattern = DAHUO;//���ͻ���
////				}
////				else
////				{
////					RetPattern = ZHONGHUO;//���ͻ���
////				}
////			}
////		}
////		else if (VehLength >= 12000 && VehLength <15000)	//12000����15000
////		{
////			if (IStou == 1)
////			{
////		         if  (SigleCheshenSD< l_nJZXThreshhold11  && MultiCheshenSD < l_nJZXThreshhold12  && VehHeight>3800 && VehHeight<4400)
////			     {
////                     RetPattern = JIZHUANGXIANG;  //��װ�䳵
////			     }
////                 else	 
////			     {
////                     RetPattern = TEDAHUO;	//�ش��ͻ���
////			     }
////			}
////			else if (l_u32Height2 - l_u32Height1 > 200 && l_u32Height1 > 0 && l_u32Height2 > 0 && l_u32Height2 > l_u32Height1)
////			{
////				RetPattern = DAHUO;	//���ͻ���
////			}
////			else if (VehHeight>3400 && l_u32Height2>3400 && l_u32Height2>l_u32Height1+100
////				&& tmpHeight>0 && tmpHeight<40) //20140808
////			{
////				RetPattern = DAHUO;	//���ͻ���
////			}
////			else if ((allSDcha < l_nDaKeThreshhold11 && MultiCheshenSD < l_nDaKeThreshhold12 && MultiCheshenSD > 0)
////			&& VehHeight>2800 && VehHeight<l_nDaKeHeightThr && pVehicle->Vdata.u16FrameCnt > DakecheFrameCnt)
////			{
////				RetPattern = DAKECHE; //��ͳ�
////			}
////			else
////			{
////				RetPattern = DAHUO;		//û��ͷ������ͷ�복��û�����ԵĽ���
////			}
////		}
////		else if(VehLength >= 15000 && VehLength <=18000 )
////		{
////			if (IStou == 1 ||((l_u32Height2 - l_u32Height1 > 200 && l_u32Height1 > 0 && l_u32Height2 > 0 && l_u32Height2 > l_u32Height1)))	//20140122
////			{
////		        if ( SigleCheshenSD< l_nJZXThreshhold11  && MultiCheshenSD < l_nJZXThreshhold12  && VehHeight>3800 && VehHeight<4400)
////			    {
////                    RetPattern = JIZHUANGXIANG;  //��װ�䳵
////                }
////			    else
////			    {
////                     RetPattern = TEDAHUO; //�ش��ͻ���
////		    	}				
////			}
////			else if (VehLength < 16000 && ((allSDcha < l_nDaKeThreshhold11 && MultiCheshenSD < l_nDaKeThreshhold12 && MultiCheshenSD > 50)
////			         && VehHeight>2800 && VehHeight<l_nDaKeHeightThr && pVehicle->Vdata.u16FrameCnt > DakecheFrameCnt))  //����20140122
////			{
////				RetPattern = DAKECHE;
////			}
////			else
////			{
////	             if  (SigleCheshenSD< l_nJZXThreshhold11  && MultiCheshenSD < l_nJZXThreshhold12  && VehHeight>3800 && VehHeight<4400)
////			     {
////                     RetPattern = JIZHUANGXIANG;  //��װ�䳵
////			     }
////                 else
////			     {
////                     RetPattern = TEDAHUO; //�ش��ͻ���
////			     }
////			}
////		}
////		else
////		{
////            if ( SigleCheshenSD< l_nJZXThreshhold11  && MultiCheshenSD < l_nJZXThreshhold12  && VehHeight>3800 && VehHeight<4400)
////		    {
////		       RetPattern = JIZHUANGXIANG; //��װ�䳵
////		    }
////		    else
////		    {
////		       RetPattern = TEDAHUO; //�ش��ͻ��� 
////		    }
////		}
////		
////		if (!IStou && Widefangcha < 70 && (RetPattern == ZHONGHUO || RetPattern == DAHUO)
////			&&(!((l_u32Height2 - l_u32Height1 > 200) 
////			 && l_u32Height1 > 0 && l_u32Height2 > 0)) && VehHeight > 2500)	//������ͳ�,�������
////		if (!IStou && Widefangcha < 70 && (RetPattern == ZHONGHUO || RetPattern == DAHUO))	//������ͳ�,�������
////		{
////			RetPattern = DAKECHE;
////	
////		}
////
////		 if ((RetPattern == DAHUO || RetPattern == ZHONGHUO)&& l_u8DaFeiFrameCnt>0&&
////			(!IStou) && ((l_u32Height2 - l_u32Height1 >= -150 && l_u32Height2 - l_u32Height1<=0) 
////			 && l_u32Height1 > 0 && l_u32Height2 > 0) && (l_u32Wide1 >l_u32Wide2 && l_u32Wide2 > 0)&&VehHeight > 2350)
////		{
////			RetPattern = DAKECHE;		
////		}
////		//���Ӷ������������������ȵ��ж�
////		TouGao = 0;
////		TouKuan = 0;
////		for(u16Index = 1; u16Index <= 4; u16Index++)
////		{
////			TouGao  += Height[u16Index];
////			TouKuan +=	pVehicle->Vdata.xMax[u16Index];
////		}
////		TouGao  = TouGao/4;
////		TouKuan = TouKuan/4;
////		if (RetPattern == ZHONGHUO)
////		{
////
////			if (TouGao >= 2900 && TouKuan >= 2200)	   //����������ͻ��� �������г�ͷ�߿�ϴ���л��ķ���
////			{
////				RetPattern = DAHUO;
////			}
////
////			if (VehLength > 7500 && VehLength < 10000 && VehHeight > 3700 && VehHeight < 4200)  //���3���������
////			{
////				for(u16Index = Veh_Num*2/5; u16Index < Veh_Num*3/5; u16Index++)
////				{
////					if(Height[u16Index+1]- Height[u16Index] >= 0)	//�����߶���������
////						l_u8HighCount++;	
////				}
////				if(l_u8HighCount >= Veh_Num/5 && Height[Veh_Num*3/5] - Height[Veh_Num*2/5] > 150)
////				{
////					RetPattern = DAHUO;	   //���ͻ���	
////				}
////			}
////		}
////		else if(RetPattern == DAHUO)							          
////		{
////			if(2 == Toupos)												 //��Գ�ͷ�ж�����л�
////			{
////				if(Height[1] - Height[0] >= 100
////					&& Height[2] - Height[1] >= 200 
////					&& TouKuan <= 2010)
////					{
////						RetPattern = ZHONGHUO;
////						return RetPattern;		
////					}		
////			}
////			else
////			{
////				if(Height[3] >= Height[2] 
////					&& Height[2] - Height[1] >= 90													   
////					&& Height[1] - Height[0] >= 200
////				    && TouKuan <= 2010 )
////					{
////						RetPattern = ZHONGHUO;
////						return RetPattern;
////					}
////			}   
////		}			
////	}
////	   
////	return RetPattern;
//    return 0;
//}
//
//
///*******************��������С�ͳ�����Ҫ����С�ͳ�1��С�ͻ���2��Ħ�г�8��������9)********************/
//uint8 GetLightVehPattern(VehicleStruct *pVehicle)
//{
//	uint8   RetPattern    = 0;	//
//	uint8   xiaokeche     = 0;
//	uint8   konghuoche    = 0;
//	uint8   xiaohuoche    = 0;
//	uint8 	l_u8index     = 0;
//	uint8   l_u8EqualNum  = 0;
//
//	uint8   l_u8FrameNum  = 0;   //���жϽ𱭳�ʱ������β��֡��
//
//	uint8   i = 0;
//	uint8	j = 0;
//	uint8 k;
//	uint8   l_u8Flag = 0;
//	int32 tmpValue2 = 0;
//	int32 tmpDiff[FRAME_MAXCNT] = {0};  //20130514  ���ڴ�Ÿ߶Ȳ��ֵ
//	uint8 l_u8MaxPos = 0;  //������ֵ����λ��
//	uint8 l_u8MinPos = 0;  //�����Сֵ����λ��
//	int32 l_n32MaxDiff = 0;
//	int32 l_n32MinDiff = 0; 
//	int32 l_n32TmpSum1 = 0;
//	int32 l_n32TmpSum2 = 0;
//	int32 l_n32SecHeight = 0;  //��2��߶�
//	int32 l_n32TempHeight = 0;
//	uint8   Veh_Num       = 0;
//	int32   VehLength     = 0;
//	int32   VehHeight     = 0;
//	int32   VehWide       = 0;
//	uint8   u8SidePlanenessFlag = 0;  //����ÿ֡�Ķ���ƽ����ʶ��0��ʾ��ƽ����1��ʾƽ��
//	uint8 l_u8PiKaFlag = 0;
//	uint8  l_u32index,l_u32index2;
//	int32 Height[FRAME_MAXCNT] = {0};   //���ڼ���ʱʹ�õĳ���
//	uint8  l_u8Left_Index = 0;
//	uint8  l_u8Right_Index = 0;
//	int32  l_n32Left_MaxZ = 0;
//	int32  l_n32Right_MaxZ = 0;
//	int32  l_n32MinChassisHeight = 600;
//	int32  l_n32Min_X            = 0;
//	int32  l_n32Max_X            = 0;
//	memset(Height,0,sizeof(Height));
//
//	//20140217 ���ӶԲ����ж�
//	if (pVehicle == NULL)
//	{
//		return 0;
//	}
//   Veh_Num       = pVehicle->Vdata.u16FrameCnt;
//	if (Veh_Num > FRAME_MAXCNT || Veh_Num < 1)
//	{
//		Veh_Num = 0;  //֡���������֡��ֵ��֡����ֵΪ0
//		pVehicle->Vdata.u16FrameCnt = 1;
//	}
//	VehLength     = pVehicle->yLen;
//	VehHeight     = pVehicle->zLen;
//	VehWide       = pVehicle->xLen;
//
// 	memcpy(Height, pVehicle->Vdata.zMax, Veh_Num*sizeof(int32));
//	if (Veh_Num <= 2)  //  2֡����������С�ͳ�1
//	{
//		RetPattern = ZHONGXIAOKE;
//		return RetPattern;
//	}
//	else
//	{
//		//�޳��߶���1֡�쳣��	20140401����
//		for (i = 0; i < Veh_Num; i++)
//		{ //���ҳ���2��߶�ֵ
//			if ((l_n32SecHeight < pVehicle->Vdata.zMax[i]) && 
//				(VehHeight > pVehicle->Vdata.zMax[i]))
//			{
//				l_n32SecHeight = pVehicle->Vdata.zMax[i];
//			}		  
//		}
//		if (VehHeight-l_n32SecHeight > 300) //�߶ȳ���300�����¼�����߶�ֵ��֡������
//		{
//			for (i = 0; i < Veh_Num; i++)
//			{ 
//				if (VehHeight == pVehicle->Vdata.zMax[i])
//				{
//					for (l_u8index=1;l_u8index<pVehicle->Vdata.zdata[i][0];l_u8index++)
//					{
//						if (l_n32TempHeight < pVehicle->Vdata.zdata[i][l_u8index] && 
//							VehHeight > pVehicle->Vdata.zdata[i][l_u8index])
//						{
//							l_n32TempHeight = pVehicle->Vdata.zdata[i][l_u8index];
//						}	
//					}
//					//�߶����Ƚϴ�
//					if (VehHeight-l_n32TempHeight >=350)
//					{
//						for (l_u8index =1 ; l_u8index<pVehicle->Vdata.zdata[i][0];l_u8index++)
//						{
//							if ((l_n32TempHeight-pVehicle->Vdata.zdata[i][l_u8index] < 350 )
//								&& (l_n32TempHeight>=pVehicle->Vdata.zdata[i][l_u8index]))
//							{
//								l_n32TmpSum1 += pVehicle->Vdata.zdata[i][l_u8index];
//								l_n32TmpSum2++;	
//							}	
//						}
//						
//						if (l_n32TmpSum2 > 0)
//							pVehicle->Vdata.zMax[i] = l_n32TmpSum1/l_n32TmpSum2;
//						else
//							pVehicle->Vdata.zMax[i] = l_n32TempHeight;
//						
//						pVehicle->zLen = (l_n32SecHeight>=pVehicle->Vdata.zMax[i])? l_n32SecHeight:pVehicle->Vdata.zMax[i];	
//						l_n32TmpSum1 = 0;
//						l_n32TmpSum2 = 0;
//						l_n32TempHeight = 0;		
//					}
//				}		  
//			}
//		}
//		VehHeight = pVehicle->zLen;
//		///////////С�γ��ж�////////////////////////
//		if (VehHeight < 2280)
//		{
//			if (pVehicle->Vdata.zMax[0] < 800 && pVehicle->Vdata.zMax[1] < 1000 && pVehicle->Vdata.zMax[Veh_Num-1] < 800)
//			{
//				xiaokeche = 1;  //��С�ͳ���־	
//			}
//		}
//
//		for (i = 0; i < Veh_Num/3; i++)//����ǰ1/3֡����ƽ����
//		{
//			tmpValue2 += pVehicle->Vdata.zMax[i];
//		}
//		if (tmpValue2/(Veh_Num/3) < 1000)	 //ǰ1/3֡����ƽ����С��1m
//		{
//			xiaokeche = 2;   //��С�ͳ�
//		}
//		////////////////С�����ж�///////////////////////////
//		tmpValue2 = 0;
//		if (VehHeight > 2000)  	//���㳵����֡�ߵĲ�ֵ�����ֵ�ľ�ֵ
//		{
//			for (i = Veh_Num/2; i < Veh_Num - 2; i++)
//			{
//				tmpValue2 += abs(VehHeight - pVehicle->Vdata.zMax[i]);
//				l_u8index++;
//			}
//			if (l_u8index > 0)
//			{
//				tmpValue2 = tmpValue2/l_u8index;
//			}
//		}
//		else if (VehHeight > 1800)  //����������֡�Ĳ�ֵ�����ֵ�ľ�ֵ
//		{
//			for (i = Veh_Num/2; i < Veh_Num - 3; i++)
//			{
//				tmpValue2 += abs(pVehicle->Vdata.zMax[i] - pVehicle->Vdata.zMax[i+1]);
//				l_u8index++;
//			}
//			if (l_u8index > 0)
//			{
//				tmpValue2 = tmpValue2/l_u8index;
//			}			
//		}
//		if (tmpValue2 > 100 && l_u8index >= 4)
//		{
//			xiaohuoche = 1;   //С����
//		}
//		/////////�ջ����ж�/////////////////////
//		if (Veh_Num >= 4 && (Veh_Num/2+1 < Veh_Num-1) &&	 //Veh_Num/2+1���������һ֡����
//		   (VehHeight - pVehicle->Vdata.zMax[Veh_Num/2] > 300 || VehHeight - pVehicle->Vdata.zMax[Veh_Num/2+1] > 300))
//		{   //20140325 ���ӷ�ֹ�쳣�߶ȵ����жϿջ�
//			for (i = 0; i < Veh_Num/2+1; i++)
//			{
//				if ((VehHeight - pVehicle->Vdata.zMax[i] < 300) && 
//					(VehHeight > pVehicle->Vdata.zMax[i] || l_u8EqualNum > 1)) 
//				{//��ǰ�벿��֡������֡�����복�������300��
//					konghuoche = 1;  //�ջ���
//					break;					
//				}
//				else if (VehHeight == pVehicle->Vdata.zMax[i])
//				{
//					l_u8EqualNum++;
//				}
//			}
//		}
//		if (Veh_Num >= 4)
//		{
//			for (i = 0; i < Veh_Num/2; i++)
//			{
//				l_n32TmpSum1 += pVehicle->Vdata.zMax[i];
//			}
//			l_n32TmpSum1 = l_n32TmpSum1/(Veh_Num/2);  //ǰһ��֡���߶ȵľ�ֵ
//			
//			for (i = Veh_Num/2; i < Veh_Num-1; i++)	  //�������һ֡����
//			{
//				l_n32TmpSum2 += pVehicle->Vdata.zMax[i];
//			}
//			l_n32TmpSum2 = l_n32TmpSum2/(Veh_Num-Veh_Num/2-1); //��һ��֡���߶ȵľ�ֵ
//			if (l_n32TmpSum1 > (l_n32TmpSum2 + 150) && VehHeight > 1700)	   //40��Ϊ150
//			{
//				konghuoche = 1;
//			}
//		}
//	}
//
//    //�����ж�С�ͳ�
//
//
//
//
//    if(VehWide < 1000 && VehLength < 2000 && VehHeight < 1800)	  //3֡������
//    {
//	    RetPattern = MOTUOCHE;	 //Ħ��
//  	}
//	else if(VehHeight > 2800)	//20140819
//	{
//		RetPattern = XIAOHUOCHE;	
//	}
//	else if(VehHeight > 2500)
//	{
//		if (konghuoche == 1 || xiaohuoche == 1)
//		{
//			RetPattern = XIAOHUOCHE;
//		}
//		else if ((konghuoche != 1 && xiaokeche==1) || xiaokeche == 2)
//		{
//			RetPattern = ZHONGXIAOKE;
//		}
//		else if(VehWide>1800 && abs(l_n32TmpSum1 - l_n32TmpSum2)<50 && tmpValue2<50)
//		{
//
//			RetPattern = DAKECHE;
//		}
//		else
//		{
//			RetPattern = XIAOHUOCHE;
//		}	
//	}
//	else if (!RetPattern)
//	{
//		if (VehHeight < 1600 || xiaokeche == 2)
//		{
//			RetPattern = ZHONGXIAOKE;
//		}
//		else if (konghuoche == 1 || xiaohuoche == 1)
//		{
//			RetPattern = XIAOHUOCHE;
//		}
//		else if (konghuoche != 1 && xiaokeche == 1)
//		{
//			RetPattern = ZHONGXIAOKE;
//		}
//		else
//		{
//			//SUV��ʶ��
//			if (VehHeight < 2000 && VehHeight >= 1600)
//			{
//				l_u8index = 0;
//				l_n32TmpSum1 = 0;
//				for (i = 0; i < Veh_Num; i++)	  //Ѱ�ҿ�ʼ֡���߶ȵ���1300��֡��
//				{
//					if (pVehicle->Vdata.zMax[i] <= 1300)
//					{
//						l_u8index++;
//					}
//					else
//					{
//						break;
//					}
//				}
//				for (i = Veh_Num/2; i < Veh_Num-1; i++)	  //�����һ��֡���߶ȵľ�ֵ
//				{
//					l_n32TmpSum1 += pVehicle->Vdata.zMax[i];
//				}
//				l_n32TmpSum1 = l_n32TmpSum1/(Veh_Num-1-Veh_Num/2);
//
//				if (l_n32TmpSum1 > 1650 && l_n32TmpSum1 < 1900 && ((l_u8index>=2 && l_u8index>=Veh_Num/3) || (l_u8index>=5 && pVehicle->speed > 30)))
//				{
//					RetPattern = ZHONGXIAOKE;
//				}
//			}
//
//			if (!RetPattern)
//			{
//				//��ʽС�����ж�
//				tmpValue2 = 0;
//				for(i = 2;i < (Veh_Num>>1);i++)
//				{
//					if(pVehicle->Vdata.zMax[i] + 300 < pVehicle->Vdata.zMax[i-1])	 //���ҳ�ͷ�����������
//					{
//						tmpValue2 = i+1; 
//						while(tmpValue2 < Veh_Num-1)
//						{
//							if(pVehicle->Vdata.zMax[tmpValue2+1] - pVehicle->Vdata.zMax[tmpValue2] > 300)	 //���ҳ�ͷ�����������
//							{
//								tmpValue2 = tmpValue2+1; 								
//								break;
//							}
//							tmpValue2++;	
//						}								
//						break;
//					}
//				
//				}  	
//				if(tmpValue2 && tmpValue2 < (Veh_Num>>1))
//				{
//					while(tmpValue2 < Veh_Num-1)
//					{
//						if(abs(pVehicle->Vdata.zMax[tmpValue2] - pVehicle->Vdata.zMax[tmpValue2 + 1]) < 150)
//						{
//							 //��ʽС�� 	
//							 RetPattern = XIAOHUOCHE;	
//							 break;
//						}
//						else
//						{
//							 RetPattern = 0;
//							 break;
//						}
//						tmpValue2++;
//					} 			
//				}	 
//			   else
//			   {
//			   	//�ж��Ƿ�Ϊ��ʽС�ͳ�
//				 tmpValue2 = Veh_Num-1;  			 
//				 while(tmpValue2 > 0 )
//				 {
//				 	if(abs(pVehicle->Vdata.zMax[tmpValue2] - pVehicle->Vdata.zMax[tmpValue2 - 1]) < 150)
//					{
//						tmpValue2--;	
//					}
//					else
//						break;
//				 }
//				 if(tmpValue2 <= (Veh_Num>>1))
//				 {
//				 	if(pVehicle->Vdata.zMax[tmpValue2] - pVehicle->Vdata.zMax[tmpValue2 - 1] < 600)
//					{
//					  	i = 1;
//					 	while(i < tmpValue2)
//						{
//							if(pVehicle->Vdata.zMax[i] < pVehicle->Vdata.zMax[tmpValue2])
//							{
//								 RetPattern = ZHONGXIAOKE; //��ʽС�ͳ�	
//								 break;
//							}
//							else
//							{
//								RetPattern = 0;
//								break; 
//							}
//							i++;
//						}
//					}
//					else
//					{
//						RetPattern = ZHONGXIAOKE;  //���� XIAOHUOCHE   ��Ϊ��С�ͳ�
//					}	 
//				 }
//			   }
//		
//				//����ʽ�͡�����
//				if(!RetPattern)
//				{
//					tmpValue2 = 0;
//					for(i = 2;i < Veh_Num-1;i++)  //���һ֡��Ҫ
//					{
//						if(pVehicle->Vdata.zMax[i] + 250 < pVehicle->Vdata.zMax[i-1])	 //���ҳ�ͷ�����������
//						{
//						   tmpValue2 = i;
//						   break;
//						}
//					}
//					if(!tmpValue2)
//					{ 	
//						RetPattern = ZHONGXIAOKE; //С�ͳ�				
//					}
//					else
//					{
//			           	for (i = 2; i < Veh_Num; i++)    //20130514 �����2֡��������2֡�ĳ��߲��
//						{
//							tmpDiff[i] = pVehicle->Vdata.zMax[i] - pVehicle->Vdata.zMax[i-1];
//						}
//						tmpDiff[0] = Veh_Num;   //֡��
//						//Ѱ�ҳ��߲���е����ֵ����Сֵ������λ��	20130514 
//						l_n32MaxDiff = tmpDiff[2];
//						l_n32MinDiff = tmpDiff[Veh_Num-1];
//						l_u8MaxPos   = 2;
//						l_u8MinPos   = Veh_Num-1;
//						for( i = 3; i < Veh_Num; i ++)
//						{				
//							if (l_n32MaxDiff < tmpDiff[i])	//Ѱ�����ֵ������
//							{
//								l_n32MaxDiff = tmpDiff[i];
//								l_u8MaxPos  = i;
//							}
//							if (l_n32MinDiff > tmpDiff[Veh_Num-i+1] && i<Veh_Num-1)
//							{
//								l_n32MinDiff = tmpDiff[Veh_Num-i+1];
//								l_u8MinPos   = Veh_Num-i+1;
//							}
//						}
//						//��ֱ仯�ܴ�˵����С����
//						if ( l_n32MinDiff < -800)
//						{
//							RetPattern = XIAOHUOCHE;   
//						}
//						else
//						{
//							//�����Сֵ�������С����֡��2/3����Ƚ����һ֡��ǰһ֡�Ĳ�֣� ��ֹ����	  20130514
//							if ( l_u8MinPos <= Veh_Num*2/3 && l_n32MinDiff > pVehicle->Vdata.zMax[Veh_Num-1]-pVehicle->Vdata.zMax[Veh_Num-2])
//							{
//								l_u8MinPos = Veh_Num-1;
//								l_n32MinDiff = pVehicle->Vdata.zMax[Veh_Num-1]-pVehicle->Vdata.zMax[Veh_Num-2];
//							}
//							
//							//�����ǰ������С�ڳ�β��������Ѱ���������������һ����ֱ仯�ϴ�ĵ�   20130514
//							if ( l_u8MaxPos -1 < (Veh_Num-l_u8MinPos+1) && tmpDiff[l_u8MaxPos+1] > 150)
//							{
//								l_u8MaxPos = l_u8MaxPos + 1;
//							}
//							//���׵������ڻ���ڳ�β����������β�����ʹ��ڻ���ڳ������ 20130514
//							if (l_u8MaxPos>=(Veh_Num-l_u8MinPos) && (l_u8MaxPos + Veh_Num-l_u8MinPos >= l_u8MinPos-l_u8MaxPos) 
//							   && l_u8MaxPos-1 < l_u8MinPos && (Veh_Num - (l_u8MaxPos-1)*2 > 0  ))
//							{
//								RetPattern = ZHONGXIAOKE;   //��С�ͳ�
//							} 
//							else
//							{
//								RetPattern = XIAOHUOCHE;  //С����
//							}
//						}
//					}
//				}
//			}
//		}
//		////��Ҫ��Ƥ���ļ��
//
//		if(RetPattern == ZHONGXIAOKE && VehHeight > 1540 && VehHeight < 1790 && Veh_Num >= 2)	 //��Χ�Ŵ�һ��1600-1700
//		{
//			//СƤ���ļ�⣬��Ϊβ�ͱȽϳ�����С����Ƥ����
//			for(l_u32index = 0; l_u32index < Veh_Num - 1; l_u32index++)
//			{
//			    if(Height[l_u32index] > Height[l_u32index+1] && Height[l_u32index] - Height[l_u32index+1] > 400)//   Height[l_u32index] > Height[l_u32index+1] && 
//				{
//					break;
//				}		
//			}
//			k = 0;
//			for( i = l_u32index+1; i < Veh_Num; i++)
//			{
//				if(Height[i] < 1250 && Height[i] > 940)
//				{
//					k++;
//				} 
//				else
//				{
//					break;
//				}			
//			}
//			if(k == Veh_Num - l_u32index-1 &&  k >= 2)
//			{
//				 l_u8PiKaFlag = 1;	   //��⵽������β���ϳ�
//			}
//			
//			if( Veh_Num >= 5 && l_u8PiKaFlag)
//			{
//				for(i = Veh_Num - 2; i > Veh_Num - 5; i--)
//		    	{
//			    	if(Height[i] < 1250 && Height[i] > 940)//Ƥ���ĳ���λ��
//					{
//						if (pVehicle->Vdata.zdata[i][0] >= FRAME_BUFLEN) //������ֵ
//							continue;
//						//Ѱ��������ֵ
//						l_n32Left_MaxZ = pVehicle->Vdata.zdata[i][1];
//						l_u8Left_Index = 1;
//						for (k = 1; k <= pVehicle->Vdata.zdata[i][0]; k++)
//						{
//							if (l_n32Left_MaxZ < pVehicle->Vdata.zdata[i][k])
//							{
//								l_n32Left_MaxZ = pVehicle->Vdata.zdata[i][k];
//								l_u8Left_Index = k;
//							}
//							else if (l_n32Left_MaxZ > pVehicle->Vdata.zdata[i][k] && 
//								l_n32Left_MaxZ > 900)
//								break;
//						}
//						//Ѱ���ұ����ֵ
//						l_u8Right_Index = pVehicle->Vdata.zdata[i][0];
//						l_n32Right_MaxZ = pVehicle->Vdata.zdata[i][l_u8Right_Index];
//						for (k = pVehicle->Vdata.zdata[i][0]; k > 0; k--)
//						{
//							if (l_n32Right_MaxZ < pVehicle->Vdata.zdata[i][k])
//							{
//								l_n32Right_MaxZ = pVehicle->Vdata.zdata[i][k];
//								l_u8Right_Index = k;
//							}
//							else if (l_n32Right_MaxZ > pVehicle->Vdata.zdata[i][k] && 
//								l_n32Right_MaxZ > 900)
//								break;
//						}
//						//�Ҹߵ���
//						l_n32TmpSum1 = 0;
//						l_n32TmpSum2 = 0;
//						l_n32TempHeight = l_n32Left_MaxZ >= l_n32Right_MaxZ ? l_n32Left_MaxZ : l_n32Right_MaxZ;
//						for (k = l_u8Left_Index+1; k < l_u8Right_Index; k++)	
//						{
//							l_n32TmpSum1 += (pVehicle->Vdata.zdata[i][k]-l_n32TempHeight);
//							l_n32TmpSum2 += abs(l_n32TempHeight-pVehicle->Vdata.zdata[i][k]);	
//						}
//						if (l_u8Right_Index > l_u8Left_Index+1)
//						{
//							l_n32TmpSum1 = l_n32TmpSum1/(l_u8Right_Index - l_u8Left_Index-1);
//							l_n32TmpSum2 = l_n32TmpSum2/(l_u8Right_Index - l_u8Left_Index-1);
//						}
//						if(l_n32TmpSum1 < 0 && l_n32TmpSum2 > 200 && (l_u8Right_Index - l_u8Left_Index-1 > 6))
//						{
//					   		RetPattern = XIAOHUOCHE;//С����Ƥ��
//					   		break;
//						}
//					}		    
//		    	}
//			}
//		} 			 			
//	}
//
//	if (g_sspSetup.u8RoadType)//����
//	{
//		if (!RetPattern || RetPattern == MOTUOCHE)
//		{
//			RetPattern = ZHONGXIAOKE;
//		}
//	}
//	else
//	{
//		if (!RetPattern)
//		{
//			RetPattern = ZHONGXIAOKE;
//		}
//	}
//
//
//	return RetPattern;
//}
//
//
//uint16 GetLimitValue(int* pg_ZdistanceI,int* pXdata, int len,int startPos, uint8 u8Flag)    //��װ��ʱ��ú��������޸�	u8FlagΪ0��ʾ��ֱ���������ݣ�1��ʾ��б����������
//{ 
////	uint16 ret = startPos;
////	uint16  l_u16CountOut = 0;
////	int l_midPt = len>>1;
////	int l_PtNum1 = 0;    //abs(g_sspSetup.u16StartPtNum - g_sspSetup.u16VerticalZeroPos);  // �������ʼ��ĵ�����
////	int l_PtNum2 = 0;    //abs(g_sspSetup.u16EndPtNum - g_sspSetup.u16VerticalZeroPos);    //�����������ĵ�����
////
////	//���ӶԲ����ĺϷ����ж� 20140214
////	if (pg_ZdistanceI == NULL || pXdata == NULL || len >= POINT_SUM )
////	{
////		return ERRORVALUE;		 //���ش���ֵ
////	}
////	
////	if (u8Flag)	 //��б������
////	{
////		l_PtNum1 = abs(g_sspSetup.u16StartPtNum - g_sspSetup.u16InclineZeroPos);  // �������ʼ��ĵ�����
////		l_PtNum2 = abs(g_sspSetup.u16EndPtNum - g_sspSetup.u16InclineZeroPos);    //�����������ĵ�����
////		
////		//20140217 ����	  ��װ��б
////		if (g_sspSetup.u8InstallFlag)
////		{
////			l_midPt = (g_sspSetup.u16InclineZeroPos > g_u16InclineStartAnglePt) ?  
////					  (g_sspSetup.u16InclineZeroPos - g_u16InclineStartAnglePt) : (len>>1);
////		}		
////	}
////	else  //��ֱ������
////	{
////		l_PtNum1 = abs(g_sspSetup.u16J0StartPos - g_sspSetup.u16J0ZeroPos);  // �������ʼ��ĵ�����
////		l_PtNum2 = abs(g_sspSetup.u16J0EndPos - g_sspSetup.u16J0ZeroPos);    //�����������ĵ�����
////
////		//20140217 ����	  ��װ��ֱ
////		if (g_sspSetup.u8InstallFlag)
////		{
////			l_midPt = (g_sspSetup.u16VerticalZeroPos > g_u16VerticalStartAnglePt) ?
////				      (g_sspSetup.u16VerticalZeroPos - g_u16VerticalStartAnglePt) : (len>>1);
////		}		
////	}
////
////
////
////	while(startPos >= 0 && startPos <= len && (l_u16CountOut++ < len))
////	{
////		if (startPos < l_midPt && abs(pXdata[startPos]) > g_MedianLeftWide &&		    //С��Ϊ������
////		    abs(pXdata[startPos]) <= g_MaxLeftWide)
////		{
////			ret = startPos;
////			break;
////		}
////		else if (startPos > l_midPt && abs(pXdata[startPos]) > g_MedianRightWide &&		//���Ϊ�Ҹ����
////		    abs(pXdata[startPos]) <= g_MaxRightWide)
////		{
////			ret = startPos;
////			break;
////		}
////		else if (startPos > l_midPt)
////		{
////			startPos--;
////		} 		
////		else 
////		{
////			startPos++;
////		}
////	}		
//
////   return ret;
//     return 0;
//}
//
//int32 GetMaxValue(const int32 *pData,uint32 start,uint32 end)
//{
//	uint32 index;
//	int32 retMax = 2000; //Ĭ��ֵ 2��
//
//	//20140217 ���ӶԲ����Ϸ����ж�
//	if (pData == NULL || start > g_sspSetup.u16J0EndPos-g_sspSetup.u16J0StartPos ||
//		end > g_sspSetup.u16J0EndPos-g_sspSetup.u16J0StartPos)	 //�����쳣�����̶�ֵ����Ӱ���������
//	{ 
//		return retMax;
//	}
//
//	retMax = pData[start];
//
//	for(index = start+1;index<end;index++)
//	{
//		if(pData[index] > retMax)
//			retMax = pData[index];		
//	}
//	return retMax;
//}
//
int32 GetMaxData(const int32 *pData,uint32 start,uint32 end)
{
	uint32 Tmpi;
	int32 retMax = 0;
	int32		ThdMaxVal = 0;
	int32		SecMaxVal = 0;
	int32		FstMaxVal = 0;	
	uint8		u8PtNum = 0;

	//20140217 ���ӶԲ����Ϸ����ж�
	if (pData == NULL)	 //�����쳣�����̶�ֵ����Ӱ���������
	{ 
		return retMax;
	}

	u8PtNum = end - start + 1;
	if (u8PtNum == 0)
	{
		return 0;
	}
	if (u8PtNum == 1)
	{
		retMax = pData[start];
		return retMax;
	}

	if (u8PtNum>=3)
	{
		for (Tmpi = start; Tmpi <= end; Tmpi++)
		{
			if (FstMaxVal < pData[Tmpi])
			{
				ThdMaxVal = SecMaxVal;
				SecMaxVal = FstMaxVal;
				FstMaxVal = pData[Tmpi];
			}
			else if (SecMaxVal < pData[Tmpi] && (FstMaxVal!=pData[Tmpi]))
			{
				ThdMaxVal = SecMaxVal;
				SecMaxVal = pData[Tmpi];			
			}
			else if (ThdMaxVal < pData[Tmpi] && (SecMaxVal!=pData[Tmpi]) && FstMaxVal!=pData[Tmpi])
			{
				ThdMaxVal = pData[Tmpi];
			}
		}
		if (FstMaxVal>SecMaxVal+800)
		{
			if (SecMaxVal>ThdMaxVal+800)
			{
				retMax = (FstMaxVal + ThdMaxVal + SecMaxVal)/3;
			}
			else
			{
				retMax = (ThdMaxVal+SecMaxVal)/2;
			}
			
		}
		else
		{
			retMax = FstMaxVal;
		}
	}
	else
	{
		retMax = (pData[start]+pData[end])/2;
	}

	return retMax;
}

////vehPosFlag ��ʶ��0��ʾ�����ڵ�ǰ�����м����ֵ��1���м����ֵ����ǰ����������2����ǰ������ұ���3
////pPtStruct ��ʾ�����ĵ�ṹ�������pPtData��ʾ��ǰ����ĵ�ṹ�����
//uint16 GetPosFromXDistance(int32 *xDistant,PointStruct *pPtStruct, PointStruct *pPtData, uint8 vehPosFlag)	 
//{
//	uint16 ret = 0,Index = pPtData->u16Leftpt;
//	uint16 l_16tmpValue = pPtData->u16Rightpt;
//	uint16 l_minValue = 0xFFFF;
//	uint16 l_u16LaneWide = g_sspSetup.LaneWide; 
//	uint8  l_u8Flag = 0;   //�����ȱ�־ 0��ʾ��ǰ�����ȴ���3.5mС��7m��1��ʾ����7m
//
//	//20140217 ���ӶԲ����Ϸ����ж�
//	if (xDistant == NULL || pPtStruct == NULL || pPtData == NULL)
//	{
//		return ERRORVALUE;
//	}
//
//	if (g_sspSetup.u8LaneNum == 6) //��װ6����ʱ�������ȱ�־���п��ܴ���7m
//	{
//		if (abs(pPtData->n32xRight-pPtData->n32xLeft) > g_sspSetup.LaneWide*2)
//		{
//			l_u8Flag = 1;
//		}
//	}
//
//
//	ret = pPtData->u16Rightpt;
//	Index = pPtData->u16Leftpt;
//	if (l_16tmpValue >= POINT_SUM-1 || ret >= POINT_SUM-1 || Index >= POINT_SUM-1)
//		return ERRORVALUE;
//
//	if (!vehPosFlag)  //���м�	������ֵ
//	{
//		for (; Index < l_16tmpValue-1; Index++)
//		{
//			if (abs(xDistant[Index] - xDistant[Index+1]) > 1000 &&
//			    abs(xDistant[Index+1]-pPtStruct->n32xRight) < l_u16LaneWide&&
//				abs(pPtStruct->n32xLeft-xDistant[Index]) < l_u16LaneWide)
//			{
//				ret = Index;
//				break;
//			}
//		}		
//		if (ret && ret < l_16tmpValue && (abs(pPtStruct->n32xRight-xDistant[Index])> (SMALL_AREA>>1)))
//		{
//			return ret;
//		}
//		ret = pPtData->u16Rightpt;
//		Index = pPtData->u16Leftpt;
//
//		while(Index < l_16tmpValue)
//		{
//			if(abs(xDistant[Index] - pPtStruct->n32xLeft) < l_minValue && (abs(pPtStruct->n32xRight-xDistant[Index])> (SMALL_AREA>>1)))
//			{
//				l_minValue = abs(xDistant[Index] - pPtStruct->n32xLeft);
//				pPtStruct->n32xLeft = xDistant[Index];
//				ret = Index;		
//			}			
//			Index++;	
//		}
//			
//	}
//	else if (vehPosFlag == 1) //���м䣬������ֵ
//	{
//		for (; Index < l_16tmpValue-1; Index++)
//		{
//			if (abs(xDistant[Index] - xDistant[Index+1]) > 1000 &&
//			    abs(xDistant[Index]-pPtStruct->n32xLeft) < l_u16LaneWide &&
//				abs(pPtData->n32xRight-xDistant[Index+1]) < l_u16LaneWide)
//			{
//				ret = Index+1;
//				break;
//			}
//		}		
//		if (ret && ret < l_16tmpValue && (abs(xDistant[Index]-pPtStruct->n32xLeft)>(SMALL_AREA>>1)))
//		{
//			return ret;
//		}
//		ret = pPtData->u16Rightpt;
//		Index = pPtData->u16Leftpt;
//		while(Index < l_16tmpValue)
//		{
//			if(abs(xDistant[Index] - pPtStruct->n32xRight) < l_minValue && (abs(xDistant[Index]-pPtStruct->n32xLeft)>(SMALL_AREA>>1)))
//			{
//				l_minValue = abs(xDistant[Index] - pPtStruct->n32xRight);
//				pPtStruct->n32xRight = xDistant[Index];
//				ret = Index;		
//			}			
//			Index++;	
//		}
//	}
//	else if (vehPosFlag == 2)  //����ߣ�����ֵ
//	{
//		for (; Index < l_16tmpValue-1; Index++)
//		{
//			if ( (!l_u8Flag) && abs(xDistant[Index] - xDistant[Index+1]) > 1000&&
//			    abs(xDistant[Index]-pPtData->n32xLeft) < l_u16LaneWide &&	  //   pPtStruct��ΪpPtData
//				abs(pPtData->n32xRight-xDistant[Index+1]) < l_u16LaneWide )
//			{
//				ret = Index;
//				break;
//			}
//			else if (l_u8Flag && abs(xDistant[Index] - xDistant[Index+1]) > 1000 &&
//			    abs(xDistant[Index]-pPtData->n32xLeft) < l_u16LaneWide &&				  //  pPtStruct��ΪpPtData
//				abs(pPtData->n32xRight-xDistant[Index+1]) > l_u16LaneWide)	  //ʣ�µ�����Ҫ����3.5m
//			{
//				ret = Index;
//				break;				
//			}
//		}		
//		if (ret && ret < l_16tmpValue && ((abs(pPtData->n32xLeft-xDistant[Index])> (SMALL_AREA>>1)) ||
//		    abs(pPtData->u16Leftpt-Index) > MIN_PTNUM) )
//		{
//			return ret;
//		}
//		ret = pPtData->u16Rightpt;
//		Index = pPtData->u16Leftpt;
//
//		if (l_u8Flag)	 //�������7M
//		{
//			while(Index < l_16tmpValue)
//			{
//				if(abs(xDistant[Index] - pPtStruct->n32xRight) < l_minValue && 
//				  (abs(pPtData->n32xLeft-xDistant[Index])> (SMALL_AREA>>1)) &&
//				  abs(pPtData->n32xLeft-xDistant[Index]) < l_u16LaneWide)		   //  ����abs(pPtData->n32xLeft-xDistant[Index]) < l_u16LaneWide)
//				{
//					l_minValue = abs(xDistant[Index] - pPtStruct->n32xRight);
//	//				pPtStruct->n32xRight = xDistant[Index];
//					ret = Index;		
//				}			
//				Index++;	
//			}
//		}
//		else
//		{
//			while(Index < l_16tmpValue)
//			{
//				if(abs(xDistant[Index] - pPtStruct->n32xRight) < l_minValue && (abs(pPtData->n32xLeft-xDistant[Index])> (SMALL_AREA>>1))
//				   && abs(pPtStruct->n32xLeft-xDistant[Index]) < l_u16LaneWide && abs(xDistant[Index+1]-pPtData->n32xRight) < l_u16LaneWide  )
//				{
//					l_minValue = abs(xDistant[Index] - pPtStruct->n32xRight);
//	//				pPtStruct->n32xRight = xDistant[Index];
//					ret = Index;		
//				}			
//				Index++;	
//			}
//		}
//	}
//	else   //���ұ�,����ֵ
//	{
//		for (; Index < l_16tmpValue-1; Index++)
//		{
//			if ( (!l_u8Flag) && abs(xDistant[Index] - xDistant[Index+1]) > 1000&& 
//			   abs(xDistant[Index+1]-pPtData->n32xRight) < l_u16LaneWide &&		   //  pPtStruct��ΪpPtData
//			   abs(pPtData->n32xLeft-xDistant[Index]) < l_u16LaneWide )
//			{
//				ret = Index+1;
//				break;
//			}
//			else if (l_u8Flag&&abs(xDistant[Index] - xDistant[Index+1]) > 1000 && 
//			   abs(xDistant[Index+1]-pPtData->n32xRight) < l_u16LaneWide &&			//	pPtStruct��ΪpPtData
//			   abs(pPtData->n32xLeft-xDistant[Index]) > l_u16LaneWide)	 //
//			{
//				ret = Index+1;
//				break;
//			}
//		}		
//		if (ret && ret < l_16tmpValue && (abs(pPtData->n32xRight-xDistant[Index])> (SMALL_AREA>>1) ||
//		    abs(pPtData->u16Rightpt-Index)> MIN_PTNUM))
//		{
//			return ret;
//		}
//		ret = pPtData->u16Rightpt;
//		Index = pPtData->u16Leftpt;
//
//		if (l_u8Flag)
//		{
//			while(Index < l_16tmpValue)
//			{
//				if(abs(xDistant[Index] - pPtStruct->n32xLeft) < l_minValue && 
//				  (abs(pPtData->n32xRight-xDistant[Index])> (SMALL_AREA>>1)) && 
//				  abs(pPtData->n32xRight-xDistant[Index]) < l_u16LaneWide)			 //����abs(pPtData->n32xRight-xDistant[Index]) < l_u16LaneWide)
//				{
//					l_minValue = abs(xDistant[Index] - pPtStruct->n32xLeft);
//	//				pPtStruct->n32xLeft = xDistant[Index];
//					ret = Index;		
//				}			
//				Index++;	
//			}
//		}
//		else
//		{
//			while(Index < l_16tmpValue-1)
//			{
//				if(abs(xDistant[Index] - pPtStruct->n32xLeft) < l_minValue && (abs(pPtData->n32xRight-xDistant[Index])> (SMALL_AREA>>1))
//				   && abs(pPtData->n32xLeft-xDistant[Index]) < l_u16LaneWide && abs(xDistant[Index+1]-pPtData->n32xRight) < l_u16LaneWide)
//				{
//					l_minValue = abs(xDistant[Index] - pPtStruct->n32xLeft);
//	//				pPtStruct->n32xLeft = xDistant[Index];
//					ret = Index+1;		
//				}			
//				Index++;	
//			}
//		}	
//	}
//	
//	return 	ret;
//}
//


int GetVehSpeed(VehIncSt *pVehInc,int Speedline, int EnterX1)
{
	int i;
	int MinD=0;
    int ret =0;
	int index,index_start=-1;
	int SpeedFlag1=0;
	int SpeedFlag2=0;

	int X1;
	int X2;
	int t1;
	int t2;
	uint16 V;

	int Distance1=2500;  //���루EnterX1+Distanc1��������ĳ�ͷ�Ǵ��֡��Ϊ�����ٶȵĵ�һ֡
	int Distance2=2500;  //����2000cm�����ڵ��ٶȲ������ٶ�

	/*********���㳵���ٶ�**********************/
	////////////////����֡��ͷû�д�ɵ�֡���㳵�����ٶ�///////

	index = pVehInc->Idata.u16FrameCnt;
	SpeedFlag1=0;
	SpeedFlag2=0;
	MinD=10000;
	if (index>=2)
	{
		for(i=0; i<index; i++)			 //�ҵ���һ����ͷû�д�ɵ�֡
		{
			if((pVehInc->Idata.ydataInfo[i][4] ==0)&&(abs(pVehInc->Idata.ydataInfo[i][0]-(EnterX1+Distance1))<MinD))
			{
				MinD=abs(pVehInc->Idata.ydataInfo[i][0]-(EnterX1+Distance1));
				X1 = pVehInc->Idata.ydataInfo[i][0];
				t1 = pVehInc->Idata.tdata[i];
				index_start=i;
			}
		}
		if (index_start!=-1)
		{
			if(index_start<index-1)
			{
				MinD=10000;		  //��ʼֵ��Ϊ10��
				for(i=index_start+1;i<index;i++)
				{
					if((pVehInc->Idata.ydataInfo[i][4]==0)&&(abs(pVehInc->Idata.ydataInfo[i][0]-(X1+Distance2))<MinD))
					{
						MinD = abs(pVehInc->Idata.ydataInfo[i][0]-(X1+Distance2));
						SpeedFlag1=1;
						SpeedFlag2=1;
						X2 = pVehInc->Idata.ydataInfo[i][0];
						t2 = pVehInc->Idata.tdata[i];
					}
				}
			}
		}

		//////////////////
		if(SpeedFlag1==0)
		{
			//    index=Lane_Vertical[m][0][0];
			SpeedFlag2=0;
			for(i=0; i<index; i++)			 //�ҵ���һ����ͷû�д�ɵ�֡
			{
				if(pVehInc->Idata.ydataInfo[i][4]==0)
				{
					X1 = pVehInc->Idata.ydataInfo[i][0];
					t1 = pVehInc->Idata.tdata[i];
					index_start=i;
				}
			}

			if (index_start!=-1)
			{
				if(index_start<index-1)
				{
					MinD=10000;		  //��ʼֵ��Ϊ10��
					for(i=index_start+1;i<index;i++)
					{
						if((pVehInc->Idata.ydataInfo[i][4]==0)&&(abs(pVehInc->Idata.ydataInfo[i][0]-(X1+Distance2))<MinD))
						{
							MinD = abs(pVehInc->Idata.ydataInfo[i][0]-(X1+Distance2));
							SpeedFlag2=1;
							X2 = pVehInc->Idata.ydataInfo[i][0];
							t2 = pVehInc->Idata.tdata[i];
						}
					}
				}
			}
		}

		//////////////////	  
		if(SpeedFlag2==0)
		{
			//	 index=Lane_Vertical[m][0][0];
			MinD=abs(pVehInc->Idata.ydataInfo[index-1][0]-(Speedline-Distance2));
			X1 = pVehInc->Idata.ydataInfo[index-1][0]; 
			t1 = pVehInc->Idata.tdata[index-1];

			for(i=index-2;i>=0;i--)
			{
				if(abs(pVehInc->Idata.ydataInfo[i][0]-(Speedline-Distance2))<MinD)
				{
					MinD=abs(pVehInc->Idata.ydataInfo[i][0]-(Speedline-Distance2));
					X1=pVehInc->Idata.ydataInfo[i][0];
					t1=pVehInc->Idata.tdata[i];
					index_start = i;
				}
			}


			MinD=X1+Distance2;

			if (index_start!=-1)
			{
				X2=pVehInc->Idata.ydataInfo[index_start+3][0];
				t2=pVehInc->Idata.tdata[index_start+3];

				if((index-index_start)>2)	//�������������㣬���ٶ����Ϊ�㣻
				{
					for(i=index_start+1;i<index;i++)	   //�Ҿ��루EnterX2-Distance������ĵ�
					{
						if(abs(pVehInc->Idata.ydataInfo[i][0]-(X1-Distance2))<MinD)
						{
							MinD=abs(pVehInc->Idata.ydataInfo[i][0]-(X1-Distance2));
							X2=pVehInc->Idata.ydataInfo[i][0];
							t2=pVehInc->Idata.tdata[i];  	
						}
					}
				}			
			}
			else
			{
				return 0;
			}
		}

		V=(((X2-X1)*3.6)/(t2-t1))*10000;
		return(V);

	}
	else
	{
		return 0;
	}
}

////����ĳ����Χ(start��end)�ڵ�α�����
int Myrand(int start, int end)
{
	int ret = start;
	//�˴�Ӧʹ��ʱ��
	srand((uint32)RTC_UCOUNT);

	if (end > start)
	{
		ret = (rand() & 0xffff)%(end - start) + start;
	}

	return ret;
}
//
uint8	IsInIncSide(int32 x11, int32 x12, int32 x21, int32 x22)
{
	if (abs(x11-x21)<3000 && abs(x12-x22)<3000)
	{
	   return 1;
	}	
	return 0;
}
//
//
//
////С�ͳ�����������ʱ��ֻ��1��2֡ʱ��������д���
//uint8 ISVehicle(VehicleDataStruct* pVdata)
//{
//	uint8 Ret = 0;
//	uint8 l_u8HeightPt = 1;   //ÿ֡�и߶���ȵĵ���
//	uint8 index = 0;
//	uint8 indexFrame = 0;
//
//	//20140217 ���ӶԲ����Ϸ����ж�
//	if (pVdata == NULL)
//	{
//		return 0;  //Ϊ�գ�����0	
//	}
//
//	for( indexFrame = 0; indexFrame < pVdata->u16FrameCnt; indexFrame++)
//	{
//		for (index = 1;pVdata->zdata[indexFrame][index] > ThresVehLow;index++) //20140226 �޸�
//		{
//			if (pVdata->zdata[indexFrame][index-1] == pVdata->zdata[indexFrame][index])
//			{
//				l_u8HeightPt++;
//			}
//			else
//			{
//				if ( Ret < l_u8HeightPt)
//				{
//					Ret = l_u8HeightPt;
//				}
//				l_u8HeightPt = 1;
//			}
//
//			if (index >= FRAME_BUFLEN - 1)	// ����
//				break;
//		}			
//	}
//
//	//���ںܶ�֡��ɣ���ÿ֡�еĸ���߲���ȵĴ���������������϶࣬��������
//	if (Ret <= 3)
//	{
//		if(max(pVdata->zdata[0][0], pVdata->zdata[1][0]) >= MIN_PTNUM && 
//		   max(pVdata->xMax[0], pVdata->xMax[1]) > 0)  //ֻ�Ƚ�1��2��֡�ĵ����Ϳ���
//		{
//			Ret = 4;
//		}	
//	}//end
//
//	if ( Ret > 3 )
//	{
//		Ret = 1;
//	}
//	else
//	{
//		Ret = 0;
//	}
//
//	return Ret;
//}
//
//
////ÿ֡���г��������Ч�����жϣ�0��ʾ��Ч��1��ʾ��Ч  (��Ҫ�Ľ���
//uint8 ISVehRegion(const uint16 u16RegionWide, const PointStruct *pPtStruct, const int32* pZdistance)
//{
////	uint8 Ret = 0;
////	uint16 l_u16Index = 0;
////	uint16 l_u16HightPtNum = 0;
////	uint16 l_u16LowPtNum   = 0;
////	//20140217 ���ӶԲ����Ϸ����ж�
////	if (pPtStruct == NULL || pZdistance == NULL)
////	{
////		return 0;
////	}
////
////	//���� �����������ڸ߶��쳣���
////	if (pPtStruct->u16Leftpt >= pPtStruct->u16Rightpt || pPtStruct->u16Rightpt >= POINT_SUM )
////	{	//�����쳣
////		return 0;
////	}
////	for (l_u16Index = pPtStruct->u16Leftpt; l_u16Index <= pPtStruct->u16Rightpt; l_u16Index++)
////	{
////		if (pZdistance[l_u16Index] >= 5000) //�߶ȳ���5m�ĵ�
////		{
////			l_u16HightPtNum++;	
////		}
////		else if (pZdistance[l_u16Index] <= -500)	//�߶���-0.5m����Ϊ�쳣
////		{
////			l_u16LowPtNum++;
////		}
////		else
////		{
////		}
////	}
////	if ((l_u16HightPtNum + l_u16LowPtNum) > 5 ||
////		(l_u16HightPtNum + l_u16LowPtNum) >= (pPtStruct->u16Rightpt - pPtStruct->u16Leftpt+1)/2)
////	{
////		//�쳣�϶�
////		return 0;
////	}
////	
////	if (pPtStruct->u16xMaxHt > 3000 &&pZdistance[pPtStruct->u16Leftpt] == pPtStruct->u16xMaxHt) //��ߵ������߶�ֵ
////	{
////		if ((g_sspSetup.u8InstallFlag &&(pPtStruct->n32xLeft-pPtStruct->n32xRight>2000)) ||
////			((g_sspSetup.u16EndPtNum < g_sspSetup.u16VerticalZeroPos)&&
////			(pPtStruct->n32xRight-pPtStruct->n32xLeft>2000)))
////		{//	���ֵ�쳣
////			return 0;	
////		}	
////	}
////	else if (pPtStruct->u16xMaxHt > 3000 && pZdistance[pPtStruct->u16Rightpt] == pPtStruct->u16xMaxHt)//�ұߵ������߶�ֵ
////	{
////		if ((g_sspSetup.u8InstallFlag &&(pPtStruct->n32xLeft-pPtStruct->n32xRight>2000)) ||
////			((g_sspSetup.u16EndPtNum < g_sspSetup.u16VerticalZeroPos)&&
////			(pPtStruct->n32xLeft-pPtStruct->n32xRight>2000)))
////		{//	�ҵ�ֵ�쳣
////			return 0;	
////		}
////	}
////
////	if (u16RegionWide > SMALL_AREA &&((pPtStruct->u16Rightpt - pPtStruct->u16Leftpt+1 >= MIN_PTNUM) && u16RegionWide > 0))
////	    ||(pPtStruct->u16Rightpt - pPtStruct->u16Leftpt + 1 >= MIN_PTNUM*2 && u16RegionWide > 0)
////	{
////		Ret = 1;
////	}	
////
////	return Ret;
//	return 0;
//}
//
///**** ***************************************
//*****20130523  ��ǰ�г�������ǰһ֡��������ƥ��
//**** ���� RegionMatch*********
//***  ����˵��******************
//*** u16LeftX  ��ǰ֡�г��������ʼ��λ��
//*** u16RightX ��ǰ֡�г�����Ľ�����λ�� 
//***	pVehicle �����ṹ�壬��ȫ�ֱ���g_VehicleSet���Ӧ
//*** u8Index  ��������
//******************/
//uint16 RegionMatch(int32 n32LeftX, int32 n32RightX, VehicleStruct *pVehicle, uint16 u16Index)
//{
//////	uint16 RetIndex = u16Index;	
////	//20140217 ���ӶԲ����Ϸ����ж�
////	if (pVehicle == NULL || u16Index > VEHICLE_MASK)
////	{
////		return ERRORVALUE; 
////	}
////
////	if (u16Index == VEHICLE_MAX-1)  //ǰһ֡�����һ���г�����,ֱ��ƥ��ɹ�
////	{
////		RetIndex = u16Index;	
////	}
////	else
////	{  //��ǰ������ǰһ֡����һ������ƥ��
////		if (IS_INSIDE(n32LeftX, n32RightX, pVehicle[u16Index+1].locateX.n32xLeft,pVehicle[u16Index+1].locateX.n32xRight)) //�����ص�
////		{
////			//��ǰ������ǰһ֡���������򶼲���ƥ��
////			if( abs(n32LeftX - pVehicle[u16Index+1].locateX.n32xRight) >= 
////			    abs(pVehicle[u16Index+1].locateX.n32xLeft - n32RightX) ) 
////			{
////				RetIndex = u16Index;
////			}	
////			else
////			{
////				RetIndex = u16Index + 1;
////			}
////		}
////		else
////		{	//��ȫ��ƥ�䣬ֱ�ӷ���
////			RetIndex = u16Index;
////		}
////	}
//	
////	return RetIndex;	
//    return 0;
//}
//

//void RegionMerging(PointSet *pFrameInfo)
//{
//////	uint16 l_u16index = 1;
//////	uint16 l_u16tmp   = 1;
//////	uint16 l_u16TmpIndex = 0;
//////
//////	//20140217 ���ӶԲ����Ϸ����ж�
//////	if (pFrameInfo == NULL || pFrameInfo->u8Sum > POINTSET_CNT)
//////	{
//////		return;
//////	}
//////
//////  //�ϲ����򣬸��ݳ����Ŀ�ȣ���һ����������ڲ��ܳ���2�������ϵĳ��������������ܳ���������	 (��4������Ч)
//////	for (l_u16index = 1;l_u16index < pFrameInfo->u8Sum;l_u16index++)
//////	{
//////		if(abs(pFrameInfo->Ptdata[l_u16index-1].n32xLeft-pFrameInfo->Ptdata[l_u16index-1].n32xRight) >= g_LaneWide ||
//////		   abs(pFrameInfo->Ptdata[l_u16index].n32xLeft-pFrameInfo->Ptdata[l_u16index].n32xRight) >= g_LaneWide)  //��һ������ڳ�����
//////		{
//////			if ((abs(pFrameInfo->Ptdata[l_u16index-1].n32xRight) < g_NearMaxWide &&
//////			    abs(pFrameInfo->Ptdata[l_u16index].n32xRight) < g_NearMaxWide) ||
//////				(abs(pFrameInfo->Ptdata[l_u16index-1].n32xLeft) > g_FarMinWide &&
//////				abs(pFrameInfo->Ptdata[l_u16index].n32xLeft) > g_FarMinWide))  //�ϲ�ͬ�������
//////			{
//////				pFrameInfo->Ptdata[l_u16index - 1].n32xRight = pFrameInfo->Ptdata[l_u16index].n32xRight;		
//////				pFrameInfo->Ptdata[l_u16index - 1].u16Rightpt = pFrameInfo->Ptdata[l_u16index].u16Rightpt;
//////				pFrameInfo->Ptdata[l_u16index - 1].u16xMaxHt = max(pFrameInfo->Ptdata[l_u16index - 1].u16xMaxHt,pFrameInfo->Ptdata[l_u16index].u16xMaxHt);
//////				pFrameInfo->Ptdata[l_u16index - 1].u16xDis = abs(pFrameInfo->Ptdata[l_u16index - 1].n32xRight - pFrameInfo->Ptdata[l_u16index - 1].n32xLeft);
//////				for (l_u16tmp = l_u16index;l_u16tmp < pFrameInfo->u8Sum - 1;l_u16tmp++)
//////				{
//////					memcpy(&(pFrameInfo->Ptdata[l_u16tmp]),&(pFrameInfo->Ptdata[l_u16tmp+1]),sizeof(PointStruct));
//////				}
//////				memset(&(pFrameInfo->Ptdata[l_u16tmp]),0,sizeof(PointStruct));
//////				pFrameInfo->u8Sum--;
//////				l_u16index--;	
//////			}	
//////		}
//////		else if (l_u16index+1 < pFrameInfo->u8Sum &&
//////		         abs(pFrameInfo->Ptdata[l_u16index-1].n32xLeft-pFrameInfo->Ptdata[l_u16index].n32xRight) < g_LaneWide &&
//////				 abs(pFrameInfo->Ptdata[l_u16index].n32xLeft-pFrameInfo->Ptdata[l_u16index+1].n32xRight) < g_LaneWide &&
//////				 ((abs(pFrameInfo->Ptdata[l_u16index-1].n32xRight) < g_NearMaxWide && 
//////				 abs(pFrameInfo->Ptdata[l_u16index+1].n32xRight) < g_NearMaxWide) ||
//////				 (abs(pFrameInfo->Ptdata[l_u16index-1].n32xLeft) > g_FarMinWide &&
//////				 abs(pFrameInfo->Ptdata[l_u16index+1].n32xLeft) > g_FarMinWide))) //3������ͬ�࣬������������С�ڳ�����
//////		{
//////			//����ϲ�ʱ��ѡ��ȽϺ��ʵ�2��������кϲ�
//////			if (abs(pFrameInfo->Ptdata[l_u16index-1].n32xLeft-pFrameInfo->Ptdata[l_u16index].n32xRight) <= 
//////			    abs(pFrameInfo->Ptdata[l_u16index].n32xLeft-pFrameInfo->Ptdata[l_u16index+1].n32xRight))  //�ϲ�ǰ������������
//////			{
//////				pFrameInfo->Ptdata[l_u16index - 1].n32xRight = pFrameInfo->Ptdata[l_u16index].n32xRight;		
//////				pFrameInfo->Ptdata[l_u16index - 1].u16Rightpt = pFrameInfo->Ptdata[l_u16index].u16Rightpt;
//////				pFrameInfo->Ptdata[l_u16index - 1].u16xMaxHt = max(pFrameInfo->Ptdata[l_u16index - 1].u16xMaxHt,pFrameInfo->Ptdata[l_u16index].u16xMaxHt);
//////				pFrameInfo->Ptdata[l_u16index - 1].u16xDis = abs(pFrameInfo->Ptdata[l_u16index - 1].n32xRight - pFrameInfo->Ptdata[l_u16index - 1].n32xLeft);
//////				for (l_u16tmp = l_u16index;l_u16tmp < pFrameInfo->u8Sum - 1;l_u16tmp++)
//////				{
//////					memcpy(&(pFrameInfo->Ptdata[l_u16tmp]),&(pFrameInfo->Ptdata[l_u16tmp+1]),sizeof(PointStruct));
//////				}
//////				memset(&(pFrameInfo->Ptdata[l_u16tmp]),0,sizeof(PointStruct));
//////				pFrameInfo->u8Sum--;
//////				l_u16index--;
//////			}
//////			else   //�ϲ���������������
//////			{
//////				pFrameInfo->Ptdata[l_u16index].n32xRight = pFrameInfo->Ptdata[l_u16index+1].n32xRight;		
//////				pFrameInfo->Ptdata[l_u16index].u16Rightpt = pFrameInfo->Ptdata[l_u16index+1].u16Rightpt;
//////				pFrameInfo->Ptdata[l_u16index].u16xMaxHt = max(pFrameInfo->Ptdata[l_u16index].u16xMaxHt,pFrameInfo->Ptdata[l_u16index+1].u16xMaxHt);
//////				pFrameInfo->Ptdata[l_u16index].u16xDis = abs(pFrameInfo->Ptdata[l_u16index].n32xRight - pFrameInfo->Ptdata[l_u16index].n32xLeft);
//////				for (l_u16tmp = l_u16index+1;l_u16tmp < pFrameInfo->u8Sum - 1;l_u16tmp++)
//////				{
//////					memcpy(&(pFrameInfo->Ptdata[l_u16tmp]),&(pFrameInfo->Ptdata[l_u16tmp+1]),sizeof(PointStruct));
//////				}
//////				memset(&(pFrameInfo->Ptdata[l_u16tmp]),0,sizeof(PointStruct));
//////				pFrameInfo->u8Sum--;
//////				l_u16index--;
//////			}	
//////		}
//////		else if (abs(pFrameInfo->Ptdata[l_u16index-1].n32xLeft-pFrameInfo->Ptdata[l_u16index].n32xRight) < g_LaneWide &&
//////		        ((abs(pFrameInfo->Ptdata[l_u16index-1].n32xRight) < g_NearMaxWide && abs(pFrameInfo->Ptdata[l_u16index].n32xRight) < g_NearMaxWide)||
//////				(abs(pFrameInfo->Ptdata[l_u16index-1].n32xLeft) > g_FarMinWide && abs(pFrameInfo->Ptdata[l_u16index].n32xLeft) > g_FarMinWide)))	//ͬ��2��������С�ڳ�����ֱ�Ӻϲ�
//////		{
//////			pFrameInfo->Ptdata[l_u16index - 1].n32xRight = pFrameInfo->Ptdata[l_u16index].n32xRight;		
//////			pFrameInfo->Ptdata[l_u16index - 1].u16Rightpt = pFrameInfo->Ptdata[l_u16index].u16Rightpt;
//////			pFrameInfo->Ptdata[l_u16index - 1].u16xMaxHt = max(pFrameInfo->Ptdata[l_u16index - 1].u16xMaxHt,pFrameInfo->Ptdata[l_u16index].u16xMaxHt);
//////			pFrameInfo->Ptdata[l_u16index - 1].u16xDis = abs(pFrameInfo->Ptdata[l_u16index - 1].n32xRight - pFrameInfo->Ptdata[l_u16index - 1].n32xLeft);
//////			for (l_u16tmp = l_u16index;l_u16tmp < pFrameInfo->u8Sum - 1;l_u16tmp++)
//////			{
//////				memcpy(&(pFrameInfo->Ptdata[l_u16tmp]),&(pFrameInfo->Ptdata[l_u16tmp+1]),sizeof(PointStruct));
//////			}
//////			memset(&(pFrameInfo->Ptdata[l_u16tmp]),0,sizeof(PointStruct));
//////			pFrameInfo->u8Sum--;
//////			l_u16index--;
//////		}
//////		
//////	}
//return ;
//}
//

//


//int get_vehicle_speed(int m, int Speedline)
//{
////	int i;
////	int MinD=0;
////	int index;
////
////	int	VehSpeed = 0;
////
////	int X1;
////	int X2;
////    int t1;
////	int t2;
////
////	int Distance=800;   //����80cm�����ڵ�ʱ��������ٶ�
//// 
////	index=Lane_Vertical[m][0][0];
////	MinD=abs(Lane_Vertical[m][index-1][4]-Speedline);
////	X2=Lane_Vertical[m][index-1][4]; 
////	t2=Lane_Vertical[m][index-1][0];
////
////	for(i=index-2;i>1;i--)
////	{
////		if(abs(Lane_Vertical[m][i][4]-Speedline)<MinD)
////		{
////			MinD=abs(Lane_Vertical[m][i][4]-Speedline);
////			X2=Lane_Vertical[m][i][4];
////			t2=Lane_Vertical[m][i][0];
////			index=i;
////		}
////	}
////		
////	MinD=Distance;
////	X1=Lane_Vertical[m][index][4];
////	t1=Lane_Vertical[m][index][0];
////	
////	if(index>2)
////	{
////		for(i=index-1;i>1;i--)	   //�Ҿ��루EnterX2-Distance������ĵ�
////		{
////			if(abs(Lane_Vertical[m][i][4]-(Speedline-Distance))<MinD)
////			{
////				MinD=abs(Lane_Vertical[m][i][4]-(Speedline-Distance));
////				X1=Lane_Vertical[m][i][4];
////				t1=Lane_Vertical[m][i][0];  	
////			}
////	 	}
////	}
////	VehSpeed=(((X2-X1)*3.6)/(t2-t1))*1000;
////	return VehSpeed;
//    return 0;
//}
//
//int get_vehicle_length(int m)
//{
////	int index = 0;
////	int VehLength = 0;
////	index=Lane_Vertical[m][0][0];
////    VehLength=(Lane_Vertical[m][index-1][4]-Lane_Vertical[m][index-1][3]);  //��������
////	return VehLength;
//    return 0;
//}
