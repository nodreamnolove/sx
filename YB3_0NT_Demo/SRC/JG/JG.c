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
#include "Task_Sv_Continue.h"
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

#define IS_INSIDE(x,y,dx,dy)	( (min(dx,dy) >= max(x,y) || max(dx,dy) <= min(x,y)) ? 0:1 )   //x,y��dx,dyһ�෵��0


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

int32 Average(const int32 *a,uint8 num)
{
   int32 aver=0;
	uint8 i;

	//20140217 �����ж�
	if (a == NULL )
	{
		return 0;
	}

	for(i=0;i<num;i++)
	{
	    aver+=a[i];
	}
	aver=aver/num;
	return aver;
	
}

void BubbleSort(int *parr, int num)
{ //������������ɨ��
	int i,j;
	int n; 
	uint8 exchange; //������־
	int temp;
	n = num; 
	for(i=0;i<n;i++)
	{ //�����n-1������ 
		exchange=0; //��������ʼǰ��������־ӦΪ�� 
		for(j=n-2;j>=i;j--)
		{
			if(parr[j+1]<parr[j])
			{//������¼ 
				temp = parr[j+1]; 
				parr[j+1] = parr[j]; 
				parr[j] = temp; 
				exchange = 1; //�����˽������ʽ�������־��Ϊ�� 
			}
		} 
		if(!exchange) //��������δ������������ǰ��ֹ�㷨 
			return;
	}
}



void clearVehicleErr()
{
	uint16 i,j,l_u16tmp;
 	if (g_totalVehicle > VEHICLE_MAX)
	{
		g_totalVehicle = 0;
		for (j = 0; j < VEHICLE_MAX; j++)
		{
			if (g_VehicleSet[j].u8Vstate != NO_USED)
				g_totalVehicle++;
		}
	}

	 for(j = 0;j < g_totalVehicle;j++)
	 {
		   i = (g_VehicleSetIndex[j] - 1) & VEHICLE_MASK;
		   if (g_VehicleSet[i].u8Vstate == PASSED_USED)  
		   {
				if (g_VehicleSet[i].Vdata.u16FrameCnt< NORMAL_MAX_EMPTYFRAME)
				{
				   memset(&g_VehicleSet[i],0,sizeof(VehicleStruct));
				   g_VehicleSet[i].u8Vstate = NO_USED;  //�󴥷��ĳ� 
				   for(l_u16tmp = j;l_u16tmp < g_totalVehicle - 1;l_u16tmp++)
					   g_VehicleSetIndex[l_u16tmp] = g_VehicleSetIndex[l_u16tmp+1];
				   
				   g_VehicleSetIndex[g_totalVehicle - 1] = 0;
				   g_totalVehicle--;
				}
		   }
	}
   }

/**************����д********************/
void sendTmpData(int *data,int len)
{  
	uint16  Index = 0,i;
	memset(Tx_Buffer,0,W5100BUFSIZE);
	Tx_Buffer[Index++] = 0x02;
	Tx_Buffer[Index++] = 0x02;
	Tx_Buffer[Index++] = 0x02;
	Tx_Buffer[Index++] = 0x02;

	Tx_Buffer[Index++] = 0x01;

	Tx_Buffer[Index++] = 0x00;
	Tx_Buffer[Index++] = 0x00;
	Tx_Buffer[Index++] = 0x00;

	memcpy(Tx_Buffer+Index, TCP_ScanSendCmd, 16);  

	Tx_Buffer[83] = ((len)>>8)& 0xFF;	   //ȡ��8λ
	Tx_Buffer[84] = (len) & 0xFF;	       //ɨ������*2����2�����ʾһ�����룬ȡ��8λ	
	Index =85;
	for(i=0; i<len; i++)
	{
		 Tx_Buffer[Index++] = (data[i]>>8)& 0xFF;
		 Tx_Buffer[Index++] = data[i] & 0xFF; 
	}

  	SendDataNet(3,Tx_Buffer,(len<<1)+85);
}
//��ԭʼ���ݽ���Ԥ���� ��Դ�ɵĵ���0ֵ����Ĵ��� 20130422
void YuChuLiData_Zero(void)
{
//	//ֻ����Ƕȷ�Χ�ڵ�����
//	uint16  index = 0;
//	uint16  u16ValidData = 800;  //�����������ֵʱ���Ĺ̶�����ֵ����ʾ�õ��г�
//	uint16  u16TmpIndex = 0;
//	uint16  u16Tmp = 0;
//	uint16  u16TmpHeight;    //�߶Ȳ�(��׼ֵ�뵱ǰ��ֵ�Ĳ
//	uint8   u8TmpFlag = 0;   //Ҫ�ı�0ֵ�ı�ʶ��0��ʾ���ı䣬1��ʾ�ı�
//
//	//��ֱ����������
//	index = g_u16VerticalStartAnglePt-1;
//	u16TmpIndex = index + 1;
//	while(index < g_u16VerticalEndAnglePt-2)
//	{  //20130506  ����0��Ϊ10 ��ΪSICK��������ɵĵ�ֵ��3
//		u16TmpHeight = 	(g_Base_data0_Value[index-1] - LMS_data_1[index-1] > u16ValidData) ?  (g_Base_data0_Value[index-1] - LMS_data_1[index-1]) : u16ValidData;
//		if ( g_Base_data0_Value[index]-LMS_data_1[index] > ThresVehLow && LMS_data_1[index] <=ERRPTTHRESHOLD )	//ԭʼ�����д�ɵĵ�Ϊ0ֵ
//		{
//			if (g_Base_data0_Value[index-1]-LMS_data_1[index-1] > ThresVehLow && LMS_data_1[index-1] > ERRPTTHRESHOLD)	 //ǰһ��ֵ�����Ҹ߶Ȳ����350
//			{
//				u16TmpHeight = 	(g_Base_data0_Value[index-1] - LMS_data_1[index-1] > u16ValidData) ?  (g_Base_data0_Value[index-1] - LMS_data_1[index-1]) : u16ValidData;
//				LMS_data_1[index] =  g_Base_data0_Value[index-1] - u16TmpHeight;
//				u16TmpIndex = index;
//			}
//			else if (g_Base_data0_Value[index+1]-LMS_data_1[index+1] > ThresVehLow && LMS_data_1[index+1] > ERRPTTHRESHOLD)  //��һ��ֵ�����Ҹ߶Ȳ����350
//			{
//				u16TmpHeight = 	(g_Base_data0_Value[index+1] - LMS_data_1[index+1] > u16ValidData) ?  (g_Base_data0_Value[index+1] - LMS_data_1[index+1]) : u16ValidData;
//				for (u16Tmp = u16TmpIndex; u16Tmp <= index; u16Tmp++)
//				{
//					LMS_data_1[u16Tmp] =  g_Base_data0_Value[u16Tmp] - u16TmpHeight;
//				}
//				u16TmpIndex = index;
//				u8TmpFlag = 0;	
//			}
//			else if (g_Base_data0_Value[index+1]-LMS_data_1[index+1] <= ThresVehLow && LMS_data_1[index+1] > ERRPTTHRESHOLD)	 //��һ����������߶Ȳ�С��350
//			{
//			    //20130508  �����ɽû�г�ʱ��4�����ɴ���
//				if (index - u16TmpIndex < 5 && index-u16TmpIndex>1)
//				{
//					for (u16Tmp = u16TmpIndex; u16Tmp <= index; u16Tmp++)
//					{
//						LMS_data_1[u16Tmp] =  g_Base_data0_Value[u16Tmp];
//					}				
//				}
//				else if (index-u16TmpIndex == 1)
//				{
//					LMS_data_1[index] =  g_Base_data0_Value[index];
//				}
//				else
//				{
//					for (u16Tmp = u16TmpIndex; u16Tmp <= index; u16Tmp++)
//					{
//						LMS_data_1[u16Tmp] =  g_Base_data0_Value[u16Tmp] - u16ValidData;	   //��ֵΪĬ�ϸ߶Ȳ�
//					}
//				}		
//				u16TmpIndex = index;
//				u8TmpFlag = 0;
//			}
//			else if (g_Base_data0_Value[index+1]-LMS_data_1[index+1] > ThresVehLow && LMS_data_1[index+1] <= ERRPTTHRESHOLD && (!u8TmpFlag))//��һ��Ҳ�Ǵ�ɵĵ�
//			{
//				u16TmpIndex = index;
//				u8TmpFlag = 1;
//			}
//		}
//		else
//		{
//			u16TmpIndex = index;
//		}
//		index++;
//	}
}

/*********************************************************************************************************
** ��������:  GetVehHeight2
** ��������:  ������ƽ����Ϊ�����߶�
** ��ڲ���:  Z��������ָ�룬��ʼλ�ã�����λ��
** ���ڲ���:  �߶�ƽ��ֵ
** ����˵��:
*********************************************************************************************************/
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

/************************************/
 //����ÿ֡���ݵĶ��ξ�ֵ��Ϊ��֡�ĳ���
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

/***********************************************************/
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
/***********************************************************/

/*********************************************************************************************************
** ��������:  GetVehLength
** ��������:  �����ĳ���
** ��ڲ���:  �������k����ƽ���ĸ�����
** ���ڲ���:  ��������ƽ��ֵ
** ����˵��:  ��Ϊ��������ʻ���м�λ��ʱ����õĳ���������׼ȷ��ȡ�м��һ����֡����ƽ����Ϊ�����ĳ���
*********************************************************************************************************/
int GetVehLength(int k,uint8 AveNum)
{

//int Tmpi=0;
//int RealAveNum=0;
//int MidNum_Flag=0;
//int CarLength=0;
//
//int MidNum=0;
//int MinXSum=abs(Lane_Vertical[k][1][3]+Lane_Vertical[k][1][4]); //����һ֡��ͷX�ͳ�βX������͵ľ���ֵ����MinXSum
//
//  
///************************************���������ҵ���С�ĳ�ͷ�ͳ�βX������͵ľ���ֵ***************************/
//   for(Tmpi=2;Tmpi<Lane_Vertical[k][0][0];Tmpi++)
//    {
//     if((Lane_Vertical[k][Tmpi][1]!=0)&&(Lane_Vertical[k][Tmpi][3]<0)&&(Lane_Vertical[k][Tmpi][4]>0)&&(abs(Lane_Vertical[k][Tmpi][3]+Lane_Vertical[k][Tmpi][4])<MinXSum))
//   	  {
//	  MinXSum=abs(Lane_Vertical[k][Tmpi][3]+Lane_Vertical[k][Tmpi][4]);
//	  MidNum=Tmpi;
//	  MidNum_Flag=1;
//	  } 
//	}
///***********************************************************************************************/
//
///******************������ʻ���м�ʱ�����ĳ���ƽ��ֵ*****************************/ 
//if(MidNum_Flag)
// {
//  if(MidNum>(AveNum/2))
//    {
//	if((Lane_Vertical[k][0][0]-MidNum)>=(AveNum/2))		     //ǰ����AveNum/2����
//	 {
//	 for(Tmpi=MidNum-(AveNum/2);Tmpi<=MidNum+(AveNum/2)-1;Tmpi++)
//	   {
//	   CarLength=CarLength+(Lane_Vertical[k][Tmpi][4]-Lane_Vertical[k][Tmpi][3]);		//��ͷX�����ȥ��βX����
//	   RealAveNum++;
//	   }
//	 }
//	else								                   //ǰ��AveNum/2����������AveNum/2����
//	  {
//	  for(Tmpi=MidNum-(AveNum/2);Tmpi<=Lane_Vertical[k][0][0]-1;Tmpi++)
//	   {
//	   CarLength=CarLength+(Lane_Vertical[k][Tmpi][4]-Lane_Vertical[k][Tmpi][3]);		//��ͷX�����ȥ��βX����
//	   RealAveNum++;
//	   }
//	 
//	  }
//	}
//
//  else 																		  
//        if((Lane_Vertical[k][0][0]-MidNum)>=(AveNum/2))	    //ǰ��AveNum/2����������AveNum/2����
//          {
//		  for(Tmpi=1;Tmpi<=MidNum+(AveNum/2)-1;Tmpi++)
//	        {
//	        
//	        }
//		  }
//		else	                                            //ǰ��AveNum/2����������AveNum/2����
//		   {
//		    for(Tmpi=1;Tmpi<=Lane_Vertical[k][0][0]-1;Tmpi++)
//	        {
//	        CarLength=CarLength+(Lane_Vertical[k][Tmpi][4]-Lane_Vertical[k][Tmpi][3]);		//��ͷX�����ȥ��βX����
//	        RealAveNum++;
//	        }
//		   }
//	
//	 CarLength=CarLength/RealAveNum;
// }
// else
//  {
//  MinXSum=abs(Lane_Vertical[k][1][3]);
//  MidNum=1;
//  for(Tmpi=2;Tmpi<Lane_Vertical[k][0][0];Tmpi++)
//	  {
//	  if(abs(Lane_Vertical[k][Tmpi][3])<MinXSum)
//	   {
//	   MinXSum=abs(Lane_Vertical[k][Tmpi][3]);
//	   MidNum=Tmpi;
//	   }
//	  }
//  CarLength=Lane_Vertical[k][MidNum][1];
//
//  }
//     return CarLength; 

	return 0;
 } 

 /***********************��ȡ�����������ݽ���************************************/
//u8StarttEndFlag 0��ʾ��ʼ�㣬1��ʾ�����㣻u8VIFlag 0��ʾ��ֱ��1��ʾ��б
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
		u16TmpZeroPos = g_sspSetup.u16VerticalZeroPos0;
		u16StartPtNum= g_sspSetup.u16StartPtNum0;
		u16EndPtNum=g_sspSetup.u16EndPtNum0;
		u8InstallFlag=0;	
	}
	else if(u8JGIndx==1)
	{
	 	u16TmpZeroPos = g_sspSetup.u16VerticalZeroPos1;
		u16StartPtNum= g_sspSetup.u16StartPtNum1;
		u16EndPtNum=g_sspSetup.u16EndPtNum1;
		u8InstallFlag=0;
	}
	else if(u8JGIndx==2)
	{
	 	u16TmpZeroPos = g_sspSetup.u16VerticalZeroPos2;
		u16StartPtNum= g_sspSetup.u16StartPtNum2;
		u16EndPtNum=g_sspSetup.u16EndPtNum2;
		u8InstallFlag=1;
	}
	else if(u8JGIndx==3)
	{
	 	u16TmpZeroPos = g_sspSetup.u16VerticalZeroPos3;
		u16StartPtNum= g_sspSetup.u16StartPtNum3;
		u16EndPtNum=g_sspSetup.u16EndPtNum3;
		u8InstallFlag=1;
	}

	if (u16TmpZeroPos >= POINT_SUM)
		return ERRORVALUE;

	if (u8InstallFlag)  //��װ��ʽ
	{
		if (startPt <= u16TmpZeroPos)	 //��ʼ��	С����������ĵ�
		{
			for (index = startPt; index < u16TmpZeroPos; index++)	
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
			for (index = startPt; index > u16TmpZeroPos; index--)	 //��ֹ��   
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
/*******************�������ִ��ͳ�����Ҫ�Ǵ�ͳ�3�����ͻ���4,���ͻ���5���ش��ͻ���6����װ��7)********************/

uint8 GetLargeVehPattern(VehicleStruct *pVeh)
{
//	VehicleStruct *pVehicle = pVeh;
//	uint8   l_u8DafeiFrm    = 0;  //���֡��־ ��1��ʾ��֡�ж�Ϊ��ɵģ� 0����
//	uint8   RetPattern       = 0;
//	uint8   i,j,k;
//	uint8   l_u8DaFeiFrameCnt = 0;  //��¼�������д�ɵ�֡��
//	uint8   l_u8DaFeiFrameCnt2 = 0;
//	uint8   l_u8EqualNum      = 0;   //��¼ÿ֡���ж��ٸ��൱�ĸ߶�ֵ�����ڴ��֡���ж�
//	uint8   l_u8TmpVehNum     = 0;
//	uint8   l_u8QianShiJing   = 0;   //����ǰ�Ӿ��ϵ�֡��־�� 1��ʾҪ�޳���֡
//	uint8   l_u8DownThresVehLow = 0;  //��¼��ֵ���µĵ�ĸ���
//
//	uint8   Toupos           = 0;    //��ͷ�복��ֽ�λ��
//	uint8   Shenpos          = 0;    //����ʼλ�ã���Ҫ���ڼ�װ�䣩
//	uint8   CheDingStartPos  = 0;    //������ʼλ��
//	uint8   IStou            = 0;
//	uint8 StartDing=0;	
//
//	uint8 l_u32index,l_u32index2,l_u32index3;
//	uint8 l_u8CheTouStart;
//	uint8 l_u8CheTouEnd=0;
//	int32 l_nCheTouHeight=0;
//	uint8 l_u8ShuiNiGuanFlag=0;
//
//	uint8   Veh_Num          = 0;
//	int32   VehLength        = 0;
//	int32   VehHeight        = 0;
//	int32   VehWide          = 0;
//	int32 l_u32Height1,l_u32Height2,l_u32Height3;
//	uint32 l_u32Wide1,l_u32Wide2,l_u32Wide3;
//
//	uint8  dakeche     = 0;  //��ͳ���ʶ��1��ʾ�Ǵ�ͳ�
//	uint8  Not_keche = 0;  //���Ǵ�ͳ���ʶ ��1��ʾ����
//	uint8  u8Lane      = 0;	//����
//
//	uint8  l_u8ChetouDafeiPt = 0;
//
//	uint8  jizhuangxiang = 0;
//	uint8  jizhuangxiang_count = 0;
//	uint8  jizhuangxiang_diffcnt = 0;   //��¼�жϼ�װ��ʱ����֡�ĸ߶Ȳ����
//	uint8  u16Index            = 0;
//	uint8  tmpFlag     = 0;
//	uint16 l_u16HeightThresh = 0;
//	uint16 tmp1   = 0;
//	uint16 tmp2   = 0;
//	uint16  l_u8Pos  = 0;
//	uint32	TouHeight =0;	  //��ʽ������ͷ��
//	uint32	TouWide =0;		  //��ʽ������ͷ��
//	uint32	TouGao =0;		  //�����������ͷ��
//	uint32	TouKuan =0;		  //�����������ͷ��
//	uint8   l_u8HighCount = 0;
//	uint8   u8Flag          = 1;
//	uint8   l_u8StartFrame  = 0;	
// 	int32   ENDRatio=750;
//	int32   l_n32DakeHeightThr = 4000;   //��ͳ��߶���ֵ						 
//	int32   l_n32AllSDThr    = 500;     //ȫ������ֵ
//	int32   l_n32MultiSDThr  = 720;   //��㳵������ֵ
//	int32   allSDcha         =0;    //ȫ����
//	int32   MultiCheshenSD   =0;    //��㳵����
//	int32   SigleCheshenSD=0;
//    int     l_nDaKeHeightThr = 3950;
//	int     l_nDaKeThreshhold11 = 500; 
//    int     l_nDaKeThreshhold12 = 700;
//	int     l_nJZXThreshhold11 = 150;  //��50��Ϊ150   �����ĵ��㳵����Ƚ�
//	int     l_nJZXThreshhold12 = 245;   //��50��Ϊ245   �����Ķ�㳵����Ƚ�
//	int     l_nDaKeThreshhold21 = 400;
//	int     l_nDaKeThreshhold22 = 500;
//	int     DakecheFrameCnt = 12;
//
//	static int32 Z[FRAME_MAXCNT][FRAME_BUFLEN]={0};	 //
//    static int32 Z3[FRAME_MAXCNT][40]={0};
//    static int32 Heightfangcha[FRAME_MAXCNT]={0}; 
//	static int32 duodianduicha[FRAME_MAXCNT]  ={0};	   //�����㷽����м�ֵ
//	static int32 Height[FRAME_MAXCNT] = {0};   //���ڼ���ʱʹ�õĳ���
//	int32   tmpHeight        = 0;
//	int32   Widefangcha      = 0;   //����������ͳ��ĳ�����
//
//	uint8	VehEnd_NumIndex	= 0;
//	uint8   l_u8TotalVehNum     = 0;
//	int		X1,X2;
//
//	memset(Z, 0, sizeof(Z));
//	memset(Z3, 0, sizeof(Z3));
//	memset(Heightfangcha, 0, sizeof(Heightfangcha));
//	memset(duodianduicha, 0, sizeof(duodianduicha));
//	memset(Height, 0, sizeof(Height));
//
//	//20140217  ���ӶԲ����ж�
//	if (pVeh == NULL)
//	{
//		return ZHONGXIAOKE;
//	}
//
//	Veh_Num          = pVehicle->Vdata.u16FrameCnt;
//	if (Veh_Num > FRAME_MAXCNT || Veh_Num < 1)
//	{
//		Veh_Num = 0;  //֡���������֡��ֵ��֡����ֵΪ0
//		pVehicle->Vdata.u16FrameCnt = 1;
//	}
//	VehLength        = pVehicle->yLen;
//	VehHeight        = pVehicle->zLen;
//	VehWide          = pVehicle->xLen;
//
//
//	l_u8TotalVehNum = Veh_Num;
//	if(g_sspSetup.u8RoadType)
//	{
//	    DakecheFrameCnt = 10;
//	}
//	else
//	{
//		DakecheFrameCnt = 15;				
//	}
//
//	//���޳���Щ�н϶��ɵ��֡
//	for (i = 0; i < Veh_Num; i++)
//	{	
//		for (j = 1; j <= pVehicle->Vdata.zdata[i][0]; j++)  //erro3   2.24 ������ѭ��,���޸�
//		{
//			//�޳���ǰ�Ӿ��Գ��ͼ����Ӱ��
//			if (((pVehicle->Vdata.zdata[i][0] <= 6) && (pVehicle->Vdata.zMax[i] > 2000) && (i < 4))
//				|| (pVehicle->Vdata.zMax[i] < 1200 && i < 4)) //���߸߶�С��1200
//			{
//				//ֻ���ǰ4֡������С��6���߶Ƚϸ�,ֱ���޳���֡
//				l_u8QianShiJing = 1;
//				break;
//			} 
//			else if ((pVehicle->Vdata.zdata[i][j] <= ThresVehLow) &&
//			         (pVehicle->Vdata.zMax[i] > 2000) && (i < 4))  // ֻ���ǰ4֡���߶Ƚϸߣ���¼��߶�����ֵ���µĸ���
//			{
//				l_u8DownThresVehLow++;
//			}
//			else
//			{
//			}
//			//Ѱ�Ҵ�ɵ�
//			if (((pVehicle->Vdata.zdata[i][j] == pVehicle->Vdata.zdata[i][j+1]) ||  //ǰ��2�������
//				(l_u8EqualNum>=3 && (0==pVehicle->Vdata.zdata[i][j+1])) 
//				||(l_u8EqualNum>=3 && (pVehicle->Vdata.zdata[i][j]!=pVehicle->Vdata.zdata[i][j+1])&& (pVehicle->Vdata.zdata[i][j+2]==pVehicle->Vdata.zdata[i][j+1]))
//				||(l_u8EqualNum<1&&(pVehicle->Vdata.zdata[i][j] == 0 && pVehicle->Vdata.zdata[i][j+1] !=0))) && (j <pVehicle->Vdata.zdata[i][0])) //20140320��������ȵĵ���ڣ��ҽ����и߶���0ֵ��
//			{
//				if (i < Veh_Num/4 && pVehicle->Vdata.zdata[i][j] >= pVehicle->Vdata.zMax[i])	//����ͳ���ͷ������Ǹ�֡�����߶ȴ�
//				{
//					l_u8ChetouDafeiPt++;			
//				}
//				l_u8EqualNum++;
//			}
//			else
//			{
//				if ((pVehicle->Vdata.zdata[i][0] > 15 && l_u8EqualNum+1 >= pVehicle->Vdata.zdata[i][0]/3) ||
//					(pVehicle->Vdata.zdata[i][0] <=15 && l_u8EqualNum>=4)  //	15�������� �г���4�������Ϊ���
//					) //����8����ֱ����Ϊ��֡�����Ǵ��	  || (l_u8EqualNum >= 8)
//				{ //��Ϊ��֡���Զ���,ֱ������
//					l_u8DafeiFrm = 1;
//					l_u8DaFeiFrameCnt++;
//					break;	
//				}
//				
//				if (l_u8ChetouDafeiPt >= 4)	
//				{
//					l_u8DafeiFrm = 1;
//					l_u8DaFeiFrameCnt++;
//					break;	
//				}
//				l_u8EqualNum = 0;
//				l_u8ChetouDafeiPt = 0;	
//			}
//		
//		}
//		if (l_u8DownThresVehLow>5)
//		{   //��֡С����ֵ�ĵ�������5
//			l_u8QianShiJing = 1;
//		}
//
//		if ((l_u8QianShiJing) || (l_u8DafeiFrm)||l_u8ChetouDafeiPt)//��Ϊ��֡���Զ���
//		{	
//		}
//		else
//		{
//			Height[l_u8TmpVehNum] = pVehicle->Vdata.zMax[i];
//			memcpy(&Z[l_u8TmpVehNum++][0], &(pVehicle->Vdata.zdata[i][0]), sizeof(int32)*(pVehicle->Vdata.zdata[i][0]+1));
//		}
//		l_u8DafeiFrm        = 0;
//		l_u8QianShiJing     = 0;
//		l_u8DownThresVehLow = 0;
//		l_u8EqualNum        = 0;
//		l_u8ChetouDafeiPt   = 0;
//	}
//	if (Veh_Num != l_u8TmpVehNum)  //����ȣ������޳���֡  �����Ҹ�
//	{
//		VehHeight = 0;
//		for (i = 0; i < l_u8TmpVehNum; i++)
//		{
//			if (VehHeight < Height[i])
//			{
//				VehHeight = Height[i];
//			}
//		}
//	}
//	Veh_Num = l_u8TmpVehNum;
//	//�޳����
//
// //���㳵��
//	l_u32Height1 = 0;
//	l_u32Height2 = 0;
//	if (l_u8TotalVehNum>10 && l_u8TmpVehNum>6 && l_u8TmpVehNum>l_u8TotalVehNum/4)
//	{
//		for (i=0; i<l_u8TmpVehNum-1; i++)
//		{
//			if (i<=l_u8TmpVehNum/3)
//			{
//				l_u32Height1 = l_u32Height1 + Height[i];
//			}
//			else if (i>=l_u8TmpVehNum*2/3)
//			{
//				l_u32Height2 = l_u32Height2 + Height[i];
//			}
//		}
//		l_u32Height1 /=(l_u8TmpVehNum/3+1);
//		l_u32Height2 /=(l_u8TmpVehNum-1-l_u8TmpVehNum*2/3); 	
//	}
//
//    //Ϊ�˷�ֹ��ͳ���ɵ�֡���ܶ�����ֱ���жϴ�ͳ��ķ���
// 	if (((l_u32Height2>0 && l_u32Height1>0 && abs(l_u32Height2-l_u32Height1)<800)|| (0 == l_u32Height2 && 0 == l_u32Height1))
//		&& Veh_Num < pVehicle->Vdata.u16FrameCnt/2 && pVehicle->Vdata.u16FrameCnt > 12 && pVehicle->zLen > 2500 )
//	{
//		RetPattern = DAKECHE;
//	}
//	else if (Veh_Num <=5 && pVehicle->zLen > 2700)
//	{
//		RetPattern = ZHONGXIAOKE;
//	}
// 	else if (Veh_Num <= 5) //5֡������ֱ������С�ͳ�
//	{
//		RetPattern = ZHONGXIAOKE;
//	}
//	else
//	{
//	     k=0; 
//		 if(pVehicle->locateX.u16Rightpt < g_sspSetup.u16VerticalZeroPos)
//		 {
//             for(i=0;i<Veh_Num;i++)
//	         {	
//		         k=0;
//				 l_u16HeightThresh = (Height[i]*4/5 > 1500)	? (Height[i]*4/5) : 1500;
//	             for(j=0;j<Z[i][0]*(1000-ENDRatio)/1000;j++)
//	             {
//		            if (Z[i][j] > l_u16HeightThresh && k < 39)
//				    {
//				        Z3[i][k]= Z[i][j];//��ÿ֡�к�����֮һ�����Z3�У�0λ�ô泵�����һ����,�쳣��ȥ��
//					    k++;
//					    Z3[i][39]= k;
//			        }
//				    if(k >= 39)
//				    {
//				        break;
//				    }
//		         }
//	         }
//		 }
//		 else
//		 {
//		     for(i=0;i<Veh_Num;i++)
//	         {	
//		         k=0;
//				 l_u16HeightThresh = (Height[i]*4/5 > 1500)	? (Height[i]*4/5) : 1500;
//	             for(j=Z[i][0];j>Z[i][0]*ENDRatio/1000;j--)
//	             {
//		            if (Z[i][j] > l_u16HeightThresh && k < 39)
//				    {
//				        Z3[i][k]= Z[i][j];//��ÿ֡�к�����֮һ�����Z3�У�0λ�ô泵�����һ����,�쳣��ȥ��
//					    k++;
//					    Z3[i][39]= k;
//			        }
//				    if(k >= 39)
//				    {
//				        break;
//				    }
//		         }
//	         }
//		 }
//	   //���㳵ͷ�ĸ߶Ⱦ�ֵ
//	   l_u8CheTouStart = 0;
//	   for ( i = 1; i < Veh_Num/3; i++)
//	   {
//	   		if (abs(Height[i]-Height[i-1])< 100 && abs(Height[i]-Height[i+1])< 100 && Height[i-1] > 1800)
//			{
//				l_u8CheTouStart = i-1;	  //��ͷ��ʼ��
//				break;
//			}
//	   }
//	   for ( i = l_u8CheTouStart; i < Veh_Num/3; i++)	  //Ѱ�ҳ�ͷ�Ľ�����
//	   {
//	   		if (abs(Height[i]-Height[i+1]) > 200)
//			{
//				l_u8CheTouEnd = i;
//				if (l_u8CheTouEnd - l_u8CheTouStart > 4)
//				{
//					l_u8CheTouEnd = l_u8CheTouStart + 4;
//				}
//				break;
//			}
//	   }
//	   //���㳵ͷ�ľ�ֵ�߶�
//	   if (l_u8CheTouEnd - l_u8CheTouStart + 1 >= 2 && l_u8CheTouEnd >= 1)
//	   {
//	   		if (l_u8CheTouStart >= 10)
//			{
//				l_nCheTouHeight =  Height[l_u8CheTouStart];
//			}
//			else
//			{
//				for(i=l_u8CheTouStart;i<=l_u8CheTouEnd;i++)
//				{
//					
//					if(i > 10)
//					{
//						break;
//					}
//					l_nCheTouHeight += Height[i];
//				}
//				l_nCheTouHeight = l_nCheTouHeight / (i - l_u8CheTouStart);     //��i>10ʱ����l_u8CheTouEnd��ĸ߶Ȳ�׼ȷ
//			}
//	   }
//
//	   //Ѱ�ҳ�����ʼ��λ��
//	   for (i = 1; i < Veh_Num/2; i++)		  //����i<    20131216
//	   {
//	   		if (abs(Height[i]-Height[i-1]) < 100 &&	abs(Height[i]-Height[i+1]) < 100 && Height[i-1] > 2000)
//			{
//				StartDing = i-1;
//				break;
//			}
//	   }
//	   if (g_sspSetup.u8RoadType && StartDing < 2)	 //����·��С��2�Ĵ�2��ʼ
//	   {
//	   		StartDing = 2;
//	   }
//
//	   //Ѱ�ҳ�ͷ�복��ķֽ��
//	   tmp1 = 0;
//	   if (Veh_Num/2 > 4)
//	   {
//	   		for (i = 4; i < Veh_Num/2; i++)
//			{
//				if (Height[i] < 2000)
//				{
//					IStou = 1;
//					Toupos = i;
//					break;
//				}
//			}
//			for (i = 1; i < Veh_Num/2; i++)
//			{
//				if (Height[i]-Height[i+1] > 400)
//				{
//					IStou = 1;
//					Toupos = i;
//					break;
//				}
//				else if (i>=3 && Height[i]-Height[i+1] < -300 )
//				{
//					IStou  = 1;
//					Toupos = i;
//					break;
//				}
//
//				if (abs(Height[i]-Height[i+1])<100)//20140811
//				{
//					tmp1++;
//				}
//				else if (Height[i]-Height[i+1]<-300 && tmp1>0)
//				{
//					IStou = 1;
//					Toupos = i;
//					break;
//				}
//			}
//	   }
//		if (IStou == 1)
//		{
//			for(i = 1; i<=Toupos; i++)
//			{
//				TouHeight += Height[i];
//				TouWide += pVehicle->Vdata.xMax[i];
//			}
//			TouHeight =  TouHeight/Toupos;			   //���㳵ͷ�ߺͳ�ͷ��Ϊʶ����ʽ���� 
//			TouWide = TouWide/Toupos;		
//			for(i=Toupos;i<Veh_Num/2;i++)
//			{
//				if (abs(Height[i]-Height[i-1])<200 && abs(Height[i]-Height[i+1])<200&& Height[i-1]>3000)
//				{
//					Shenpos=i-1;	 //����λ�ã���Ҫ��Ϊ��װ�����
//					break;
//				}
//			}
//		}
//
//		//���㳵��ĵ��㷽��
//		if (Shenpos>0 && Shenpos < Veh_Num-1)
//		{
//			j=0;
//			for (i=Shenpos;i<Veh_Num-1;i++)
//			{
//				j=j+1;
//				Heightfangcha[i-Shenpos]=(Height[i]-VehHeight)*(Height[i]-VehHeight);
//			}
//			SigleCheshenSD=Average(Heightfangcha,j);
//			SigleCheshenSD=SigleCheshenSD/100;
//		}
//		else
//		{
//			j=0;
//			for (i=Veh_Num/3;i<Veh_Num-1;i++)
//			{
//				Heightfangcha[i-Veh_Num/3]=(Height[i]-VehHeight)*(Height[i]-VehHeight);
//				j++;
//			}
//			SigleCheshenSD=Average(Heightfangcha,j);
//			SigleCheshenSD=SigleCheshenSD/100;
//		}
//		//�����㳵����
//		j=0;
//		k=0;
//		for (i=0;i<Veh_Num;i++)
//		{
//			for (j=0;j<Z3[i][39];j++)
//			{
//				duodianduicha[i]=duodianduicha[i] + (Z3[i][j]-VehHeight)*(Z3[i][j]-VehHeight);
//			}
//			if (Z3[i][39] > 0)
//			{
//				duodianduicha[i]=duodianduicha[i]/Z3[i][39];
//			}
//			else
//			{
//				duodianduicha[i] = 0;
//			}
//		}
//		if (Shenpos > 0)
//		{
//			MultiCheshenSD = Average(&duodianduicha[Shenpos],Veh_Num-Shenpos-1);
//		}
//		else
//		{
//			MultiCheshenSD = Average(&duodianduicha[Veh_Num/3],Veh_Num-Veh_Num/3-1);
//		}
//		MultiCheshenSD=MultiCheshenSD/100;
//		//����ȫ��ķ���
//		allSDcha=0;
//		if (StartDing>0)
//		{
//			for (i=StartDing;i<Veh_Num-1;i++)//StartDing
//			{
//				allSDcha=allSDcha+((Height[i]-VehHeight)*(Height[i]-VehHeight));
//			}
//			allSDcha=allSDcha/((Veh_Num-StartDing-1)*100);
//		}
//		else
//		{
//			for (i=1;i<Veh_Num-1;i++)
//			{
//				allSDcha=allSDcha+((Height[i]-VehHeight)*(Height[i]-VehHeight));
//			}
//			allSDcha=allSDcha/((Veh_Num-2)*100);
//		}
//	    //����ǰ����֮һ��ȡ��߶�ƽ����������֮����ȸ߶�ƽ�����Լ�����ƽ����
//		//ǰ����֮һ��ȸ߶�ƽ��
//		l_u32Wide1 = 0;
//		l_u32Height1 = 0;
//		l_u32index2 = 0;
//		l_u32index3 = 0;
//		for(l_u32index = 1;l_u32index < Veh_Num/3;l_u32index++)
//		{
//		    if(pVehicle->Vdata.xMax[l_u32index] < 3000)
//			{
//			    l_u32Wide1 += pVehicle->Vdata.xMax[l_u32index];
//				l_u32index2++;
//			}
//			if(Height[l_u32index] < 5000)
//			{
//			    if(l_u32index <= 2 && Height[l_u32index] > 700)	  //ljj�޸� л��@20130809
//				{
//					l_u32Height1 += Height[l_u32index];
//					l_u32index3++;				
//				}
//				else if(l_u32index >= 3)
//				{
//					l_u32Height1 += Height[l_u32index];
//					l_u32index3++;
//				}
//			} 	    
//		}
//		if(l_u32index2 > 0)
//		{
//		    l_u32Wide1 = l_u32Wide1/l_u32index2;
//		}
//		if(l_u32index3 > 0)
//		{
//		    l_u32Height1 = l_u32Height1/l_u32index3;
//		}
//	
//		//������֮����ȸ߶�ƽ��
//		l_u32Wide2 = 0;
//		l_u32Height2 = 0;
//		l_u32index2 = 0;
//		l_u32index3 = 0;
//		for(l_u32index = Veh_Num/3;l_u32index < Veh_Num - 1;l_u32index++)
//		{
//		    if(pVehicle->Vdata.xMax[l_u32index] < 3000)
//			{
//			    l_u32Wide2 += pVehicle->Vdata.xMax[l_u32index];
//				l_u32index2++;
//			}
//			if(Height[l_u32index] < 5000)
//			{
//			    	l_u32Height2 += Height[l_u32index];
//					l_u32index3++;
//			} 	    
//		}
//		if(l_u32index2 > 0)
//		{
//		    l_u32Wide2 = l_u32Wide2/l_u32index2;
//		}
//		if(l_u32index3 > 0)
//		{
//		    l_u32Height2 = l_u32Height2/l_u32index3;
//		}
//		//�����ȸ߶�ƽ��
//		l_u32Wide3 = 0;
//		l_u32Height3 = 0;
//		l_u32index2 = 0;
//		l_u32index3 = 0;
//		for(l_u32index = 1;l_u32index < Veh_Num - 1;l_u32index++)
//		{
//		    if(pVehicle->Vdata.xMax[l_u32index] < 3000)
//			{
//			    l_u32Wide3 += pVehicle->Vdata.xMax[l_u32index];
//				l_u32index2++;
//			}
//			if(Height[l_u32index] < 5000)
//			{
//			    l_u32Height3 += Height[l_u32index];
//				l_u32index3++;
//			} 	    
//		}
//		if(l_u32index2 > 0)
//		{
//		    l_u32Wide3 = l_u32Wide3/l_u32index2;
//		}
//		if(l_u32index3 > 0)
//		{
//		    l_u32Height3 = l_u32Height3/l_u32index3;
//		} 
//
//		//�����ȷ���	 ���ڴ������
//		l_u32index2 = 0;
//		Widefangcha = 0;
//		if (g_sspSetup.u8RoadType) //����·�����ٽϿ�
//		{
//			if (pVehicle->xLen >= 2000)//������2000������
//			{
//				for (l_u32index = 2; l_u32index < Veh_Num /2 ; l_u32index ++)
//				{	
//					if( pVehicle->Vdata.xMax[l_u32index] < 2800)	  //�������³���Ƚϴ�
//					{
//					   l_u32index2++ ;
//					   Widefangcha += (pVehicle->Vdata.xMax[l_u32index]-l_u32Wide2)*(pVehicle->Vdata.xMax[l_u32index]-l_u32Wide2);
//					}
//					
//				}
//			}
//		}
//		else   //��ͨ·
//		{
//
//			for (l_u32index = 2; l_u32index < Veh_Num /2 ; l_u32index ++)
//			{	
//				if( pVehicle->Vdata.xMax[l_u32index] < 2800)	  //�������³���Ƚϴ�
//				{
//				   l_u32index2++ ;
//				   Widefangcha += (pVehicle->Vdata.xMax[l_u32index]-l_u32Wide2)*(pVehicle->Vdata.xMax[l_u32index]-l_u32Wide2);
//				}
//				
//			}
//			
//		}
//		if(l_u32index2 > 0)
//		{
//		   Widefangcha = Widefangcha / l_u32index2 /100;
//		}
//		else
//			Widefangcha	 = 1000;
//
////�ҳ��������ɵ���ȷ����ͳ�
////��֡�Ľ���		
//		VehEnd_NumIndex = 0;
//		if (IStou!=1 && allSDcha>l_nDaKeThreshhold11 && MultiCheshenSD>l_nDaKeThreshhold12)
//		{
//			for	(i=l_u8TotalVehNum*2/3; i< l_u8TotalVehNum; i++)
//			{
//				if (VehHeight>2000 && (2*(pVehicle->Vdata.zdata[i-1][0]+2) >= 3*pVehicle->Vdata.zdata[i][0]) 
//					&& pVehicle->Vdata.zMax[i-1]-pVehicle->Vdata.zMax[i]>1000)
//				{
//					if (i>=l_u8TotalVehNum-2)
//					{
//						VehEnd_NumIndex = i;
//						break;					
//					}					
//					
//				}							
//			}
//		
//		}
//		else		//β֡�е������� 20140809
//		{
//			tmp1 = 0;
//			if (VehHeight>2000)
//			{
//				for (i=0; i< l_u8TotalVehNum; i++)
//				{
//					if (pVehicle->Vdata.zdata[i][0]>tmp1)
//					{
//						tmp1 = 	pVehicle->Vdata.zdata[i][0];
//					}					
//				}
//				for	(i=l_u8TotalVehNum/3; i< l_u8TotalVehNum; i++)
//				{
//					if (pVehicle->Vdata.zdata[i][0]<=tmp1/2 && pVehicle->Vdata.zMax[i] + 1000 <VehHeight)
//					{
//						VehEnd_NumIndex = i;
//						break;
//					}
//				}			
//			}
//		}
//
//		VehEnd_NumIndex = (VehEnd_NumIndex!=0) ? VehEnd_NumIndex : l_u8TotalVehNum;
//		if (IStou==0 && VehHeight>2500)
//		{
//			l_u8DaFeiFrameCnt = 0;
//			l_u8DaFeiFrameCnt2 = 0;
//			for	(i=l_u8TotalVehNum/3; i< VehEnd_NumIndex; i++)
//			{
//				l_u32index2 = 0;
//				l_u32index3 = 0;
//				l_u8EqualNum = 0;
//				X1 = abs(pVehicle->Vdata.xdata[i][pVehicle->Vdata.zdata[i][0]]);
//				X2 = abs(pVehicle->Vdata.xdata[i][1]);
//				l_u32index = (X1<X2) ? pVehicle->Vdata.zdata[i][0] : 1;
//				if (l_u32index != 1)
//				{
//					for (j=l_u32index; j>=1; j--)
//					{
//						if (pVehicle->Vdata.zdata[i][j] >500 && pVehicle->Vdata.zdata[i][j] > pVehicle->Vdata.zMax[i]*2/3)
//						{							
//							break;
//						}
//						l_u32index2++;
//
//					}
//					if (l_u32index2>5)
//					{
//						tmp1 = 0;
//						tmp2 = 0;
//						for(j=l_u32index; j>=1; j--)
//						{
//						   if (pVehicle->Vdata.zdata[i][j]>=pVehicle->Vdata.zMax[i]/2 && pVehicle->Vdata.zdata[i][j]<=pVehicle->Vdata.zMax[i]*9/10)
//						   {
//								if (((pVehicle->Vdata.zdata[i][j] == pVehicle->Vdata.zdata[i][j-1])   //ǰ��2������� 
//									||(l_u8EqualNum>=1 && (pVehicle->Vdata.zdata[i][j]!=pVehicle->Vdata.zdata[i][j-1])&& (pVehicle->Vdata.zdata[i][j-2]==pVehicle->Vdata.zdata[i][j-1]))
//									||(l_u8EqualNum<1&&(pVehicle->Vdata.zdata[i][j] == 0 && pVehicle->Vdata.zdata[i][j-1] !=0))) && (j <pVehicle->Vdata.zdata[i][0]-1)) //20140320��������ȵĵ���ڣ��ҽ����и߶���0ֵ��
//								{
//									l_u8EqualNum++;
//								}
//								else
//								{
//									if (l_u8EqualNum<4)
//									{
//										if (++tmp2>=2)
//										{
//											l_u8EqualNum = 0;
//										}									
//									}
//
//								}													   
//						   }
//						   else if (pVehicle->Vdata.zdata[i][j]>=pVehicle->Vdata.zMax[i]*9/10)
//						   {
//						   		if (++tmp1>=3)
//								{
//									break;								
//								}
//
//						   }
//						   else
//						   {
//						   		continue;
//						   }
//						   l_u32index3++;
//						}
//						if (l_u8EqualNum>=4 && l_u8EqualNum>=(l_u32index3-tmp1)/2)
//						{
//						   l_u8DaFeiFrameCnt++;
//						}
//						else if (l_u8EqualNum>=2)
//						{
//						   l_u8DaFeiFrameCnt2++;
//						}				
//					}				
//								
//				}
//				else
//				{
//					for (j=l_u32index; j<=pVehicle->Vdata.zdata[i][0]; j++)
//					{
//						if (pVehicle->Vdata.zdata[i][j] >500 && pVehicle->Vdata.zdata[i][j] > pVehicle->Vdata.zMax[i]*2/3)
//						{						
//							break;
//						}
//						l_u32index2++;
//					}
//					if (l_u32index2>5)
//					{
//						tmp1 = 0;
//						for(j=l_u32index; j<=pVehicle->Vdata.zdata[i][0]; j++)
//						{
//						   if (pVehicle->Vdata.zdata[i][j]>=pVehicle->Vdata.zMax[i]/2 && pVehicle->Vdata.zdata[i][j]>=pVehicle->Vdata.zMax[i]*9/10)
//						   {
//								if (((pVehicle->Vdata.zdata[i][j] == pVehicle->Vdata.zdata[i][j+1])   //ǰ��2������� 
//									||(l_u8EqualNum>=1 && (pVehicle->Vdata.zdata[i][j]!=pVehicle->Vdata.zdata[i][j+1])&& (pVehicle->Vdata.zdata[i][j+2]==pVehicle->Vdata.zdata[i][j+1]))
//									||(l_u8EqualNum<1&&(pVehicle->Vdata.zdata[i][j] == 0 && pVehicle->Vdata.zdata[i][j+1] !=0))) && (j <pVehicle->Vdata.zdata[i][0]-1)) //20140320��������ȵĵ���ڣ��ҽ����и߶���0ֵ��
//								{
//									l_u8EqualNum++;
//								}
//								else
//								{
//									if (l_u8EqualNum<4)
//									{
//										if (++tmp2>=2)
//										{
//											l_u8EqualNum = 0;
//										}									
//									}
//								}					   
//						   }
//						   else if (pVehicle->Vdata.zdata[i][j]>=pVehicle->Vdata.zMax[i]*9/10)
//						   {
//						   		if (++tmp1 >=3)
//								{
//									break;								
//								}
//
//						   }
//						   else
//						   {
//						   		continue;
//						   }
//						   l_u32index3++;
//						}
//						if (l_u8EqualNum>=4 && l_u8EqualNum>=(l_u32index3-tmp1)/2)
//						{
//						   l_u8DaFeiFrameCnt++;
//						}
//						else if (l_u8EqualNum>=2)
//						{
//						   l_u8DaFeiFrameCnt2++;
//						}				
//					}								
//								
//				}			
//			}		
//			if (l_u8DaFeiFrameCnt>3 && ((l_u8DaFeiFrameCnt >= (VehEnd_NumIndex-l_u8TotalVehNum/3)/3) 
//				|| (l_u8DaFeiFrameCnt2>0 && l_u8DaFeiFrameCnt+l_u8DaFeiFrameCnt2>=(VehEnd_NumIndex-l_u8TotalVehNum/3)/2)))
//			{
//				dakeche = 1;
//			}						
//		}
//
//		if (0 == dakeche && VehHeight>3400 && l_u32Height2>3400)
//		{
//			tmpHeight = 0;
//			for (i=l_u8TotalVehNum*2/3; i<VehEnd_NumIndex; i++)
//			{
//				tmpHeight = tmpHeight + abs(pVehicle->Vdata.zMax[i] - l_u32Height2);
//			}
//			tmpHeight /= (VehEnd_NumIndex-l_u8TotalVehNum*2/3);
//		}
//
//
//		//���ͻ���
//		if (VehHeight < 1600 && pVehicle->Vdata.u16FrameCnt <= 20)	//�������ش������Ϊ��С�ͳ�
//		{
//			RetPattern = ZHONGXIAOKE;
//		}  
//		else if (VehLength >= BIGANGSMALLTHR && VehLength < 6500) //6000����6500
//		{
//			if(VehWide < 1700 && VehHeight < 2100)
//			{
//				RetPattern=XIAOHUOCHE;//С�ͻ���
//			}
//			else if (IStou!=1 && (!(l_u32Height2 - l_u32Height1 > 180 && l_u32Height1 > 0 && l_u32Height2 > 0 && l_u32Height2 > l_u32Height1)))
//			{
//				if (((allSDcha < l_nDaKeThreshhold11 && MultiCheshenSD < l_nDaKeThreshhold12 && MultiCheshenSD > 20))
//				&& VehHeight>2800 && VehHeight<l_nDaKeHeightThr && VehLength >= BIGANGSMALLTHR && pVehicle->Vdata.u16FrameCnt > DakecheFrameCnt)		
//				{
//					RetPattern = DAKECHE;	//��ͳ�
//				}
//				else if((allSDcha < 100 && MultiCheshenSD < 150)
//				&& VehLength <= 7500 && VehHeight > 2350 && VehHeight <= 2700) ////�жϿ�˹�س�Ϊ��ͳ�
//				{
//					RetPattern = DAKECHE;
//				}
//				else
//				{
//					RetPattern = XIAOHUOCHE;//С�ͻ���
//				}				
//			}
//			else
//			{
//				RetPattern= ZHONGHUO;//���ͻ���  20131223��С�ͻ���Ϊ���ͻ���  XIAOHUOCHE
//			}
//			if(RetPattern == XIAOHUOCHE && (VehHeight > 3400 ||(VehWide > 2500 && VehHeight > 3000)))
//			{
//				RetPattern=ZHONGHUO;//���ͻ���
//			}	
//		}
//		else if (VehLength >= 6500 && VehLength <12000)	   //6500����12000
//		{
//			if (VehHeight < 2200 )
//			{
//				RetPattern = ZHONGHUO;   //�л���
//			}
//			else if (IStou==1 || (l_u32Height2 - l_u32Height1 > 200 && l_u32Height1 > 0 && l_u32Height2 > 0 ) )
//			{
//				if(VehLength <= 7500 || VehHeight <= 2900)
//				{
//					RetPattern = ZHONGHUO;//���ͻ���
//				}		
//				else if(l_nCheTouHeight < 2800 && l_nCheTouHeight > 1000)
//				{
//					RetPattern = ZHONGHUO;//���ͻ���
//				}
//				else if(VehLength > 9000)
//				{
//					RetPattern = DAHUO;//���ͻ���
//				}
//				else
//				{
//					RetPattern = ZHONGHUO;//���ͻ���
//				}
//			}
//			else if (VehHeight>3400 && l_u32Height2>3400 && l_u32Height2>l_u32Height1+50
//				&& tmpHeight>0 && tmpHeight<40) //20140809
//			{
//				RetPattern = DAHUO;	//���ͻ���
//			}
//			else if ((allSDcha < l_nDaKeThreshhold11 && MultiCheshenSD < l_nDaKeThreshhold12 && MultiCheshenSD > 20)//20130704,�����㷽������ޣ���ֹС��������
//			&& VehHeight>2800 && VehHeight<l_nDaKeHeightThr && VehLength >= BIGANGSMALLTHR  && pVehicle->Vdata.u16FrameCnt > DakecheFrameCnt)  	 //		֡������
//			{
//				RetPattern = DAKECHE;	//��ͳ�
//			}
//			else if((allSDcha < l_nDaKeThreshhold11 && MultiCheshenSD < l_nDaKeThreshhold12)
//			&& VehLength < 7500 && VehHeight > 2500 && VehHeight < 2750)   //��2600��Ϊ2500
//			{
//				RetPattern = DAKECHE;	//��ͳ���19�����Ͽͳ���25�����£�
//			}
//			else if((allSDcha < 100 && MultiCheshenSD < 150&&VehLength < 7500)
//			 && VehHeight > 2350 && VehHeight <= 2500) ////�жϿ�˹�س�Ϊ��ͳ�
//			{
//				RetPattern = DAKECHE;
//			}
//			else if (1 == dakeche && VehHeight>=2500)		   //20140808
//			{
//				RetPattern = DAKECHE;
//			}
//			else
//			{
//				if(VehLength <= 7500 || VehHeight <= 2900)
//				{
//					RetPattern = ZHONGHUO;//���ͻ���
//				}		
//				else if(l_nCheTouHeight < 2800 && l_nCheTouHeight > 1000)
//				{
//					RetPattern = ZHONGHUO;//���ͻ���
//				}
//				else if(VehLength > 9000)
//				{
//					RetPattern = DAHUO;//���ͻ���
//				}
//				else
//				{
//					RetPattern = ZHONGHUO;//���ͻ���
//				}
//			}
//		}
//		else if (VehLength >= 12000 && VehLength <15000)	//12000����15000
//		{
//			if (IStou == 1)
//			{
//		         if  (SigleCheshenSD< l_nJZXThreshhold11  && MultiCheshenSD < l_nJZXThreshhold12  && VehHeight>3800 && VehHeight<4400)
//			     {
//                     RetPattern = JIZHUANGXIANG;  //��װ�䳵
//			     }
//                 else	 
//			     {
//                     RetPattern = TEDAHUO;	//�ش��ͻ���
//			     }
//			}
//			else if (l_u32Height2 - l_u32Height1 > 200 && l_u32Height1 > 0 && l_u32Height2 > 0 && l_u32Height2 > l_u32Height1)
//			{
//				RetPattern = DAHUO;	//���ͻ���
//			}
//			else if (VehHeight>3400 && l_u32Height2>3400 && l_u32Height2>l_u32Height1+100
//				&& tmpHeight>0 && tmpHeight<40) //20140808
//			{
//				RetPattern = DAHUO;	//���ͻ���
//			}
//			else if ((allSDcha < l_nDaKeThreshhold11 && MultiCheshenSD < l_nDaKeThreshhold12 && MultiCheshenSD > 0)
//			&& VehHeight>2800 && VehHeight<l_nDaKeHeightThr && pVehicle->Vdata.u16FrameCnt > DakecheFrameCnt)
//			{
//				RetPattern = DAKECHE; //��ͳ�
//			}
//			else
//			{
//				RetPattern = DAHUO;		//û��ͷ������ͷ�복��û�����ԵĽ���
//			}
//		}
//		else if(VehLength >= 15000 && VehLength <=18000 )
//		{
//			if (IStou == 1 ||((l_u32Height2 - l_u32Height1 > 200 && l_u32Height1 > 0 && l_u32Height2 > 0 && l_u32Height2 > l_u32Height1)))	//20140122
//			{
//		        if ( SigleCheshenSD< l_nJZXThreshhold11  && MultiCheshenSD < l_nJZXThreshhold12  && VehHeight>3800 && VehHeight<4400)
//			    {
//                    RetPattern = JIZHUANGXIANG;  //��װ�䳵
//                }
//			    else
//			    {
//                     RetPattern = TEDAHUO; //�ش��ͻ���
//		    	}				
//			}
//			else if (VehLength < 16000 && ((allSDcha < l_nDaKeThreshhold11 && MultiCheshenSD < l_nDaKeThreshhold12 && MultiCheshenSD > 50)
//			         && VehHeight>2800 && VehHeight<l_nDaKeHeightThr && pVehicle->Vdata.u16FrameCnt > DakecheFrameCnt))  //����20140122
//			{
//				RetPattern = DAKECHE;
//			}
//			else
//			{
//	             if  (SigleCheshenSD< l_nJZXThreshhold11  && MultiCheshenSD < l_nJZXThreshhold12  && VehHeight>3800 && VehHeight<4400)
//			     {
//                     RetPattern = JIZHUANGXIANG;  //��װ�䳵
//			     }
//                 else
//			     {
//                     RetPattern = TEDAHUO; //�ش��ͻ���
//			     }
//			}
//		}
//		else
//		{
//            if ( SigleCheshenSD< l_nJZXThreshhold11  && MultiCheshenSD < l_nJZXThreshhold12  && VehHeight>3800 && VehHeight<4400)
//		    {
//		       RetPattern = JIZHUANGXIANG; //��װ�䳵
//		    }
//		    else
//		    {
//		       RetPattern = TEDAHUO; //�ش��ͻ��� 
//		    }
//		}
//		
//		if (!IStou && Widefangcha < 70 && (RetPattern == ZHONGHUO || RetPattern == DAHUO)
//			&&(!((l_u32Height2 - l_u32Height1 > 200) 
//			 && l_u32Height1 > 0 && l_u32Height2 > 0)) && VehHeight > 2500)	//������ͳ�,�������
//		if (!IStou && Widefangcha < 70 && (RetPattern == ZHONGHUO || RetPattern == DAHUO))	//������ͳ�,�������
//		{
//			RetPattern = DAKECHE;
//	
//		}
//
//		 if ((RetPattern == DAHUO || RetPattern == ZHONGHUO)&& l_u8DaFeiFrameCnt>0&&
//			(!IStou) && ((l_u32Height2 - l_u32Height1 >= -150 && l_u32Height2 - l_u32Height1<=0) 
//			 && l_u32Height1 > 0 && l_u32Height2 > 0) && (l_u32Wide1 >l_u32Wide2 && l_u32Wide2 > 0)&&VehHeight > 2350)
//		{
//			RetPattern = DAKECHE;		
//		}
//		//���Ӷ������������������ȵ��ж�
//		TouGao = 0;
//		TouKuan = 0;
//		for(u16Index = 1; u16Index <= 4; u16Index++)
//		{
//			TouGao  += Height[u16Index];
//			TouKuan +=	pVehicle->Vdata.xMax[u16Index];
//		}
//		TouGao  = TouGao/4;
//		TouKuan = TouKuan/4;
//		if (RetPattern == ZHONGHUO)
//		{
//
//			if (TouGao >= 2900 && TouKuan >= 2200)	   //����������ͻ��� �������г�ͷ�߿�ϴ���л��ķ���
//			{
//				RetPattern = DAHUO;
//			}
//
//			if (VehLength > 7500 && VehLength < 10000 && VehHeight > 3700 && VehHeight < 4200)  //���3���������
//			{
//				for(u16Index = Veh_Num*2/5; u16Index < Veh_Num*3/5; u16Index++)
//				{
//					if(Height[u16Index+1]- Height[u16Index] >= 0)	//�����߶���������
//						l_u8HighCount++;	
//				}
//				if(l_u8HighCount >= Veh_Num/5 && Height[Veh_Num*3/5] - Height[Veh_Num*2/5] > 150)
//				{
//					RetPattern = DAHUO;	   //���ͻ���	
//				}
//			}
//		}
//		else if(RetPattern == DAHUO)							          
//		{
//			if(2 == Toupos)												 //��Գ�ͷ�ж�����л�
//			{
//				if(Height[1] - Height[0] >= 100
//					&& Height[2] - Height[1] >= 200 
//					&& TouKuan <= 2010)
//					{
//						RetPattern = ZHONGHUO;
//						return RetPattern;		
//					}		
//			}
//			else
//			{
//				if(Height[3] >= Height[2] 
//					&& Height[2] - Height[1] >= 90													   
//					&& Height[1] - Height[0] >= 200
//				    && TouKuan <= 2010 )
//					{
//						RetPattern = ZHONGHUO;
//						return RetPattern;
//					}
//			}   
//		}			
//	}
//	   
//	return RetPattern;
    return 0;
}


/*******************��������С�ͳ�����Ҫ����С�ͳ�1��С�ͻ���2��Ħ�г�8��������9)********************/
uint8 GetLightVehPattern(VehicleStruct *pVehicle)
{
	uint8   RetPattern    = 0;	//
	uint8   xiaokeche     = 0;
	uint8   konghuoche    = 0;
	uint8   xiaohuoche    = 0;
	uint8 	l_u8index     = 0;
	uint8   l_u8EqualNum  = 0;

	uint8   l_u8FrameNum  = 0;   //���жϽ𱭳�ʱ������β��֡��

	uint8   i = 0;
	uint8	j = 0;
	uint8 k;
	uint8   l_u8Flag = 0;
	int32 tmpValue2 = 0;
	int32 tmpDiff[FRAME_MAXCNT] = {0};  //20130514  ���ڴ�Ÿ߶Ȳ��ֵ
	uint8 l_u8MaxPos = 0;  //������ֵ����λ��
	uint8 l_u8MinPos = 0;  //�����Сֵ����λ��
	int32 l_n32MaxDiff = 0;
	int32 l_n32MinDiff = 0; 
	int32 l_n32TmpSum1 = 0;
	int32 l_n32TmpSum2 = 0;
	int32 l_n32SecHeight = 0;  //��2��߶�
	int32 l_n32TempHeight = 0;
	uint8   Veh_Num       = 0;
	int32   VehLength     = 0;
	int32   VehHeight     = 0;
	int32   VehWide       = 0;
	uint8   u8SidePlanenessFlag = 0;  //����ÿ֡�Ķ���ƽ����ʶ��0��ʾ��ƽ����1��ʾƽ��
	uint8 l_u8PiKaFlag = 0;
	uint8  l_u32index,l_u32index2;
	int32 Height[FRAME_MAXCNT] = {0};   //���ڼ���ʱʹ�õĳ���
	uint8  l_u8Left_Index = 0;
	uint8  l_u8Right_Index = 0;
	int32  l_n32Left_MaxZ = 0;
	int32  l_n32Right_MaxZ = 0;
	int32  l_n32MinChassisHeight = 600;
	int32  l_n32Min_X            = 0;
	int32  l_n32Max_X            = 0;
	memset(Height,0,sizeof(Height));

	//20140217 ���ӶԲ����ж�
	if (pVehicle == NULL)
	{
		return 0;
	}
   Veh_Num       = pVehicle->Vdata.u16FrameCnt;
	if (Veh_Num > FRAME_MAXCNT || Veh_Num < 1)
	{
		Veh_Num = 0;  //֡���������֡��ֵ��֡����ֵΪ0
		pVehicle->Vdata.u16FrameCnt = 1;
	}
	VehLength     = pVehicle->yLen;
	VehHeight     = pVehicle->zLen;
	VehWide       = pVehicle->xLen;

 	memcpy(Height, pVehicle->Vdata.zMax, Veh_Num*sizeof(int32));
	if (Veh_Num <= 2)  //  2֡����������С�ͳ�1
	{
		RetPattern = ZHONGXIAOKE;
		return RetPattern;
	}
	else
	{
		//�޳��߶���1֡�쳣��	20140401����
		for (i = 0; i < Veh_Num; i++)
		{ //���ҳ���2��߶�ֵ
			if ((l_n32SecHeight < pVehicle->Vdata.zMax[i]) && 
				(VehHeight > pVehicle->Vdata.zMax[i]))
			{
				l_n32SecHeight = pVehicle->Vdata.zMax[i];
			}		  
		}
		if (VehHeight-l_n32SecHeight > 300) //�߶ȳ���300�����¼�����߶�ֵ��֡������
		{
			for (i = 0; i < Veh_Num; i++)
			{ 
				if (VehHeight == pVehicle->Vdata.zMax[i])
				{
					for (l_u8index=1;l_u8index<pVehicle->Vdata.zdata[i][0];l_u8index++)
					{
						if (l_n32TempHeight < pVehicle->Vdata.zdata[i][l_u8index] && 
							VehHeight > pVehicle->Vdata.zdata[i][l_u8index])
						{
							l_n32TempHeight = pVehicle->Vdata.zdata[i][l_u8index];
						}	
					}
					//�߶����Ƚϴ�
					if (VehHeight-l_n32TempHeight >=350)
					{
						for (l_u8index =1 ; l_u8index<pVehicle->Vdata.zdata[i][0];l_u8index++)
						{
							if ((l_n32TempHeight-pVehicle->Vdata.zdata[i][l_u8index] < 350 )
								&& (l_n32TempHeight>=pVehicle->Vdata.zdata[i][l_u8index]))
							{
								l_n32TmpSum1 += pVehicle->Vdata.zdata[i][l_u8index];
								l_n32TmpSum2++;	
							}	
						}
						
						if (l_n32TmpSum2 > 0)
							pVehicle->Vdata.zMax[i] = l_n32TmpSum1/l_n32TmpSum2;
						else
							pVehicle->Vdata.zMax[i] = l_n32TempHeight;
						
						pVehicle->zLen = (l_n32SecHeight>=pVehicle->Vdata.zMax[i])? l_n32SecHeight:pVehicle->Vdata.zMax[i];	
						l_n32TmpSum1 = 0;
						l_n32TmpSum2 = 0;
						l_n32TempHeight = 0;		
					}
				}		  
			}
		}
		VehHeight = pVehicle->zLen;
		///////////С�γ��ж�////////////////////////
		if (VehHeight < 2280)
		{
			if (pVehicle->Vdata.zMax[0] < 800 && pVehicle->Vdata.zMax[1] < 1000 && pVehicle->Vdata.zMax[Veh_Num-1] < 800)
			{
				xiaokeche = 1;  //��С�ͳ���־	
			}
		}

		for (i = 0; i < Veh_Num/3; i++)//����ǰ1/3֡����ƽ����
		{
			tmpValue2 += pVehicle->Vdata.zMax[i];
		}
		if (tmpValue2/(Veh_Num/3) < 1000)	 //ǰ1/3֡����ƽ����С��1m
		{
			xiaokeche = 2;   //��С�ͳ�
		}
		////////////////С�����ж�///////////////////////////
		tmpValue2 = 0;
		if (VehHeight > 2000)  	//���㳵����֡�ߵĲ�ֵ�����ֵ�ľ�ֵ
		{
			for (i = Veh_Num/2; i < Veh_Num - 2; i++)
			{
				tmpValue2 += abs(VehHeight - pVehicle->Vdata.zMax[i]);
				l_u8index++;
			}
			if (l_u8index > 0)
			{
				tmpValue2 = tmpValue2/l_u8index;
			}
		}
		else if (VehHeight > 1800)  //����������֡�Ĳ�ֵ�����ֵ�ľ�ֵ
		{
			for (i = Veh_Num/2; i < Veh_Num - 3; i++)
			{
				tmpValue2 += abs(pVehicle->Vdata.zMax[i] - pVehicle->Vdata.zMax[i+1]);
				l_u8index++;
			}
			if (l_u8index > 0)
			{
				tmpValue2 = tmpValue2/l_u8index;
			}			
		}
		if (tmpValue2 > 100 && l_u8index >= 4)
		{
			xiaohuoche = 1;   //С����
		}
		/////////�ջ����ж�/////////////////////
		if (Veh_Num >= 4 && (Veh_Num/2+1 < Veh_Num-1) &&	 //Veh_Num/2+1���������һ֡����
		   (VehHeight - pVehicle->Vdata.zMax[Veh_Num/2] > 300 || VehHeight - pVehicle->Vdata.zMax[Veh_Num/2+1] > 300))
		{   //20140325 ���ӷ�ֹ�쳣�߶ȵ����жϿջ�
			for (i = 0; i < Veh_Num/2+1; i++)
			{
				if ((VehHeight - pVehicle->Vdata.zMax[i] < 300) && 
					(VehHeight > pVehicle->Vdata.zMax[i] || l_u8EqualNum > 1)) 
				{//��ǰ�벿��֡������֡�����복�������300��
					konghuoche = 1;  //�ջ���
					break;					
				}
				else if (VehHeight == pVehicle->Vdata.zMax[i])
				{
					l_u8EqualNum++;
				}
			}
		}
		if (Veh_Num >= 4)
		{
			for (i = 0; i < Veh_Num/2; i++)
			{
				l_n32TmpSum1 += pVehicle->Vdata.zMax[i];
			}
			l_n32TmpSum1 = l_n32TmpSum1/(Veh_Num/2);  //ǰһ��֡���߶ȵľ�ֵ
			
			for (i = Veh_Num/2; i < Veh_Num-1; i++)	  //�������һ֡����
			{
				l_n32TmpSum2 += pVehicle->Vdata.zMax[i];
			}
			l_n32TmpSum2 = l_n32TmpSum2/(Veh_Num-Veh_Num/2-1); //��һ��֡���߶ȵľ�ֵ
			if (l_n32TmpSum1 > (l_n32TmpSum2 + 150) && VehHeight > 1700)	   //40��Ϊ150
			{
				konghuoche = 1;
			}
		}
	}

    //�����ж�С�ͳ�




    if(VehWide < 1000 && VehLength < 2000 && VehHeight < 1800)	  //3֡������
    {
	    RetPattern = MOTUOCHE;	 //Ħ��
  	}
	else if(VehHeight > 2800)	//20140819
	{
		RetPattern = XIAOHUOCHE;	
	}
	else if(VehHeight > 2500)
	{
		if (konghuoche == 1 || xiaohuoche == 1)
		{
			RetPattern = XIAOHUOCHE;
		}
		else if ((konghuoche != 1 && xiaokeche==1) || xiaokeche == 2)
		{
			RetPattern = ZHONGXIAOKE;
		}
		else if(VehWide>1800 && abs(l_n32TmpSum1 - l_n32TmpSum2)<50 && tmpValue2<50)
		{

			RetPattern = DAKECHE;
		}
		else
		{
			RetPattern = XIAOHUOCHE;
		}	
	}
	else if (!RetPattern)
	{
		if (VehHeight < 1600 || xiaokeche == 2)
		{
			RetPattern = ZHONGXIAOKE;
		}
		else if (konghuoche == 1 || xiaohuoche == 1)
		{
			RetPattern = XIAOHUOCHE;
		}
		else if (konghuoche != 1 && xiaokeche == 1)
		{
			RetPattern = ZHONGXIAOKE;
		}
		else
		{
			//SUV��ʶ��
			if (VehHeight < 2000 && VehHeight >= 1600)
			{
				l_u8index = 0;
				l_n32TmpSum1 = 0;
				for (i = 0; i < Veh_Num; i++)	  //Ѱ�ҿ�ʼ֡���߶ȵ���1300��֡��
				{
					if (pVehicle->Vdata.zMax[i] <= 1300)
					{
						l_u8index++;
					}
					else
					{
						break;
					}
				}
				for (i = Veh_Num/2; i < Veh_Num-1; i++)	  //�����һ��֡���߶ȵľ�ֵ
				{
					l_n32TmpSum1 += pVehicle->Vdata.zMax[i];
				}
				l_n32TmpSum1 = l_n32TmpSum1/(Veh_Num-1-Veh_Num/2);

				if (l_n32TmpSum1 > 1650 && l_n32TmpSum1 < 1900 && ((l_u8index>=2 && l_u8index>=Veh_Num/3) || (l_u8index>=5 && pVehicle->speed > 30)))
				{
					RetPattern = ZHONGXIAOKE;
				}
			}

			if (!RetPattern)
			{
				//��ʽС�����ж�
				tmpValue2 = 0;
				for(i = 2;i < (Veh_Num>>1);i++)
				{
					if(pVehicle->Vdata.zMax[i] + 300 < pVehicle->Vdata.zMax[i-1])	 //���ҳ�ͷ�����������
					{
						tmpValue2 = i+1; 
						while(tmpValue2 < Veh_Num-1)
						{
							if(pVehicle->Vdata.zMax[tmpValue2+1] - pVehicle->Vdata.zMax[tmpValue2] > 300)	 //���ҳ�ͷ�����������
							{
								tmpValue2 = tmpValue2+1; 								
								break;
							}
							tmpValue2++;	
						}								
						break;
					}
				
				}  	
				if(tmpValue2 && tmpValue2 < (Veh_Num>>1))
				{
					while(tmpValue2 < Veh_Num-1)
					{
						if(abs(pVehicle->Vdata.zMax[tmpValue2] - pVehicle->Vdata.zMax[tmpValue2 + 1]) < 150)
						{
							 //��ʽС�� 	
							 RetPattern = XIAOHUOCHE;	
							 break;
						}
						else
						{
							 RetPattern = 0;
							 break;
						}
						tmpValue2++;
					} 			
				}	 
			   else
			   {
			   	//�ж��Ƿ�Ϊ��ʽС�ͳ�
				 tmpValue2 = Veh_Num-1;  			 
				 while(tmpValue2 > 0 )
				 {
				 	if(abs(pVehicle->Vdata.zMax[tmpValue2] - pVehicle->Vdata.zMax[tmpValue2 - 1]) < 150)
					{
						tmpValue2--;	
					}
					else
						break;
				 }
				 if(tmpValue2 <= (Veh_Num>>1))
				 {
				 	if(pVehicle->Vdata.zMax[tmpValue2] - pVehicle->Vdata.zMax[tmpValue2 - 1] < 600)
					{
					  	i = 1;
					 	while(i < tmpValue2)
						{
							if(pVehicle->Vdata.zMax[i] < pVehicle->Vdata.zMax[tmpValue2])
							{
								 RetPattern = ZHONGXIAOKE; //��ʽС�ͳ�	
								 break;
							}
							else
							{
								RetPattern = 0;
								break; 
							}
							i++;
						}
					}
					else
					{
						RetPattern = ZHONGXIAOKE;  //���� XIAOHUOCHE   ��Ϊ��С�ͳ�
					}	 
				 }
			   }
		
				//����ʽ�͡�����
				if(!RetPattern)
				{
					tmpValue2 = 0;
					for(i = 2;i < Veh_Num-1;i++)  //���һ֡��Ҫ
					{
						if(pVehicle->Vdata.zMax[i] + 250 < pVehicle->Vdata.zMax[i-1])	 //���ҳ�ͷ�����������
						{
						   tmpValue2 = i;
						   break;
						}
					}
					if(!tmpValue2)
					{ 	
						RetPattern = ZHONGXIAOKE; //С�ͳ�				
					}
					else
					{
			           	for (i = 2; i < Veh_Num; i++)    //20130514 �����2֡��������2֡�ĳ��߲��
						{
							tmpDiff[i] = pVehicle->Vdata.zMax[i] - pVehicle->Vdata.zMax[i-1];
						}
						tmpDiff[0] = Veh_Num;   //֡��
						//Ѱ�ҳ��߲���е����ֵ����Сֵ������λ��	20130514 
						l_n32MaxDiff = tmpDiff[2];
						l_n32MinDiff = tmpDiff[Veh_Num-1];
						l_u8MaxPos   = 2;
						l_u8MinPos   = Veh_Num-1;
						for( i = 3; i < Veh_Num; i ++)
						{				
							if (l_n32MaxDiff < tmpDiff[i])	//Ѱ�����ֵ������
							{
								l_n32MaxDiff = tmpDiff[i];
								l_u8MaxPos  = i;
							}
							if (l_n32MinDiff > tmpDiff[Veh_Num-i+1] && i<Veh_Num-1)
							{
								l_n32MinDiff = tmpDiff[Veh_Num-i+1];
								l_u8MinPos   = Veh_Num-i+1;
							}
						}
						//��ֱ仯�ܴ�˵����С����
						if ( l_n32MinDiff < -800)
						{
							RetPattern = XIAOHUOCHE;   
						}
						else
						{
							//�����Сֵ�������С����֡��2/3����Ƚ����һ֡��ǰһ֡�Ĳ�֣� ��ֹ����	  20130514
							if ( l_u8MinPos <= Veh_Num*2/3 && l_n32MinDiff > pVehicle->Vdata.zMax[Veh_Num-1]-pVehicle->Vdata.zMax[Veh_Num-2])
							{
								l_u8MinPos = Veh_Num-1;
								l_n32MinDiff = pVehicle->Vdata.zMax[Veh_Num-1]-pVehicle->Vdata.zMax[Veh_Num-2];
							}
							
							//�����ǰ������С�ڳ�β��������Ѱ���������������һ����ֱ仯�ϴ�ĵ�   20130514
							if ( l_u8MaxPos -1 < (Veh_Num-l_u8MinPos+1) && tmpDiff[l_u8MaxPos+1] > 150)
							{
								l_u8MaxPos = l_u8MaxPos + 1;
							}
							//���׵������ڻ���ڳ�β����������β�����ʹ��ڻ���ڳ������ 20130514
							if (l_u8MaxPos>=(Veh_Num-l_u8MinPos) && (l_u8MaxPos + Veh_Num-l_u8MinPos >= l_u8MinPos-l_u8MaxPos) 
							   && l_u8MaxPos-1 < l_u8MinPos && (Veh_Num - (l_u8MaxPos-1)*2 > 0  ))
							{
								RetPattern = ZHONGXIAOKE;   //��С�ͳ�
							} 
							else
							{
								RetPattern = XIAOHUOCHE;  //С����
							}
						}
					}
				}
			}
		}
		////��Ҫ��Ƥ���ļ��

		if(RetPattern == ZHONGXIAOKE && VehHeight > 1540 && VehHeight < 1790 && Veh_Num >= 2)	 //��Χ�Ŵ�һ��1600-1700
		{
			//СƤ���ļ�⣬��Ϊβ�ͱȽϳ�����С����Ƥ����
			for(l_u32index = 0; l_u32index < Veh_Num - 1; l_u32index++)
			{
			    if(Height[l_u32index] > Height[l_u32index+1] && Height[l_u32index] - Height[l_u32index+1] > 400)//   Height[l_u32index] > Height[l_u32index+1] && 
				{
					break;
				}		
			}
			k = 0;
			for( i = l_u32index+1; i < Veh_Num; i++)
			{
				if(Height[i] < 1250 && Height[i] > 940)
				{
					k++;
				} 
				else
				{
					break;
				}			
			}
			if(k == Veh_Num - l_u32index-1 &&  k >= 2)
			{
				 l_u8PiKaFlag = 1;	   //��⵽������β���ϳ�
			}
			
			if( Veh_Num >= 5 && l_u8PiKaFlag)
			{
				for(i = Veh_Num - 2; i > Veh_Num - 5; i--)
		    	{
			    	if(Height[i] < 1250 && Height[i] > 940)//Ƥ���ĳ���λ��
					{
						if (pVehicle->Vdata.zdata[i][0] >= FRAME_BUFLEN) //������ֵ
							continue;
						//Ѱ��������ֵ
						l_n32Left_MaxZ = pVehicle->Vdata.zdata[i][1];
						l_u8Left_Index = 1;
						for (k = 1; k <= pVehicle->Vdata.zdata[i][0]; k++)
						{
							if (l_n32Left_MaxZ < pVehicle->Vdata.zdata[i][k])
							{
								l_n32Left_MaxZ = pVehicle->Vdata.zdata[i][k];
								l_u8Left_Index = k;
							}
							else if (l_n32Left_MaxZ > pVehicle->Vdata.zdata[i][k] && 
								l_n32Left_MaxZ > 900)
								break;
						}
						//Ѱ���ұ����ֵ
						l_u8Right_Index = pVehicle->Vdata.zdata[i][0];
						l_n32Right_MaxZ = pVehicle->Vdata.zdata[i][l_u8Right_Index];
						for (k = pVehicle->Vdata.zdata[i][0]; k > 0; k--)
						{
							if (l_n32Right_MaxZ < pVehicle->Vdata.zdata[i][k])
							{
								l_n32Right_MaxZ = pVehicle->Vdata.zdata[i][k];
								l_u8Right_Index = k;
							}
							else if (l_n32Right_MaxZ > pVehicle->Vdata.zdata[i][k] && 
								l_n32Right_MaxZ > 900)
								break;
						}
						//�Ҹߵ���
						l_n32TmpSum1 = 0;
						l_n32TmpSum2 = 0;
						l_n32TempHeight = l_n32Left_MaxZ >= l_n32Right_MaxZ ? l_n32Left_MaxZ : l_n32Right_MaxZ;
						for (k = l_u8Left_Index+1; k < l_u8Right_Index; k++)	
						{
							l_n32TmpSum1 += (pVehicle->Vdata.zdata[i][k]-l_n32TempHeight);
							l_n32TmpSum2 += abs(l_n32TempHeight-pVehicle->Vdata.zdata[i][k]);	
						}
						if (l_u8Right_Index > l_u8Left_Index+1)
						{
							l_n32TmpSum1 = l_n32TmpSum1/(l_u8Right_Index - l_u8Left_Index-1);
							l_n32TmpSum2 = l_n32TmpSum2/(l_u8Right_Index - l_u8Left_Index-1);
						}
						if(l_n32TmpSum1 < 0 && l_n32TmpSum2 > 200 && (l_u8Right_Index - l_u8Left_Index-1 > 6))
						{
					   		RetPattern = XIAOHUOCHE;//С����Ƥ��
					   		break;
						}
					}		    
		    	}
			}
		} 			 			
	}

	if (g_sspSetup.u8RoadType)//����
	{
		if (!RetPattern || RetPattern == MOTUOCHE)
		{
			RetPattern = ZHONGXIAOKE;
		}
	}
	else
	{
		if (!RetPattern)
		{
			RetPattern = ZHONGXIAOKE;
		}
	}


	return RetPattern;
}


uint16 GetLimitValue(int* pg_ZdistanceI,int* pXdata, int len,int startPos, uint8 u8Flag)    //��װ��ʱ��ú��������޸�	u8FlagΪ0��ʾ��ֱ���������ݣ�1��ʾ��б����������
{ 
//	uint16 ret = startPos;
//	uint16  l_u16CountOut = 0;
//	int l_midPt = len>>1;
//	int l_PtNum1 = 0;    //abs(g_sspSetup.u16StartPtNum - g_sspSetup.u16VerticalZeroPos);  // �������ʼ��ĵ�����
//	int l_PtNum2 = 0;    //abs(g_sspSetup.u16EndPtNum - g_sspSetup.u16VerticalZeroPos);    //�����������ĵ�����
//
//	//���ӶԲ����ĺϷ����ж� 20140214
//	if (pg_ZdistanceI == NULL || pXdata == NULL || len >= POINT_SUM )
//	{
//		return ERRORVALUE;		 //���ش���ֵ
//	}
//	
//	if (u8Flag)	 //��б������
//	{
//		l_PtNum1 = abs(g_sspSetup.u16StartPtNum - g_sspSetup.u16InclineZeroPos);  // �������ʼ��ĵ�����
//		l_PtNum2 = abs(g_sspSetup.u16EndPtNum - g_sspSetup.u16InclineZeroPos);    //�����������ĵ�����
//		
//		//20140217 ����	  ��װ��б
//		if (g_sspSetup.u8InstallFlag)
//		{
//			l_midPt = (g_sspSetup.u16InclineZeroPos > g_u16InclineStartAnglePt) ?  
//					  (g_sspSetup.u16InclineZeroPos - g_u16InclineStartAnglePt) : (len>>1);
//		}		
//	}
//	else  //��ֱ������
//	{
//		l_PtNum1 = abs(g_sspSetup.u16StartPtNum0 - g_sspSetup.u16VerticalZeroPos0);  // �������ʼ��ĵ�����
//		l_PtNum2 = abs(g_sspSetup.u16EndPtNum0 - g_sspSetup.u16VerticalZeroPos0);    //�����������ĵ�����
//
//		//20140217 ����	  ��װ��ֱ
//		if (g_sspSetup.u8InstallFlag)
//		{
//			l_midPt = (g_sspSetup.u16VerticalZeroPos > g_u16VerticalStartAnglePt) ?
//				      (g_sspSetup.u16VerticalZeroPos - g_u16VerticalStartAnglePt) : (len>>1);
//		}		
//	}
//
//
//
//	while(startPos >= 0 && startPos <= len && (l_u16CountOut++ < len))
//	{
//		if (startPos < l_midPt && abs(pXdata[startPos]) > g_MedianLeftWide &&		    //С��Ϊ������
//		    abs(pXdata[startPos]) <= g_MaxLeftWide)
//		{
//			ret = startPos;
//			break;
//		}
//		else if (startPos > l_midPt && abs(pXdata[startPos]) > g_MedianRightWide &&		//���Ϊ�Ҹ����
//		    abs(pXdata[startPos]) <= g_MaxRightWide)
//		{
//			ret = startPos;
//			break;
//		}
//		else if (startPos > l_midPt)
//		{
//			startPos--;
//		} 		
//		else 
//		{
//			startPos++;
//		}
//	}		

//   return ret;
     return 0;
}

int32 GetMaxValue(const int32 *pData,uint32 start,uint32 end)
{
	uint32 index;
	int32 retMax = 2000; //Ĭ��ֵ 2��

	//20140217 ���ӶԲ����Ϸ����ж�
	if (pData == NULL || start > g_sspSetup.u16EndPtNum0-g_sspSetup.u16StartPtNum0 ||
		end > g_sspSetup.u16EndPtNum0-g_sspSetup.u16StartPtNum0)	 //�����쳣�����̶�ֵ����Ӱ���������
	{ 
		return retMax;
	}

	retMax = pData[start];

	for(index = start+1;index<end;index++)
	{
		if(pData[index] > retMax)
			retMax = pData[index];		
	}
	return retMax;
}

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

//vehPosFlag ��ʶ��0��ʾ�����ڵ�ǰ�����м����ֵ��1���м����ֵ����ǰ����������2����ǰ������ұ���3
//pPtStruct ��ʾ�����ĵ�ṹ�������pPtData��ʾ��ǰ����ĵ�ṹ�����
uint16 GetPosFromXDistance(int32 *xDistant,PointStruct *pPtStruct, PointStruct *pPtData, uint8 vehPosFlag)	 
{
	uint16 ret = 0,Index = pPtData->u16Leftpt;
	uint16 l_16tmpValue = pPtData->u16Rightpt;
	uint16 l_minValue = 0xFFFF;
	uint16 l_u16LaneWide = g_sspSetup.LaneWide; 
	uint8  l_u8Flag = 0;   //�����ȱ�־ 0��ʾ��ǰ�����ȴ���3.5mС��7m��1��ʾ����7m

	//20140217 ���ӶԲ����Ϸ����ж�
	if (xDistant == NULL || pPtStruct == NULL || pPtData == NULL)
	{
		return ERRORVALUE;
	}

	if (g_sspSetup.u8LaneNum == 6) //��װ6����ʱ�������ȱ�־���п��ܴ���7m
	{
		if (abs(pPtData->n32xRight-pPtData->n32xLeft) > g_sspSetup.LaneWide*2)
		{
			l_u8Flag = 1;
		}
	}


	ret = pPtData->u16Rightpt;
	Index = pPtData->u16Leftpt;
	if (l_16tmpValue >= POINT_SUM-1 || ret >= POINT_SUM-1 || Index >= POINT_SUM-1)
		return ERRORVALUE;

	if (!vehPosFlag)  //���м�	������ֵ
	{
		for (; Index < l_16tmpValue-1; Index++)
		{
			if (abs(xDistant[Index] - xDistant[Index+1]) > 1000 &&
			    abs(xDistant[Index+1]-pPtStruct->n32xRight) < l_u16LaneWide&&
				abs(pPtStruct->n32xLeft-xDistant[Index]) < l_u16LaneWide)
			{
				ret = Index;
				break;
			}
		}		
		if (ret && ret < l_16tmpValue && (abs(pPtStruct->n32xRight-xDistant[Index])> (SMALL_AREA>>1)))
		{
			return ret;
		}
		ret = pPtData->u16Rightpt;
		Index = pPtData->u16Leftpt;

		while(Index < l_16tmpValue)
		{
			if(abs(xDistant[Index] - pPtStruct->n32xLeft) < l_minValue && (abs(pPtStruct->n32xRight-xDistant[Index])> (SMALL_AREA>>1)))
			{
				l_minValue = abs(xDistant[Index] - pPtStruct->n32xLeft);
				pPtStruct->n32xLeft = xDistant[Index];
				ret = Index;		
			}			
			Index++;	
		}
			
	}
	else if (vehPosFlag == 1) //���м䣬������ֵ
	{
		for (; Index < l_16tmpValue-1; Index++)
		{
			if (abs(xDistant[Index] - xDistant[Index+1]) > 1000 &&
			    abs(xDistant[Index]-pPtStruct->n32xLeft) < l_u16LaneWide &&
				abs(pPtData->n32xRight-xDistant[Index+1]) < l_u16LaneWide)
			{
				ret = Index+1;
				break;
			}
		}		
		if (ret && ret < l_16tmpValue && (abs(xDistant[Index]-pPtStruct->n32xLeft)>(SMALL_AREA>>1)))
		{
			return ret;
		}
		ret = pPtData->u16Rightpt;
		Index = pPtData->u16Leftpt;
		while(Index < l_16tmpValue)
		{
			if(abs(xDistant[Index] - pPtStruct->n32xRight) < l_minValue && (abs(xDistant[Index]-pPtStruct->n32xLeft)>(SMALL_AREA>>1)))
			{
				l_minValue = abs(xDistant[Index] - pPtStruct->n32xRight);
				pPtStruct->n32xRight = xDistant[Index];
				ret = Index;		
			}			
			Index++;	
		}
	}
	else if (vehPosFlag == 2)  //����ߣ�����ֵ
	{
		for (; Index < l_16tmpValue-1; Index++)
		{
			if ( (!l_u8Flag) && abs(xDistant[Index] - xDistant[Index+1]) > 1000&&
			    abs(xDistant[Index]-pPtData->n32xLeft) < l_u16LaneWide &&	  //   pPtStruct��ΪpPtData
				abs(pPtData->n32xRight-xDistant[Index+1]) < l_u16LaneWide )
			{
				ret = Index;
				break;
			}
			else if (l_u8Flag && abs(xDistant[Index] - xDistant[Index+1]) > 1000 &&
			    abs(xDistant[Index]-pPtData->n32xLeft) < l_u16LaneWide &&				  //  pPtStruct��ΪpPtData
				abs(pPtData->n32xRight-xDistant[Index+1]) > l_u16LaneWide)	  //ʣ�µ�����Ҫ����3.5m
			{
				ret = Index;
				break;				
			}
		}		
		if (ret && ret < l_16tmpValue && ((abs(pPtData->n32xLeft-xDistant[Index])> (SMALL_AREA>>1)) ||
		    abs(pPtData->u16Leftpt-Index) > MIN_PTNUM) )
		{
			return ret;
		}
		ret = pPtData->u16Rightpt;
		Index = pPtData->u16Leftpt;

		if (l_u8Flag)	 //�������7M
		{
			while(Index < l_16tmpValue)
			{
				if(abs(xDistant[Index] - pPtStruct->n32xRight) < l_minValue && 
				  (abs(pPtData->n32xLeft-xDistant[Index])> (SMALL_AREA>>1)) &&
				  abs(pPtData->n32xLeft-xDistant[Index]) < l_u16LaneWide)		   //  ����abs(pPtData->n32xLeft-xDistant[Index]) < l_u16LaneWide)
				{
					l_minValue = abs(xDistant[Index] - pPtStruct->n32xRight);
	//				pPtStruct->n32xRight = xDistant[Index];
					ret = Index;		
				}			
				Index++;	
			}
		}
		else
		{
			while(Index < l_16tmpValue)
			{
				if(abs(xDistant[Index] - pPtStruct->n32xRight) < l_minValue && (abs(pPtData->n32xLeft-xDistant[Index])> (SMALL_AREA>>1))
				   && abs(pPtStruct->n32xLeft-xDistant[Index]) < l_u16LaneWide && abs(xDistant[Index+1]-pPtData->n32xRight) < l_u16LaneWide  )
				{
					l_minValue = abs(xDistant[Index] - pPtStruct->n32xRight);
	//				pPtStruct->n32xRight = xDistant[Index];
					ret = Index;		
				}			
				Index++;	
			}
		}
	}
	else   //���ұ�,����ֵ
	{
		for (; Index < l_16tmpValue-1; Index++)
		{
			if ( (!l_u8Flag) && abs(xDistant[Index] - xDistant[Index+1]) > 1000&& 
			   abs(xDistant[Index+1]-pPtData->n32xRight) < l_u16LaneWide &&		   //  pPtStruct��ΪpPtData
			   abs(pPtData->n32xLeft-xDistant[Index]) < l_u16LaneWide )
			{
				ret = Index+1;
				break;
			}
			else if (l_u8Flag&&abs(xDistant[Index] - xDistant[Index+1]) > 1000 && 
			   abs(xDistant[Index+1]-pPtData->n32xRight) < l_u16LaneWide &&			//	pPtStruct��ΪpPtData
			   abs(pPtData->n32xLeft-xDistant[Index]) > l_u16LaneWide)	 //
			{
				ret = Index+1;
				break;
			}
		}		
		if (ret && ret < l_16tmpValue && (abs(pPtData->n32xRight-xDistant[Index])> (SMALL_AREA>>1) ||
		    abs(pPtData->u16Rightpt-Index)> MIN_PTNUM))
		{
			return ret;
		}
		ret = pPtData->u16Rightpt;
		Index = pPtData->u16Leftpt;

		if (l_u8Flag)
		{
			while(Index < l_16tmpValue)
			{
				if(abs(xDistant[Index] - pPtStruct->n32xLeft) < l_minValue && 
				  (abs(pPtData->n32xRight-xDistant[Index])> (SMALL_AREA>>1)) && 
				  abs(pPtData->n32xRight-xDistant[Index]) < l_u16LaneWide)			 //����abs(pPtData->n32xRight-xDistant[Index]) < l_u16LaneWide)
				{
					l_minValue = abs(xDistant[Index] - pPtStruct->n32xLeft);
	//				pPtStruct->n32xLeft = xDistant[Index];
					ret = Index;		
				}			
				Index++;	
			}
		}
		else
		{
			while(Index < l_16tmpValue-1)
			{
				if(abs(xDistant[Index] - pPtStruct->n32xLeft) < l_minValue && (abs(pPtData->n32xRight-xDistant[Index])> (SMALL_AREA>>1))
				   && abs(pPtData->n32xLeft-xDistant[Index]) < l_u16LaneWide && abs(xDistant[Index+1]-pPtData->n32xRight) < l_u16LaneWide)
				{
					l_minValue = abs(xDistant[Index] - pPtStruct->n32xLeft);
	//				pPtStruct->n32xLeft = xDistant[Index];
					ret = Index+1;		
				}			
				Index++;	
			}
		}	
	}
	
	return 	ret;
}

/*************************************************************************************/

/*********************************************************************************************************
** ��������:  Get_Vehicle_Info
** ��������:  �õ�������Ϣ
** ��ڲ���:  ��
** ���ڲ���:  ��
** ����˵��:  ��ɨ���һ֡���ݴ�������������ת�����ֳ�����ȡ������Ϣ��ʶ�����洢�����ֳ��źţ��ж����͵�
*********************************************************************************************************/
void Get_Vehicle_Info(void)					
{	
//	uint8  l_DafeiPtNum = 0;   //��ɵ����������ϵ��ɵ�4��
//	uint8  l_u8Ret      = 0;
//	uint8  l_u8Count    = 0;
//	uint8  RetIsVehicle = 0;  //�����С�ͳ������жϵķ���ֵ		
//	int		*data;
//	int 	*data0;
//	int		JG_time;
//	uint32	Time_Vertical;
//	PointSet l_FrameInfo1;
//	int32 l_leftX,l_rightX;    //����x����
//	uint16 l_leftXpt, l_rightXpt;   //����X�����Ӧ�ĵ���
//	uint16 l_leftPt ;	//�����ʼ���������� ������������������֮��ĳ�����
//	uint16 l_rightPt;  //�ұ���ʼ���������� ��Զ�뼤����������֮��ĳ�����
//
//	int i=0;
//	int j=0;
//	int k=0;		            //�ֳ���
//	int m=0;
//	int index=0;
//
//	int32 l_32tmpValue,l_32tmp,l_32tmp2,l_tmp1,TempVaule1;
//	PointStruct l_u16PosVect[POINTSET_CNT] = {0};	//��Ų�����λ����Ϣ
//	uint16 l_u16index,l_u16tmp;
//	uint16	l_index;
//	uint16	l_u16StartPt,l_u16EndPt;
//	int32	l_n32StartY,l_n32EndY;
//	uint16	l_u16StartYpt,l_u16EndYpt;
//	IncPtSt l_u16IncPosVect[POINTSET_CNT] = {0};	//��Ų�����λ����Ϣ
//
//	//�Դ�ɶεĴ�������ɶ�����ֱ��
//    int Dafeiflag=0;
//    int DafeiData[180][4]={0};
//	int MaxZ=0;
//	int MaxX=0;
//	int MinZ=0;
//	int MinX=0;
//	int Maxi=0;
//	 
///*EnterX1 ��ͷ�����̨ǰ��EnterX2��ͷ���ٶȣ�ExitX3�����������׳���ExitX4�޳�����*/
//    int EnterX1=-10000;			  //��̨����ɨ������
//    int EnterX2=-3000;			  //������ٶ�Ϊ����ͷ������ʱ���ٶ�
//	int ExitX3=-2000;                 //��ͷ�����ߺ��׳�
//	int ExitX4=-600;              //��ͷ�����ߺ��޳�����
//
//    int Lengthline1=-7500;
//	int Lengthline2=ExitX3;
//
//    int Tmp_Z = 0;                 //�����߶�
//	int	Tmp_Y = 0;
//
//	PtIncSet l_FrameInfo;
//	memset(&l_FrameInfo, 0, sizeof(l_FrameInfo));
//	memset(&l_FrameInfo1,0, sizeof(PointSet));
///*************************************/	 
//	data = LMS_data_2;     //%���ټ�������
//
//	data0 = LMS_data_1;		//%��߼�������	
//
//	JG_time = data[361];	 //%���ټ���
//	Time_Vertical = (uint32)LMS_data_1[361];
//
//    //�жϼ�������0��ֵ��180��ĵ�λ�ù�ϵ��Ȼ������ʼ����ֹ��λ�� 20130426  	  20130614����˳��λ��
//	g_u16InclineStartAnglePt = GetStartEndPt(data, 30, 0, 1);/*Ѱ�Ҽ���������ʼ��*/
//	g_u16InclineEndAnglePt	  = GetStartEndPt(data, 300, 1, 1);
//
///*****************/	
//JG_T0TC[0] = T0TC;
//JG_counter2[0]=t0_count2;
///*****************/
//
//
///***************************������ɨ����������ת��*******************************/
//    k=0;
//    //20130426  �޸ļ�������ʼ�Ƕȵ㡢�����Ƕȵ�ļ��㷽��		
//	j=2- 1;//��ֱ����������ֵɨ���ĵ���ֵ-1	   
//	//j=j-2;
//	/*ͨ����õľ���ֵ�����������ʼ���ÿ�����ֱ������ֵ��X��Zֵ��*/								  //32768
// //   for(i=g_sspSetup.u16InclineZeroPos-1;i >= g_u16InclineStartAnglePt;i--)
//	{
//		if(data[i]>ThresOrigineDataLow && data[i]<ThresOrigineDataHigh)//�������£���õľ�����0.03m��20m��
//		{ 
////             g_ZdistanceI[j] = g_sspSetup.IncHeightLaser - ((data[i]*Tabcos[g_sspSetup.u16InclineZeroPos-i])>>15);//����
////			 g_YdistanceI[j]= -1*((data[i]*Tabsin[g_sspSetup.u16InclineZeroPos-i])>>15);//ɨ����X����ֵ	
//			 Dafeiflag=0;			 
//		}
//		else															 //��õľ��벻��0.03m��20m��
//		{
//		   
//		   if(Dafeiflag==0)
//		     {
//			 k++;
//		     DafeiData[k][1]=j;		 //����ɶε���ֹ������¼��DafeiData[k][1]�У���ʼ������¼��DafeiData[k][0]�У�ͬʱ��¼g_ZdistanceIΪ��ʱ��X����
//			 DafeiData[k][3]=-(Tabsin[g_sspSetup.u16InclineZeroPos-i]*(g_sspSetup.IncHeightLaser-100)/Tabcos[g_sspSetup.u16InclineZeroPos-i]);
//			 }
//		     DafeiData[k][0]=j;
//			 DafeiData[k][2]=-(Tabsin[g_sspSetup.u16InclineZeroPos-i]*(g_sspSetup.IncHeightLaser-100)/Tabcos[g_sspSetup.u16InclineZeroPos-i]);
//		     Dafeiflag=1;		 
//		}
//		j=j-1;	//������1
//	}
//    
//	DafeiData[0][0]=k;
//
///*******************************************************************/	
///*****************/
//JG_T0TC[1] = T0TC;
//JG_counter2[1]=t0_count2;
///*****************/
//	 j=g_sspSetup.u16InclineZeroPos-g_u16InclineStartAnglePt;   //��ֱ����������ֵɨ���ĵ���ֵ	 
//	 j=j-1;//j=143;			  //20140121
//	 /*ͨ����õľ���ֵ��������ͽ������ÿ�����ʵ������ֵ*/	
//	 for(i=g_sspSetup.u16InclineZeroPos;i <= g_u16InclineEndAnglePt;i++) 
//	 {
//	 	if(data[i] > ThresOrigineDataLow && data[i]<ThresOrigineDataHigh)//��õľ�����0.03m��20m��
//		{ 		
//			g_ZdistanceI[j] = g_sspSetup.IncHeightLaser - ((data[i]*Tabcos[i-g_sspSetup.u16InclineZeroPos])>>15);//����
//			g_YdistanceI[j] = ((data[i]*Tabsin[i-g_sspSetup.u16InclineZeroPos])>>15);//ɨ����X����ֵ	
//			Dafeiflag=0;
//		}
//		else															   //��õľ��벻��0.03m��20m��
//		{
//		   
//		    if(Dafeiflag==0)
//			 {
//			 k++;
//			 DafeiData[k][0]=j;
//			 DafeiData[k][2]=Tabsin[i-g_sspSetup.u16InclineZeroPos]*(g_sspSetup.IncHeightLaser-100)/Tabcos[i-g_sspSetup.u16InclineZeroPos];
//			 }
//			 DafeiData[k][1]=j;
//			 DafeiData[k][3]=Tabsin[i-g_sspSetup.u16InclineZeroPos]*(g_sspSetup.IncHeightLaser0-100)/Tabcos[i-g_sspSetup.u16InclineZeroPos];
//		     Dafeiflag=1;		       
//		} 				 //�����ɵ�ʱ��X����ʱ�����и���������Ҫʹ��g_sspSetup.HeightLaser	  zyj 20130607
//		j=j+1;	//������1
//	 }
//	 DafeiData[0][1]=k; 
//	 
//  /**********�Դ�ɵ�Ĵ���*************/
//
/////////��ƽ����///////////
//for(k=1; k<=DafeiData[0][1];k++ )
//  {
//		  /////���ڳ�β��ɵĴ���
//		if(((DafeiData[k][0]==0)||(g_ZdistanceI[DafeiData[k][0]-1]<=ThresVehLow))&&(g_ZdistanceI[DafeiData[k][1]+1]>ThresVehLow))
//		   {
//		   m=DafeiData[k][0]+g_u16InclineStartAnglePt;	 //mΪ����data[]����������β������
//		   if(g_sspSetup.u16InclineZeroPos>=m)
//		     {
//			 g_YdistanceI[DafeiData[k][0]]=-(Tabsin[g_sspSetup.u16InclineZeroPos-m]*(g_sspSetup.IncHeightLaser-g_ZdistanceI[DafeiData[k][1]+1])/Tabcos[g_sspSetup.u16InclineZeroPos-m]);
//			 }
//		   else
//		      {
//			  g_YdistanceI[DafeiData[k][0]]=Tabsin[m-g_sspSetup.u16InclineZeroPos]*(g_sspSetup.IncHeightLaser0-g_ZdistanceI[DafeiData[k][1]+1])/Tabcos[m-g_sspSetup.u16InclineZeroPos];
//			  }
//
//			g_ZdistanceI[DafeiData[k][0]] = ThresVehLow+1;//20140915 ��ɵ㴦������
//			for(i=DafeiData[k][0]+1;i<=DafeiData[k][1];i++)
//			{
//				g_ZdistanceI[i]=g_ZdistanceI[DafeiData[k][1]+1];
//				g_YdistanceI[i]=g_YdistanceI[DafeiData[k][0]]+((g_YdistanceI[DafeiData[k][1]+1]-g_YdistanceI[DafeiData[k][0]])*(i-DafeiData[k][0]))/(DafeiData[k][1]-DafeiData[k][0]+1);
//			}
//		}
//
//		//////���ڳ�ͷ��ɵĴ���
//		else if((DafeiData[k][1]==g_u16InclineEndAnglePt-g_u16InclineStartAnglePt||g_ZdistanceI[DafeiData[k][1]+1]<=ThresVehLow)&&(g_ZdistanceI[DafeiData[k][0]-1]>ThresVehLow))
//		{
//			m=DafeiData[k][1]+g_u16InclineStartAnglePt;	 //mΪ����data[]����������ͷ������
//			if(g_sspSetup.u16InclineZeroPos>=m)
//			{
//				g_YdistanceI[DafeiData[k][1]]=-(Tabsin[g_sspSetup.u16InclineZeroPos-m]*(g_sspSetup.IncHeightLaser-ThresVehLow/2)/Tabcos[g_sspSetup.u16InclineZeroPos-m]);
//			}
//			else
//			{
//				g_YdistanceI[DafeiData[k][1]]=Tabsin[m-g_sspSetup.u16InclineZeroPos]*(g_sspSetup.IncHeightLaser-ThresVehLow/2)/Tabcos[m-g_sspSetup.u16InclineZeroPos];
//			}
//			g_ZdistanceI[DafeiData[k][1]]=ThresVehLow+1;//20140915 ThresVehLow/2->ThresVehLow+1
//			for(i=DafeiData[k][0];i<DafeiData[k][1];i++)
//			{
//				g_ZdistanceI[i]=g_ZdistanceI[DafeiData[k][0]-1];
//				g_YdistanceI[i]=g_YdistanceI[DafeiData[k][0]-1]+((g_YdistanceI[DafeiData[k][1]]-g_YdistanceI[DafeiData[k][0]-1])*(i-DafeiData[k][0]+1))/(DafeiData[k][1]-DafeiData[k][0]+1);
//			}			
//		}
//		///////���ڳ���ȫ����ɵ����
//		else if((g_ZdistanceI[DafeiData[k][0]-1]<=ThresVehLow)
//				&&(g_ZdistanceI[DafeiData[k][1]+1]<=ThresVehLow)
//				&&DafeiData[k][0]<DafeiData[k][1])	//20140121 ���Ӵ�ɵ�λ�ò����
//	    {
//			m=DafeiData[k][0]+g_u16InclineStartAnglePt;	 //mΪ����data[]����������β������
//			if(g_sspSetup.u16InclineZeroPos>=m)
//			{
//				g_YdistanceI[DafeiData[k][0]]=-(Tabsin[g_sspSetup.u16InclineZeroPos-m]*(g_sspSetup.IncHeightLaser-500)/Tabcos[g_sspSetup.u16InclineZeroPos-m]);
//			}
//			else
//			{
//				g_YdistanceI[DafeiData[k][0]]=Tabsin[m-g_sspSetup.u16InclineZeroPos]*(g_sspSetup.IncHeightLaser-500)/Tabcos[m-g_sspSetup.u16InclineZeroPos];
//			}
//			
//			m=DafeiData[k][1]+g_u16InclineStartAnglePt;	 //mΪ����data[]����������ͷ������
//			if(g_sspSetup.u16InclineZeroPos>=m)
//			{
//				g_YdistanceI[DafeiData[k][1]]=-(Tabsin[g_sspSetup.u16InclineZeroPos-m]*(g_sspSetup.IncHeightLaser-ThresVehLow/2)/Tabcos[g_sspSetup.u16InclineZeroPos-m]);
//			}
//			else
//			{
//				g_YdistanceI[DafeiData[k][1]]=Tabsin[m-g_sspSetup.u16InclineZeroPos]*(g_sspSetup.IncHeightLaser-ThresVehLow/2)/Tabcos[m-g_sspSetup.u16InclineZeroPos];
//			}
//			
//			for(i=DafeiData[k][0];i<=DafeiData[k][1];i++)
//			{
//				g_ZdistanceI[i]=ThresVehLow+1;
//				g_YdistanceI[i]=g_YdistanceI[DafeiData[k][0]]+((g_YdistanceI[DafeiData[k][1]]-g_YdistanceI[DafeiData[k][0]])*(i-DafeiData[k][0]))/(DafeiData[k][1]-DafeiData[k][0]);
//			}
//	    }
//		/////////���ڳ����м���д�ɵ����
//		else if((g_ZdistanceI[DafeiData[k][0]-1]>ThresVehLow)&&(g_ZdistanceI[DafeiData[k][1]+1]>ThresVehLow))
//		    {
//			for(i=DafeiData[k][0]; i<=DafeiData[k][1];i++)
//			  {
//			  g_ZdistanceI[i]=g_ZdistanceI[DafeiData[k][0]-1]+((g_ZdistanceI[DafeiData[k][1]+1]-g_ZdistanceI[DafeiData[k][0]-1])*(i-DafeiData[k][0]+1))/(DafeiData[k][1]-DafeiData[k][0]+2);
//			  g_YdistanceI[i]=g_YdistanceI[DafeiData[k][0]-1]+((g_YdistanceI[DafeiData[k][1]+1]-g_YdistanceI[DafeiData[k][0]-1])*(i-DafeiData[k][0]+1))/(DafeiData[k][1]-DafeiData[k][0]+2);
//			  }
//		    }
//  }
//	 sendTmpData(g_ZdistanceI,361);
//  
//  /**********��ɵ㴦�����*************/	
//
///***************************������ɨ����������ת�����*******************************/
//
//
///********************************��߼�����������ת����ʼ****************************/
//	g_u16VerticalStartAnglePt = GetStartEndPt(data0,g_sspSetup.u16StartPtNum, 0, 0);	//��ֱ����������ת����ʼ��	 �����е�1��0��ʾ�ҿ�ʼ���־����2��0��ʾ��ֱ������
//	g_u16VerticalEndAnglePt   = GetStartEndPt(data0,g_sspSetup.u16EndPtNum, 1, 0);   //��ֱ����������ת��������
// 
//	//��߶�ֵ 
//	//��ֱ����������ת��
//	if (( g_u16VerticalStartAnglePt >= g_sspSetup.u16VerticalZeroPos &&  g_u16VerticalEndAnglePt >= g_sspSetup.u16VerticalZeroPos) ||
//		( g_u16VerticalStartAnglePt <= g_sspSetup.u16VerticalZeroPos &&  g_u16VerticalEndAnglePt <= g_sspSetup.u16VerticalZeroPos)) //ת��һ�ߣ���ʼ��ͽ�����������һ�࣬��װX���궼Ϊ����
//	{	//��Ҫ�޸�
//		l_tmp1 = 0;
//		for(i=g_u16VerticalStartAnglePt;i <= g_u16VerticalEndAnglePt;i++)	
//		{
//			if(data0[i]>ThresOrigineDataLow && data0[i]<ThresOrigineDataHigh)
//			{ 
//				l_DafeiPtNum = 0;
//				g_ZdistanceV[l_tmp1]= g_sspSetup.HeightLaser0 - ((data0[i]*Tabcos[abs(g_sspSetup.u16VerticalZeroPos-i)])>>15); // HeightLaser +
//				g_XdistanceV[l_tmp1]= ((data0[i]*Tabsin[abs(g_sspSetup.u16VerticalZeroPos-i)])>>15);					 
//			}
//			else
//			{
//				if (l_tmp1 == 0)	 //��1����
//				{
//					g_ZdistanceV[l_tmp1] = 0;
//					g_XdistanceV[l_tmp1]=Tabsin[abs(g_sspSetup.u16VerticalZeroPos-i)]*g_sspSetup.HeightLaser0/Tabcos[abs(g_sspSetup.u16VerticalZeroPos-i)];
//				}
//				else 
//				{
//					if (l_DafeiPtNum < MAXDAFEIPTNUM)
//					{
//						l_DafeiPtNum++;
//						g_ZdistanceV[l_tmp1]=g_ZdistanceV[l_tmp1-1];
//					}
//					else
//					{
//						g_ZdistanceV[l_tmp1]=0; 
//					}  
//					g_XdistanceV[l_tmp1]=Tabsin[abs(g_sspSetup.u16VerticalZeroPos-i)]*g_sspSetup.HeightLaser0/Tabcos[abs(g_sspSetup.u16VerticalZeroPos-i)] 
//					- Tabsin[abs(g_sspSetup.u16VerticalZeroPos-i)]*g_ZdistanceV[l_tmp1-1]/Tabcos[abs(g_sspSetup.u16VerticalZeroPos-i)];
//				}	
//			}
//			l_tmp1++;
//		}		
//	}
//	else   //��ʼ��ͽ���������������
//	{
//		l_tmp1 = g_sspSetup.u16VerticalZeroPos - g_u16VerticalStartAnglePt - 1;
//		for(i=g_sspSetup.u16VerticalZeroPos-1;i >= g_u16VerticalStartAnglePt;i--)	//��װС��X����Ϊ��
//		{
//			if(data0[i]>ThresOrigineDataLow && data0[i]<ThresOrigineDataHigh)
//			{ 
//				l_DafeiPtNum = 0;
//				g_ZdistanceV[l_tmp1]= g_sspSetup.HeightLaser0 - ((data0[i]*Tabcos[g_sspSetup.u16VerticalZeroPos-i])>>15); // HeightLaser +
//				g_XdistanceV[l_tmp1]= -1*((data0[i]*Tabsin[g_sspSetup.u16VerticalZeroPos-i])>>15) + g_sspSetup.n32LaserHorizOff;  //g_sspSetup.u32LaserHorizOffӦ�ö���Ϊint32�ͣ�Ϊ�����㷽���ƶ���Ϊ��ֵʱ��С�㷽���ƶ�					 
//			}
//			else
//			{
//				if (l_DafeiPtNum < MAXDAFEIPTNUM)
//				{
//					l_DafeiPtNum++;
//					if (data0[i+1]>ThresOrigineDataLow)
//					{
//						g_ZdistanceV[l_tmp1] = g_ZdistanceV[l_tmp1+1];
//					}
//					else
//					{
//						g_ZdistanceV[l_tmp1] = ThresVehLow+1;
//					}
//					 
//				}
//				else
//				{
//					g_ZdistanceV[l_tmp1] =  ThresVehLow+1;//20140916
//				}  
//				g_XdistanceV[l_tmp1]=-(Tabsin[g_sspSetup.u16VerticalZeroPos-i]*(g_sspSetup.HeightLaser0-g_ZdistanceV[l_tmp1+1])/Tabcos[g_sspSetup.u16VerticalZeroPos-i])
//					+ g_sspSetup.n32LaserHorizOff;	
//			}			 //�޸Ĵ�ɵ� X����ļ��㷽��  zyj 20130609
//			l_tmp1=l_tmp1-1;
//		}	
//
//		l_tmp1 = g_sspSetup.u16VerticalZeroPos - g_u16VerticalStartAnglePt;
//		l_DafeiPtNum = 0;
//		for(i=g_sspSetup.u16VerticalZeroPos;i < g_u16VerticalEndAnglePt;i++) 	//��װ���X����Ϊ��
//		{
//			if(data0[i] > ThresOrigineDataLow && data0[i]<ThresOrigineDataHigh)
//			{ 	
//				l_DafeiPtNum = 0;	
//				g_ZdistanceV[l_tmp1] = g_sspSetup.HeightLaser - ((data0[i]*Tabcos[i-g_sspSetup.u16VerticalZeroPos])>>15);//
//				g_XdistanceV[l_tmp1] = ((data0[i]*Tabsin[i-g_sspSetup.u16VerticalZeroPos])>>15) + g_sspSetup.n32LaserHorizOff;//
//			}
//			else
//			{
//				if (l_DafeiPtNum < MAXDAFEIPTNUM)
//				{
//					l_DafeiPtNum++;
//					if (data0[i-1]>ThresOrigineDataLow)
//					{
//						g_ZdistanceV[l_tmp1] = g_ZdistanceV[l_tmp1-1];
//					}
//					else
//					{
//						g_ZdistanceV[l_tmp1] = ThresVehLow+1;//20140916
//					}
//					
//				}
//				else
//				{
//					g_ZdistanceV[l_tmp1] = ThresVehLow+1;
//				}
//				g_XdistanceV[l_tmp1]= Tabsin[i-g_sspSetup.u16VerticalZeroPos]*(g_sspSetup.HeightLaser-g_ZdistanceV[l_tmp1-1])/Tabcos[i-g_sspSetup.u16VerticalZeroPos]	
//				+ g_sspSetup.n32LaserHorizOff;		   //20130614  �޸ģ���ǰ������������������
//			} 
//			l_tmp1=l_tmp1+1;
//		}	
//	} //��ֱ��������ת����ϣ�
//
///********************************��߼�����������ת������****************************/  
//
///********************************��߼����г�������ҿ�ʼ***************************************/
//    l_32tmpValue = g_u16VerticalEndAnglePt-g_u16VerticalStartAnglePt+1;
//	//֡���ݽ��з��г�����,��¼ÿ�����������λ����Ϣ
//	l_leftPt = GetLimitValue(g_ZdistanceV,g_XdistanceV,l_32tmpValue,0, 0);		 //ÿ֡�����г��������ʼ��
//	l_rightPt = GetLimitValue(g_ZdistanceV,g_XdistanceV,l_32tmpValue,g_u16VerticalEndAnglePt-g_u16VerticalStartAnglePt, 0);	   //ÿ֡�������г�����ֹ��
//
//	//20140217 ���Ӷ�l_leftPt��l_rightPt�ĺϷ����ж�
//	//��ߵ�����С���ұߵ������������ҵ�����������Ч��ֵ��Χ�ڣ���������
//	if (!((l_leftPt < l_rightPt)&& (l_rightPt <= g_u16VerticalEndAnglePt)))
//	{
//		return;
//	}
//	l_u16index = 0;   //�г�������������ֲ�����Ϊ��
//	for(i = l_leftPt;i<=l_rightPt;i++)
//	{
//		if( (!g_sspSetup.u8InstallFlag && ((abs(g_XdistanceV[i]) < g_NearMaxWide && abs(g_XdistanceV[i]) > g_NearMinWide ) || 
//		   (abs(g_XdistanceV[i]) > g_FarMinWide && abs(g_XdistanceV[i]) < g_FarMaxWide))) ||  //��װ��ʽ
//		   	(g_sspSetup.u8InstallFlag && ((g_XdistanceV[i]<g_MedianLeftWide && abs(g_XdistanceV[i])>g_MedianLeftWide &&
//			abs(g_XdistanceV[i])<g_MaxLeftWide) ||(g_XdistanceV[i]>g_MedianRightWide && abs(g_XdistanceV[i])<g_MaxRightWide)))	//��װ��ʽ
//		   )	//�жϲ��ڸ������Χ
//		{
//			if((g_ZdistanceV[i] > ThresVehLow) && (g_ZdistanceV[i] < ThresVehHigh))	
//			{	 			
//				if(l_FrameInfo1.Ptdata[l_u16index].n32xLeft == 0)		  //ÿ֡���г����ֵĵ�1����
//				{
//					l_FrameInfo1.Ptdata[l_u16index].n32xLeft = g_XdistanceV[i];
//					l_FrameInfo1.Ptdata[l_u16index].n32xRight = g_XdistanceV[i];
//					l_FrameInfo1.Ptdata[l_u16index].u16Leftpt  = i;
//					l_FrameInfo1.Ptdata[l_u16index].u16Rightpt = i;
//					l_FrameInfo1.Ptdata[l_u16index].u16xDis = 0;
//	
//					l_FrameInfo1.Ptdata[l_u16index].u16xMaxHt = g_ZdistanceV[i];
//					
//					if(l_u16index)	  //��֡�������г�
//					{ 
//						l_u16tmp = abs(l_FrameInfo1.Ptdata[l_u16index - 1].n32xLeft - l_FrameInfo1.Ptdata[l_u16index - 1].n32xRight);
//						l_FrameInfo1.Ptdata[l_u16index - 1].u16xDis = l_u16tmp;	  //��֡�������г����ֵĿ��
//	
//						if(ISVehRegion(l_u16tmp, &l_FrameInfo1.Ptdata[l_u16index-1], g_ZdistanceV))	  	   //��Ч��ȴ���300mm���г���������4����,��Ϊ�Ǹ�֡���µ��г�����
//							l_FrameInfo1.u8Sum = (l_FrameInfo1.u8Sum+1)&POINTSET_MASK;			//�г������1
//						else 	 //��Ч���С��300mm ,��Ϊ����Ч���� ������ǰһ�����ĵ�
//						{
//							memcpy(&l_FrameInfo1.Ptdata[l_u16index - 1],&l_FrameInfo1.Ptdata[l_u16index],sizeof(PointStruct));
//							memset(&l_FrameInfo1.Ptdata[l_u16index],0,sizeof(PointStruct));						
//		
//							l_u16index--;	  				
//						} 					
//					}
//					else
//						l_FrameInfo1.u8Sum = (l_FrameInfo1.u8Sum+1)&POINTSET_MASK;	
//				}				
//				else
//				{
//					if ( (!g_sspSetup.u8InstallFlag && ((l_FrameInfo1.Ptdata[l_u16index].n32xLeft <g_NearMaxWide && g_XdistanceV[i]<g_NearMaxWide ) ||
//						(l_FrameInfo1.Ptdata[l_u16index].n32xLeft > g_FarMinWide && g_XdistanceV[i] > g_FarMinWide))) //��װ��ʽ�����г�����ĵ�һ��������һ�����ֵ�����һ��
//						  || (g_sspSetup.u8InstallFlag && ((l_FrameInfo1.Ptdata[l_u16index].n32xLeft<0 && g_XdistanceV[i]<0) ||
//						     (l_FrameInfo1.Ptdata[l_u16index].n32xLeft>0 && g_XdistanceV[i]>0))) ) //��ֹ��װ������,��װ���������ֵ���ϲ�ͬ 20140217 �޸�
//					{
//						l_FrameInfo1.Ptdata[l_u16index].n32xRight = g_XdistanceV[i];
//						l_FrameInfo1.Ptdata[l_u16index].u16Rightpt = i;
//						l_FrameInfo1.Ptdata[l_u16index].u16xDis = abs(g_XdistanceV[i] - l_FrameInfo1.Ptdata[l_u16index].n32xLeft);
//					
//						l_32tmp = l_FrameInfo1.Ptdata[l_u16index].u16xMaxHt;	
//						if(g_ZdistanceV[i] > l_32tmp)				
//							l_FrameInfo1.Ptdata[l_u16index].u16xMaxHt = g_ZdistanceV[i];  //ȡ���ֵΪ��
//					}
//					else
//					{
//						l_u16index = l_FrameInfo1.u8Sum;
//					}	 				 
//				}																	 		
//			}
//			else
//			{
//				l_u16index = l_FrameInfo1.u8Sum;	
//			}
//		} 
//		else
//		{
//			l_u16index = l_FrameInfo1.u8Sum;
//		}
//	}  
//
//  	if(l_FrameInfo1.u8Sum)	 //��Ե�ǰ֡�����һ������ ����Ϊ��ǰ֡�����һ��������������ʱû�г�����
//	{
//		l_u16index = l_FrameInfo1.u8Sum - 1;
//		l_u16tmp = abs(l_FrameInfo1.Ptdata[l_u16index].n32xLeft - l_FrameInfo1.Ptdata[l_u16index].n32xRight);
//		l_FrameInfo1.Ptdata[l_u16index].u16xDis = l_u16tmp;
//		
//		if(!ISVehRegion(l_u16tmp, &l_FrameInfo1.Ptdata[l_u16index], g_ZdistanceV)) 	  //��Ч���С��300mm , �г��������С�����ٵ�����,Ϊ����Ч����  
//		{		   
//			memset(&l_FrameInfo1.Ptdata[l_u16index],0,sizeof(PointStruct));
//			l_FrameInfo1.u8Sum--;					
//		}
//	}
//
//  //������ǰ֡�����������ص�����Ĳ���
//	for (l_u16index = 1;l_u16index<l_FrameInfo1.u8Sum;l_u16index++)
//	{
//		if(IS_INSIDE(l_FrameInfo1.Ptdata[l_u16index - 1].n32xLeft,l_FrameInfo1.Ptdata[l_u16index - 1].n32xRight,l_FrameInfo1.Ptdata[l_u16index].n32xLeft,l_FrameInfo1.Ptdata[l_u16index].n32xRight))
//		{
//			//		
//			l_FrameInfo1.Ptdata[l_u16index - 1].n32xRight = l_FrameInfo1.Ptdata[l_u16index].n32xRight;		
//			l_FrameInfo1.Ptdata[l_u16index - 1].u16Rightpt = l_FrameInfo1.Ptdata[l_u16index].u16Rightpt;
//			l_FrameInfo1.Ptdata[l_u16index - 1].u16xMaxHt = max(l_FrameInfo1.Ptdata[l_u16index - 1].u16xMaxHt,l_FrameInfo1.Ptdata[l_u16index].u16xMaxHt);
//			l_FrameInfo1.Ptdata[l_u16index - 1].u16xDis = abs(l_FrameInfo1.Ptdata[l_u16index - 1].n32xRight - l_FrameInfo1.Ptdata[l_u16index - 1].n32xLeft);
//			for (l_u16tmp = l_u16index;l_u16tmp<l_FrameInfo1.u8Sum - 1;l_u16tmp++)
//			{
//				memcpy(&l_FrameInfo1.Ptdata[l_u16tmp],&l_FrameInfo1.Ptdata[l_u16tmp+1],sizeof(PointStruct));
//			}
//			memset(&l_FrameInfo1.Ptdata[l_u16tmp],0,sizeof(PointStruct));
//			l_FrameInfo1.u8Sum--;
//			l_u16index--;
//
//		}
//	}
//  //�ϲ����򣬸��ݳ����Ŀ�ȣ���һ����������ڲ��ܳ���2�������ϵĳ�
//  	if(!g_sspSetup.u8InstallFlag)  //��װ��ʽ
//	{
//		RegionMerging(&l_FrameInfo1);
//	}
//	else	  //��װ��ʽ
//	{
//		RegionMergingEx(&l_FrameInfo1);		
//	}
//
///********************************��߼����г�������ҽ���***************************************/
//
//   //���������ƥ�����
//   for(l_u16index = 0;l_u16index < l_FrameInfo1.u8Sum;l_u16index++)
//   {
//      	if (l_u8Count++ > g_sspSetup.u8LaneNum)		 //����ֵ���ڳ�������������ѭ������ֹ����������ѭ��
//		{	
//			l_u8Count = 0;
//			break;
//		}
//
//   		l_32tmp = 0;  //ƥ���ʶ���ɹ���1��
//		l_leftX = l_FrameInfo1.Ptdata[l_u16index].n32xLeft;	   //ÿ֡���г����ֵ����  
//		l_rightX = l_FrameInfo1.Ptdata[l_u16index].n32xRight;	//�г����ֵĽ�����	
//		l_leftXpt = l_FrameInfo1.Ptdata[l_u16index].u16Leftpt;
//		l_rightXpt = l_FrameInfo1.Ptdata[l_u16index].u16Rightpt;
//
//		if (l_leftXpt >= l_rightXpt) //�����ߵ����������ұߵ㣬���г����򲻺��� 20140217 ����
//		{
//			continue;
//		}
//
//		i = 0;
//		if (g_totalVehicle > VEHICLE_MAX)
//			break;
//
//        for(j = 0;j < g_totalVehicle;j++)
//		{  	
//			i = (g_VehicleSetIndex[j] - 1) & VEHICLE_MASK;	//20140217 	��g_VehicleSetIndex[j] - 1�޸�Ϊ(g_VehicleSetIndex[j] - 1) & VEHICLE_MASK
//			if(g_VehicleSet[i].u8Vstate == OCCURING_USED || (g_VehicleSet[i].u8Vstate == NO_USED))
//			{
//				l_32tmp2 =g_VehicleSet[i].Vdata.u16FrameCnt &  FRAME_MASK;
//				if(IS_INSIDE(l_leftX,l_rightX,g_VehicleSet[i].locateX.n32xLeft,g_VehicleSet[i].locateX.n32xRight) ||
//				  (RegionMatch_Point(&g_VehicleSet[i].VLocateX, &g_VehicleSet[i].locateX, &l_FrameInfo1.Ptdata[l_u16index], 0))
//				  || (l_32tmp2>=2 && abs(l_rightX-l_leftX)*2<g_VehicleSet[i].Vdata.xMax[l_32tmp2-2] && g_VehicleSet[i].Vdata.xMax[l_32tmp2-1]<g_VehicleSet[i].Vdata.xMax[l_32tmp2-2]*2/3 && IS_INSIDE(l_leftX,l_rightX,g_VehicleSet[i].Vdata.xdata[l_32tmp2-2][1],g_VehicleSet[i].Vdata.xdata[l_32tmp2-2][g_VehicleSet[i].Vdata.xdata[l_32tmp2-2][0]])))	 //��ƥ�䲻�ܿ缤��������
//				{
//					l_32tmp = 1;  //����ƥ��ɹ�
//					g_VehicleSet[i].u8Vstate = OCCURING_USED;	
//					if(l_FrameInfo1.Ptdata[l_u16index].u16xDis <= g_LaneWide)
//					{  //����
//						i = RegionMatch(l_leftX, l_rightX, g_VehicleSet, i); //Ѱ�ҵ�ǰ������ǰһ֡�����ƥ������
//						//20140217 ����
//						if (i == ERRORVALUE || i > VEHICLE_MASK)  //����ֵ��������ѭ��
//						{
//							break;
//						}								
//						g_VehicleSet[i].locateX.n32xLeft = min(l_leftX,g_VehicleSet[i].locateX.n32xLeft);	 //	�����ʼ��ֵ�ı� ��Ҫ�޸�
//						g_VehicleSet[i].locateX.n32xRight = max(l_rightX,g_VehicleSet[i].locateX.n32xRight);  //	��Ƚ�����ֵ�ı�  ��Ҫ�޸�
//						//�޸� 20140805
//						if (l_rightX - l_leftX > MINWIDE_THRESHOLD ||
//						   (!g_sspSetup.u8InstallFlag))	 //��ǰ���������ȴ�����ֵ,����   ���߲�װ
//						{
//							g_VehicleSet[i].locateX.n32xLeft  = l_leftX;
//							g_VehicleSet[i].locateX.n32xRight = l_rightX;
//						}
//						l_32tmp2 =g_VehicleSet[i].Vdata.u16FrameCnt &  FRAME_MASK;
//						l_leftPt  = l_FrameInfo1.Ptdata[l_u16index].u16Leftpt;
//						l_rightPt = l_FrameInfo1.Ptdata[l_u16index].u16Rightpt;
//						if(l_32tmp2 > 0 && Time_Vertical == g_VehicleSet[i].Vdata.tdata[l_32tmp2 - 1] && l_rightPt>l_leftPt)
//						{
//							//ͬһ֡�У�������������ͬһ����   ��Ҫ�޸�
//							g_VehicleSet[i].locateX.u16Leftpt = min(g_VehicleSet[i].locateX.u16Leftpt,l_FrameInfo1.Ptdata[l_u16index].u16Leftpt);
//							g_VehicleSet[i].locateX.u16Rightpt = max(g_VehicleSet[i].locateX.u16Rightpt,l_FrameInfo1.Ptdata[l_u16index].u16Rightpt);
//							if (l_rightX - l_leftX > MINWIDE_THRESHOLD || (!g_sspSetup.u8InstallFlag))
//							{
//								g_VehicleSet[i].locateX.u16Leftpt = l_leftPt;
//								g_VehicleSet[i].locateX.u16Rightpt = l_rightPt;
//								g_VehicleSet[i].VLocateX.u16Leftpt = l_leftPt;
//								g_VehicleSet[i].VLocateX.u16Rightpt = l_rightPt;
//								g_VehicleSet[i].locateX.u16Startpt = g_u16VerticalStartAnglePt;
//								g_VehicleSet[i].VLocateX.u16Startpt = g_u16VerticalStartAnglePt;
//								g_VehicleSet[i].Vdata.xMax[l_32tmp2] = l_FrameInfo1.Ptdata[l_u16index].u16xDis; 
//							}
//							if((l_rightPt - l_leftPt+1)>=0 && (l_rightPt - l_leftPt+1)<=(FRAME_BUFLEN-1))	   // �ж�Խ��erro1
//							{
//								memcpy(&g_VehicleSet[i].Vdata.zdata[l_32tmp2][1],g_ZdistanceV + l_leftPt,sizeof(int32)*(l_rightPt - l_leftPt+1));
//								memcpy(&g_VehicleSet[i].Vdata.xdata[l_32tmp2][1],g_XdistanceV + l_leftPt,sizeof(int32)*(l_rightPt - l_leftPt+1));
//							 	g_VehicleSet[i].Vdata.zdata[l_32tmp2][0]  = l_rightPt - l_leftPt + 1;
//								g_VehicleSet[i].Vdata.xdata[l_32tmp2][0]  = l_rightPt - l_leftPt + 1;
//							}
//							else
//							{
//							  	memcpy(&g_VehicleSet[i].Vdata.zdata[l_32tmp2][1],g_ZdistanceV + l_leftPt,sizeof(int32)*(FRAME_BUFLEN-1));
//								memcpy(&g_VehicleSet[i].Vdata.xdata[l_32tmp2][1],g_XdistanceV + l_leftPt,sizeof(int32)*(FRAME_BUFLEN-1));
//							 	g_VehicleSet[i].Vdata.zdata[l_32tmp2][0]  = FRAME_BUFLEN-1;
//								g_VehicleSet[i].Vdata.xdata[l_32tmp2][0]  = FRAME_BUFLEN-1;
//							}
//						}
//						else
//						{
//							l_leftPt  = l_FrameInfo1.Ptdata[l_u16index].u16Leftpt;
//							l_rightPt = l_FrameInfo1.Ptdata[l_u16index].u16Rightpt;
//							if (l_rightPt > l_leftPt)
//							{
//								if(g_VehicleSet[i].Vdata.u16FrameCnt < FRAME_MASK )
//								{
//									if((l_rightPt - l_leftPt+1)>=0 && (l_rightPt - l_leftPt+1)<=(FRAME_BUFLEN-1))	   // �ж�Խ��
//									{ 
//										memcpy(&g_VehicleSet[i].Vdata.zdata[l_32tmp2][1],g_ZdistanceV + l_leftPt,sizeof(int32)*(l_rightPt - l_leftPt+1));
//										memcpy(&g_VehicleSet[i].Vdata.xdata[l_32tmp2][1],g_XdistanceV + l_leftPt,sizeof(int32)*(l_rightPt - l_leftPt+1));
//										g_VehicleSet[i].Vdata.zdata[l_32tmp2][0] = l_rightPt - l_leftPt + 1;
//										g_VehicleSet[i].Vdata.xdata[l_32tmp2][0]  = l_rightPt - l_leftPt + 1;
//									}
//									else
//									{
//									 	memcpy(&g_VehicleSet[i].Vdata.zdata[l_32tmp2][1],g_ZdistanceV + l_leftPt,sizeof(int32)*(FRAME_BUFLEN-1));
//										memcpy(&g_VehicleSet[i].Vdata.xdata[l_32tmp2][1],g_XdistanceV + l_leftPt,sizeof(int32)*(FRAME_BUFLEN-1));
//										g_VehicleSet[i].Vdata.zdata[l_32tmp2][0] = FRAME_BUFLEN-1;
//										g_VehicleSet[i].Vdata.xdata[l_32tmp2][0] = FRAME_BUFLEN-1;
//									}
//
//									g_VehicleSet[i].Vdata.xMax[l_32tmp2] = l_FrameInfo1.Ptdata[l_u16index].u16xDis; 
//									g_VehicleSet[i].Vdata.zMax[l_32tmp2] = 	l_FrameInfo1.Ptdata[l_u16index].u16xMaxHt; 						
//								}
//								if (l_rightX - l_leftX > MINWIDE_THRESHOLD || (!g_sspSetup.u8InstallFlag))
//								{
//									g_VehicleSet[i].locateX.u16Leftpt = l_FrameInfo1.Ptdata[l_u16index].u16Leftpt;
//									g_VehicleSet[i].locateX.u16Rightpt = l_FrameInfo1.Ptdata[l_u16index].u16Rightpt;
//								    g_VehicleSet[i].VLocateX.u16Leftpt = g_VehicleSet[i].locateX.u16Leftpt;
//								    g_VehicleSet[i].VLocateX.u16Rightpt = g_VehicleSet[i].locateX.u16Rightpt;
//									g_VehicleSet[i].locateX.u16Startpt = g_u16VerticalStartAnglePt;
//									g_VehicleSet[i].VLocateX.u16Startpt = g_u16VerticalStartAnglePt;
//									g_VehicleSet[i].locateX.u16xDis = abs(g_VehicleSet[i].locateX.n32xRight - g_VehicleSet[i].locateX.n32xLeft);
//								}
//								g_VehicleSet[i].Vdata.tdata[l_32tmp2] = Time_Vertical;
//							}
//							if (g_VehicleSet[i].Vdata.u16FrameCnt < FRAME_MASK )
//							{
//								g_VehicleSet[i].Vdata.u16FrameCnt++;	
//							}
//							else
//							{	
//								g_VehicleSet[i].Vdata.u16FrameCnt = FRAME_MASK;
//							}					
//						} 						
//						g_VehicleSet[i].VemptFrame = 0;										
//					}
//					else
//					{
//						l_32tmp2 =g_VehicleSet[i].Vdata.u16FrameCnt &  FRAME_MASK;
//						//���� 
//						if((g_sspSetup.u8LaneNum==6) && abs(g_VehicleSet[i].locateX.n32xRight - l_rightX) > VEHICHLE_DISTANT_GAP && 
//						   abs(g_VehicleSet[i].locateX.n32xLeft - l_leftX) > VEHICHLE_DISTANT_GAP &&
//						   g_VehicleSet[i].locateX.n32xRight < l_rightX &&
//						   g_VehicleSet[i].locateX.n32xLeft > l_leftX)	//6�����������м�����ƥ�� ǰһ֡�ĳ��������ڵ�ǰ������м䲿��
//						   {
//							   	//����ָ��������ͬ������
//								//ԭ������֡����   								
//								l_leftPt  = GetPosFromXDistance(g_XdistanceV,&g_VehicleSet[i].locateX,&l_FrameInfo1.Ptdata[l_u16index],0);
//								l_rightPt = GetPosFromXDistance(g_XdistanceV,&g_VehicleSet[i].locateX,&l_FrameInfo1.Ptdata[l_u16index],1);   //��λ�ò�֧����͵��ڳ�����
//								//20140217 ���ӶԷ���ֵ���ж�
//								if (l_leftPt >= POINT_SUM-1 || l_rightPt >= POINT_SUM-1)
//								{
//									break;	  //����ֵ�쳣������ѭ��
//								}
//
//								if (l_rightPt>l_leftPt)
//								{
//								   	if(g_VehicleSet[i].Vdata.u16FrameCnt < FRAME_MASK)
//									{	
//										if((l_rightPt - l_leftPt+1)>=0 && (l_rightPt - l_leftPt+1)<=(FRAME_BUFLEN-1))	   // �ж�Խ��
//										{   			
//											memcpy(&g_VehicleSet[i].Vdata.zdata[l_32tmp2][1],g_ZdistanceV + l_leftPt,sizeof(int32)*(l_rightPt - l_leftPt+1));
//											memcpy(&g_VehicleSet[i].Vdata.xdata[l_32tmp2][1],g_XdistanceV + l_leftPt,sizeof(int32)*(l_rightPt - l_leftPt+1));
//											g_VehicleSet[i].Vdata.zdata[l_32tmp2][0] = l_rightPt - l_leftPt + 1;
//											g_VehicleSet[i].Vdata.xdata[l_32tmp2][0] = l_rightPt - l_leftPt + 1;
//										}
//										else
//										{
//										 	memcpy(&g_VehicleSet[i].Vdata.zdata[l_32tmp2][1],g_ZdistanceV + l_leftPt,sizeof(int32)*(FRAME_BUFLEN-1));
//											memcpy(&g_VehicleSet[i].Vdata.xdata[l_32tmp2][1],g_XdistanceV + l_leftPt,sizeof(int32)*(FRAME_BUFLEN-1));
//											g_VehicleSet[i].Vdata.zdata[l_32tmp2][0] = FRAME_BUFLEN-1;
//											g_VehicleSet[i].Vdata.xdata[l_32tmp2][0] = FRAME_BUFLEN-1;
//
//										} 								
//										g_VehicleSet[i].Vdata.xMax[l_32tmp2] = abs(g_XdistanceV[l_rightPt] - g_XdistanceV[l_leftPt]);										
//										g_VehicleSet[i].Vdata.zMax[l_32tmp2] = GetMaxValue(g_ZdistanceV,l_leftPt,l_rightPt); 														
//									}
//									if (g_XdistanceV[l_rightPt] - g_XdistanceV[l_leftPt] > MINWIDE_THRESHOLD ||
//										(!g_sspSetup.u8InstallFlag))
//									{
//										g_VehicleSet[i].locateX.u16Startpt = g_u16VerticalStartAnglePt;
//										g_VehicleSet[i].VLocateX.u16Startpt = g_u16VerticalStartAnglePt;	 	
//										g_VehicleSet[i].locateX.u16Leftpt = l_leftPt;
//										g_VehicleSet[i].locateX.u16Rightpt = l_rightPt;
//										g_VehicleSet[i].VLocateX.u16Leftpt = g_VehicleSet[i].locateX.u16Leftpt;
//								    	g_VehicleSet[i].VLocateX.u16Rightpt = g_VehicleSet[i].locateX.u16Rightpt;
//										g_VehicleSet[i].locateX.n32xLeft = 	g_XdistanceV[l_leftPt];
//										g_VehicleSet[i].locateX.n32xRight = g_XdistanceV[l_rightPt];
//										g_VehicleSet[i].locateX.u16xDis = abs(g_VehicleSet[i].locateX.n32xRight-g_VehicleSet[i].locateX.n32xLeft);
//									}
//									g_VehicleSet[i].Vdata.tdata[l_32tmp2] = Time_Vertical;
//								}	
//								if (g_VehicleSet[i].Vdata.u16FrameCnt < FRAME_MASK )
//								{
//									g_VehicleSet[i].Vdata.u16FrameCnt++;	
//								}
//								else
//								{	
//									g_VehicleSet[i].Vdata.u16FrameCnt = FRAME_MASK;
//								}
//								g_VehicleSet[i].VemptFrame = 0;	
//											
//								//����һ���µ�����
//								if(abs(l_FrameInfo1.Ptdata[l_u16index].n32xRight - g_VehicleSet[i].locateX.n32xRight) > SMALL_AREA
//								   && l_FrameInfo1.u8Sum < POINTSET_MASK)
//								{
//									l_leftPt = 	l_rightPt;
//									l_rightPt = l_FrameInfo1.Ptdata[l_u16index].u16Rightpt;
//									l_u16tmp = l_FrameInfo1.u8Sum++;
//									l_FrameInfo1.Ptdata[l_u16tmp].u16xDis = abs(g_XdistanceV[l_rightPt] - g_XdistanceV[l_leftPt]); 
//									l_FrameInfo1.Ptdata[l_u16tmp].u16Leftpt = l_leftPt + 1;	 
//									l_FrameInfo1.Ptdata[l_u16tmp].u16Rightpt = l_rightPt;  						
//									l_FrameInfo1.Ptdata[l_u16tmp].n32xLeft  = g_XdistanceV[l_leftPt+1];  
//									l_FrameInfo1.Ptdata[l_u16tmp].n32xRight = g_XdistanceV[l_rightPt]; 							
//									l_FrameInfo1.Ptdata[l_u16tmp].u16xMaxHt	= GetMaxValue(g_ZdistanceV,l_leftPt,l_rightPt);	
//								}
//								//�޸�ԭ����					
//								l_leftPt = 	l_FrameInfo1.Ptdata[l_u16index].u16Leftpt;
//								l_rightPt = g_VehicleSet[i].locateX.u16Leftpt -  1;
//								if (l_rightPt >= POINT_SUM-1 || l_leftPt >= POINT_SUM-1)
//									break;
//								l_FrameInfo1.Ptdata[l_u16index].u16Rightpt = l_rightPt;
//								l_FrameInfo1.Ptdata[l_u16index].n32xRight = g_XdistanceV[l_rightPt];
//								l_FrameInfo1.Ptdata[l_u16index].u16xDis = abs(g_XdistanceV[l_rightPt] - g_XdistanceV[l_leftPt]);
//								if(l_FrameInfo1.Ptdata[l_u16index].u16xDis < SMALL_AREA)
//								if(!ISVehRegion(l_FrameInfo1.Ptdata[l_u16index].u16xDis,&l_FrameInfo1.Ptdata[l_u16index], g_ZdistanceV))
//								{
//								  l_u16index++;
//								}									
//						   }
//						   else if(abs(g_VehicleSet[i].locateX.n32xRight - l_rightX) < abs(g_VehicleSet[i].locateX.n32xLeft - l_leftX))  //ǰһ֡���������ڵ�ǰ����������ұ�
//						   {
// 								l_leftPt  = GetPosFromXDistance(g_XdistanceV,&g_VehicleSet[i].locateX,&l_FrameInfo1.Ptdata[l_u16index],3);
//								l_rightPt = l_FrameInfo1.Ptdata[l_u16index].u16Rightpt;   		//��λ�ò�֧����͵��ڳ�����
//
//								//20140217 ���ӶԷ���ֵ���ж�
//								if (l_leftPt >= POINT_SUM-1 || l_rightPt >= POINT_SUM-1)
//								{
//									break;	  //����ֵ�쳣������ѭ��
//								}
//
//								if (l_rightPt > l_leftPt)
//								{
//								   	if(g_VehicleSet[i].Vdata.u16FrameCnt < FRAME_MASK )
//									{
//										if((l_rightPt - l_leftPt+1)>=0 && (l_rightPt - l_leftPt+1)<=(FRAME_BUFLEN-1))	   // �ж�Խ��
//										{	  			
//											memcpy(&g_VehicleSet[i].Vdata.zdata[l_32tmp2][1],g_ZdistanceV + l_leftPt,sizeof(int32)*(l_rightPt - l_leftPt+1));
//											memcpy(&g_VehicleSet[i].Vdata.xdata[l_32tmp2][1],g_XdistanceV + l_leftPt,sizeof(int32)*(l_rightPt - l_leftPt+1));
//											g_VehicleSet[i].Vdata.zdata[l_32tmp2][0] = 	l_rightPt - l_leftPt + 1;
//											g_VehicleSet[i].Vdata.xdata[l_32tmp2][0] = l_rightPt - l_leftPt + 1;
//										}
//										else
//										{
//											memcpy(&g_VehicleSet[i].Vdata.zdata[l_32tmp2][1],g_ZdistanceV + l_leftPt,sizeof(int32)*(FRAME_BUFLEN-1));
//											memcpy(&g_VehicleSet[i].Vdata.xdata[l_32tmp2][1],g_XdistanceV + l_leftPt,sizeof(int32)*(FRAME_BUFLEN-1));
//											g_VehicleSet[i].Vdata.zdata[l_32tmp2][0] = FRAME_BUFLEN-1;
//											g_VehicleSet[i].Vdata.xdata[l_32tmp2][0] = FRAME_BUFLEN-1;										
//										}
//
//
//										g_VehicleSet[i].Vdata.xMax[l_32tmp2] = abs(g_XdistanceV[l_rightPt] - g_XdistanceV[l_leftPt]); 								
//										g_VehicleSet[i].Vdata.zMax[l_32tmp2] = GetMaxValue(g_ZdistanceV,l_leftPt,l_rightPt);																							
//									
//									}
//									if (l_FrameInfo1.Ptdata[l_u16index].n32xRight-g_XdistanceV[l_leftPt] > MINWIDE_THRESHOLD ||
//										(!g_sspSetup.u8InstallFlag))
//									{
//										g_VehicleSet[i].locateX.u16Startpt = g_u16VerticalStartAnglePt;
//										g_VehicleSet[i].VLocateX.u16Startpt = g_u16VerticalStartAnglePt;
//										g_VehicleSet[i].locateX.n32xLeft =g_XdistanceV[l_leftPt];
//										g_VehicleSet[i].locateX.u16Leftpt = l_leftPt;
//										g_VehicleSet[i].locateX.u16Rightpt = l_rightPt;
//										g_VehicleSet[i].VLocateX.u16Leftpt = g_VehicleSet[i].locateX.u16Leftpt;
//								    	g_VehicleSet[i].VLocateX.u16Rightpt = g_VehicleSet[i].locateX.u16Rightpt;
//										g_VehicleSet[i].locateX.n32xRight = l_FrameInfo1.Ptdata[l_u16index].n32xRight;	
//										g_VehicleSet[i].locateX.u16xDis = abs(g_VehicleSet[i].locateX.n32xRight-g_VehicleSet[i].locateX.n32xLeft);
//									}
//									g_VehicleSet[i].Vdata.tdata[l_32tmp2] = Time_Vertical;
//								} 	
//								if (g_VehicleSet[i].Vdata.u16FrameCnt < FRAME_MASK )
//								{
//									g_VehicleSet[i].Vdata.u16FrameCnt++;	
//								}
//								else
//								{	
//									g_VehicleSet[i].Vdata.u16FrameCnt = FRAME_MASK;
//								} 
//								g_VehicleSet[i].VemptFrame = 0;	
//							   	 //�������	  	
//								 l_FrameInfo1.Ptdata[l_u16index].n32xRight = g_VehicleSet[i].locateX.n32xLeft;
//								if (g_u8JGFXFlag)  //�������㣬������ɨ��������ӷ�����x��ֵ����һ��
//								{
//									if (g_XdistanceV[l_leftPt - 1] <= g_XdistanceV[l_leftPt] && l_leftPt >=1)
//									 	l_FrameInfo1.Ptdata[l_u16index].n32xRight = g_XdistanceV[l_leftPt - 1];
//									else
//										l_FrameInfo1.Ptdata[l_u16index].n32xRight = g_XdistanceV[l_leftPt];
//								}
//								else
//								{
//									if (g_XdistanceV[l_leftPt - 1] >= g_XdistanceV[l_leftPt] && l_leftPt >=1)
//									 	l_FrameInfo1.Ptdata[l_u16index].n32xRight = g_XdistanceV[l_leftPt - 1];
//									else
//										l_FrameInfo1.Ptdata[l_u16index].n32xRight = g_XdistanceV[l_leftPt];
//								}
//									 
//								 l_FrameInfo1.Ptdata[l_u16index].u16Rightpt = l_leftPt - 1;
//							 	 l_leftPt = l_FrameInfo1.Ptdata[l_u16index].u16Leftpt;
//								 l_rightPt = l_FrameInfo1.Ptdata[l_u16index].u16Rightpt;
//								 l_FrameInfo1.Ptdata[l_u16index].u16xMaxHt = GetMaxValue(g_ZdistanceV,l_leftPt,l_rightPt);	
//								 l_FrameInfo1.Ptdata[l_u16index].u16xDis = abs(l_FrameInfo1.Ptdata[l_u16index].n32xLeft-l_FrameInfo1.Ptdata[l_u16index].n32xRight);
//
//								if(!ISVehRegion(l_FrameInfo1.Ptdata[l_u16index].u16xDis,&l_FrameInfo1.Ptdata[l_u16index], g_ZdistanceV))
//								{
//								    l_u16index++;
//								}
//						   }
//						   else   //ǰһ֡���������ڵ�ǰ��������
//						   {
//								l_leftPt = l_FrameInfo1.Ptdata[l_u16index].u16Leftpt;
//								
//								l_rightPt = GetPosFromXDistance(g_XdistanceV,&g_VehicleSet[i].locateX,&l_FrameInfo1.Ptdata[l_u16index],2); //��λ�ò�֧����͵��ڳ�����
//								//20140217 ���ӶԷ���ֵ���ж�
//								if (l_leftPt >= POINT_SUM-1 || l_rightPt >= POINT_SUM-1)
//								{
//									break;	  //����ֵ�쳣������ѭ��
//								}
//
//								if (l_rightPt > l_leftPt)
//								{
//									if(g_VehicleSet[i].Vdata.u16FrameCnt < FRAME_MASK)
//									{
//										if((l_rightPt - l_leftPt+1)>=0 && (l_rightPt - l_leftPt+1)<=(FRAME_BUFLEN-1))	   // �ж�Խ��
//										{
//											memcpy(&g_VehicleSet[i].Vdata.zdata[l_32tmp2][1],g_ZdistanceV + l_leftPt,sizeof(int32)*(l_rightPt - l_leftPt+1));
//											memcpy(&g_VehicleSet[i].Vdata.xdata[l_32tmp2][1],g_XdistanceV + l_leftPt,sizeof(int32)*(l_rightPt - l_leftPt+1));
//											g_VehicleSet[i].Vdata.zdata[l_32tmp2][0] = l_rightPt - l_leftPt + 1;
//											g_VehicleSet[i].Vdata.xdata[l_32tmp2][0] = l_rightPt - l_leftPt + 1;
//										}
//										else
//										{
//											memcpy(&g_VehicleSet[i].Vdata.zdata[l_32tmp2][1],g_ZdistanceV + l_leftPt,sizeof(int32)*(FRAME_BUFLEN-1));
//											memcpy(&g_VehicleSet[i].Vdata.xdata[l_32tmp2][1],g_XdistanceV + l_leftPt,sizeof(int32)*(FRAME_BUFLEN-1));
//											g_VehicleSet[i].Vdata.zdata[l_32tmp2][0] = FRAME_BUFLEN-1;
//											g_VehicleSet[i].Vdata.xdata[l_32tmp2][0] = FRAME_BUFLEN-1;
//										}  			
//
//										g_VehicleSet[i].Vdata.xMax[l_32tmp2] = abs(g_XdistanceV[l_rightPt] - g_XdistanceV[l_leftPt]); 				
//										
//										g_VehicleSet[i].Vdata.zMax[l_32tmp2] = GetMaxValue(g_ZdistanceV,l_leftPt,l_rightPt); 														
//									}
//									if (g_XdistanceV[l_rightPt] - l_leftX > MINWIDE_THRESHOLD || (!g_sspSetup.u8InstallFlag))
//									{
//									 	g_VehicleSet[i].locateX.n32xLeft = l_leftX;
//										g_VehicleSet[i].locateX.u16Leftpt = l_FrameInfo1.Ptdata[l_u16index].u16Leftpt;
//										g_VehicleSet[i].locateX.u16Rightpt = l_rightPt;
//										g_VehicleSet[i].VLocateX.u16Leftpt = g_VehicleSet[i].locateX.u16Leftpt;
//								    	g_VehicleSet[i].VLocateX.u16Rightpt = g_VehicleSet[i].locateX.u16Rightpt;
//										g_VehicleSet[i].locateX.n32xRight = g_XdistanceV[l_rightPt];
//										g_VehicleSet[i].locateX.u16Startpt = g_u16VerticalStartAnglePt;
//										g_VehicleSet[i].VLocateX.u16Startpt = g_u16VerticalStartAnglePt;
//										g_VehicleSet[i].locateX.u16xDis = abs(g_VehicleSet[i].locateX.n32xRight-g_VehicleSet[i].locateX.n32xLeft);
//									}	
//									g_VehicleSet[i].Vdata.tdata[l_32tmp2] = Time_Vertical;
//								}	
//								if (g_VehicleSet[i].Vdata.u16FrameCnt < FRAME_MASK )
//								{
//									g_VehicleSet[i].Vdata.u16FrameCnt++;	
//								}
//								else
//								{	
//									g_VehicleSet[i].Vdata.u16FrameCnt = FRAME_MASK;
//								} 
//								g_VehicleSet[i].VemptFrame = 0;	  
//								
//								//�������	  	
//								l_FrameInfo1.Ptdata[l_u16index].n32xLeft = g_VehicleSet[i].locateX.n32xRight; 
//								if (g_u8JGFXFlag)
//								{
//									if (g_XdistanceV[l_rightPt] <= g_XdistanceV[l_rightPt+1])
//										l_FrameInfo1.Ptdata[l_u16index].n32xLeft = g_XdistanceV[l_rightPt+1];
//									else
//										l_FrameInfo1.Ptdata[l_u16index].n32xLeft = g_XdistanceV[l_rightPt];
//								}
//								else
//								{
//									if (g_XdistanceV[l_rightPt] >= g_XdistanceV[l_rightPt+1])
//										l_FrameInfo1.Ptdata[l_u16index].n32xLeft = g_XdistanceV[l_rightPt+1];
//									else
//										l_FrameInfo1.Ptdata[l_u16index].n32xLeft = g_XdistanceV[l_rightPt];
//								}
//
//								l_FrameInfo1.Ptdata[l_u16index].n32xLeft = g_XdistanceV[l_rightPt + 1];
//								l_FrameInfo1.Ptdata[l_u16index].u16Leftpt = l_rightPt + 1;
//								l_leftPt = l_FrameInfo1.Ptdata[l_u16index].u16Leftpt;
//								l_rightPt = l_FrameInfo1.Ptdata[l_u16index].u16Rightpt;
//								l_FrameInfo1.Ptdata[l_u16index].u16xMaxHt = GetMaxValue(g_ZdistanceV,l_leftPt,l_rightPt);	
//								l_FrameInfo1.Ptdata[l_u16index].u16xDis = abs(l_FrameInfo1.Ptdata[l_u16index].n32xLeft-l_FrameInfo1.Ptdata[l_u16index].n32xRight);
//
//								if(!ISVehRegion(l_FrameInfo1.Ptdata[l_u16index].u16xDis,&l_FrameInfo1.Ptdata[l_u16index], g_ZdistanceV))
//								{
//								    l_u16index++;
//								}								 	
//						   }													 	  							 								
//						 l_u16index--; 										
//					} 
//				  break;
//				}  
//			}
//			i++;		
//		}
//				
//		if(l_32tmp == 0)  //�³�����
//		{
//			if(l_FrameInfo1.Ptdata[l_u16index].u16xDis > g_LaneWide)  //���ڳ�����ȣ���Ϊ�ǲ���
//			{ 	
//				l_leftX = l_FrameInfo1.Ptdata[l_u16index].n32xLeft;
//				l_rightX = l_FrameInfo1.Ptdata[l_u16index].n32xRight;	
//				l_leftPt = l_FrameInfo1.Ptdata[l_u16index].u16Leftpt;
//				l_rightPt = l_FrameInfo1.Ptdata[l_u16index].u16Rightpt;		
//				l_u16PosVect[l_32tmp].n32xLeft = l_leftX; //��һ��λ�ô��ԭʼλ��	
//				l_u16PosVect[l_32tmp].u16Leftpt = l_leftPt;
//				l_32tmp2 = l_leftPt + 1;
//				if (l_leftPt >= POINT_SUM-1 || l_rightPt >= POINT_SUM-1)
//					continue;
//				while(l_32tmp2 < l_rightPt)
//				{ 		   
//					l_u16tmp = abs(g_XdistanceV[l_32tmp2] - l_leftX); 	
//					if(l_u16tmp >= g_LaneWide)
//					{
//						l_u16PosVect[l_32tmp].u16xDis = abs(g_XdistanceV[l_32tmp2-1]- l_leftX);
//						l_u16PosVect[l_32tmp].n32xRight = g_XdistanceV[l_32tmp2-1];
//						l_u16PosVect[l_32tmp].u16Rightpt = l_32tmp2-1;
//						l_u16PosVect[l_32tmp].u16xMaxHt = GetMaxValue(g_ZdistanceV,l_leftPt,l_32tmp2-1);	  
//		
//						l_leftPt =  l_32tmp2;
//						l_32tmp = (l_32tmp + 1) & POINTSET_MASK;
//					   	l_u16PosVect[l_32tmp].n32xLeft = g_XdistanceV[l_32tmp2]; //��һ��λ�ô��ԭʼλ��	
//						l_u16PosVect[l_32tmp].u16Leftpt = l_leftPt;
//						l_leftX = g_XdistanceV[l_32tmp2];
//					}  
//				   	l_32tmp2++;	
//				}
//				l_u16tmp = abs(l_rightX - g_XdistanceV[l_leftPt]);	
//				if(l_u16tmp > SMALL_AREA)
//				{
//					l_u16PosVect[l_32tmp].u16xDis = l_u16tmp;
//					l_u16PosVect[l_32tmp].n32xRight = l_rightX;
//					l_u16PosVect[l_32tmp].u16Rightpt = l_rightPt;
//					l_u16PosVect[l_32tmp].u16xMaxHt = GetMaxValue(g_ZdistanceV,l_leftPt,l_rightPt);
//					l_32tmp = (l_32tmp + 1) & POINTSET_MASK;
//				} //end if									
//			}
//			else
//			{
//				memcpy(&l_u16PosVect[0],&l_FrameInfo1.Ptdata[l_u16index],sizeof(PointStruct)); 
//				l_32tmp = 1;
//			}
//			if (l_32tmp > VEHICLE_MAX)
//				continue;
//			for(l_u16tmp = 0;l_u16tmp<l_32tmp;l_u16tmp++)	   //�г�����
//			{							
//			  	//������δƥ��ɹ�����Ϊ���³�
//				for(i = 0;i < VEHICLE_MAX;i++)
//				{
//					if(g_VehicleSet[i].u8Vstate == NO_USED)
//					{
//						if (g_totalVehicle >= 10)	//20140217 �޸�
//						{
//							clearVehicleErr();
//						}
//						if (g_totalVehicle >= VEHICLE_MAX)	 //20140217 ���� ����������������������������ѭ������ֹԽ��
//						{
//							break;
//						}
//						g_VehicleSetIndex[g_totalVehicle++] = i+1;
//						if(g_totalVehicle >= 10)
//							clearVehicleErr();
//
//						g_VehicleSet[i].u8Vstate = OCCURING_USED;
//
//						l_leftPt = l_u16PosVect[l_u16tmp].u16Leftpt;
//						l_rightPt = l_u16PosVect[l_u16tmp].u16Rightpt;	
//						l_leftX = l_u16PosVect[l_u16tmp].n32xLeft;
//						l_rightX = l_u16PosVect[l_u16tmp].n32xRight;	
//													
//						l_32tmp2 =g_VehicleSet[i].Vdata.u16FrameCnt &  FRAME_MASK;
//						if(g_VehicleSet[i].Vdata.u16FrameCnt < FRAME_MASK  && l_rightPt>l_leftPt)
//						{
//							if (l_u16PosVect[l_u16tmp].n32xRight-l_u16PosVect[l_u16tmp].n32xLeft > MINWIDE_THRESHOLD ||
//								(!g_sspSetup.u8InstallFlag))
//							{
//								g_VehicleSet[i].locateX.n32xLeft = l_u16PosVect[l_u16tmp].n32xLeft;
//								g_VehicleSet[i].locateX.n32xRight = l_u16PosVect[l_u16tmp].n32xRight;
//								g_VehicleSet[i].locateX.u16Leftpt = l_u16PosVect[l_u16tmp].u16Leftpt;
//								g_VehicleSet[i].locateX.u16Rightpt = l_u16PosVect[l_u16tmp].u16Rightpt;	
//								g_VehicleSet[i].VLocateX.u16Leftpt = g_VehicleSet[i].locateX.u16Leftpt;
//								g_VehicleSet[i].VLocateX.u16Rightpt = g_VehicleSet[i].locateX.u16Rightpt;
//								g_VehicleSet[i].locateX.u16xDis = abs(g_VehicleSet[i].locateX.n32xRight-g_VehicleSet[i].locateX.n32xLeft);
//								g_VehicleSet[i].locateX.u16Startpt = g_u16VerticalStartAnglePt;
//								g_VehicleSet[i].VLocateX.u16Startpt = g_u16VerticalStartAnglePt;
//							}
//							g_VehicleSet[i].Vdata.xMax[l_32tmp2] = abs(l_rightX - l_leftX);  								
//							g_VehicleSet[i].Vdata.zMax[l_32tmp2] = l_u16PosVect[l_u16tmp].u16xMaxHt; 
//							if((l_rightPt - l_leftPt+1)>=0 && (l_rightPt - l_leftPt+1)<=(FRAME_BUFLEN-1))	   // �ж�Խ��
//							{
//							  	memcpy(&g_VehicleSet[i].Vdata.zdata[l_32tmp2][1],g_ZdistanceV + l_leftPt,sizeof(int32)*(l_rightPt - l_leftPt+1));
//							  	memcpy(&g_VehicleSet[i].Vdata.xdata[l_32tmp2][1],g_XdistanceV + l_leftPt,sizeof(int32)*(l_rightPt - l_leftPt+1));
//								g_VehicleSet[i].Vdata.xdata[l_32tmp2][0] = l_rightPt - l_leftPt + 1;
//								g_VehicleSet[i].Vdata.zdata[l_32tmp2][0] = l_rightPt - l_leftPt + 1;
//							}
//							else
//							{
//								memcpy(&g_VehicleSet[i].Vdata.zdata[l_32tmp2][1],g_ZdistanceV + l_leftPt,sizeof(int32)*(FRAME_BUFLEN-1));
//							  	memcpy(&g_VehicleSet[i].Vdata.xdata[l_32tmp2][1],g_XdistanceV + l_leftPt,sizeof(int32)*(FRAME_BUFLEN-1));
//								g_VehicleSet[i].Vdata.xdata[l_32tmp2][0] = FRAME_BUFLEN-1;
//								g_VehicleSet[i].Vdata.zdata[l_32tmp2][0] = FRAME_BUFLEN-1;
//							}
//																			
//						}
//						g_VehicleSet[i].Vdata.tdata[l_32tmp2] = Time_Vertical;	
//						if (g_VehicleSet[i].Vdata.u16FrameCnt < FRAME_MASK )
//						{
//							g_VehicleSet[i].Vdata.u16FrameCnt++;	
//						}
//						else
//						{	
//							g_VehicleSet[i].Vdata.u16FrameCnt = FRAME_MASK;
//						}
//						g_VehicleSet[i].VemptFrame = 0;	
//						break;			
//					}
//				}//end for	 					
//			}
//		}
//	}
//
///*****************�ֳ���һ�׶Σ�Ԥ�ֳ�------����Ч�������򣬽���Ч���ݴ浽TmpNum[k][]������******************/
//
//	l_u16StartPt = 0;
//	l_u16EndPt = g_u16InclineEndAnglePt-g_u16InclineStartAnglePt+1;
//	l_u16index = l_FrameInfo.u8Sum;
//	for (i=l_u16StartPt; i<l_u16EndPt; i++)
//	{
//		if ((g_ZdistanceI[i] >= ThresVehLow)&&(g_ZdistanceI[i] <= ThresVehHigh))
//		{
//			if (!l_FrameInfo.uValid[l_u16index])
//			{
//				l_FrameInfo.uValid[l_u16index] = 1;
//							
//				l_FrameInfo.IncPtdata[l_u16index].u16Pt1 = i;
//				l_FrameInfo.IncPtdata[l_u16index].u16Pt2 = i;
//				l_FrameInfo.IncPtdata[l_u16index].n32y1 = g_YdistanceI[i];
//				l_FrameInfo.IncPtdata[l_u16index].n32y2 = g_YdistanceI[i];
//				l_FrameInfo.IncPtdata[l_u16index].u16yDis = 0;
//				l_FrameInfo.IncPtdata[l_u16index].u16yMaxHt = g_ZdistanceI[i];
//				
//				if (l_u16index)
//				{
//					l_u16tmp = abs(l_FrameInfo.IncPtdata[l_u16index - 1].n32y1 - l_FrameInfo.IncPtdata[l_u16index - 1].n32y2);
//					l_FrameInfo.IncPtdata[l_u16index - 1].u16yDis = l_u16tmp;	  //��֡�������г����ֵĿ��				
//				 	l_u16StartYpt = l_FrameInfo.IncPtdata[l_u16index-1].u16Pt1;
//					l_u16EndYpt = l_FrameInfo.IncPtdata[l_u16index-1].u16Pt2;
//					if (l_u16tmp>500 && l_u16EndYpt-l_u16StartYpt+1>=3)
//					{
//						l_FrameInfo.u8Sum = (l_FrameInfo.u8Sum+1)&POINTSET_MASK;
//					} //�м���ּ�ϵ�����
//					else if ((abs(l_u16EndYpt-l_FrameInfo.IncPtdata[l_u16index].u16Pt1)<=3 && abs(l_FrameInfo.IncPtdata[l_u16index].n32y1 - l_FrameInfo.IncPtdata[l_u16index-1].n32y2)<800)||
//						(abs(l_u16EndYpt-l_FrameInfo.IncPtdata[l_u16index].u16Pt1)<=10 && (abs(g_ZdistanceI[i]-l_FrameInfo.IncPtdata[l_u16index-1].u16yMaxHt)<500 || abs(l_FrameInfo.IncPtdata[l_u16index].n32y1 - l_FrameInfo.IncPtdata[l_u16index-1].n32y2)<500)))
//					{
//						l_FrameInfo.IncPtdata[l_u16index-1].u16Pt2 = i;
//						l_FrameInfo.IncPtdata[l_u16index-1].n32y2 = g_YdistanceI[i];
//						l_FrameInfo.IncPtdata[l_u16index-1].u16yDis = abs(l_FrameInfo.IncPtdata[l_u16index-1].n32y2-l_FrameInfo.IncPtdata[l_u16index-1].n32y1);											
//						l_32tmp = l_FrameInfo.IncPtdata[l_u16index-1].u16yMaxHt;	
//						if(g_ZdistanceI[i] > l_32tmp)
//						{
//							l_FrameInfo.IncPtdata[l_u16index-1].u16yMaxHt = g_ZdistanceI[i];  //ȡ���ֵΪ��				
//						}
//						l_FrameInfo.uValid[l_u16index] = 0;
//						memset(&l_FrameInfo.IncPtdata[l_u16index],0,sizeof(IncPtSt));
//						l_u16index--;
//					}
//					else if (l_FrameInfo.u8Sum == 1)
//					{
//						memcpy(&l_FrameInfo.IncPtdata[l_u16index - 1],&l_FrameInfo.IncPtdata[l_u16index],sizeof(IncPtSt));
//						l_FrameInfo.uValid[l_u16index] = 0;
//						memset(&l_FrameInfo.IncPtdata[l_u16index],0,sizeof(IncPtSt));								
//						l_u16index--;					
//					}
//					else
//					{
//						l_FrameInfo.u8Sum = (l_FrameInfo.u8Sum+1)&POINTSET_MASK;
//					}
//				}
//				else
//				{
//					l_FrameInfo.u8Sum = (l_FrameInfo.u8Sum + 1) & POINTSET_MASK;
//				}													
//			}
//			else
//			{
//				l_FrameInfo.IncPtdata[l_u16index].u16Pt2 = i;
//				l_FrameInfo.IncPtdata[l_u16index].n32y2 = g_YdistanceI[i];
//				l_FrameInfo.IncPtdata[l_u16index].u16yDis = abs(l_FrameInfo.IncPtdata[l_u16index].n32y2-l_FrameInfo.IncPtdata[l_u16index].n32y1);
//				
//				l_32tmp = l_FrameInfo.IncPtdata[l_u16index].u16yMaxHt;	
//				if(g_ZdistanceI[i] > l_32tmp)
//				{
//					l_FrameInfo.IncPtdata[l_u16index].u16yMaxHt = g_ZdistanceI[i];  //ȡ���ֵΪ��				
//				}				
//			}			
//		}
//		else
//		{
//			l_u16index = l_FrameInfo.u8Sum;
//		}
//	}
//
//������һ������
//	if (l_FrameInfo.u8Sum)
//	{
//		l_u16index = l_FrameInfo.u8Sum - 1;
//		l_u16tmp = abs(l_FrameInfo.IncPtdata[l_u16index].n32y1 - l_FrameInfo.IncPtdata[l_u16index].n32y2);
//		l_FrameInfo.IncPtdata[l_u16index].u16yDis = l_u16tmp;	  //��֡�������г����ֵĿ��				
//	 	l_u16StartYpt = l_FrameInfo.IncPtdata[l_u16index].u16Pt1;
//		l_u16EndYpt = l_FrameInfo.IncPtdata[l_u16index].u16Pt2;
//		if (l_u16tmp<=500 || l_u16EndYpt-l_u16StartYpt+1<=3)
//		{
//			if (l_FrameInfo.u8Sum>=2)
//			{
//				if (abs(l_FrameInfo.IncPtdata[l_FrameInfo.u8Sum-2].u16Pt2-l_u16StartYpt)<=5 
//					&& abs(l_FrameInfo.IncPtdata[l_FrameInfo.u8Sum-2].n32y2-l_FrameInfo.IncPtdata[l_u16index].n32y1)<500)
//				{					
//					l_FrameInfo.IncPtdata[l_FrameInfo.u8Sum-2].u16Pt2 = l_FrameInfo.IncPtdata[l_u16index].u16Pt2;
//					l_FrameInfo.IncPtdata[l_FrameInfo.u8Sum-2].n32y2 = l_FrameInfo.IncPtdata[l_u16index].n32y2;
//					l_FrameInfo.IncPtdata[l_FrameInfo.u8Sum-2].u16yDis = abs(l_FrameInfo.IncPtdata[l_FrameInfo.u8Sum-2].n32y2-l_FrameInfo.IncPtdata[l_FrameInfo.u8Sum-2].n32y1);											
//					l_32tmp = l_FrameInfo.IncPtdata[l_FrameInfo.u8Sum-2].u16yMaxHt;	
//					if(l_FrameInfo.IncPtdata[l_u16index].u16yMaxHt > l_32tmp)
//					{
//						l_FrameInfo.IncPtdata[l_FrameInfo.u8Sum-2].u16yMaxHt = l_FrameInfo.IncPtdata[l_u16index].u16yMaxHt;  //ȡ���ֵΪ��				
//					}
//					l_FrameInfo.uValid[l_u16index] = 0;
//					memset(&l_FrameInfo.IncPtdata[l_u16index], 0, sizeof(IncPtSt));
//					l_FrameInfo.u8Sum--;
//				}
//				else if (l_u16EndYpt-l_u16StartYpt+1<=3 && l_u16tmp<=500 && l_FrameInfo.IncPtdata[l_u16index].n32y1-l_FrameInfo.IncPtdata[l_FrameInfo.u8Sum-2].n32y2<500)
//				{
//					l_FrameInfo.IncPtdata[l_FrameInfo.u8Sum-2].u16Pt2 = l_FrameInfo.IncPtdata[l_u16index].u16Pt2;
//					l_FrameInfo.IncPtdata[l_FrameInfo.u8Sum-2].n32y2 = l_FrameInfo.IncPtdata[l_u16index].n32y2;
//					l_FrameInfo.IncPtdata[l_FrameInfo.u8Sum-2].u16yDis = abs(l_FrameInfo.IncPtdata[l_FrameInfo.u8Sum-2].n32y2-l_FrameInfo.IncPtdata[l_FrameInfo.u8Sum-2].n32y1);											
//					l_32tmp = l_FrameInfo.IncPtdata[l_FrameInfo.u8Sum-2].u16yMaxHt;	
//					if(l_FrameInfo.IncPtdata[l_u16index].u16yMaxHt > l_32tmp)
//					{
//						l_FrameInfo.IncPtdata[l_FrameInfo.u8Sum-2].u16yMaxHt = l_FrameInfo.IncPtdata[l_u16index].u16yMaxHt;  //ȡ���ֵΪ��				
//					}
//					l_FrameInfo.uValid[l_u16index] = 0;
//					memset(&l_FrameInfo.IncPtdata[l_u16index], 0, sizeof(IncPtSt));
//					l_FrameInfo.u8Sum--;
//				}
//				else
//				{
//					l_FrameInfo.uValid[l_u16index] = 0;
//					memset(&l_FrameInfo.IncPtdata[l_u16index], 0, sizeof(IncPtSt));
//					l_FrameInfo.u8Sum--;					
//				}
//			}
//			else if(l_FrameInfo.IncPtdata[l_u16index].u16yMaxHt<1000)
//			{
//				l_FrameInfo.uValid[l_u16index] = 0;
//				memset(&l_FrameInfo.IncPtdata[l_u16index], 0, sizeof(IncPtSt));
//				l_FrameInfo.u8Sum--;				
//			}			
//		}			
//	}
//
//
//	//����ϲ�
//	for(l_u16index = 0;l_u16index < l_FrameInfo.u8Sum-1;l_u16index=l_u16index)
//	{
//		l_u16StartYpt = l_FrameInfo.IncPtdata[l_u16index].u16Pt1;
//		l_u16EndYpt = l_FrameInfo.IncPtdata[l_u16index].u16Pt2;
//		l_n32StartY = l_FrameInfo.IncPtdata[l_u16index].n32y1;
//		l_n32EndY = l_FrameInfo.IncPtdata[l_u16index].n32y2;
//		l_u16tmp = abs(l_FrameInfo.IncPtdata[l_u16index+1].n32y1-l_FrameInfo.IncPtdata[l_u16index+1].n32y2);
//		if (abs(l_n32EndY-l_FrameInfo.IncPtdata[l_u16index+1].n32y1)<2000  
//			&& abs(l_u16EndYpt-l_FrameInfo.IncPtdata[l_u16index+1].u16Pt1)<=5
//			&& abs(l_n32StartY-l_n32EndY)<l_u16tmp && abs(l_n32StartY-l_n32EndY)<2000 && l_u16tmp>2*abs(l_n32StartY-l_n32EndY)
//			&& (abs(l_u16StartYpt-l_u16EndYpt+1)<10 ))
//		{
//			l_FrameInfo.IncPtdata[l_u16index].u16Pt2 = l_FrameInfo.IncPtdata[l_u16index+1].u16Pt2;
//			l_FrameInfo.IncPtdata[l_u16index].n32y2 = l_FrameInfo.IncPtdata[l_u16index+1].n32y2;
//			l_FrameInfo.IncPtdata[l_u16index].u16yDis = abs(l_FrameInfo.IncPtdata[l_u16index].n32y2-l_FrameInfo.IncPtdata[l_u16index].n32y1);											
//			l_32tmp = l_FrameInfo.IncPtdata[l_u16index].u16yMaxHt;	
//			if(l_FrameInfo.IncPtdata[l_u16index+1].u16yMaxHt > l_32tmp)
//			{
//				l_FrameInfo.IncPtdata[l_u16index].u16yMaxHt = l_FrameInfo.IncPtdata[l_u16index+1].u16yMaxHt;  //ȡ���ֵΪ��				
//			}
//			//l_FrameInfo.uValid[l_u16index+1] = 0;
//			//memset(&l_FrameInfo.IncPtdata[l_u16index+1], 0, sizeof(IncPtSt));
//
//			for (l_index=l_u16index+1;l_index<l_FrameInfo.u8Sum-1;l_index++)
//			{
//				memcpy(&l_FrameInfo.IncPtdata[l_index],&l_FrameInfo.IncPtdata[l_index+1],sizeof(IncPtSt));
//			}
//			l_FrameInfo.uValid[l_index] = 0;
//			memset(&l_FrameInfo.IncPtdata[l_index], 0, sizeof(IncPtSt));
//			l_FrameInfo.u8Sum--;
//		}
//		else if (abs(l_n32EndY-l_FrameInfo.IncPtdata[l_u16index+1].n32y1)<2000 
//			&& abs(l_u16EndYpt-l_FrameInfo.IncPtdata[l_u16index+1].u16Pt1)<=5 
//			&& abs(l_n32StartY-l_n32EndY)>l_u16tmp && l_u16tmp<2000 && l_u16tmp*2<abs(l_n32StartY-l_n32EndY)
//			&& abs(l_FrameInfo.IncPtdata[l_u16index+1].u16Pt1-l_FrameInfo.IncPtdata[l_u16index+1].u16Pt2+1)<10)
//		{
//			l_FrameInfo.IncPtdata[l_u16index].u16Pt2 = l_FrameInfo.IncPtdata[l_u16index+1].u16Pt2;
//			l_FrameInfo.IncPtdata[l_u16index].n32y2 = l_FrameInfo.IncPtdata[l_u16index+1].n32y2;
//			l_FrameInfo.IncPtdata[l_u16index].u16yDis = abs(l_FrameInfo.IncPtdata[l_u16index].n32y2-l_FrameInfo.IncPtdata[l_u16index].n32y1);											
//			l_32tmp = l_FrameInfo.IncPtdata[l_u16index].u16yMaxHt;	
//			if(l_FrameInfo.IncPtdata[l_u16index+1].u16yMaxHt > l_32tmp)
//			{
//				l_FrameInfo.IncPtdata[l_u16index].u16yMaxHt = l_FrameInfo.IncPtdata[l_u16index+1].u16yMaxHt;  //ȡ���ֵΪ��				
//			}
//			//l_FrameInfo.uValid[l_u16index+1] = 0;
//			//memset(&l_FrameInfo.IncPtdata[l_u16index+1], 0, sizeof(IncPtSt));
//			for (l_index=l_u16index+1;l_index<l_FrameInfo.u8Sum-1;l_index++)
//			{
//				memcpy(&l_FrameInfo.IncPtdata[l_index],&l_FrameInfo.IncPtdata[l_index+1],sizeof(IncPtSt));
//			}
//			l_FrameInfo.uValid[l_index] = 0;
//			memset(&l_FrameInfo.IncPtdata[l_index], 0, sizeof(IncPtSt));
//			l_FrameInfo.u8Sum--;		
//		}
//		else if (abs(l_n32EndY-l_FrameInfo.IncPtdata[l_u16index+1].n32y1)<1000
//			&& abs(l_u16EndYpt-l_FrameInfo.IncPtdata[l_u16index+1].u16Pt1)<=10
//			&& abs(l_n32StartY-l_n32EndY)>l_u16tmp && l_u16tmp<1000)
//		{
//			l_FrameInfo.IncPtdata[l_u16index].u16Pt2 = l_FrameInfo.IncPtdata[l_u16index+1].u16Pt2;
//			l_FrameInfo.IncPtdata[l_u16index].n32y2 = l_FrameInfo.IncPtdata[l_u16index+1].n32y2;
//			l_FrameInfo.IncPtdata[l_u16index].u16yDis = abs(l_FrameInfo.IncPtdata[l_u16index].n32y2-l_FrameInfo.IncPtdata[l_u16index].n32y1);											
//			l_32tmp = l_FrameInfo.IncPtdata[l_u16index].u16yMaxHt;	
//			if(l_FrameInfo.IncPtdata[l_u16index+1].u16yMaxHt > l_32tmp)
//			{
//				l_FrameInfo.IncPtdata[l_u16index].u16yMaxHt = l_FrameInfo.IncPtdata[l_u16index+1].u16yMaxHt;  //ȡ���ֵΪ��				
//			}
//			//l_FrameInfo.uValid[l_u16index+1] = 0;
//			//memset(&l_FrameInfo.IncPtdata[l_u16index+1], 0, sizeof(IncPtSt));
//			//l_FrameInfo.u8Sum--;
//			for (l_index=l_u16index+1;l_index<l_FrameInfo.u8Sum-1;l_index++)
//			{
//				memcpy(&l_FrameInfo.IncPtdata[l_index],&l_FrameInfo.IncPtdata[l_index+1],sizeof(IncPtSt));
//			}
//			l_FrameInfo.uValid[l_index] = 0;
//			memset(&l_FrameInfo.IncPtdata[l_index], 0, sizeof(IncPtSt));
//			l_FrameInfo.u8Sum--;
//		}
//		else if (abs(l_n32EndY-l_FrameInfo.IncPtdata[l_u16index+1].n32y1)<1000
//			&& abs(l_u16EndYpt-l_FrameInfo.IncPtdata[l_u16index+1].u16Pt1)<=10
//			&& abs(g_ZdistanceI[l_u16EndYpt]-g_ZdistanceI[l_FrameInfo.IncPtdata[l_u16index+1].u16Pt1])<1000
//			&& l_FrameInfo.IncPtdata[l_u16index].u16yMaxHt>1200 && l_FrameInfo.IncPtdata[l_u16index+1].u16yMaxHt>1200)
//		{
//			l_FrameInfo.IncPtdata[l_u16index].u16Pt2 = l_FrameInfo.IncPtdata[l_u16index+1].u16Pt2;
//			l_FrameInfo.IncPtdata[l_u16index].n32y2 = l_FrameInfo.IncPtdata[l_u16index+1].n32y2;
//			l_FrameInfo.IncPtdata[l_u16index].u16yDis = abs(l_FrameInfo.IncPtdata[l_u16index].n32y2-l_FrameInfo.IncPtdata[l_u16index].n32y1);											
//			l_32tmp = l_FrameInfo.IncPtdata[l_u16index].u16yMaxHt;	
//			if(l_FrameInfo.IncPtdata[l_u16index+1].u16yMaxHt > l_32tmp)
//			{
//				l_FrameInfo.IncPtdata[l_u16index].u16yMaxHt = l_FrameInfo.IncPtdata[l_u16index+1].u16yMaxHt;  //ȡ���ֵΪ��				
//			}
//			//l_FrameInfo.uValid[l_u16index+1] = 0;
//			//memset(&l_FrameInfo.IncPtdata[l_u16index+1], 0, sizeof(IncPtSt));
//			//l_FrameInfo.u8Sum--;
//			for (l_index=l_u16index+1;l_index<l_FrameInfo.u8Sum-1;l_index++)
//			{
//				memcpy(&l_FrameInfo.IncPtdata[l_index],&l_FrameInfo.IncPtdata[l_index+1],sizeof(IncPtSt));
//			}
//			l_FrameInfo.uValid[l_index] = 0;
//			memset(&l_FrameInfo.IncPtdata[l_index], 0, sizeof(IncPtSt));
//			l_FrameInfo.u8Sum--;
//		}
//		else
//		{
//			l_u16index++;
//		}
//	}
//
///*******���ÿһ�����г�ͷ����β�Ƿ���*********/
//	for(l_index = 0;l_index < l_FrameInfo.u8Sum;l_index++)
//	{
//		l_FrameInfo.IncPtdata[l_index].u8DaFeiFlag1 = 0;
//		l_FrameInfo.IncPtdata[l_index].u8DaFeiFlag2 = 0;
//		for(i=1;i<=DafeiData[0][0]+DafeiData[0][1];i++)//20140919
//		{
//			if((DafeiData[i][0]<=l_FrameInfo.IncPtdata[l_index].u16Pt2)&&(DafeiData[i][1]>=l_FrameInfo.IncPtdata[l_index].u16Pt2)) //����ɶε���ֹ���Ƿ��복����ʼ����ͬ������ͬ��ͷ���
//			{
//				l_FrameInfo.IncPtdata[l_index].u8DaFeiFlag1 = 1;
//			}
//			if((DafeiData[i][0]<=l_FrameInfo.IncPtdata[l_index].u16Pt1)&&(DafeiData[i][1]>=l_FrameInfo.IncPtdata[l_index].u16Pt1))		   //����ɶε���ʼ���Ƿ��복�Ľ�β����ͬ������ͬ��β���
//			{
//				l_FrameInfo.IncPtdata[l_index].u8DaFeiFlag2 = 1;
//			}
//		}	
//	}
//JG_T0TC[2] = T0TC;
//JG_counter2[2]=t0_count2;	
//���㳵�������ߡ���ͷʱ�䡢��ͷ��βX���ꡢ��ͷ�㳵β��
//	for(l_index = 0;l_index < l_FrameInfo.u8Sum;l_index++)
//	{
//		l_u16StartYpt = l_FrameInfo.IncPtdata[l_index].u16Pt1;
//		l_u16EndYpt = l_FrameInfo.IncPtdata[l_index].u16Pt2;
//		if(((abs(l_FrameInfo.IncPtdata[l_index].n32y2- l_FrameInfo.IncPtdata[l_index].n32y1)>500) && (l_u16EndYpt - l_u16StartYpt + 1)>=5 && l_FrameInfo.IncPtdata[l_index].n32y2 >=EnterX1 && l_FrameInfo.IncPtdata[l_index].n32y1<=ExitX4+2000)
//			|| (l_FrameInfo.IncPtdata[l_index].n32y1>=ExitX4 && l_FrameInfo.IncPtdata[l_index].n32y2>=EnterX1 && (l_u16EndYpt - l_u16StartYpt + 1)>=3 && l_FrameInfo.IncPtdata[l_index].u16yMaxHt>1000 && l_FrameInfo.IncPtdata[l_index].n32y1<=ExitX4+2000))//20140904
//		{
//			Tmp_Z=0;
//			Tmp_Z = GetVehHeight2(g_ZdistanceI, l_u16StartYpt, l_u16EndYpt);
//
//			
//			l_FrameInfo.IncPtdata[l_index].u16yMaxHt = Tmp_Z;	    //����
//			
//			//*�ó�ͷ��ǰ��ĵ���Ϊ��ͷλ��*/	
//			MaxX=l_FrameInfo.IncPtdata[l_index].n32y2;
//			Maxi=l_FrameInfo.IncPtdata[l_index].u16Pt2;
//			for(i=l_u16StartYpt;i<=l_u16EndYpt;i++)
//			{
//				if(g_YdistanceI[i]>MaxX && g_ZdistanceI[i]>=ThresVehLow)	//20140327 ZҪ������ֵ
//				{
//					MaxX=g_YdistanceI[i];
//					Maxi=i;
//				}
//			}
//			l_FrameInfo.IncPtdata[l_index].n32y2 = MaxX;
//			l_FrameInfo.IncPtdata[l_index].u16Pt2 = Maxi;
//			//*Ѱ�ҳ�ͷ��ǰ�����*/
//
//
//			//*�ó�β��ǰ��ĵ���Ϊ��βλ��*/	
//			MinX=l_FrameInfo.IncPtdata[l_index].n32y1;
//			Maxi=l_FrameInfo.IncPtdata[l_index].u16Pt1;
//			for(i=l_u16StartYpt;i<=l_u16EndYpt;i++)//20140916
//			{
//				if(g_YdistanceI[i]<MinX && g_ZdistanceI[i]>=ThresVehLow)
//				{
//					MinX=g_YdistanceI[i];
//					Maxi=i;
//				}
//			}
//			l_FrameInfo.IncPtdata[l_index].n32y1 = MinX;
//			l_FrameInfo.IncPtdata[l_index].u16Pt1 = Maxi;
//			//*Ѱ�ҳ�β��ǰ�����*/		
//
//
//						
//			if(l_FrameInfo.IncPtdata[l_index].n32y1<0&&l_FrameInfo.IncPtdata[l_index].n32y2>0)		 //ֻ�ǵ���ͷλ����X�����򣬳�βλ����X�Ḻ��ʱ�ż��㳵���ĳ���
//			{
//				l_FrameInfo.IncPtdata[l_index].u16yDis=l_FrameInfo.IncPtdata[l_index].n32y2-l_FrameInfo.IncPtdata[l_index].n32y1;  //����
//			}
//			else 
//			{
//				l_FrameInfo.IncPtdata[l_index].u16yDis = 0;
//			}		
///*��ͷʱ�������ں������*/								
//		}
//		else 
//		{
//			l_FrameInfo.uValid[l_index] = 0;		   //�޳�δ����������ĳ�
//			memset(&l_FrameInfo.IncPtdata, 0, sizeof(IncPtSt));
//			for (i=l_index; i<l_FrameInfo.u8Sum-1; i++)
//			{
//				memcpy(&l_FrameInfo.IncPtdata[i], &l_FrameInfo.IncPtdata[i+1], sizeof(l_FrameInfo.IncPtdata[i+1]));
//			}
//			l_FrameInfo.u8Sum = l_FrameInfo.u8Sum - 1;
//		}				 	
//	}		
//
//	 														  
//����ƥ��
//	for(l_u16index = 0; l_u16index < l_FrameInfo.u8Sum; l_u16index++)
//	{
//		l_32tmp = 0;  //ƥ���ʶ���ɹ���1��
//		l_n32StartY = l_FrameInfo.IncPtdata[l_u16index].n32y1;
//		l_n32EndY = l_FrameInfo.IncPtdata[l_u16index].n32y2;
//		l_u16StartYpt = l_FrameInfo.IncPtdata[l_u16index].u16Pt1;
//		l_u16EndYpt = l_FrameInfo.IncPtdata[l_u16index].u16Pt2;
//		if (l_u16EndYpt <= l_u16StartYpt ||	l_n32EndY <= l_n32StartY)
//		{
//			continue;
//		}
//		
//		if (l_n32EndY>=EnterX1 && l_n32StartY<=0)			//��ͷ����EnterX1&&��βδ�����·�
//		{
//			for(j = 0;j < g_VehIncTotal;j++)//�����е�˳ɨ�������д洢�ĳ������ݽ���ƥ��
//			{
//				i = g_VehIncSetIndex[j] -1;		
//				if(g_VehIncSet[i].u8Istate == OCCURING_USED)
//				{
//					l_32tmp2 = g_VehIncSet[i].Idata.u16FrameCnt &  FRAME_MASK;
//					if (IsInIncSide(l_n32StartY, l_n32EndY, g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][1],g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][0])
//						|| (l_n32EndY<0 && abs(l_n32EndY-g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][0])<1500 && abs(l_u16StartYpt-g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][3])<=10 && g_ZdistanceI[l_u16StartYpt]+1000<l_FrameInfo.IncPtdata[l_u16index].u16yMaxHt)
//						|| (l_n32EndY>0 && abs(l_n32StartY-g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][1])<1500 && g_ZdistanceI[l_u16EndYpt]+1000<l_FrameInfo.IncPtdata[l_u16index].u16yMaxHt))
//					{
//						 l_32tmp = 1;  //����ƥ��ɹ�
//						 if ((l_n32StartY>=0 && abs(l_n32StartY-g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][1])>50)//��β
//						 	|| (l_n32EndY<=0 && abs(l_n32EndY-g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][0])>50)
//							|| (l_n32StartY<0 && abs(l_n32StartY-g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][1])>50 && l_n32EndY>ExitX4))	  // && abs(l_n32EndY-g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][0])>50
//						 {
//						 	g_VehIncSet[i].u8Istate = OCCURING_USED;
//
//							
//							memcpy(&g_VehIncSet[i].Idata.zdata[l_32tmp2][1], g_ZdistanceI + l_u16StartYpt, sizeof(int32)*(l_u16EndYpt - l_u16StartYpt + 1));
//							g_VehIncSet[i].Idata.zdata[l_32tmp2][0] = l_u16EndYpt - l_u16StartYpt + 1;
//							//ydata
//							memcpy(&g_VehIncSet[i].Idata.ydata[l_32tmp2][1], g_YdistanceI + l_u16StartYpt, sizeof(int32)*(l_u16EndYpt - l_u16StartYpt + 1));
//							g_VehIncSet[i].Idata.ydata[l_32tmp2][0] = l_u16EndYpt - l_u16StartYpt + 1;
//
//							g_VehIncSet[i].Idata.yMax[l_32tmp2] = abs(l_n32EndY - l_n32StartY);  								
//							g_VehIncSet[i].Idata.zMax[l_32tmp2] = l_FrameInfo.IncPtdata[l_u16index].u16yMaxHt;
//							g_VehIncSet[i].Idata.ydataInfo[l_32tmp2][0] = l_FrameInfo.IncPtdata[l_u16index].n32y2;
//							g_VehIncSet[i].Idata.ydataInfo[l_32tmp2][1] = l_FrameInfo.IncPtdata[l_u16index].n32y1;
//							g_VehIncSet[i].Idata.ydataInfo[l_32tmp2][2] = l_FrameInfo.IncPtdata[l_u16index].u16Pt2;
//							g_VehIncSet[i].Idata.ydataInfo[l_32tmp2][3] = l_FrameInfo.IncPtdata[l_u16index].u16Pt1;
//							
//							g_VehIncSet[i].Idata.ydataInfo[l_32tmp2][4]	= l_FrameInfo.IncPtdata[l_u16index].u8DaFeiFlag1;
//							g_VehIncSet[i].Idata.ydataInfo[l_32tmp2][5] = l_FrameInfo.IncPtdata[l_u16index].u8DaFeiFlag2; 							 																				
//							g_VehIncSet[i].Idata.tdata[l_32tmp2] = JG_time 
//								+ (g_VehIncSet[i].Idata.ydataInfo[l_32tmp2][2]+g_u16InclineStartAnglePt)*10000/360;
//						 
//							if (g_VehIncSet[i].Idata.u16FrameCnt < FRAME_MASK )
//							{
//								g_VehIncSet[i].Idata.u16FrameCnt++;	
//							}
//							else
//							{	
//								g_VehIncSet[i].Idata.u16FrameCnt = FRAME_MASK;
//							}						 						 
//						}
//						g_VehIncSet[i].IemptFrame = 0;
//						//�Գ���2000���ϣ�����6000���ϵ�����������֡���д�����Ҫ��Գ�ͷ
//						if ((g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][0]>0)
//							&& (g_VehIncSet[i].Idata.zMax[l_32tmp2-1]>2000) 
//							&& (abs(g_VehIncSet[i].Idata.zMax[l_32tmp2]-g_VehIncSet[i].Idata.zMax[l_32tmp2-1])<1000)
//							&& (abs(g_VehIncSet[i].Idata.ydataInfo[l_32tmp2][0]-g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][1])>6000)
//							&& (abs(g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][0]-g_VehIncSet[i].Idata.ydataInfo[l_32tmp2][0])>1500)
//							&& (abs(g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][0]-g_VehIncSet[i].Idata.ydataInfo[l_32tmp2][0])>abs(g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][1]-g_VehIncSet[i].Idata.ydataInfo[l_32tmp2][1])+1500))
//						{
//							g_VehIncSet[i].Idata.ydataInfo[l_32tmp2][0] = g_VehIncSet[i].Idata.ydataInfo[l_32tmp2][1] + (g_VehIncSet[i].Idata.yMax[l_32tmp2-1]+g_VehIncSet[i].Idata.yMax[l_32tmp2-2])/2;
//							g_VehIncSet[i].Idata.yMax[l_32tmp2] = abs(g_VehIncSet[i].Idata.ydataInfo[l_32tmp2][0]-g_VehIncSet[i].Idata.ydataInfo[l_32tmp2][1]);
//						}
//					}
//					else if (l_32tmp2>2 && IsInIncSide(l_n32StartY, l_n32EndY, g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-2][1],g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-2][0]))
//					{
//						l_32tmp = 1;  //����ƥ��ɹ�
//						if ((l_n32StartY>=0 && abs(l_n32StartY-g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-2][1])>50)//��β
//							|| (l_n32EndY<=0 && abs(l_n32EndY-g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-2][0])>50)
//							|| (l_n32StartY<0 && abs(l_n32StartY-g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-2][1])>50 && l_n32EndY>ExitX4))	  // && abs(l_n32EndY-g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][0])>50
//						{
//							g_VehIncSet[i].u8Istate = OCCURING_USED;
//
//							memcpy(&g_VehIncSet[i].Idata.zdata[l_32tmp2][1], g_ZdistanceI + l_u16StartYpt, sizeof(int32)*(l_u16EndYpt - l_u16StartYpt + 1));
//							g_VehIncSet[i].Idata.zdata[l_32tmp2][0] = l_u16EndYpt - l_u16StartYpt + 1;
//							//ydata
//							memcpy(&g_VehIncSet[i].Idata.ydata[l_32tmp2][1], g_YdistanceI + l_u16StartYpt, sizeof(int32)*(l_u16EndYpt - l_u16StartYpt + 1));
//							g_VehIncSet[i].Idata.ydata[l_32tmp2][0] = l_u16EndYpt - l_u16StartYpt + 1;
//
//							g_VehIncSet[i].Idata.yMax[l_32tmp2] = abs(l_n32EndY - l_n32StartY);  								
//							g_VehIncSet[i].Idata.zMax[l_32tmp2] = l_FrameInfo.IncPtdata[l_u16index].u16yMaxHt;
//							g_VehIncSet[i].Idata.ydataInfo[l_32tmp2][0] = l_FrameInfo.IncPtdata[l_u16index].n32y2;
//							g_VehIncSet[i].Idata.ydataInfo[l_32tmp2][1] = l_FrameInfo.IncPtdata[l_u16index].n32y1;
//							g_VehIncSet[i].Idata.ydataInfo[l_32tmp2][2] = l_FrameInfo.IncPtdata[l_u16index].u16Pt2;
//							g_VehIncSet[i].Idata.ydataInfo[l_32tmp2][3] = l_FrameInfo.IncPtdata[l_u16index].u16Pt1;
//
//							g_VehIncSet[i].Idata.ydataInfo[l_32tmp2][4]	= l_FrameInfo.IncPtdata[l_u16index].u8DaFeiFlag1;
//							g_VehIncSet[i].Idata.ydataInfo[l_32tmp2][5] = l_FrameInfo.IncPtdata[l_u16index].u8DaFeiFlag2; 							 																				
//							g_VehIncSet[i].Idata.tdata[l_32tmp2] = JG_time 
//								+ (g_VehIncSet[i].Idata.ydataInfo[l_32tmp2][2]+g_u16InclineStartAnglePt)*10000/360;
//
//							if (g_VehIncSet[i].Idata.u16FrameCnt < FRAME_MASK )
//							{
//								g_VehIncSet[i].Idata.u16FrameCnt++;	
//							}
//							else
//							{	
//								g_VehIncSet[i].Idata.u16FrameCnt = FRAME_MASK;
//							}						 						 
//						}
//						g_VehIncSet[i].IemptFrame = 0;
//
//						if (g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][1]<g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-2][1]-2500 && l_n32StartY-g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-2][1]-2500)//���쳣�洢��֡��������20140914
//						{
//							g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][1] = (g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-2][1]+l_n32StartY)/2;
//							g_VehIncSet[i].Idata.yMax[l_32tmp2-1] = abs(g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-2][0] - g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-2][1]);
//						}
//					}
//					break;
//				}										
//			
//			}
//			if (0 == l_32tmp && l_FrameInfo.IncPtdata[l_u16index].n32y1<0)	 //�³�����
//			{
//				memcpy(&l_u16IncPosVect[0],&l_FrameInfo.IncPtdata[l_u16index],sizeof(IncPtSt)); 				
//				l_32tmp = 1;				
//				for(l_u16tmp = 0;l_u16tmp<l_32tmp;l_u16tmp++)
//				{
//					//������δƥ��ɹ�����Ϊ���³�
//					for(i = 0;i < VEHICLE_MAX;i++)
//					{
//						if(g_VehIncSet[i].u8Istate == NO_USED)
//						{
//							g_VehIncSetIndex[g_VehIncTotal++] = i+1;
//	//							if(g_VehIncTotal >= 10)
//	//								clearVehicleErr();
//							g_VehIncSet[i].u8Istate = OCCURING_USED;	
//																			  
//							l_u16StartYpt = l_u16IncPosVect[l_u16tmp].u16Pt1;
//							l_u16EndYpt = l_u16IncPosVect[l_u16tmp].u16Pt2;	
//							l_n32StartY = l_u16IncPosVect[l_u16tmp].n32y1;
//							l_n32EndY = l_u16IncPosVect[l_u16tmp].n32y2;	
//							
//							l_32tmp2 =g_VehIncSet[i].Idata.u16FrameCnt &  FRAME_MASK;
//							if(g_VehIncSet[i].Idata.u16FrameCnt < FRAME_MAXCNT && l_u16EndYpt>l_u16StartYpt)
//							{
//								memcpy(&g_VehIncSet[i].Idata.zdata[l_32tmp2][1],g_ZdistanceI + l_u16StartYpt,sizeof(int32)*(l_u16EndYpt - l_u16StartYpt + 1));
//								g_VehIncSet[i].Idata.zdata[l_32tmp2][0] = l_u16EndYpt - l_u16StartYpt + 1;
//								//ydata
//								memcpy(&g_VehIncSet[i].Idata.ydata[l_32tmp2][1], g_YdistanceI + l_u16StartYpt,sizeof(int32)*(l_u16EndYpt - l_u16StartYpt + 1));
//								g_VehIncSet[i].Idata.ydata[l_32tmp2][0] = l_u16EndYpt - l_u16StartYpt + 1;
//	
//								g_VehIncSet[i].Idata.yMax[l_32tmp2] = abs(l_n32EndY - l_n32StartY);  								
//								g_VehIncSet[i].Idata.zMax[l_32tmp2] = l_u16IncPosVect[l_u16tmp].u16yMaxHt;
//								g_VehIncSet[i].Idata.ydataInfo[l_32tmp2][0] = l_u16IncPosVect[l_u16tmp].n32y2;
//								g_VehIncSet[i].Idata.ydataInfo[l_32tmp2][1] = l_u16IncPosVect[l_u16tmp].n32y1;
//								g_VehIncSet[i].Idata.ydataInfo[l_32tmp2][2] = l_u16IncPosVect[l_u16tmp].u16Pt2;
//								g_VehIncSet[i].Idata.ydataInfo[l_32tmp2][3] = l_u16IncPosVect[l_u16tmp].u16Pt1;
//								
//								g_VehIncSet[i].Idata.ydataInfo[l_32tmp2][4]	= l_u16IncPosVect[l_u16tmp].u8DaFeiFlag1;
//								g_VehIncSet[i].Idata.ydataInfo[l_32tmp2][5] = l_u16IncPosVect[l_u16tmp].u8DaFeiFlag2; 							 																				
//							}
//							g_VehIncSet[i].Idata.tdata[l_32tmp2] = JG_time + (g_VehIncSet[i].Idata.ydataInfo[l_32tmp2][2]+g_u16InclineStartAnglePt)*10000/360;
//							if (g_VehIncSet[i].Idata.u16FrameCnt < FRAME_MASK )
//							{
//								g_VehIncSet[i].Idata.u16FrameCnt++;	
//							}
//							else
//							{	
//								g_VehIncSet[i].Idata.u16FrameCnt = FRAME_MASK;
//							}	
//							g_VehIncSet[i].IemptFrame = 0;	
//							break;
//						}
//					}							
//				}			
//			}		
//		}					
//	}
//
//�׳����㳵�ټ���֡����
//	for(j = 0;j < g_VehIncTotal;j++)
//	{
//		i = (g_VehIncSetIndex[j] - 1) & VEHICLE_MASK;	//20140217 	��g_VehicleSetIndex[j] - 1�޸�Ϊ(g_VehicleSetIndex[j] - 1) & VEHICLE_MASK
//		if (g_VehIncSet[i].u8Istate != NO_USED)
//		{
//			l_32tmp2 = g_VehIncSet[i].Idata.u16FrameCnt & FRAME_MASK;
//			if( l_32tmp2 >0
//				&& (g_VehIncSet[i].IemptFrame > NORMAL_MAX_EMPTYFRAME 
//				&&(g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][1]>ExitX4)))
//			{ 		
//			   g_VehIncSet[i].u8Istate = PASSED_USED;  //�ѽ���������β�ĳ�
//			}
//			else if (l_32tmp2 >0 && (0 == g_VehIncSet[i].u8ThrowFlag) 
//				&& g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][0]>ExitX3)   //��ͷ����
//			{
//			   g_VehIncSet[i].speed = GetVehSpeed(&g_VehIncSet[i], EnterX2, EnterX1);	//��ͷ�ٶ�
//			   g_VehIncSet[i].u8ThrowFlag = 1;
//			}
//			else if (l_32tmp2 >0 && g_VehIncSet[i].IemptFrame > ERR_MAX_EMPTYFRAME)
//			{
//				memset(&g_VehIncSet[i], 0,sizeof(VehIncSt)); //���˳���������ü�¼
//				g_VehIncSet[i].u8Istate = NO_USED; 
//				for(l_u16tmp = j;l_u16tmp < g_VehIncTotal - 1;l_u16tmp++)
//					g_VehIncSetIndex[l_u16tmp] = g_VehIncSetIndex[l_u16tmp+1];
//					
//				if (g_VehIncTotal <= VEHICLE_MAX)  //20140217 �޸�
//				{
//					g_VehIncSetIndex[g_VehIncTotal - 1] = 0;
//					g_VehIncTotal--;
//				}
//				else
//				{
//					g_VehIncTotal = 0;
//				}
//				continue;
//			}
//			if (l_32tmp2==1 && g_VehIncSet[i].IemptFrame>10)//20140918
//			{
//				memset(&g_VehIncSet[i], 0,sizeof(VehIncSt)); //���˳���������ü�¼
//				g_VehIncSet[i].u8Istate = NO_USED; 
//				for(l_u16tmp = j;l_u16tmp < g_VehIncTotal - 1;l_u16tmp++)
//					g_VehIncSetIndex[l_u16tmp] = g_VehIncSetIndex[l_u16tmp+1];
//
//				if (g_VehIncTotal <= VEHICLE_MAX)  
//				{
//					g_VehIncSetIndex[g_VehIncTotal - 1] = 0;
//					g_VehIncTotal--;
//				}
//				else
//				{
//					g_VehIncTotal = 0;
//				}
//				continue;
//			}		
//			g_VehIncSet[i].IemptFrame++;		
//		}					
//	}
//
//�������ȼ���
////////��ͷ����Lengthline�ߺ��ʱ��ʼ/////////
//	for (j = 0;j < g_VehIncTotal;j++)
//	{
//		i = (g_VehIncSetIndex[j] - 1) & VEHICLE_MASK;
//		if (g_VehIncSet[i].u8Istate != NO_USED)
//		{
//		   l_32tmp2 = g_VehIncSet[i].Idata.u16FrameCnt &  FRAME_MASK;
//		   if((0 == g_VehIncSet[i].u8LineFlag1)  
//				&& g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][0]>Lengthline1 
//				&& (g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][1]<Lengthline1)
//				&& g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][0]<Lengthline2)
//		    {
//				g_VehIncSet[i].nStartTime =	g_VehIncSet[i].Idata.tdata[l_32tmp2-1];      //��ʼʱ��Ϊ��ͷʱ�䣬�Ѿ���������ʱ��
//				g_VehIncSet[i].ndeltaY= g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][0];	
//				g_VehIncSet[i].nEndTime = g_VehIncSet[i].nStartTime;
//				g_VehIncSet[i].u8LineFlag1 = 1; 
//			}
//			else if (1 == g_VehIncSet[i].u8LineFlag1)
//			{
//				if (0 == g_VehIncSet[i].u8LineFlag2)
//				{
//					if(g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][0]>=Lengthline2 
//						&& g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][1]>=Lengthline2)
//					{
//						g_VehIncSet[i].u8LineFlag2 = 1;
//					}				
//				}
//				else if (1 == g_VehIncSet[i].u8LineFlag2)
//				{						
//					g_VehIncSet[i].nEndTime = g_VehIncSet[i].Idata.tdata[l_32tmp2-1] 
//						- (g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][2]-g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][3])*10000/360; //����ʱ��
//					
//					g_VehIncSet[i].yLen= g_VehIncSet[i].speed*(g_VehIncSet[i].nEndTime-g_VehIncSet[i].nStartTime)/36000 
//						- abs(g_VehIncSet[i].ndeltaY-g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][1]);  //��������
//					g_VehIncSet[i].ndeltaY = 0;
//					g_VehIncSet[i].nStartTime = 0;
//					g_VehIncSet[i].nEndTime = 0;
//					g_VehIncSet[i].u8LineFlag1 = 2;
//					g_VehIncSet[i].u8LineFlag2 = 2;									
//				}			
//			}		
//		}		
//	}
//
//
//
//
//
//   k=0;      
//for (i=g_u16InclineStartAnglePt;i<=g_u16InclineEndAnglePt;i++)
//for (i=0;i<g_u16InclineEndAnglePt-g_u16InclineStartAnglePt;i++)
//{
//	if((g_ZdistanceI[i] >= ThresVehLow)&&(g_ZdistanceI[i] <= ThresVehHigh) 
//			&& abs(g_YdistanceI[i]-g_YdistanceI[i+1])<8000) //20140107�߶�������ˮƽ��������
//	{
//		tempcout = 0;								 //20140106
//		while(i <= (g_u16InclineEndAnglePt - g_u16InclineStartAnglePt))  //20140327 ע�������λ��	ԭ��Ϊ(i<=g_u16InclineEndAnglePt)
//		  {
//		    if((g_ZdistanceI[i] > ThresVehLow) && (g_ZdistanceI[i] < ThresVehHigh) 
//				&& ((tempcout==0)||(tempcout>0 && g_YdistanceI[i]-g_YdistanceI[i-1]<2000)))//20140122����֮���Xֵ����
//		     {
//			   nocar=0;
//			   Fenche1=i;
//		     }
//		    else 
//			 {
//			   nocar++;
//		     }
//
//		   TmpNum[k][++tempcout]=i; 	  //20140106
//		    
//		   if(((nocar>7)&& ((g_YdistanceI[i]-g_YdistanceI[Fenche1])>=1600))	//20140326
//		   	||(nocar>0 && (g_YdistanceI[i]-g_YdistanceI[Fenche1]>2000)))	//�����ĸ��㲻���㳵����ֵ������һ�����ϵĵ㲻���㳵����ֵͬʱ�õ���֮ǰ���㳵����ֵ��X��ֵ����2�׵�����£���Ϊǰ�����ڲ�ͬ�ĳ�
//		    {break;}								   
//		    
//			i++	;	
//		  }	
//
//		   if(tempcout-nocar>=20)			//20140106
//		   {
//				TmpNum[k++][0]=tempcout-nocar;		   
//		   }
//										 
//		   CurrentCarNum=k;							 //����CurrentCarNum��¼��֡��ͳ�Ƶĳ�����
//		}
//	}	
//
///***************************�ֳ���һ�׶ν���************************************************/
//
///**********************************************************************************************************************/
//
//     for(m=0;m<TOTALCAR;m++)
//	  {
//	  if(Lane_Vertical[m][0][0]!=0)
//	    {
//		Lane_Vertical[m][0][6]=1;			 //��־λ��ʼ������ʶ�������ڴ�������û�����ݴ��룬�����ݴ��룺1�������ݴ��룺0
//		}					  
//	  } 
//
///**************************************ʶ���洢����*************************************************/
//      for(k=0;k<CurrentCarNum;k++)
//	  {
//		if(Tmp_Lane[k][6]==0)		 //���ڳ�ͷ��β��EnterX1�봹ֱ������֮��ĳ���
//		 { 
//
//		  CurrentCar_UseFlag=0;	     //�ȶ��Ƿ�ɹ��ı�־λ����ʼֵΪ��
//
//		  /****����ʷ�泵���������뵱ǰ����ƥ��ĳ�����������ʷ������*****/
//		  for(m=0;m<TOTALCAR;m++)
//		  {
//		     tempcout=g_VehIncSet[m].Idata.u16FrameCnt;			  //���泵�����е�֡����ֵ��tempcout
//			if(tempcout!=0)
//		    {												 
//				if(((Tmp_Lane[k][3]>=0)&&(abs(Tmp_Lane[k][3]-g_VehIncSet[m].Idata.ydataInfo[tempcout-1][0])<3000))
//				     ||((Tmp_Lane[k][4]<=0)&&(abs(Tmp_Lane[k][4]-g_VehIncSet[m].Idata.ydataInfo[tempcout-1][1])<3000))
//					 ||((Tmp_Lane[k][3]<0)&&(Tmp_Lane[k][4]>0)&&((abs(Tmp_Lane[k][3]-g_VehIncSet[m].Idata.ydataInfo[tempcout-1][0])<3000)&&(abs(Tmp_Lane[k][4]-g_VehIncSet[k].Idata.ydataInfo[tempcout-1][1])<3000)))) //����ͷ�ͳ�β���¼���ĳ�ͷ�ͳ�β��λ�ò�С��1��
//			   	{
//				    if(tempcout>=2 && (Tmp_Lane[k][3]<0) 							  //20140326
//						&& abs(Tmp_Lane[k][3]-g_VehIncSet[k].Idata.ydataInfo[tempcout-1][0])<1000 
//						&& abs(Tmp_Lane[k][4]-g_VehIncSet[k].Idata.ydataInfo[tempcout-1][1])>=1000
//						&& abs(g_VehIncSet[m].Idata.ydataInfo[tempcout-1][1] - g_VehIncSet[m].Idata.ydataInfo[tempcout-2][1])<1000)
//					{
//						if(k+1<CurrentCarNum 
//							&& abs(Tmp_Lane[k+1][4] - g_VehIncSet[k].Idata.ydataInfo[tempcout-1][1])<1000)
//						{
//							Tmp_Lane[k][4] = Tmp_Lane[k+1][4];	 //��ͷ������
//							Tmp_Lane[k][12] = Tmp_Lane[k+1][12]; //��ͷ��λ��						
//						
//						    //ʱ�䡢���������ߡ���ͷλ�á���βλ��
//							g_VehIncSet[m].Idata.tdata[tempcout] = Tmp_Lane[k][0];
//							g_VehIncSet[m].Idata.yMax[tempcout] =  Tmp_Lane[k][1];
//							g_VehIncSet[m].Idata.zMax[tempcout] =  Tmp_Lane[k][2];
//							g_VehIncSet[m].Idata.ydataInfo[tempcout][0] =  Tmp_Lane[k][3];//��ͷλ��
//							g_VehIncSet[m].Idata.ydataInfo[tempcout][1] =  Tmp_Lane[k][4];
//							memcpy(&g_VehIncSet[m].Idata.ydataInfo[tempcout][2], &Tmp_Lane[k][9],4*sizeof(int32));
//																					
//							Tmp_Lane[k+1][6] == 1;						//20140326�޳�
//							g_VehIncSet[m].Idata.u16FrameCnt = g_VehIncSet[k].Idata.u16FrameCnt+1;				//֡����1
//							Lane_Vertical[m][0][6]=0;					//�����������ݴ���
//							CurrentCar_UseFlag=1;						//�Ѽ�¼���ĳ�
//							continue;							
//						}
//						else if (k==CurrentCarNum-1 
//							&& Tmp_Lane[k][12]-g_VehIncSet[m].Idata.ydataInfo[tempcout-1][2]>=2  //��������ԭ����2������
//							&& Tmp_Lane[k][4]>g_VehIncSet[m].Idata.ydataInfo[tempcout-1][1])	//��ͷXֵ����ʷXֵ�Ƚ�
//						{
//							j = -1;
//							for(i=DafeiData[0][0]; i<=DafeiData[0][1]; i++)
//							{													
//								if(DafeiData[i][0] + g_u16InclineStartAnglePt >=g_VehIncSet[m].Idata.ydataInfo[tempcout-1][2] //��ͷ��ɵ�����
//									&& DafeiData[i][0] + g_u16InclineStartAnglePt <= Tmp_Lane[k][12]) //��ͷ��ɵ�����
//								{
//									j = DafeiData[i][0];
//									break;
//								}
//
//							}
//							if(j != -1)//�ҵ��˾�������Ĵ�ɵ�
//							{
//								for(i=j-1; i>=g_VehIncSet[m].Idata.ydataInfo[tempcout-1][2]-g_u16InclineStartAnglePt; i--)
//								{
//								 	if(g_ZdistanceI[i]>=ThresVehLow)		//�ҵ��˵�һ���г��ߵĵ�
//									{
//										Tmp_Lane[k][4] = g_YdistanceI[i];
//										Tmp_Lane[k][12] = i + g_u16InclineStartAnglePt;
//										break;
//									}
//								}							
//							}
//						}
//					}   					   
//					   
//					   //ֻ�е�ǰ����֡�ĳ�ͷλ�����50mmʱ���ŻὫ�µ�һ֡���ݼ�¼��泵����
//				    if(((abs(Tmp_Lane[k][3]-g_VehIncSet[k].Idata.ydataInfo[tempcout-1][0])>50)&&(Tmp_Lane[k][3]>=0))
//					   ||((abs(Tmp_Lane[k][4]-g_VehIncSet[k].Idata.ydataInfo[tempcout-1][1])>50)&&(Tmp_Lane[k][4]<=0))
//					   ||((Tmp_Lane[k][3]<0)&&(Tmp_Lane[k][4]>0)&&((abs(Tmp_Lane[k][3]-g_VehIncSet[k].Idata.ydataInfo[tempcout-1][0])>50)||(abs(Tmp_Lane[k][4]-g_VehIncSet[k].Idata.ydataInfo[tempcout-1][1])>50))))	 //�������ƶ�����0.5����ʱ����¼�µ�һ֡��
//					{
//					   for(i=0;i<5;i++)           //��ɨ��õ������������泵����
//					    {		
//							Lane_Vertical[m][tempcout][i]=Tmp_Lane[k][i];
//						}
//						Lane_Vertical[m][tempcout][9] = Tmp_Lane[k][9];			//��ͷ�Ƿ��ɱ�־λ
//						Lane_Vertical[m][tempcout][10]= Tmp_Lane[k][10];		//��β�Ƿ��ɱ�־λ
//						Lane_Vertical[m][tempcout][5] = Tmp_Lane[k][11];		//��β��
//						Lane_Vertical[m][tempcout][6] = Tmp_Lane[k][12];		//��ͷ��
//					    //ʱ�䡢���������ߡ���ͷλ�á���βλ��
//						g_VehIncSet[m].Idata.tdata[tempcout] = Tmp_Lane[k][0];
//						g_VehIncSet[m].Idata.yMax[tempcout] =  Tmp_Lane[k][1];
//						g_VehIncSet[m].Idata.zMax[tempcout] =  Tmp_Lane[k][2];
//						g_VehIncSet[m].Idata.ydataInfo[tempcout][0] =  Tmp_Lane[k][3];//��ͷλ��
//						g_VehIncSet[m].Idata.ydataInfo[tempcout][1] =  Tmp_Lane[k][4];
//						memcpy(&g_VehIncSet[m].Idata.ydataInfo[tempcout][2], &Tmp_Lane[k][9],4*sizeof(int32));
//		
//					   //**********************************/				   
//		             	Lane_Vertical[m][0][0]=Lane_Vertical[m][0][0]+1 ;				//֡����1
//						g_VehIncSet[m].Idata.u16FrameCnt = g_VehIncSet[m].Idata.u16FrameCnt+1;				//֡����1
//						
//					}
//					else   //20140126
//					{
//						if(Tmp_Lane[k][3]<0 && Tmp_Lane[k][4]>0)
//						{
//							l_u8SlowFlag = 1;//						
//						}
//					}
//					g_VehIncSet[m].IemptFrame = 0;		
//					Lane_Vertical[m][0][6]=0;										//��־λ��0��������������ݴ���
//					CurrentCar_UseFlag=1;											//���������Ϊ��¼���ĳ�
//				    		
//			  	}				 
//			}
//		  }
//
//			  /****����ó�û�м�¼���ȶԲ��ɹ�����Ϊ�³������ó��ĳ�βλ��С����ʱ����¼�³�, ����mֵ��С�Ŀհ״洢�ռ�*****/
//		    if((CurrentCar_UseFlag==0)&&(Tmp_Lane[k][3]<0)&& (Tmp_Lane[k][4]<0)) //20140124 (Tmp_Lane[k][4]<0)
//		    {
//			   	for(m=0;m<TOTALCAR;m++)		     //�ҵ���Сmֵ�Ŀհ״洢�ռ�
//			    {
//				  if(g_VehIncSet[m].Idata.u16FrameCnt==0)
//				     break;
//				}
//			    Lane_Vertical[m][0][0]=1;                       //��ʱֻ��1֡
//				Lane_Vertical[m][0][5]=JG_time;                 //���뼤���ʱ���¼
//				for(i=0;i<5;i++)                                 //��ɨ��õ������������泵����
//				{		
//					Lane_Vertical[m][Lane_Vertical[m][0][0]][i]=Tmp_Lane[k][i];
//				}
//
//				Lane_Vertical[m][1][9] = Tmp_Lane[k][9];		//��ͷ�Ƿ��ɱ�־λ
//				Lane_Vertical[m][1][10]= Tmp_Lane[k][10];		//��β�Ƿ��ɱ�־λ
//				Lane_Vertical[m][1][5] = Tmp_Lane[k][11];		//��β��
//				Lane_Vertical[m][1][6] = Tmp_Lane[k][12];		//��ͷ��
//			    //ʱ�䡢���������ߡ���ͷλ�á���βλ��
//				g_VehIncSet[m].Idata.tdata[tempcout] = Tmp_Lane[k][0];
//				g_VehIncSet[m].Idata.yMax[tempcout] =  Tmp_Lane[k][1];
//				g_VehIncSet[m].Idata.zMax[tempcout] =  Tmp_Lane[k][2];
//				g_VehIncSet[m].Idata.ydataInfo[tempcout][0] =  Tmp_Lane[k][3];//��ͷλ��
//				g_VehIncSet[m].Idata.ydataInfo[tempcout][1] =  Tmp_Lane[k][4];
//				memcpy(&g_VehIncSet[m].Idata.ydataInfo[tempcout][2], &Tmp_Lane[k][9],4*sizeof(int32));
//
//			    Lane_Vertical[m][0][6]=0;	                     // ��־�����ݴ���
//				g_VehIncSet[m].IemptFrame=0;	                     //�³���¼
//				Lane_Vertical[m][0][0]=Lane_Vertical[m][0][0]+1;	   //֡��1
//				g_VehIncSet[m].Idata.u16FrameCnt = g_VehIncSet[m].Idata.u16FrameCnt+1;				//֡����1
//						   
//			}
//		   }	  
//	     } 
///***********************************ʶ���洢���ݽ���********************************************/
//
///*************�������������ݴ���Lane_Vertical[m][][]��Lane_Vettical[m][0][7]++********************/
//
///************************************************/
//
//
///****************��ͷ����ExitX3���׳�***************************/
//
//	for(m=0;m<TOTALCAR;m++)
//	{
//		if((g_VehIncSet[m].Idata.u16FrameCnt>4)
//			&&(Lane_Vertical[m][Lane_Vertical[m][0][0]-1][4]>ExitX3) 
//			&&(g_VehIncSet[m].u8Istate != PASSED_USED))
//		{		  
//			g_nTmpVehSpeed = GetVehSpeed(m, EnterX2, EnterX1);	//��ͷ�ٶ�
//			g_nTmpVehLength3 = get_vehicle_length(m);	  
//			g_VehIncSet[m].u8Istate = PASSED_USED;		   //�׳��󣬽��׳���־λ��1
//		}	
//	}
///******************�׳��߼�����*******************************/
//
//
///**********************�޳�����������5�������ݴ��롢��ͷ��ʻ����Exit3���Ѿ��׳��ġ����һ֡���ĳ�ͷ��EnterX2�������޳�****************************/
//	for(m=0;m<TOTALCAR;m++)	   //20140106
//	{
//		if(g_VehIncSet[m].Idata.u16FrameCnt>=2)
//		{			
//			if((g_VehIncSet[m].IemptFrame>4)
//			   ||((g_VehIncSet[m].u8Istate == PASSED_USED) && 
//			   		((g_sspSetup.LaserDistance<=ExitX3 && Lane_Vertical[m][Lane_Vertical[m][0][0]-1][3]>g_sspSetup.LaserDistance-500))
//						|| (g_sspSetup.LaserDistance>ExitX3 && Lane_Vertical[m][Lane_Vertical[m][0][0]-1][3]>ExitX3-500)))			
//			{
//				memset(Lane_Vertical[m],0,sizeof(Lane_Vertical[m])); //�洢ɨ��֡��Ϣ����������
//			}
//		}
//	}
//
///////////////////////////////��ֱ����//////////////////////////////////
// 	if (g_totalVehicle > VEHICLE_MAX)
//	{
//		g_totalVehicle = 0;
//		for (j = 0; j < VEHICLE_MAX; j++)
//		{
//			if (g_VehicleSet[j].u8Vstate != NO_USED)
//				g_totalVehicle++;
//		}
//	}
//	for(j = 0;j < g_totalVehicle;j++)
//	{
//		i = (g_VehicleSetIndex[j] - 1) & VEHICLE_MASK;	//20140217 	��g_VehicleSetIndex[j] - 1�޸�Ϊ(g_VehicleSetIndex[j] - 1) & VEHICLE_MASK
//		if(g_VehicleSet[i].u8Vstate != NO_USED)
//		{
//			if(g_VehicleSet[i].VemptFrame > NORMAL_MAX_EMPTYFRAME && g_VehicleSet[i].Vdata.u16FrameCnt)
//			{ 		
//			   g_VehicleSet[i].u8Vstate = PASSED_USED;  //�ѽ���������β�ĳ�
//			} 
//
//			if((g_VehicleSet[i].VemptFrame > ERR_MAX_EMPTYFRAME)) // ֻ�д�ֱ������������Ϣ			   
//			{ 
//				//�೵�������θĲ���
//				if (g_VehicleSet[i].Vdata.u16FrameCnt < 10 && g_VehicleSet[i].Vdata.u16FrameCnt >=2)    //���ӶԳ�������ʱ�������֡�ĵ�ȫ������������
//				{
//					RetIsVehicle = ISVehicle(&g_VehicleSet[i].Vdata);
//				}
//				if(RetIsVehicle) 	 				   
//				{							  				   
//					//VehModels2(&g_VehicleSet[i]);
//					g_VehicleSet[i].u8Vstate = PASSED_USED;  //�ѽ���������β�ĳ� 
//				}//end
//				else
//				{
//					memset(&g_VehicleSet[i],0,sizeof(VehicleStruct));
//					g_VehicleSet[i].u8Vstate = NO_USED;  //�󴥷��ĳ�
//					for(l_u16tmp = j;l_u16tmp < g_totalVehicle - 1;l_u16tmp++)
//					   g_VehicleSetIndex[l_u16tmp] = g_VehicleSetIndex[l_u16tmp+1];
//					
//					if (g_totalVehicle <= VEHICLE_MAX)  //20140217 �޸�
//					{
//						g_VehicleSetIndex[g_totalVehicle - 1] = 0;
//						g_totalVehicle--;
//					}
//					else
//					{
//						g_totalVehicle = 0;
//					}
//					continue;				
//				}			
//			} 
//	        else if(g_VehicleSet[i].u8Vstate == PASSED_USED) 
//			{  
//				//��β����		
//				if (g_VehicleSet[i].Vdata.u16FrameCnt < 3)    //���ӶԳ�������ʱ�������֡�ĵ�ȫ������������
//				{
//					RetIsVehicle = ISVehicle(&g_VehicleSet[i].Vdata);
//				}
//				if((g_VehicleSet[i].Vdata.u16FrameCnt >= 3) || RetIsVehicle) 	 
//				   
//				{							  				   
//				  	VehModels2(&g_VehicleSet[i]); 
//				}				   
//				memset(&g_VehicleSet[i],0,sizeof(VehicleStruct));
//				g_VehicleSet[i].u8Vstate = NO_USED;
//				for(l_u16tmp = j;l_u16tmp < g_totalVehicle - 1;l_u16tmp++)
//				   g_VehicleSetIndex[l_u16tmp] = g_VehicleSetIndex[l_u16tmp+1];
//				
//				if (g_totalVehicle <= VEHICLE_MAX)  //20140217 �޸�
//				{
//					g_VehicleSetIndex[g_totalVehicle - 1] = 0;
//					g_totalVehicle--;
//				}
//				else
//				{
//					g_totalVehicle = 0;
//				}
//				continue;
//			}
//			g_VehicleSet[i].VemptFrame++;
//		} 	
//	}
//
//	/********************��ֱ������˳������ƥ�䴦��***********************/
//	/*��һ���������ֱ��������ƥ�������¼ */
//	/**********************************************************************/
//	��ֱ����������β������locatexλ���ڷ�ƥ������ֱ�����
//	/***********************************************************************/
//	for(j = 0;j < g_totalVehicle;j++)
//	{
//		i = g_VehicleSetIndex[j]-1;
//		l_leftX = g_VehicleSet[i].locateX.n32xLeft;
//		l_rightX = g_VehicleSet[i].locateX.n32xRight;
//		if(g_VehicleSet[i].u8Vstate==PASSED_USED 
//			&& (((l_leftX <-1*(g_sspSetup.u8LaneNum-1)*g_LaneWide/2) && ((l_leftX + l_rightX)/2<-1*(g_sspSetup.u8LaneNum-2)*g_LaneWide/2-200)) || l_rightX>0))
//		{
//			memset(&g_VehicleSet[i],0,sizeof(VehicleStruct));
//			g_VehicleSet[i].u8Vstate = NO_USED;  
//
//			for(l_u16tmp = j;l_u16tmp < g_totalVehicle - 1;l_u16tmp++)
//				g_VehicleSetIndex[l_u16tmp] = g_VehicleSetIndex[l_u16tmp+1];
//
//			if (g_totalVehicle <= VEHICLE_MAX)  //20140217 �޸�
//			{
//				g_VehicleSetIndex[g_totalVehicle - 1] = 0;
//				g_totalVehicle--;
//			}
//			else
//			{
//				g_totalVehicle = 0;
//			}
//		}
//	}
//	/*�ڶ�������ʼƥ��*/
//	/************************************************************/
//	ƥ��׼�򣺳����߽ӽ�����λ����ƥ������ƥ��ɹ�������˫��
//	/*************************************************************/
//	for(j = 0;j < g_totalVehicle;j++)
//	{
//		i = g_VehicleSetIndex[j] -1;
//		if(g_VehicleSet[i].u8Vstate==PASSED_USED)		  //��ֱ
//		{
//			l_leftX = g_VehicleSet[i].locateX.n32xLeft;
//			l_rightX = g_VehicleSet[i].locateX.n32xRight;
//			if (l_leftX>=-1*(g_sspSetup.u8LaneNum-1)*g_LaneWide/2 && l_rightX<=0 && l_leftX != 0)//20140922
//			{
//				for(m=0;m<g_VehIncTotal;m++)
//				{
//					k = g_VehIncSetIndex[m]-1;
//					if(g_VehIncSet[k].u8Istate == PASSED_USED)
//					{
//						l_32tmp2 = g_VehIncSet[k].Idata.u16FrameCnt &  FRAME_MASK;
//						if ((l_32tmp2>2) && (g_VehIncSet[k].IemptFrame < ERR_MAX_EMPTYFRAME))
//						{
//							
//							Tmp_Z = 0;
//							Tmp_Y = 0;
//							g_VehIncSet[k].zLen = GetMaxData(g_VehIncSet[k].Idata.zMax, l_32tmp2/3, l_32tmp2-1);		
//
//							/*��ֱ���ⳤ�����*/
//							l_32tmp2 = g_VehicleSet[i].Vdata.u16FrameCnt &  FRAME_MASK;
//							Tmp_Z=GetVHeight(&g_VehicleSet[i].Vdata, l_32tmp2);
//
//							if (Tmp_Z<1000)
//							{
//								g_VehicleSet[i].xLen = GetMaxData(g_VehicleSet[k].Vdata.xMax, 0, l_32tmp2-1);
//								if (g_VehicleSet[i].xLen<2500)
//								{
//									Tmp_Z = Myrand(1400,1700);
//								}
//							}
//							Tmp_Y=abs((int)(g_VehicleSet[i].Vdata.tdata[l_32tmp2-1]-g_VehicleSet[i].Vdata.tdata[0])) * g_VehIncSet[k].speed/36;
//							
//							if (Tmp_Z<1800 && g_VehicleSet[i].xLen<2000)
//							{
//								if (Tmp_Y>6000)
//								{
//									Tmp_Y = Myrand(3500,5000);
//								}
//							}
//							
//							if (g_VehIncSet[k].yLen == 0)//20140914
//							{
//								if (g_VehIncSet[k].u8ThrowFlag==1)
//								{
//									l_32tmp2 = g_VehIncSet[k].Idata.u16FrameCnt &  FRAME_MASK;
//									g_VehIncSet[k].yLen = GetMaxData(g_VehIncSet[k].Idata.yMax, 0, l_32tmp2-1);
//								}
//							}
//							else if (g_VehIncSet[k].yLen>0)
//							{	
//								l_32tmp2 = g_VehIncSet[k].Idata.u16FrameCnt &  FRAME_MASK;
//								g_VehIncSet[k].ndeltaY = GetMaxData(g_VehIncSet[k].Idata.yMax, l_32tmp2/3, l_32tmp2*2/3);
//								if (abs(g_VehIncSet[k].ndeltaY-g_VehIncSet[k].yLen)>1000)
//								{
//									g_VehIncSet[k].yLen = g_VehIncSet[k].ndeltaY;
//								}
//								if (g_VehIncSet[k].ndeltaY>0 && g_VehIncSet[k].yLen>0 
//									&& g_VehIncSet[k].ndeltaY<6000 && g_VehIncSet[k].ndeltaY>g_VehIncSet[k].yLen)//20140919
//								{
//									g_VehIncSet[k].yLen = g_VehIncSet[k].ndeltaY;
//								}
//							}
//
//							l_u8Count = 0;
//							if (g_VehIncSet[k].yLen<7000)
//							{
//								l_32tmp2 = g_VehIncSet[k].Idata.u16FrameCnt &  FRAME_MASK;
//								for (l_u16index=0;l_u16index<l_32tmp2; l_u16index++)
//								{
//									if (g_VehIncSet[k].Idata.ydataInfo[l_u16index][4]==1 && g_VehIncSet[k].Idata.ydataInfo[l_u16index][5]==1)
//									{
//										l_u8Count++;
//									}
//								}
//								if (l_u8Count>l_32tmp2/2 && g_VehIncSet[k].zLen<1200)//20140922
//								{
//									g_VehIncSet[k].zLen = Myrand(1400,1700);
//								}
//							}
//
//
//							if( ((g_VehIncSet[k].yLen+2500 >= Tmp_Y && g_VehIncSet[k].yLen<Tmp_Y+2500)||(g_VehIncSet[k].zLen<6000 && abs(g_VehIncSet[k].yLen-Tmp_Y)<2000 && g_VehIncSet[k].zLen<2000 && Tmp_Z<2000))
//								 && g_VehIncSet[k].zLen+1500>=Tmp_Z && g_VehIncSet[k].zLen <=Tmp_Z+300)//
//							{
//								/*ƥ��ɹ�*/
//								//���´�ֱ������������¼���ĳ����ߺ��ٶ�			 
//								//g_VehicleSet[i].zLen = g_VehIncSet[k].zLen;
//								if (abs(g_VehIncSet[k].yLen-Tmp_Y)<1000 && Tmp_Y>6000)//20140914
//								{
//									if (abs(g_VehIncSet[k].yLen-Tmp_Y)<300 && g_VehIncSet[k].yLen>Tmp_Y && g_VehIncSet[k].yLen>g_VehIncSet[k].ndeltaY)
//									{
//										g_VehicleSet[i].yLen = g_VehIncSet[k].yLen;
//									}
//									else
//									{
//										g_VehicleSet[i].yLen = Tmp_Y;
//									}									
//								}
//								else
//								{
//									g_VehicleSet[i].yLen = g_VehIncSet[k].yLen;
//								}
//								g_VehicleSet[i].speed = (g_VehIncSet[k].speed+5)/10;
//								VehModels2(&g_VehicleSet[i]);
//								/*����˳����*/
//								memset(&g_VehIncSet[k], 0,sizeof(VehIncSt)); //���˳���������ü�¼
//								g_VehIncSet[k].u8Istate = NO_USED; 
//								for(l_u16tmp = m;l_u16tmp < g_VehIncTotal - 1;l_u16tmp++)
//									g_VehIncSetIndex[l_u16tmp] = g_VehIncSetIndex[l_u16tmp+1];
//
//								if (g_VehIncTotal <= VEHICLE_MAX)  //20140217 �޸�
//								{
//									g_VehIncSetIndex[g_VehIncTotal - 1] = 0;
//									g_VehIncTotal--;
//								}
//								else
//								{
//									g_VehIncTotal = 0;
//								}
//							}
//						}
//						else if (l_32tmp2<=2 && g_VehIncSet[k].IemptFrame>20)
//						{
//							memset(&g_VehIncSet[k], 0,sizeof(VehIncSt)); //���˳���������ü�¼
//							g_VehIncSet[k].u8Istate = NO_USED; 
//							for(l_u16tmp = m;l_u16tmp < g_VehIncTotal - 1;l_u16tmp++)
//								g_VehIncSetIndex[l_u16tmp] = g_VehIncSetIndex[l_u16tmp+1];
//
//							if (g_VehIncTotal <= VEHICLE_MAX)  //20140217 �޸�
//							{
//								g_VehIncSetIndex[g_VehIncTotal - 1] = 0;
//								g_VehIncTotal--;
//							}
//							else
//							{
//								g_VehIncTotal = 0;
//							}
//						}					
//					}
//					else if ((g_VehIncSet[k].u8Istate == OCCURING_USED) && (2 == g_VehIncSet[k].u8LineFlag2) 
//						&& g_VehIncSet[k].Idata.u16FrameCnt>3 
//						&& g_VehIncSet[i].IemptFrame > NORMAL_MAX_EMPTYFRAME)//20140905
//					{
//						g_VehIncSet[k].u8Istate = PASSED_USED;//20140916
//						continue;
//					}
//				}
//			}
//			
//			memset(&g_VehicleSet[i], 0,sizeof(VehicleStruct));
//			g_VehicleSet[i].u8Vstate = NO_USED;  
//
//			for(l_u16tmp = j;l_u16tmp < g_totalVehicle - 1;l_u16tmp++)
//				g_VehicleSetIndex[l_u16tmp] = g_VehicleSetIndex[l_u16tmp+1];
//
//			if (g_totalVehicle <= VEHICLE_MAX)  //20140217 �޸�
//			{
//				g_VehicleSetIndex[g_totalVehicle - 1] = 0;
//				g_totalVehicle--;
//			}
//			else
//			{
//				g_totalVehicle = 0;
//			}			   
//		}
//	}
//
//
//	memset(g_ZdistanceV, 0, sizeof(int)*POINT_SUM);
//	memset(g_XdistanceV, 0, sizeof(int)*POINT_SUM);
//	memset(g_ZdistanceI, 0, sizeof(int)*POINT_SUM);
//	memset(g_YdistanceI, 0, sizeof(int)*POINT_SUM);
///***********************/
//JG_T0TC[8] = T0TC;
//JG_counter2[8]=t0_count2;
//	return;
}

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

//����ĳ����Χ(start��end)�ڵ�α�����
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

uint8	IsInIncSide(int32 x11, int32 x12, int32 x21, int32 x22)
{
	if (abs(x11-x21)<3000 && abs(x12-x22)<3000)
	{
	   return 1;
	}	
	return 0;
}



//С�ͳ�����������ʱ��ֻ��1��2֡ʱ��������д���
uint8 ISVehicle(VehicleDataStruct* pVdata)
{
	uint8 Ret = 0;
	uint8 l_u8HeightPt = 1;   //ÿ֡�и߶���ȵĵ���
	uint8 index = 0;
	uint8 indexFrame = 0;

	//20140217 ���ӶԲ����Ϸ����ж�
	if (pVdata == NULL)
	{
		return 0;  //Ϊ�գ�����0	
	}

	for( indexFrame = 0; indexFrame < pVdata->u16FrameCnt; indexFrame++)
	{
		for (index = 1;pVdata->zdata[indexFrame][index] > ThresVehLow;index++) //20140226 �޸�
		{
			if (pVdata->zdata[indexFrame][index-1] == pVdata->zdata[indexFrame][index])
			{
				l_u8HeightPt++;
			}
			else
			{
				if ( Ret < l_u8HeightPt)
				{
					Ret = l_u8HeightPt;
				}
				l_u8HeightPt = 1;
			}

			if (index >= FRAME_BUFLEN - 1)	// ����
				break;
		}			
	}

	//���ںܶ�֡��ɣ���ÿ֡�еĸ���߲���ȵĴ���������������϶࣬��������
	if (Ret <= 3)
	{
		if(max(pVdata->zdata[0][0], pVdata->zdata[1][0]) >= MIN_PTNUM && 
		   max(pVdata->xMax[0], pVdata->xMax[1]) > 0)  //ֻ�Ƚ�1��2��֡�ĵ����Ϳ���
		{
			Ret = 4;
		}	
	}//end

	if ( Ret > 3 )
	{
		Ret = 1;
	}
	else
	{
		Ret = 0;
	}

	return Ret;
}


//ÿ֡���г��������Ч�����жϣ�0��ʾ��Ч��1��ʾ��Ч  (��Ҫ�Ľ���
uint8 ISVehRegion(const uint16 u16RegionWide, const PointStruct *pPtStruct, const int32* pZdistance)
{
//	uint8 Ret = 0;
//	uint16 l_u16Index = 0;
//	uint16 l_u16HightPtNum = 0;
//	uint16 l_u16LowPtNum   = 0;
//	//20140217 ���ӶԲ����Ϸ����ж�
//	if (pPtStruct == NULL || pZdistance == NULL)
//	{
//		return 0;
//	}
//
//	//���� �����������ڸ߶��쳣���
//	if (pPtStruct->u16Leftpt >= pPtStruct->u16Rightpt || pPtStruct->u16Rightpt >= POINT_SUM )
//	{	//�����쳣
//		return 0;
//	}
//	for (l_u16Index = pPtStruct->u16Leftpt; l_u16Index <= pPtStruct->u16Rightpt; l_u16Index++)
//	{
//		if (pZdistance[l_u16Index] >= 5000) //�߶ȳ���5m�ĵ�
//		{
//			l_u16HightPtNum++;	
//		}
//		else if (pZdistance[l_u16Index] <= -500)	//�߶���-0.5m����Ϊ�쳣
//		{
//			l_u16LowPtNum++;
//		}
//		else
//		{
//		}
//	}
//	if ((l_u16HightPtNum + l_u16LowPtNum) > 5 ||
//		(l_u16HightPtNum + l_u16LowPtNum) >= (pPtStruct->u16Rightpt - pPtStruct->u16Leftpt+1)/2)
//	{
//		//�쳣�϶�
//		return 0;
//	}
//	
//	if (pPtStruct->u16xMaxHt > 3000 &&pZdistance[pPtStruct->u16Leftpt] == pPtStruct->u16xMaxHt) //��ߵ������߶�ֵ
//	{
//		if ((g_sspSetup.u8InstallFlag &&(pPtStruct->n32xLeft-pPtStruct->n32xRight>2000)) ||
//			((g_sspSetup.u16EndPtNum < g_sspSetup.u16VerticalZeroPos)&&
//			(pPtStruct->n32xRight-pPtStruct->n32xLeft>2000)))
//		{//	���ֵ�쳣
//			return 0;	
//		}	
//	}
//	else if (pPtStruct->u16xMaxHt > 3000 && pZdistance[pPtStruct->u16Rightpt] == pPtStruct->u16xMaxHt)//�ұߵ������߶�ֵ
//	{
//		if ((g_sspSetup.u8InstallFlag &&(pPtStruct->n32xLeft-pPtStruct->n32xRight>2000)) ||
//			((g_sspSetup.u16EndPtNum < g_sspSetup.u16VerticalZeroPos)&&
//			(pPtStruct->n32xLeft-pPtStruct->n32xRight>2000)))
//		{//	�ҵ�ֵ�쳣
//			return 0;	
//		}
//	}
//
//	if (u16RegionWide > SMALL_AREA &&((pPtStruct->u16Rightpt - pPtStruct->u16Leftpt+1 >= MIN_PTNUM) && u16RegionWide > 0))
//	    ||(pPtStruct->u16Rightpt - pPtStruct->u16Leftpt + 1 >= MIN_PTNUM*2 && u16RegionWide > 0)
//	{
//		Ret = 1;
//	}	
//
//	return Ret;
	return 0;
}

/**** ***************************************
*****20130523  ��ǰ�г�������ǰһ֡��������ƥ��
**** ���� RegionMatch*********
***  ����˵��******************
*** u16LeftX  ��ǰ֡�г��������ʼ��λ��
*** u16RightX ��ǰ֡�г�����Ľ�����λ�� 
***	pVehicle �����ṹ�壬��ȫ�ֱ���g_VehicleSet���Ӧ
*** u8Index  ��������
******************/
uint16 RegionMatch(int32 n32LeftX, int32 n32RightX, VehicleStruct *pVehicle, uint16 u16Index)
{
////	uint16 RetIndex = u16Index;	
//	//20140217 ���ӶԲ����Ϸ����ж�
//	if (pVehicle == NULL || u16Index > VEHICLE_MASK)
//	{
//		return ERRORVALUE; 
//	}
//
//	if (u16Index == VEHICLE_MAX-1)  //ǰһ֡�����һ���г�����,ֱ��ƥ��ɹ�
//	{
//		RetIndex = u16Index;	
//	}
//	else
//	{  //��ǰ������ǰһ֡����һ������ƥ��
//		if (IS_INSIDE(n32LeftX, n32RightX, pVehicle[u16Index+1].locateX.n32xLeft,pVehicle[u16Index+1].locateX.n32xRight)) //�����ص�
//		{
//			//��ǰ������ǰһ֡���������򶼲���ƥ��
//			if( abs(n32LeftX - pVehicle[u16Index+1].locateX.n32xRight) >= 
//			    abs(pVehicle[u16Index+1].locateX.n32xLeft - n32RightX) ) 
//			{
//				RetIndex = u16Index;
//			}	
//			else
//			{
//				RetIndex = u16Index + 1;
//			}
//		}
//		else
//		{	//��ȫ��ƥ�䣬ֱ�ӷ���
//			RetIndex = u16Index;
//		}
//	}
	
//	return RetIndex;	
    return 0;
}

uint8 RegionMatch_Point(PointStruct *pVILocateX, PointStruct *pLocateX, PointStruct *pPtData, uint8 u8Flag)
{
//	uint8 ret = 0;
//	uint16 l_u16Startpt  = g_u16VerticalStartAnglePt;
//	uint16 u16PreStartpt = 	pVILocateX->u16Startpt;
//	uint16 u16PreLeftpt	 =  pVILocateX->u16Leftpt;
//	uint16 u16PreRightpt =	pVILocateX->u16Rightpt;
//	uint16 u16Leftpt     =  pPtData->u16Leftpt;
//	uint16 u16Rightpt    =	pPtData->u16Rightpt;
//
//	if (pVILocateX == NULL || pLocateX==NULL || pPtData==NULL)
//		return ret;
//
//	if (u8Flag)	 //1��ʾ��б������
//	{
//		l_u16Startpt  = g_u16InclineStartAnglePt;
//		u16PreStartpt = pVILocateX->u16Startpt;
//		u16PreLeftpt  = pVILocateX->u16Leftpt;
//		u16PreRightpt =	pVILocateX->u16Rightpt;
//		u16Leftpt     = pPtData->u16Leftpt;
//		u16Rightpt    =	pPtData->u16Rightpt;
//	}
//
//	if (abs(pLocateX->n32xLeft - pPtData->n32xLeft) > 3000 ||
//	    abs(pLocateX->n32xRight - pPtData->n32xRight) > 3000)	  //�������ܴ�ƥ�䲻�ɹ�
//	{
//		return ret;
//	}
//
//	if (u16PreRightpt-u16PreLeftpt > u16Rightpt-u16Leftpt)	//�����ٵ����ĵ����ڵ�����ķ�Χ�ڣ�˵��ƥ��ɹ�
//	{
//		if ((((u16Rightpt+u16Leftpt)>>1) + l_u16Startpt >=  u16PreLeftpt + u16PreStartpt)
//		    && (((u16Rightpt+u16Leftpt)>>1) + l_u16Startpt <=  u16PreRightpt+u16PreStartpt))
//		{
//			ret = 1;
//		}
//	}
//	else
//	{
//		if ((((u16PreRightpt+u16PreLeftpt)>>1) + u16PreStartpt >=  u16Leftpt + l_u16Startpt) 
//		    && (((u16PreRightpt+u16PreLeftpt)>>1) + u16PreStartpt <=  u16Rightpt + l_u16Startpt))
//		{
//			ret = 1;
//		}		
//	}
	return 0;
}

void RegionMerging(PointSet *pFrameInfo)
{
////	uint16 l_u16index = 1;
////	uint16 l_u16tmp   = 1;
////	uint16 l_u16TmpIndex = 0;
////
////	//20140217 ���ӶԲ����Ϸ����ж�
////	if (pFrameInfo == NULL || pFrameInfo->u8Sum > POINTSET_CNT)
////	{
////		return;
////	}
////
////  //�ϲ����򣬸��ݳ����Ŀ�ȣ���һ����������ڲ��ܳ���2�������ϵĳ��������������ܳ���������	 (��4������Ч)
////	for (l_u16index = 1;l_u16index < pFrameInfo->u8Sum;l_u16index++)
////	{
////		if(abs(pFrameInfo->Ptdata[l_u16index-1].n32xLeft-pFrameInfo->Ptdata[l_u16index-1].n32xRight) >= g_LaneWide ||
////		   abs(pFrameInfo->Ptdata[l_u16index].n32xLeft-pFrameInfo->Ptdata[l_u16index].n32xRight) >= g_LaneWide)  //��һ������ڳ�����
////		{
////			if ((abs(pFrameInfo->Ptdata[l_u16index-1].n32xRight) < g_NearMaxWide &&
////			    abs(pFrameInfo->Ptdata[l_u16index].n32xRight) < g_NearMaxWide) ||
////				(abs(pFrameInfo->Ptdata[l_u16index-1].n32xLeft) > g_FarMinWide &&
////				abs(pFrameInfo->Ptdata[l_u16index].n32xLeft) > g_FarMinWide))  //�ϲ�ͬ�������
////			{
////				pFrameInfo->Ptdata[l_u16index - 1].n32xRight = pFrameInfo->Ptdata[l_u16index].n32xRight;		
////				pFrameInfo->Ptdata[l_u16index - 1].u16Rightpt = pFrameInfo->Ptdata[l_u16index].u16Rightpt;
////				pFrameInfo->Ptdata[l_u16index - 1].u16xMaxHt = max(pFrameInfo->Ptdata[l_u16index - 1].u16xMaxHt,pFrameInfo->Ptdata[l_u16index].u16xMaxHt);
////				pFrameInfo->Ptdata[l_u16index - 1].u16xDis = abs(pFrameInfo->Ptdata[l_u16index - 1].n32xRight - pFrameInfo->Ptdata[l_u16index - 1].n32xLeft);
////				for (l_u16tmp = l_u16index;l_u16tmp < pFrameInfo->u8Sum - 1;l_u16tmp++)
////				{
////					memcpy(&(pFrameInfo->Ptdata[l_u16tmp]),&(pFrameInfo->Ptdata[l_u16tmp+1]),sizeof(PointStruct));
////				}
////				memset(&(pFrameInfo->Ptdata[l_u16tmp]),0,sizeof(PointStruct));
////				pFrameInfo->u8Sum--;
////				l_u16index--;	
////			}	
////		}
////		else if (l_u16index+1 < pFrameInfo->u8Sum &&
////		         abs(pFrameInfo->Ptdata[l_u16index-1].n32xLeft-pFrameInfo->Ptdata[l_u16index].n32xRight) < g_LaneWide &&
////				 abs(pFrameInfo->Ptdata[l_u16index].n32xLeft-pFrameInfo->Ptdata[l_u16index+1].n32xRight) < g_LaneWide &&
////				 ((abs(pFrameInfo->Ptdata[l_u16index-1].n32xRight) < g_NearMaxWide && 
////				 abs(pFrameInfo->Ptdata[l_u16index+1].n32xRight) < g_NearMaxWide) ||
////				 (abs(pFrameInfo->Ptdata[l_u16index-1].n32xLeft) > g_FarMinWide &&
////				 abs(pFrameInfo->Ptdata[l_u16index+1].n32xLeft) > g_FarMinWide))) //3������ͬ�࣬������������С�ڳ�����
////		{
////			//����ϲ�ʱ��ѡ��ȽϺ��ʵ�2��������кϲ�
////			if (abs(pFrameInfo->Ptdata[l_u16index-1].n32xLeft-pFrameInfo->Ptdata[l_u16index].n32xRight) <= 
////			    abs(pFrameInfo->Ptdata[l_u16index].n32xLeft-pFrameInfo->Ptdata[l_u16index+1].n32xRight))  //�ϲ�ǰ������������
////			{
////				pFrameInfo->Ptdata[l_u16index - 1].n32xRight = pFrameInfo->Ptdata[l_u16index].n32xRight;		
////				pFrameInfo->Ptdata[l_u16index - 1].u16Rightpt = pFrameInfo->Ptdata[l_u16index].u16Rightpt;
////				pFrameInfo->Ptdata[l_u16index - 1].u16xMaxHt = max(pFrameInfo->Ptdata[l_u16index - 1].u16xMaxHt,pFrameInfo->Ptdata[l_u16index].u16xMaxHt);
////				pFrameInfo->Ptdata[l_u16index - 1].u16xDis = abs(pFrameInfo->Ptdata[l_u16index - 1].n32xRight - pFrameInfo->Ptdata[l_u16index - 1].n32xLeft);
////				for (l_u16tmp = l_u16index;l_u16tmp < pFrameInfo->u8Sum - 1;l_u16tmp++)
////				{
////					memcpy(&(pFrameInfo->Ptdata[l_u16tmp]),&(pFrameInfo->Ptdata[l_u16tmp+1]),sizeof(PointStruct));
////				}
////				memset(&(pFrameInfo->Ptdata[l_u16tmp]),0,sizeof(PointStruct));
////				pFrameInfo->u8Sum--;
////				l_u16index--;
////			}
////			else   //�ϲ���������������
////			{
////				pFrameInfo->Ptdata[l_u16index].n32xRight = pFrameInfo->Ptdata[l_u16index+1].n32xRight;		
////				pFrameInfo->Ptdata[l_u16index].u16Rightpt = pFrameInfo->Ptdata[l_u16index+1].u16Rightpt;
////				pFrameInfo->Ptdata[l_u16index].u16xMaxHt = max(pFrameInfo->Ptdata[l_u16index].u16xMaxHt,pFrameInfo->Ptdata[l_u16index+1].u16xMaxHt);
////				pFrameInfo->Ptdata[l_u16index].u16xDis = abs(pFrameInfo->Ptdata[l_u16index].n32xRight - pFrameInfo->Ptdata[l_u16index].n32xLeft);
////				for (l_u16tmp = l_u16index+1;l_u16tmp < pFrameInfo->u8Sum - 1;l_u16tmp++)
////				{
////					memcpy(&(pFrameInfo->Ptdata[l_u16tmp]),&(pFrameInfo->Ptdata[l_u16tmp+1]),sizeof(PointStruct));
////				}
////				memset(&(pFrameInfo->Ptdata[l_u16tmp]),0,sizeof(PointStruct));
////				pFrameInfo->u8Sum--;
////				l_u16index--;
////			}	
////		}
////		else if (abs(pFrameInfo->Ptdata[l_u16index-1].n32xLeft-pFrameInfo->Ptdata[l_u16index].n32xRight) < g_LaneWide &&
////		        ((abs(pFrameInfo->Ptdata[l_u16index-1].n32xRight) < g_NearMaxWide && abs(pFrameInfo->Ptdata[l_u16index].n32xRight) < g_NearMaxWide)||
////				(abs(pFrameInfo->Ptdata[l_u16index-1].n32xLeft) > g_FarMinWide && abs(pFrameInfo->Ptdata[l_u16index].n32xLeft) > g_FarMinWide)))	//ͬ��2��������С�ڳ�����ֱ�Ӻϲ�
////		{
////			pFrameInfo->Ptdata[l_u16index - 1].n32xRight = pFrameInfo->Ptdata[l_u16index].n32xRight;		
////			pFrameInfo->Ptdata[l_u16index - 1].u16Rightpt = pFrameInfo->Ptdata[l_u16index].u16Rightpt;
////			pFrameInfo->Ptdata[l_u16index - 1].u16xMaxHt = max(pFrameInfo->Ptdata[l_u16index - 1].u16xMaxHt,pFrameInfo->Ptdata[l_u16index].u16xMaxHt);
////			pFrameInfo->Ptdata[l_u16index - 1].u16xDis = abs(pFrameInfo->Ptdata[l_u16index - 1].n32xRight - pFrameInfo->Ptdata[l_u16index - 1].n32xLeft);
////			for (l_u16tmp = l_u16index;l_u16tmp < pFrameInfo->u8Sum - 1;l_u16tmp++)
////			{
////				memcpy(&(pFrameInfo->Ptdata[l_u16tmp]),&(pFrameInfo->Ptdata[l_u16tmp+1]),sizeof(PointStruct));
////			}
////			memset(&(pFrameInfo->Ptdata[l_u16tmp]),0,sizeof(PointStruct));
////			pFrameInfo->u8Sum--;
////			l_u16index--;
////		}
////		
////	}
return ;
}

//������װ��ʽʱ���ϲ�����
void RegionMergingEx(PointSet *pFrameInfo)
{
	uint16 l_u16index = 1;
	uint16 l_u16tmp   = 1;
	uint8  l_u8Region1 = 0;
	uint8  l_u8Region2 = 0;

	//20140217 ���ӶԲ����Ϸ����ж�
	if (pFrameInfo == NULL || pFrameInfo->u8Sum > POINTSET_CNT)
	{
		return;
	}

  //�ϲ����򣬸��ݳ����Ŀ�ȣ���һ����������ڲ��ܳ���2�������ϵĳ��������������ܳ���������	 
	for (l_u16index = 1;l_u16index < pFrameInfo->u8Sum;l_u16index++)
	{
		if(abs(pFrameInfo->Ptdata[l_u16index-1].n32xLeft-pFrameInfo->Ptdata[l_u16index-1].n32xRight) >= g_LaneWide ||
		   abs(pFrameInfo->Ptdata[l_u16index].n32xLeft-pFrameInfo->Ptdata[l_u16index].n32xRight) >= g_LaneWide)  //��������ڳ�����
		{
			if (((pFrameInfo->Ptdata[l_u16index-1].n32xRight + g_MedianLeftWide) < 0 &&			   //���ϸ���
				    (pFrameInfo->Ptdata[l_u16index].n32xRight+g_MedianLeftWide) < 0) ||			   //���ϸ���
					(pFrameInfo->Ptdata[l_u16index-1].n32xLeft > g_MedianRightWide &&
					pFrameInfo->Ptdata[l_u16index].n32xLeft > g_MedianRightWide))  //�ϲ�ͬ�������
			{
				if (g_sspSetup.u8LaneNum == 4)  //4����
				{
					pFrameInfo->Ptdata[l_u16index - 1].n32xRight = pFrameInfo->Ptdata[l_u16index].n32xRight;		
					pFrameInfo->Ptdata[l_u16index - 1].u16Rightpt = pFrameInfo->Ptdata[l_u16index].u16Rightpt;
					pFrameInfo->Ptdata[l_u16index - 1].u16xMaxHt = max(pFrameInfo->Ptdata[l_u16index - 1].u16xMaxHt,pFrameInfo->Ptdata[l_u16index].u16xMaxHt);
					pFrameInfo->Ptdata[l_u16index - 1].u16xDis = abs(pFrameInfo->Ptdata[l_u16index - 1].n32xRight - pFrameInfo->Ptdata[l_u16index - 1].n32xLeft);
					for (l_u16tmp = l_u16index;l_u16tmp < pFrameInfo->u8Sum - 1;l_u16tmp++)
					{
						memcpy(&(pFrameInfo->Ptdata[l_u16tmp]),&(pFrameInfo->Ptdata[l_u16tmp+1]),sizeof(PointStruct));
					}
					memset(&(pFrameInfo->Ptdata[l_u16tmp]),0,sizeof(PointStruct));
					pFrameInfo->u8Sum--;
					l_u16index--;
				}
				else //6����
				{
					if ((abs(pFrameInfo->Ptdata[l_u16index-1].n32xLeft-pFrameInfo->Ptdata[l_u16index].n32xRight) < g_sspSetup.LaneWide*2) ||
					    (abs(pFrameInfo->Ptdata[l_u16index-1].n32xLeft-pFrameInfo->Ptdata[l_u16index-1].n32xRight) >g_sspSetup.LaneWide*2) ||
						 abs(pFrameInfo->Ptdata[l_u16index].n32xLeft-pFrameInfo->Ptdata[l_u16index].n32xRight) >g_sspSetup.LaneWide*2) //2�������С��2�������������һ�������2��������
					{
						pFrameInfo->Ptdata[l_u16index - 1].n32xRight = pFrameInfo->Ptdata[l_u16index].n32xRight;		
						pFrameInfo->Ptdata[l_u16index - 1].u16Rightpt = pFrameInfo->Ptdata[l_u16index].u16Rightpt;
						pFrameInfo->Ptdata[l_u16index - 1].u16xMaxHt = max(pFrameInfo->Ptdata[l_u16index - 1].u16xMaxHt,pFrameInfo->Ptdata[l_u16index].u16xMaxHt);
						pFrameInfo->Ptdata[l_u16index - 1].u16xDis = abs(pFrameInfo->Ptdata[l_u16index - 1].n32xRight - pFrameInfo->Ptdata[l_u16index - 1].n32xLeft);
						for (l_u16tmp = l_u16index;l_u16tmp < pFrameInfo->u8Sum - 1;l_u16tmp++)
						{
							memcpy(&(pFrameInfo->Ptdata[l_u16tmp]),&(pFrameInfo->Ptdata[l_u16tmp+1]),sizeof(PointStruct));
						}
						memset(&(pFrameInfo->Ptdata[l_u16tmp]),0,sizeof(PointStruct));
						pFrameInfo->u8Sum--;
						l_u16index--;
					}
				}	
			}		
		}
		else if (l_u16index+1 < pFrameInfo->u8Sum &&
		         abs(pFrameInfo->Ptdata[l_u16index-1].n32xLeft-pFrameInfo->Ptdata[l_u16index].n32xRight) < g_LaneWide &&
				 abs(pFrameInfo->Ptdata[l_u16index].n32xLeft-pFrameInfo->Ptdata[l_u16index+1].n32xRight) < g_LaneWide &&
				 (((pFrameInfo->Ptdata[l_u16index-1].n32xRight+g_MedianLeftWide) < 0 && 	 //��ֵ
				 (pFrameInfo->Ptdata[l_u16index+1].n32xRight+g_MedianLeftWide) < 0) ||		 //���ϸ���
				 (pFrameInfo->Ptdata[l_u16index-1].n32xLeft > g_MedianRightWide &&
				 pFrameInfo->Ptdata[l_u16index+1].n32xLeft > g_MedianRightWide))) //3������ͬ�࣬������������С�ڳ�����
		{
			//����ϲ�ʱ��ѡ��ȽϺ��ʵ�2��������кϲ�
			if (abs(pFrameInfo->Ptdata[l_u16index-1].n32xLeft-pFrameInfo->Ptdata[l_u16index].n32xRight) <= 
			    abs(pFrameInfo->Ptdata[l_u16index].n32xLeft-pFrameInfo->Ptdata[l_u16index+1].n32xRight))  //�ϲ�ǰ������������
			{
				pFrameInfo->Ptdata[l_u16index - 1].n32xRight = pFrameInfo->Ptdata[l_u16index].n32xRight;		
				pFrameInfo->Ptdata[l_u16index - 1].u16Rightpt = pFrameInfo->Ptdata[l_u16index].u16Rightpt;
				pFrameInfo->Ptdata[l_u16index - 1].u16xMaxHt = max(pFrameInfo->Ptdata[l_u16index - 1].u16xMaxHt,pFrameInfo->Ptdata[l_u16index].u16xMaxHt);
				pFrameInfo->Ptdata[l_u16index - 1].u16xDis = abs(pFrameInfo->Ptdata[l_u16index - 1].n32xRight - pFrameInfo->Ptdata[l_u16index - 1].n32xLeft);
				for (l_u16tmp = l_u16index;l_u16tmp < pFrameInfo->u8Sum - 1;l_u16tmp++)
				{
					memcpy(&(pFrameInfo->Ptdata[l_u16tmp]),&(pFrameInfo->Ptdata[l_u16tmp+1]),sizeof(PointStruct));
				}
				memset(&(pFrameInfo->Ptdata[l_u16tmp]),0,sizeof(PointStruct));
				pFrameInfo->u8Sum--;
				l_u16index--;
			}
			else   //�ϲ���������������
			{
				pFrameInfo->Ptdata[l_u16index].n32xRight = pFrameInfo->Ptdata[l_u16index+1].n32xRight;		
				pFrameInfo->Ptdata[l_u16index].u16Rightpt = pFrameInfo->Ptdata[l_u16index+1].u16Rightpt;
				pFrameInfo->Ptdata[l_u16index].u16xMaxHt = max(pFrameInfo->Ptdata[l_u16index].u16xMaxHt,pFrameInfo->Ptdata[l_u16index+1].u16xMaxHt);
				pFrameInfo->Ptdata[l_u16index].u16xDis = abs(pFrameInfo->Ptdata[l_u16index].n32xRight - pFrameInfo->Ptdata[l_u16index].n32xLeft);
				for (l_u16tmp = l_u16index+1;l_u16tmp < pFrameInfo->u8Sum - 1;l_u16tmp++)
				{
					memcpy(&(pFrameInfo->Ptdata[l_u16tmp]),&(pFrameInfo->Ptdata[l_u16tmp+1]),sizeof(PointStruct));
				}
				memset(&(pFrameInfo->Ptdata[l_u16tmp]),0,sizeof(PointStruct));
				pFrameInfo->u8Sum--;
				l_u16index--;
			}	
		}
		else if (abs(pFrameInfo->Ptdata[l_u16index-1].n32xLeft-pFrameInfo->Ptdata[l_u16index].n32xRight) < g_LaneWide &&
		        (((pFrameInfo->Ptdata[l_u16index-1].n32xRight+g_MedianLeftWide) < 0 && 		//���ϸ���
				(pFrameInfo->Ptdata[l_u16index].n32xRight+g_MedianLeftWide) < 0)||			//���ϸ���
				(pFrameInfo->Ptdata[l_u16index-1].n32xLeft > g_MedianRightWide && 
				pFrameInfo->Ptdata[l_u16index].n32xLeft > g_MedianRightWide)))	//ͬ��2��������С�ڳ�����ֱ�Ӻϲ�
		{
			pFrameInfo->Ptdata[l_u16index - 1].n32xRight = pFrameInfo->Ptdata[l_u16index].n32xRight;		
			pFrameInfo->Ptdata[l_u16index - 1].u16Rightpt = pFrameInfo->Ptdata[l_u16index].u16Rightpt;
			pFrameInfo->Ptdata[l_u16index - 1].u16xMaxHt = max(pFrameInfo->Ptdata[l_u16index - 1].u16xMaxHt,pFrameInfo->Ptdata[l_u16index].u16xMaxHt);
			pFrameInfo->Ptdata[l_u16index - 1].u16xDis = abs(pFrameInfo->Ptdata[l_u16index - 1].n32xRight - pFrameInfo->Ptdata[l_u16index - 1].n32xLeft);
			for (l_u16tmp = l_u16index;l_u16tmp < pFrameInfo->u8Sum - 1;l_u16tmp++)
			{
				memcpy(&(pFrameInfo->Ptdata[l_u16tmp]),&(pFrameInfo->Ptdata[l_u16tmp+1]),sizeof(PointStruct));
			}
			memset(&(pFrameInfo->Ptdata[l_u16tmp]),0,sizeof(PointStruct));
			pFrameInfo->u8Sum--;
			l_u16index--;
		}
		
	}
}

void VehModels2(VehicleStruct *pVehicle)
{  
//	uint16 Veh_Num    = 0;  //����֡��
//	uint32 VehHeight  = 0;
//	uint32  VehWide   = 0;
//	uint32 VehLength = 0;
//	uint16 u16Index  = 0;
//	uint32  VehSpeed = 0; 
//	uint8  VehPattern = 0;  //����
//	uint8  u8Lane    = 0;	//����
//	uint8  l_u8TestFrame = 0;
//
//	uint8 strsend[10] = {0};
//	static uint32 send_count = 1;
//	uint16  l_anTopPlanenessFlag[FRAME_MAXCNT] = {0};  //����ÿ֡�Ķ���ƽ����ʶ��0��ʾ��ƽ����1��ʾƽ��
//	uint16 tmp1   = 0;
//	uint8 wtmp[4];
//	uint32  l_u32Distance  = 0;
//	
//	 _Cycle_Que_Continue cycle_que_continue;
//	
//	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
//	uint32 save_add = 0; 
//	uint8 ReadFWtemp[8];
//	uint8 WriteFWtemp[8];
//	_VehSendInfo VehSendInfo_tmp;	//UART1�������ṹ��
//
//	uint32 day_temp = 0;
//	uint32 year_temp = 0;
//	uint8 WriteSDtemp[512] = {0};
// 
//	
//    //20140217 �����Ϸ����ж�
//	if (pVehicle == NULL)
//	{
//		return;
//	}
//
//	Veh_Num = pVehicle->Vdata.u16FrameCnt;
//	//20140217 ����
//	if (Veh_Num > FRAME_MAXCNT || Veh_Num < 1)
//	{
//		Veh_Num = 0;  //֡���������֡��ֵ��֡����ֵΪ0
//		pVehicle->Vdata.u16FrameCnt = 1;
//	}
//	
//	for( u16Index = 0; u16Index < Veh_Num; u16Index++)
//	{	//���㳵��
//		if (pVehicle->xLen < pVehicle->Vdata.xMax[u16Index])
//		{
//			pVehicle->xLen = pVehicle->Vdata.xMax[u16Index];	
//		}
//		//����ÿ֡�ĳ��� 
//		pVehicle->Vdata.zMax[u16Index] = GetVehHeight(&pVehicle->Vdata, u16Index);
//		//���㳵��
//		if (pVehicle->zLen < pVehicle->Vdata.zMax[u16Index] && pVehicle->Vdata.zMax[u16Index] < 4800 )
//		{
//			pVehicle->zLen = pVehicle->Vdata.zMax[u16Index];
//		}		
//	}
//
//	// ����ż������ë�̣�����1��2֡�ĸ߶��쳣�����³����жϴ������޳���Щ�쳣�߶�
//	for (u16Index = 0; u16Index < Veh_Num; u16Index++)
//	{
//		if (pVehicle->zLen - pVehicle->Vdata.zMax[u16Index] < 1000 && pVehicle->zLen > 3000 )	
//		{
//			l_u8TestFrame++;
//		}		
//	}
//	//���֡��С��3�������ҳ���
//	if (l_u8TestFrame <=2 && l_u8TestFrame > 0)
//	{
//		for (u16Index = 0; u16Index < Veh_Num; u16Index++)
//		{
//			if (pVehicle->zLen - VehHeight >= 1000 && VehHeight < pVehicle->Vdata.zMax[u16Index] &&
//				pVehicle->zLen - pVehicle->Vdata.zMax[u16Index] >= 1000)	
//			{
//				VehHeight = pVehicle->Vdata.zMax[u16Index];
//			}
//			else if (pVehicle->zLen - pVehicle->Vdata.zMax[u16Index] < 1000 && u16Index >= 1)
//			{
//				pVehicle->Vdata.zMax[u16Index] = pVehicle->Vdata.zMax[u16Index-1];
//			}
//			else if (pVehicle->zLen - pVehicle->Vdata.zMax[u16Index] < 1000)
//			{
//				pVehicle->Vdata.zMax[u16Index] = 800;	
//			}			
//		}
//		if(VehHeight)
//		{
//			pVehicle->zLen = VehHeight;		
//		}
//	}
//
//	VehSpeed  = pVehicle->speed;
//	VehHeight = pVehicle->zLen;
//	VehWide   = pVehicle->xLen;
//	VehLength = pVehicle->yLen;
//
//	if (VehHeight > 2700 && Veh_Num < 3)  //�󳵲��������ж���С�ͳ�  ����20131212
//	{
//		return;
//	}
//
//	//������  (��Ҫ�޸ģ�
//	if(g_sspSetup.LaneWide)
//	{
//		tmp1 = abs(pVehicle->locateX.n32xLeft+pVehicle->locateX.n32xRight)>>1;
//		if ( !g_sspSetup.u8InstallFlag && g_sspSetup.u8LaneNum < 6)  //��װ4����
//		{
//			if (abs(tmp1) < g_sspSetup.n32LaserHorizOff+g_sspSetup.MedianWide+
//							 g_sspSetup.LaneWide*(g_sspSetup.u8LaneNum>>1))
//			{
//				pVehicle->u8Lane = (abs(tmp1) - g_sspSetup.n32LaserHorizOff)/g_LaneWide;
//				if (pVehicle->u8Lane < 1)
//				{
//					pVehicle->u8Lane = 1;
//				}
//				else 
//				{
//					pVehicle->u8Lane = 2;
//				}
//			}
//			else
//			{
//				pVehicle->u8Lane = abs(tmp1 - (g_sspSetup.MedianWide+g_sspSetup.n32LaserHorizOff))/g_LaneWide;
//				if (pVehicle->u8Lane < 3)
//				{
//					pVehicle->u8Lane = 3;
//				}
//				else
//				{
//					pVehicle->u8Lane = 4;
//				}
//			}
//		}
//		else if (g_sspSetup.u8LaneNum == 4)  //��װ4����
//		{
//			if(pVehicle->locateX.u16Leftpt < g_sspSetup.u16VerticalZeroPos-g_u16VerticalStartAnglePt)
//			{ 	
//				pVehicle->u8Lane = abs(tmp1 - g_MedianLeftWide) /g_LaneWide;
//				pVehicle->u8Lane +=0x02;	 	
//				if(pVehicle->u8Lane < 0x03)
//					pVehicle->u8Lane = 0x03;
//				else
//				    pVehicle->u8Lane = 0x04;
//			}
//			else
//			{
//				pVehicle->u8Lane = abs(tmp1 - g_MedianRightWide) /g_LaneWide;	
//				pVehicle->u8Lane +=0x02;
//				if(pVehicle->u8Lane < 0x01)
//					pVehicle->u8Lane = 0x01; 
//				else
//					pVehicle->u8Lane = 0x02;
//			}
//		}
//		else //��װ6����
//		{	//20140829
//			if (abs(g_sspSetup.n32LaserHorizOff)>g_LaneWide  //ƫ�Ƴ���1���������
//				&& 0 == g_MedianLeftWide && 0 == g_MedianRightWide)	 //��������Ϊ0
//			{
//				if (pVehicle->locateX.u16Leftpt < g_sspSetup.u16VerticalZeroPos-g_u16VerticalStartAnglePt)
//				{
//				   	pVehicle->u8Lane = abs(tmp1 - g_MedianLeftWide) /g_LaneWide+1;
//					pVehicle->u8Lane +=0x03;
//					if(pVehicle->u8Lane > 0x06)
//						pVehicle->u8Lane = 0x06; 
//					if(pVehicle->u8Lane <= 0x03)
//						pVehicle->u8Lane = 0x04;
//				}
//				else
//				{
//					pVehicle->u8Lane = abs(tmp1 - g_MedianRightWide) /g_LaneWide+1;	
//					if(pVehicle->u8Lane > 0x03)
//						pVehicle->u8Lane = 0x03; 
//					if(pVehicle->u8Lane <= 0)
//						pVehicle->u8Lane = 0x01;				
//				}
//			}
//			else
//			{
//				if(pVehicle->locateX.u16Leftpt < g_sspSetup.u16VerticalZeroPos-g_u16VerticalStartAnglePt)
//				{ 	
//					pVehicle->u8Lane = abs(tmp1 - g_MedianLeftWide) /g_LaneWide+1;		 //20131210��������1 ������
//					pVehicle->u8Lane +=0x03;	 	
//					if(pVehicle->u8Lane > 0x06)
//						pVehicle->u8Lane = 0x06; 
//					if(pVehicle->u8Lane <= 0x03)
//						pVehicle->u8Lane = 0x04;
//				}
//				else
//				{
//					pVehicle->u8Lane = abs(tmp1 - g_MedianRightWide) /g_LaneWide+1;	
//	//				pVehicle->u8Lane +=0x03;
//					if(pVehicle->u8Lane > 0x03)
//						pVehicle->u8Lane = 0x03; 
//					if(pVehicle->u8Lane <= 0)
//						pVehicle->u8Lane = 0x01;
//				}			
//			}
//		}	  	
//	}
//
//	u8Lane = pVehicle->u8Lane;
//
//	 //1��С�� 2С�ͻ��� 3 ��ͳ� 4���ͻ��� 5���ͻ��� 6�ش��ͻ��� 7��װ�䳵  8������ 9Ħ�г�
//	 if(VehLength <  BIGANGSMALLTHR)//	20131223  ��|| VehHeight < 2300ɾȥ
//	 {         
//	    //6000����С���ж�
//		VehPattern = GetLightVehPattern(pVehicle);		
//	 }
//	 else if (VehLength >= BIGANGSMALLTHR)
//	 {	 	
//	 	//6000���ϴ��ͳ��ж�		
//		VehPattern = GetLargeVehPattern(pVehicle);	
//	 }
//
//
//	 //20130403 �������������ڲ���
//	 if (VehPattern  < ZHONGXIAOKE)
//	 {
//	 	VehPattern = ZHONGXIAOKE;	
//	 }
//
//	 if (g_sspSetup.u8RoadType && (VehPattern == MOTUOCHE))	   //����·�ϲ���Ħ�г�
//	 {
//	 	VehPattern = ZHONGXIAOKE;
//	 }
	//�����Ѿ��г����͵ĳ����������ٶȵȲ���
	/**************************************************
	if (VehPattern <= XIAOHUOCHE)	//С�ͳ���С����
	{
		if (VehLength > BIGANGSMALLTHR || VehLength < 1340)		 //��������
		{
			VehLength = Myrand(3500, 5800);
		}
		if (VehWide < 1000 || VehWide > 2500) //��������
		{
			VehWide = Myrand(1400, 2100);
		}
		if (VehHeight < 800 || VehHeight > 4000)  //��������
		{
			VehHeight = Myrand(1400, 2200);
		}
		if (g_sspSetup.u8RoadType)  //����
		{
			if (VehPattern == ZHONGXIAOKE)
			{
				if (VehSpeed < 40 || VehSpeed > 150)	 //��������
				{
					VehSpeed = Myrand(80, 110);
				}
			}
			else
			{
				if (VehSpeed < 40 || VehSpeed > 110)	 //��������
				{
					VehSpeed = Myrand(65, 90);
				}				
			}
		}
		else
		{
			if (VehPattern == ZHONGXIAOKE)
			{
				if (VehSpeed < 1 || VehSpeed > 110)	 //��������
				{
					VehSpeed = Myrand(70, 100);
				}
			}
			else
			{
		   		if (VehSpeed < 1 || VehSpeed > 100)	 //��������
				{
					VehSpeed = Myrand(60, 80);
				}
			}			
		}
	}
	else if (VehPattern <= DAHUO)		//��ͳ����л������
	{
		if (VehLength < BIGANGSMALLTHR || VehLength > 13000)		 //��������
		{
			VehLength = Myrand(6800, 11000);
		}
		if (VehWide < 1800 || VehWide > 3500) //��������
		{
			VehWide = Myrand(1900, 2700);
		}
		if (VehHeight < 1800 || VehHeight > 5000)  //��������
		{
			VehHeight = Myrand(2100, 4500);
		}
		if (g_sspSetup.u8RoadType)  //����
		{
			if (VehSpeed < 40 || VehSpeed > 90)	 //��������
			{
				VehSpeed = Myrand(65, 90);
			}
		}
		else
		{
			if (VehSpeed < 1 || VehSpeed > 90)	 //��������
			{
				VehSpeed = Myrand(50, 80);
			}			
		}
		if (VehHeight < 2300 || VehHeight > 4800)
		{
			VehHeight = Myrand(3100, 3450);
		}
	}
	else if (VehPattern <= JIZHUANGXIANG)	 //�ش��������װ��
	{
		if (VehLength < 12000 || VehLength > 35000)		 //��������
		{
			VehLength = Myrand(12500, 18000);
		}
		if (VehWide < 2000 || VehWide > 3500) //��������
		{
			VehWide = Myrand(2200, 3000);
		}
		if (VehHeight < 2600 || VehHeight > 5000)  //��������
		{
			if (VehPattern == TEDAHUO)
				VehHeight = Myrand(2800, 4500);
			else
				VehHeight = Myrand(3800, 4200);
		}
		if (g_sspSetup.u8RoadType)  //����
		{
			if (VehSpeed < 30 || VehSpeed > 90)	 //��������
			{
				VehSpeed = Myrand(55, 80);
			}
		}
		else
		{
			if (VehSpeed < 1 || VehSpeed > 90)	 //��������
			{
				VehSpeed = Myrand(50, 75);
			}			
		}
	}
	else if (VehPattern == MOTUOCHE)	  //Ħ�г�
	{
		if (VehLength >  2500 || VehLength < 500)		 //��������
		{
			VehLength = Myrand(1500, 2000);
		}
		if (VehWide < 500 || VehWide > 1500) //��������
		{
			VehWide = Myrand(700, 1000);
		}
		if (VehHeight < 800 || VehHeight > 1800)  //��������
		{
			VehHeight = Myrand(900, 1600);
		}
		if (VehSpeed < 1 || VehSpeed > 70)	 //��������
		{
			VehSpeed = Myrand(40, 60);
		}		
	}
	else 								//������
	{
		if (VehLength < 5000 || VehLength > 7000)		 //��������
		{
			VehLength = Myrand(5000, 6000);
		}
		if (VehWide < 1400 || VehWide > 3500) //��������
		{
			VehWide = Myrand(1700, 3000);
		}
		if (VehHeight < 800 || VehHeight > 4000)  //��������
		{
			VehHeight = Myrand(1700, 3000);
		}
		if (VehSpeed < 1 || VehSpeed > 60)	 //��������
		{
			VehSpeed = Myrand(20, 40);
		}		
	}
	************************************************/
//	if(VehPattern == MOTUOCHE&&g_sspSetup.u8RoadType)
//	{
//	   VehPattern = ZHONGXIAOKE;
//	}
//	if(VehPattern == TUOLAJI&&g_sspSetup.u8RoadType)
//	{
//	   VehPattern = XIAOHUOCHE;	  	
//	}
//
//	pVehicle->u8VehPattern = VehPattern;
///////////////////////////////////////////////////////////////////////////////////////////
//	 Veh_Info[0] = VehLength;	//����
//	 Veh_Info[1] = VehWide;		//���
//	 Veh_Info[2] = VehHeight;	///�߶�
//	 Veh_Info[3] = VehSpeed;	//�ٶ�
//	 Veh_Info[4] = VehPattern;	//����
//	 Veh_Info[5] = 0x12;		//��
//	 Veh_Info[6] = MONTH;		//��
//	 Veh_Info[7] = DAY;			//��
//	 Veh_Info[8] = HOUR;		//ʱ
//	 Veh_Info[9] = MIN;			//��
//	 Veh_Info[10] = SEC;		//��
//	 Veh_Info[11] = 0;//  ��������ͷʱ��
//	 Veh_Info[12] =	0;//  �Ƿ������
//     Veh_Info[13] =	0;//��������ͷ���
//	 Veh_Info[14] = 0;//ͨ����ֱ����ʱ��
//
//	switch(u8Lane)
//	{
//		case 1:
//			   Veh_Info[11] =  Get_shiju(Veh_Info,g_ai32Pre_Veh_Info_1_Lane);	 //��������ͷʱ��
//			   Veh_Info[11] =  (pVehicle->Vdata.tdata[0] - g_ai32Pre_Veh_Info_1_Lane[25])/1000;
//			   if(Veh_Info[11] < 0)
//			   {
//			    Veh_Info[11] = 20; 	 //��λ���룻
//			   }
//			   if(0==Veh_Info[11])
//			   {
//			   	Veh_Info[11] = 1;
//			   }
//			   if( Veh_Info[11] < DisTime )
//			   {
//			   	   Veh_Info[12] = 1;//1 ����
//			   }
//			   else
//			   {
//			   	   Veh_Info[12] = 0;//0 ������
//			   }		  					   	   
//			   Veh_Info[13] =	Veh_Info[11] * VehSpeed/3.6;//��������ͷ���
//			   if((pVehicle->Vdata.tdata[pVehicle->Vdata.u16FrameCnt-1] - pVehicle->Vdata.tdata[0]) < 1000)
//			   {
//			   	   Veh_Info[14] = 1;//��Ϊ1S��
//			   }
//			   else
//			   {
//			   		Veh_Info[14] = pVehicle->Vdata.tdata[pVehicle->Vdata.u16FrameCnt-1] - pVehicle->Vdata.tdata[0];//ͨ����ֱ����ʱ�� ms
//			   }
//			   memcpy(g_ai32Pre_Veh_Info_1_Lane,Veh_Info,sizeof(Veh_Info));
//			   g_ai32Pre_Veh_Info_1_Lane[25] = pVehicle->Vdata.tdata[0];
//			   break;
//		case 2:
//			  
//			   Veh_Info[11] =  Get_shiju(Veh_Info,g_ai32Pre_Veh_Info_2_Lane);
//			   Veh_Info[11] =  (pVehicle->Vdata.tdata[0] - g_ai32Pre_Veh_Info_2_Lane[25])/1000;
//			   if(Veh_Info[11] < 0)
//			   {
//			    Veh_Info[11] = 20; 	 //��λ���룻
//			   }
//			   if(0==Veh_Info[11])
//			   {
//			   	Veh_Info[11] = 1;
//			   }
//			   if( Veh_Info[11] < DisTime )
//			   {
//			   	   Veh_Info[12] = 1;//1 ����
//			   }
//			   else
//			   {
//			   	   Veh_Info[12] = 0;//0 ������
//			   } // Veh_Info[12]	//�ж��Ƿ������		  					   	   
//			   Veh_Info[13] =	Veh_Info[11] * VehSpeed/3.6;//��������ͷ���
//			   if((pVehicle->Vdata.tdata[pVehicle->Vdata.u16FrameCnt-1] - pVehicle->Vdata.tdata[0]) < 1000)
//			   {
//			   	   Veh_Info[14] = 1;//��Ϊ1S��
//			   }
//			   else
//			   {
//			   		Veh_Info[14] = pVehicle->Vdata.tdata[pVehicle->Vdata.u16FrameCnt-1] - pVehicle->Vdata.tdata[0];//ͨ����ֱ����ʱ�� ms
//			   }
//			   memcpy(g_ai32Pre_Veh_Info_2_Lane,Veh_Info,sizeof(Veh_Info));
//			   g_ai32Pre_Veh_Info_2_Lane[25] = pVehicle->Vdata.tdata[0];
//			   break;
//		case 3:
//			   Veh_Info[11] =  Get_shiju(Veh_Info,g_ai32Pre_Veh_Info_3_Lane);
//			   Veh_Info[11] =  (pVehicle->Vdata.tdata[0] - g_ai32Pre_Veh_Info_3_Lane[25])/1000;
//			   if(Veh_Info[11] < 0)
//			   {
//			    Veh_Info[11] = 20; 	 //��λ���룻
//			   }
//			   if(0==Veh_Info[11])
//			   {
//			   	Veh_Info[11] = 1;
//			   }
//			   if( Veh_Info[11] < DisTime )
//			   {
//			   	   Veh_Info[12] = 1;//1 ����
//			   }
//			   else
//			   {
//			   	   Veh_Info[12] = 0;//0 ������
//			   } // Veh_Info[12]	//�ж��Ƿ������		  					   	   
//			   Veh_Info[13] =	Veh_Info[11] * VehSpeed/3.6;//��������ͷ���
//			   if((pVehicle->Vdata.tdata[pVehicle->Vdata.u16FrameCnt-1] - pVehicle->Vdata.tdata[0]) < 1000)
//			   {
//			   	   Veh_Info[14] = 1;//��Ϊ1S��
//			   }
//			   else
//			   {
//			   		Veh_Info[14] = pVehicle->Vdata.tdata[pVehicle->Vdata.u16FrameCnt-1] - pVehicle->Vdata.tdata[0];//ͨ����ֱ����ʱ�� ms
//			   }
//			   memcpy(g_ai32Pre_Veh_Info_3_Lane,Veh_Info,sizeof(Veh_Info));
//			   g_ai32Pre_Veh_Info_3_Lane[25] = pVehicle->Vdata.tdata[0];
//			   break;
//		case 4:
//			   Veh_Info[11] =  Get_shiju(Veh_Info,g_ai32Pre_Veh_Info_4_Lane);	  //Pre_Veh_Info_4_Lane
//               Veh_Info[11] =  (pVehicle->Vdata.tdata[0] - g_ai32Pre_Veh_Info_4_Lane[25])/1000;
//			   if(Veh_Info[11] < 0)
//			   {
//			    Veh_Info[11] = 20; 	 //��λ���룻
//			   }
//			   if(0==Veh_Info[11])
//			   {
//			   	Veh_Info[11] = 1;
//			   }
//			   if( Veh_Info[11] < DisTime )
//			   {
//			   	   Veh_Info[12] = 1;//1 ����
//			   }
//			   else
//			   {
//			   	   Veh_Info[12] = 0;//0 ������
//			   } // Veh_Info[12]	//�ж��Ƿ������		  					   	   
//			   Veh_Info[13] =	Veh_Info[11] * VehSpeed/3.6;//��������ͷ���
//			   if((pVehicle->Vdata.tdata[pVehicle->Vdata.u16FrameCnt-1] - pVehicle->Vdata.tdata[0]) < 1000)
//			   {
//			   	   Veh_Info[14] = 1;//��Ϊ1S��
//			   }
//			   else
//			   {
//			   		Veh_Info[14] = pVehicle->Vdata.tdata[pVehicle->Vdata.u16FrameCnt-1] - pVehicle->Vdata.tdata[0];//ͨ����ֱ����ʱ�� ms
//			   }
//			   memcpy(g_ai32Pre_Veh_Info_4_Lane,Veh_Info,sizeof(Veh_Info));
//			   g_ai32Pre_Veh_Info_4_Lane[25] = pVehicle->Vdata.tdata[0];
//			   break;
//		case 5:
//			   Veh_Info[11] =  Get_shiju(Veh_Info,g_ai32Pre_Veh_Info_5_Lane);	  //Pre_Veh_Info_4_Lane
//               Veh_Info[11] =  (pVehicle->Vdata.tdata[0] - g_ai32Pre_Veh_Info_5_Lane[25])/1000;
//			   if(Veh_Info[11] < 0)
//			   {
//			    Veh_Info[11] = 20; 	 //��λ���룻
//			   }
//			   if(0==Veh_Info[11])
//			   {
//			   	Veh_Info[11] = 1;
//			   }
//			   if( Veh_Info[11] < DisTime )
//			   {
//			   	   Veh_Info[12] = 1;//1 ����
//			   }
//			   else
//			   {
//			   	   Veh_Info[12] = 0;//0 ������
//			   } // Veh_Info[12]	//�ж��Ƿ������		  					   	   
//			   Veh_Info[13] =	Veh_Info[11] * VehSpeed/3.6;//��������ͷ���
//			   if((pVehicle->Vdata.tdata[pVehicle->Vdata.u16FrameCnt-1] - pVehicle->Vdata.tdata[0]) < 1000)
//			   {
//			   	   Veh_Info[14] = 1;//��Ϊ1S��
//			   }
//			   else
//			   {
//			   		Veh_Info[14] = pVehicle->Vdata.tdata[pVehicle->Vdata.u16FrameCnt-1] - pVehicle->Vdata.tdata[0];//ͨ����ֱ����ʱ�� ms
//			   }
//			   memcpy(g_ai32Pre_Veh_Info_5_Lane,Veh_Info,sizeof(Veh_Info));
//			   g_ai32Pre_Veh_Info_5_Lane[25] = pVehicle->Vdata.tdata[0];
//			   break;
//	    case 6:
//			   Veh_Info[11] =  Get_shiju(Veh_Info,g_ai32Pre_Veh_Info_4_Lane);	  //Pre_Veh_Info_4_Lane
//               Veh_Info[11] =  (pVehicle->Vdata.tdata[0] - g_ai32Pre_Veh_Info_6_Lane[25])/1000;
//			   if(Veh_Info[11] < 0)
//			   {
//			    Veh_Info[11] = 20; 	 //��λ���룻
//			   }
//			   if(0==Veh_Info[11])
//			   {
//			   	Veh_Info[11] = 1;
//			   }
//			   if( Veh_Info[11] < DisTime )
//			   {
//			   	   Veh_Info[12] = 1;//1 ����
//			   }
//			   else
//			   {
//			   	   Veh_Info[12] = 0;//0 ������
//			   } // Veh_Info[12]	//�ж��Ƿ������		  					   	   
//			   Veh_Info[13] =	Veh_Info[11] * VehSpeed/3.6;//��������ͷ���
//			   if((pVehicle->Vdata.tdata[pVehicle->Vdata.u16FrameCnt-1] - pVehicle->Vdata.tdata[0]) < 1000)
//			   {
//			   	   Veh_Info[14] = 1;//��Ϊ1S��
//			   }
//			   else
//			   {
//			   		Veh_Info[14] = pVehicle->Vdata.tdata[pVehicle->Vdata.u16FrameCnt-1] - pVehicle->Vdata.tdata[0];//ͨ����ֱ����ʱ�� ms
//			   }
//			   memcpy(g_ai32Pre_Veh_Info_6_Lane,Veh_Info,sizeof(Veh_Info));
//			   g_ai32Pre_Veh_Info_6_Lane[25] = pVehicle->Vdata.tdata[0];
//			   break;
//		default:
//		       break;
//	}
////uint32 g_total_veh[9]; //С�ͳ�	С����	��ͳ�	���ͻ���	���ͻ���	�ش��ͻ���	��װ�䳵	������	Ħ�г�
////uint32 g_speed_veh_sum[9];
//	if(VehPattern>=1 && VehPattern<=9)
//	{	  //g_total_veh[4][9]:�ĳ������ų���
//		g_total_veh[u8Lane-1][VehPattern-1] =   g_total_veh[pVehicle->u8Lane-1][VehPattern-1] +  1;
//		g_speed_veh_sum[u8Lane-1][VehPattern-1] =   g_speed_veh_sum[u8Lane-1][VehPattern-1] + VehSpeed;	
//	}
//	/******��Ч�Լ���*******/
//	if(Veh_Info[11]<=0 || Veh_Info[11] >60)
//	{
//	   	Veh_Info[11] = 30;
//	}
//	if(Veh_Info[13]<=0 ||Veh_Info[13]>1500)
//	{
//	   Veh_Info[13] = 1000;	
//	}
//	if(Veh_Info[14]<=0 || Veh_Info[14] >25)
//	{
//	    Veh_Info[14] = 2;
//	}
//	/******��Ч�Լ���over*******/
//	if(u8Lane>=1 && u8Lane<=g_sspSetup.u8LaneNum)
//	{
//		 g_total_Lane[u8Lane-1] = g_total_Lane[u8Lane-1] +1;//�ĳ����ܳ���
//		 g_sum_shiju[u8Lane-1]  = g_sum_shiju[u8Lane-1]  +  Veh_Info[11];
//		 g_sum_shijian[u8Lane-1]= g_sum_shijian[u8Lane-1] + Veh_Info[14];
//		 g_sum_shijian_share[u8Lane-1] = 100*g_sum_shijian[u8Lane-1] / (ProCycle*60);	  //ʱ��ռ����//��ǰ����ʱ��ռ���ʣ�
//		 g_average_shiju[u8Lane-1] =    g_sum_shiju[u8Lane-1] / g_total_Lane[u8Lane-1];
//		 g_average_jianju[u8Lane-1] =  (g_average_jianju[u8Lane-1]*(g_total_Lane[u8Lane-1]-1) + Veh_Info[13])/g_total_Lane[u8Lane-1];
//	}
//	if(Veh_Info[12] == 1)
//	{
//		g_total_genche[u8Lane-1] = g_total_genche[u8Lane-1] + 1;	   //�ĳ��������ܳ���
//	}
//	 Veh_Info[15] = g_total_veh[u8Lane-1][VehPattern-1];//��ǰ����������
//	 Veh_Info[16] = g_speed_veh_sum[u8Lane-1][VehPattern-1];//��ǰ�����ٶȣ�
//	 Veh_Info[17] = g_average_shiju[u8Lane-1];//��ǰ����	��������ͷʱ��
//	 Veh_Info[18] = 100 * g_total_genche[u8Lane-1] / g_total_Lane[u8Lane-1];//�����ٷֱȣ�
//	 g_percent_genche[u8Lane-1] = 100 * g_total_genche[u8Lane-1] / g_total_Lane[u8Lane-1];//�����ٷֱȣ�
//	 Veh_Info[19] = g_average_jianju[u8Lane-1]; //��ǰ����	��������ͷ���
//	 Veh_Info[20] = 0;//ʱ��ռ����//��ǰ����ʱ��ռ���ʣ�
//	 Veh_Info[20] = g_sum_shijian_share[u8Lane-1];	  //ʱ��ռ����//��ǰ����ʱ��ռ���ʣ�
//	 Veh_Info[21] = u8Lane;//����
//	 Veh_Info[22] =	g_total_count_Veh;//�ܳ���
//
////��λ�������ʾ��
//	Send_VehInfo_Uart1[9]= VehSpeed;  //�ٶ�	  	
//	Send_VehInfo_Uart1[15]= b2bcd(YEAR_uint8);
//	Send_VehInfo_Uart1[16]= b2bcd(MONTH); //ת��ΪBCD��
//	Send_VehInfo_Uart1[17]= b2bcd(DAY);
//	Send_VehInfo_Uart1[18]= b2bcd(HOUR);
//	Send_VehInfo_Uart1[19]= b2bcd(MIN);
//	Send_VehInfo_Uart1[20]= b2bcd(SEC);
//	Send_VehInfo_Uart1[18]= g_u8TimeHour;     //20130416 likang ���Ͳ���ʱ��
//	Send_VehInfo_Uart1[19]= g_u8TimeMin;
//	Send_VehInfo_Uart1[20]= g_u8TimeSec;
//	Send_VehInfo_Uart1[21]= VehPattern;// ����
//	Send_VehInfo_Uart1[5]= VehLength>>24;
//	Send_VehInfo_Uart1[6]= VehLength>>16;
//	Send_VehInfo_Uart1[7]= VehLength>>8;
//	Send_VehInfo_Uart1[8]= VehLength;
//	Send_VehInfo_Uart1[41]=VehHeight>>8; //���
//	Send_VehInfo_Uart1[42]=VehHeight;
//	Send_VehInfo_Uart1[43]=VehWide>>8;//fuzhi
//	Send_VehInfo_Uart1[44]=VehWide;
//	Send_VehInfo_Uart1[37]=0x01;
//	Send_VehInfo_Uart1[38]=u8Lane;//����
//
//	crc_create(Send_VehInfo_Uart1,52);
//	OSSemPost(g_Uart1_send);
//	UART1_SendBuf(Send_VehInfo_Uart1,55);	   //TOTC  = 32
//	if((VehLength > limit_length) && (VehHeight > limit_height) && (VehWide > limit_width))
//	{
//		U5SendBytes(Send_VehInfo_Uart1,55);
//		sprintf(strsend,"����%d,%d\n",gv_index,send_count++);
//		U5SendBytes(strsend,strlen(strsend)+1);
//	}
//	 save_add = sv_write_sd_add;
//	 IF_SAME_DAY = (YEAR - 2013) * 372 + (MONTH - 1) * 31 + DAY;
//	if(gv_index == 1)
//	{
//		if(sv_count!=0)	   //����ʱ��Ĵ���ʽ ��һ����ʣ��Ĳ���24���������ݴ�SD����
//		{
//			save_add = save11_to_sd(sv_frame_data,21*sv_count);	
//		}
//	 	day_temp = (YEAR - 2013) * 372 + (MONTH - 1) * 31 + DAY;
//		Read256_full(SVWRITESDADD,(uint8 *)&sv_write_sd_add,4);
//		WriteSDtemp[0] = SD_BLOCK_HEAD[0];
//		WriteSDtemp[1] = SD_BLOCK_HEAD[1];
//		WriteSDtemp[2] = SD_BLOCK_HEAD[2];
//		WriteSDtemp[3] = SD_BLOCK_HEAD[3];
//		WriteSDtemp[4] = sv_write_sd_add&0xFF;
//		WriteSDtemp[5] = (sv_write_sd_add>>8)&0xFF;
//		WriteSDtemp[6] = (sv_write_sd_add>>16)&0xFF;
//		WriteSDtemp[7] = (sv_write_sd_add>>24)&0xFF;
//		full_write_sd(day_temp+SD_STAT1_ADD_START, WriteSDtemp);   //һ���ͷһ������SD����ʼ��ַ
//	}
//	gen_sv_fram(Send_VehInfo_Uart1);   //ͨ��������Ϣ���11֡
//	memcpy(&sv_frame_data[sv_count*21],sv_sd_frame,21);
//	sv_count = gv_index%24; 
//	if(sv_count==0)		//24��������Ϣ���ˣ�ͳһ���SD����һ������
//	{
//		save_add = save11_to_sd(sv_frame_data,504);		
//	}
//	if_send_flag = 0;
//	if((VehLength > limit_length) && (VehHeight > limit_height) && (VehWide > limit_width))	  	//����߳��ޣ�����־λ��1
//		if_send_flag = 1;
//	if(if_send_flag == 1)
//	{
//		if(g_u8Flag_wireless == 0)	//������������״̬����
//		{
//			UART1_SendBuf_full(frame11_buf,57);	//ͨ������1���͵�������
//			 VehSendInfo_tmp.u16Year = frame11_buf[22] + ((frame11_buf[23] & 0xFF) << 8);
//			 VehSendInfo_tmp.u8Month = frame11_buf[24];
//			 VehSendInfo_tmp.u8Day = frame11_buf[25];
//			 VehSendInfo_tmp.u8Hour = frame11_buf[26];
//			 VehSendInfo_tmp.u8Minute = frame11_buf[27];
//			 VehSendInfo_tmp.u8Second = frame11_buf[28];
//			 VehSendInfo_tmp.u32Veh_Index = gv_index;
//			if(CycleQue_Cnt_VehSendInfo == 0)	//���п�
//			{
//	//			break;
//			}
//			else
//			{
//				memcpy((unsigned char*)&VehSendInfo[Que_VehSendInfo[head_VehSendInfo]],(unsigned char*)&VehSendInfo_tmp,sizeof(_VehSendInfo));	
//				head_VehSendInfo= (head_VehSendInfo+1)%30;
//				CycleQue_Cnt_VehSendInfo--;
//			}	  
//		}
//		else
//		{
//			//��������SD����ַ����������
//			if(save_add != 0)
//			{
//				//���24���������ݻ�û�д�������û��д��SD������ôsave_add��ֵ������
//				Set_Que_Cycle_Continue(save_add,gv_index);
//			}
//		}
//	}	 // end if(if_send_flag == 1)
//	gv_index++;
//
//	wtmp[0]=gv_index&0xFF;
//	wtmp[1]=(gv_index>>8)&0xFF;
//	wtmp[2]=(gv_index>>16)&0xFF;
//	Write256_full(PARA_ADD+0x100+100+1,wtmp,3);
//	memcpy(SD_Buff_Send_VehInfo_Uart1[SD_store_count],Send_VehInfo_Uart1,55);
//
//	g_total_count_Veh = g_total_count_Veh + 1 ;
//  
}

int get_vehicle_speed(int m, int Speedline)
{
//	int i;
//	int MinD=0;
//	int index;
//
//	int	VehSpeed = 0;
//
//	int X1;
//	int X2;
//    int t1;
//	int t2;
//
//	int Distance=800;   //����80cm�����ڵ�ʱ��������ٶ�
// 
//	index=Lane_Vertical[m][0][0];
//	MinD=abs(Lane_Vertical[m][index-1][4]-Speedline);
//	X2=Lane_Vertical[m][index-1][4]; 
//	t2=Lane_Vertical[m][index-1][0];
//
//	for(i=index-2;i>1;i--)
//	{
//		if(abs(Lane_Vertical[m][i][4]-Speedline)<MinD)
//		{
//			MinD=abs(Lane_Vertical[m][i][4]-Speedline);
//			X2=Lane_Vertical[m][i][4];
//			t2=Lane_Vertical[m][i][0];
//			index=i;
//		}
//	}
//		
//	MinD=Distance;
//	X1=Lane_Vertical[m][index][4];
//	t1=Lane_Vertical[m][index][0];
//	
//	if(index>2)
//	{
//		for(i=index-1;i>1;i--)	   //�Ҿ��루EnterX2-Distance������ĵ�
//		{
//			if(abs(Lane_Vertical[m][i][4]-(Speedline-Distance))<MinD)
//			{
//				MinD=abs(Lane_Vertical[m][i][4]-(Speedline-Distance));
//				X1=Lane_Vertical[m][i][4];
//				t1=Lane_Vertical[m][i][0];  	
//			}
//	 	}
//	}
//	VehSpeed=(((X2-X1)*3.6)/(t2-t1))*1000;
//	return VehSpeed;
    return 0;
}

int get_vehicle_length(int m)
{
//	int index = 0;
//	int VehLength = 0;
//	index=Lane_Vertical[m][0][0];
//    VehLength=(Lane_Vertical[m][index-1][4]-Lane_Vertical[m][index-1][3]);  //��������
//	return VehLength;
    return 0;
}
