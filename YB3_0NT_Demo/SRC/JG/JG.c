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
//单车信息结构体数组维护队列   定义在SDRAM中，断电丢失
uint32 tail_VehSendInfo = 0;
uint32 head_VehSendInfo = 0;
uint32 CycleQue_Cnt_VehSendInfo = 30;	  
unsigned char Que_VehSendInfo[30] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29};	//初始化
uint32 head_continue,tail_continue;	  //续传队列	存铁电中
#define    ERRORVALUE       0xFFFF
#define    MAXDAFEIPTNUM    4

#define    MINWIDE_THRESHOLD     100   //当前车辆区域小于该值，不更新匹配区域
#define		POINTNUM	360
#define		TOTALCAR	10			//20140213

#define		VEH_HEAD_RES	2600	  //车头限
#define		VEH_BODY_H_RES	3200	  //车身高度限值
	 

/**************抛车用变量---待删减************************/
							   //四车道分开；
uint32 g_total_veh[6][9]={0}; //九车型总数；小客车	小货车	大客车	中型货车	大型货车	特大型货车	集装箱车	拖拉机	摩托车
uint32 g_total_veh_temp[6][9]={0}; //G4协议用于车辆缓存
uint32 g_speed_veh_sum[6][9]={0}; //九车型速度(统计周期内累加)；小客车	小货车	大客车	中型货车	大型货车	特大型货车	集装箱车	拖拉机	摩托车
uint32 g_total_Lane[6]={0};// 四车道各自车辆总数；
uint32 g_average_shiju[6]={0}; //机动车车头时距数据
uint32 g_sum_shiju[6] = {0};   //机动车车头时距sum ；
uint32 g_sum_shijian_share[6] = {0};   //四车道时间占有率；
uint32 g_sum_shijian[6] = {0};		//四车道车辆占有时间总和；
uint32 g_total_genche[6]={0};//跟车总数；
uint32 g_percent_genche[6]={0};//跟车百分比；
uint32 g_average_jianju[6]={0}; //平均机动车车头间距


int32  g_ai32Pre_Veh_Info_1_Lane[26]={0};  //保存1车道前一辆车的信息；最后一位保存前一辆车的车头时间
int32  g_ai32Pre_Veh_Info_2_Lane[26]={0};  //保存2车道前一辆车的信息；
int32  g_ai32Pre_Veh_Info_3_Lane[26]={0};  //保存3车道前一辆车的信息；
int32  g_ai32Pre_Veh_Info_4_Lane[26]={0};  //保存4车道前一辆车的信息；
int32  g_ai32Pre_Veh_Info_5_Lane[26]={0};  //保存5车道前一辆车的信息；
int32  g_ai32Pre_Veh_Info_6_Lane[26]={0};  //保存6车道前一辆车的信息；


int32 Veh_Info[25]={0};   //当前车辆的信息


int32 ManualforWide = 0;
int32 ManualforHeight = 0;


/******************提取出的车辆信息****************************/
uint8  VehMod=0;
uint16 VehLength=0;	           //车长
uint16 VehSpeed=0;             //车速
uint8  VehTime=0;              //时间:年月日、时分秒

uint16 VehHeight=0;            //车高
uint8  VehHeadFlag=0;          //是否区分出车头，	1：区分出车头；  0：没有区分出车头
uint16 VehHeadLength=0;        //车头长度
uint8  VehTopPlaneness=0;      //车顶部平整度		1：平整； 0不平整
uint8  VehBackPlaneness=0;     //车后部平整度		1：平整； 0不平整




//车辆长度计算变量
int Length_S1=0;
int Length_S2=0;
int StartTime=0;
int EndTime=0;
int LengthFlag1=0;
int LengthFlag2=0;



/***************************************************************/



uint8 VehInfo_Buffer[19];	 //用于向上位机发送车辆速度

uint8 DivideSigBuffer[8];	 //用于发送分车信号
extern uint16  g_u16DataFrame;
extern uint16  g_u16QufaFrame;
extern uint8 send_flag;
extern uint8 send_count;
uint8   l_u8VehHeight[20];
uint8 g_data=0;
uint8 g_match = 1;			 //20140404


/*定义全局变量*/ 
#define   ANGLEDEVIATION_V     18      //垂直方向角度偏差9度
#define   ANGLEDEVIATION_I    	20       //倾斜方向角度偏差7度
#define   SMALL_CAR            600

uint8  g_u8TimeHour;     //波形时间数据 20130416 
uint8  g_u8TimeMin;
uint8  g_u8TimeSec;


uint16  g_u16InclineStartAnglePt  = 0;   //倾斜激光器的起始点数
uint16  g_u16InclineEndAnglePt    = 0;   //倾斜激光器的终止点数


int32  PastPoint[10]={0};         //采用分时分车算法中记录历史分车点


int32   ThresVehLengthLow=1000;     // 车辆长度误差下限，用于识别车辆



#define ERRPTTHRESHOLD       100  //   20130510 打飞的点的阈值

/***********命令号***************/
#define FENCHEFLAG      0x22	  //分车命令号
#define VEHINFOR	   0x10       //命令号：向上位机发送车辆信息


/*时间测试*********************************/
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

#define IS_INSIDE(x,y,dx,dy)	( (min(dx,dy) >= max(x,y) || max(dx,dy) <= min(x,y)) ? 0:1 )   //x,y在dx,dy一侧返回0


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

	//20140217 参数判断
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
{ //采用自下向上扫描
	int i,j;
	int n; 
	uint8 exchange; //交换标志
	int temp;
	n = num; 
	for(i=0;i<n;i++)
	{ //最多做n-1趟排序 
		exchange=0; //本趟排序开始前，交换标志应为假 
		for(j=n-2;j>=i;j--)
		{
			if(parr[j+1]<parr[j])
			{//交换记录 
				temp = parr[j+1]; 
				parr[j+1] = parr[j]; 
				parr[j] = temp; 
				exchange = 1; //发生了交换，故将交换标志置为真 
			}
		} 
		if(!exchange) //本趟排序未发生交换，提前终止算法 
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
				   g_VehicleSet[i].u8Vstate = NO_USED;  //误触发的车 
				   for(l_u16tmp = j;l_u16tmp < g_totalVehicle - 1;l_u16tmp++)
					   g_VehicleSetIndex[l_u16tmp] = g_VehicleSetIndex[l_u16tmp+1];
				   
				   g_VehicleSetIndex[g_totalVehicle - 1] = 0;
				   g_totalVehicle--;
				}
		   }
	}
   }

/**************待编写********************/
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

	Tx_Buffer[83] = ((len)>>8)& 0xFF;	   //取低8位
	Tx_Buffer[84] = (len) & 0xFF;	       //扫描点个数*2，即2个点表示一个距离，取高8位	
	Index =85;
	for(i=0; i<len; i++)
	{
		 Tx_Buffer[Index++] = (data[i]>>8)& 0xFF;
		 Tx_Buffer[Index++] = data[i] & 0xFF; 
	}

  	SendDataNet(3,Tx_Buffer,(len<<1)+85);
}
//对原始数据进行预处理 针对打飞的点用0值替代的处理 20130422
void YuChuLiData_Zero(void)
{
//	//只处理角度范围内的数据
//	uint16  index = 0;
//	uint16  u16ValidData = 800;  //在修正错误点值时给的固定修正值，表示该点有车
//	uint16  u16TmpIndex = 0;
//	uint16  u16Tmp = 0;
//	uint16  u16TmpHeight;    //高度差(基准值与当前点值的差）
//	uint8   u8TmpFlag = 0;   //要改变0值的标识，0表示不改变，1表示改变
//
//	//垂直激光器处理
//	index = g_u16VerticalStartAnglePt-1;
//	u16TmpIndex = index + 1;
//	while(index < g_u16VerticalEndAnglePt-2)
//	{  //20130506  所有0改为10 因为SICK激光器打飞的点值是3
//		u16TmpHeight = 	(g_Base_data0_Value[index-1] - LMS_data_1[index-1] > u16ValidData) ?  (g_Base_data0_Value[index-1] - LMS_data_1[index-1]) : u16ValidData;
//		if ( g_Base_data0_Value[index]-LMS_data_1[index] > ThresVehLow && LMS_data_1[index] <=ERRPTTHRESHOLD )	//原始数据中打飞的点为0值
//		{
//			if (g_Base_data0_Value[index-1]-LMS_data_1[index-1] > ThresVehLow && LMS_data_1[index-1] > ERRPTTHRESHOLD)	 //前一点值正常且高度差大于350
//			{
//				u16TmpHeight = 	(g_Base_data0_Value[index-1] - LMS_data_1[index-1] > u16ValidData) ?  (g_Base_data0_Value[index-1] - LMS_data_1[index-1]) : u16ValidData;
//				LMS_data_1[index] =  g_Base_data0_Value[index-1] - u16TmpHeight;
//				u16TmpIndex = index;
//			}
//			else if (g_Base_data0_Value[index+1]-LMS_data_1[index+1] > ThresVehLow && LMS_data_1[index+1] > ERRPTTHRESHOLD)  //后一点值正常且高度差大于350
//			{
//				u16TmpHeight = 	(g_Base_data0_Value[index+1] - LMS_data_1[index+1] > u16ValidData) ?  (g_Base_data0_Value[index+1] - LMS_data_1[index+1]) : u16ValidData;
//				for (u16Tmp = u16TmpIndex; u16Tmp <= index; u16Tmp++)
//				{
//					LMS_data_1[u16Tmp] =  g_Base_data0_Value[u16Tmp] - u16TmpHeight;
//				}
//				u16TmpIndex = index;
//				u8TmpFlag = 0;	
//			}
//			else if (g_Base_data0_Value[index+1]-LMS_data_1[index+1] <= ThresVehLow && LMS_data_1[index+1] > ERRPTTHRESHOLD)	 //后一点是正常点高度差小于350
//			{
//			    //20130508  针对唐山没有车时有4个点打飞处理
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
//						LMS_data_1[u16Tmp] =  g_Base_data0_Value[u16Tmp] - u16ValidData;	   //赋值为默认高度差
//					}
//				}		
//				u16TmpIndex = index;
//				u8TmpFlag = 0;
//			}
//			else if (g_Base_data0_Value[index+1]-LMS_data_1[index+1] > ThresVehLow && LMS_data_1[index+1] <= ERRPTTHRESHOLD && (!u8TmpFlag))//后一点也是打飞的点
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
** 函数名称:  GetVehHeight2
** 函数功能:  做二次平均作为车辆高度
** 入口参数:  Z坐标数组指针，开始位置，结束位置
** 出口参数:  高度平均值
** 函数说明:
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

	//先检查是否有异常点 最大检查2个点异常
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

	if (ThdHeight && SecHeight > ThdHeight + 600 && SecHeight > 2500)  //2个点异常
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
		    //重新找高度
			if (NewHeight < pg_ZdistanceI[Tmpi])
			{
				NewHeight = pg_ZdistanceI[Tmpi];
			}					
		}
	}
	else if (SecHeight && MaxHeight > SecHeight + 600 && MaxHeight > 2500) //有一个点异常高度
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
		    //重新找高度
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
			if (NewHeight - pg_ZdistanceI[Tmpi] < 200 && Tmpj <5)	//最多5个点	  将500改为200
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
		RetHeight = RetHeight/Tmpj;    //计算出的第2次均值
	}

	return  RetHeight;	
	
	  
}

/************************************/
 //计算每帧数据的二次均值作为该帧的车高
int GetVehHeight(VehicleDataStruct *pdata, uint16 u16FrameNum)
{
	int    Tmpi = 0;
	int    Tmpj = 0;
	int    RetHeight = 0;
	int    ThdHeight = 0;
	int    SecHeight = 0;
	int    MaxHeight = 0;
	int    NewHeight = 0;
	uint8  l_u8AfreshFlag = 0;   //重新计算第一次均值的标识，1表示重新计算
	uint8  l_u8AfreshPtNum = 0;   //重新计算第一次均值的点数
	uint8  u8PtNum = 0;
	u8PtNum = pdata->zdata[u16FrameNum][0];   //点数
	if (u8PtNum == 0)
	{
		return 0;
	}
	if (u8PtNum == 1)
	{
		RetHeight = pdata->zMax[u16FrameNum];
		return RetHeight;
	}

	//先检查是否有异常点 最大检查2个点异常
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

	if (ThdHeight && SecHeight > ThdHeight + 600 && SecHeight > 2500)  //2个点异常
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
		    //重新找高度
			if (NewHeight < pdata->zdata[u16FrameNum][Tmpi])
				NewHeight = pdata->zdata[u16FrameNum][Tmpi];	
		}
	}
	else if (SecHeight && MaxHeight > SecHeight + 600 && MaxHeight > 2500) //有一个点异常高度
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
		    //重新找高度
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
			if (pdata->zMax[u16FrameNum] - pdata->zdata[u16FrameNum][Tmpi] < 200 && Tmpj <5)	//最多5个点	  将500改为200
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
		RetHeight = RetHeight/Tmpj;    //计算出的第2次均值
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
** 函数名称:  GetVehLength
** 函数功能:  求车辆的长度
** 入口参数:  车辆编号k；求平均的个数；
** 出口参数:  车辆长度平均值
** 函数说明:  认为当车辆行驶到中间位置时，测得的车辆长度最准确，取中间的一定的帧数做平均作为车辆的长度
*********************************************************************************************************/
int GetVehLength(int k,uint8 AveNum)
{

//int Tmpi=0;
//int RealAveNum=0;
//int MidNum_Flag=0;
//int CarLength=0;
//
//int MidNum=0;
//int MinXSum=abs(Lane_Vertical[k][1][3]+Lane_Vertical[k][1][4]); //将第一帧车头X和车尾X坐标求和的绝对值赋给MinXSum
//
//  
///************************************在数组中找到最小的车头和车尾X坐标求和的绝对值***************************/
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
///******************求车辆行驶到中间时车辆的长度平均值*****************************/ 
//if(MidNum_Flag)
// {
//  if(MidNum>(AveNum/2))
//    {
//	if((Lane_Vertical[k][0][0]-MidNum)>=(AveNum/2))		     //前后都有AveNum/2个数
//	 {
//	 for(Tmpi=MidNum-(AveNum/2);Tmpi<=MidNum+(AveNum/2)-1;Tmpi++)
//	   {
//	   CarLength=CarLength+(Lane_Vertical[k][Tmpi][4]-Lane_Vertical[k][Tmpi][3]);		//车头X坐标减去车尾X坐标
//	   RealAveNum++;
//	   }
//	 }
//	else								                   //前有AveNum/2个数；后无AveNum/2个数
//	  {
//	  for(Tmpi=MidNum-(AveNum/2);Tmpi<=Lane_Vertical[k][0][0]-1;Tmpi++)
//	   {
//	   CarLength=CarLength+(Lane_Vertical[k][Tmpi][4]-Lane_Vertical[k][Tmpi][3]);		//车头X坐标减去车尾X坐标
//	   RealAveNum++;
//	   }
//	 
//	  }
//	}
//
//  else 																		  
//        if((Lane_Vertical[k][0][0]-MidNum)>=(AveNum/2))	    //前无AveNum/2个数；后有AveNum/2个数
//          {
//		  for(Tmpi=1;Tmpi<=MidNum+(AveNum/2)-1;Tmpi++)
//	        {
//	        
//	        }
//		  }
//		else	                                            //前无AveNum/2个数；后有AveNum/2个数
//		   {
//		    for(Tmpi=1;Tmpi<=Lane_Vertical[k][0][0]-1;Tmpi++)
//	        {
//	        CarLength=CarLength+(Lane_Vertical[k][Tmpi][4]-Lane_Vertical[k][Tmpi][3]);		//车头X坐标减去车尾X坐标
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

 /***********************提取车辆长度数据结束************************************/
//u8StarttEndFlag 0表示起始点，1表示结束点；u8VIFlag 0表示垂直，1表示倾斜
uint16 GetStartEndPt(const int* const pdata, const uint16 startPt, const uint8 u8StartEndFlag, const uint8 u8JGIndx)  
{
	uint16 Ret = startPt;
	uint16 index = 0;
	uint16 minHeight = 4000;   //在寻找起始点、结束点时的最小高度
	uint16 u16TmpZeroPos = 180;
	uint16 u16StartPtNum=0;
	uint16 u16EndPtNum=0;
	uint16 u8InstallFlag=0;

	//增加对参数的判断
	if (pdata == NULL || startPt >= POINT_SUM)  //为空 返回错误值  20140214
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

	if (u8InstallFlag)  //正装方式
	{
		if (startPt <= u16TmpZeroPos)	 //起始点	小于零点索引的点
		{
			for (index = startPt; index < u16TmpZeroPos; index++)	
			{
				if (pdata[index] > minHeight)	
				{
					Ret = index;
					break;
				}
			}
			if(index>= u16TmpZeroPos)	   //起始点没找到	  20140217 修改
			{
				Ret = u16StartPtNum;
			}
		}
		else
		{
			for (index = startPt; index > u16TmpZeroPos; index--)	 //终止点   
			{
				if (pdata[index] > minHeight)
				{
					Ret = index;
					break;
				}
			}
			if(index <= u16TmpZeroPos)	   //起始点没找到	  20140217 修改
			{
				Ret = u16EndPtNum;
			}
		}
	}
	else  //侧装方式
	{
		//侧装时，起始点和结束点均在零点的同侧	且起始点总小于结束点
		if (u8StartEndFlag)	  //寻找结束点
		{
			for (index = startPt; index > u16StartPtNum; index--)	 
			{
				if (pdata[index] > minHeight)
				{
					Ret = index;
					break;
				}
			}
			if(index <= u16StartPtNum)	   //起始点没找到	 20140217 修改
			{
				Ret = u16EndPtNum;
			}
		}
		else  //寻找开始点
		{
			for (index = startPt; index < u16EndPtNum; index++)  
			{
				if (pdata[index] > minHeight)
				{
					Ret = index;
					break;
				}
			}
			if(index >= u16EndPtNum)	   //起始点没找到	  20140217 修改
			{
				Ret = u16StartPtNum;
			}
		}
	}
	return Ret;
}
/*******************计算区分大型车（主要是大客车3，中型货车4,大型货车5，特大型货车6，集装箱7)********************/

uint8 GetLargeVehPattern(VehicleStruct *pVeh)
{
//	VehicleStruct *pVehicle = pVeh;
//	uint8   l_u8DafeiFrm    = 0;  //打飞帧标志 ，1表示该帧判断为打飞的， 0正常
//	uint8   RetPattern       = 0;
//	uint8   i,j,k;
//	uint8   l_u8DaFeiFrameCnt = 0;  //记录车辆车中打飞的帧数
//	uint8   l_u8DaFeiFrameCnt2 = 0;
//	uint8   l_u8EqualNum      = 0;   //记录每帧中有多少个相当的高度值，用于打飞帧的判断
//	uint8   l_u8TmpVehNum     = 0;
//	uint8   l_u8QianShiJing   = 0;   //打在前视镜上的帧标志， 1表示要剔除该帧
//	uint8   l_u8DownThresVehLow = 0;  //记录阈值以下的点的个数
//
//	uint8   Toupos           = 0;    //车头与车身分界位置
//	uint8   Shenpos          = 0;    //车身开始位置（主要用于集装箱）
//	uint8   CheDingStartPos  = 0;    //车顶开始位置
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
//	uint8  dakeche     = 0;  //大客车标识，1表示是大客车
//	uint8  Not_keche = 0;  //不是大客车标识 ，1表示不是
//	uint8  u8Lane      = 0;	//车道
//
//	uint8  l_u8ChetouDafeiPt = 0;
//
//	uint8  jizhuangxiang = 0;
//	uint8  jizhuangxiang_count = 0;
//	uint8  jizhuangxiang_diffcnt = 0;   //记录判断集装箱时相邻帧的高度差个数
//	uint8  u16Index            = 0;
//	uint8  tmpFlag     = 0;
//	uint16 l_u16HeightThresh = 0;
//	uint16 tmp1   = 0;
//	uint16 tmp2   = 0;
//	uint16  l_u8Pos  = 0;
//	uint32	TouHeight =0;	  //厢式货车车头高
//	uint32	TouWide =0;		  //厢式货车车头宽
//	uint32	TouGao =0;		  //渣土大货车车头高
//	uint32	TouKuan =0;		  //渣土大货车车头宽
//	uint8   l_u8HighCount = 0;
//	uint8   u8Flag          = 1;
//	uint8   l_u8StartFrame  = 0;	
// 	int32   ENDRatio=750;
//	int32   l_n32DakeHeightThr = 4000;   //大客车高度阈值						 
//	int32   l_n32AllSDThr    = 500;     //全身方差阈值
//	int32   l_n32MultiSDThr  = 720;   //多点车身方差阈值
//	int32   allSDcha         =0;    //全身方差
//	int32   MultiCheshenSD   =0;    //多点车身方差
//	int32   SigleCheshenSD=0;
//    int     l_nDaKeHeightThr = 3950;
//	int     l_nDaKeThreshhold11 = 500; 
//    int     l_nDaKeThreshhold12 = 700;
//	int     l_nJZXThreshhold11 = 150;  //将50改为150   与计算的单点车身方差比较
//	int     l_nJZXThreshhold12 = 245;   //将50改为245   与计算的多点车身方差比较
//	int     l_nDaKeThreshhold21 = 400;
//	int     l_nDaKeThreshhold22 = 500;
//	int     DakecheFrameCnt = 12;
//
//	static int32 Z[FRAME_MAXCNT][FRAME_BUFLEN]={0};	 //
//    static int32 Z3[FRAME_MAXCNT][40]={0};
//    static int32 Heightfangcha[FRAME_MAXCNT]={0}; 
//	static int32 duodianduicha[FRAME_MAXCNT]  ={0};	   //计算多点方差的中间值
//	static int32 Height[FRAME_MAXCNT] = {0};   //用于计算时使用的车高
//	int32   tmpHeight        = 0;
//	int32   Widefangcha      = 0;   //用于修正大客车的车宽方差
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
//	//20140217  增加对参数判断
//	if (pVeh == NULL)
//	{
//		return ZHONGXIAOKE;
//	}
//
//	Veh_Num          = pVehicle->Vdata.u16FrameCnt;
//	if (Veh_Num > FRAME_MAXCNT || Veh_Num < 1)
//	{
//		Veh_Num = 0;  //帧数超过最大帧数值，帧数赋值为0
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
//	//先剔除那些有较多打飞点的帧
//	for (i = 0; i < Veh_Num; i++)
//	{	
//		for (j = 1; j <= pVehicle->Vdata.zdata[i][0]; j++)  //erro3   2.24 发生死循环,已修改
//		{
//			//剔除有前视镜对车型计算的影响
//			if (((pVehicle->Vdata.zdata[i][0] <= 6) && (pVehicle->Vdata.zMax[i] > 2000) && (i < 4))
//				|| (pVehicle->Vdata.zMax[i] < 1200 && i < 4)) //或者高度小于1200
//			{
//				//只针对前4帧，点数小于6，高度较高,直接剔除该帧
//				l_u8QianShiJing = 1;
//				break;
//			} 
//			else if ((pVehicle->Vdata.zdata[i][j] <= ThresVehLow) &&
//			         (pVehicle->Vdata.zMax[i] > 2000) && (i < 4))  // 只针对前4帧，高度较高，记录点高度在阈值以下的个数
//			{
//				l_u8DownThresVehLow++;
//			}
//			else
//			{
//			}
//			//寻找打飞点
//			if (((pVehicle->Vdata.zdata[i][j] == pVehicle->Vdata.zdata[i][j+1]) ||  //前后2个点相等
//				(l_u8EqualNum>=3 && (0==pVehicle->Vdata.zdata[i][j+1])) 
//				||(l_u8EqualNum>=3 && (pVehicle->Vdata.zdata[i][j]!=pVehicle->Vdata.zdata[i][j+1])&& (pVehicle->Vdata.zdata[i][j+2]==pVehicle->Vdata.zdata[i][j+1]))
//				||(l_u8EqualNum<1&&(pVehicle->Vdata.zdata[i][j] == 0 && pVehicle->Vdata.zdata[i][j+1] !=0))) && (j <pVehicle->Vdata.zdata[i][0])) //20140320或者有相等的点存在，且接着有高度是0值的
//			{
//				if (i < Veh_Num/4 && pVehicle->Vdata.zdata[i][j] >= pVehicle->Vdata.zMax[i])	//计算客车车头打飞且是该帧的最大高度处
//				{
//					l_u8ChetouDafeiPt++;			
//				}
//				l_u8EqualNum++;
//			}
//			else
//			{
//				if ((pVehicle->Vdata.zdata[i][0] > 15 && l_u8EqualNum+1 >= pVehicle->Vdata.zdata[i][0]/3) ||
//					(pVehicle->Vdata.zdata[i][0] <=15 && l_u8EqualNum>=4)  //	15个点以下 有超过4个点就认为打飞
//					) //超过8个点直接认为该帧数据是打飞	  || (l_u8EqualNum >= 8)
//				{ //认为该帧可以丢弃,直接跳出
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
//		{   //该帧小于阈值的点数超过5
//			l_u8QianShiJing = 1;
//		}
//
//		if ((l_u8QianShiJing) || (l_u8DafeiFrm)||l_u8ChetouDafeiPt)//认为该帧可以丢弃
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
//	if (Veh_Num != l_u8TmpVehNum)  //不相等，表明剔除了帧  重新找高
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
//	//剔除完毕
//
// //计算车型
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
//    //为了防止大客车打飞的帧数很多增加直接判断大客车的方法
// 	if (((l_u32Height2>0 && l_u32Height1>0 && abs(l_u32Height2-l_u32Height1)<800)|| (0 == l_u32Height2 && 0 == l_u32Height1))
//		&& Veh_Num < pVehicle->Vdata.u16FrameCnt/2 && pVehicle->Vdata.u16FrameCnt > 12 && pVehicle->zLen > 2500 )
//	{
//		RetPattern = DAKECHE;
//	}
//	else if (Veh_Num <=5 && pVehicle->zLen > 2700)
//	{
//		RetPattern = ZHONGXIAOKE;
//	}
// 	else if (Veh_Num <= 5) //5帧或以下直接是中小客车
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
//				        Z3[i][k]= Z[i][j];//将每帧中后三分之一点存在Z3中，0位置存车顶最后一个点,异常点去掉
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
//				        Z3[i][k]= Z[i][j];//将每帧中后三分之一点存在Z3中，0位置存车顶最后一个点,异常点去掉
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
//	   //计算车头的高度均值
//	   l_u8CheTouStart = 0;
//	   for ( i = 1; i < Veh_Num/3; i++)
//	   {
//	   		if (abs(Height[i]-Height[i-1])< 100 && abs(Height[i]-Height[i+1])< 100 && Height[i-1] > 1800)
//			{
//				l_u8CheTouStart = i-1;	  //车头起始点
//				break;
//			}
//	   }
//	   for ( i = l_u8CheTouStart; i < Veh_Num/3; i++)	  //寻找车头的结束点
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
//	   //计算车头的均值高度
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
//				l_nCheTouHeight = l_nCheTouHeight / (i - l_u8CheTouStart);     //当i>10时，用l_u8CheTouEnd算的高度不准确
//			}
//	   }
//
//	   //寻找车顶开始的位置
//	   for (i = 1; i < Veh_Num/2; i++)		  //加上i<    20131216
//	   {
//	   		if (abs(Height[i]-Height[i-1]) < 100 &&	abs(Height[i]-Height[i+1]) < 100 && Height[i-1] > 2000)
//			{
//				StartDing = i-1;
//				break;
//			}
//	   }
//	   if (g_sspSetup.u8RoadType && StartDing < 2)	 //高速路上小于2的从2开始
//	   {
//	   		StartDing = 2;
//	   }
//
//	   //寻找车头与车身的分界点
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
//			TouHeight =  TouHeight/Toupos;			   //计算车头高和车头宽，为识别厢式货车 
//			TouWide = TouWide/Toupos;		
//			for(i=Toupos;i<Veh_Num/2;i++)
//			{
//				if (abs(Height[i]-Height[i-1])<200 && abs(Height[i]-Height[i+1])<200&& Height[i-1]>3000)
//				{
//					Shenpos=i-1;	 //求车身位置，主要是为集装箱服务
//					break;
//				}
//			}
//		}
//
//		//计算车身的单点方差
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
//		//计算多点车身方差
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
//		//计算全身的方差
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
//	    //计算前三分之一宽度、高度平均，后三分之二宽度高度平均，以及整体平均。
//		//前三分之一宽度高度平均
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
//			    if(l_u32index <= 2 && Height[l_u32index] > 700)	  //ljj修改 谢理@20130809
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
//		//后三分之二宽度高度平均
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
//		//整体宽度高度平均
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
//		//车身宽度方差	 用于大客修正
//		l_u32index2 = 0;
//		Widefangcha = 0;
//		if (g_sspSetup.u8RoadType) //高速路，车速较快
//		{
//			if (pVehicle->xLen >= 2000)//车宽在2000或以上
//			{
//				for (l_u32index = 2; l_u32index < Veh_Num /2 ; l_u32index ++)
//				{	
//					if( pVehicle->Vdata.xMax[l_u32index] < 2800)	  //打飞情况下车宽比较大
//					{
//					   l_u32index2++ ;
//					   Widefangcha += (pVehicle->Vdata.xMax[l_u32index]-l_u32Wide2)*(pVehicle->Vdata.xMax[l_u32index]-l_u32Wide2);
//					}
//					
//				}
//			}
//		}
//		else   //普通路
//		{
//
//			for (l_u32index = 2; l_u32index < Veh_Num /2 ; l_u32index ++)
//			{	
//				if( pVehicle->Vdata.xMax[l_u32index] < 2800)	  //打飞情况下车宽比较大
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
////找车窗侧面打飞点来确定大客车
////找帧的结束		
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
//		else		//尾帧中点数过少 20140809
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
//								if (((pVehicle->Vdata.zdata[i][j] == pVehicle->Vdata.zdata[i][j-1])   //前后2个点相等 
//									||(l_u8EqualNum>=1 && (pVehicle->Vdata.zdata[i][j]!=pVehicle->Vdata.zdata[i][j-1])&& (pVehicle->Vdata.zdata[i][j-2]==pVehicle->Vdata.zdata[i][j-1]))
//									||(l_u8EqualNum<1&&(pVehicle->Vdata.zdata[i][j] == 0 && pVehicle->Vdata.zdata[i][j-1] !=0))) && (j <pVehicle->Vdata.zdata[i][0]-1)) //20140320或者有相等的点存在，且接着有高度是0值的
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
//								if (((pVehicle->Vdata.zdata[i][j] == pVehicle->Vdata.zdata[i][j+1])   //前后2个点相等 
//									||(l_u8EqualNum>=1 && (pVehicle->Vdata.zdata[i][j]!=pVehicle->Vdata.zdata[i][j+1])&& (pVehicle->Vdata.zdata[i][j+2]==pVehicle->Vdata.zdata[i][j+1]))
//									||(l_u8EqualNum<1&&(pVehicle->Vdata.zdata[i][j] == 0 && pVehicle->Vdata.zdata[i][j+1] !=0))) && (j <pVehicle->Vdata.zdata[i][0]-1)) //20140320或者有相等的点存在，且接着有高度是0值的
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
//		//车型划分
//		if (VehHeight < 1600 && pVehicle->Vdata.u16FrameCnt <= 20)	//有少数特大货车判为中小客车
//		{
//			RetPattern = ZHONGXIAOKE;
//		}  
//		else if (VehLength >= BIGANGSMALLTHR && VehLength < 6500) //6000——6500
//		{
//			if(VehWide < 1700 && VehHeight < 2100)
//			{
//				RetPattern=XIAOHUOCHE;//小型货车
//			}
//			else if (IStou!=1 && (!(l_u32Height2 - l_u32Height1 > 180 && l_u32Height1 > 0 && l_u32Height2 > 0 && l_u32Height2 > l_u32Height1)))
//			{
//				if (((allSDcha < l_nDaKeThreshhold11 && MultiCheshenSD < l_nDaKeThreshhold12 && MultiCheshenSD > 20))
//				&& VehHeight>2800 && VehHeight<l_nDaKeHeightThr && VehLength >= BIGANGSMALLTHR && pVehicle->Vdata.u16FrameCnt > DakecheFrameCnt)		
//				{
//					RetPattern = DAKECHE;	//大客车
//				}
//				else if((allSDcha < 100 && MultiCheshenSD < 150)
//				&& VehLength <= 7500 && VehHeight > 2350 && VehHeight <= 2700) ////判断考斯特车为大客车
//				{
//					RetPattern = DAKECHE;
//				}
//				else
//				{
//					RetPattern = XIAOHUOCHE;//小型货车
//				}				
//			}
//			else
//			{
//				RetPattern= ZHONGHUO;//中型货车  20131223改小型货车为中型货车  XIAOHUOCHE
//			}
//			if(RetPattern == XIAOHUOCHE && (VehHeight > 3400 ||(VehWide > 2500 && VehHeight > 3000)))
//			{
//				RetPattern=ZHONGHUO;//中型货车
//			}	
//		}
//		else if (VehLength >= 6500 && VehLength <12000)	   //6500——12000
//		{
//			if (VehHeight < 2200 )
//			{
//				RetPattern = ZHONGHUO;   //中货车
//			}
//			else if (IStou==1 || (l_u32Height2 - l_u32Height1 > 200 && l_u32Height1 > 0 && l_u32Height2 > 0 ) )
//			{
//				if(VehLength <= 7500 || VehHeight <= 2900)
//				{
//					RetPattern = ZHONGHUO;//中型货车
//				}		
//				else if(l_nCheTouHeight < 2800 && l_nCheTouHeight > 1000)
//				{
//					RetPattern = ZHONGHUO;//中型货车
//				}
//				else if(VehLength > 9000)
//				{
//					RetPattern = DAHUO;//大型货车
//				}
//				else
//				{
//					RetPattern = ZHONGHUO;//中型货车
//				}
//			}
//			else if (VehHeight>3400 && l_u32Height2>3400 && l_u32Height2>l_u32Height1+50
//				&& tmpHeight>0 && tmpHeight<40) //20140809
//			{
//				RetPattern = DAHUO;	//大型货车
//			}
//			else if ((allSDcha < l_nDaKeThreshhold11 && MultiCheshenSD < l_nDaKeThreshhold12 && MultiCheshenSD > 20)//20130704,加入多点方差的下限，防止小货的误判
//			&& VehHeight>2800 && VehHeight<l_nDaKeHeightThr && VehLength >= BIGANGSMALLTHR  && pVehicle->Vdata.u16FrameCnt > DakecheFrameCnt)  	 //		帧数限制
//			{
//				RetPattern = DAKECHE;	//大客车
//			}
//			else if((allSDcha < l_nDaKeThreshhold11 && MultiCheshenSD < l_nDaKeThreshhold12)
//			&& VehLength < 7500 && VehHeight > 2500 && VehHeight < 2750)   //将2600改为2500
//			{
//				RetPattern = DAKECHE;	//大客车（19座以上客车，25座以下）
//			}
//			else if((allSDcha < 100 && MultiCheshenSD < 150&&VehLength < 7500)
//			 && VehHeight > 2350 && VehHeight <= 2500) ////判断考斯特车为大客车
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
//					RetPattern = ZHONGHUO;//中型货车
//				}		
//				else if(l_nCheTouHeight < 2800 && l_nCheTouHeight > 1000)
//				{
//					RetPattern = ZHONGHUO;//中型货车
//				}
//				else if(VehLength > 9000)
//				{
//					RetPattern = DAHUO;//大型货车
//				}
//				else
//				{
//					RetPattern = ZHONGHUO;//中型货车
//				}
//			}
//		}
//		else if (VehLength >= 12000 && VehLength <15000)	//12000——15000
//		{
//			if (IStou == 1)
//			{
//		         if  (SigleCheshenSD< l_nJZXThreshhold11  && MultiCheshenSD < l_nJZXThreshhold12  && VehHeight>3800 && VehHeight<4400)
//			     {
//                     RetPattern = JIZHUANGXIANG;  //集装箱车
//			     }
//                 else	 
//			     {
//                     RetPattern = TEDAHUO;	//特大型货车
//			     }
//			}
//			else if (l_u32Height2 - l_u32Height1 > 200 && l_u32Height1 > 0 && l_u32Height2 > 0 && l_u32Height2 > l_u32Height1)
//			{
//				RetPattern = DAHUO;	//大型货车
//			}
//			else if (VehHeight>3400 && l_u32Height2>3400 && l_u32Height2>l_u32Height1+100
//				&& tmpHeight>0 && tmpHeight<40) //20140808
//			{
//				RetPattern = DAHUO;	//大型货车
//			}
//			else if ((allSDcha < l_nDaKeThreshhold11 && MultiCheshenSD < l_nDaKeThreshhold12 && MultiCheshenSD > 0)
//			&& VehHeight>2800 && VehHeight<l_nDaKeHeightThr && pVehicle->Vdata.u16FrameCnt > DakecheFrameCnt)
//			{
//				RetPattern = DAKECHE; //大客车
//			}
//			else
//			{
//				RetPattern = DAHUO;		//没有头，即车头与车身没有明显的界限
//			}
//		}
//		else if(VehLength >= 15000 && VehLength <=18000 )
//		{
//			if (IStou == 1 ||((l_u32Height2 - l_u32Height1 > 200 && l_u32Height1 > 0 && l_u32Height2 > 0 && l_u32Height2 > l_u32Height1)))	//20140122
//			{
//		        if ( SigleCheshenSD< l_nJZXThreshhold11  && MultiCheshenSD < l_nJZXThreshhold12  && VehHeight>3800 && VehHeight<4400)
//			    {
//                    RetPattern = JIZHUANGXIANG;  //集装箱车
//                }
//			    else
//			    {
//                     RetPattern = TEDAHUO; //特大型货车
//		    	}				
//			}
//			else if (VehLength < 16000 && ((allSDcha < l_nDaKeThreshhold11 && MultiCheshenSD < l_nDaKeThreshhold12 && MultiCheshenSD > 50)
//			         && VehHeight>2800 && VehHeight<l_nDaKeHeightThr && pVehicle->Vdata.u16FrameCnt > DakecheFrameCnt))  //增加20140122
//			{
//				RetPattern = DAKECHE;
//			}
//			else
//			{
//	             if  (SigleCheshenSD< l_nJZXThreshhold11  && MultiCheshenSD < l_nJZXThreshhold12  && VehHeight>3800 && VehHeight<4400)
//			     {
//                     RetPattern = JIZHUANGXIANG;  //集装箱车
//			     }
//                 else
//			     {
//                     RetPattern = TEDAHUO; //特大型货车
//			     }
//			}
//		}
//		else
//		{
//            if ( SigleCheshenSD< l_nJZXThreshhold11  && MultiCheshenSD < l_nJZXThreshhold12  && VehHeight>3800 && VehHeight<4400)
//		    {
//		       RetPattern = JIZHUANGXIANG; //集装箱车
//		    }
//		    else
//		    {
//		       RetPattern = TEDAHUO; //特大型货车 
//		    }
//		}
//		
//		if (!IStou && Widefangcha < 70 && (RetPattern == ZHONGHUO || RetPattern == DAHUO)
//			&&(!((l_u32Height2 - l_u32Height1 > 200) 
//			 && l_u32Height1 > 0 && l_u32Height2 > 0)) && VehHeight > 2500)	//修正大客车,添加条件
//		if (!IStou && Widefangcha < 70 && (RetPattern == ZHONGHUO || RetPattern == DAHUO))	//修正大客车,添加条件
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
//		//增加对渣土车、混泥土车等的判断
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
//			if (TouGao >= 2900 && TouKuan >= 2200)	   //针对渣土大型货车 存在误判车头高宽较大的中货的风险
//			{
//				RetPattern = DAHUO;
//			}
//
//			if (VehLength > 7500 && VehLength < 10000 && VehHeight > 3700 && VehHeight < 4200)  //针对3轴混凝土车
//			{
//				for(u16Index = Veh_Num*2/5; u16Index < Veh_Num*3/5; u16Index++)
//				{
//					if(Height[u16Index+1]- Height[u16Index] >= 0)	//顶部高度连续增加
//						l_u8HighCount++;	
//				}
//				if(l_u8HighCount >= Veh_Num/5 && Height[Veh_Num*3/5] - Height[Veh_Num*2/5] > 150)
//				{
//					RetPattern = DAHUO;	   //大型货车	
//				}
//			}
//		}
//		else if(RetPattern == DAHUO)							          
//		{
//			if(2 == Toupos)												 //针对车头有斗篷的中货
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


/*******************计算区分小型车（主要是中小客车1，小型货车2，摩托车8，拖拉机9)********************/
uint8 GetLightVehPattern(VehicleStruct *pVehicle)
{
	uint8   RetPattern    = 0;	//
	uint8   xiaokeche     = 0;
	uint8   konghuoche    = 0;
	uint8   xiaohuoche    = 0;
	uint8 	l_u8index     = 0;
	uint8   l_u8EqualNum  = 0;

	uint8   l_u8FrameNum  = 0;   //在判断金杯车时丢掉的尾部帧数

	uint8   i = 0;
	uint8	j = 0;
	uint8 k;
	uint8   l_u8Flag = 0;
	int32 tmpValue2 = 0;
	int32 tmpDiff[FRAME_MAXCNT] = {0};  //20130514  用于存放高度差分值
	uint8 l_u8MaxPos = 0;  //差分最大值索引位置
	uint8 l_u8MinPos = 0;  //差分最小值索引位置
	int32 l_n32MaxDiff = 0;
	int32 l_n32MinDiff = 0; 
	int32 l_n32TmpSum1 = 0;
	int32 l_n32TmpSum2 = 0;
	int32 l_n32SecHeight = 0;  //第2大高度
	int32 l_n32TempHeight = 0;
	uint8   Veh_Num       = 0;
	int32   VehLength     = 0;
	int32   VehHeight     = 0;
	int32   VehWide       = 0;
	uint8   u8SidePlanenessFlag = 0;  //保存每帧的顶部平整标识，0表示不平整，1表示平整
	uint8 l_u8PiKaFlag = 0;
	uint8  l_u32index,l_u32index2;
	int32 Height[FRAME_MAXCNT] = {0};   //用于计算时使用的车高
	uint8  l_u8Left_Index = 0;
	uint8  l_u8Right_Index = 0;
	int32  l_n32Left_MaxZ = 0;
	int32  l_n32Right_MaxZ = 0;
	int32  l_n32MinChassisHeight = 600;
	int32  l_n32Min_X            = 0;
	int32  l_n32Max_X            = 0;
	memset(Height,0,sizeof(Height));

	//20140217 增加对参数判断
	if (pVehicle == NULL)
	{
		return 0;
	}
   Veh_Num       = pVehicle->Vdata.u16FrameCnt;
	if (Veh_Num > FRAME_MAXCNT || Veh_Num < 1)
	{
		Veh_Num = 0;  //帧数超过最大帧数值，帧数赋值为0
		pVehicle->Vdata.u16FrameCnt = 1;
	}
	VehLength     = pVehicle->yLen;
	VehHeight     = pVehicle->zLen;
	VehWide       = pVehicle->xLen;

 	memcpy(Height, pVehicle->Vdata.zMax, Veh_Num*sizeof(int32));
	if (Veh_Num <= 2)  //  2帧或以下是中小客车1
	{
		RetPattern = ZHONGXIAOKE;
		return RetPattern;
	}
	else
	{
		//剔除高度中1帧异常点	20140401增加
		for (i = 0; i < Veh_Num; i++)
		{ //先找出第2大高度值
			if ((l_n32SecHeight < pVehicle->Vdata.zMax[i]) && 
				(VehHeight > pVehicle->Vdata.zMax[i]))
			{
				l_n32SecHeight = pVehicle->Vdata.zMax[i];
			}		  
		}
		if (VehHeight-l_n32SecHeight > 300) //高度超过300，重新检查最大高度值那帧的数据
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
					//高度相差比较大
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
		///////////小轿车判定////////////////////////
		if (VehHeight < 2280)
		{
			if (pVehicle->Vdata.zMax[0] < 800 && pVehicle->Vdata.zMax[1] < 1000 && pVehicle->Vdata.zMax[Veh_Num-1] < 800)
			{
				xiaokeche = 1;  //中小客车标志	
			}
		}

		for (i = 0; i < Veh_Num/3; i++)//计算前1/3帧数的平均高
		{
			tmpValue2 += pVehicle->Vdata.zMax[i];
		}
		if (tmpValue2/(Veh_Num/3) < 1000)	 //前1/3帧数的平均高小于1m
		{
			xiaokeche = 2;   //中小客车
		}
		////////////////小货车判定///////////////////////////
		tmpValue2 = 0;
		if (VehHeight > 2000)  	//计算车高与帧高的差值，求差值的均值
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
		else if (VehHeight > 1800)  //计算相邻两帧的差值，求差值的均值
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
			xiaohuoche = 1;   //小货车
		}
		/////////空货车判定/////////////////////
		if (Veh_Num >= 4 && (Veh_Num/2+1 < Veh_Num-1) &&	 //Veh_Num/2+1不能是最后一帧数据
		   (VehHeight - pVehicle->Vdata.zMax[Veh_Num/2] > 300 || VehHeight - pVehicle->Vdata.zMax[Veh_Num/2+1] > 300))
		{   //20140325 增加防止异常高度导致判断空货
			for (i = 0; i < Veh_Num/2+1; i++)
			{
				if ((VehHeight - pVehicle->Vdata.zMax[i] < 300) && 
					(VehHeight > pVehicle->Vdata.zMax[i] || l_u8EqualNum > 1)) 
				{//在前半部分帧数内有帧车高与车高相差在300内
					konghuoche = 1;  //空货车
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
			l_n32TmpSum1 = l_n32TmpSum1/(Veh_Num/2);  //前一半帧数高度的均值
			
			for (i = Veh_Num/2; i < Veh_Num-1; i++)	  //丢掉最后一帧数据
			{
				l_n32TmpSum2 += pVehicle->Vdata.zMax[i];
			}
			l_n32TmpSum2 = l_n32TmpSum2/(Veh_Num-Veh_Num/2-1); //后一半帧数高度的均值
			if (l_n32TmpSum1 > (l_n32TmpSum2 + 150) && VehHeight > 1700)	   //40改为150
			{
				konghuoche = 1;
			}
		}
	}

    //增加判断小客车




    if(VehWide < 1000 && VehLength < 2000 && VehHeight < 1800)	  //3帧或以上
    {
	    RetPattern = MOTUOCHE;	 //摩托
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
			//SUV的识别
			if (VehHeight < 2000 && VehHeight >= 1600)
			{
				l_u8index = 0;
				l_n32TmpSum1 = 0;
				for (i = 0; i < Veh_Num; i++)	  //寻找开始帧数高度低于1300的帧数
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
				for (i = Veh_Num/2; i < Veh_Num-1; i++)	  //计算后一半帧数高度的均值
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
				//箱式小货的判断
				tmpValue2 = 0;
				for(i = 2;i < (Veh_Num>>1);i++)
				{
					if(pVehicle->Vdata.zMax[i] + 300 < pVehicle->Vdata.zMax[i-1])	 //查找车头的最低特征点
					{
						tmpValue2 = i+1; 
						while(tmpValue2 < Veh_Num-1)
						{
							if(pVehicle->Vdata.zMax[tmpValue2+1] - pVehicle->Vdata.zMax[tmpValue2] > 300)	 //查找车头的最低特征点
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
							 //箱式小货 	
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
			   	//判断是否为箱式小客车
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
								 RetPattern = ZHONGXIAOKE; //箱式小客车	
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
						RetPattern = ZHONGXIAOKE;  //满货 XIAOHUOCHE   改为中小客车
					}	 
				 }
			   }
		
				//非箱式客、货车
				if(!RetPattern)
				{
					tmpValue2 = 0;
					for(i = 2;i < Veh_Num-1;i++)  //最后一帧不要
					{
						if(pVehicle->Vdata.zMax[i] + 250 < pVehicle->Vdata.zMax[i-1])	 //查找车头的最低特征点
						{
						   tmpValue2 = i;
						   break;
						}
					}
					if(!tmpValue2)
					{ 	
						RetPattern = ZHONGXIAOKE; //小客车				
					}
					else
					{
			           	for (i = 2; i < Veh_Num; i++)    //20130514 计算第2帧到倒数第2帧的车高差分
						{
							tmpDiff[i] = pVehicle->Vdata.zMax[i] - pVehicle->Vdata.zMax[i-1];
						}
						tmpDiff[0] = Veh_Num;   //帧数
						//寻找车高差分中的最大值和最小值及索引位置	20130514 
						l_n32MaxDiff = tmpDiff[2];
						l_n32MinDiff = tmpDiff[Veh_Num-1];
						l_u8MaxPos   = 2;
						l_u8MinPos   = Veh_Num-1;
						for( i = 3; i < Veh_Num; i ++)
						{				
							if (l_n32MaxDiff < tmpDiff[i])	//寻找最大值及索引
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
						//差分变化很大，说明是小货车
						if ( l_n32MinDiff < -800)
						{
							RetPattern = XIAOHUOCHE;   
						}
						else
						{
							//如果最小值点的索引小于总帧数2/3，则比较最后一帧与前一帧的差分， 防止出错	  20130514
							if ( l_u8MinPos <= Veh_Num*2/3 && l_n32MinDiff > pVehicle->Vdata.zMax[Veh_Num-1]-pVehicle->Vdata.zMax[Veh_Num-2])
							{
								l_u8MinPos = Veh_Num-1;
								l_n32MinDiff = pVehicle->Vdata.zMax[Veh_Num-1]-pVehicle->Vdata.zMax[Veh_Num-2];
							}
							
							//如果车前部点数小于车尾点数，则寻找最大差分索引的下一个差分变化较大的点   20130514
							if ( l_u8MaxPos -1 < (Veh_Num-l_u8MinPos+1) && tmpDiff[l_u8MaxPos+1] > 150)
							{
								l_u8MaxPos = l_u8MaxPos + 1;
							}
							//车首点数大于或等于车尾点数，且首尾点数和大于或等于车身点数 20130514
							if (l_u8MaxPos>=(Veh_Num-l_u8MinPos) && (l_u8MaxPos + Veh_Num-l_u8MinPos >= l_u8MinPos-l_u8MaxPos) 
							   && l_u8MaxPos-1 < l_u8MinPos && (Veh_Num - (l_u8MaxPos-1)*2 > 0  ))
							{
								RetPattern = ZHONGXIAOKE;   //中小客车
							} 
							else
							{
								RetPattern = XIAOHUOCHE;  //小货车
							}
						}
					}
				}
			}
		}
		////主要是皮卡的检测

		if(RetPattern == ZHONGXIAOKE && VehHeight > 1540 && VehHeight < 1790 && Veh_Num >= 2)	 //范围放大一点1600-1700
		{
			//小皮卡的检测，认为尾巴比较长的中小客是皮卡。
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
				 l_u8PiKaFlag = 1;	   //检测到可能是尾部较长
			}
			
			if( Veh_Num >= 5 && l_u8PiKaFlag)
			{
				for(i = Veh_Num - 2; i > Veh_Num - 5; i--)
		    	{
			    	if(Height[i] < 1250 && Height[i] > 940)//皮卡的车斗位置
					{
						if (pVehicle->Vdata.zdata[i][0] >= FRAME_BUFLEN) //超过阈值
							continue;
						//寻找左边最大值
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
						//寻找右边最大值
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
						//找高点差和
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
					   		RetPattern = XIAOHUOCHE;//小货，皮卡
					   		break;
						}
					}		    
		    	}
			}
		} 			 			
	}

	if (g_sspSetup.u8RoadType)//高速
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


uint16 GetLimitValue(int* pg_ZdistanceI,int* pXdata, int len,int startPos, uint8 u8Flag)    //侧装的时候该函数还有修改	u8Flag为0表示垂直激光器数据，1表示倾斜激光器数据
{ 
//	uint16 ret = startPos;
//	uint16  l_u16CountOut = 0;
//	int l_midPt = len>>1;
//	int l_PtNum1 = 0;    //abs(g_sspSetup.u16StartPtNum - g_sspSetup.u16VerticalZeroPos);  // 零点与起始点的点数差
//	int l_PtNum2 = 0;    //abs(g_sspSetup.u16EndPtNum - g_sspSetup.u16VerticalZeroPos);    //结束点与零点的点数差
//
//	//增加对参数的合法性判断 20140214
//	if (pg_ZdistanceI == NULL || pXdata == NULL || len >= POINT_SUM )
//	{
//		return ERRORVALUE;		 //返回错误值
//	}
//	
//	if (u8Flag)	 //倾斜激光器
//	{
//		l_PtNum1 = abs(g_sspSetup.u16StartPtNum - g_sspSetup.u16InclineZeroPos);  // 零点与起始点的点数差
//		l_PtNum2 = abs(g_sspSetup.u16EndPtNum - g_sspSetup.u16InclineZeroPos);    //结束点与零点的点数差
//		
//		//20140217 增加	  正装倾斜
//		if (g_sspSetup.u8InstallFlag)
//		{
//			l_midPt = (g_sspSetup.u16InclineZeroPos > g_u16InclineStartAnglePt) ?  
//					  (g_sspSetup.u16InclineZeroPos - g_u16InclineStartAnglePt) : (len>>1);
//		}		
//	}
//	else  //垂直激光器
//	{
//		l_PtNum1 = abs(g_sspSetup.u16StartPtNum0 - g_sspSetup.u16VerticalZeroPos0);  // 零点与起始点的点数差
//		l_PtNum2 = abs(g_sspSetup.u16EndPtNum0 - g_sspSetup.u16VerticalZeroPos0);    //结束点与零点的点数差
//
//		//20140217 增加	  正装垂直
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
//		if (startPos < l_midPt && abs(pXdata[startPos]) > g_MedianLeftWide &&		    //小点为左隔离带
//		    abs(pXdata[startPos]) <= g_MaxLeftWide)
//		{
//			ret = startPos;
//			break;
//		}
//		else if (startPos > l_midPt && abs(pXdata[startPos]) > g_MedianRightWide &&		//大点为右隔离带
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
	int32 retMax = 2000; //默认值 2米

	//20140217 增加对参数合法性判断
	if (pData == NULL || start > g_sspSetup.u16EndPtNum0-g_sspSetup.u16StartPtNum0 ||
		end > g_sspSetup.u16EndPtNum0-g_sspSetup.u16StartPtNum0)	 //参数异常，给固定值，不影响程序运行
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

	//20140217 增加对参数合法性判断
	if (pData == NULL)	 //参数异常，给固定值，不影响程序运行
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

//vehPosFlag 标识是0表示车辆在当前区域中间的左值，1是中间的右值，当前区域的左边是2，当前区域的右边是3
//pPtStruct 表示车辆的点结构体变量，pPtData表示当前区域的点结构体变量
uint16 GetPosFromXDistance(int32 *xDistant,PointStruct *pPtStruct, PointStruct *pPtData, uint8 vehPosFlag)	 
{
	uint16 ret = 0,Index = pPtData->u16Leftpt;
	uint16 l_16tmpValue = pPtData->u16Rightpt;
	uint16 l_minValue = 0xFFFF;
	uint16 l_u16LaneWide = g_sspSetup.LaneWide; 
	uint8  l_u8Flag = 0;   //区域宽度标志 0表示当前区域宽度大于3.5m小于7m，1表示大于7m

	//20140217 增加对参数合法性判断
	if (xDistant == NULL || pPtStruct == NULL || pPtData == NULL)
	{
		return ERRORVALUE;
	}

	if (g_sspSetup.u8LaneNum == 6) //正装6车道时，区域宽度标志才有可能大于7m
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

	if (!vehPosFlag)  //在中间	计算左值
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
	else if (vehPosFlag == 1) //在中间，计算右值
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
	else if (vehPosFlag == 2)  //在左边，求右值
	{
		for (; Index < l_16tmpValue-1; Index++)
		{
			if ( (!l_u8Flag) && abs(xDistant[Index] - xDistant[Index+1]) > 1000&&
			    abs(xDistant[Index]-pPtData->n32xLeft) < l_u16LaneWide &&	  //   pPtStruct改为pPtData
				abs(pPtData->n32xRight-xDistant[Index+1]) < l_u16LaneWide )
			{
				ret = Index;
				break;
			}
			else if (l_u8Flag && abs(xDistant[Index] - xDistant[Index+1]) > 1000 &&
			    abs(xDistant[Index]-pPtData->n32xLeft) < l_u16LaneWide &&				  //  pPtStruct改为pPtData
				abs(pPtData->n32xRight-xDistant[Index+1]) > l_u16LaneWide)	  //剩下的区域要大于3.5m
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

		if (l_u8Flag)	 //区域大于7M
		{
			while(Index < l_16tmpValue)
			{
				if(abs(xDistant[Index] - pPtStruct->n32xRight) < l_minValue && 
				  (abs(pPtData->n32xLeft-xDistant[Index])> (SMALL_AREA>>1)) &&
				  abs(pPtData->n32xLeft-xDistant[Index]) < l_u16LaneWide)		   //  加上abs(pPtData->n32xLeft-xDistant[Index]) < l_u16LaneWide)
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
	else   //在右边,求左值
	{
		for (; Index < l_16tmpValue-1; Index++)
		{
			if ( (!l_u8Flag) && abs(xDistant[Index] - xDistant[Index+1]) > 1000&& 
			   abs(xDistant[Index+1]-pPtData->n32xRight) < l_u16LaneWide &&		   //  pPtStruct改为pPtData
			   abs(pPtData->n32xLeft-xDistant[Index]) < l_u16LaneWide )
			{
				ret = Index+1;
				break;
			}
			else if (l_u8Flag&&abs(xDistant[Index] - xDistant[Index+1]) > 1000 && 
			   abs(xDistant[Index+1]-pPtData->n32xRight) < l_u16LaneWide &&			//	pPtStruct改为pPtData
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
				  abs(pPtData->n32xRight-xDistant[Index]) < l_u16LaneWide)			 //加上abs(pPtData->n32xRight-xDistant[Index]) < l_u16LaneWide)
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
** 函数名称:  Get_Vehicle_Info
** 函数功能:  得到车辆信息
** 入口参数:  无
** 出口参数:  无
** 函数说明:  对扫描的一帧数据处理，包括：坐标转换，分车，提取车辆信息，识别车辆存储，发分车信号，判定车型等
*********************************************************************************************************/
void Get_Vehicle_Info(void)					
{	
//	uint8  l_DafeiPtNum = 0;   //打飞点数，最多联系打飞点4个
//	uint8  l_u8Ret      = 0;
//	uint8  l_u8Count    = 0;
//	uint8  RetIsVehicle = 0;  //针对中小客车点打飞判断的返回值		
//	int		*data;
//	int 	*data0;
//	int		JG_time;
//	uint32	Time_Vertical;
//	PointSet l_FrameInfo1;
//	int32 l_leftX,l_rightX;    //左，右x距离
//	uint16 l_leftXpt, l_rightXpt;   //左右X距离对应的点数
//	uint16 l_leftPt ;	//左边起始、结束点数 （靠近激光器与隔离带之间的车道）
//	uint16 l_rightPt;  //右边起始、结束点数 （远离激光器与隔离带之间的车道）
//
//	int i=0;
//	int j=0;
//	int k=0;		            //分车用
//	int m=0;
//	int index=0;
//
//	int32 l_32tmpValue,l_32tmp,l_32tmp2,l_tmp1,TempVaule1;
//	PointStruct l_u16PosVect[POINTSET_CNT] = {0};	//存放并车的位置信息
//	uint16 l_u16index,l_u16tmp;
//	uint16	l_index;
//	uint16	l_u16StartPt,l_u16EndPt;
//	int32	l_n32StartY,l_n32EndY;
//	uint16	l_u16StartYpt,l_u16EndYpt;
//	IncPtSt l_u16IncPosVect[POINTSET_CNT] = {0};	//存放并车的位置信息
//
//	//对打飞段的处理，将打飞段拉成直线
//    int Dafeiflag=0;
//    int DafeiData[180][4]={0};
//	int MaxZ=0;
//	int MaxX=0;
//	int MinZ=0;
//	int MinX=0;
//	int Maxi=0;
//	 
///*EnterX1 车头到达秤台前，EnterX2车头测速度，ExitX3车辆过此线抛车，ExitX4剔除车辆*/
//    int EnterX1=-10000;			  //秤台进入扫描区域
//    int EnterX2=-3000;			  //输出的速度为过车头过此线时的速度
//	int ExitX3=-2000;                 //车头过此线后抛车
//	int ExitX4=-600;              //车头过此线后剔除车辆
//
//    int Lengthline1=-7500;
//	int Lengthline2=ExitX3;
//
//    int Tmp_Z = 0;                 //车辆高度
//	int	Tmp_Y = 0;
//
//	PtIncSet l_FrameInfo;
//	memset(&l_FrameInfo, 0, sizeof(l_FrameInfo));
//	memset(&l_FrameInfo1,0, sizeof(PointSet));
///*************************************/	 
//	data = LMS_data_2;     //%测速激光数据
//
//	data0 = LMS_data_1;		//%宽高激光数据	
//
//	JG_time = data[361];	 //%测速激光
//	Time_Vertical = (uint32)LMS_data_1[361];
//
//    //判断激光器的0点值与180点的的位置关系，然后处理起始或终止点位置 20130426  	  20130614调正顺序位置
//	g_u16InclineStartAnglePt = GetStartEndPt(data, 30, 0, 1);/*寻找激光器的起始点*/
//	g_u16InclineEndAnglePt	  = GetStartEndPt(data, 300, 1, 1);
//
///*****************/	
//JG_T0TC[0] = T0TC;
//JG_counter2[0]=t0_count2;
///*****************/
//
//
///***************************激光器扫描数据坐标转换*******************************/
//    k=0;
//    //20130426  修改激光器起始角度点、结束角度点的计算方法		
//	j=2- 1;//垂直激光器的零值扫描点的点数值-1	   
//	//j=j-2;
//	/*通过测得的距离值计算出零点和起始点间每个点的直角坐标值（X和Z值）*/								  //32768
// //   for(i=g_sspSetup.u16InclineZeroPos-1;i >= g_u16InclineStartAnglePt;i--)
//	{
//		if(data[i]>ThresOrigineDataLow && data[i]<ThresOrigineDataHigh)//极坐标下，测得的距离在0.03m到20m间
//		{ 
////             g_ZdistanceI[j] = g_sspSetup.IncHeightLaser - ((data[i]*Tabcos[g_sspSetup.u16InclineZeroPos-i])>>15);//车高
////			 g_YdistanceI[j]= -1*((data[i]*Tabsin[g_sspSetup.u16InclineZeroPos-i])>>15);//扫描点的X坐标值	
//			 Dafeiflag=0;			 
//		}
//		else															 //测得的距离不在0.03m到20m间
//		{
//		   
//		   if(Dafeiflag==0)
//		     {
//			 k++;
//		     DafeiData[k][1]=j;		 //将打飞段的终止索引记录在DafeiData[k][1]中，起始索引记录在DafeiData[k][0]中，同时记录g_ZdistanceI为零时的X坐标
//			 DafeiData[k][3]=-(Tabsin[g_sspSetup.u16InclineZeroPos-i]*(g_sspSetup.IncHeightLaser-100)/Tabcos[g_sspSetup.u16InclineZeroPos-i]);
//			 }
//		     DafeiData[k][0]=j;
//			 DafeiData[k][2]=-(Tabsin[g_sspSetup.u16InclineZeroPos-i]*(g_sspSetup.IncHeightLaser-100)/Tabcos[g_sspSetup.u16InclineZeroPos-i]);
//		     Dafeiflag=1;		 
//		}
//		j=j-1;	//点数减1
//	}
//    
//	DafeiData[0][0]=k;
//
///*******************************************************************/	
///*****************/
//JG_T0TC[1] = T0TC;
//JG_counter2[1]=t0_count2;
///*****************/
//	 j=g_sspSetup.u16InclineZeroPos-g_u16InclineStartAnglePt;   //垂直激光器的零值扫描点的点数值	 
//	 j=j-1;//j=143;			  //20140121
//	 /*通过测得的距离值计算出零点和结束点间每个点的实际坐标值*/	
//	 for(i=g_sspSetup.u16InclineZeroPos;i <= g_u16InclineEndAnglePt;i++) 
//	 {
//	 	if(data[i] > ThresOrigineDataLow && data[i]<ThresOrigineDataHigh)//测得的距离在0.03m到20m间
//		{ 		
//			g_ZdistanceI[j] = g_sspSetup.IncHeightLaser - ((data[i]*Tabcos[i-g_sspSetup.u16InclineZeroPos])>>15);//车高
//			g_YdistanceI[j] = ((data[i]*Tabsin[i-g_sspSetup.u16InclineZeroPos])>>15);//扫描点的X坐标值	
//			Dafeiflag=0;
//		}
//		else															   //测得的距离不在0.03m到20m间
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
//		} 				 //计算打飞点时的X坐标时，对有隔离带情况，要使用g_sspSetup.HeightLaser	  zyj 20130607
//		j=j+1;	//点数加1
//	 }
//	 DafeiData[0][1]=k; 
//	 
//  /**********对打飞点的处理*************/
//
/////////拉平处理///////////
//for(k=1; k<=DafeiData[0][1];k++ )
//  {
//		  /////对于车尾打飞的处理
//		if(((DafeiData[k][0]==0)||(g_ZdistanceI[DafeiData[k][0]-1]<=ThresVehLow))&&(g_ZdistanceI[DafeiData[k][1]+1]>ThresVehLow))
//		   {
//		   m=DafeiData[k][0]+g_u16InclineStartAnglePt;	 //m为数组data[]的索引；车尾的索引
//		   if(g_sspSetup.u16InclineZeroPos>=m)
//		     {
//			 g_YdistanceI[DafeiData[k][0]]=-(Tabsin[g_sspSetup.u16InclineZeroPos-m]*(g_sspSetup.IncHeightLaser-g_ZdistanceI[DafeiData[k][1]+1])/Tabcos[g_sspSetup.u16InclineZeroPos-m]);
//			 }
//		   else
//		      {
//			  g_YdistanceI[DafeiData[k][0]]=Tabsin[m-g_sspSetup.u16InclineZeroPos]*(g_sspSetup.IncHeightLaser0-g_ZdistanceI[DafeiData[k][1]+1])/Tabcos[m-g_sspSetup.u16InclineZeroPos];
//			  }
//
//			g_ZdistanceI[DafeiData[k][0]] = ThresVehLow+1;//20140915 打飞点处的修正
//			for(i=DafeiData[k][0]+1;i<=DafeiData[k][1];i++)
//			{
//				g_ZdistanceI[i]=g_ZdistanceI[DafeiData[k][1]+1];
//				g_YdistanceI[i]=g_YdistanceI[DafeiData[k][0]]+((g_YdistanceI[DafeiData[k][1]+1]-g_YdistanceI[DafeiData[k][0]])*(i-DafeiData[k][0]))/(DafeiData[k][1]-DafeiData[k][0]+1);
//			}
//		}
//
//		//////对于车头打飞的处理
//		else if((DafeiData[k][1]==g_u16InclineEndAnglePt-g_u16InclineStartAnglePt||g_ZdistanceI[DafeiData[k][1]+1]<=ThresVehLow)&&(g_ZdistanceI[DafeiData[k][0]-1]>ThresVehLow))
//		{
//			m=DafeiData[k][1]+g_u16InclineStartAnglePt;	 //m为数组data[]的索引；车头的索引
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
//		///////对于车身全部打飞的情况
//		else if((g_ZdistanceI[DafeiData[k][0]-1]<=ThresVehLow)
//				&&(g_ZdistanceI[DafeiData[k][1]+1]<=ThresVehLow)
//				&&DafeiData[k][0]<DafeiData[k][1])	//20140121 增加打飞点位置不相等
//	    {
//			m=DafeiData[k][0]+g_u16InclineStartAnglePt;	 //m为数组data[]的索引；车尾的索引
//			if(g_sspSetup.u16InclineZeroPos>=m)
//			{
//				g_YdistanceI[DafeiData[k][0]]=-(Tabsin[g_sspSetup.u16InclineZeroPos-m]*(g_sspSetup.IncHeightLaser-500)/Tabcos[g_sspSetup.u16InclineZeroPos-m]);
//			}
//			else
//			{
//				g_YdistanceI[DafeiData[k][0]]=Tabsin[m-g_sspSetup.u16InclineZeroPos]*(g_sspSetup.IncHeightLaser-500)/Tabcos[m-g_sspSetup.u16InclineZeroPos];
//			}
//			
//			m=DafeiData[k][1]+g_u16InclineStartAnglePt;	 //m为数组data[]的索引；车头的索引
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
//		/////////对于车身中间段有打飞的情况
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
//  /**********打飞点处理结束*************/	
//
///***************************激光器扫描数据坐标转换完毕*******************************/
//
//
///********************************宽高激光数据坐标转换开始****************************/
//	g_u16VerticalStartAnglePt = GetStartEndPt(data0,g_sspSetup.u16StartPtNum, 0, 0);	//垂直激光器坐标转换开始点	 参数中第1个0表示找开始点标志，第2个0表示垂直激光器
//	g_u16VerticalEndAnglePt   = GetStartEndPt(data0,g_sspSetup.u16EndPtNum, 1, 0);   //垂直激光器坐标转换结束点
// 
//	//求高度值 
//	//垂直激光器坐标转换
//	if (( g_u16VerticalStartAnglePt >= g_sspSetup.u16VerticalZeroPos &&  g_u16VerticalEndAnglePt >= g_sspSetup.u16VerticalZeroPos) ||
//		( g_u16VerticalStartAnglePt <= g_sspSetup.u16VerticalZeroPos &&  g_u16VerticalEndAnglePt <= g_sspSetup.u16VerticalZeroPos)) //转换一边（起始点和结束点在零点的一侧，侧装X坐标都为正）
//	{	//需要修改
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
//				if (l_tmp1 == 0)	 //第1个点
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
//	else   //起始点和结束点在零点的两侧
//	{
//		l_tmp1 = g_sspSetup.u16VerticalZeroPos - g_u16VerticalStartAnglePt - 1;
//		for(i=g_sspSetup.u16VerticalZeroPos-1;i >= g_u16VerticalStartAnglePt;i--)	//正装小点X坐标为负
//		{
//			if(data0[i]>ThresOrigineDataLow && data0[i]<ThresOrigineDataHigh)
//			{ 
//				l_DafeiPtNum = 0;
//				g_ZdistanceV[l_tmp1]= g_sspSetup.HeightLaser0 - ((data0[i]*Tabcos[g_sspSetup.u16VerticalZeroPos-i])>>15); // HeightLaser +
//				g_XdistanceV[l_tmp1]= -1*((data0[i]*Tabsin[g_sspSetup.u16VerticalZeroPos-i])>>15) + g_sspSetup.n32LaserHorizOff;  //g_sspSetup.u32LaserHorizOff应该定义为int32型，为正向大点方向移动，为负值时向小点方向移动					 
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
//			}			 //修改打飞点 X坐标的计算方法  zyj 20130609
//			l_tmp1=l_tmp1-1;
//		}	
//
//		l_tmp1 = g_sspSetup.u16VerticalZeroPos - g_u16VerticalStartAnglePt;
//		l_DafeiPtNum = 0;
//		for(i=g_sspSetup.u16VerticalZeroPos;i < g_u16VerticalEndAnglePt;i++) 	//正装大点X坐标为正
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
//				+ g_sspSetup.n32LaserHorizOff;		   //20130614  修改，以前方法超过正余弦索引
//			} 
//			l_tmp1=l_tmp1+1;
//		}	
//	} //垂直激光坐标转换完毕；
//
///********************************宽高激光数据坐标转换结束****************************/  
//
///********************************宽高激光有车区域查找开始***************************************/
//    l_32tmpValue = g_u16VerticalEndAnglePt-g_u16VerticalStartAnglePt+1;
//	//帧数据进行分有车区域,记录每个区域的左右位置信息
//	l_leftPt = GetLimitValue(g_ZdistanceV,g_XdistanceV,l_32tmpValue,0, 0);		 //每帧数据有车的左边起始点
//	l_rightPt = GetLimitValue(g_ZdistanceV,g_XdistanceV,l_32tmpValue,g_u16VerticalEndAnglePt-g_u16VerticalStartAnglePt, 0);	   //每帧数据中有车的终止点
//
//	//20140217 增加对l_leftPt和l_rightPt的合法性判断
//	//左边点索引小于右边点索引，且左右点索引都在有效数值范围内，否则跳出
//	if (!((l_leftPt < l_rightPt)&& (l_rightPt <= g_u16VerticalEndAnglePt)))
//	{
//		return;
//	}
//	l_u16index = 0;   //有车区域的索引，局部变量为零
//	for(i = l_leftPt;i<=l_rightPt;i++)
//	{
//		if( (!g_sspSetup.u8InstallFlag && ((abs(g_XdistanceV[i]) < g_NearMaxWide && abs(g_XdistanceV[i]) > g_NearMinWide ) || 
//		   (abs(g_XdistanceV[i]) > g_FarMinWide && abs(g_XdistanceV[i]) < g_FarMaxWide))) ||  //侧装方式
//		   	(g_sspSetup.u8InstallFlag && ((g_XdistanceV[i]<g_MedianLeftWide && abs(g_XdistanceV[i])>g_MedianLeftWide &&
//			abs(g_XdistanceV[i])<g_MaxLeftWide) ||(g_XdistanceV[i]>g_MedianRightWide && abs(g_XdistanceV[i])<g_MaxRightWide)))	//正装方式
//		   )	//判断不在隔离带范围
//		{
//			if((g_ZdistanceV[i] > ThresVehLow) && (g_ZdistanceV[i] < ThresVehHigh))	
//			{	 			
//				if(l_FrameInfo1.Ptdata[l_u16index].n32xLeft == 0)		  //每帧中有车部分的第1个点
//				{
//					l_FrameInfo1.Ptdata[l_u16index].n32xLeft = g_XdistanceV[i];
//					l_FrameInfo1.Ptdata[l_u16index].n32xRight = g_XdistanceV[i];
//					l_FrameInfo1.Ptdata[l_u16index].u16Leftpt  = i;
//					l_FrameInfo1.Ptdata[l_u16index].u16Rightpt = i;
//					l_FrameInfo1.Ptdata[l_u16index].u16xDis = 0;
//	
//					l_FrameInfo1.Ptdata[l_u16index].u16xMaxHt = g_ZdistanceV[i];
//					
//					if(l_u16index)	  //该帧数据中有车
//					{ 
//						l_u16tmp = abs(l_FrameInfo1.Ptdata[l_u16index - 1].n32xLeft - l_FrameInfo1.Ptdata[l_u16index - 1].n32xRight);
//						l_FrameInfo1.Ptdata[l_u16index - 1].u16xDis = l_u16tmp;	  //该帧数据中有车部分的宽度
//	
//						if(ISVehRegion(l_u16tmp, &l_FrameInfo1.Ptdata[l_u16index-1], g_ZdistanceV))	  	   //有效宽度大于300mm，有车区域最少4个点,认为是该帧中新的有车部分
//							l_FrameInfo1.u8Sum = (l_FrameInfo1.u8Sum+1)&POINTSET_MASK;			//有车区域加1
//						else 	 //有效宽度小于300mm ,认为是无效区域 ，属于前一辆车的点
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
//						(l_FrameInfo1.Ptdata[l_u16index].n32xLeft > g_FarMinWide && g_XdistanceV[i] > g_FarMinWide))) //侧装方式，在有车区域的第一个点和最后一个点均值隔离带一侧
//						  || (g_sspSetup.u8InstallFlag && ((l_FrameInfo1.Ptdata[l_u16index].n32xLeft<0 && g_XdistanceV[i]<0) ||
//						     (l_FrameInfo1.Ptdata[l_u16index].n32xLeft>0 && g_XdistanceV[i]>0))) ) //防止正装跨隔离带,正装隔离带两边值符合不同 20140217 修改
//					{
//						l_FrameInfo1.Ptdata[l_u16index].n32xRight = g_XdistanceV[i];
//						l_FrameInfo1.Ptdata[l_u16index].u16Rightpt = i;
//						l_FrameInfo1.Ptdata[l_u16index].u16xDis = abs(g_XdistanceV[i] - l_FrameInfo1.Ptdata[l_u16index].n32xLeft);
//					
//						l_32tmp = l_FrameInfo1.Ptdata[l_u16index].u16xMaxHt;	
//						if(g_ZdistanceV[i] > l_32tmp)				
//							l_FrameInfo1.Ptdata[l_u16index].u16xMaxHt = g_ZdistanceV[i];  //取最大值为高
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
//  	if(l_FrameInfo1.u8Sum)	 //针对当前帧的最后一个区域 （因为当前帧的最后一个区域在找区域时没有出来）
//	{
//		l_u16index = l_FrameInfo1.u8Sum - 1;
//		l_u16tmp = abs(l_FrameInfo1.Ptdata[l_u16index].n32xLeft - l_FrameInfo1.Ptdata[l_u16index].n32xRight);
//		l_FrameInfo1.Ptdata[l_u16index].u16xDis = l_u16tmp;
//		
//		if(!ISVehRegion(l_u16tmp, &l_FrameInfo1.Ptdata[l_u16index], g_ZdistanceV)) 	  //有效宽度小于300mm , 有车区域点数小于最少点数认,为是无效区域  
//		{		   
//			memset(&l_FrameInfo1.Ptdata[l_u16index],0,sizeof(PointStruct));
//			l_FrameInfo1.u8Sum--;					
//		}
//	}
//
//  //消除当前帧中区域中有重叠区域的部分
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
//  //合并区域，根据车道的宽度，在一个车道宽度内不能出现2辆或以上的车
//  	if(!g_sspSetup.u8InstallFlag)  //侧装方式
//	{
//		RegionMerging(&l_FrameInfo1);
//	}
//	else	  //正装方式
//	{
//		RegionMergingEx(&l_FrameInfo1);		
//	}
//
///********************************宽高激光有车区域查找结束***************************************/
//
//   //对区域进行匹配操作
//   for(l_u16index = 0;l_u16index < l_FrameInfo1.u8Sum;l_u16index++)
//   {
//      	if (l_u8Count++ > g_sspSetup.u8LaneNum)		 //计数值大于车道数，则跳出循环，防止错误或进入死循环
//		{	
//			l_u8Count = 0;
//			break;
//		}
//
//   		l_32tmp = 0;  //匹配标识，成功置1。
//		l_leftX = l_FrameInfo1.Ptdata[l_u16index].n32xLeft;	   //每帧中有车部分的起点  
//		l_rightX = l_FrameInfo1.Ptdata[l_u16index].n32xRight;	//有车部分的结束点	
//		l_leftXpt = l_FrameInfo1.Ptdata[l_u16index].u16Leftpt;
//		l_rightXpt = l_FrameInfo1.Ptdata[l_u16index].u16Rightpt;
//
//		if (l_leftXpt >= l_rightXpt) //如果左边点索引大于右边点，则有车区域不合理 20140217 增加
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
//			i = (g_VehicleSetIndex[j] - 1) & VEHICLE_MASK;	//20140217 	将g_VehicleSetIndex[j] - 1修改为(g_VehicleSetIndex[j] - 1) & VEHICLE_MASK
//			if(g_VehicleSet[i].u8Vstate == OCCURING_USED || (g_VehicleSet[i].u8Vstate == NO_USED))
//			{
//				l_32tmp2 =g_VehicleSet[i].Vdata.u16FrameCnt &  FRAME_MASK;
//				if(IS_INSIDE(l_leftX,l_rightX,g_VehicleSet[i].locateX.n32xLeft,g_VehicleSet[i].locateX.n32xRight) ||
//				  (RegionMatch_Point(&g_VehicleSet[i].VLocateX, &g_VehicleSet[i].locateX, &l_FrameInfo1.Ptdata[l_u16index], 0))
//				  || (l_32tmp2>=2 && abs(l_rightX-l_leftX)*2<g_VehicleSet[i].Vdata.xMax[l_32tmp2-2] && g_VehicleSet[i].Vdata.xMax[l_32tmp2-1]<g_VehicleSet[i].Vdata.xMax[l_32tmp2-2]*2/3 && IS_INSIDE(l_leftX,l_rightX,g_VehicleSet[i].Vdata.xdata[l_32tmp2-2][1],g_VehicleSet[i].Vdata.xdata[l_32tmp2-2][g_VehicleSet[i].Vdata.xdata[l_32tmp2-2][0]])))	 //点匹配不能跨激光器进行
//				{
//					l_32tmp = 1;  //区域匹配成功
//					g_VehicleSet[i].u8Vstate = OCCURING_USED;	
//					if(l_FrameInfo1.Ptdata[l_u16index].u16xDis <= g_LaneWide)
//					{  //单车
//						i = RegionMatch(l_leftX, l_rightX, g_VehicleSet, i); //寻找当前区域与前一帧的最佳匹配区域
//						//20140217 增加
//						if (i == ERRORVALUE || i > VEHICLE_MASK)  //返回值错误，跳出循环
//						{
//							break;
//						}								
//						g_VehicleSet[i].locateX.n32xLeft = min(l_leftX,g_VehicleSet[i].locateX.n32xLeft);	 //	宽度起始点值改变 需要修改
//						g_VehicleSet[i].locateX.n32xRight = max(l_rightX,g_VehicleSet[i].locateX.n32xRight);  //	宽度结束点值改变  需要修改
//						//修改 20140805
//						if (l_rightX - l_leftX > MINWIDE_THRESHOLD ||
//						   (!g_sspSetup.u8InstallFlag))	 //当前车辆区域宽度大于阈值,更新   或者侧装
//						{
//							g_VehicleSet[i].locateX.n32xLeft  = l_leftX;
//							g_VehicleSet[i].locateX.n32xRight = l_rightX;
//						}
//						l_32tmp2 =g_VehicleSet[i].Vdata.u16FrameCnt &  FRAME_MASK;
//						l_leftPt  = l_FrameInfo1.Ptdata[l_u16index].u16Leftpt;
//						l_rightPt = l_FrameInfo1.Ptdata[l_u16index].u16Rightpt;
//						if(l_32tmp2 > 0 && Time_Vertical == g_VehicleSet[i].Vdata.tdata[l_32tmp2 - 1] && l_rightPt>l_leftPt)
//						{
//							//同一帧中，有两个区域是同一车的   需要修改
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
//							if((l_rightPt - l_leftPt+1)>=0 && (l_rightPt - l_leftPt+1)<=(FRAME_BUFLEN-1))	   // 判断越界erro1
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
//									if((l_rightPt - l_leftPt+1)>=0 && (l_rightPt - l_leftPt+1)<=(FRAME_BUFLEN-1))	   // 判断越界
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
//						//并车 
//						if((g_sspSetup.u8LaneNum==6) && abs(g_VehicleSet[i].locateX.n32xRight - l_rightX) > VEHICHLE_DISTANT_GAP && 
//						   abs(g_VehicleSet[i].locateX.n32xLeft - l_leftX) > VEHICHLE_DISTANT_GAP &&
//						   g_VehicleSet[i].locateX.n32xRight < l_rightX &&
//						   g_VehicleSet[i].locateX.n32xLeft > l_leftX)	//6车道可能有中间区域匹配 前一帧的车辆区域在当前区域的中间部分
//						   {
//							   	//区域分割成两个不同的区域
//								//原来车辆帧数据   								
//								l_leftPt  = GetPosFromXDistance(g_XdistanceV,&g_VehicleSet[i].locateX,&l_FrameInfo1.Ptdata[l_u16index],0);
//								l_rightPt = GetPosFromXDistance(g_XdistanceV,&g_VehicleSet[i].locateX,&l_FrameInfo1.Ptdata[l_u16index],1);   //此位置不支持最低点在车道上
//								//20140217 增加对返回值的判断
//								if (l_leftPt >= POINT_SUM-1 || l_rightPt >= POINT_SUM-1)
//								{
//									break;	  //返回值异常，跳出循环
//								}
//
//								if (l_rightPt>l_leftPt)
//								{
//								   	if(g_VehicleSet[i].Vdata.u16FrameCnt < FRAME_MASK)
//									{	
//										if((l_rightPt - l_leftPt+1)>=0 && (l_rightPt - l_leftPt+1)<=(FRAME_BUFLEN-1))	   // 判断越界
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
//								//加入一个新的区域
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
//								//修改原区域					
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
//						   else if(abs(g_VehicleSet[i].locateX.n32xRight - l_rightX) < abs(g_VehicleSet[i].locateX.n32xLeft - l_leftX))  //前一帧车辆区域在当前并车区域的右边
//						   {
// 								l_leftPt  = GetPosFromXDistance(g_XdistanceV,&g_VehicleSet[i].locateX,&l_FrameInfo1.Ptdata[l_u16index],3);
//								l_rightPt = l_FrameInfo1.Ptdata[l_u16index].u16Rightpt;   		//此位置不支持最低点在车道上
//
//								//20140217 增加对返回值的判断
//								if (l_leftPt >= POINT_SUM-1 || l_rightPt >= POINT_SUM-1)
//								{
//									break;	  //返回值异常，跳出循环
//								}
//
//								if (l_rightPt > l_leftPt)
//								{
//								   	if(g_VehicleSet[i].Vdata.u16FrameCnt < FRAME_MASK )
//									{
//										if((l_rightPt - l_leftPt+1)>=0 && (l_rightPt - l_leftPt+1)<=(FRAME_BUFLEN-1))	   // 判断越界
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
//							   	 //区域更新	  	
//								 l_FrameInfo1.Ptdata[l_u16index].n32xRight = g_VehicleSet[i].locateX.n32xLeft;
//								if (g_u8JGFXFlag)  //相对于零点，激光器扫描点数增加方向与x轴值方向一致
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
//						   else   //前一帧车辆区域在当前区域的左边
//						   {
//								l_leftPt = l_FrameInfo1.Ptdata[l_u16index].u16Leftpt;
//								
//								l_rightPt = GetPosFromXDistance(g_XdistanceV,&g_VehicleSet[i].locateX,&l_FrameInfo1.Ptdata[l_u16index],2); //此位置不支持最低点在车道上
//								//20140217 增加对返回值的判断
//								if (l_leftPt >= POINT_SUM-1 || l_rightPt >= POINT_SUM-1)
//								{
//									break;	  //返回值异常，跳出循环
//								}
//
//								if (l_rightPt > l_leftPt)
//								{
//									if(g_VehicleSet[i].Vdata.u16FrameCnt < FRAME_MASK)
//									{
//										if((l_rightPt - l_leftPt+1)>=0 && (l_rightPt - l_leftPt+1)<=(FRAME_BUFLEN-1))	   // 判断越界
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
//								//区域更新	  	
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
//		if(l_32tmp == 0)  //新车进入
//		{
//			if(l_FrameInfo1.Ptdata[l_u16index].u16xDis > g_LaneWide)  //大于车道宽度，认为是并车
//			{ 	
//				l_leftX = l_FrameInfo1.Ptdata[l_u16index].n32xLeft;
//				l_rightX = l_FrameInfo1.Ptdata[l_u16index].n32xRight;	
//				l_leftPt = l_FrameInfo1.Ptdata[l_u16index].u16Leftpt;
//				l_rightPt = l_FrameInfo1.Ptdata[l_u16index].u16Rightpt;		
//				l_u16PosVect[l_32tmp].n32xLeft = l_leftX; //第一个位置存放原始位置	
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
//					   	l_u16PosVect[l_32tmp].n32xLeft = g_XdistanceV[l_32tmp2]; //第一个位置存放原始位置	
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
//			for(l_u16tmp = 0;l_u16tmp<l_32tmp;l_u16tmp++)	   //有车区域
//			{							
//			  	//该区域未匹配成功，认为有新车
//				for(i = 0;i < VEHICLE_MAX;i++)
//				{
//					if(g_VehicleSet[i].u8Vstate == NO_USED)
//					{
//						if (g_totalVehicle >= 10)	//20140217 修改
//						{
//							clearVehicleErr();
//						}
//						if (g_totalVehicle >= VEHICLE_MAX)	 //20140217 增加 如果车辆数大于最大车辆数，则跳出循环，防止越界
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
//							if((l_rightPt - l_leftPt+1)>=0 && (l_rightPt - l_leftPt+1)<=(FRAME_BUFLEN-1))	   // 判断越界
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
///*****************分车第一阶段：预分车------找有效数据区域，将有效数据存到TmpNum[k][]数组中******************/
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
//					l_FrameInfo.IncPtdata[l_u16index - 1].u16yDis = l_u16tmp;	  //该帧数据中有车部分的宽度				
//				 	l_u16StartYpt = l_FrameInfo.IncPtdata[l_u16index-1].u16Pt1;
//					l_u16EndYpt = l_FrameInfo.IncPtdata[l_u16index-1].u16Pt2;
//					if (l_u16tmp>500 && l_u16EndYpt-l_u16StartYpt+1>=3)
//					{
//						l_FrameInfo.u8Sum = (l_FrameInfo.u8Sum+1)&POINTSET_MASK;
//					} //中间出现间断点的情况
//					else if ((abs(l_u16EndYpt-l_FrameInfo.IncPtdata[l_u16index].u16Pt1)<=3 && abs(l_FrameInfo.IncPtdata[l_u16index].n32y1 - l_FrameInfo.IncPtdata[l_u16index-1].n32y2)<800)||
//						(abs(l_u16EndYpt-l_FrameInfo.IncPtdata[l_u16index].u16Pt1)<=10 && (abs(g_ZdistanceI[i]-l_FrameInfo.IncPtdata[l_u16index-1].u16yMaxHt)<500 || abs(l_FrameInfo.IncPtdata[l_u16index].n32y1 - l_FrameInfo.IncPtdata[l_u16index-1].n32y2)<500)))
//					{
//						l_FrameInfo.IncPtdata[l_u16index-1].u16Pt2 = i;
//						l_FrameInfo.IncPtdata[l_u16index-1].n32y2 = g_YdistanceI[i];
//						l_FrameInfo.IncPtdata[l_u16index-1].u16yDis = abs(l_FrameInfo.IncPtdata[l_u16index-1].n32y2-l_FrameInfo.IncPtdata[l_u16index-1].n32y1);											
//						l_32tmp = l_FrameInfo.IncPtdata[l_u16index-1].u16yMaxHt;	
//						if(g_ZdistanceI[i] > l_32tmp)
//						{
//							l_FrameInfo.IncPtdata[l_u16index-1].u16yMaxHt = g_ZdistanceI[i];  //取最大值为高				
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
//					l_FrameInfo.IncPtdata[l_u16index].u16yMaxHt = g_ZdistanceI[i];  //取最大值为高				
//				}				
//			}			
//		}
//		else
//		{
//			l_u16index = l_FrameInfo.u8Sum;
//		}
//	}
//
//针对最后一个区域
//	if (l_FrameInfo.u8Sum)
//	{
//		l_u16index = l_FrameInfo.u8Sum - 1;
//		l_u16tmp = abs(l_FrameInfo.IncPtdata[l_u16index].n32y1 - l_FrameInfo.IncPtdata[l_u16index].n32y2);
//		l_FrameInfo.IncPtdata[l_u16index].u16yDis = l_u16tmp;	  //该帧数据中有车部分的宽度				
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
//						l_FrameInfo.IncPtdata[l_FrameInfo.u8Sum-2].u16yMaxHt = l_FrameInfo.IncPtdata[l_u16index].u16yMaxHt;  //取最大值为高				
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
//						l_FrameInfo.IncPtdata[l_FrameInfo.u8Sum-2].u16yMaxHt = l_FrameInfo.IncPtdata[l_u16index].u16yMaxHt;  //取最大值为高				
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
//	//区域合并
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
//				l_FrameInfo.IncPtdata[l_u16index].u16yMaxHt = l_FrameInfo.IncPtdata[l_u16index+1].u16yMaxHt;  //取最大值为高				
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
//				l_FrameInfo.IncPtdata[l_u16index].u16yMaxHt = l_FrameInfo.IncPtdata[l_u16index+1].u16yMaxHt;  //取最大值为高				
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
//				l_FrameInfo.IncPtdata[l_u16index].u16yMaxHt = l_FrameInfo.IncPtdata[l_u16index+1].u16yMaxHt;  //取最大值为高				
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
//				l_FrameInfo.IncPtdata[l_u16index].u16yMaxHt = l_FrameInfo.IncPtdata[l_u16index+1].u16yMaxHt;  //取最大值为高				
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
///*******标记每一辆车中车头、车尾是否打飞*********/
//	for(l_index = 0;l_index < l_FrameInfo.u8Sum;l_index++)
//	{
//		l_FrameInfo.IncPtdata[l_index].u8DaFeiFlag1 = 0;
//		l_FrameInfo.IncPtdata[l_index].u8DaFeiFlag2 = 0;
//		for(i=1;i<=DafeiData[0][0]+DafeiData[0][1];i++)//20140919
//		{
//			if((DafeiData[i][0]<=l_FrameInfo.IncPtdata[l_index].u16Pt2)&&(DafeiData[i][1]>=l_FrameInfo.IncPtdata[l_index].u16Pt2)) //检查打飞段的终止点是否与车的起始点相同，若相同则车头打飞
//			{
//				l_FrameInfo.IncPtdata[l_index].u8DaFeiFlag1 = 1;
//			}
//			if((DafeiData[i][0]<=l_FrameInfo.IncPtdata[l_index].u16Pt1)&&(DafeiData[i][1]>=l_FrameInfo.IncPtdata[l_index].u16Pt1))		   //检查打飞段的起始点是否与车的结尾点相同，若相同则车尾打飞
//			{
//				l_FrameInfo.IncPtdata[l_index].u8DaFeiFlag2 = 1;
//			}
//		}	
//	}
//JG_T0TC[2] = T0TC;
//JG_counter2[2]=t0_count2;	
//计算车长、车高、车头时间、车头车尾X坐标、车头点车尾点
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
//			l_FrameInfo.IncPtdata[l_index].u16yMaxHt = Tmp_Z;	    //车高
//			
//			//*用车头最前面的点作为车头位置*/	
//			MaxX=l_FrameInfo.IncPtdata[l_index].n32y2;
//			Maxi=l_FrameInfo.IncPtdata[l_index].u16Pt2;
//			for(i=l_u16StartYpt;i<=l_u16EndYpt;i++)
//			{
//				if(g_YdistanceI[i]>MaxX && g_ZdistanceI[i]>=ThresVehLow)	//20140327 Z要大于阈值
//				{
//					MaxX=g_YdistanceI[i];
//					Maxi=i;
//				}
//			}
//			l_FrameInfo.IncPtdata[l_index].n32y2 = MaxX;
//			l_FrameInfo.IncPtdata[l_index].u16Pt2 = Maxi;
//			//*寻找车头最前点结束*/
//
//
//			//*用车尾最前面的点作为车尾位置*/	
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
//			//*寻找车尾最前点结束*/		
//
//
//						
//			if(l_FrameInfo.IncPtdata[l_index].n32y1<0&&l_FrameInfo.IncPtdata[l_index].n32y2>0)		 //只是当车头位置在X轴正向，车尾位置在X轴负向时才计算车辆的长度
//			{
//				l_FrameInfo.IncPtdata[l_index].u16yDis=l_FrameInfo.IncPtdata[l_index].n32y2-l_FrameInfo.IncPtdata[l_index].n32y1;  //车长
//			}
//			else 
//			{
//				l_FrameInfo.IncPtdata[l_index].u16yDis = 0;
//			}		
///*车头时间修正在后面计算*/								
//		}
//		else 
//		{
//			l_FrameInfo.uValid[l_index] = 0;		   //剔除未进入检测区域的车
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
//车辆匹配
//	for(l_u16index = 0; l_u16index < l_FrameInfo.u8Sum; l_u16index++)
//	{
//		l_32tmp = 0;  //匹配标识，成功置1。
//		l_n32StartY = l_FrameInfo.IncPtdata[l_u16index].n32y1;
//		l_n32EndY = l_FrameInfo.IncPtdata[l_u16index].n32y2;
//		l_u16StartYpt = l_FrameInfo.IncPtdata[l_u16index].u16Pt1;
//		l_u16EndYpt = l_FrameInfo.IncPtdata[l_u16index].u16Pt2;
//		if (l_u16EndYpt <= l_u16StartYpt ||	l_n32EndY <= l_n32StartY)
//		{
//			continue;
//		}
//		
//		if (l_n32EndY>=EnterX1 && l_n32StartY<=0)			//车头过了EnterX1&&车尾未出正下方
//		{
//			for(j = 0;j < g_VehIncTotal;j++)//与已有的顺扫激光器中存储的车辆数据进行匹配
//			{
//				i = g_VehIncSetIndex[j] -1;		
//				if(g_VehIncSet[i].u8Istate == OCCURING_USED)
//				{
//					l_32tmp2 = g_VehIncSet[i].Idata.u16FrameCnt &  FRAME_MASK;
//					if (IsInIncSide(l_n32StartY, l_n32EndY, g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][1],g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][0])
//						|| (l_n32EndY<0 && abs(l_n32EndY-g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][0])<1500 && abs(l_u16StartYpt-g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][3])<=10 && g_ZdistanceI[l_u16StartYpt]+1000<l_FrameInfo.IncPtdata[l_u16index].u16yMaxHt)
//						|| (l_n32EndY>0 && abs(l_n32StartY-g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][1])<1500 && g_ZdistanceI[l_u16EndYpt]+1000<l_FrameInfo.IncPtdata[l_u16index].u16yMaxHt))
//					{
//						 l_32tmp = 1;  //区域匹配成功
//						 if ((l_n32StartY>=0 && abs(l_n32StartY-g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][1])>50)//车尾
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
//						//对车高2000以上，车长6000以上的有问题数据帧进行处理，主要针对车头
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
//						l_32tmp = 1;  //区域匹配成功
//						if ((l_n32StartY>=0 && abs(l_n32StartY-g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-2][1])>50)//车尾
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
//						if (g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][1]<g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-2][1]-2500 && l_n32StartY-g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-2][1]-2500)//对异常存储的帧进行修正20140914
//						{
//							g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][1] = (g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-2][1]+l_n32StartY)/2;
//							g_VehIncSet[i].Idata.yMax[l_32tmp2-1] = abs(g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-2][0] - g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-2][1]);
//						}
//					}
//					break;
//				}										
//			
//			}
//			if (0 == l_32tmp && l_FrameInfo.IncPtdata[l_u16index].n32y1<0)	 //新车进入
//			{
//				memcpy(&l_u16IncPosVect[0],&l_FrameInfo.IncPtdata[l_u16index],sizeof(IncPtSt)); 				
//				l_32tmp = 1;				
//				for(l_u16tmp = 0;l_u16tmp<l_32tmp;l_u16tmp++)
//				{
//					//该区域未匹配成功，认为有新车
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
//抛车计算车速及单帧车长
//	for(j = 0;j < g_VehIncTotal;j++)
//	{
//		i = (g_VehIncSetIndex[j] - 1) & VEHICLE_MASK;	//20140217 	将g_VehicleSetIndex[j] - 1修改为(g_VehicleSetIndex[j] - 1) & VEHICLE_MASK
//		if (g_VehIncSet[i].u8Istate != NO_USED)
//		{
//			l_32tmp2 = g_VehIncSet[i].Idata.u16FrameCnt & FRAME_MASK;
//			if( l_32tmp2 >0
//				&& (g_VehIncSet[i].IemptFrame > NORMAL_MAX_EMPTYFRAME 
//				&&(g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][1]>ExitX4)))
//			{ 		
//			   g_VehIncSet[i].u8Istate = PASSED_USED;  //已结束，可收尾的车
//			}
//			else if (l_32tmp2 >0 && (0 == g_VehIncSet[i].u8ThrowFlag) 
//				&& g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][0]>ExitX3)   //车头过线
//			{
//			   g_VehIncSet[i].speed = GetVehSpeed(&g_VehIncSet[i], EnterX2, EnterX1);	//车头速度
//			   g_VehIncSet[i].u8ThrowFlag = 1;
//			}
//			else if (l_32tmp2 >0 && g_VehIncSet[i].IemptFrame > ERR_MAX_EMPTYFRAME)
//			{
//				memset(&g_VehIncSet[i], 0,sizeof(VehIncSt)); //清除顺道激光器该记录
//				g_VehIncSet[i].u8Istate = NO_USED; 
//				for(l_u16tmp = j;l_u16tmp < g_VehIncTotal - 1;l_u16tmp++)
//					g_VehIncSetIndex[l_u16tmp] = g_VehIncSetIndex[l_u16tmp+1];
//					
//				if (g_VehIncTotal <= VEHICLE_MAX)  //20140217 修改
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
//				memset(&g_VehIncSet[i], 0,sizeof(VehIncSt)); //清除顺道激光器该记录
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
//车辆长度计算
////////车头过了Lengthline线后计时开始/////////
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
//				g_VehIncSet[i].nStartTime =	g_VehIncSet[i].Idata.tdata[l_32tmp2-1];      //开始时间为车头时间，已经修正过的时间
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
//						- (g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][2]-g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][3])*10000/360; //结束时间
//					
//					g_VehIncSet[i].yLen= g_VehIncSet[i].speed*(g_VehIncSet[i].nEndTime-g_VehIncSet[i].nStartTime)/36000 
//						- abs(g_VehIncSet[i].ndeltaY-g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][1]);  //车辆长度
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
//			&& abs(g_YdistanceI[i]-g_YdistanceI[i+1])<8000) //20140107高度与两点水平距离限制
//	{
//		tempcout = 0;								 //20140106
//		while(i <= (g_u16InclineEndAnglePt - g_u16InclineStartAnglePt))  //20140327 注意结束点位置	原来为(i<=g_u16InclineEndAnglePt)
//		  {
//		    if((g_ZdistanceI[i] > ThresVehLow) && (g_ZdistanceI[i] < ThresVehHigh) 
//				&& ((tempcout==0)||(tempcout>0 && g_YdistanceI[i]-g_YdistanceI[i-1]<2000)))//20140122两点之间的X值限制
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
//		   	||(nocar>0 && (g_YdistanceI[i]-g_YdistanceI[Fenche1]>2000)))	//连续四个点不满足车高阈值，或有一个以上的点不满足车高阈值同时该点与之前满足车高阈值的X差值大于2米的情况下，认为前后属于不同的车
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
//		   CurrentCarNum=k;							 //变量CurrentCarNum记录此帧中统计的车数；
//		}
//	}	
//
///***************************分车第一阶段结束************************************************/
//
///**********************************************************************************************************************/
//
//     for(m=0;m<TOTALCAR;m++)
//	  {
//	  if(Lane_Vertical[m][0][0]!=0)
//	    {
//		Lane_Vertical[m][0][6]=1;			 //标志位初始化：标识该数组在此轮中有没有数据存入，无数据存入：1；有数据存入：0
//		}					  
//	  } 
//
///**************************************识车存储数据*************************************************/
//      for(k=0;k<CurrentCarNum;k++)
//	  {
//		if(Tmp_Lane[k][6]==0)		 //对于车头或车尾在EnterX1与垂直激光器之间的车辆
//		 { 
//
//		  CurrentCar_UseFlag=0;	     //比对是否成功的标志位，初始值为零
//
//		  /****在历史存车数据中找与当前数据匹配的车，并存入历史车辆中*****/
//		  for(m=0;m<TOTALCAR;m++)
//		  {
//		     tempcout=g_VehIncSet[m].Idata.u16FrameCnt;			  //将存车数组中的帧数赋值给tempcout
//			if(tempcout!=0)
//		    {												 
//				if(((Tmp_Lane[k][3]>=0)&&(abs(Tmp_Lane[k][3]-g_VehIncSet[m].Idata.ydataInfo[tempcout-1][0])<3000))
//				     ||((Tmp_Lane[k][4]<=0)&&(abs(Tmp_Lane[k][4]-g_VehIncSet[m].Idata.ydataInfo[tempcout-1][1])<3000))
//					 ||((Tmp_Lane[k][3]<0)&&(Tmp_Lane[k][4]>0)&&((abs(Tmp_Lane[k][3]-g_VehIncSet[m].Idata.ydataInfo[tempcout-1][0])<3000)&&(abs(Tmp_Lane[k][4]-g_VehIncSet[k].Idata.ydataInfo[tempcout-1][1])<3000)))) //若车头和车尾与记录过的车头和车尾，位置差小于1米
//			   	{
//				    if(tempcout>=2 && (Tmp_Lane[k][3]<0) 							  //20140326
//						&& abs(Tmp_Lane[k][3]-g_VehIncSet[k].Idata.ydataInfo[tempcout-1][0])<1000 
//						&& abs(Tmp_Lane[k][4]-g_VehIncSet[k].Idata.ydataInfo[tempcout-1][1])>=1000
//						&& abs(g_VehIncSet[m].Idata.ydataInfo[tempcout-1][1] - g_VehIncSet[m].Idata.ydataInfo[tempcout-2][1])<1000)
//					{
//						if(k+1<CurrentCarNum 
//							&& abs(Tmp_Lane[k+1][4] - g_VehIncSet[k].Idata.ydataInfo[tempcout-1][1])<1000)
//						{
//							Tmp_Lane[k][4] = Tmp_Lane[k+1][4];	 //车头点坐标
//							Tmp_Lane[k][12] = Tmp_Lane[k+1][12]; //车头点位置						
//						
//						    //时间、车长、车高、车头位置、车尾位置
//							g_VehIncSet[m].Idata.tdata[tempcout] = Tmp_Lane[k][0];
//							g_VehIncSet[m].Idata.yMax[tempcout] =  Tmp_Lane[k][1];
//							g_VehIncSet[m].Idata.zMax[tempcout] =  Tmp_Lane[k][2];
//							g_VehIncSet[m].Idata.ydataInfo[tempcout][0] =  Tmp_Lane[k][3];//车头位置
//							g_VehIncSet[m].Idata.ydataInfo[tempcout][1] =  Tmp_Lane[k][4];
//							memcpy(&g_VehIncSet[m].Idata.ydataInfo[tempcout][2], &Tmp_Lane[k][9],4*sizeof(int32));
//																					
//							Tmp_Lane[k+1][6] == 1;						//20140326剔除
//							g_VehIncSet[m].Idata.u16FrameCnt = g_VehIncSet[k].Idata.u16FrameCnt+1;				//帧数加1
//							Lane_Vertical[m][0][6]=0;					//该数组有数据存入
//							CurrentCar_UseFlag=1;						//已记录过的车
//							continue;							
//						}
//						else if (k==CurrentCarNum-1 
//							&& Tmp_Lane[k][12]-g_VehIncSet[m].Idata.ydataInfo[tempcout-1][2]>=2  //点数大于原点数2个以上
//							&& Tmp_Lane[k][4]>g_VehIncSet[m].Idata.ydataInfo[tempcout-1][1])	//车头X值与历史X值比较
//						{
//							j = -1;
//							for(i=DafeiData[0][0]; i<=DafeiData[0][1]; i++)
//							{													
//								if(DafeiData[i][0] + g_u16InclineStartAnglePt >=g_VehIncSet[m].Idata.ydataInfo[tempcout-1][2] //车头打飞点下限
//									&& DafeiData[i][0] + g_u16InclineStartAnglePt <= Tmp_Lane[k][12]) //车头打飞点上限
//								{
//									j = DafeiData[i][0];
//									break;
//								}
//
//							}
//							if(j != -1)//找到了距离最近的打飞点
//							{
//								for(i=j-1; i>=g_VehIncSet[m].Idata.ydataInfo[tempcout-1][2]-g_u16InclineStartAnglePt; i--)
//								{
//								 	if(g_ZdistanceI[i]>=ThresVehLow)		//找到了第一个有车高的点
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
//					   //只有当前后两帧的车头位置相差50mm时，才会将新的一帧数据记录入存车数组
//				    if(((abs(Tmp_Lane[k][3]-g_VehIncSet[k].Idata.ydataInfo[tempcout-1][0])>50)&&(Tmp_Lane[k][3]>=0))
//					   ||((abs(Tmp_Lane[k][4]-g_VehIncSet[k].Idata.ydataInfo[tempcout-1][1])>50)&&(Tmp_Lane[k][4]<=0))
//					   ||((Tmp_Lane[k][3]<0)&&(Tmp_Lane[k][4]>0)&&((abs(Tmp_Lane[k][3]-g_VehIncSet[k].Idata.ydataInfo[tempcout-1][0])>50)||(abs(Tmp_Lane[k][4]-g_VehIncSet[k].Idata.ydataInfo[tempcout-1][1])>50))))	 //当车辆移动超过0.5分米时，记录新的一帧。
//					{
//					   for(i=0;i<5;i++)           //将扫描得到的数据添加入存车数组
//					    {		
//							Lane_Vertical[m][tempcout][i]=Tmp_Lane[k][i];
//						}
//						Lane_Vertical[m][tempcout][9] = Tmp_Lane[k][9];			//车头是否打飞标志位
//						Lane_Vertical[m][tempcout][10]= Tmp_Lane[k][10];		//车尾是否打飞标志位
//						Lane_Vertical[m][tempcout][5] = Tmp_Lane[k][11];		//车尾点
//						Lane_Vertical[m][tempcout][6] = Tmp_Lane[k][12];		//车头点
//					    //时间、车长、车高、车头位置、车尾位置
//						g_VehIncSet[m].Idata.tdata[tempcout] = Tmp_Lane[k][0];
//						g_VehIncSet[m].Idata.yMax[tempcout] =  Tmp_Lane[k][1];
//						g_VehIncSet[m].Idata.zMax[tempcout] =  Tmp_Lane[k][2];
//						g_VehIncSet[m].Idata.ydataInfo[tempcout][0] =  Tmp_Lane[k][3];//车头位置
//						g_VehIncSet[m].Idata.ydataInfo[tempcout][1] =  Tmp_Lane[k][4];
//						memcpy(&g_VehIncSet[m].Idata.ydataInfo[tempcout][2], &Tmp_Lane[k][9],4*sizeof(int32));
//		
//					   //**********************************/				   
//		             	Lane_Vertical[m][0][0]=Lane_Vertical[m][0][0]+1 ;				//帧数加1
//						g_VehIncSet[m].Idata.u16FrameCnt = g_VehIncSet[m].Idata.u16FrameCnt+1;				//帧数加1
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
//					Lane_Vertical[m][0][6]=0;										//标志位存0代表该数组有数据存入
//					CurrentCar_UseFlag=1;											//标记这辆车为记录过的车
//				    		
//			  	}				 
//			}
//		  }
//
//			  /****如果该车没有记录即比对不成功，则为新车，当该车的车尾位置小于零时，记录新车, 存入m值最小的空白存储空间*****/
//		    if((CurrentCar_UseFlag==0)&&(Tmp_Lane[k][3]<0)&& (Tmp_Lane[k][4]<0)) //20140124 (Tmp_Lane[k][4]<0)
//		    {
//			   	for(m=0;m<TOTALCAR;m++)		     //找到最小m值的空白存储空间
//			    {
//				  if(g_VehIncSet[m].Idata.u16FrameCnt==0)
//				     break;
//				}
//			    Lane_Vertical[m][0][0]=1;                       //暂时只有1帧
//				Lane_Vertical[m][0][5]=JG_time;                 //进入激光的时间记录
//				for(i=0;i<5;i++)                                 //将扫描得到的数据添加入存车数组
//				{		
//					Lane_Vertical[m][Lane_Vertical[m][0][0]][i]=Tmp_Lane[k][i];
//				}
//
//				Lane_Vertical[m][1][9] = Tmp_Lane[k][9];		//车头是否打飞标志位
//				Lane_Vertical[m][1][10]= Tmp_Lane[k][10];		//车尾是否打飞标志位
//				Lane_Vertical[m][1][5] = Tmp_Lane[k][11];		//车尾点
//				Lane_Vertical[m][1][6] = Tmp_Lane[k][12];		//车头点
//			    //时间、车长、车高、车头位置、车尾位置
//				g_VehIncSet[m].Idata.tdata[tempcout] = Tmp_Lane[k][0];
//				g_VehIncSet[m].Idata.yMax[tempcout] =  Tmp_Lane[k][1];
//				g_VehIncSet[m].Idata.zMax[tempcout] =  Tmp_Lane[k][2];
//				g_VehIncSet[m].Idata.ydataInfo[tempcout][0] =  Tmp_Lane[k][3];//车头位置
//				g_VehIncSet[m].Idata.ydataInfo[tempcout][1] =  Tmp_Lane[k][4];
//				memcpy(&g_VehIncSet[m].Idata.ydataInfo[tempcout][2], &Tmp_Lane[k][9],4*sizeof(int32));
//
//			    Lane_Vertical[m][0][6]=0;	                     // 标志有数据存入
//				g_VehIncSet[m].IemptFrame=0;	                     //新车记录
//				Lane_Vertical[m][0][0]=Lane_Vertical[m][0][0]+1;	   //帧加1
//				g_VehIncSet[m].Idata.u16FrameCnt = g_VehIncSet[m].Idata.u16FrameCnt+1;				//帧数加1
//						   
//			}
//		   }	  
//	     } 
///***********************************识车存储数据结束********************************************/
//
///*************此轮中若无数据存入Lane_Vertical[m][][]则Lane_Vettical[m][0][7]++********************/
//
///************************************************/
//
//
///****************车头到达ExitX3后抛车***************************/
//
//	for(m=0;m<TOTALCAR;m++)
//	{
//		if((g_VehIncSet[m].Idata.u16FrameCnt>4)
//			&&(Lane_Vertical[m][Lane_Vertical[m][0][0]-1][4]>ExitX3) 
//			&&(g_VehIncSet[m].u8Istate != PASSED_USED))
//		{		  
//			g_nTmpVehSpeed = GetVehSpeed(m, EnterX2, EnterX1);	//车头速度
//			g_nTmpVehLength3 = get_vehicle_length(m);	  
//			g_VehIncSet[m].u8Istate = PASSED_USED;		   //抛车后，将抛车标志位置1
//		}	
//	}
///******************抛车逻辑结束*******************************/
//
//
///**********************剔除车辆：连续5次无数据存入、或车头行驶过了Exit3且已经抛车的、或第一帧数的车头过EnterX2的数则剔除****************************/
//	for(m=0;m<TOTALCAR;m++)	   //20140106
//	{
//		if(g_VehIncSet[m].Idata.u16FrameCnt>=2)
//		{			
//			if((g_VehIncSet[m].IemptFrame>4)
//			   ||((g_VehIncSet[m].u8Istate == PASSED_USED) && 
//			   		((g_sspSetup.LaserDistance<=ExitX3 && Lane_Vertical[m][Lane_Vertical[m][0][0]-1][3]>g_sspSetup.LaserDistance-500))
//						|| (g_sspSetup.LaserDistance>ExitX3 && Lane_Vertical[m][Lane_Vertical[m][0][0]-1][3]>ExitX3-500)))			
//			{
//				memset(Lane_Vertical[m],0,sizeof(Lane_Vertical[m])); //存储扫描帧信息的数组清理
//			}
//		}
//	}
//
///////////////////////////////垂直出车//////////////////////////////////
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
//		i = (g_VehicleSetIndex[j] - 1) & VEHICLE_MASK;	//20140217 	将g_VehicleSetIndex[j] - 1修改为(g_VehicleSetIndex[j] - 1) & VEHICLE_MASK
//		if(g_VehicleSet[i].u8Vstate != NO_USED)
//		{
//			if(g_VehicleSet[i].VemptFrame > NORMAL_MAX_EMPTYFRAME && g_VehicleSet[i].Vdata.u16FrameCnt)
//			{ 		
//			   g_VehicleSet[i].u8Vstate = PASSED_USED;  //已结束，可收尾的车
//			} 
//
//			if((g_VehicleSet[i].VemptFrame > ERR_MAX_EMPTYFRAME)) // 只有垂直激光器进车信息			   
//			{ 
//				//多车可以屏蔽改部分
//				if (g_VehicleSet[i].Vdata.u16FrameCnt < 10 && g_VehicleSet[i].Vdata.u16FrameCnt >=2)    //增加对车辆经过时绝大多数帧的点全部打飞情况处理
//				{
//					RetIsVehicle = ISVehicle(&g_VehicleSet[i].Vdata);
//				}
//				if(RetIsVehicle) 	 				   
//				{							  				   
//					//VehModels2(&g_VehicleSet[i]);
//					g_VehicleSet[i].u8Vstate = PASSED_USED;  //已结束，可收尾的车 
//				}//end
//				else
//				{
//					memset(&g_VehicleSet[i],0,sizeof(VehicleStruct));
//					g_VehicleSet[i].u8Vstate = NO_USED;  //误触发的车
//					for(l_u16tmp = j;l_u16tmp < g_totalVehicle - 1;l_u16tmp++)
//					   g_VehicleSetIndex[l_u16tmp] = g_VehicleSetIndex[l_u16tmp+1];
//					
//					if (g_totalVehicle <= VEHICLE_MAX)  //20140217 修改
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
//				//收尾处理。		
//				if (g_VehicleSet[i].Vdata.u16FrameCnt < 3)    //增加对车辆经过时绝大多数帧的点全部打飞情况处理
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
//				if (g_totalVehicle <= VEHICLE_MAX)  //20140217 修改
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
//	/********************垂直激光与顺道激光匹配处理***********************/
//	/*第一步：清除垂直激光器非匹配区域记录 */
//	/**********************************************************************/
//	垂直激光器已收尾车辆，locatex位置在非匹配区域，直接清除
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
//			if (g_totalVehicle <= VEHICLE_MAX)  //20140217 修改
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
//	/*第二步：开始匹配*/
//	/************************************************************/
//	匹配准则：长、高接近，且位置在匹配区域；匹配成功出车后双清
//	/*************************************************************/
//	for(j = 0;j < g_totalVehicle;j++)
//	{
//		i = g_VehicleSetIndex[j] -1;
//		if(g_VehicleSet[i].u8Vstate==PASSED_USED)		  //垂直
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
//							/*垂直激光长高求解*/
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
//								/*匹配成功*/
//								//更新垂直激光器车辆记录集的长、高和速度			 
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
//								/*单清顺车道*/
//								memset(&g_VehIncSet[k], 0,sizeof(VehIncSt)); //清除顺道激光器该记录
//								g_VehIncSet[k].u8Istate = NO_USED; 
//								for(l_u16tmp = m;l_u16tmp < g_VehIncTotal - 1;l_u16tmp++)
//									g_VehIncSetIndex[l_u16tmp] = g_VehIncSetIndex[l_u16tmp+1];
//
//								if (g_VehIncTotal <= VEHICLE_MAX)  //20140217 修改
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
//							memset(&g_VehIncSet[k], 0,sizeof(VehIncSt)); //清除顺道激光器该记录
//							g_VehIncSet[k].u8Istate = NO_USED; 
//							for(l_u16tmp = m;l_u16tmp < g_VehIncTotal - 1;l_u16tmp++)
//								g_VehIncSetIndex[l_u16tmp] = g_VehIncSetIndex[l_u16tmp+1];
//
//							if (g_VehIncTotal <= VEHICLE_MAX)  //20140217 修改
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
//			if (g_totalVehicle <= VEHICLE_MAX)  //20140217 修改
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

	int Distance1=2500;  //找离（EnterX1+Distanc1）线最近的车头非打飞帧作为计算速度的第一帧
	int Distance2=2500;  //测量2000cm距离内的速度差，以求出速度

	/*********计算车辆速度**********************/
	////////////////找两帧车头没有打飞的帧计算车辆的速度///////

	index = pVehInc->Idata.u16FrameCnt;
	SpeedFlag1=0;
	SpeedFlag2=0;
	MinD=10000;
	if (index>=2)
	{
		for(i=0; i<index; i++)			 //找到第一个车头没有打飞的帧
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
				MinD=10000;		  //初始值定为10米
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
			for(i=0; i<index; i++)			 //找到第一个车头没有打飞的帧
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
					MinD=10000;		  //初始值定为10米
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

				if((index-index_start)>2)	//若此条件不满足，则速度输出为零；
				{
					for(i=index_start+1;i<index;i++)	   //找距离（EnterX2-Distance）最近的点
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

//产生某个范围(start道end)内的伪随机数
int Myrand(int start, int end)
{
	int ret = start;
	//此处应使用时间
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



//小客车经过激光器时，只有1、2帧时的情况进行处理
uint8 ISVehicle(VehicleDataStruct* pVdata)
{
	uint8 Ret = 0;
	uint8 l_u8HeightPt = 1;   //每帧中高度相等的点数
	uint8 index = 0;
	uint8 indexFrame = 0;

	//20140217 增加对参数合法性判断
	if (pVdata == NULL)
	{
		return 0;  //为空，返回0	
	}

	for( indexFrame = 0; indexFrame < pVdata->u16FrameCnt; indexFrame++)
	{
		for (index = 1;pVdata->zdata[indexFrame][index] > ThresVehLow;index++) //20140226 修改
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

			if (index >= FRAME_BUFLEN - 1)	// 增加
				break;
		}			
	}

	//对于很多帧打飞，且每帧中的各点高不相等的处理方法，如果出车较多，可以屏蔽
	if (Ret <= 3)
	{
		if(max(pVdata->zdata[0][0], pVdata->zdata[1][0]) >= MIN_PTNUM && 
		   max(pVdata->xMax[0], pVdata->xMax[1]) > 0)  //只比较1、2两帧的点数和宽即可
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


//每帧中有车区域的有效区域判断，0表示无效，1表示有效  (需要改进）
uint8 ISVehRegion(const uint16 u16RegionWide, const PointStruct *pPtStruct, const int32* pZdistance)
{
//	uint8 Ret = 0;
//	uint16 l_u16Index = 0;
//	uint16 l_u16HightPtNum = 0;
//	uint16 l_u16LowPtNum   = 0;
//	//20140217 增加对参数合法性判断
//	if (pPtStruct == NULL || pZdistance == NULL)
//	{
//		return 0;
//	}
//
//	//增加 检查这个区间内高度异常情况
//	if (pPtStruct->u16Leftpt >= pPtStruct->u16Rightpt || pPtStruct->u16Rightpt >= POINT_SUM )
//	{	//数据异常
//		return 0;
//	}
//	for (l_u16Index = pPtStruct->u16Leftpt; l_u16Index <= pPtStruct->u16Rightpt; l_u16Index++)
//	{
//		if (pZdistance[l_u16Index] >= 5000) //高度超过5m的点
//		{
//			l_u16HightPtNum++;	
//		}
//		else if (pZdistance[l_u16Index] <= -500)	//高度在-0.5m，认为异常
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
//		//异常较多
//		return 0;
//	}
//	
//	if (pPtStruct->u16xMaxHt > 3000 &&pZdistance[pPtStruct->u16Leftpt] == pPtStruct->u16xMaxHt) //左边点是最大高度值
//	{
//		if ((g_sspSetup.u8InstallFlag &&(pPtStruct->n32xLeft-pPtStruct->n32xRight>2000)) ||
//			((g_sspSetup.u16EndPtNum < g_sspSetup.u16VerticalZeroPos)&&
//			(pPtStruct->n32xRight-pPtStruct->n32xLeft>2000)))
//		{//	左点值异常
//			return 0;	
//		}	
//	}
//	else if (pPtStruct->u16xMaxHt > 3000 && pZdistance[pPtStruct->u16Rightpt] == pPtStruct->u16xMaxHt)//右边点是最大高度值
//	{
//		if ((g_sspSetup.u8InstallFlag &&(pPtStruct->n32xLeft-pPtStruct->n32xRight>2000)) ||
//			((g_sspSetup.u16EndPtNum < g_sspSetup.u16VerticalZeroPos)&&
//			(pPtStruct->n32xLeft-pPtStruct->n32xRight>2000)))
//		{//	右点值异常
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
*****20130523  当前有车区域与前一帧进行最优匹配
**** 函数 RegionMatch*********
***  参数说明******************
*** u16LeftX  当前帧有车区域的起始点位置
*** u16RightX 当前帧有车区域的结束点位置 
***	pVehicle 车辆结构体，与全局变量g_VehicleSet相对应
*** u8Index  车辆索引
******************/
uint16 RegionMatch(int32 n32LeftX, int32 n32RightX, VehicleStruct *pVehicle, uint16 u16Index)
{
////	uint16 RetIndex = u16Index;	
//	//20140217 增加对参数合法性判断
//	if (pVehicle == NULL || u16Index > VEHICLE_MASK)
//	{
//		return ERRORVALUE; 
//	}
//
//	if (u16Index == VEHICLE_MAX-1)  //前一帧的最后一个有车区域,直接匹配成功
//	{
//		RetIndex = u16Index;	
//	}
//	else
//	{  //当前区域与前一帧中下一个区域匹配
//		if (IS_INSIDE(n32LeftX, n32RightX, pVehicle[u16Index+1].locateX.n32xLeft,pVehicle[u16Index+1].locateX.n32xRight)) //部分重叠
//		{
//			//当前区域与前一帧的两个区域都部分匹配
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
//		{	//完全不匹配，直接返回
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
//	if (u8Flag)	 //1表示倾斜激光器
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
//	    abs(pLocateX->n32xRight - pPtData->n32xRight) > 3000)	  //距离相差很大，匹配不成功
//	{
//		return ret;
//	}
//
//	if (u16PreRightpt-u16PreLeftpt > u16Rightpt-u16Leftpt)	//点数少的中心点落在点数多的范围内，说明匹配成功
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
////	//20140217 增加对参数合法性判断
////	if (pFrameInfo == NULL || pFrameInfo->u8Sum > POINTSET_CNT)
////	{
////		return;
////	}
////
////  //合并区域，根据车道的宽度，在一个车道宽度内不能出现2辆或以上的车，总区域数不能超过车道数	 (对4车道有效)
////	for (l_u16index = 1;l_u16index < pFrameInfo->u8Sum;l_u16index++)
////	{
////		if(abs(pFrameInfo->Ptdata[l_u16index-1].n32xLeft-pFrameInfo->Ptdata[l_u16index-1].n32xRight) >= g_LaneWide ||
////		   abs(pFrameInfo->Ptdata[l_u16index].n32xLeft-pFrameInfo->Ptdata[l_u16index].n32xRight) >= g_LaneWide)  //有一区域大于车道宽
////		{
////			if ((abs(pFrameInfo->Ptdata[l_u16index-1].n32xRight) < g_NearMaxWide &&
////			    abs(pFrameInfo->Ptdata[l_u16index].n32xRight) < g_NearMaxWide) ||
////				(abs(pFrameInfo->Ptdata[l_u16index-1].n32xLeft) > g_FarMinWide &&
////				abs(pFrameInfo->Ptdata[l_u16index].n32xLeft) > g_FarMinWide))  //合并同侧的区域
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
////				 abs(pFrameInfo->Ptdata[l_u16index+1].n32xLeft) > g_FarMinWide))) //3个区域同侧，且相邻区域宽度小于车道宽
////		{
////			//区域合并时，选择比较合适的2辆区域进行合并
////			if (abs(pFrameInfo->Ptdata[l_u16index-1].n32xLeft-pFrameInfo->Ptdata[l_u16index].n32xRight) <= 
////			    abs(pFrameInfo->Ptdata[l_u16index].n32xLeft-pFrameInfo->Ptdata[l_u16index+1].n32xRight))  //合并前两个相邻区域
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
////			else   //合并后两个相邻区域
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
////				(abs(pFrameInfo->Ptdata[l_u16index-1].n32xLeft) > g_FarMinWide && abs(pFrameInfo->Ptdata[l_u16index].n32xLeft) > g_FarMinWide)))	//同侧2个区域宽度小于车道宽，直接合并
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

//用于正装方式时，合并区域
void RegionMergingEx(PointSet *pFrameInfo)
{
	uint16 l_u16index = 1;
	uint16 l_u16tmp   = 1;
	uint8  l_u8Region1 = 0;
	uint8  l_u8Region2 = 0;

	//20140217 增加对参数合法性判断
	if (pFrameInfo == NULL || pFrameInfo->u8Sum > POINTSET_CNT)
	{
		return;
	}

  //合并区域，根据车道的宽度，在一个车道宽度内不能出现2辆或以上的车，总区域数不能超过车道数	 
	for (l_u16index = 1;l_u16index < pFrameInfo->u8Sum;l_u16index++)
	{
		if(abs(pFrameInfo->Ptdata[l_u16index-1].n32xLeft-pFrameInfo->Ptdata[l_u16index-1].n32xRight) >= g_LaneWide ||
		   abs(pFrameInfo->Ptdata[l_u16index].n32xLeft-pFrameInfo->Ptdata[l_u16index].n32xRight) >= g_LaneWide)  //有区域大于车道宽
		{
			if (((pFrameInfo->Ptdata[l_u16index-1].n32xRight + g_MedianLeftWide) < 0 &&			   //加上负号
				    (pFrameInfo->Ptdata[l_u16index].n32xRight+g_MedianLeftWide) < 0) ||			   //加上负号
					(pFrameInfo->Ptdata[l_u16index-1].n32xLeft > g_MedianRightWide &&
					pFrameInfo->Ptdata[l_u16index].n32xLeft > g_MedianRightWide))  //合并同侧的区域
			{
				if (g_sspSetup.u8LaneNum == 4)  //4车道
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
				else //6车道
				{
					if ((abs(pFrameInfo->Ptdata[l_u16index-1].n32xLeft-pFrameInfo->Ptdata[l_u16index].n32xRight) < g_sspSetup.LaneWide*2) ||
					    (abs(pFrameInfo->Ptdata[l_u16index-1].n32xLeft-pFrameInfo->Ptdata[l_u16index-1].n32xRight) >g_sspSetup.LaneWide*2) ||
						 abs(pFrameInfo->Ptdata[l_u16index].n32xLeft-pFrameInfo->Ptdata[l_u16index].n32xRight) >g_sspSetup.LaneWide*2) //2个区域和小于2个车道宽或是有一区域大于2个车道宽
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
				 (((pFrameInfo->Ptdata[l_u16index-1].n32xRight+g_MedianLeftWide) < 0 && 	 //负值
				 (pFrameInfo->Ptdata[l_u16index+1].n32xRight+g_MedianLeftWide) < 0) ||		 //加上负号
				 (pFrameInfo->Ptdata[l_u16index-1].n32xLeft > g_MedianRightWide &&
				 pFrameInfo->Ptdata[l_u16index+1].n32xLeft > g_MedianRightWide))) //3个区域同侧，且相邻区域宽度小于车道宽
		{
			//区域合并时，选择比较合适的2辆区域进行合并
			if (abs(pFrameInfo->Ptdata[l_u16index-1].n32xLeft-pFrameInfo->Ptdata[l_u16index].n32xRight) <= 
			    abs(pFrameInfo->Ptdata[l_u16index].n32xLeft-pFrameInfo->Ptdata[l_u16index+1].n32xRight))  //合并前两个相邻区域
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
			else   //合并后两个相邻区域
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
		        (((pFrameInfo->Ptdata[l_u16index-1].n32xRight+g_MedianLeftWide) < 0 && 		//加上负号
				(pFrameInfo->Ptdata[l_u16index].n32xRight+g_MedianLeftWide) < 0)||			//加上负号
				(pFrameInfo->Ptdata[l_u16index-1].n32xLeft > g_MedianRightWide && 
				pFrameInfo->Ptdata[l_u16index].n32xLeft > g_MedianRightWide)))	//同侧2个区域宽度小于车道宽，直接合并
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
//	uint16 Veh_Num    = 0;  //车辆帧数
//	uint32 VehHeight  = 0;
//	uint32  VehWide   = 0;
//	uint32 VehLength = 0;
//	uint16 u16Index  = 0;
//	uint32  VehSpeed = 0; 
//	uint8  VehPattern = 0;  //车型
//	uint8  u8Lane    = 0;	//车道
//	uint8  l_u8TestFrame = 0;
//
//	uint8 strsend[10] = {0};
//	static uint32 send_count = 1;
//	uint16  l_anTopPlanenessFlag[FRAME_MAXCNT] = {0};  //保存每帧的顶部平整标识，0表示不平整，1表示平整
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
//	_VehSendInfo VehSendInfo_tmp;	//UART1出单车结构体
//
//	uint32 day_temp = 0;
//	uint32 year_temp = 0;
//	uint8 WriteSDtemp[512] = {0};
// 
//	
//    //20140217 参数合法性判断
//	if (pVehicle == NULL)
//	{
//		return;
//	}
//
//	Veh_Num = pVehicle->Vdata.u16FrameCnt;
//	//20140217 增加
//	if (Veh_Num > FRAME_MAXCNT || Veh_Num < 1)
//	{
//		Veh_Num = 0;  //帧数超过最大帧数值，帧数赋值为0
//		pVehicle->Vdata.u16FrameCnt = 1;
//	}
//	
//	for( u16Index = 0; u16Index < Veh_Num; u16Index++)
//	{	//计算车宽
//		if (pVehicle->xLen < pVehicle->Vdata.xMax[u16Index])
//		{
//			pVehicle->xLen = pVehicle->Vdata.xMax[u16Index];	
//		}
//		//计算每帧的车高 
//		pVehicle->Vdata.zMax[u16Index] = GetVehHeight(&pVehicle->Vdata, u16Index);
//		//计算车高
//		if (pVehicle->zLen < pVehicle->Vdata.zMax[u16Index] && pVehicle->Vdata.zMax[u16Index] < 4800 )
//		{
//			pVehicle->zLen = pVehicle->Vdata.zMax[u16Index];
//		}		
//	}
//
//	// 由于偶尔出现毛刺，会有1、2帧的高度异常，导致车型判断错误，先剔除这些异常高度
//	for (u16Index = 0; u16Index < Veh_Num; u16Index++)
//	{
//		if (pVehicle->zLen - pVehicle->Vdata.zMax[u16Index] < 1000 && pVehicle->zLen > 3000 )	
//		{
//			l_u8TestFrame++;
//		}		
//	}
//	//检查帧数小于3，重新找车高
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
//	if (VehHeight > 2700 && Veh_Num < 3)  //大车并车可能有多中小客车  增加20131212
//	{
//		return;
//	}
//
//	//车道号  (需要修改）
//	if(g_sspSetup.LaneWide)
//	{
//		tmp1 = abs(pVehicle->locateX.n32xLeft+pVehicle->locateX.n32xRight)>>1;
//		if ( !g_sspSetup.u8InstallFlag && g_sspSetup.u8LaneNum < 6)  //侧装4车道
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
//		else if (g_sspSetup.u8LaneNum == 4)  //正装4车道
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
//		else //正装6车道
//		{	//20140829
//			if (abs(g_sspSetup.n32LaserHorizOff)>g_LaneWide  //偏移超过1个车道宽度
//				&& 0 == g_MedianLeftWide && 0 == g_MedianRightWide)	 //隔离带宽度为0
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
//					pVehicle->u8Lane = abs(tmp1 - g_MedianLeftWide) /g_LaneWide+1;		 //20131210车道加上1 不出车
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
//	 //1中小客 2小型货车 3 大客车 4中型货车 5大型货车 6特大型货车 7集装箱车  8拖拉机 9摩托车
//	 if(VehLength <  BIGANGSMALLTHR)//	20131223  将|| VehHeight < 2300删去
//	 {         
//	    //6000以下小车判断
//		VehPattern = GetLightVehPattern(pVehicle);		
//	 }
//	 else if (VehLength >= BIGANGSMALLTHR)
//	 {	 	
//	 	//6000以上大型车判断		
//		VehPattern = GetLargeVehPattern(pVehicle);	
//	 }
//
//
//	 //20130403 增加拖拉机用于测试
//	 if (VehPattern  < ZHONGXIAOKE)
//	 {
//	 	VehPattern = ZHONGXIAOKE;	
//	 }
//
//	 if (g_sspSetup.u8RoadType && (VehPattern == MOTUOCHE))	   //高速路上不出摩托车
//	 {
//	 	VehPattern = ZHONGXIAOKE;
//	 }
	//修正已经判出车型的车辆长、宽、速度等参数
	/**************************************************
	if (VehPattern <= XIAOHUOCHE)	//小客车、小货车
	{
		if (VehLength > BIGANGSMALLTHR || VehLength < 1340)		 //车长修正
		{
			VehLength = Myrand(3500, 5800);
		}
		if (VehWide < 1000 || VehWide > 2500) //车宽修正
		{
			VehWide = Myrand(1400, 2100);
		}
		if (VehHeight < 800 || VehHeight > 4000)  //车高修正
		{
			VehHeight = Myrand(1400, 2200);
		}
		if (g_sspSetup.u8RoadType)  //高速
		{
			if (VehPattern == ZHONGXIAOKE)
			{
				if (VehSpeed < 40 || VehSpeed > 150)	 //车速修正
				{
					VehSpeed = Myrand(80, 110);
				}
			}
			else
			{
				if (VehSpeed < 40 || VehSpeed > 110)	 //车速修正
				{
					VehSpeed = Myrand(65, 90);
				}				
			}
		}
		else
		{
			if (VehPattern == ZHONGXIAOKE)
			{
				if (VehSpeed < 1 || VehSpeed > 110)	 //车速修正
				{
					VehSpeed = Myrand(70, 100);
				}
			}
			else
			{
		   		if (VehSpeed < 1 || VehSpeed > 100)	 //车速修正
				{
					VehSpeed = Myrand(60, 80);
				}
			}			
		}
	}
	else if (VehPattern <= DAHUO)		//大客车、中货、大货
	{
		if (VehLength < BIGANGSMALLTHR || VehLength > 13000)		 //车长修正
		{
			VehLength = Myrand(6800, 11000);
		}
		if (VehWide < 1800 || VehWide > 3500) //车宽修正
		{
			VehWide = Myrand(1900, 2700);
		}
		if (VehHeight < 1800 || VehHeight > 5000)  //车高修正
		{
			VehHeight = Myrand(2100, 4500);
		}
		if (g_sspSetup.u8RoadType)  //高速
		{
			if (VehSpeed < 40 || VehSpeed > 90)	 //车速修正
			{
				VehSpeed = Myrand(65, 90);
			}
		}
		else
		{
			if (VehSpeed < 1 || VehSpeed > 90)	 //车速修正
			{
				VehSpeed = Myrand(50, 80);
			}			
		}
		if (VehHeight < 2300 || VehHeight > 4800)
		{
			VehHeight = Myrand(3100, 3450);
		}
	}
	else if (VehPattern <= JIZHUANGXIANG)	 //特大货车、集装箱
	{
		if (VehLength < 12000 || VehLength > 35000)		 //车长修正
		{
			VehLength = Myrand(12500, 18000);
		}
		if (VehWide < 2000 || VehWide > 3500) //车宽修正
		{
			VehWide = Myrand(2200, 3000);
		}
		if (VehHeight < 2600 || VehHeight > 5000)  //车高修正
		{
			if (VehPattern == TEDAHUO)
				VehHeight = Myrand(2800, 4500);
			else
				VehHeight = Myrand(3800, 4200);
		}
		if (g_sspSetup.u8RoadType)  //高速
		{
			if (VehSpeed < 30 || VehSpeed > 90)	 //车速修正
			{
				VehSpeed = Myrand(55, 80);
			}
		}
		else
		{
			if (VehSpeed < 1 || VehSpeed > 90)	 //车速修正
			{
				VehSpeed = Myrand(50, 75);
			}			
		}
	}
	else if (VehPattern == MOTUOCHE)	  //摩托车
	{
		if (VehLength >  2500 || VehLength < 500)		 //车长修正
		{
			VehLength = Myrand(1500, 2000);
		}
		if (VehWide < 500 || VehWide > 1500) //车宽修正
		{
			VehWide = Myrand(700, 1000);
		}
		if (VehHeight < 800 || VehHeight > 1800)  //车高修正
		{
			VehHeight = Myrand(900, 1600);
		}
		if (VehSpeed < 1 || VehSpeed > 70)	 //车速修正
		{
			VehSpeed = Myrand(40, 60);
		}		
	}
	else 								//拖拉机
	{
		if (VehLength < 5000 || VehLength > 7000)		 //车长修正
		{
			VehLength = Myrand(5000, 6000);
		}
		if (VehWide < 1400 || VehWide > 3500) //车宽修正
		{
			VehWide = Myrand(1700, 3000);
		}
		if (VehHeight < 800 || VehHeight > 4000)  //车高修正
		{
			VehHeight = Myrand(1700, 3000);
		}
		if (VehSpeed < 1 || VehSpeed > 60)	 //车速修正
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
//	 Veh_Info[0] = VehLength;	//长度
//	 Veh_Info[1] = VehWide;		//宽度
//	 Veh_Info[2] = VehHeight;	///高度
//	 Veh_Info[3] = VehSpeed;	//速度
//	 Veh_Info[4] = VehPattern;	//车型
//	 Veh_Info[5] = 0x12;		//年
//	 Veh_Info[6] = MONTH;		//月
//	 Veh_Info[7] = DAY;			//日
//	 Veh_Info[8] = HOUR;		//时
//	 Veh_Info[9] = MIN;			//分
//	 Veh_Info[10] = SEC;		//秒
//	 Veh_Info[11] = 0;//  机动车车头时距
//	 Veh_Info[12] =	0;//  是否跟车；
//     Veh_Info[13] =	0;//机动车车头间距
//	 Veh_Info[14] = 0;//通过垂直断面时间
//
//	switch(u8Lane)
//	{
//		case 1:
//			   Veh_Info[11] =  Get_shiju(Veh_Info,g_ai32Pre_Veh_Info_1_Lane);	 //机动车车头时距
//			   Veh_Info[11] =  (pVehicle->Vdata.tdata[0] - g_ai32Pre_Veh_Info_1_Lane[25])/1000;
//			   if(Veh_Info[11] < 0)
//			   {
//			    Veh_Info[11] = 20; 	 //单位：秒；
//			   }
//			   if(0==Veh_Info[11])
//			   {
//			   	Veh_Info[11] = 1;
//			   }
//			   if( Veh_Info[11] < DisTime )
//			   {
//			   	   Veh_Info[12] = 1;//1 跟车
//			   }
//			   else
//			   {
//			   	   Veh_Info[12] = 0;//0 不跟车
//			   }		  					   	   
//			   Veh_Info[13] =	Veh_Info[11] * VehSpeed/3.6;//机动车车头间距
//			   if((pVehicle->Vdata.tdata[pVehicle->Vdata.u16FrameCnt-1] - pVehicle->Vdata.tdata[0]) < 1000)
//			   {
//			   	   Veh_Info[14] = 1;//定为1S；
//			   }
//			   else
//			   {
//			   		Veh_Info[14] = pVehicle->Vdata.tdata[pVehicle->Vdata.u16FrameCnt-1] - pVehicle->Vdata.tdata[0];//通过垂直断面时间 ms
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
//			    Veh_Info[11] = 20; 	 //单位：秒；
//			   }
//			   if(0==Veh_Info[11])
//			   {
//			   	Veh_Info[11] = 1;
//			   }
//			   if( Veh_Info[11] < DisTime )
//			   {
//			   	   Veh_Info[12] = 1;//1 跟车
//			   }
//			   else
//			   {
//			   	   Veh_Info[12] = 0;//0 不跟车
//			   } // Veh_Info[12]	//判断是否跟车；		  					   	   
//			   Veh_Info[13] =	Veh_Info[11] * VehSpeed/3.6;//机动车车头间距
//			   if((pVehicle->Vdata.tdata[pVehicle->Vdata.u16FrameCnt-1] - pVehicle->Vdata.tdata[0]) < 1000)
//			   {
//			   	   Veh_Info[14] = 1;//定为1S；
//			   }
//			   else
//			   {
//			   		Veh_Info[14] = pVehicle->Vdata.tdata[pVehicle->Vdata.u16FrameCnt-1] - pVehicle->Vdata.tdata[0];//通过垂直断面时间 ms
//			   }
//			   memcpy(g_ai32Pre_Veh_Info_2_Lane,Veh_Info,sizeof(Veh_Info));
//			   g_ai32Pre_Veh_Info_2_Lane[25] = pVehicle->Vdata.tdata[0];
//			   break;
//		case 3:
//			   Veh_Info[11] =  Get_shiju(Veh_Info,g_ai32Pre_Veh_Info_3_Lane);
//			   Veh_Info[11] =  (pVehicle->Vdata.tdata[0] - g_ai32Pre_Veh_Info_3_Lane[25])/1000;
//			   if(Veh_Info[11] < 0)
//			   {
//			    Veh_Info[11] = 20; 	 //单位：秒；
//			   }
//			   if(0==Veh_Info[11])
//			   {
//			   	Veh_Info[11] = 1;
//			   }
//			   if( Veh_Info[11] < DisTime )
//			   {
//			   	   Veh_Info[12] = 1;//1 跟车
//			   }
//			   else
//			   {
//			   	   Veh_Info[12] = 0;//0 不跟车
//			   } // Veh_Info[12]	//判断是否跟车；		  					   	   
//			   Veh_Info[13] =	Veh_Info[11] * VehSpeed/3.6;//机动车车头间距
//			   if((pVehicle->Vdata.tdata[pVehicle->Vdata.u16FrameCnt-1] - pVehicle->Vdata.tdata[0]) < 1000)
//			   {
//			   	   Veh_Info[14] = 1;//定为1S；
//			   }
//			   else
//			   {
//			   		Veh_Info[14] = pVehicle->Vdata.tdata[pVehicle->Vdata.u16FrameCnt-1] - pVehicle->Vdata.tdata[0];//通过垂直断面时间 ms
//			   }
//			   memcpy(g_ai32Pre_Veh_Info_3_Lane,Veh_Info,sizeof(Veh_Info));
//			   g_ai32Pre_Veh_Info_3_Lane[25] = pVehicle->Vdata.tdata[0];
//			   break;
//		case 4:
//			   Veh_Info[11] =  Get_shiju(Veh_Info,g_ai32Pre_Veh_Info_4_Lane);	  //Pre_Veh_Info_4_Lane
//               Veh_Info[11] =  (pVehicle->Vdata.tdata[0] - g_ai32Pre_Veh_Info_4_Lane[25])/1000;
//			   if(Veh_Info[11] < 0)
//			   {
//			    Veh_Info[11] = 20; 	 //单位：秒；
//			   }
//			   if(0==Veh_Info[11])
//			   {
//			   	Veh_Info[11] = 1;
//			   }
//			   if( Veh_Info[11] < DisTime )
//			   {
//			   	   Veh_Info[12] = 1;//1 跟车
//			   }
//			   else
//			   {
//			   	   Veh_Info[12] = 0;//0 不跟车
//			   } // Veh_Info[12]	//判断是否跟车；		  					   	   
//			   Veh_Info[13] =	Veh_Info[11] * VehSpeed/3.6;//机动车车头间距
//			   if((pVehicle->Vdata.tdata[pVehicle->Vdata.u16FrameCnt-1] - pVehicle->Vdata.tdata[0]) < 1000)
//			   {
//			   	   Veh_Info[14] = 1;//定为1S；
//			   }
//			   else
//			   {
//			   		Veh_Info[14] = pVehicle->Vdata.tdata[pVehicle->Vdata.u16FrameCnt-1] - pVehicle->Vdata.tdata[0];//通过垂直断面时间 ms
//			   }
//			   memcpy(g_ai32Pre_Veh_Info_4_Lane,Veh_Info,sizeof(Veh_Info));
//			   g_ai32Pre_Veh_Info_4_Lane[25] = pVehicle->Vdata.tdata[0];
//			   break;
//		case 5:
//			   Veh_Info[11] =  Get_shiju(Veh_Info,g_ai32Pre_Veh_Info_5_Lane);	  //Pre_Veh_Info_4_Lane
//               Veh_Info[11] =  (pVehicle->Vdata.tdata[0] - g_ai32Pre_Veh_Info_5_Lane[25])/1000;
//			   if(Veh_Info[11] < 0)
//			   {
//			    Veh_Info[11] = 20; 	 //单位：秒；
//			   }
//			   if(0==Veh_Info[11])
//			   {
//			   	Veh_Info[11] = 1;
//			   }
//			   if( Veh_Info[11] < DisTime )
//			   {
//			   	   Veh_Info[12] = 1;//1 跟车
//			   }
//			   else
//			   {
//			   	   Veh_Info[12] = 0;//0 不跟车
//			   } // Veh_Info[12]	//判断是否跟车；		  					   	   
//			   Veh_Info[13] =	Veh_Info[11] * VehSpeed/3.6;//机动车车头间距
//			   if((pVehicle->Vdata.tdata[pVehicle->Vdata.u16FrameCnt-1] - pVehicle->Vdata.tdata[0]) < 1000)
//			   {
//			   	   Veh_Info[14] = 1;//定为1S；
//			   }
//			   else
//			   {
//			   		Veh_Info[14] = pVehicle->Vdata.tdata[pVehicle->Vdata.u16FrameCnt-1] - pVehicle->Vdata.tdata[0];//通过垂直断面时间 ms
//			   }
//			   memcpy(g_ai32Pre_Veh_Info_5_Lane,Veh_Info,sizeof(Veh_Info));
//			   g_ai32Pre_Veh_Info_5_Lane[25] = pVehicle->Vdata.tdata[0];
//			   break;
//	    case 6:
//			   Veh_Info[11] =  Get_shiju(Veh_Info,g_ai32Pre_Veh_Info_4_Lane);	  //Pre_Veh_Info_4_Lane
//               Veh_Info[11] =  (pVehicle->Vdata.tdata[0] - g_ai32Pre_Veh_Info_6_Lane[25])/1000;
//			   if(Veh_Info[11] < 0)
//			   {
//			    Veh_Info[11] = 20; 	 //单位：秒；
//			   }
//			   if(0==Veh_Info[11])
//			   {
//			   	Veh_Info[11] = 1;
//			   }
//			   if( Veh_Info[11] < DisTime )
//			   {
//			   	   Veh_Info[12] = 1;//1 跟车
//			   }
//			   else
//			   {
//			   	   Veh_Info[12] = 0;//0 不跟车
//			   } // Veh_Info[12]	//判断是否跟车；		  					   	   
//			   Veh_Info[13] =	Veh_Info[11] * VehSpeed/3.6;//机动车车头间距
//			   if((pVehicle->Vdata.tdata[pVehicle->Vdata.u16FrameCnt-1] - pVehicle->Vdata.tdata[0]) < 1000)
//			   {
//			   	   Veh_Info[14] = 1;//定为1S；
//			   }
//			   else
//			   {
//			   		Veh_Info[14] = pVehicle->Vdata.tdata[pVehicle->Vdata.u16FrameCnt-1] - pVehicle->Vdata.tdata[0];//通过垂直断面时间 ms
//			   }
//			   memcpy(g_ai32Pre_Veh_Info_6_Lane,Veh_Info,sizeof(Veh_Info));
//			   g_ai32Pre_Veh_Info_6_Lane[25] = pVehicle->Vdata.tdata[0];
//			   break;
//		default:
//		       break;
//	}
////uint32 g_total_veh[9]; //小客车	小货车	大客车	中型货车	大型货车	特大型货车	集装箱车	拖拉机	摩托车
////uint32 g_speed_veh_sum[9];
//	if(VehPattern>=1 && VehPattern<=9)
//	{	  //g_total_veh[4][9]:四车道，九车型
//		g_total_veh[u8Lane-1][VehPattern-1] =   g_total_veh[pVehicle->u8Lane-1][VehPattern-1] +  1;
//		g_speed_veh_sum[u8Lane-1][VehPattern-1] =   g_speed_veh_sum[u8Lane-1][VehPattern-1] + VehSpeed;	
//	}
//	/******有效性检验*******/
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
//	/******有效性检验over*******/
//	if(u8Lane>=1 && u8Lane<=g_sspSetup.u8LaneNum)
//	{
//		 g_total_Lane[u8Lane-1] = g_total_Lane[u8Lane-1] +1;//四车道总车数
//		 g_sum_shiju[u8Lane-1]  = g_sum_shiju[u8Lane-1]  +  Veh_Info[11];
//		 g_sum_shijian[u8Lane-1]= g_sum_shijian[u8Lane-1] + Veh_Info[14];
//		 g_sum_shijian_share[u8Lane-1] = 100*g_sum_shijian[u8Lane-1] / (ProCycle*60);	  //时间占有率//当前车道时间占有率；
//		 g_average_shiju[u8Lane-1] =    g_sum_shiju[u8Lane-1] / g_total_Lane[u8Lane-1];
//		 g_average_jianju[u8Lane-1] =  (g_average_jianju[u8Lane-1]*(g_total_Lane[u8Lane-1]-1) + Veh_Info[13])/g_total_Lane[u8Lane-1];
//	}
//	if(Veh_Info[12] == 1)
//	{
//		g_total_genche[u8Lane-1] = g_total_genche[u8Lane-1] + 1;	   //四车道跟车总车数
//	}
//	 Veh_Info[15] = g_total_veh[u8Lane-1][VehPattern-1];//当前车型总数；
//	 Veh_Info[16] = g_speed_veh_sum[u8Lane-1][VehPattern-1];//当前车型速度；
//	 Veh_Info[17] = g_average_shiju[u8Lane-1];//当前车道	机动车车头时距
//	 Veh_Info[18] = 100 * g_total_genche[u8Lane-1] / g_total_Lane[u8Lane-1];//跟车百分比；
//	 g_percent_genche[u8Lane-1] = 100 * g_total_genche[u8Lane-1] / g_total_Lane[u8Lane-1];//跟车百分比；
//	 Veh_Info[19] = g_average_jianju[u8Lane-1]; //当前车道	机动车车头间距
//	 Veh_Info[20] = 0;//时间占有率//当前车道时间占有率；
//	 Veh_Info[20] = g_sum_shijian_share[u8Lane-1];	  //时间占有率//当前车道时间占有率；
//	 Veh_Info[21] = u8Lane;//车道
//	 Veh_Info[22] =	g_total_count_Veh;//总车数
//
////上位机软件显示：
//	Send_VehInfo_Uart1[9]= VehSpeed;  //速度	  	
//	Send_VehInfo_Uart1[15]= b2bcd(YEAR_uint8);
//	Send_VehInfo_Uart1[16]= b2bcd(MONTH); //转化为BCD码
//	Send_VehInfo_Uart1[17]= b2bcd(DAY);
//	Send_VehInfo_Uart1[18]= b2bcd(HOUR);
//	Send_VehInfo_Uart1[19]= b2bcd(MIN);
//	Send_VehInfo_Uart1[20]= b2bcd(SEC);
//	Send_VehInfo_Uart1[18]= g_u8TimeHour;     //20130416 likang 发送波形时间
//	Send_VehInfo_Uart1[19]= g_u8TimeMin;
//	Send_VehInfo_Uart1[20]= g_u8TimeSec;
//	Send_VehInfo_Uart1[21]= VehPattern;// 车型
//	Send_VehInfo_Uart1[5]= VehLength>>24;
//	Send_VehInfo_Uart1[6]= VehLength>>16;
//	Send_VehInfo_Uart1[7]= VehLength>>8;
//	Send_VehInfo_Uart1[8]= VehLength;
//	Send_VehInfo_Uart1[41]=VehHeight>>8; //轴距
//	Send_VehInfo_Uart1[42]=VehHeight;
//	Send_VehInfo_Uart1[43]=VehWide>>8;//fuzhi
//	Send_VehInfo_Uart1[44]=VehWide;
//	Send_VehInfo_Uart1[37]=0x01;
//	Send_VehInfo_Uart1[38]=u8Lane;//车道
//
//	crc_create(Send_VehInfo_Uart1,52);
//	OSSemPost(g_Uart1_send);
//	UART1_SendBuf(Send_VehInfo_Uart1,55);	   //TOTC  = 32
//	if((VehLength > limit_length) && (VehHeight > limit_height) && (VehWide > limit_width))
//	{
//		U5SendBytes(Send_VehInfo_Uart1,55);
//		sprintf(strsend,"发送%d,%d\n",gv_index,send_count++);
//		U5SendBytes(strsend,strlen(strsend)+1);
//	}
//	 save_add = sv_write_sd_add;
//	 IF_SAME_DAY = (YEAR - 2013) * 372 + (MONTH - 1) * 31 + DAY;
//	if(gv_index == 1)
//	{
//		if(sv_count!=0)	   //跨天时候的处理方式 将一天中剩余的不满24辆车的数据存SD卡中
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
//		full_write_sd(day_temp+SD_STAT1_ADD_START, WriteSDtemp);   //一天的头一辆车的SD卡起始地址
//	}
//	gen_sv_fram(Send_VehInfo_Uart1);   //通过单车信息组成11帧
//	memcpy(&sv_frame_data[sv_count*21],sv_sd_frame,21);
//	sv_count = gv_index%24; 
//	if(sv_count==0)		//24辆车的信息满了，统一存进SD卡的一个扇区
//	{
//		save_add = save11_to_sd(sv_frame_data,504);		
//	}
//	if_send_flag = 0;
//	if((VehLength > limit_length) && (VehHeight > limit_height) && (VehWide > limit_width))	  	//长宽高超限，将标志位置1
//		if_send_flag = 1;
//	if(if_send_flag == 1)
//	{
//		if(g_u8Flag_wireless == 0)	//无线网络连接状态正常
//		{
//			UART1_SendBuf_full(frame11_buf,57);	//通过串口1发送单车数据
//			 VehSendInfo_tmp.u16Year = frame11_buf[22] + ((frame11_buf[23] & 0xFF) << 8);
//			 VehSendInfo_tmp.u8Month = frame11_buf[24];
//			 VehSendInfo_tmp.u8Day = frame11_buf[25];
//			 VehSendInfo_tmp.u8Hour = frame11_buf[26];
//			 VehSendInfo_tmp.u8Minute = frame11_buf[27];
//			 VehSendInfo_tmp.u8Second = frame11_buf[28];
//			 VehSendInfo_tmp.u32Veh_Index = gv_index;
//			if(CycleQue_Cnt_VehSendInfo == 0)	//队列空
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
//			//将单车的SD卡地址存续传队列
//			if(save_add != 0)
//			{
//				//如果24辆车的数据还没有存满，就没有写入SD卡，那么save_add的值？？？
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
//	int Distance=800;   //测量80cm距离内的时间差，以求出速度
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
//		for(i=index-1;i>1;i--)	   //找距离（EnterX2-Distance）最近的点
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
//    VehLength=(Lane_Vertical[m][index-1][4]-Lane_Vertical[m][index-1][3]);  //车辆长度
//	return VehLength;
    return 0;
}
