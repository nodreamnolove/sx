/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			Task_Data_JG.C
** Last modified Date:  20120718
** Last Version:		1.0
** Descriptions:		激光数据处理任务
**
**--------------------------------------------------------------------------------------------------------
** Created by:			Hong XiangYuan
** Created date:		20120718
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
#include "Task_Data_JG.h"
#include "Task_JG2.h"
#include "Task_JG1.h"
#include "Task_JG0.h"
#include "W5100.h"
#include "WT_Task.h"
#include "common.h"
#include "CMD.h"
#include "Timer0.h"
#include "TDC256.h"
#include "JG.h"
#include "UART5.h"
#include "crc.h"
#include "sort.h"
#define		SETUPALIAS				g_sspSetup

unsigned short Tabsin[181] = 
{			  //0:0.5:90 度； 乘以32768；2的15次方，运算时左移15位； Tabsin[ ]>>15;
        3,           286,         572,         858,         1144,        1429,        1715,        2000,        2286, 
        2571,        2856,        3141,        3425,        3709,        3993,        4277,        4560,        4843,
        5126,        5408,        5690,        5971,        6252,        6533,        6813,        7092,        7371,
        7650,        7927,        8204,        8481,        8757,        9032,        9307,        9580,        9854,
       10126,       10397,       10668,       10938,       11207,       11476,       11743,       12010,       12275,
       12540,       12803,       13066,       13328,       13589,       13848,       14107,       14365,       14621,
       14876,       15131,       15384,       15636,       15886,       16136,       16384,       16631,       16877,
       17121,       17364,       17606,       17847,       18086,       18324,       18560,       18795,       19028,
       19261,       19491,       19720,       19948,       20174,       20399,       20622,       20843,       21063,
       21281,       21498,       21713,       21926,       22138,       22348,       22556,       22763,       22967,
       23170,       23372,       23571,       23769,       23965,       24159,       24351,       24542,       24730,
       24917,       25102,       25285,       25466,       25645,       25822,       25997,       26170,       26341,
       26510,       26677,       26842,       27005,       27166,       27325,       27482,       27636,       27789,
       27939,       28088,       28234,       28378,       28520,       28660,       28797,       28932,       29066,
       29197,       29325,       29452,       29576,       29698,       29818,       29935,       30050,       30163,
       30274,       30382,       30488,       30592,       30693,       30792,       30888,       30983,       31075,
       31164,       31251,       31336,       31419,       31499,       31576,       31651,       31724,       31795,
       31863,       31928,       31991,       32052,       32110,       32166,       32219,       32270,       32319,
       32365,       32408,       32449,       32488,       32524,       32557,       32588,       32617,       32643,
       32667,       32688,       32707,       32723,       32737,       32748,       32757,       32763,       32767,
       32768
};

unsigned short Tabcos[181] =
{
       32768,       32767,       32763,       32757,       32748,       32737,       32723,       32707,       32688,
       32667,       32643,       32617,       32588,       32557,       32524,       32488,       32449,       32408,
       32365,       32319,       32270,       32219,       32166,       32110,       32052,       31991,       31928,
       31863,       31795,       31724,       31651,       31576,       31499,       31419,       31336,       31251,
       31164,       31075,       30983,       30888,       30792,       30693,       30592,       30488,       30382,
       30274,       30163,       30050,       29935,       29818,       29698,       29576,       29452,       29325,
       29197,       29066,       28932,       28797,       28660,       28520,       28378,       28234,       28088,
       27939,       27789,       27636,       27482,       27325,       27166,       27005,       26842,       26677,
       26510,       26341,       26170,       25997,       25822,       25645,       25466,       25285,       25102,
       24917,       24730,       24542,       24351,       24159,       23965,       23769,       23571,       23372,
       23170,       22967,       22763,       22556,       22348,       22138,       21926,       21713,       21498,
       21281,       21063,       20843,       20622,       20399,       20174,       19948,       19720,       19491,
       19261,       19028,       18795,       18560,       18324,       18086,       17847,       17606,       17364,
       17121,       16877,       16631,       16384,       16136,       15886,       15636,       15384,       15131,
       14876,       14621,       14365,       14107,       13848,       13589,       13328,       13066,       12803,
       12540,       12275,       12010,       11743,       11476,       11207,       10938,       10668,       10397,
       10126,        9854,        9580,        9307,        9032,        8757,        8481,        8204,        7927,
        7650,        7371,        7092,        6813,        6533,        6252,        5971,        5690,        5408,
        5126,        4843,        4560,        4277,        3993,        3709,        3425,        3141,        2856,
        2571,        2286,        2000,        1715,        1429,        1144,         858,         572,         286,
           3
};
int32 Zdata2_Base[361]={0};		  //倾斜激光，Z值基准，	2 * StartAngle:2 * EndAngle		 共285个
//{
//	6690,	6690,	6690,	6690,	6690,	6690,	6690,	6690,	6690,	6690,	6690,	1431,	1325,	1196,	6690,	6690,	6690,	6690,
//	6690,	6690,	6690,	234,	217,	213,	186,	119,	22,		10,		9,		3,		-3,		-15,	6690,	-7,		-13,	-7,
//	-20,	-27,	-27,	-12,	-23,	-21,	-33,	-37,	-18,	-44,	-50,	-57,	-58,	-37,	-60,	-74,	-56,	-71,	
//	-76,	268,	364,	331,	278,	264,	349,	283,	299,	311,	334,	249,	129,	46,		-6,		-7,		19,		18,	
//	-79,	-121,	-115,	-130,	-132,	-132,	-134,	-131,	-146,	-131,	-135,	-143,	-163,	-147,	-151,	-139,	-142,	-142,
//	-143,	-141,	-150,	-142,	-153,	-138,	-144,	-152,	-156,	-144,	-156,	-162,	-147,	-160,	-169,	-150,	-159,	-174,	
//	-176,	-172,	-174,	-171,	-162,	-164,	-175,	-177,	-178,	-173,	-179,	-185,	-195,	-193,	-179,	-182,	-192,	-205,	
//	-206,	-193,	-188,	-195,	-195,	-194,	-192,	-212,	-200,	-206,	-206,	-210,	-213,	-219,	-211,	-216,	-224,	-212,	
//	-217,	-217,	-228,	-235,	-220,	-226,	-225,	-235,	-239,	-250,	-242,	-252,	-233,	-245,	-249,	-253,	-262,	-273,	
//	-276,	-271,	-284,	-283,	-293,	-281,	-292,	-286,	-293,	-296,	-310,	-313,	-309,	-300,	-323,	-325,	-312,	-313,	
//	-333,	-339,	-329,	-356,	-347,	-331,	-348,	-349,	-354,	-374,	-360,	-374,	-375,	-380,	-395,	-393,	-385,	-402,	
//	-393,	-411,	-413,	-396,	-412,	-421,	-423,	-435,	-435,	-443,	-434,	-446,	-451,	-454,	-467,	-442,	-445,	-395,	
//	-340,	-367,	-337,	-209,	-107,	-70,	4,		21,		30,		75,		128,	112,	145,	158,	159,	165,	207,	6690,
//	-569,	-563,	-595,	-578,	-560,	-599,	-584,	-612,	-626,	-624,	-631,	-638,	-662,	-679,	-698,	-708,	-722,	-713,	
//	-741,	-746,	6690,	6690,	6690,	6690,	6690,	6690,	6690,	6690,	6690,	6690,	6690,	6690,	6690,	6690,	6690,	6690,	
//	6690,	6690,	6690,	6690,	6690,	6690,	1513,	1629,	4361,	4411,	4459,	6690,	6690,	6690,	6690,	0
//		
//};
extern int32   LMS_data_0[10][365];
extern int32   LMS_data_1[10][365];
extern uint8 	g_u8JG1_RBuff_Count; //两激光缓存计数；
extern uint8 	g_u8JG1_PBuff_Count; //两激光缓存计数；
extern uint8 	g_u8JG0_RBuff_Count; //两激光缓存计数；
extern uint8 	g_u8JG0_PBuff_Count; //两激光缓存计数；
PointSet l_FrameInfo;
/***************JG0入口系统参数************************/
/* JG0起点        g_sspSetup.u16J0StartPos; 
/* JG0止点	      g_sspSetup.u16EndPtNum0
/* JG0零点	      g_sspSetup.u16J0ZeroPos
/* JG0高度		  g_sspSetup.J0_Height
/* JG0/1距离	  g_sspSetup.LaserDistance
/**************************************************/
/********定义JG0数据读和处理记录变量***********/
uint16  g_u16VerticalStartAnglePt0;  
uint16  g_u16VerticalEndAnglePt0;   
/************************************************************/
/***************JG1入口系统参数************************/
/* JG1起点        g_sspSetup.u16J1StartPos; 
/* JG1止点	      g_sspSetup.u16J1EndPos
/* JG1零点	      g_sspSetup.u16J1ZeroPos
/* JG1高度		  g_sspSetup.HeightLaser1
/* JG0/1距离	  g_sspSetup.LaserDistance
/**************************************************/
/********定义JG1数据读和处理记录变量***********/
uint16  g_u16VerticalStartAnglePt1;  
uint16  g_u16VerticalEndAnglePt1;   
/************************************************************/
uint16  g_u16VerticalStartAnglePt=0;  
uint16  g_u16VerticalEndAnglePt=0; 
#define IS_INSIDE(x,y,dx,dy)	( (min(dx,dy) >= max(x,y) || max(dx,dy) <= min(x,y)) ? 0:1 )
#define min(x,y)	(x > y ? y:x) 
#define MAXDAFEIPTNUM    4
#define MINWIDE_THRESHOLD     100 
#define    ERRORVALUE       0xFFFF 
int GetFramAreaHeight(int *pg_ZV, uint16 u16StartPt, uint16 u16EndPt);
int SeachAreaMatchIndex(PointStruct FramInfo);
void OutPutVeh(VehicleStruct *pVehicle);
uint16 GetVehicleHeight(uint16 *PxMaxHt,uint16 u16FrameCnt);
uint16 GetVehWidth(uint16 *PxDis,uint16 u16FrameCnt,uint16 u16heigh);

uint32 g_u32VehWideHeigh[10][10];
uint16 g_u16Recvcount;
uint16 g_u16DealCount;
uint32 g_count;
uint32 g_Z[50][100];
uint32 g_X[50][100];
int32 g_ZV[500]={0};
int32 g_XV[500]={0};
#define ISLEGAL0(x) ((x>ThresOrigineDataLow0) && (x<ThresOrigineDataHigh0))?1:0
#define ISLEGAL1(x) ((x>ThresOrigineDataLow0) && (x<ThresOrigineDataHigh0))?1:0
void Task_Data_JG(void *tdata)
{
   	uint8 err;
	PointStruct l_u16PosVect[POINTSET_CNT] = {0};
	int i=0;			   
	int j=0;

	int32 l_tmp1=0;
	int32 l_tmp2=0;
	uint8 Mathfalg=0;
	uint32 CurTime=0;
	uint8   MatchIdx=0;
    uint8   MatchedIdx=0;
	uint8 l_DafeiPtNum;
	int32 l_32tmpValue=0;
	int32 l_32tmp2,TempVaule1,l_32tmp;
	uint16 l_u16tmp,l_u16index; 
	uint32 l_leftPt=0;
	uint32 l_rightPt=0;
	int32 l_leftX,l_rightX;    //左，右x距离
	uint16 l_leftXpt, l_rightXpt;   //左右X距离对应的点数
	uint32 Time_Vertical;
	uint8 l_u8Count;
	int Tmp_Z = 0;                 //车辆高度
	int	Tmp_Y = 0;
	int m=0;
	int k=0;
	int AreaMatchIndex=0;		            //分车用
	/**分区用**/
	uint8 state=0;
	int16 Curindx=0;
	int16 Firstindx=-1;
	int16 Secondindx=0;
	int16 Previndx=0;
	int32 l_curpos=0;
	int32 l_lastpos=0;
	uint16 l_JiaJiao=0;
	int32  tempmid = 0;
	tdata=tdata;
	while(1)
	{
		 P3_OUTP_CLR = (1 << 9);	 //亮
		 P3_OUTP_SET = (1 << 9);	 //灭 
	 	 CurTime=t0_count2;
		 /*********初始化参数*****************************/
		 Mathfalg=0;
		 MatchIdx=0;
		 MatchedIdx=0;
		 state=0;
		 Curindx=0;
		 Firstindx=-1;
		 Secondindx=0;
		 Previndx=0;
		 memset(&l_FrameInfo,0,sizeof(l_FrameInfo));
		 /*********初始化参数*****************************/
		 OSSemPend(g_JG_Pro,0,&err);
		 memset(g_ZV,0,sizeof(g_ZV));
		 memset(g_XV,0,sizeof(g_XV));
//#ifndef SIM_SOFTWARE
		 /*****************开始对扫激光器帧匹配**********************/
		 //1 2有数据
#ifdef SIM_SOFTWARE
		 g_u8JG0_PBuff_Count = 0;
		 g_u8JG1_PBuff_Count = 0;
#endif
		 if(g_u8JG1_RBuff_Count!=g_u8JG1_PBuff_Count&&g_u8JG0_RBuff_Count!=g_u8JG0_PBuff_Count)
		 {
				if(LMS_data_0[g_u8JG0_PBuff_Count][362]-LMS_data_1[g_u8JG1_PBuff_Count][362]<-11) //-20ms ~ 20ms
				{		 //0数据有效
				  if(abs(LMS_data_0[g_u8JG0_PBuff_Count][362]-CurTime)<80)
				  {
					  Mathfalg=2;
					  MatchIdx=g_u8JG0_PBuff_Count;
				  }
//				  g_u8JG0_PBuff_Count=g_u8JG0_PBuff_Count+1;
//				  g_u8JG0_PBuff_Count=g_u8JG0_PBuff_Count%10; 
				
				}
				else if(abs(LMS_data_0[g_u8JG0_PBuff_Count][362]-LMS_data_1[g_u8JG1_PBuff_Count][362])<=11)
				{	//有效匹配
					Mathfalg=1;
					MatchIdx=g_u8JG0_PBuff_Count;
					MatchedIdx=g_u8JG1_PBuff_Count;
#ifndef SIM_SOFTWARE
					g_u8JG0_PBuff_Count=g_u8JG0_PBuff_Count+1;
					g_u8JG0_PBuff_Count=g_u8JG0_PBuff_Count%10;
					g_u8JG1_PBuff_Count=g_u8JG1_PBuff_Count+1;
					g_u8JG1_PBuff_Count=g_u8JG1_PBuff_Count%10;
#endif
				}	    
				else if((LMS_data_0[g_u8JG0_PBuff_Count][362]-LMS_data_1[g_u8JG1_PBuff_Count][362])>11)
				{	//1数据有效
					if(abs(LMS_data_1[g_u8JG1_PBuff_Count][362]-CurTime)<80)
					{
					 	Mathfalg=3;
					 	MatchedIdx=g_u8JG1_PBuff_Count;
					}
//					g_u8JG1_PBuff_Count=g_u8JG1_PBuff_Count+1;
//					g_u8JG1_PBuff_Count=g_u8JG1_PBuff_Count%10;
				}
			} // 0 无数据 1有数据

		   if(Mathfalg==1)//||Mathfalg==2)	//0、1都有效或者 0号数据有效
		   {
		  
		       /*****************侧装垂直激光0坐标转换开始************/	
				g_u16VerticalStartAnglePt0 = GetStartEndPt(LMS_data_0[MatchIdx],g_sspSetup.u16J0StartPos, 0, 0);	//垂直激光器坐标转换开始点	 参数中第1个0表示找开始点标志，第2个0表示垂直激光器
				g_u16VerticalEndAnglePt0   = GetStartEndPt(LMS_data_0[MatchIdx],g_sspSetup.u16J0EndPos, 1, 0);   //垂直激光器坐标转换结束点
				if (( g_u16VerticalStartAnglePt0 >= g_sspSetup.u16J0ZeroPos &&  g_u16VerticalEndAnglePt0 >= g_sspSetup.u16J0ZeroPos) ||
					( g_u16VerticalStartAnglePt0 <= g_sspSetup.u16J0ZeroPos &&  g_u16VerticalEndAnglePt0 <= g_sspSetup.u16J0ZeroPos)) //转换一边（起始点和结束点在零点的一侧，侧装X坐标都为正）
				{	//需要修改
					l_tmp1 = 0;
					for(i= g_u16VerticalEndAnglePt0 ;i > g_u16VerticalStartAnglePt0;i--)	
					{
						l_curpos = LMS_data_0[MatchIdx][i];
						l_JiaJiao = abs(g_sspSetup.u16J0ZeroPos-i);
						if(ISLEGAL0(l_curpos))
						{ 
							l_DafeiPtNum = 0;
							g_ZdistanceV[l_tmp1]= g_sspSetup.J0_Height - ((l_curpos*Tabcos[l_JiaJiao])>>15); // HeightLaser +
							if(g_ZdistanceV[l_tmp1] < 500)
								g_ZdistanceV[l_tmp1] = 0;

//warning 可能会有 x如何处理
							g_XdistanceV[l_tmp1]= ((l_curpos*Tabsin[l_JiaJiao])>>15);
							if(g_XdistanceV[l_tmp1]<g_sspSetup.MedianWide)//0激光器边界
							{
							  g_XdistanceV[l_tmp1]=0;
							  g_ZdistanceV[l_tmp1]=0;
							  l_tmp1=l_tmp1-1;
							  l_curpos = 0;	 
							}
							else if(g_XdistanceV[l_tmp1]>g_sspSetup.LaserDistance-g_sspSetup.MedianLeftWide) //1激光器边界
							{
							  g_XdistanceV[l_tmp1]=0;
							  g_ZdistanceV[l_tmp1]=0;
							  l_tmp1=l_tmp1-1;
							  l_curpos = 0;
							}
						
											 
						}
						else							
						{
							if (l_tmp1 == 0)	 //第1个点
							{
								g_ZdistanceV[l_tmp1] = 2;
								g_XdistanceV[l_tmp1]=Tabsin[l_JiaJiao]*g_sspSetup.J0_Height/Tabcos[l_JiaJiao];
								if(g_XdistanceV[l_tmp1]>g_sspSetup.LaserDistance)
								{
								  g_XdistanceV[l_tmp1]=0;
								  g_ZdistanceV[l_tmp1]=0;
								  l_tmp1=l_tmp1-1;	
								}
							}
							else 
							{
							    if(ISLEGAL0(l_lastpos))	   //上一点有效
								{
								    
									g_XdistanceV[l_tmp1] = ((Tabsin[l_JiaJiao]*l_lastpos)>>15);
								  
									g_ZdistanceV[l_tmp1]=2;
								}
								else
								{	
							     
													  //上一点无效
									g_XdistanceV[l_tmp1] = g_XdistanceV[l_tmp1-1]+ abs(g_sspSetup.J0_Height*Tabsin[l_JiaJiao]/Tabcos[l_JiaJiao]-g_sspSetup.J0_Height*Tabsin[l_JiaJiao-1]/Tabcos[l_JiaJiao-1]);
								
									g_ZdistanceV[l_tmp1]=2;
								} 
								if(g_XdistanceV[l_tmp1]>g_sspSetup.LaserDistance)
								{
								  g_XdistanceV[l_tmp1]=0;
								  g_ZdistanceV[l_tmp1]=0;
								  l_tmp1=l_tmp1-1;	
								}
							}	
						}
					l_tmp1++;
					l_lastpos = l_curpos;				
				  }	//for	
			   }
	          /*****************侧装垂直激光0坐标转换完毕************/
			 /*******************激光0寻找有车区域 ***********************/
	//		 Delege0SanDian(g_XdistanceV, g_ZdistanceV,l_tmp1);
			 InsertSort(g_XdistanceV, g_ZdistanceV,l_tmp1); 
			   
			 }	


			/*******************激光0寻找有车区域结束 ***********************/

		  if(Mathfalg==1)	 //0、1都有效或者  1号数据有效
		  {
			  /*****************侧装垂直激光1坐标转换开始************/
			  	g_u16VerticalStartAnglePt1 = GetStartEndPt(LMS_data_1[MatchedIdx],g_sspSetup.u16J1StartPos, 0, 1);	//垂直激光器坐标转换开始点	 参数中第1个0表示找开始点标志，第2个0表示垂直激光器
				g_u16VerticalEndAnglePt1   = GetStartEndPt(LMS_data_1[MatchedIdx],g_sspSetup.u16J1EndPos, 1, 1);   //垂直激光器坐标转换结束点
				if (( g_u16VerticalStartAnglePt1 >= g_sspSetup.u16J1ZeroPos &&  g_u16VerticalEndAnglePt1 >= g_sspSetup.u16J1ZeroPos) ||
					( g_u16VerticalStartAnglePt1 <= g_sspSetup.u16J1ZeroPos &&  g_u16VerticalEndAnglePt1 <= g_sspSetup.u16J1ZeroPos)) //转换一边（起始点和结束点在零点的一侧，侧装X坐标都为正）
				{	//需要修改
					l_tmp2 = 0;								
					for(i=g_u16VerticalStartAnglePt1;i <= g_u16VerticalEndAnglePt1;i++)	
					{
					    l_curpos = LMS_data_1[MatchedIdx][i];					 
						l_JiaJiao = abs(g_sspSetup.u16J1ZeroPos-i);

						if(ISLEGAL1(l_curpos))
						{ 
							l_DafeiPtNum = 0;
							g_ZdistanceV1[l_tmp2]= g_sspSetup.J1_Height - ((l_curpos*Tabcos[l_JiaJiao])>>15); // HeightLaser +
							g_XdistanceV1[l_tmp2]= g_sspSetup.LaserDistance - ((l_curpos*Tabsin[l_JiaJiao])>>15);
							if(g_ZdistanceV1[l_tmp2] < 500)
								g_ZdistanceV1[l_tmp2] = 0;
						    if(g_XdistanceV1[l_tmp2]<g_sspSetup.MedianWide)	//激光0边界
							{
								 g_XdistanceV1[l_tmp2]=0;		  
							  	 g_ZdistanceV1[l_tmp2]=0;
							  	 l_tmp2 = l_tmp2-1;
								 l_curpos =0;
							}
							else if(g_XdistanceV1[l_tmp2]>g_sspSetup.LaserDistance -g_sspSetup.MedianLeftWide)//激光1边界
							{
							  g_XdistanceV1[l_tmp2]=0;		  
							  g_ZdistanceV1[l_tmp2]=0;
							  l_tmp2 = l_tmp2-1;
							  l_curpos = 0;																
							}					 
						}
						else //打飞处理
						{
							if (l_tmp2 == 0)	 //第1个点
							{
								g_ZdistanceV1[l_tmp2] = 2;
								g_XdistanceV1[l_tmp2]=g_sspSetup.LaserDistance-Tabsin[l_JiaJiao]*g_sspSetup.J1_Height/Tabcos[l_JiaJiao];
								if(g_XdistanceV1[l_tmp2]<0)
								{
								  g_XdistanceV1[l_tmp2]=0;
								  g_ZdistanceV1[l_tmp2]=0;
								  l_tmp2=l_tmp2-1;	
								}
							}
							else 
							{
							    if(ISLEGAL1(l_lastpos))	   //上一点有效
								{
									tempmid = (Tabsin[l_JiaJiao]*l_lastpos)>>15	;
									g_XdistanceV1[l_tmp2] = g_sspSetup.LaserDistance -tempmid;
									g_ZdistanceV1[l_tmp2]=2;
								}
								else
								{						  //上一点无效
									g_XdistanceV1[l_tmp2] = g_XdistanceV1[l_tmp2-1]- abs(g_sspSetup.J1_Height*Tabsin[l_JiaJiao]/Tabcos[l_JiaJiao]-g_sspSetup.J1_Height*Tabsin[l_JiaJiao-1]/Tabcos[l_JiaJiao-1]);
									g_ZdistanceV1[l_tmp2]=2;
								} 
								if(g_XdistanceV1[l_tmp2]>g_sspSetup.LaserDistance)
								{
								  g_XdistanceV1[l_tmp2]=0;
								  g_ZdistanceV1[l_tmp2]=0;
								  l_tmp2=l_tmp2-1;	
								}
								
								if(g_XdistanceV1[l_tmp2]<0)
								{
								  g_XdistanceV1[l_tmp2]=0;
								  g_ZdistanceV1[l_tmp2]=0;
								  l_tmp2=l_tmp2-1; 	
								}
							}	
						}
					  l_tmp2++;
					  l_lastpos = l_curpos;
					}		
				}
			  /*****************侧装垂直激光1坐标转换完成************/

			  /*******************激光1区域穿插 ***********************/
		//	 	Delege1SanDian(g_XdistanceV1, g_ZdistanceV1,l_tmp2);
			    InsertSort(g_XdistanceV1, g_ZdistanceV1,l_tmp2); 
		  }
		  if(Mathfalg==1)  //配置成功
		  {	 
		  /************************双路穿插*************************/
//			 for(i=0,j=0;i<l_tmp1&&j<l_tmp2;)
//			 {
//				 if(g_XdistanceV[i]>g_XdistanceV1[j])
//				 {
//					  g_XV[i+j]=g_XdistanceV[i];
//					  g_ZV[i+j]=g_ZdistanceV[i];
//					  i++;
//				 }
//				 else if(g_XdistanceV[i]<g_XdistanceV1[j])
//				 {
//					  g_XV[i+j]=g_XdistanceV1[j];
//					  g_ZV[i+j]=g_ZdistanceV1[j];
//					  j++;
//				 }
//				 else
//				 {
//					  g_XV[i+j]=g_XdistanceV[i];
//					  g_ZV[i+j]=g_ZdistanceV[i];
//					  i++;
//					  g_XV[i+j]=g_XdistanceV1[j];
//					  g_ZV[i+j]=g_ZdistanceV1[j];
//					  j++;
//				 }
//			 }
//			 if(i==l_tmp1&&j<l_tmp2)       //剩下一路2
//			 {
//				  while(j<l_tmp2)
//				  {
//				   	  g_XV[i+j]=g_XdistanceV1[j];
//					  g_ZV[i+j]=g_ZdistanceV1[j];
//					  j++;
//				  }
//			 }
//			 else if(j==l_tmp2&&i<l_tmp1)  //剩下一路1
//			 {
//				  while(i<l_tmp1)
//				  {
//					  g_XV[i+j]=g_XdistanceV[i];
//					  g_ZV[i+j]=g_ZdistanceV[i];
//					  i++;
//				  }
//			 } 
		   /************************双路穿插*************************/
		 Interweave_Denoise(l_tmp1, l_tmp2);
		 /************************穿插完毕*************************/
		 l_32tmpValue=l_tmp1+l_tmp2;
		 Time_Vertical=LMS_data_0[MatchIdx][361];		 	 
		}
		l_leftPt=0;	   //0
		l_rightPt=l_32tmpValue;//合并后的数据长度 
		l_u16index = 0;   //有车区域的索引，局部变量为零
		//state    0 表示有车 1表示 2表示 3表示 
		//Firstindx 有车左边界 	 Secondindx 有车右边界
		for(i = l_leftPt;i<=l_rightPt;i++)
		{	  
			    Curindx=i;
				if(state==0)
				{
					  if((g_ZV[i] > 0))//ThresVehLow) && (g_ZV[i] < ThresVehHigh))
					  {
						 state=1;
						 Firstindx=Curindx;					 //记录第一个有效点 左边界
						 Previndx=Curindx;
					  }		 				
				}
				else if(state==1)
				{
					 if((g_ZV[i] > 0))//ThresVehLow) && (g_ZV[i] < ThresVehHigh))
					 {
						  if(abs(g_XV[Curindx]-g_XV[Previndx])<200)
						  {
						      Previndx=Curindx;
							  if(abs(g_XV[Curindx]-g_XV[Firstindx])>200)	 //有效左边界
							  {
							     state=2;	
							  }			 
						  }
						  else				  //无效左边界 边界点右移
						  {
						   	Firstindx=Curindx;
							Previndx=Curindx;
							state=1;
						  }
					  }
					  else//无效点
					  {	 //不处理
					  //	 if(abs(g_XV[Curindx]-g_XV[Previndx])>200)
				    	//	 	state = 0;
	
					  }			 
				}
				else if(state==2)				 //找到有效左边界
				{				
				      if((g_ZV[i] > 2))//ThresVehLow) && (g_ZV[i] < ThresVehHigh))
					  {
						if (abs(g_XV[Curindx]-g_XV[Firstindx])>1600)
                		{
                    		state=1;
                    		Firstindx=Curindx;
                    		Previndx=Curindx;
                		}
                		else if(abs(g_XV[Curindx]-g_XV[Firstindx])>800)
                		{
                    	    if(g_ZV[i]>0)
                    		{
                        		state=3;
                        		Previndx=Curindx;
                        		Secondindx=Previndx;
                    		}
//							else if(g_ZV[i]==2)//打飞的点
//							{
//							}
                		}
                		else													//有效点
                		{
                    		state=2;
                    		Previndx=Curindx;
                    		Secondindx=Curindx;
                		}	 
					  }				
				}
				else if(state==3)  //右边界
				{							
				  if((g_ZV[i] > 2))//ThresVehLow)&& (g_ZV[i] < ThresVehHigh))  //有效点
				  {
				  	  if(i==l_rightPt)
					  {
					  	   if((abs(g_XV[Secondindx]-g_XV[Firstindx])>800))
							  {
							    l_FrameInfo.u8Sum=(l_FrameInfo.u8Sum+1)&POINTSET_MASK;
							//	Firstindx = deleteRota(g_XV,Firstindx,0);
							//    Secondindx = deleteRota(g_XV,Secondindx,1);							
						    	l_FrameInfo.Ptdata[l_u16index].n32xLeft = g_XV[Firstindx];
								l_FrameInfo.Ptdata[l_u16index].n32xRight = g_XV[Secondindx];
								l_FrameInfo.Ptdata[l_u16index].u16Leftpt  =Firstindx;
								l_FrameInfo.Ptdata[l_u16index].u16Rightpt = Secondindx;
								l_FrameInfo.Ptdata[l_u16index].u16xDis = abs(g_XV[Secondindx]-g_XV[Firstindx]);
								l_FrameInfo.Ptdata[l_u16index].u16xMaxHt= GetFramAreaHeight(g_ZV,Firstindx,Secondindx);	//计算最大高度
							    l_u16index=l_u16index+1;
								state=1;
						        Firstindx=Curindx;
						        Previndx=Curindx;
							  }	 	
					  }
					  else if((abs(g_XV[Curindx]-g_XV[Secondindx]))>400)
					  {
							  if((abs(g_XV[Secondindx]-g_XV[Firstindx])>800))
							  {
							    l_FrameInfo.u8Sum=(l_FrameInfo.u8Sum+1)&POINTSET_MASK;
						//		Firstindx = deleteRota(g_XV,Firstindx,0);
						//	    Secondindx = deleteRota(g_XV,Secondindx,1);
						    	l_FrameInfo.Ptdata[l_u16index].n32xLeft = g_XV[Firstindx];
								l_FrameInfo.Ptdata[l_u16index].n32xRight = g_XV[Secondindx];
								l_FrameInfo.Ptdata[l_u16index].u16Leftpt  =Firstindx;
								l_FrameInfo.Ptdata[l_u16index].u16Rightpt = Secondindx;
								l_FrameInfo.Ptdata[l_u16index].u16xDis = abs(g_XV[Secondindx]-g_XV[Firstindx]);
								l_FrameInfo.Ptdata[l_u16index].u16xMaxHt= GetFramAreaHeight(g_ZV,Firstindx,Secondindx);	//计算最大高度
							    l_u16index=l_u16index+1;
								state=1;
						        Firstindx=Curindx;
						        Previndx=Curindx;
							  }							  
							  else
							  {
								state=1;
						        Firstindx=Curindx;
						        Previndx=Curindx;
							  }
					   }	
					   else
					   {
					   	if(g_ZV[i]>2) {
						     state=3;
							 Previndx=Curindx;
							 Secondindx=Curindx;
						} else{	  // ==2 
							 state=3;
							 Previndx=Curindx;
						}
					  
					   }
				   }
				   else	   //无效点
				   {
					 state=3;
					 Previndx=Curindx;
					 if((abs(g_XV[Previndx]-g_XV[Secondindx]))>800||abs(Previndx-Secondindx)>=6||i==l_rightPt)
					 {
						 if(abs(g_XV[Secondindx]-g_XV[Firstindx])>800)		 //第一个区域
						  {
						    l_FrameInfo.u8Sum=(l_FrameInfo.u8Sum+1)&POINTSET_MASK;
					//		Firstindx = deleteRota(g_XV,Firstindx,0);
					//		Secondindx = deleteRota(g_XV,Secondindx,1);
							l_FrameInfo.Ptdata[l_u16index].n32xLeft = g_XV[Firstindx];
							l_FrameInfo.Ptdata[l_u16index].n32xRight = g_XV[Secondindx];
							l_FrameInfo.Ptdata[l_u16index].u16Leftpt  =Firstindx;
							l_FrameInfo.Ptdata[l_u16index].u16Rightpt = Secondindx;
							l_FrameInfo.Ptdata[l_u16index].u16xDis = abs(g_XV[Secondindx]-g_XV[Firstindx]);
							//计算最大高度
							l_FrameInfo.Ptdata[l_u16index].u16xMaxHt= GetFramAreaHeight(g_ZV,Firstindx,Secondindx);
						    l_u16index=l_u16index+1;
							state=0;
						  }
						  else
						  {
                            state=0;
						  }
					 }	 
				   }
				}
		}	
						
/******************有车区域与垂直车辆记录集匹配**********************************************/
	   for(l_u16index = 0;l_u16index < l_FrameInfo.u8Sum;l_u16index++)
	   {
			 AreaMatchIndex=SeachAreaMatchIndex(l_FrameInfo.Ptdata[l_u16index]);  //返回与当前帧匹配的序列号
			 if(AreaMatchIndex==ERRORVALUE) 
			 {
                 //出错处理
                 continue;
			 }
             else if(AreaMatchIndex==-1)
			 {
	              //新目标进入
	             for(m = 0;m<VEHICLE_MAX;m++)	 //进车
				 {
	                 if(g_VehicleSet[m].u8Vstate == NO_USED ) //找到空闲的区域存车
					 {
	                     AreaMatchIndex=m;
	                     break;
				     }
	              }
				 g_VehicleSetIndex[g_totalVehicle] = AreaMatchIndex+1;
	             g_totalVehicle = g_totalVehicle + 1;

             }
			 /*********更新匹配上记录集信息******/
			 g_VehicleSet[AreaMatchIndex].u8Vstate = OCCURING_USED;
             l_leftPt =l_FrameInfo.Ptdata[l_u16index].u16Leftpt;	//左侧位置点
             l_rightPt=l_FrameInfo.Ptdata[l_u16index].u16Rightpt;
             g_VehicleSet[AreaMatchIndex].VLocateX.n32xLeft  = l_FrameInfo.Ptdata[l_u16index].n32xLeft;
             g_VehicleSet[AreaMatchIndex].VLocateX.n32xRight = l_FrameInfo.Ptdata[l_u16index].n32xRight;
            
			 g_VehicleSet[AreaMatchIndex].VLocateX.u16Leftpt = l_FrameInfo.Ptdata[l_u16index].u16Leftpt;;
             g_VehicleSet[AreaMatchIndex].VLocateX.u16Rightpt = l_FrameInfo.Ptdata[l_u16index].u16Rightpt;
			
             l_32tmp2 = g_VehicleSet[AreaMatchIndex].Vdata.u16FrameCnt&FRAME_MASK;	 //总帧数 
             l_32tmp2 = l_32tmp2 + 1;
             if((l_rightPt - l_leftPt+1)>=0 && (l_rightPt - l_leftPt+1)<=(FRAME_BUFLEN-1))	 //数据拷进记录集
			 {
				 memcpy(&g_VehicleSet[AreaMatchIndex].Vdata.zdata[l_32tmp2-1][1],g_ZV + l_leftPt,sizeof(int32)*(l_rightPt - l_leftPt+1));
				 memcpy(&g_VehicleSet[AreaMatchIndex].Vdata.xdata[l_32tmp2-1][1],g_XV + l_leftPt,sizeof(int32)*(l_rightPt - l_leftPt+1));
                 g_VehicleSet[AreaMatchIndex].Vdata.zdata[l_32tmp2-1][0]  = l_rightPt - l_leftPt+1;
                 g_VehicleSet[AreaMatchIndex].Vdata.xdata[l_32tmp2-1][0]  = l_rightPt - l_leftPt + 1;
			 }
             else
			 {
				memcpy(&g_VehicleSet[AreaMatchIndex].Vdata.zdata[l_32tmp2-1][1],g_ZV + l_leftPt,sizeof(int32)*(FRAME_BUFLEN-1));
				memcpy(&g_VehicleSet[AreaMatchIndex].Vdata.xdata[l_32tmp2-1][1],g_XV + l_leftPt,sizeof(int32)*(FRAME_BUFLEN-1));
				g_VehicleSet[AreaMatchIndex].Vdata.zdata[l_32tmp2-1][0]  = FRAME_BUFLEN-1;
				g_VehicleSet[AreaMatchIndex].Vdata.xdata[l_32tmp2-1][0]  = FRAME_BUFLEN-1;
             }
             g_VehicleSet[AreaMatchIndex].Vdata.u16xDis[l_32tmp2-1]=l_FrameInfo.Ptdata[l_u16index].u16xDis;
             g_VehicleSet[AreaMatchIndex].Vdata.u16xMaxHt[l_32tmp2-1] =l_FrameInfo.Ptdata[l_u16index].u16xMaxHt; //最大值Z
             g_VehicleSet[AreaMatchIndex].Vdata.tdata[l_32tmp2-1] = Time_Vertical;
             g_VehicleSet[AreaMatchIndex].VemptFrame = 0;
             if (g_VehicleSet[AreaMatchIndex].Vdata.u16FrameCnt < FRAME_MASK )
			 {
                 g_VehicleSet[AreaMatchIndex].Vdata.u16FrameCnt = g_VehicleSet[AreaMatchIndex].Vdata.u16FrameCnt + 1;
			 }
             else
			 {
                 g_VehicleSet[AreaMatchIndex].Vdata.u16FrameCnt = FRAME_MASK;
             }
			 g_VehicleSet[AreaMatchIndex].VemptFrame= 0;	 //垂直空白帧数
		}
/***********************宽高激光器车辆收尾判断**************************/
	 	if (g_totalVehicle > VEHICLE_MAX)
		{
			g_totalVehicle = 0;
			for (j = 0; j < VEHICLE_MAX; j++)
			{
				if (g_VehicleSet[j].u8Vstate != NO_USED)
					g_totalVehicle++;	//车辆总数加1
			}
		}

		for(j = 0;j < g_totalVehicle;j++)
		{
			i = (g_VehicleSetIndex[j] - 1) & VEHICLE_MASK;	
			if(g_VehicleSet[i].u8Vstate != NO_USED )
			{
				if(g_VehicleSet[i].VemptFrame > NORMAL_MAX_EMPTYFRAME && g_VehicleSet[i].Vdata.u16FrameCnt)
				{ 		
				   g_VehicleSet[i].u8Vstate = PASSED_USED;  //已结束，可收尾的车
				} 
				//正在使用 并且数据帧数大于100则出车
				if(g_VehicleSet[i].u8Vstate == OCCURING_USED && g_VehicleSet[i].Vdata.u16FrameCnt>255 )     
				{ 
					g_VehicleSet[i].Vdata.u16FrameCnt -=10;
					//	OutPutVeh(&g_VehicleSet[i]);
					//	memset(&g_VehicleSet[i],0,sizeof(VehicleStruct));
				//	g_VehicleSet[i].u8Vstate = PASSED_USED;  
//					for(l_u16tmp = j;l_u16tmp < g_totalVehicle - 1;l_u16tmp++)
//					   g_VehicleSetIndex[l_u16tmp] = g_VehicleSetIndex[l_u16tmp+1];
//					
//					if (g_totalVehicle <= VEHICLE_MAX)  
//					{
//						g_VehicleSetIndex[g_totalVehicle - 1] = 0;
//						g_totalVehicle--;
//					}
//					else
//					{
//						g_totalVehicle = 0;
//					}	
//					continue;			
				}
				if(g_VehicleSet[i].u8Vstate == PASSED_USED)
				{
				    OutPutVeh(&g_VehicleSet[i]);
				    CurTime = sizeof(VehicleStruct);
				   	memset(&g_VehicleSet[i],0,sizeof(VehicleStruct));
					g_VehicleSet[i].u8Vstate = NO_USED;  
					for(l_u16tmp = j;l_u16tmp < g_totalVehicle - 1;l_u16tmp++)
					   g_VehicleSetIndex[l_u16tmp] = g_VehicleSetIndex[l_u16tmp+1];
					
					if (g_totalVehicle <= VEHICLE_MAX)  
					{
						g_VehicleSetIndex[g_totalVehicle - 1] = 0;
						g_totalVehicle--;
					 }
					else
					{
						g_totalVehicle = 0;
					}

				} 
			} 
		g_VehicleSet[i].VemptFrame++;				
		}
//		 VehicleMatch(); //长 宽 高匹配 
		/******************初始化变量****************************/
		memset(g_ZdistanceV, 0, sizeof(int32)*POINT_SUM);
		memset(g_XdistanceV, 0, sizeof(int32)*POINT_SUM);
		memset(g_ZdistanceI, 0, sizeof(int32)*POINT_SUM);
		memset(g_YdistanceI, 0, sizeof(int32)*POINT_SUM);
		memset(g_ZdistanceV1, 0, sizeof(int32)*POINT_SUM);
		memset(g_XdistanceV1, 0, sizeof(int32)*POINT_SUM);
		memset(g_ZdistanceI3, 0, sizeof(int32)*POINT_SUM);
		memset(g_YdistanceI3, 0, sizeof(int32)*POINT_SUM);


        /******************初始化变量****************************/
	   OSTimeDly(3);
	}
}

/******************************************************
/*垂直车辆记录集与两个顺车道车辆记录集进行车辆匹配
/*
******************************************************/
//void VehicleMatch(void)
//{
//	uint8	i=0;
//	uint8	j=0;

//	uint8	m=0;
//	uint8	k=0;
//	uint8	l_u8Count=0;
//	uint16	l_u16tmp=0;
//	uint16	l_u16index=0;
//	int32	l_32tmp2=0;
//	int 	Tmp_Z=0;                 //车辆高度
//	int		Tmp_Y=0;
//	int32	l_leftX=0;
//	int32	l_rightX=0;
//	/************************顺车道激光器与垂直激光器匹配出车*****************************/
//	/*第一步：清除垂直激光器两车道非匹配区域记录 */
//	/*********************************************************************
//	/***垂直激光器已收尾车辆，locatex位置在非匹配区域，直接清除
//	/***********************************************************************/
//	for(j = 0;j < g_totalVehicle;j++)	//宽高 车数
//	{
//		i = g_VehicleSetIndex[j]-1;
//		l_leftX = g_VehicleSet[i].locateX.n32xLeft;
//		l_rightX = g_VehicleSet[i].locateX.n32xRight;
//		if(g_VehicleSet[i].u8Vstate==PASSED_USED 
//		//	&& (((l_leftX <-1*(g_sspSetup.u8LaneNum-1)*g_LaneWide/2) && ((l_leftX + l_rightX)/2<-1*(g_sspSetup.u8LaneNum-2)*g_LaneWide/2-200)) || l_rightX>0))
//		  &&l_leftX<0 || l_leftX> l_rightX || l_rightX>g_sspSetup.LaserDistance)
//		
//		{
//			memset(&g_VehicleSet[i],0,sizeof(VehicleStruct));		 
//			g_VehicleSet[i].u8Vstate = NO_USED;  
//			for(l_u16tmp = j;l_u16tmp < g_totalVehicle - 1;l_u16tmp++)
//				g_VehicleSetIndex[l_u16tmp] = g_VehicleSetIndex[l_u16tmp+1];
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
//	/*第二步：开始匹配
//	/*****************************************************************************/
//	/*匹配准则：长、高接近，且位置在匹配区域；匹配成功出车后双清
//	/*以垂直激光器为基准，顺车道与垂直激光器匹配：
//	/*①顺车道记录集中无车与垂直激光器匹配，清除垂直激光器该车信息或估计一个车速出车
//	/*②顺车道中有车与垂直激光器匹配，则匹配成功出车双清
//	/*****************************************************************************/
//	for(j = 0;j < g_totalVehicle;j++)
//	{
//		i = g_VehicleSetIndex[j] -1;
//		if(g_VehicleSet[i].u8Vstate==PASSED_USED)		  //垂直  收尾
//		{
//			l_leftX = g_VehicleSet[i].locateX.n32xLeft;	 //左边界
//			l_rightX = g_VehicleSet[i].locateX.n32xRight;//右边界
//			if (l_leftX>=-1*(g_sspSetup.u8LaneNum-1)*g_LaneWide/2 && l_rightX<=0 && l_leftX != 0)//20140922
//			{
//				//激光器2
//				for(m=0;m<g_VehIncTotal;m++)
//				{
//					k = g_VehIncSetIndex[m]-1;
//					if(g_VehIncSet[k].u8Istate == PASSED_USED)
//					{
//						l_32tmp2 = g_VehIncSet[k].Idata.u16FrameCnt &  FRAME_MASK;
//						if ((l_32tmp2>2) && (g_VehIncSet[k].IemptFrame < ERR_MAX_EMPTYFRAME))
//						{
//							Tmp_Z = 0;
//							Tmp_Y = 0;
//							g_VehIncSet[k].zLen = GetMaxData(g_VehIncSet[k].Idata.zMax, l_32tmp2/3, l_32tmp2-1);		
//	
//							/*垂直激光长高求解*/
//							l_32tmp2 = g_VehicleSet[i].Vdata.u16FrameCnt &  FRAME_MASK;
//							Tmp_Z=GetVHeight(&g_VehicleSet[i].Vdata, l_32tmp2);
//							if (Tmp_Z<1000)
//							{
//								g_VehicleSet[i].xLen = GetMaxData(g_VehicleSet[k].Vdata.xMax, 0, l_32tmp2-1);
//								if (g_VehicleSet[i].xLen<2500)
//								{
//									Tmp_Z = Myrand(1400,1700);
//								}
//							}
//							Tmp_Y=abs((int)(g_VehicleSet[i].Vdata.tdata[l_32tmp2-1]-g_VehicleSet[i].Vdata.tdata[0])) * g_VehIncSet[k].speed/36;
//							if (Tmp_Z<1800 && g_VehicleSet[i].xLen<2000) //低于1800mm 宽小于2000mm 小车
//							{
//								if (Tmp_Y>6000)		  //长大于6000mm
//								{
//									Tmp_Y = Myrand(3500,5000);
//								}
//							}
//							if (g_VehIncSet[k].yLen == 0)//20140914	 速度得到的长度为0 
//							{
//								if (g_VehIncSet[k].u8ThrowFlag==1)	 //抛车
//								{
//									l_32tmp2 = g_VehIncSet[k].Idata.u16FrameCnt &  FRAME_MASK;
//									g_VehIncSet[k].yLen = GetMaxData(g_VehIncSet[k].Idata.yMax, 0, l_32tmp2-1);//测量得到车长
//								}
//							}
//							else if (g_VehIncSet[k].yLen>0)		//速度计算得到的顺车道 车长
//							{	
//								l_32tmp2 = g_VehIncSet[k].Idata.u16FrameCnt &  FRAME_MASK;
//								g_VehIncSet[k].ndeltaY = GetMaxData(g_VehIncSet[k].Idata.yMax, l_32tmp2/3, l_32tmp2*2/3);
//								if (abs(g_VehIncSet[k].ndeltaY-g_VehIncSet[k].yLen)>1000)	//以测量为准
//								{
//									g_VehIncSet[k].yLen = g_VehIncSet[k].ndeltaY;
//								}
//								else if (g_VehIncSet[k].ndeltaY>0 && g_VehIncSet[k].yLen>0 
//									&& g_VehIncSet[k].ndeltaY<6000 && g_VehIncSet[k].ndeltaY>g_VehIncSet[k].yLen)//20140919
//								{
//									g_VehIncSet[k].yLen = g_VehIncSet[k].ndeltaY;
//								}
//							}
//							l_u8Count = 0;
//							if (g_VehIncSet[k].yLen<7000) //速度计算得到的长度小于7000mm
//							{
//								l_32tmp2 = g_VehIncSet[k].Idata.u16FrameCnt &  FRAME_MASK;
//								for (l_u16index=0;l_u16index<l_32tmp2; l_u16index++)
//								{													 //打飞 异常处理
//									if (g_VehIncSet[k].Idata.ydataInfo[l_u16index][4]==1 && g_VehIncSet[k].Idata.ydataInfo[l_u16index][5]==1)
//									{
//										l_u8Count++;
//									}
//								}
//								//一半打飞 高度小1200  随机出高
//								if (l_u8Count>l_32tmp2/2 && g_VehIncSet[k].zLen<1200)//20140922
//								{
//									g_VehIncSet[k].zLen = Myrand(1400,1700);
//								}
//							}
//							/// 三个长度
//							if( ((g_VehIncSet[k].yLen+2500 >= Tmp_Y && g_VehIncSet[k].yLen<Tmp_Y+2500)
//										||(g_VehIncSet[k].zLen<6000 && abs(g_VehIncSet[k].yLen-Tmp_Y)<2000 && g_VehIncSet[k].zLen<2000 && Tmp_Z<2000))
//								 && g_VehIncSet[k].zLen+1500>=Tmp_Z && g_VehIncSet[k].zLen <=Tmp_Z+300)//
//							{
//								if (abs(g_VehIncSet[k].yLen-Tmp_Y)<1000 && Tmp_Y>6000)//20140914
//								{
//									if (abs(g_VehIncSet[k].yLen-Tmp_Y)<300 && g_VehIncSet[k].yLen>Tmp_Y && g_VehIncSet[k].yLen>g_VehIncSet[k].ndeltaY)
//									{
//										g_VehicleSet[i].yLen = g_VehIncSet[k].yLen;
//									}
//									else
//									{
//										g_VehicleSet[i].yLen = Tmp_Y;	//长
//									}									
//								}
//								else
//								{
//									g_VehicleSet[i].yLen = g_VehIncSet[k].yLen;
//								}
//								g_VehicleSet[i].speed = (g_VehIncSet[k].speed+5)/10;
//								VehModels2(&g_VehicleSet[i]);  				//车道2出车
//								/*单清顺车道*/
//								memset(&g_VehIncSet[k], 0,sizeof(VehIncSt)); //清除顺道激光器该记录
//								g_VehIncSet[k].u8Istate = NO_USED; 
//								for(l_u16tmp = m;l_u16tmp < g_VehIncTotal - 1;l_u16tmp++)
//									g_VehIncSetIndex[l_u16tmp] = g_VehIncSetIndex[l_u16tmp+1];
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
//				//激光器3
//				for(m=0;m<g_VehIncTotal3;m++)
//				{
//				
//				
//				}
//			}
//			memset(&g_VehicleSet[i], 0,sizeof(VehicleStruct));
//			g_VehicleSet[i].u8Vstate = NO_USED;  
//			for(l_u16tmp = j;l_u16tmp < g_totalVehicle - 1;l_u16tmp++)
//				g_VehicleSetIndex[l_u16tmp] = g_VehicleSetIndex[l_u16tmp+1];
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
//}
/******************************************************/
//计算每帧数据的有车区域高度二次均值作为该区域的车高
int GetFramAreaHeight(int *pg_ZV, uint16 u16StartPt, uint16 u16EndPt)
{
	/*室内模拟用**/
	int Tmpj = 0;
	int Tmpi=0;
	int RetHeight = 0;
	int ThdHeight = 0;
	int SecHeight = 0;
	int MaxHeight = 0;
	int NewHeight = 0;
	int u8PtNum = u16EndPt - u16StartPt + 1;
	if (u8PtNum == 0)
	   return RetHeight;
	else if (u8PtNum == 1)
	{
		RetHeight = pg_ZV[u16StartPt];	
		return RetHeight;
	}
	
	//%先检查是否有异常点 最大检查2个点异常
	for (Tmpi =u16StartPt;Tmpi<=u16EndPt;Tmpi++)
	{  
		if (MaxHeight < pg_ZV[Tmpi])
		{
		    ThdHeight = SecHeight;
		    SecHeight = MaxHeight;
		    MaxHeight = pg_ZV[Tmpi];
		}  
		else if (SecHeight < pg_ZV[Tmpi] &&(MaxHeight>pg_ZV[Tmpi]))
		{
		    ThdHeight = SecHeight;
		    SecHeight = pg_ZV[Tmpi];   
		}  
		else if (ThdHeight < pg_ZV[Tmpi] && SecHeight>pg_ZV[Tmpi] && MaxHeight>pg_ZV[Tmpi])
		{
		    ThdHeight = pg_ZV[Tmpi];
		}  
	}
	
	if (ThdHeight && SecHeight > ThdHeight + 30 && SecHeight > 140)  //%2个点异常 
	{ 
	    for(Tmpi = u16StartPt; Tmpi<=u16EndPt;Tmpi++)
		{  
	        if (SecHeight <= pg_ZV[Tmpi] && Tmpi > u16StartPt)
			{
	            pg_ZV[Tmpi] = pg_ZV[Tmpi-1];
			}
	        else if (SecHeight <= pg_ZV[Tmpi])	   		
			{
	            pg_ZV[Tmpi] = 100;
	            if (NewHeight < pg_ZV[Tmpi])
				{
	              NewHeight = pg_ZV[Tmpi];
				}
			}
		}
	}
	else if (SecHeight && MaxHeight > SecHeight + 30 && MaxHeight > 160) //%有一个点异常高度
	{
	    for(Tmpi = u16StartPt; Tmpi<=u16EndPt;Tmpi++)
		{
	        if (MaxHeight == pg_ZV[Tmpi] && Tmpi > u16StartPt)
			{
	            pg_ZV[Tmpi] = pg_ZV[Tmpi-1];
			}
	        else if (MaxHeight == pg_ZV[Tmpi])
			{
	            pg_ZV[Tmpi] = 100;
	            //%重新找高度
	            if (NewHeight < pg_ZV[Tmpi])
				{
	               NewHeight = pg_ZV[Tmpi];
				}
	        }
	    }	
	}
	
	if (NewHeight > 500)
	{
	    for(Tmpi = u16StartPt; Tmpi<=u16EndPt;Tmpi++)
		{
	        if (NewHeight - pg_ZV[Tmpi] < 20 && Tmpj <5)	//%%最多5个点 将500改为200
			{
	            RetHeight =RetHeight+ pg_ZV[Tmpi];
	            Tmpj=Tmpj+1;
	        }
	    }
	}
	else if (NewHeight>0)
	{
	    for(Tmpi=u16StartPt; Tmpi<=u16EndPt;Tmpi++)
		{
	        if (NewHeight - pg_ZV[Tmpi] < 35)
			{ 
	            RetHeight= RetHeight+pg_ZV[Tmpi];
	            Tmpj=Tmpj+1;
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

/*******寻找当前帧当前有车区域与有车记录集匹配索引号**************/
int SeachAreaMatchIndex(PointStruct FramInfo)
{
	int Searchidx=-1;
	int m;
	int j;
	uint16 l_leftX = FramInfo.n32xLeft;	   //%每帧中有车部分的起点
	uint16 l_rightX =FramInfo.n32xRight;	//%有车部分的结束点
	uint16 l_leftXpt = FramInfo.u16Leftpt;
	uint16 l_rightXpt = FramInfo.u16Rightpt;
	uint16 l_ul6Dix=FramInfo.u16xDis;       //车辆宽度
	// l_Hmax=FramInfo.u16xMaxHt;
	if(l_leftXpt>=l_rightXpt||g_totalVehicle>VEHICLE_MAX)
	{
	   Searchidx=ERRORVALUE;  //返回错误
	   return Searchidx;
	}
//	if((abs(g_XV[l_leftXpt]-g_XV[l_rightXpt]) > 4000) )
//	//	if(abs(g_XV[l_leftXpt]-g_XV[l_rightXpt])-abs(g_XV[l_leftXpt]-g_XV[l_rightXpt]))
//	       return ERRORVALUE;
	for (j=0;j<g_totalVehicle;j++)
	{
	    m = (g_VehicleSetIndex[j]-1)&VEHICLE_MASK;
	    if(g_VehicleSet[m].u8Vstate == OCCURING_USED||g_VehicleSet[m].u8Vstate == NO_USED)
		{
			//两侧点有交汇
	        if(IS_INSIDE(l_leftX,l_rightX,g_VehicleSet[m].VLocateX.n32xLeft,g_VehicleSet[m].VLocateX.n32xRight))
			{
	      //      if(abs(l_ul6Dix-g_VehicleSet[m].Vdata.u16xDis[g_VehicleSet[m].Vdata.u16FrameCnt])<50 )
				{
	                Searchidx=m;
	                break;
	            }
	        }
	    }
	}
	return Searchidx;  //返回-1新车 或 车的帧号
}
uint16 GetVehLane(VehicleStruct *PlocateX);
//出车
void OutPutVeh(VehicleStruct *pVehicle)
{
	 uint16 u16FrameCnt=0;  
	 uint16 Width=0; 
	 uint16 Height=0;
	 uint8  lane = 0;
	 uint8  cnt=0;
	 uint8 U5Buff[15]={0x00};
	 u16FrameCnt=pVehicle->Vdata.u16FrameCnt;
	 g_count = 	 u16FrameCnt;
	 if(u16FrameCnt<5)
	            return;
	 Height=GetVehicleHeight(pVehicle->Vdata.u16xMaxHt,u16FrameCnt);
	 if(Height<1000)
	    return ;
	 Width =GetVehWidth(pVehicle->Vdata.u16xDis,u16FrameCnt, Height);
	 if( Width <1000)
	 {
	 	  return ;
	 
	 }

	 lane = GetVehLane(pVehicle);
	 if(lane==0)
	 {	
	  //	lane = 1;
	 	return ;
	 }
	 if(g_sspSetup.u32DevID==1)
	 {
		 if(lane == 2&&Width>1700)
		 {
		     Width -= 40;
			
		 }
		 if( Width>2500&&Height>2500)
			     Width -= 50;
	}
	 g_u32VehWideHeigh[g_u16Recvcount][0] = t0_count2;
	 g_u32VehWideHeigh[g_u16Recvcount][1] = lane;
	 g_u32VehWideHeigh[g_u16Recvcount][2] = Width;
	 g_u32VehWideHeigh[g_u16Recvcount][3] = Height;
	 g_u16Recvcount = (g_u16Recvcount+1)%10;

	 OSSemPost(g_sendVeh);
	 cnt=0;
	 U5Buff[cnt++]=lane;
	 U5Buff[cnt++]=0;	 
	 U5Buff[cnt++]=0;
	 U5Buff[cnt++]=0;
	 U5Buff[cnt++]=0;

	 U5Buff[cnt++]=0;
	 U5Buff[cnt++]=0;
	 U5Buff[cnt++]=Width>>8;
	 U5Buff[cnt++]=Width;

	 U5Buff[cnt++]=0;
	 U5Buff[cnt++]=0;
	 U5Buff[cnt++]=Height>>8;
	 U5Buff[cnt++]=Height;
	 U5Buff[cnt++]=u16FrameCnt;


}
uint16 GetVehLane(VehicleStruct *pVehiclet)
{
       uint16 i;
	   int j;
	   uint32 sum=0;
	   uint16 location;
	   if(pVehiclet->Vdata.u16FrameCnt==0)
		 return 0;

	   for(i=0;i<pVehiclet->Vdata.u16FrameCnt ;i++)
	   {
		  j = pVehiclet->Vdata.xdata[i][0];	 
		  sum += (pVehiclet->Vdata.xdata[i][1]+ pVehiclet->Vdata.xdata[i][j])/2;

	   }
	   location =  sum / pVehiclet->Vdata.u16FrameCnt ;
	   if(location>3500+g_sspSetup.MedianWide&&location<7000+g_sspSetup.MedianWide)
	   		return 2;
	   else	if(location<3500+g_sspSetup.MedianWide&& location>g_sspSetup.MedianWide)
	   		return 1;
	   else 
			return 0;
}
//返回车辆高度
uint16 GetVehicleHeight(uint16 *PxMaxHt,uint16 u16FrameCnt)
{
	int Tmpj = 0;
	int Tmpi=0;
	int Height = 0;
	int ThdHeight = 0;
	int SecHeight = 0;
	int MaxHeight = 0;
	int NewHeight = 0;
	uint16 u16StartPt=0;  //
	uint16 u16EndPt=u16FrameCnt;  //4
	int u8PtNum = u16EndPt - u16StartPt; //5
	if (u8PtNum == 0)
	   return Height;
	else if (u8PtNum == 1)
	{
		Height = PxMaxHt[u16StartPt];	
		return Height;
	}
	
	//%先检查是否有异常点 最大检查2个点异常
	for (Tmpi =u16StartPt;Tmpi<u16EndPt;Tmpi++)
	{  
		if (MaxHeight < PxMaxHt[Tmpi])
		{
		    ThdHeight = SecHeight;
		    SecHeight = MaxHeight;
		    MaxHeight = PxMaxHt[Tmpi];
		}  
		else if (SecHeight < PxMaxHt[Tmpi] &&(MaxHeight>PxMaxHt[Tmpi]))
		{
		    ThdHeight = SecHeight;
		    SecHeight = PxMaxHt[Tmpi];   
		}  
		else if (ThdHeight < PxMaxHt[Tmpi] && SecHeight>PxMaxHt[Tmpi] && MaxHeight>PxMaxHt[Tmpi])
		{
		    ThdHeight = PxMaxHt[Tmpi];
		}  
	}
	
	if (ThdHeight && SecHeight > ThdHeight + 30 && SecHeight > 140)  //%2个点异常 
	{ 
	    for(Tmpi = u16StartPt; Tmpi<=u16EndPt;Tmpi++)
		{  
	        if (SecHeight <= PxMaxHt[Tmpi] && Tmpi > u16StartPt)
			{
	            PxMaxHt[Tmpi] = PxMaxHt[Tmpi-1];
			}
	        else if (SecHeight <= PxMaxHt[Tmpi])	   		
			{
	            PxMaxHt[Tmpi] = 100;
	            if (NewHeight < PxMaxHt[Tmpi])
				{
	              NewHeight = PxMaxHt[Tmpi];
				}
			}
		}
	}
	else if (SecHeight && MaxHeight > SecHeight + 30 && MaxHeight > 160) //%有一个点异常高度
	{
	    for(Tmpi = u16StartPt; Tmpi<=u16EndPt;Tmpi++)
		{
	        if (MaxHeight == PxMaxHt[Tmpi] && Tmpi > u16StartPt)
			{
	            PxMaxHt[Tmpi] = PxMaxHt[Tmpi-1];
			}
	        else if (MaxHeight == PxMaxHt[Tmpi])
			{
	            PxMaxHt[Tmpi] = 100;
	            //%重新找高度
	            if (NewHeight < PxMaxHt[Tmpi])
				{
	               NewHeight = PxMaxHt[Tmpi];
				}
	        }
	    }	
	}
	
	if (NewHeight > 500)
	{
	    for(Tmpi = u16StartPt; Tmpi<=u16EndPt;Tmpi++)
		{
	        if (NewHeight - PxMaxHt[Tmpi] < 20 && Tmpj <5)	//%%最多5个点 将500改为200
			{
	            Height =Height+ PxMaxHt[Tmpi];
	            Tmpj=Tmpj+1;
	        }
	    }
	}
	else if (NewHeight>0)
	{
	    for(Tmpi=u16StartPt; Tmpi<=u16EndPt;Tmpi++)
		{
	        if (NewHeight - PxMaxHt[Tmpi] < 35)
			{ 
	            Height= Height+PxMaxHt[Tmpi];
	            Tmpj=Tmpj+1;
	        }
	    }
	}
	if (Tmpj < 1)
	{
		if (NewHeight>0)
		{
			Height = NewHeight;		
		}
		else
		{
			Height = (MaxHeight+SecHeight+ThdHeight)/3;
		}

	}
	else
	{
		Height = Height/Tmpj;    //计算出的第2次均值
	}

  return Height;

}
//返回宽度
uint16 GetVehWidth(uint16 *PxDis,uint16 u16FrameCnt,uint16 u16heigh)
{

 uint16 Width=0;
 int i=0;
 uint16 width1=0;
 uint16 width2=0;
 uint16 width3=0;
 uint16 width4=0;
 uint16 width5=0;
 uint16 width6=0;
 uint16 width7=0;
 uint16 avg = 0;
 
 for(i=0;i<u16FrameCnt;i++)
 {
   if(width1<PxDis[i])
   {
   		width6 = width5;
		width5 = width4;
   		width4 = width3;
   		width3 = width2;
		width2 = width1;
		width1= PxDis[i];
	}
   else if(width2<PxDis[i])
   {	
   		width6 = width5;
		width5 = width4;
   		width4 = width3;
   		width3 = width2;
		width2=PxDis[i];
	}
   else if(width3<PxDis[i])
   {
   		width6 = width5;
   		width5 = width4;
		width4 = width3;
		width3=PxDis[i];
   }
   else	if(width4<PxDis[i])
   {
    	width6 = width5;
		width5 = width4;
		width4 = PxDis[i];
   }
   else if(width5<PxDis[i])
   {
		width6 = width5;
		width5=PxDis[i];
	}
   else if(width6<PxDis[i])
		width6=PxDis[i];
   
 }
// if(width7>0)
// 	avg = (width1+width2+width3+width4+width5+width6+width7)/7;
// else 
if(width6>0)
{
	if(u16heigh>2000)
 		avg = (width2+width3+width4+width5+width6 )/5;
	else
		avg = (width2+width3)/2;
}
 else if(width5>0)
 {
 	if(u16heigh>2000)
 		avg = (width2+width3+width4+width5 )/4;
		
	else 
		avg =  (width2+width3)/2;
}
 else if(width4>0)
 {
 	if(u16heigh>2000)
 		avg = (width4+width2+width3 )/3;  
	else 
		avg =  (width2+width3)/2;
}
 else if(width3>0)
 	avg = (width3+width2 )/2;
 else if(width2>0)
 	avg = width2;
 else 
    avg = width1; 

 return avg;

}