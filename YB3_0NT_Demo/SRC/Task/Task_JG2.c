/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			Task_JG2.C
** Last modified Date:  20120718
** Last Version:		1.0
** Descriptions:		激光模拟软件接收任务（网络端口2接收）
**
**--------------------------------------------------------------------------------------------------------
** Created by:			Hong XiangYuan
** Created date:		20120718
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
#include "Task_JG2.h"
#include "Task_JG1.h"
#include "Task_JG0.h"
#include "W5100.h"
#include "WT_Task.h"
#include "common.h"
#include "CMD.h"
#include "Uart1.h"
#include "crc.h"
#include "Timer0.h"
#include "Task_Data_JG.h"
#include "JG.h"

uint32 JG2_Flag_Rec=1;
uint32 g_u32count_1459=0;
uint32 JG2_T0TC[4] = {0} ;
uint32 jg2 =0 ;
 extern uint32   g_au32Tempa[5000][4];
/***************JG2入口系统参数************************/
/* JG2起点        g_sspSetup.u16StartPtNum2; 
/* JG2止点	      g_sspSetup.u16EndPtNum2
/* JG2零点	      g_sspSetup.u16VerticalZeroPos2
/* JG2高度		  g_sspSetup.HeightLaser2
/**************************************************/
/********定义JG2数据读和处理记录变量***********/
uint8 	g_u8JG2_RBuff_Count=0; //两激光缓存计数；
uint8 	g_u8JG2_PBuff_Count=0; //两激光缓存计数；
uint8 	g_au8JG2_3Buff[3][831];  //激光0数据缓存3个；
int32   LMS_data_2[362]={0};
uint8   JG2_CurBuff[831]={0};
uint8   g_au8recvBuff2[1460];
uint16  g_u16VerticalStartAnglePt2;   //垂直激光器的起始点数 20130426
uint16  g_u16VerticalEndAnglePt2;   //垂直激光器的终止点数
extern 	uint8 	g_u8JG3_PBuff_Count;
extern int32   LMS_data_3[362];
/**********************************************/
void Task_JG2(void *tdata)
{	
	uint8 err;
	uint32	i;
	uint32  j;
	uint16  l_leftXpt, l_rightXpt;   //左右X距离对应的点数
	uint16  l_u16index,l_u16tmp;
	int32   Len_2=0;
	int     m=0;
	int k=0;
	
	uint16	l_index;
	uint16	l_u16StartPt,l_u16EndPt;
	int32	l_n32StartY,l_n32EndY;
	uint32	Time_Vertical;
	uint16	l_u16StartYpt,l_u16EndYpt;
	int32   l_32tmp2,l_32tmp,l_tmp1,TempVaule1;
	//IncPtSt l_u16IncPosVect[POINTSET_CNT] = {0};	//存放并车的位置信息
    int Dafeiflag=0;
    int DafeiData[180][4]={0};
	int MaxZ=0;
	int MaxX=0;
	int MinZ=0;
	int MinX=0;
	int Maxi=0;
    int EnterX1=-10000;			  //秤台进入扫描区域
    int EnterX2=-3000;			  //输出的速度为过车头过此线时的速度
	int ExitX3=-2000;                 //车头过此线后抛车
	int ExitX4=-600;              //车头过此线后剔除车辆
    int Lengthline1=-7500;
	int Lengthline2=ExitX3;
    int Tmp_Z = 0;                 //车辆高度
	int	Tmp_Y = 0;
	PtIncSet l_FrameInfo;
//	uint8  Port3_Buff[831]={0};
//	uint8 RDid[]	 		= "0012121110010001";	//设备身份识别码
	//接收两路激光合并后数据总共1459字节；
	   //0-7开头，8-11时间；12 13:01 69;14-735（垂直）;	736- 1457（倾斜）;1458(校验)

	tdata=tdata;
//	crc_create(Send_VehInfo_Uart1,52);
//	UART1_SendBuf(Send_VehInfo_Uart1,55);
	while(1)
	{
		OSSemPend(g_JG2flag,0,&err);
        if(OS_NO_ERR == err)  //调用成功
		{
			S2_Data&=~S_RECEIVE;
			i = S_rx_process(2);
#ifndef SIM_SOFTWARE			 
			if(i != 831)
			{
	
			}
			else
			{
			  jg2++;	
				memcpy(g_au8JG2_3Buff[g_u8JG2_RBuff_Count],Rx_Buffer+2*Max_Size,Max_Size);
				g_u8JG2_RBuff_Count=g_u8JG2_RBuff_Count+1;
				g_u8JG2_RBuff_Count=g_u8JG2_RBuff_Count%3;
			}
#else 
   			if(i!=1460)
			{
			
			}
			else
			{
				memcpy(g_au8recvBuff2 , Rx_Buffer+2*Max_Size,i);
				for(i=85,j=0;i<806&&j<POINT_SUM;i=i+2,j++)
				{		
					LMS_data_2[j]=	(g_au8recvBuff2[i-71]<<8)+g_au8recvBuff2[i-70];
					LMS_data_3[j]=	(g_au8recvBuff2[i+651]<<8)+g_au8recvBuff2[i+652];	
				}  
				LMS_data_2[361] = ((g_au8recvBuff2[8]<<24)+(g_au8recvBuff2[9]<<16)+(g_au8recvBuff2[10]<<8)+g_au8recvBuff2[11])/1000;				
				LMS_data_3[361] = ((g_au8recvBuff2[8]<<24)+(g_au8recvBuff2[9]<<16)+(g_au8recvBuff2[10]<<8)+g_au8recvBuff2[11])/1000;
				OSSemPost(g_JG3flag);
			}

#endif
		}

#ifndef SIM_SOFTWARE

#else
		if(g_u8JG2_PBuff_Count!=g_u8JG2_RBuff_Count)
		{
		 	memcpy(JG2_CurBuff,g_au8JG2_3Buff[g_u8JG2_PBuff_Count],831);	
			g_u8JG2_PBuff_Count=g_u8JG2_PBuff_Count+1;
		    g_u8JG2_PBuff_Count=g_u8JG2_PBuff_Count%3;
			Len_2= (JG2_CurBuff[83]<<8)+JG2_CurBuff[84];
			if(Len_2 == POINT_SUM)
			{ 
				for(i=85,j=0;i<807 && j < POINT_SUM; i=i+2,j++)	//20130426 修改，去掉偏移量	
				{
					LMS_data_2[j]=	(JG2_CurBuff[i]<<8)+JG2_CurBuff[i+1];	 //每点座标
					if(LMS_data_2[j] < 0)									 //每点座标
					LMS_data_2[j] = 0;										 //每点座标
				}	
				LMS_data_2[361] = ((JG2_CurBuff[42]<<24)+(JG2_CurBuff[43]<<16)+(JG2_CurBuff[44]<<8)+JG2_CurBuff[45])/1000;
			}
		}

#endif
		{
		/*****************顺车道激光器求车长与车辆区域匹配***************/
		Time_Vertical = (uint32)LMS_data_2[361];
		/*寻找激光器的起止点*/
		g_u16VerticalStartAnglePt2 = GetStartEndPt(LMS_data_2, 30, 0, 2);
		g_u16VerticalEndAnglePt2	 = GetStartEndPt(LMS_data_2, 300, 1, 2);
	    k=0;		
		j=g_sspSetup.u16VerticalZeroPos2-g_u16VerticalStartAnglePt2 - 1;   
	    for(i=g_sspSetup.u16VerticalZeroPos2-1;i >= g_u16VerticalStartAnglePt2;i--)
		{
			if(LMS_data_2[i]>ThresOrigineDataLow && LMS_data_2[i]<ThresOrigineDataHigh)//极坐标下，测得的距离在0.03m到20m间
			{ 
	             g_ZdistanceI[j] = g_sspSetup.HeightLaser2 - ((LMS_data_2[i]*Tabcos[g_sspSetup.u16VerticalZeroPos2-i])>>15);
				 g_YdistanceI[j]= -1*((LMS_data_2[i]*Tabsin[g_sspSetup.u16VerticalZeroPos2-i])>>15);//扫描点的y坐标值	
				 Dafeiflag=0;			 
			}
			else															 //测得的距离不在0.03m到20m间
			{
				 if(Dafeiflag==0)
			     {
					 k++;
				     DafeiData[k][1]=j;		 //将打飞段的终止索引记录在DafeiData[k][1]中，起始索引记录在DafeiData[k][0]中，同时记录g_ZdistanceI为零时的X坐标
					 DafeiData[k][3]=-(Tabsin[g_sspSetup.u16VerticalZeroPos2-i]*(g_sspSetup.HeightLaser2-100)/Tabcos[g_sspSetup.u16VerticalZeroPos2-i]);
				 }
			     DafeiData[k][0]=j;
				 DafeiData[k][2]=-(Tabsin[g_sspSetup.u16VerticalZeroPos2-i]*(g_sspSetup.HeightLaser2-100)/Tabcos[g_sspSetup.u16VerticalZeroPos2-i]);
			     Dafeiflag=1;		 
			}
			j=j-1;	//点数减1
		} 
		DafeiData[0][0]=k;
		/*******************************************************************/	
		/*****************/
		/*JG_T0TC[1] = T0TC;
		/*JG_counter2[1]=t0_count2;
		/*****************/
		/********************顺车道激光器0点与结束点之间坐标转换************************/
		 j=g_sspSetup.u16VerticalZeroPos2-g_u16VerticalStartAnglePt2;   	 
		 for(i=g_sspSetup.u16VerticalZeroPos2;i <= g_u16VerticalEndAnglePt2;i++) 
		 {
		 	if(LMS_data_2[i] > ThresOrigineDataLow && LMS_data_2[i]<ThresOrigineDataHigh)//测得的距离在0.03m到20m间
			{ 		
				g_ZdistanceI[j] = g_sspSetup.HeightLaser2 - ((LMS_data_2[i]*Tabcos[i-g_sspSetup.u16VerticalZeroPos2])>>15);//车高
				g_YdistanceI[j] = ((LMS_data_2[i]*Tabsin[i-g_sspSetup.u16VerticalZeroPos2])>>15);//扫描点的y坐标值	
				Dafeiflag=0;
			}
			else															   //测得的距离不在0.03m到20m间
			{
			    if(Dafeiflag==0)
				 {
				 k++;
				 DafeiData[k][0]=j;
				 DafeiData[k][2]=Tabsin[i-g_sspSetup.u16VerticalZeroPos2]*(g_sspSetup.HeightLaser2-100)/Tabcos[i-g_sspSetup.u16VerticalZeroPos2];
				 }
				 DafeiData[k][1]=j;
				 DafeiData[k][3]=Tabsin[i-g_sspSetup.u16VerticalZeroPos2]*(g_sspSetup.HeightLaser2-100)/Tabcos[i-g_sspSetup.u16VerticalZeroPos2];
			     Dafeiflag=1;		       
			} 				 //计算打飞点时的X坐标时，对有隔离带情况，要使用g_sspSetup.HeightLaser	  zyj 20130607
			j=j+1;	//点数加1
		 }
		 DafeiData[0][1]=k; 
		 
		/**********对打飞点的处理:拉平处理*************/
		for(k=1; k<=DafeiData[0][1];k++ )
		{
			if(((DafeiData[k][0]==0)||(g_ZdistanceI[DafeiData[k][0]-1]<=ThresVehLow))&&(g_ZdistanceI[DafeiData[k][1]+1]>ThresVehLow))
			{
			   m=DafeiData[k][0]+g_u16VerticalStartAnglePt2;	 //m为数组data[]的索引；车尾的索引
			   if(g_sspSetup.u16VerticalZeroPos2>=m)
			   {
				 g_YdistanceI[DafeiData[k][0]]=-(Tabsin[g_sspSetup.u16VerticalZeroPos2-m]*(g_sspSetup.HeightLaser2-g_ZdistanceI[DafeiData[k][1]+1])/Tabcos[g_sspSetup.u16VerticalZeroPos2-m]);
			   }
			   else
			   {
				  g_YdistanceI[DafeiData[k][0]]=Tabsin[m-g_sspSetup.u16VerticalZeroPos2]*(g_sspSetup.HeightLaser2-g_ZdistanceI[DafeiData[k][1]+1])/Tabcos[m-g_sspSetup.u16VerticalZeroPos2];
			   }
				g_ZdistanceI[DafeiData[k][0]] = ThresVehLow+1;//20140915 打飞点处的修正
				for(i=DafeiData[k][0]+1;i<=DafeiData[k][1];i++)
				{
					g_ZdistanceI[i]=g_ZdistanceI[DafeiData[k][1]+1];
					g_YdistanceI[i]=g_YdistanceI[DafeiData[k][0]]+((g_YdistanceI[DafeiData[k][1]+1]-g_YdistanceI[DafeiData[k][0]])*(i-DafeiData[k][0]))/(DafeiData[k][1]-DafeiData[k][0]+1);
				}
			}
			//////对于车头打飞的处理
			else if((DafeiData[k][1]==g_u16VerticalEndAnglePt2-g_u16VerticalStartAnglePt2||g_ZdistanceI[DafeiData[k][1]+1]<=ThresVehLow)&&(g_ZdistanceI[DafeiData[k][0]-1]>ThresVehLow))
			{
				m=DafeiData[k][1]+g_u16VerticalStartAnglePt2;	 //m为数组data[]的索引；车头的索引
				if(g_sspSetup.u16VerticalZeroPos2>=m)
				{
					g_YdistanceI[DafeiData[k][1]]=-(Tabsin[g_sspSetup.u16VerticalZeroPos2-m]*(g_sspSetup.HeightLaser2-ThresVehLow/2)/Tabcos[g_sspSetup.u16VerticalZeroPos2-m]);
				}
				else
				{
					g_YdistanceI[DafeiData[k][1]]=Tabsin[m-g_sspSetup.u16VerticalZeroPos2]*(g_sspSetup.HeightLaser2-ThresVehLow/2)/Tabcos[m-g_sspSetup.u16VerticalZeroPos2];
				}
				g_ZdistanceI[DafeiData[k][1]]=ThresVehLow+1;//20140915 ThresVehLow/2->ThresVehLow+1
				for(i=DafeiData[k][0];i<DafeiData[k][1];i++)
				{
					g_ZdistanceI[i]=g_ZdistanceI[DafeiData[k][0]-1];
					g_YdistanceI[i]=g_YdistanceI[DafeiData[k][0]-1]+((g_YdistanceI[DafeiData[k][1]]-g_YdistanceI[DafeiData[k][0]-1])*(i-DafeiData[k][0]+1))/(DafeiData[k][1]-DafeiData[k][0]+1);
				}			
			}
			///////对于车身全部打飞的情况
			else if((g_ZdistanceI[DafeiData[k][0]-1]<=ThresVehLow)
					&&(g_ZdistanceI[DafeiData[k][1]+1]<=ThresVehLow)
					&&DafeiData[k][0]<DafeiData[k][1])	//20140121 增加打飞点位置不相等
		    {
				m=DafeiData[k][0]+g_u16VerticalStartAnglePt2;	 //m为数组data[]的索引；车尾的索引
				if(g_sspSetup.u16VerticalZeroPos2>=m)
				{
					g_YdistanceI[DafeiData[k][0]]=-(Tabsin[g_sspSetup.u16VerticalZeroPos2-m]*(g_sspSetup.HeightLaser2-500)/Tabcos[g_sspSetup.u16VerticalZeroPos2-m]);
				}
				else
				{
					g_YdistanceI[DafeiData[k][0]]=Tabsin[m-g_sspSetup.u16VerticalZeroPos2]*(g_sspSetup.HeightLaser2-500)/Tabcos[m-g_sspSetup.u16VerticalZeroPos2];
				}
				m=DafeiData[k][1]+g_u16VerticalStartAnglePt2;	 //m为数组data[]的索引；车头的索引
				if(g_sspSetup.u16VerticalZeroPos2>=m)
				{
					g_YdistanceI[DafeiData[k][1]]=-(Tabsin[g_sspSetup.u16VerticalZeroPos2-m]*(g_sspSetup.HeightLaser2-ThresVehLow/2)/Tabcos[g_sspSetup.u16VerticalZeroPos2-m]);
				}
				else
				{
					g_YdistanceI[DafeiData[k][1]]=Tabsin[m-g_sspSetup.u16VerticalZeroPos2]*(g_sspSetup.HeightLaser2-ThresVehLow/2)/Tabcos[m-g_sspSetup.u16VerticalZeroPos2];
				}
				for(i=DafeiData[k][0];i<=DafeiData[k][1];i++)
				{
					g_ZdistanceI[i]=ThresVehLow+1;
					g_YdistanceI[i]=g_YdistanceI[DafeiData[k][0]]+((g_YdistanceI[DafeiData[k][1]]-g_YdistanceI[DafeiData[k][0]])*(i-DafeiData[k][0]))/(DafeiData[k][1]-DafeiData[k][0]);
				}
		    }
			/////////对于车身中间段有打飞的情况
			else if((g_ZdistanceI[DafeiData[k][0]-1]>ThresVehLow)&&(g_ZdistanceI[DafeiData[k][1]+1]>ThresVehLow))
			    {
				for(i=DafeiData[k][0]; i<=DafeiData[k][1];i++)
				  {
					  g_ZdistanceI[i]=g_ZdistanceI[DafeiData[k][0]-1]+((g_ZdistanceI[DafeiData[k][1]+1]-g_ZdistanceI[DafeiData[k][0]-1])*(i-DafeiData[k][0]+1))/(DafeiData[k][1]-DafeiData[k][0]+2);
					  g_YdistanceI[i]=g_YdistanceI[DafeiData[k][0]-1]+((g_YdistanceI[DafeiData[k][1]+1]-g_YdistanceI[DafeiData[k][0]-1])*(i-DafeiData[k][0]+1))/(DafeiData[k][1]-DafeiData[k][0]+2);
				  }
			    }
		}	  
		/******************打飞点处理结束********************/	
		/***************************顺车道激光器扫描数据坐标转换完毕*******************************/
		/*****************分车第一阶段：预分车------找有效数据区域，将有效数据存到TmpNum[k][]数组中******************/
		l_u16StartPt = 0;
		memset(&l_FrameInfo, 0, sizeof(l_FrameInfo));
		l_u16EndPt = g_u16VerticalEndAnglePt2-g_u16VerticalStartAnglePt2+1;
		l_u16index = l_FrameInfo.u8Sum;
		for (i=l_u16StartPt; i<l_u16EndPt; i++)
		{							
			if ((g_ZdistanceI[i] >= ThresVehLow)&&(g_ZdistanceI[i] <= ThresVehHigh))
			{
				if (!l_FrameInfo.uValid[l_u16index])
				{
					l_FrameInfo.uValid[l_u16index] = 1;
					l_FrameInfo.IncPtdata[l_u16index].u16Pt1 = i;
					l_FrameInfo.IncPtdata[l_u16index].u16Pt2 = i;
					l_FrameInfo.IncPtdata[l_u16index].n32y1 = g_YdistanceI[i];
					l_FrameInfo.IncPtdata[l_u16index].n32y2 = g_YdistanceI[i];
					l_FrameInfo.IncPtdata[l_u16index].u16yDis = 0;
					l_FrameInfo.IncPtdata[l_u16index].u16yMaxHt = g_ZdistanceI[i];
					if (l_u16index)
					{
						l_u16tmp = abs(l_FrameInfo.IncPtdata[l_u16index - 1].n32y1 - l_FrameInfo.IncPtdata[l_u16index - 1].n32y2);
						l_FrameInfo.IncPtdata[l_u16index - 1].u16yDis = l_u16tmp;	  //该帧数据中有车部分的宽度				
					 	l_u16StartYpt = l_FrameInfo.IncPtdata[l_u16index-1].u16Pt1;
						l_u16EndYpt = l_FrameInfo.IncPtdata[l_u16index-1].u16Pt2;
						if (l_u16tmp>500 && l_u16EndYpt-l_u16StartYpt+1>=3)
						{
							l_FrameInfo.u8Sum = (l_FrameInfo.u8Sum+1)&POINTSET_MASK;
						} //中间出现间断点的情况
						else if ((abs(l_u16EndYpt-l_FrameInfo.IncPtdata[l_u16index].u16Pt1)<=3 && abs(l_FrameInfo.IncPtdata[l_u16index].n32y1 - l_FrameInfo.IncPtdata[l_u16index-1].n32y2)<800)||
							(abs(l_u16EndYpt-l_FrameInfo.IncPtdata[l_u16index].u16Pt1)<=10 && (abs(g_ZdistanceI[i]-l_FrameInfo.IncPtdata[l_u16index-1].u16yMaxHt)<500 || abs(l_FrameInfo.IncPtdata[l_u16index].n32y1 - l_FrameInfo.IncPtdata[l_u16index-1].n32y2)<500)))
						{
							l_FrameInfo.IncPtdata[l_u16index-1].u16Pt2 = i;
							l_FrameInfo.IncPtdata[l_u16index-1].n32y2 = g_YdistanceI[i];
							l_FrameInfo.IncPtdata[l_u16index-1].u16yDis = abs(l_FrameInfo.IncPtdata[l_u16index-1].n32y2-l_FrameInfo.IncPtdata[l_u16index-1].n32y1);											
							l_32tmp = l_FrameInfo.IncPtdata[l_u16index-1].u16yMaxHt;	
							if(g_ZdistanceI[i] > l_32tmp)
							{
								l_FrameInfo.IncPtdata[l_u16index-1].u16yMaxHt = g_ZdistanceI[i];  //取最大值为高				
							}
							l_FrameInfo.uValid[l_u16index] = 0;
							memset(&l_FrameInfo.IncPtdata[l_u16index],0,sizeof(IncPtSt));
							l_u16index--;
						}
						else if (l_FrameInfo.u8Sum == 1)
						{
							memcpy(&l_FrameInfo.IncPtdata[l_u16index - 1],&l_FrameInfo.IncPtdata[l_u16index],sizeof(IncPtSt));
							l_FrameInfo.uValid[l_u16index] = 0;
							memset(&l_FrameInfo.IncPtdata[l_u16index],0,sizeof(IncPtSt));								
							l_u16index--;					
						}
						else
						{
							l_FrameInfo.u8Sum = (l_FrameInfo.u8Sum+1)&POINTSET_MASK;
						}
					}
					else
					{
						l_FrameInfo.u8Sum = (l_FrameInfo.u8Sum + 1) & POINTSET_MASK;
					}													
				}
				else
				{
					l_FrameInfo.IncPtdata[l_u16index].u16Pt2 = i;
					l_FrameInfo.IncPtdata[l_u16index].n32y2 = g_YdistanceI[i];
					l_FrameInfo.IncPtdata[l_u16index].u16yDis = abs(l_FrameInfo.IncPtdata[l_u16index].n32y2-l_FrameInfo.IncPtdata[l_u16index].n32y1);
					l_32tmp = l_FrameInfo.IncPtdata[l_u16index].u16yMaxHt;	
					if(g_ZdistanceI[i] > l_32tmp)
					{
						l_FrameInfo.IncPtdata[l_u16index].u16yMaxHt = g_ZdistanceI[i];  //取最大值为高				
					}				
				}			
			}
			else
			{
				l_u16index = l_FrameInfo.u8Sum;
			}
		}
		if (l_FrameInfo.u8Sum)
		{
			l_u16index = l_FrameInfo.u8Sum - 1;
			l_u16tmp = abs(l_FrameInfo.IncPtdata[l_u16index].n32y1 - l_FrameInfo.IncPtdata[l_u16index].n32y2);
			l_FrameInfo.IncPtdata[l_u16index].u16yDis = l_u16tmp;	  //该帧数据中有车部分的宽度				
		 	l_u16StartYpt = l_FrameInfo.IncPtdata[l_u16index].u16Pt1;
			l_u16EndYpt = l_FrameInfo.IncPtdata[l_u16index].u16Pt2;
			if (l_u16tmp<=500 || l_u16EndYpt-l_u16StartYpt+1<=3)
			{
				if (l_FrameInfo.u8Sum>=2)
				{
					if (abs(l_FrameInfo.IncPtdata[l_FrameInfo.u8Sum-2].u16Pt2-l_u16StartYpt)<=5 
						&& abs(l_FrameInfo.IncPtdata[l_FrameInfo.u8Sum-2].n32y2-l_FrameInfo.IncPtdata[l_u16index].n32y1)<500)
					{					
						l_FrameInfo.IncPtdata[l_FrameInfo.u8Sum-2].u16Pt2 = l_FrameInfo.IncPtdata[l_u16index].u16Pt2;
						l_FrameInfo.IncPtdata[l_FrameInfo.u8Sum-2].n32y2 = l_FrameInfo.IncPtdata[l_u16index].n32y2;
						l_FrameInfo.IncPtdata[l_FrameInfo.u8Sum-2].u16yDis = abs(l_FrameInfo.IncPtdata[l_FrameInfo.u8Sum-2].n32y2-l_FrameInfo.IncPtdata[l_FrameInfo.u8Sum-2].n32y1);											
						l_32tmp = l_FrameInfo.IncPtdata[l_FrameInfo.u8Sum-2].u16yMaxHt;	
						if(l_FrameInfo.IncPtdata[l_u16index].u16yMaxHt > l_32tmp)
						{
							l_FrameInfo.IncPtdata[l_FrameInfo.u8Sum-2].u16yMaxHt = l_FrameInfo.IncPtdata[l_u16index].u16yMaxHt;  //取最大值为高				
						}
						l_FrameInfo.uValid[l_u16index] = 0;
						memset(&l_FrameInfo.IncPtdata[l_u16index], 0, sizeof(IncPtSt));
						l_FrameInfo.u8Sum--;
					}
					else if (l_u16EndYpt-l_u16StartYpt+1<=3 && l_u16tmp<=500 && l_FrameInfo.IncPtdata[l_u16index].n32y1-l_FrameInfo.IncPtdata[l_FrameInfo.u8Sum-2].n32y2<500)
					{
						l_FrameInfo.IncPtdata[l_FrameInfo.u8Sum-2].u16Pt2 = l_FrameInfo.IncPtdata[l_u16index].u16Pt2;
						l_FrameInfo.IncPtdata[l_FrameInfo.u8Sum-2].n32y2 = l_FrameInfo.IncPtdata[l_u16index].n32y2;
						l_FrameInfo.IncPtdata[l_FrameInfo.u8Sum-2].u16yDis = abs(l_FrameInfo.IncPtdata[l_FrameInfo.u8Sum-2].n32y2-l_FrameInfo.IncPtdata[l_FrameInfo.u8Sum-2].n32y1);											
						l_32tmp = l_FrameInfo.IncPtdata[l_FrameInfo.u8Sum-2].u16yMaxHt;	
						if(l_FrameInfo.IncPtdata[l_u16index].u16yMaxHt > l_32tmp)
						{
							l_FrameInfo.IncPtdata[l_FrameInfo.u8Sum-2].u16yMaxHt = l_FrameInfo.IncPtdata[l_u16index].u16yMaxHt;  //取最大值为高				
						}
						l_FrameInfo.uValid[l_u16index] = 0;
						memset(&l_FrameInfo.IncPtdata[l_u16index], 0, sizeof(IncPtSt));
						l_FrameInfo.u8Sum--;
					}
					else
					{
						l_FrameInfo.uValid[l_u16index] = 0;
						memset(&l_FrameInfo.IncPtdata[l_u16index], 0, sizeof(IncPtSt));
						l_FrameInfo.u8Sum--;					
					}
				}
				else if(l_FrameInfo.IncPtdata[l_u16index].u16yMaxHt<1000)
				{
					l_FrameInfo.uValid[l_u16index] = 0;
					memset(&l_FrameInfo.IncPtdata[l_u16index], 0, sizeof(IncPtSt));
					l_FrameInfo.u8Sum--;				
				}			
			}			
		}
	
		//区域合并
		for(l_u16index = 0;l_u16index < l_FrameInfo.u8Sum-1;l_u16index=l_u16index)
		{
			l_u16StartYpt = l_FrameInfo.IncPtdata[l_u16index].u16Pt1;
			l_u16EndYpt = l_FrameInfo.IncPtdata[l_u16index].u16Pt2;
			l_n32StartY = l_FrameInfo.IncPtdata[l_u16index].n32y1;
			l_n32EndY = l_FrameInfo.IncPtdata[l_u16index].n32y2;
			l_u16tmp = abs(l_FrameInfo.IncPtdata[l_u16index+1].n32y1-l_FrameInfo.IncPtdata[l_u16index+1].n32y2);
			if (abs(l_n32EndY-l_FrameInfo.IncPtdata[l_u16index+1].n32y1)<2000  
				&& abs(l_u16EndYpt-l_FrameInfo.IncPtdata[l_u16index+1].u16Pt1)<=5
				&& abs(l_n32StartY-l_n32EndY)<l_u16tmp && abs(l_n32StartY-l_n32EndY)<2000 && l_u16tmp>2*abs(l_n32StartY-l_n32EndY)
				&& (abs(l_u16StartYpt-l_u16EndYpt+1)<10 ))
			{
				l_FrameInfo.IncPtdata[l_u16index].u16Pt2 = l_FrameInfo.IncPtdata[l_u16index+1].u16Pt2;
				l_FrameInfo.IncPtdata[l_u16index].n32y2 = l_FrameInfo.IncPtdata[l_u16index+1].n32y2;
				l_FrameInfo.IncPtdata[l_u16index].u16yDis = abs(l_FrameInfo.IncPtdata[l_u16index].n32y2-l_FrameInfo.IncPtdata[l_u16index].n32y1);											
				l_32tmp = l_FrameInfo.IncPtdata[l_u16index].u16yMaxHt;	
				if(l_FrameInfo.IncPtdata[l_u16index+1].u16yMaxHt > l_32tmp)
				{
					l_FrameInfo.IncPtdata[l_u16index].u16yMaxHt = l_FrameInfo.IncPtdata[l_u16index+1].u16yMaxHt;  //取最大值为高				
				}
				for (l_index=l_u16index+1;l_index<l_FrameInfo.u8Sum-1;l_index++)
				{
					memcpy(&l_FrameInfo.IncPtdata[l_index],&l_FrameInfo.IncPtdata[l_index+1],sizeof(IncPtSt));
				}
				l_FrameInfo.uValid[l_index] = 0;
				memset(&l_FrameInfo.IncPtdata[l_index], 0, sizeof(IncPtSt));
				l_FrameInfo.u8Sum--;
			}
			else if (abs(l_n32EndY-l_FrameInfo.IncPtdata[l_u16index+1].n32y1)<2000 
				&& abs(l_u16EndYpt-l_FrameInfo.IncPtdata[l_u16index+1].u16Pt1)<=5 
				&& abs(l_n32StartY-l_n32EndY)>l_u16tmp && l_u16tmp<2000 && l_u16tmp*2<abs(l_n32StartY-l_n32EndY)
				&& abs(l_FrameInfo.IncPtdata[l_u16index+1].u16Pt1-l_FrameInfo.IncPtdata[l_u16index+1].u16Pt2+1)<10)
			{
				l_FrameInfo.IncPtdata[l_u16index].u16Pt2 = l_FrameInfo.IncPtdata[l_u16index+1].u16Pt2;
				l_FrameInfo.IncPtdata[l_u16index].n32y2 = l_FrameInfo.IncPtdata[l_u16index+1].n32y2;
				l_FrameInfo.IncPtdata[l_u16index].u16yDis = abs(l_FrameInfo.IncPtdata[l_u16index].n32y2-l_FrameInfo.IncPtdata[l_u16index].n32y1);											
				l_32tmp = l_FrameInfo.IncPtdata[l_u16index].u16yMaxHt;	
				if(l_FrameInfo.IncPtdata[l_u16index+1].u16yMaxHt > l_32tmp)
				{
					l_FrameInfo.IncPtdata[l_u16index].u16yMaxHt = l_FrameInfo.IncPtdata[l_u16index+1].u16yMaxHt;  //取最大值为高				
				}
				//l_FrameInfo.uValid[l_u16index+1] = 0;
				//memset(&l_FrameInfo.IncPtdata[l_u16index+1], 0, sizeof(IncPtSt));
				for (l_index=l_u16index+1;l_index<l_FrameInfo.u8Sum-1;l_index++)
				{
					memcpy(&l_FrameInfo.IncPtdata[l_index],&l_FrameInfo.IncPtdata[l_index+1],sizeof(IncPtSt));
				}
				l_FrameInfo.uValid[l_index] = 0;
				memset(&l_FrameInfo.IncPtdata[l_index], 0, sizeof(IncPtSt));
				l_FrameInfo.u8Sum--;		
			}
			else if (abs(l_n32EndY-l_FrameInfo.IncPtdata[l_u16index+1].n32y1)<1000
				&& abs(l_u16EndYpt-l_FrameInfo.IncPtdata[l_u16index+1].u16Pt1)<=10
				&& abs(l_n32StartY-l_n32EndY)>l_u16tmp && l_u16tmp<1000)
			{
				l_FrameInfo.IncPtdata[l_u16index].u16Pt2 = l_FrameInfo.IncPtdata[l_u16index+1].u16Pt2;
				l_FrameInfo.IncPtdata[l_u16index].n32y2 = l_FrameInfo.IncPtdata[l_u16index+1].n32y2;
				l_FrameInfo.IncPtdata[l_u16index].u16yDis = abs(l_FrameInfo.IncPtdata[l_u16index].n32y2-l_FrameInfo.IncPtdata[l_u16index].n32y1);											
				l_32tmp = l_FrameInfo.IncPtdata[l_u16index].u16yMaxHt;	
				if(l_FrameInfo.IncPtdata[l_u16index+1].u16yMaxHt > l_32tmp)
				{
					l_FrameInfo.IncPtdata[l_u16index].u16yMaxHt = l_FrameInfo.IncPtdata[l_u16index+1].u16yMaxHt;  //取最大值为高				
				}
				//l_FrameInfo.uValid[l_u16index+1] = 0;
				//memset(&l_FrameInfo.IncPtdata[l_u16index+1], 0, sizeof(IncPtSt));
				//l_FrameInfo.u8Sum--;
				for (l_index=l_u16index+1;l_index<l_FrameInfo.u8Sum-1;l_index++)
				{
					memcpy(&l_FrameInfo.IncPtdata[l_index],&l_FrameInfo.IncPtdata[l_index+1],sizeof(IncPtSt));
				}
				l_FrameInfo.uValid[l_index] = 0;
				memset(&l_FrameInfo.IncPtdata[l_index], 0, sizeof(IncPtSt));
				l_FrameInfo.u8Sum--;
			}
			else if (abs(l_n32EndY-l_FrameInfo.IncPtdata[l_u16index+1].n32y1)<1000
				&& abs(l_u16EndYpt-l_FrameInfo.IncPtdata[l_u16index+1].u16Pt1)<=10
				&& abs(g_ZdistanceI[l_u16EndYpt]-g_ZdistanceI[l_FrameInfo.IncPtdata[l_u16index+1].u16Pt1])<1000
				&& l_FrameInfo.IncPtdata[l_u16index].u16yMaxHt>1200 && l_FrameInfo.IncPtdata[l_u16index+1].u16yMaxHt>1200)
			{
				l_FrameInfo.IncPtdata[l_u16index].u16Pt2 = l_FrameInfo.IncPtdata[l_u16index+1].u16Pt2;
				l_FrameInfo.IncPtdata[l_u16index].n32y2 = l_FrameInfo.IncPtdata[l_u16index+1].n32y2;
				l_FrameInfo.IncPtdata[l_u16index].u16yDis = abs(l_FrameInfo.IncPtdata[l_u16index].n32y2-l_FrameInfo.IncPtdata[l_u16index].n32y1);											
				l_32tmp = l_FrameInfo.IncPtdata[l_u16index].u16yMaxHt;	
				if(l_FrameInfo.IncPtdata[l_u16index+1].u16yMaxHt > l_32tmp)
				{
					l_FrameInfo.IncPtdata[l_u16index].u16yMaxHt = l_FrameInfo.IncPtdata[l_u16index+1].u16yMaxHt;  //取最大值为高				
				}
				//l_FrameInfo.uValid[l_u16index+1] = 0;
				//memset(&l_FrameInfo.IncPtdata[l_u16index+1], 0, sizeof(IncPtSt));
				//l_FrameInfo.u8Sum--;
				for (l_index=l_u16index+1;l_index<l_FrameInfo.u8Sum-1;l_index++)
				{
					memcpy(&l_FrameInfo.IncPtdata[l_index],&l_FrameInfo.IncPtdata[l_index+1],sizeof(IncPtSt));
				}
				l_FrameInfo.uValid[l_index] = 0;
				memset(&l_FrameInfo.IncPtdata[l_index], 0, sizeof(IncPtSt));
				l_FrameInfo.u8Sum--;
			}
			else
			{
				l_u16index++;
			}
		}
	
		/*******标记每一辆车中车头、车尾是否打飞*********/
		for(l_index = 0;l_index < l_FrameInfo.u8Sum;l_index++)
		{
			l_FrameInfo.IncPtdata[l_index].u8DaFeiFlag1 = 0;
			l_FrameInfo.IncPtdata[l_index].u8DaFeiFlag2 = 0;
			for(i=1;i<=DafeiData[0][0]+DafeiData[0][1];i++)//20140919
			{
				if((DafeiData[i][0]<=l_FrameInfo.IncPtdata[l_index].u16Pt2)&&(DafeiData[i][1]>=l_FrameInfo.IncPtdata[l_index].u16Pt2)) //检查打飞段的终止点是否与车的起始点相同，若相同则车头打飞
				{
					l_FrameInfo.IncPtdata[l_index].u8DaFeiFlag1 = 1;
				}
				if((DafeiData[i][0]<=l_FrameInfo.IncPtdata[l_index].u16Pt1)&&(DafeiData[i][1]>=l_FrameInfo.IncPtdata[l_index].u16Pt1))		   //检查打飞段的起始点是否与车的结尾点相同，若相同则车尾打飞
				{
					l_FrameInfo.IncPtdata[l_index].u8DaFeiFlag2 = 1;
				}
			}	
		}
		/*******************************/
		/*JG_T0TC[2] = T0TC;
		/*JG_counter2[2]=t0_count2;
		/*******************************/	
		//计算车长、车高、车头时间、车头车尾X坐标、车头点车尾点
		for(l_index = 0;l_index < l_FrameInfo.u8Sum;l_index++)
		{
			l_u16StartYpt = l_FrameInfo.IncPtdata[l_index].u16Pt1;
			l_u16EndYpt = l_FrameInfo.IncPtdata[l_index].u16Pt2;
			if(((abs(l_FrameInfo.IncPtdata[l_index].n32y2- l_FrameInfo.IncPtdata[l_index].n32y1)>500) && (l_u16EndYpt - l_u16StartYpt + 1)>=5 && l_FrameInfo.IncPtdata[l_index].n32y2 >=EnterX1 && l_FrameInfo.IncPtdata[l_index].n32y1<=ExitX4+2000)
				|| (l_FrameInfo.IncPtdata[l_index].n32y1>=ExitX4 && l_FrameInfo.IncPtdata[l_index].n32y2>=EnterX1 && (l_u16EndYpt - l_u16StartYpt + 1)>=3 && l_FrameInfo.IncPtdata[l_index].u16yMaxHt>1000 && l_FrameInfo.IncPtdata[l_index].n32y1<=ExitX4+2000))//20140904
			{
				Tmp_Z=0;
				Tmp_Z = GetVehHeight2(g_ZdistanceI, l_u16StartYpt, l_u16EndYpt);
				l_FrameInfo.IncPtdata[l_index].u16yMaxHt = Tmp_Z;	    //车高
				
				//*用车头最前面的点作为车头位置*/	
				MaxX=l_FrameInfo.IncPtdata[l_index].n32y2;
				Maxi=l_FrameInfo.IncPtdata[l_index].u16Pt2;
				for(i=l_u16StartYpt;i<=l_u16EndYpt;i++)
				{
					if(g_YdistanceI[i]>MaxX && g_ZdistanceI[i]>=ThresVehLow)	//20140327 Z要大于阈值
					{
						MaxX=g_YdistanceI[i];
						Maxi=i;
					}
				}
				l_FrameInfo.IncPtdata[l_index].n32y2 = MaxX;
				l_FrameInfo.IncPtdata[l_index].u16Pt2 = Maxi;
				//*寻找车头最前点结束*/
	
	
				//*用车尾最前面的点作为车尾位置*/	
				MinX=l_FrameInfo.IncPtdata[l_index].n32y1;
				Maxi=l_FrameInfo.IncPtdata[l_index].u16Pt1;
				for(i=l_u16StartYpt;i<=l_u16EndYpt;i++)//20140916
				{
					if(g_YdistanceI[i]<MinX && g_ZdistanceI[i]>=ThresVehLow)
					{
						MinX=g_YdistanceI[i];
						Maxi=i;
					}
				}
				l_FrameInfo.IncPtdata[l_index].n32y1 = MinX;
				l_FrameInfo.IncPtdata[l_index].u16Pt1 = Maxi;
				//*寻找车尾最前点结束*/		
		
				l_FrameInfo.IncPtdata[l_index].u16yDis=l_FrameInfo.IncPtdata[l_index].n32y2-l_FrameInfo.IncPtdata[l_index].n32y1;  //车长
			
				/*车头时间修正在后面计算*/								
			}
			else 
			{
				l_FrameInfo.uValid[l_index] = 0;		   //剔除未进入检测区域的车
				memset(&l_FrameInfo.IncPtdata, 0, sizeof(IncPtSt));
				for (i=l_index; i<l_FrameInfo.u8Sum-1; i++)
				{
					memcpy(&l_FrameInfo.IncPtdata[i], &l_FrameInfo.IncPtdata[i+1], sizeof(l_FrameInfo.IncPtdata[i+1]));
				}
				l_FrameInfo.u8Sum = l_FrameInfo.u8Sum - 1;
			}				 	
		}		
		 														  
		//车辆匹配
		for(l_u16index = 0; l_u16index < l_FrameInfo.u8Sum; l_u16index++)
		{
			l_32tmp = 0;  //匹配标识，成功置1。
			l_n32StartY = l_FrameInfo.IncPtdata[l_u16index].n32y1;
			l_n32EndY = l_FrameInfo.IncPtdata[l_u16index].n32y2;
			l_u16StartYpt = l_FrameInfo.IncPtdata[l_u16index].u16Pt1;
			l_u16EndYpt = l_FrameInfo.IncPtdata[l_u16index].u16Pt2;
			if (l_u16EndYpt <= l_u16StartYpt ||	l_n32EndY <= l_n32StartY)
			{
				continue;
			}
			if (l_n32EndY>=EnterX1 && l_n32StartY<=0)			//车头过了EnterX1&&车尾未出正下方
			{
				for(j = 0;j < g_VehIncTotal;j++)//与已有的顺扫激光器中存储的车辆数据进行匹配
				{
					i = g_VehIncSetIndex[j] -1;		
					if(g_VehIncSet[i].u8Istate == OCCURING_USED)
					{
						l_32tmp2 = g_VehIncSet[i].Idata.u16FrameCnt &  FRAME_MASK;
						if (IsInIncSide(l_n32StartY, l_n32EndY, g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][1],g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][0])
							|| (l_n32EndY<0 && abs(l_n32EndY-g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][0])<1500 && abs(l_u16StartYpt-g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][3])<=10 && g_ZdistanceI[l_u16StartYpt]+1000<l_FrameInfo.IncPtdata[l_u16index].u16yMaxHt)
							|| (l_n32EndY>0 && abs(l_n32StartY-g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][1])<1500 && g_ZdistanceI[l_u16EndYpt]+1000<l_FrameInfo.IncPtdata[l_u16index].u16yMaxHt))
						{
							 l_32tmp = 1;  //区域匹配成功
							 if ((l_n32StartY>=0 && abs(l_n32StartY-g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][1])>50)//车尾
							 	|| (l_n32EndY<=0 && abs(l_n32EndY-g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][0])>50)
								|| (l_n32StartY<0 && abs(l_n32StartY-g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][1])>50 && l_n32EndY>ExitX4))	  // && abs(l_n32EndY-g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][0])>50
							 {
							 	g_VehIncSet[i].u8Istate = OCCURING_USED;
								memcpy(&g_VehIncSet[i].Idata.zdata[l_32tmp2][1], g_ZdistanceI + l_u16StartYpt, sizeof(int32)*(l_u16EndYpt - l_u16StartYpt + 1));
								g_VehIncSet[i].Idata.zdata[l_32tmp2][0] = l_u16EndYpt - l_u16StartYpt + 1;
								memcpy(&g_VehIncSet[i].Idata.ydata[l_32tmp2][1], g_YdistanceI + l_u16StartYpt, sizeof(int32)*(l_u16EndYpt - l_u16StartYpt + 1));
								g_VehIncSet[i].Idata.ydata[l_32tmp2][0] = l_u16EndYpt - l_u16StartYpt + 1;
								g_VehIncSet[i].Idata.yMax[l_32tmp2] = abs(l_n32EndY - l_n32StartY);  								
								g_VehIncSet[i].Idata.zMax[l_32tmp2] = l_FrameInfo.IncPtdata[l_u16index].u16yMaxHt;
								g_VehIncSet[i].Idata.ydataInfo[l_32tmp2][0] = l_FrameInfo.IncPtdata[l_u16index].n32y2;
								g_VehIncSet[i].Idata.ydataInfo[l_32tmp2][1] = l_FrameInfo.IncPtdata[l_u16index].n32y1;
								g_VehIncSet[i].Idata.ydataInfo[l_32tmp2][2] = l_FrameInfo.IncPtdata[l_u16index].u16Pt2;
								g_VehIncSet[i].Idata.ydataInfo[l_32tmp2][3] = l_FrameInfo.IncPtdata[l_u16index].u16Pt1;
								g_VehIncSet[i].Idata.ydataInfo[l_32tmp2][4]	= l_FrameInfo.IncPtdata[l_u16index].u8DaFeiFlag1;
								g_VehIncSet[i].Idata.ydataInfo[l_32tmp2][5] = l_FrameInfo.IncPtdata[l_u16index].u8DaFeiFlag2; 							 																				
								g_VehIncSet[i].Idata.tdata[l_32tmp2] = Time_Vertical 
									+ (g_VehIncSet[i].Idata.ydataInfo[l_32tmp2][2]+g_u16VerticalStartAnglePt2)*10000/360;
								if (g_VehIncSet[i].Idata.u16FrameCnt < FRAME_MASK )
								{
									g_VehIncSet[i].Idata.u16FrameCnt++;	
								}
								else
								{	
									g_VehIncSet[i].Idata.u16FrameCnt = FRAME_MASK;
								}						 						 
							}
							g_VehIncSet[i].IemptFrame = 0;
							if ((g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][0]>0)
								&& (g_VehIncSet[i].Idata.zMax[l_32tmp2-1]>2000) 
								&& (abs(g_VehIncSet[i].Idata.zMax[l_32tmp2]-g_VehIncSet[i].Idata.zMax[l_32tmp2-1])<1000)
								&& (abs(g_VehIncSet[i].Idata.ydataInfo[l_32tmp2][0]-g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][1])>6000)
								&& (abs(g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][0]-g_VehIncSet[i].Idata.ydataInfo[l_32tmp2][0])>1500)
								&& (abs(g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][0]-g_VehIncSet[i].Idata.ydataInfo[l_32tmp2][0])>abs(g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][1]-g_VehIncSet[i].Idata.ydataInfo[l_32tmp2][1])+1500))
							{
								g_VehIncSet[i].Idata.ydataInfo[l_32tmp2][0] = g_VehIncSet[i].Idata.ydataInfo[l_32tmp2][1] + (g_VehIncSet[i].Idata.yMax[l_32tmp2-1]+g_VehIncSet[i].Idata.yMax[l_32tmp2-2])/2;
								g_VehIncSet[i].Idata.yMax[l_32tmp2] = abs(g_VehIncSet[i].Idata.ydataInfo[l_32tmp2][0]-g_VehIncSet[i].Idata.ydataInfo[l_32tmp2][1]);
							}
						}
						else if (l_32tmp2>2 && IsInIncSide(l_n32StartY, l_n32EndY, g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-2][1],g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-2][0]))
						{
							l_32tmp = 1;  //区域匹配成功
							if ((l_n32StartY>=0 && abs(l_n32StartY-g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-2][1])>50)//车尾
								|| (l_n32EndY<=0 && abs(l_n32EndY-g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-2][0])>50)
								|| (l_n32StartY<0 && abs(l_n32StartY-g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-2][1])>50 && l_n32EndY>ExitX4))	  // && abs(l_n32EndY-g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][0])>50
							{
								g_VehIncSet[i].u8Istate = OCCURING_USED;
								memcpy(&g_VehIncSet[i].Idata.zdata[l_32tmp2][1], g_ZdistanceI + l_u16StartYpt, sizeof(int32)*(l_u16EndYpt - l_u16StartYpt + 1));
								g_VehIncSet[i].Idata.zdata[l_32tmp2][0] = l_u16EndYpt - l_u16StartYpt + 1;
								memcpy(&g_VehIncSet[i].Idata.ydata[l_32tmp2][1], g_YdistanceI + l_u16StartYpt, sizeof(int32)*(l_u16EndYpt - l_u16StartYpt + 1));
								g_VehIncSet[i].Idata.ydata[l_32tmp2][0] = l_u16EndYpt - l_u16StartYpt + 1;
								g_VehIncSet[i].Idata.yMax[l_32tmp2] = abs(l_n32EndY - l_n32StartY);  								
								g_VehIncSet[i].Idata.zMax[l_32tmp2] = l_FrameInfo.IncPtdata[l_u16index].u16yMaxHt;
								g_VehIncSet[i].Idata.ydataInfo[l_32tmp2][0] = l_FrameInfo.IncPtdata[l_u16index].n32y2;
								g_VehIncSet[i].Idata.ydataInfo[l_32tmp2][1] = l_FrameInfo.IncPtdata[l_u16index].n32y1;
								g_VehIncSet[i].Idata.ydataInfo[l_32tmp2][2] = l_FrameInfo.IncPtdata[l_u16index].u16Pt2;
								g_VehIncSet[i].Idata.ydataInfo[l_32tmp2][3] = l_FrameInfo.IncPtdata[l_u16index].u16Pt1;
								g_VehIncSet[i].Idata.ydataInfo[l_32tmp2][4]	= l_FrameInfo.IncPtdata[l_u16index].u8DaFeiFlag1;
								g_VehIncSet[i].Idata.ydataInfo[l_32tmp2][5] = l_FrameInfo.IncPtdata[l_u16index].u8DaFeiFlag2; 							 																				
								g_VehIncSet[i].Idata.tdata[l_32tmp2] = Time_Vertical 
									+ (g_VehIncSet[i].Idata.ydataInfo[l_32tmp2][2]+g_u16VerticalStartAnglePt2)*10000/360;
								if (g_VehIncSet[i].Idata.u16FrameCnt < FRAME_MASK )
								{
									g_VehIncSet[i].Idata.u16FrameCnt++;	
								}
								else
								{	
									g_VehIncSet[i].Idata.u16FrameCnt = FRAME_MASK;
								}						 						 
							}
							g_VehIncSet[i].IemptFrame = 0;
							if (g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][1]<g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-2][1]-2500 && l_n32StartY-g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-2][1]-2500)//对异常存储的帧进行修正20140914
							{
								g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][1] = (g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-2][1]+l_n32StartY)/2;
								g_VehIncSet[i].Idata.yMax[l_32tmp2-1] = abs(g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-2][0] - g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-2][1]);
							}
						}
						break;
					}										
				}
				if (0 == l_32tmp && l_FrameInfo.IncPtdata[l_u16index].n32y1<0)	 //新车进入
				{				
					l_32tmp = 1;				
					for(l_u16tmp = 0;l_u16tmp<l_32tmp;l_u16tmp++)
					{
						for(i = 0;i < VEHICLE_MAX;i++)
						{
							if(g_VehIncSet[i].u8Istate == NO_USED)
							{
								g_VehIncSetIndex[g_VehIncTotal++] = i+1;
								g_VehIncSet[i].u8Istate = OCCURING_USED;	
								l_u16StartYpt = l_FrameInfo.IncPtdata[l_u16index].u16Pt1;
								l_u16EndYpt = l_FrameInfo.IncPtdata[l_u16index].u16Pt2;	
								l_n32StartY = l_FrameInfo.IncPtdata[l_u16index].n32y1;
								l_n32EndY = l_FrameInfo.IncPtdata[l_u16index].n32y2;	
								l_32tmp2 =g_VehIncSet[i].Idata.u16FrameCnt &  FRAME_MASK;
								if(g_VehIncSet[i].Idata.u16FrameCnt < FRAME_MAXCNT && l_u16EndYpt>l_u16StartYpt)
								{
									memcpy(&g_VehIncSet[i].Idata.zdata[l_32tmp2][1],g_ZdistanceI + l_u16StartYpt,sizeof(int32)*(l_u16EndYpt - l_u16StartYpt + 1));
									g_VehIncSet[i].Idata.zdata[l_32tmp2][0] = l_u16EndYpt - l_u16StartYpt + 1;
									memcpy(&g_VehIncSet[i].Idata.ydata[l_32tmp2][1], g_YdistanceI + l_u16StartYpt,sizeof(int32)*(l_u16EndYpt - l_u16StartYpt + 1));
									g_VehIncSet[i].Idata.ydata[l_32tmp2][0] = l_u16EndYpt - l_u16StartYpt + 1;
									g_VehIncSet[i].Idata.yMax[l_32tmp2] = abs(l_n32EndY - l_n32StartY);  								
									g_VehIncSet[i].Idata.zMax[l_32tmp2] = l_FrameInfo.IncPtdata[l_u16index].u16yMaxHt;
									g_VehIncSet[i].Idata.ydataInfo[l_32tmp2][0] = l_FrameInfo.IncPtdata[l_u16index].n32y2;
									g_VehIncSet[i].Idata.ydataInfo[l_32tmp2][1] = l_FrameInfo.IncPtdata[l_u16index].n32y1;
									g_VehIncSet[i].Idata.ydataInfo[l_32tmp2][2] = l_FrameInfo.IncPtdata[l_u16index].u16Pt2;
									g_VehIncSet[i].Idata.ydataInfo[l_32tmp2][3] = l_FrameInfo.IncPtdata[l_u16index].u16Pt1;
									g_VehIncSet[i].Idata.ydataInfo[l_32tmp2][4]	= l_FrameInfo.IncPtdata[l_u16index].u8DaFeiFlag1;
									g_VehIncSet[i].Idata.ydataInfo[l_32tmp2][5] = l_FrameInfo.IncPtdata[l_u16index].u8DaFeiFlag2; 							 																				
								}
								g_VehIncSet[i].Idata.tdata[l_32tmp2] = Time_Vertical + (g_VehIncSet[i].Idata.ydataInfo[l_32tmp2][2]+g_u16VerticalStartAnglePt2)*10000/360;
								if (g_VehIncSet[i].Idata.u16FrameCnt < FRAME_MASK )
								{
									g_VehIncSet[i].Idata.u16FrameCnt++;	
								}
								else
								{	
									g_VehIncSet[i].Idata.u16FrameCnt = FRAME_MASK;
								}	
								g_VehIncSet[i].IemptFrame = 0;	
								break;
							}
						}							
					}			
				}		
			}					
		}
	
		//抛车计算车速及单帧车长
		for(j = 0;j < g_VehIncTotal;j++)
		{
			i = (g_VehIncSetIndex[j] - 1) & VEHICLE_MASK;
			if (g_VehIncSet[i].u8Istate != NO_USED)
			{
				l_32tmp2 = g_VehIncSet[i].Idata.u16FrameCnt & FRAME_MASK;
				if( l_32tmp2 >0
					&& (g_VehIncSet[i].IemptFrame > NORMAL_MAX_EMPTYFRAME 
					&&(g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][1]>ExitX4)))
				{ 		
				   g_VehIncSet[i].u8Istate = PASSED_USED;  //已结束，可收尾的车
				}
				else if (l_32tmp2 >0 && (0 == g_VehIncSet[i].u8ThrowFlag) 
					&& g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][0]>ExitX3)   //车头过线
				{
				   g_VehIncSet[i].speed = GetVehSpeed(&g_VehIncSet[i], EnterX2, EnterX1);	//车头速度
				   g_VehIncSet[i].u8ThrowFlag = 1;
				}
				else if (l_32tmp2 >0 && g_VehIncSet[i].IemptFrame > ERR_MAX_EMPTYFRAME)
				{
					memset(&g_VehIncSet[i], 0,sizeof(VehIncSt)); //清除顺道激光器该记录
					g_VehIncSet[i].u8Istate = NO_USED; 
					for(l_u16tmp = j;l_u16tmp < g_VehIncTotal - 1;l_u16tmp++)
						g_VehIncSetIndex[l_u16tmp] = g_VehIncSetIndex[l_u16tmp+1];
					if (g_VehIncTotal <= VEHICLE_MAX)  //20140217 修改
					{
						g_VehIncSetIndex[g_VehIncTotal - 1] = 0;
						g_VehIncTotal--;
					}
					else
					{
						g_VehIncTotal = 0;
					}
					continue;
				}
				if (l_32tmp2==1 && g_VehIncSet[i].IemptFrame>10)//20140918
				{
					memset(&g_VehIncSet[i], 0,sizeof(VehIncSt)); //清除顺道激光器该记录
					g_VehIncSet[i].u8Istate = NO_USED; 
					for(l_u16tmp = j;l_u16tmp < g_VehIncTotal - 1;l_u16tmp++)
						g_VehIncSetIndex[l_u16tmp] = g_VehIncSetIndex[l_u16tmp+1];
					if (g_VehIncTotal <= VEHICLE_MAX)  
					{
						g_VehIncSetIndex[g_VehIncTotal - 1] = 0;
						g_VehIncTotal--;
					}
					else
					{
						g_VehIncTotal = 0;
					}
					continue;
				}		
				g_VehIncSet[i].IemptFrame++;		
			}					
		}
		//车辆长度计算
		/**********车头过了Lengthline线后计时开始************/
		for (j = 0;j < g_VehIncTotal;j++)
		{
			i = (g_VehIncSetIndex[j] - 1) & VEHICLE_MASK;
			if (g_VehIncSet[i].u8Istate != NO_USED)
			{
			   l_32tmp2 = g_VehIncSet[i].Idata.u16FrameCnt &  FRAME_MASK;
			   if((0 == g_VehIncSet[i].u8LineFlag1)  
					&& g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][0]>Lengthline1 
					&& (g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][1]<Lengthline1)
					&& g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][0]<Lengthline2)
			    {
					g_VehIncSet[i].nStartTime =	g_VehIncSet[i].Idata.tdata[l_32tmp2-1];      //开始时间为车头时间，已经修正过的时间
					g_VehIncSet[i].ndeltaY= g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][0];	
					g_VehIncSet[i].nEndTime = g_VehIncSet[i].nStartTime;
					g_VehIncSet[i].u8LineFlag1 = 1; 
				}
				else if (1 == g_VehIncSet[i].u8LineFlag1)
				{
					if (0 == g_VehIncSet[i].u8LineFlag2)
					{
						if(g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][0]>=Lengthline2 
							&& g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][1]>=Lengthline2)
						{
							g_VehIncSet[i].u8LineFlag2 = 1;
						}				
					}
					else if (1 == g_VehIncSet[i].u8LineFlag2)
					{						
						g_VehIncSet[i].nEndTime = g_VehIncSet[i].Idata.tdata[l_32tmp2-1] 
							- (g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][2]-g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][3])*10000/360; //结束时间
						g_VehIncSet[i].yLen= g_VehIncSet[i].speed*(g_VehIncSet[i].nEndTime-g_VehIncSet[i].nStartTime)/36000 
							- abs(g_VehIncSet[i].ndeltaY-g_VehIncSet[i].Idata.ydataInfo[l_32tmp2-1][1]);  //车辆长度
						g_VehIncSet[i].ndeltaY = 0;
						g_VehIncSet[i].nStartTime = 0;
						g_VehIncSet[i].nEndTime = 0;
						g_VehIncSet[i].u8LineFlag1 = 2;
						g_VehIncSet[i].u8LineFlag2 = 2;									
					}			
				}		
			}		
		}
        /****************垂直激光器与顺车道激光器匹配不放在此处进行*********************************/		
		/********************垂直激光与顺道激光匹配处理**********************
		/*第一步：清除垂直激光器两车道非匹配区域记录 */
		/*********************************************************************
	    /***垂直激光器已收尾车辆，locatex位置在非匹配区域，直接清除
		/***********************************************************************/
		/*
		for(j = 0;j < g_totalVehicle;j++)
		{
			i = g_VehicleSetIndex[j]-1;
			l_leftX = g_VehicleSet[i].locateX.n32xLeft;
			l_rightX = g_VehicleSet[i].locateX.n32xRight;
			if(g_VehicleSet[i].u8Vstate==PASSED_USED 
				&& (((l_leftX <-1*(g_sspSetup.u8LaneNum-1)*g_LaneWide/2) && ((l_leftX + l_rightX)/2<-1*(g_sspSetup.u8LaneNum-2)*g_LaneWide/2-200)) || l_rightX>0))
			{
				memset(&g_VehicleSet[i],0,sizeof(VehicleStruct));
				g_VehicleSet[i].u8Vstate = NO_USED;  
	
				for(l_u16tmp = j;l_u16tmp < g_totalVehicle - 1;l_u16tmp++)
					g_VehicleSetIndex[l_u16tmp] = g_VehicleSetIndex[l_u16tmp+1];
	
				if (g_totalVehicle <= VEHICLE_MAX)  //20140217 修改
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
		/*第二步：开始匹配*/
		/*****************************************************************************/
		/*匹配准则：长、高接近，且位置在匹配区域；匹配成功出车后双清
		/*以垂直激光器为基准，顺车道与垂直激光器匹配：
		/*①顺车道记录集中无车与垂直激光器匹配，清除垂直激光器该车信息或估计一个车速出车
		/*②顺车道中有车与垂直激光器匹配，则匹配成功出车双清
		/*****************************************************************************/
	  } 
	}
}
