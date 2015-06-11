#include "MathFunction.h"
#include "JZStructure.h"
#include "JZGlobal.h"
#define   ERRORVALUE       0xFFFF
uint8 IsInIncSide(int32 x11, int32 x12, int32 x21, int32 x22)
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
//u8StarttEndFlag 0表示起始点，1表示结束点；u8JGIndx 0 1 2 3分别表示四路激光
uint16 GetStartEndPt(const int* const pdata, const uint16 startPt, const uint8 u8StartEndFlag, const uint8 u8JGIndx)  
{
	uint16 Ret = startPt;
	uint16 index = 0;
	uint16 minHeight = 4000;   //在寻找起始点、结束点时的最小高度
	uint16 u16TmpZeroPos = 180;
	uint16 u16StartPtNum=0;
	uint16 u16EndPtNum=0;

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
	}
	else if(u8JGIndx==1)
	{
	 	u16TmpZeroPos = g_sspSetup.u16VerticalZeroPos1;
		u16StartPtNum= g_sspSetup.u16StartPtNum1;
		u16EndPtNum=g_sspSetup.u16EndPtNum1;
	}
	else if(u8JGIndx==2)
	{
	 	u16TmpZeroPos = g_sspSetup.u16VerticalZeroPos2;
		u16StartPtNum= g_sspSetup.u16StartPtNum2;
		u16EndPtNum=g_sspSetup.u16EndPtNum2;
	}
	else if(u8JGIndx==3)
	{
	 	u16TmpZeroPos = g_sspSetup.u16VerticalZeroPos3;
		u16StartPtNum= g_sspSetup.u16StartPtNum3;
		u16EndPtNum=g_sspSetup.u16EndPtNum3;
	}

	if (u16TmpZeroPos >= POINT_SUM)
		return ERRORVALUE;

	if (g_sspSetup.u8InstallFlag)  //正装方式
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
				Ret = g_sspSetup.u16StartPtNum;
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
				Ret = g_sspSetup.u16EndPtNum;
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
//每帧中有车区域的有效区域判断，0表示无效，1表示有效  (需要改进）
uint8 ISVehRegion(const uint16 u16RegionWide, const PointStruct *pPtStruct, const int32* pZdistance)
{
	uint8 Ret = 0;
	uint16 l_u16Index = 0;
	uint16 l_u16HightPtNum = 0;
	uint16 l_u16LowPtNum   = 0;
	//20140217 增加对参数合法性判断
	if (pPtStruct == NULL || pZdistance == NULL)
	{
		return 0;
	}

	//增加 检查这个区间内高度异常情况
	if (pPtStruct->u16Leftpt >= pPtStruct->u16Rightpt || pPtStruct->u16Rightpt >= POINT_SUM )
	{	//数据异常
		return 0;
	}
	for (l_u16Index = pPtStruct->u16Leftpt; l_u16Index <= pPtStruct->u16Rightpt; l_u16Index++)
	{
		if (pZdistance[l_u16Index] >= 5000) //高度超过5m的点
		{
			l_u16HightPtNum++;	
		}
		else if (pZdistance[l_u16Index] <= -500)	//高度在-0.5m，认为异常
		{
			l_u16LowPtNum++;
		}
		else
		{
		}
	}
	if ((l_u16HightPtNum + l_u16LowPtNum) > 5 ||
		(l_u16HightPtNum + l_u16LowPtNum) >= (pPtStruct->u16Rightpt - pPtStruct->u16Leftpt+1)/2)
	{
		//异常较多
		return 0;
	}
	
	if (pPtStruct->u16xMaxHt > 3000 &&pZdistance[pPtStruct->u16Leftpt] == pPtStruct->u16xMaxHt) //左边点是最大高度值
	{
		if ((g_sspSetup.u8InstallFlag &&(pPtStruct->n32xLeft-pPtStruct->n32xRight>2000)) ||
			((g_sspSetup.u16EndPtNum < g_sspSetup.u16VerticalZeroPos)&&
			(pPtStruct->n32xRight-pPtStruct->n32xLeft>2000)))
		{//	左点值异常
			return 0;	
		}	
	}
	else if (pPtStruct->u16xMaxHt > 3000 && pZdistance[pPtStruct->u16Rightpt] == pPtStruct->u16xMaxHt)//右边点是最大高度值
	{
		if ((g_sspSetup.u8InstallFlag &&(pPtStruct->n32xLeft-pPtStruct->n32xRight>2000)) ||
			((g_sspSetup.u16EndPtNum < g_sspSetup.u16VerticalZeroPos)&&
			(pPtStruct->n32xLeft-pPtStruct->n32xRight>2000)))
		{//	右点值异常
			return 0;	
		}
	}

	if ((u16RegionWide > SMALL_AREA &&((pPtStruct->u16Rightpt - pPtStruct->u16Leftpt+1 >= MIN_PTNUM) && u16RegionWide > 0))
	    ||(pPtStruct->u16Rightpt - pPtStruct->u16Leftpt + 1 >= MIN_PTNUM*2 && u16RegionWide > 0))
	{
		Ret = 1;
	}	

	return Ret;

}
