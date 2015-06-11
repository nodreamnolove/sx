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
//u8StarttEndFlag 0��ʾ��ʼ�㣬1��ʾ�����㣻u8JGIndx 0 1 2 3�ֱ��ʾ��·����
uint16 GetStartEndPt(const int* const pdata, const uint16 startPt, const uint8 u8StartEndFlag, const uint8 u8JGIndx)  
{
	uint16 Ret = startPt;
	uint16 index = 0;
	uint16 minHeight = 4000;   //��Ѱ����ʼ�㡢������ʱ����С�߶�
	uint16 u16TmpZeroPos = 180;
	uint16 u16StartPtNum=0;
	uint16 u16EndPtNum=0;

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

	if (g_sspSetup.u8InstallFlag)  //��װ��ʽ
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
				Ret = g_sspSetup.u16StartPtNum;
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
				Ret = g_sspSetup.u16EndPtNum;
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
//ÿ֡���г��������Ч�����жϣ�0��ʾ��Ч��1��ʾ��Ч  (��Ҫ�Ľ���
uint8 ISVehRegion(const uint16 u16RegionWide, const PointStruct *pPtStruct, const int32* pZdistance)
{
	uint8 Ret = 0;
	uint16 l_u16Index = 0;
	uint16 l_u16HightPtNum = 0;
	uint16 l_u16LowPtNum   = 0;
	//20140217 ���ӶԲ����Ϸ����ж�
	if (pPtStruct == NULL || pZdistance == NULL)
	{
		return 0;
	}

	//���� �����������ڸ߶��쳣���
	if (pPtStruct->u16Leftpt >= pPtStruct->u16Rightpt || pPtStruct->u16Rightpt >= POINT_SUM )
	{	//�����쳣
		return 0;
	}
	for (l_u16Index = pPtStruct->u16Leftpt; l_u16Index <= pPtStruct->u16Rightpt; l_u16Index++)
	{
		if (pZdistance[l_u16Index] >= 5000) //�߶ȳ���5m�ĵ�
		{
			l_u16HightPtNum++;	
		}
		else if (pZdistance[l_u16Index] <= -500)	//�߶���-0.5m����Ϊ�쳣
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
		//�쳣�϶�
		return 0;
	}
	
	if (pPtStruct->u16xMaxHt > 3000 &&pZdistance[pPtStruct->u16Leftpt] == pPtStruct->u16xMaxHt) //��ߵ������߶�ֵ
	{
		if ((g_sspSetup.u8InstallFlag &&(pPtStruct->n32xLeft-pPtStruct->n32xRight>2000)) ||
			((g_sspSetup.u16EndPtNum < g_sspSetup.u16VerticalZeroPos)&&
			(pPtStruct->n32xRight-pPtStruct->n32xLeft>2000)))
		{//	���ֵ�쳣
			return 0;	
		}	
	}
	else if (pPtStruct->u16xMaxHt > 3000 && pZdistance[pPtStruct->u16Rightpt] == pPtStruct->u16xMaxHt)//�ұߵ������߶�ֵ
	{
		if ((g_sspSetup.u8InstallFlag &&(pPtStruct->n32xLeft-pPtStruct->n32xRight>2000)) ||
			((g_sspSetup.u16EndPtNum < g_sspSetup.u16VerticalZeroPos)&&
			(pPtStruct->n32xLeft-pPtStruct->n32xRight>2000)))
		{//	�ҵ�ֵ�쳣
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
