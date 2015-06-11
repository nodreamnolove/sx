#include "sort.h"
#define MAX_REGION_NUM 10
#define MAX_POINT_DIS  15
static int32 c[320];
static int32 d[320];
extern 	int32 g_ZV[500];
extern	int32 g_XV[500];
//��С����˳��
void InsertSort(int32 a[], int32 b[],int32 n)  
{  
	int32 i,j,x,y;
//	int32 * c = (int32*) malloc(n*sizeof(int32));
//	int32 * d = (int32*) malloc(n*sizeof(int32));

	memcpy(c,a,n*sizeof(int32));
	memcpy(d,b,n*sizeof(int32));
    for( i= 1; i<n; i++){  
        if(c[i] > c[i-1]){              
             j= i-1;   
             x = c[i]; 
			 y = d[i];       
            c[i] = c[i-1];         
            while(x > c[j]){   
                c[j+1] = c[j];
				d[j+1] = d[j];  
                j--;    
				if(j==-1)
				  break;      
            }  
            c[j+1] = x;
			d[j+1] = y;      
        }
    }
	memcpy(a,c,n*sizeof(int32));
	memcpy(b,d,n*sizeof(int32));
//	free(b);  
}  
//a b n laser
void  Delege0SanDian(int32 a[], int32 b[],int32 n)
{
     int16 i;
	 int32 lastx1 = 0;
	 int32 lastx2 = 0;
	 for(i=0;i<n;i++)
	 {
		if(b[i]>0&&lastx1==0)
		{
			lastx1 = b[i];
		}
		else if(b[i]>0&&lastx1)
		{
			lastx2= b[i];
			if(abs(a[i]-a[i-1]) >200)
			{
				b[i-1] = 0;
			}
		}
		else
		{
			lastx1 = 0;
	        lastx2 = 0;
		}	 	
	 }
}
void  Delege1SanDian(int32 a[], int32 b[],int32 n)
{
  int16 i;
	 int32 lastx1 = 0;
	 int32 lastx2 = 0;
	 for(i=n-1;i>=0;i--)
	 {
		if(b[i]>0&&lastx1==0)
		{
			lastx1 = b[i];
		}
		else if(b[i]>0&&lastx1)
		{
			lastx2= b[i];  
			if(abs(a[i]-a[i+1]) >200)
			{
				b[i+1] = 0;
			}
		}
		else
		{
			lastx1 = 0;
	        lastx2 = 0;
		}	 	
	 }
}
int deleteRota(uint32 *a,uint32 pos,uint8 flag)
{
	int i,j;
	int num=0;
	i=1;
	j=pos;
	
	while(i<5)
	{
	   if(flag ==0){
			if(abs(a[j]-a[j+i])<100)
			{
				i++;
			
			}
			else
			{
			    i=0;
				j++;
			
			}
		}else
		{
		   	if(abs(a[j]-a[j-i])<100)
			{
				i++;
			
			}
			else
			{
			    i=0;
				j--;
				
			}
		}
		num++;
	   if(num>10) break;	 
	}
	return j;	
}


void Interweave_Denoise(int32 p_n32tmp1, int32 p_n32tmp2)
{
    int32 i;
    int32 l_n32totalLen;
    int32 l_n32tmpRight;
    uint8 l_u8Flag;
    uint8 l_u8Count;
    uint8 l_u8DenoiseNum = 1;   // ÿ��ɾ��������ޣ�����1ʱ��Ӧ���޸�һ�£����� l_an32JG0RightPt[l_u8RegionIndex] - i �� �Ƿ񳬹�������ж�
    uint8 l_u8RegionIndex;
    uint8 l_u8RegionNum = 0;
    int32 l_an32LeftPt[MAX_REGION_NUM];
    int32 l_an32RightPt[MAX_REGION_NUM];
    int32 l_an32JG0LeftPt[MAX_REGION_NUM];
    int32 l_an32JG0RightPt[MAX_REGION_NUM];
    int32 l_an32JG1LeftPt[MAX_REGION_NUM];
    int32 l_an32JG1RightPt[MAX_REGION_NUM];
    uint8 l_au8RegionInterweaveFlag[MAX_REGION_NUM];
    
    // �Ƚ���һ�δ���
    Interweave2channel(p_n32tmp1, p_n32tmp2); 
    l_n32totalLen = p_n32tmp1 + p_n32tmp2;

    // �����������������һ������Ӧһ������
    l_u8Flag = 0;
    for (i = 0; i < l_n32totalLen; i++)
    {
        if (l_u8Flag == 0)
        {
           if (g_ZV[i] > 0);//ThresVehLow && g_ZV[i] < ThresVehHigh) //��Ч���¼Ϊ��߽��
            {
                l_u8Flag = 1;
                l_u8Count = 0;
                l_u8RegionNum++;
                l_an32LeftPt[l_u8RegionNum - 1] = i;
                l_n32tmpRight = i;
            }
        }
        else if (l_u8Flag == 1)
        {
            if (g_ZV[i] > 0)//ThresVehLow && g_ZV[i] < ThresVehHigh) //��Ч��
            {
                l_u8Count = 0;      // ��������
                l_n32tmpRight = i;  // �ұ߽�����
            } 
            else
            {
                l_u8Count++;
                if (l_u8Count >= MAX_POINT_DIS)
                {
                    l_u8Flag = 0;
                    l_an32RightPt[l_u8RegionNum - 1] = l_n32tmpRight; // ��¼�ұ߽�
                }
            }
        }
    }
    if (l_u8Flag == 1) // ������һ����ֻ�ҵ���߽��͵��߽��ˣ���¼�ұ߽�
    {
        l_an32RightPt[l_u8RegionNum - 1] = l_n32tmpRight;
    }

    // <ע>��Ĭ�ϼ�����0���ұߣ�x����ֵ��Сһ����
    // ��g_XdistanceV��������ж�Ӧλ��
    for (l_u8RegionIndex = 0; l_u8RegionIndex < l_u8RegionNum; l_u8RegionIndex++)
    {
        l_au8RegionInterweaveFlag[l_u8RegionIndex] = 0;
        for (i = 0; i < p_n32tmp1; i++)
        {
            if (g_ZdistanceV[i] > 2//ThresVehLow && g_ZdistanceV[i] < ThresVehHigh
               && g_XdistanceV[i] >= g_XV[l_an32RightPt[l_u8RegionIndex]]
               && g_XdistanceV[i] <= g_XV[l_an32LeftPt[l_u8RegionIndex]])
               //ע�⣬��Pt��Xֵ�����ұߣ�
            {
                l_an32JG0LeftPt[l_u8RegionIndex] = i;
                l_au8RegionInterweaveFlag[l_u8RegionIndex]++;
                break;
            }
        }
        for (i = 0; i < p_n32tmp2; i++)
        {
            if (g_ZdistanceV1[i] > 2//ThresVehLow && g_ZdistanceV1[i] < ThresVehHigh
                && g_XdistanceV1[i] >= g_XV[l_an32RightPt[l_u8RegionIndex]]
                && g_XdistanceV1[i] <= g_XV[l_an32LeftPt[l_u8RegionIndex]])
            {
                l_an32JG1LeftPt[l_u8RegionIndex] = i;
                l_au8RegionInterweaveFlag[l_u8RegionIndex]++;
                break;
            }
        }
        for (i = p_n32tmp1 - 1;  i >= 0; i--)
        {
            if (g_ZdistanceV[i] > 2//ThresVehLow && g_ZdistanceV[i] < ThresVehHigh
               && g_XdistanceV[i] <= g_XV[l_an32LeftPt[l_u8RegionIndex]]
               && g_XdistanceV[i] > g_XV[l_an32RightPt[l_u8RegionIndex]])
            {
                l_an32JG0RightPt[l_u8RegionIndex] = i;
                l_au8RegionInterweaveFlag[l_u8RegionIndex]++;
                break;
            }
        }
        for (i = p_n32tmp2 - 1; i >= 0; i--)
        {
            if (g_ZdistanceV1[i] > 2//ThresVehLow && g_ZdistanceV1[i] < ThresVehHigh
                && g_XdistanceV1[i] <= g_XV[l_an32LeftPt[l_u8RegionIndex]]
                && g_XdistanceV1[i] >= g_XV[l_an32RightPt[l_u8RegionIndex]])
            {
                l_an32JG1RightPt[l_u8RegionIndex] = i;
                l_au8RegionInterweaveFlag[l_u8RegionIndex]++;
                break;
            }
        }
    }
    // ���������ȥ���
    for (l_u8RegionIndex = 0; l_u8RegionIndex < l_u8RegionNum; l_u8RegionIndex++)
    {
        if (l_au8RegionInterweaveFlag[l_u8RegionIndex] != 4)
        {
            continue;
        }
        for (i = 0; i < l_u8DenoiseNum; i++)
        {
            if (g_XdistanceV[l_an32JG0LeftPt[l_u8RegionIndex] - i] > g_XdistanceV1[l_an32JG1LeftPt[l_u8RegionIndex]]) // ע��Խ�����
            {
                g_ZdistanceV[l_an32JG0LeftPt[l_u8RegionIndex] - i] = 0;
            }
        }
        for (i = 0; i < l_u8DenoiseNum; i++)
        {
            if (g_XdistanceV1[l_an32JG1RightPt[l_u8RegionIndex] + i] < g_XdistanceV[l_an32JG0RightPt[l_u8RegionIndex]]) // ע��Խ�����
            {
                g_ZdistanceV1[l_an32JG1RightPt[l_u8RegionIndex] + i] = 0;
            }
        }
    }

    // �ٴν��д���
    Interweave2channel(p_n32tmp1, p_n32tmp2); 
}

void Interweave2channel(int32 p_n32tmp1, int32 p_n32tmp2)
{
    int32 i, j;

    memset(g_XV, 0, sizeof(int32) * 400);
    memset(g_ZV, 0, sizeof(int32) * 400);
    for(i=0,j=0;i<p_n32tmp1&&j<p_n32tmp2;)
    {
        if(g_XdistanceV[i] > g_XdistanceV1[j])
        {
            g_XV[i+j]=g_XdistanceV[i];
            g_ZV[i+j]=g_ZdistanceV[i];
            i++;
        }
        else if(g_XdistanceV[i]<g_XdistanceV1[j])
        {
            g_XV[i+j]=g_XdistanceV1[j];
            g_ZV[i+j]=g_ZdistanceV1[j];
            j++;
        }
        else
        {
            g_XV[i+j]=g_XdistanceV[i];
            g_ZV[i+j]=g_ZdistanceV[i];
            i++;
            g_XV[i+j]=g_XdistanceV1[j];
            g_ZV[i+j]=g_ZdistanceV1[j];
            j++;
        }
    }
    if(i==p_n32tmp1&&j<p_n32tmp2)       //ʣ��һ·2
    {
        while(j<p_n32tmp2)
        {
            g_XV[i+j]=g_XdistanceV1[j];
            g_ZV[i+j]=g_ZdistanceV1[j];
            j++;
        }
    }
    else if(j==p_n32tmp2&&i<p_n32tmp1)  //ʣ��һ·1
    {
        while(i<p_n32tmp1)
        {
            g_XV[i+j]=g_XdistanceV[i];
            g_ZV[i+j]=g_ZdistanceV[i];
            i++;
        }
    }
}
