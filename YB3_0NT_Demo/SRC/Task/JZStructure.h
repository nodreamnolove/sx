/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			JZStructure.h
** Last modified Date:  2011511
** Last Version:		1.0
** Descriptions:		���س���ṹ
**
**--------------------------------------------------------------------------------------------------------
** Created by:			ZHANG Ye
** Created date:		2011511
** Version:				1.0
** Descriptions:		
**
**--------------------------------------------------------------------------------------------------------
** Modified by:			Wang ZiFeng
** Modified date:		20130318
** Version:				2.0
** Descriptions:		
**
*********************************************************************************************************/
#ifndef	__JZSTRUCTURE_H
#define	__JZSTRUCTURE_H

#include "config.h"


#if	YBVERSION >= 30		//3.0�Ǳ���

typedef	struct tagNetInfo
{
	uint8	au8ServerIP1[4];				//������1(��ֱ����)IP��ַ 
	uint32	u32ServerPortNO1;				//������1�˿ں�
	uint8	au8ServerIP2[4];				//������2IP��ַ
	uint32	u32ServerPortNO2;				//������2�˿ں�	
	uint8	au8ServerIP3[4];				//������3IP��ַ
	uint32	u32ServerPortNO3;				//������3�˿ں�	
	uint8	au8ServerIP4[4];				//������3IP��ַ
	uint32	u32ServerPortNO4;				//������3�˿ں�	
	uint8	au8IPAddr[4];					//IP	
	uint8	au8SubMask[4];					//��������
	uint8	au8GatewayIP[4];				//Ĭ������	
	uint32	u32LocalPortNO;
//32		
	uint8	au8MACAddr[6];					//MAC
	uint8	au8CRC[2];						//CRC
	
} NetInfo;	 

#endif	//#if	YBVERSION >= 30		//3.0�Ǳ���

typedef	struct tagSystemTime
{
	uint16	u16Year;				//��
	uint8	u8Month;				//��
	uint8	u8Day;					//��
	uint8	u8Week;					//���ڼ�
	uint8	u8Hour;					//ʱ
	uint8	u8Minute;				//��
	uint8	u8Second;				//��
} SystemTime;

typedef struct tagSetupParam				     //2013-04-15 �����޸Ľṹ���ϵĲ���
{
	uint8 	u8BaudRate;			  				//����0������ 
	uint8 	u8DOG;			 					//���Ź�����
	uint8  	u8Year;							 	//�����޸�����--��
	uint8  	u8Month;							//�����޸�����--��
	uint8  	u8Day;								//�����޸�����--�� 
	uint8	au8ReserveByte1[3];					//20130418
//8
	int32	 HeightLaser0;                           //������0�߶�ֵ	 
	int32	 HeightLaser1;                           //������1�߶�ֵ	 
	int32	 HeightLaser2;                           //������2�߶�ֵ	 
	int32	 HeightLaser3;                           //������3�߶�ֵ	   	  
//	int32	 IncHeightLaser;                        //��������б�߶�ֵ	
	int32    LaserDistance;	                        //������֮�����
//	int32    Angle12;						    	//��б�������ĽǶ�
	int32    LaneWide;                              //%�������
    int32    MedianWide;                       //������ұ߿��
    int32    MedianLeftWide;         	            //�������߿��
	int32	 VdirAngle;								//��ֱƫ��:����
    int32	 IdirAngle;								//��бƫ��:����
	uint32   resetCnt;								//��¼��������
	uint32	 u32LaserRoadAngle;						//����ɨ�����ƫ��
	uint32	 VerticalLaser_IP;						//��ֱ����IP	20130418  VerticalLaser_IP
	uint32	 VerticalLaser_Port;					//
	uint32	 ParallerLaser_IP1;						// ƽ�з��򼤹���1
	uint32	 ParallerLaser_Port1;						//
	uint32	 ParallerLaser_IP2;						//InclineLaser_IP ƽ�з��򼤹���2
	uint32	 ParallerLaser_Port2;						//
	uint32	 ParallerLaser_IP3;						//InclineLaser_IP ƽ�з��򼤹���2
	uint32	 ParallerLaser_Port3;						//
	uint32	 u32LocalIPAddress;						//
	uint32	 u32SubMask;							    //
	uint32	 u32GatewayIP;								//
	uint32	 u32LocalPortNO;							//
	uint8	 au8LocalMAC[6];							//
	uint8	 au8ReserveByte2[2];
//92	 
	uint32	 u32Net1_DisconnectNum;
	uint32	 u32Net2_DisconnectNum;
	uint32	 u32Net3_DisconnectNum;
	uint32	 u32Net4_DisconnectNum;
	uint32	 u32Net1_InvalidRecNum;
	uint32	 u32Net2_InvalidRecNum;
	uint32	 u32Net3_InvalidRecNum;
	uint32	 u32Net4_InvalidRecNum;
	uint32	 u32DataProcException;
//112
	uint8    au8ProgramVersion[11];					//����汾�� 20130922
	int32	 n32LaserHorizOff;					    //����ˮƽλ��ƫ��(X����)
	uint16   u16VerticalZeroPos0;                       //��ֱ��������0��λ��
	uint16   u16VerticalZeroPos1;                       //��ֱ��������1��λ��
	uint16   u16VerticalZeroPos2;                       //��ֱ��������2��λ��
	uint16   u16VerticalZeroPos3;                       //��ֱ��������3��λ��
//	uint16   u16InclineZeroPos;                       //��б��������0��λ��
	uint16   u16StartPtNum0;                         //��ʼ����1
	uint16   u16EndPtNum0;                           //��ֹ����1
	uint16   u16StartPtNum1;                         //��ʼ����2
	uint16   u16EndPtNum1;                           //��ֹ����2
	uint16   u16StartPtNum2;                         //��ʼ���� 3
	uint16   u16EndPtNum2;                           //��ֹ����3
	uint16   u16StartPtNum3;                         //��ʼ����4
	uint16   u16EndPtNum3;                           //��ֹ����4
	uint8	 u8LaserDevType;						//������������(0:sick,1:wj)
	uint8	 u8TrafficType;							//��������
	uint8    u8RoadType;                            //��·���� 0 ���� 1 ����
	uint8    u8LaneNum;                             //������ 0-- 4���� 1-- 6����
	uint8    u8NetType;							 //�������� ��0���ߴ��䣬1���ߴ���	������û�У�
	uint8    u8InstallFlag;                         //��װ��ʽ��0��װ��1��װ
	uint8    u8SDEnable;                  //SD���ɼ�����ʹ�� ��0��ʾ��ʹ�ܣ�1��ʾʹ�ܣ�Ĭ����0	������û�У�

//130
	uint8 	au8ReserveByte3[4];				//Ԥ���ֽ�	
	uint16	u16CRC;								//CRCУ����	
	
} SetupParam;

#define POINTSET_MASK	0x0F
#define POINTSET_CNT	0x10

typedef struct tagIncPtSt
{ 
	int32  n32y1;  //λ��1 
	int32  n32y2; //λ��2
	uint16  u16yMaxHt; //���ֵZ  
		
	uint16   u16Pt1; //λ�õ�
	uint16   u16Pt2; //λ�õ�2
	uint16  u16yDis;
	uint8	u8DaFeiFlag1;	 //������	��ͷ
	uint8	u8DaFeiFlag2;    //������	��β
}IncPtSt;

typedef struct tagPtIncSet
{
	uint8  u8Sum;	   //�г�������
	uint8   uValid[POINTSET_CNT]; //�������Ч��ʶ
	IncPtSt	IncPtdata[POINTSET_CNT]; 
}PtIncSet;

#define FRAME_MASK	   0xFF
#define FRAME_MAXCNT   0x100 //256
#define FRAME_BUFLEN   0x80 //128

#define NO_USED			0x00  //��Ч
#define OCCURING_USED	0x01  //���ڽ���ʱ
#define PASSED_USED		0x02  //��ʻ��

typedef struct tagVehIncDataSt
{
	 uint16 u16FrameCnt; //��֡�� 

	 int32  ydata[FRAME_MAXCNT][FRAME_BUFLEN];
	 int32	ydataInfo[FRAME_MAXCNT][10]; //��ͷ����βλ�ü���Ӧ��/��ͷ����β��ɱ�־/���޳�ͷ
	 int32  yMax[FRAME_MAXCNT];	 
	 int32  zdata[FRAME_MAXCNT][FRAME_BUFLEN];
	 int32  zMax[FRAME_MAXCNT]; 
	 uint32 tdata[FRAME_MAXCNT]; //ʱ��
}VehIncDataSt;

typedef struct tagVehIncSt
{
	uint8 	u8Istate; //������Ч��־ NO_USED,OCCURING_USED��PASSED_USED
	uint8	u8ThrowFlag;
	uint8 	u8LineFlag1;		//���㳵�����߱�־
	uint8	u8LineFlag2;
	int32	nStartTime;
	int32	nEndTime;
	int32	ndeltaY;
	int32  yLen;  		//������
	int32  zLen;  		//����ĸ߶�
	int32  speed;
	int32  IemptFrame; //��ֱ�հ�֡��
	VehIncDataSt  Idata;  //��ֱ
}VehIncSt;

typedef struct tagPointStruct
{ 
	int32   n32xLeft;  //���λ�� 
	int32   n32xRight; //�Ҳ�λ�� //wzf
	uint16  u16xMaxHt; //���ֵZ  
		
	uint16  u16Leftpt; //���λ�õ�
	uint16  u16Rightpt; //�Ҳ�λ�õ�
	uint16  u16xDis;	  //�������
	uint16  u16Startpt;  //ÿ֡��ʼ��Ч������������ڵ�ƥ��
}PointStruct;

typedef struct tagPointSet
{
	uint8  u8Sum;	   //�г�������
	uint8   uValid[POINTSET_CNT]; //�������Ч��ʶ
	PointStruct	Ptdata[POINTSET_CNT]; 
}PointSet;

/************����ÿ֡�ṹ��**************/
/****************************************/

typedef struct tagVehicleDataStruct
{
	 uint16 u16FrameCnt; //��֡�� 
	 int32  xdata[FRAME_MAXCNT][FRAME_BUFLEN];
	 int32  xMax[FRAME_MAXCNT];	 
	 int32  zdata[FRAME_MAXCNT][FRAME_BUFLEN];
	 int32  zMax[FRAME_MAXCNT]; 
	 uint32 tdata[FRAME_MAXCNT]; //ʱ��
	 uint16 u16xDis[FRAME_MAXCNT]; 
	 uint16 u16xMaxHt[FRAME_MAXCNT];
}VehicleDataStruct;
#define MAX_SVEH_NUM 20
typedef struct __Sveh_Que
{
	uint8 head;
	uint8 tail;
	uint8 frame11_buf_Que[MAX_SVEH_NUM][49];
}_Sveh_Que;

typedef struct __tagVehSendInfo
{
	uint16	u16Year;			//��
	uint8	u8Month;			//��
	uint8	u8Day;				//��
	uint8	u8Hour;				//ʱ
	uint8	u8Minute;			//��
	uint8	u8Second;			//��
	uint32 u32Veh_Index;		//������� ռ3���ֽ�
}_VehSendInfo;

#define NORMAL_MAX_EMPTYFRAME	0x05  //�����հ�֡����
#define ERR_MAX_EMPTYFRAME		0x32  //�������հ�֡����
#define ERR_MAXVER_EMPTYFRAME      0x0A  //����������б��������������ֱ�����հ�֡����
typedef struct tagVehicleStruct
{
	uint8 u8Vstate; //��Ч��־ NO_USED,OCCURING_USED��PASSED_USED
	uint8 u8VehPattern;   //����
	uint8 u8Lane; //����
	int32  yLen;  //������
	int32  zLen;  //����ĸ߶�
	int32  xLen;   //����Ŀ��
	int32  speed;
	int32  VemptFrame; //��ֱ�հ�֡��
	PointStruct	locateX; 
	PointStruct VLocateX;  // ��ֱ�������г�������λ��
	VehicleDataStruct  Vdata;  //��ֱ
}VehicleStruct;

#if 1 == TEST_PROBE
#define	RESETINFO_BUFFERSIZE	10
typedef struct tagCycleBufferStruct
{
	uint8	u8CurrentPos;
	uint8	u8CurrentEntries;
	uint8	u8MaxBufSize;
	uint8	u8Reserved;
	uint8	u8Type[RESETINFO_BUFFERSIZE];			
	struct 	tagSystemTime	SysTimeTable[];
}CycleBufferStruct;


#endif




typedef enum{SICKJGDATA=0,WJJGDATA=1}JGTYPE;
typedef enum{FIRSTDEVTYPE=0,SECONDDEVTYPE=1}TRAFFICTYPE;

#endif
