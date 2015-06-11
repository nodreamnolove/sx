/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			JZStructure.h
** Last modified Date:  2011511
** Last Version:		1.0
** Descriptions:		计重程序结构
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


#if	YBVERSION >= 30		//3.0仪表功能
#pragma pack(1)
typedef	struct tagNetInfo
{
	uint8	au8IPAddr[4];					//IP
	uint32	u32LocalPortNO;	
	uint8	au8SubMask[4];					//子网掩码
	uint8	au8GatewayIP[4];				//默认网关	

//32		
	uint8	au8MACAddr[6];					//MAC
	uint8	au8ServerIP1[4];				//服务器1(垂直激光)IP地址 
	uint32	u32ServerPortNO1;				//服务器1端口号
	uint8	au8ServerIP2[4];				//服务器2IP地址
	uint32	u32ServerPortNO2;				//服务器2端口号	
	uint8	au8ServerIP3[4];				//服务器3IP地址
	uint32	u32ServerPortNO3;				//服务器3端口号	
	uint8	au8ServerIP4[4];				//服务器3IP地址
	uint32	u32ServerPortNO4;				//服务器3端口号	

	uint8	au8CRC[2];						//CRC
	
} NetInfo;	 
#pragma pack() 
#endif	//#if	YBVERSION >= 30		//3.0仪表功能
	
typedef	struct tagSystemTime
{
	uint16	u16Year;				//年
	uint8	u8Month;				//月
	uint8	u8Day;					//日
	uint8	u8Week;					//星期几
	uint8	u8Hour;					//时
	uint8	u8Minute;				//分
	uint8	u8Second;				//秒
} SystemTime;
#pragma pack(1)
typedef struct tagSetupParam				     //2013-04-15 徐威修改结构体上的参数
{
//版本
  	uint8    au8ProgramVersion[11];					//程序版本号 20130922
//安装方式 
   	uint8    u8InstallFlag;                         //安装方式，0侧装，1正装
//设备ID
	uint32   u32DevID;   
//波特率
	uint8 	u8BaudRate;			  				//串口0波特率
//看门狗
	uint8 	u8DOG;			 				
//设备分类
	uint8	 u8TrafficType;							//交调类型
//数据上传类型
	uint8    u8NetType;							 //网络类型 ，0无线传输，1有线传输	（功能没有）
//重启次数
	uint32   resetCnt;								//记录重启次数 
//设备厂家
	uint8	 u8LaserDevType;						//激光设置类型(0:sick,1:wj)
//sd卡使能
	uint8    u8SDEnable;                  //SD卡采集数据使能 ，0表示不使能，1表示使能，默认是0	（功能没有）
//激光0、1、2、3高度 
	int32	 J0_Height;                           //激光器0高度值	 
	int32	 J1_Height;                           //激光器1高度值	 
	int32	 J2_Height;                           //激光器2高度值	 
	int32	 J3_Height;                           //激光器3高度值
//水平激光间距
	int32    LaserDistance;	                        //激光器之间距离      非常重要
//激光0中起始点
	uint16   u16J0ZeroPos;                       
	uint16   u16J0StartPos;                         //起始点数0
	uint16   u16J0EndPos;                           //终止点数0
//激光1中起始点
	uint16   u16J1ZeroPos;                      
	uint16   u16J1StartPos;                         //起始点数2
	uint16   u16J1EndPos;                           //终止点数2
//激光2中起始点
	uint16   u16J2ZeroPos;                      
	uint16   u16J2StartPos;                         //起始点数 3
	uint16   u16J2EndPos;                           //终止点数3
//激光3中起始点
	uint16   u16J3ZeroPos;  
	uint16   u16J3StartPos;                         //起始点数4
	uint16   u16J3EndPos;                           //终止点数4
//车道宽度
 	int32    LaneWide;                              //%车道宽度
//激光0距车道边距
 	int32    MedianWide;                       		//车道左边坐标	 激光0距车道距离
//激光1距车道边距
	int32    MedianLeftWide;         	            //车道右边坐标	激光1距车道距离
//车道数  	
	uint8    u8LaneNum;                             //车道数 0-- 4车道 1-- 6车道
//车道类型
	uint8    u8RoadType;                            //道路类型 0 国道 1 高速
//控制器IP	  port
	uint32	 u32LocalIPAddress;						
	uint32	 u32LocalPortNO;						
//控制器子网掩码
	uint32	 u32SubMask;
//控制器网关
	uint32	 u32GatewayIP;		
//控制器mac
	uint8	 au8LocalMAC[6];
//激光0 ip  port
	uint32	 J0_IP;						//垂直激光IP	20130418  VerticalLaser_IP
	uint32	 J0_Port;					//
//激光1 ip  port
	uint32	 J1_IP;						// 平行方向激光器1
	uint32	 J1_Port;						//
//激光2 ip	port
	uint32	 J2_IP;						//InclineLaser_IP 平行方向激光器2
	uint32	 J2_Port;						//
//激光3 ip  port
	uint32	 J3_IP;						//InclineLaser_IP 平行方向激光器2
	uint32	 J3_Port;		
//服务器 ip port
	uint32  u32ServerIP;					//服务器IP
	uint16  u16ServerPort;					//服务器port	
//网络0 断开 无效
	uint32	 u32Net1_DisconnectNum;
	uint32	 u32Net1_InvalidRecNum;	
//网络1 断开 无效
	uint32	 u32Net2_DisconnectNum;
	uint32	 u32Net2_InvalidRecNum;
//网络2 断开 无效
	uint32	 u32Net3_DisconnectNum;
	uint32	 u32Net3_InvalidRecNum;
//网络3 断开 无效
	uint32	 u32Net4_DisconnectNum;
	uint32	 u32Net4_InvalidRecNum;
//异常码
	uint32    u32nonormalNum;
//预留 
	uint8 	au8ReserveByte[4];				//预留字节
	uint16	u16CRC;								//CRC校验码		
} SetupParam;
#pragma pack()
#define POINTSET_MASK	0x0F
#define POINTSET_CNT	0x10

typedef struct tagIncPtSt
{ 
	int32  n32y1;  //位置1 
	int32  n32y2; //位置2
	uint16  u16yMaxHt; //最大值Z  
		
	uint16   u16Pt1; //位置点
	uint16   u16Pt2; //位置点2
	uint16  u16yDis;
	uint8	u8DaFeiFlag1;	 //打飞情况	车头
	uint8	u8DaFeiFlag2;    //打飞情况	车尾
}IncPtSt;

typedef struct tagPtIncSet
{
	uint8  u8Sum;	   //有车的数量
	uint8   uValid[POINTSET_CNT]; //区域的有效标识
	IncPtSt	IncPtdata[POINTSET_CNT]; 
}PtIncSet;

#define FRAME_MASK	   0xFF
#define FRAME_MAXCNT   0x100 //256
#define FRAME_BUFLEN   0xff //128

#define NO_USED			0x00  //无效
#define OCCURING_USED	0x01  //正在进行时
#define PASSED_USED		0x02  //已驶过

typedef struct tagVehIncDataSt
{
	 uint16 u16FrameCnt; //总帧数 

	 int32  ydata[FRAME_MAXCNT][FRAME_BUFLEN];
	 int32	ydataInfo[FRAME_MAXCNT][10]; //车头、车尾位置及对应点/车头、车尾打飞标志/有无车头
	 int32  yMax[FRAME_MAXCNT];	 
	 int32  zdata[FRAME_MAXCNT][FRAME_BUFLEN];
	 int32  zMax[FRAME_MAXCNT]; 
	 uint32 tdata[FRAME_MAXCNT]; //时间
}VehIncDataSt;

typedef struct tagVehIncSt
{
	uint8 	u8Istate; //过车有效标志 NO_USED,OCCURING_USED，PASSED_USED
	uint8	u8ThrowFlag;
	uint8 	u8LineFlag1;		//计算车长过线标志
	uint8	u8LineFlag2;
	int32	nStartTime;
	int32	nEndTime;
	int32	ndeltaY;
	int32  yLen;  		//车身长度
	int32  zLen;  		//车身的高度
	int32  speed;
	int32  IemptFrame; //垂直空白帧数
	VehIncDataSt  Idata;  //垂直
}VehIncSt;

typedef struct tagPointStruct
{ 
	int32   n32xLeft;  //左侧位置 
	int32   n32xRight; //右侧位置 //wzf
	uint16  u16xMaxHt; //最大值Z  
		
	uint16  u16Leftpt; //左侧位置点
	uint16  u16Rightpt; //右侧位置点
	uint16  u16xDis;	  //车辆宽度
	uint16  u16Startpt;  //每帧开始有效点的索引，用于点匹配
}PointStruct;

typedef struct tagPointSet
{
	uint8  u8Sum;	   //有车的数量
	uint8   uValid[POINTSET_CNT]; //区域的有效标识
	PointStruct	Ptdata[POINTSET_CNT]; 
}PointSet;

/************定义每帧结构体**************/
/****************************************/

typedef struct tagVehicleDataStruct
{
	 uint16 u16FrameCnt; //总帧数 
	 int32  xdata[FRAME_MAXCNT][FRAME_BUFLEN];
	 int32  xMax[FRAME_MAXCNT];	 
	 int32  zdata[FRAME_MAXCNT][FRAME_BUFLEN];
	 int32  zMax[FRAME_MAXCNT]; 
	 uint32 tdata[FRAME_MAXCNT]; //时间
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
	uint16	u16Year;			//年
	uint8	u8Month;			//月
	uint8	u8Day;				//日
	uint8	u8Hour;				//时
	uint8	u8Minute;			//分
	uint8	u8Second;			//秒
	uint32 u32Veh_Index;		//数据序号 占3个字节
}_VehSendInfo;

#define NORMAL_MAX_EMPTYFRAME	0x0A  //正常空白帧上限
#define ERR_MAX_EMPTYFRAME		0x14  //非正常空白帧上限
#define ERR_MAXVER_EMPTYFRAME      0x0A  //非正常（倾斜激光器进车）垂直出车空白帧上限
typedef struct tagVehicleStruct
{
	uint8 u8Vstate; //有效标志 NO_USED,OCCURING_USED，PASSED_USED
	uint8 u8VehPattern;   //车型
	uint8 u8Lane; //车道
	int32  yLen;  //车身长度
	int32  zLen;  //车身的高度
	int32  xLen;   //车身的宽度
	int32  speed;
	int32  VemptFrame; //垂直空白帧数
	PointStruct	locateX; 
	PointStruct VLocateX;  // 垂直激光器中车的左右位置
	VehicleDataStruct  Vdata;  //垂直
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
