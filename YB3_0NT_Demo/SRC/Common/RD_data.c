//=========================
//本文件来自JD	 360点采集工程
//==========================
#include "RD_data.H"
#include <string.h>
#include "common.h"
#include "config.h"
#include "crc.h"
#include "W5100.h"
#include "WT_Task.h"
#include "PCF8563.h"
#include "Uart5.h"
#include "Task_Uart5_Senddata_NEW.h"
#include "Task_SD.h"      //20130826 读写SD卡增加或修改
#include "TDC256.h"			//20130826 读写SD卡增加或修改
#include "Task_SD.h"
#include "Uart1.h"								   //20130609
#define ResetSystem()  			{WDTInit(1),WDTIM_COUNTER =  0x10000000-100;while(1); }
#define		SETUPALIAS				g_sspSetup
//#include

uint16	TcpRec_len,TcpSend_len;				//TCP接收、发送数据长度
extern uint8 Flag_NetConnect;//已有在WT_task中		//网络连接标志位，为1表示网络连接成功，为0表示网络中断
uint8 Flag_08_NotReturn;						//08包未返回标志位，为1表示还已经接收到数据，但还没有返回08包
uint8 Flag_02Rev;							//02包收到标志位，收到02包该标志置1
uint8 Flag_0ARev;							//08包收到标志位，收到08包该标志置1

OS_EVENT *Flag_SendOK;						//数据发送成功信号量，数据发送后释放该信号量，发送数据时请求该信号量
OS_EVENT *EVENT_02Rev;						//02包收到信号量，收到时释放信号量，发送01包前先发送02包，再请求该信号量
OS_EVENT *EVENT_0ARev;						//0A包收到信号量，收到0A包后释放该信号量，表明01包数据发送成功
OS_EVENT *EVENT_11Rev;

union ASK08Data	ASK08_data;
union ASK02Data	ASK02_data;
union ASK08Data_Modify ASK08_data_Modify;
char FlagSentVechicle;
uint8 Flag_ReConnect;
uint8 Flag_08_ReturnChangePara;

#define SENDTIME  80000 //需改 						//判断多长时间没有收到包认为网络中断，5s

#define wireless_len 100

uint8	Hisyear;
uint8	Hismonth;
uint8	Hisday;
//uint8	
//共0-0xECDFFF块
uint32 SD_Base_address_01  =0xE00000;
uint32 SD_address_01 =0;



uint8 UserName[] 		= "11111111"; 			//用户名
uint8 KeyNum[]	 		= "11111111";			//密码

uint8 RDid[]	 		= "0171170313079999";	//设备身份识别码 16位
uint8 RDNum[15]	 		= "G104L410339999";		//站点编号，15位，不足15位补0


uint16 ProCycle			= 5;					//交通数据处理周期 
uint8 InvContents		= 2; 					//调查内容 	
uint8 DisTime			= 10;					//跟车百分比鉴别时间 

uint8 NewUserName[]		= "11111111"; 			//新用户名
uint8 NewKeyNum[]		= "11111111";			//新密码
uint8 NewRDid[]			= "0171170313079999";	//新设备身份识别码
uint8 NewRDNum[15]		= "G104L410339999";		//新站点编号，15位，不足15位补0
uint16 NewProCycle		= 5;					//新交通数据处理周期 
uint8 NewInvContents	= 2; 					//新调查内容 	
uint8 NewDisTime		= 10;					//新跟车百分比鉴别时间
uint8 NewDscIp[4]; 								//新服务器IP地址

uint8 ConnectType		= 1;					//当前传输方式。01：有线网络传输  02：无线网络传输
double Longitude		= 113.55002565147197;	//经度
double Latitude			= 24.040477460757732;	//纬度
uint16 Elevation		= 8814;	
				//海拔  	 
uint16 DataResendBgnNum;
uint16 DataResendEndNum;
uint16 DataResendCnt;
struct ASK_09 ASK09;

double ParsReturn[8];
int LMS_Buf0[600];
int LMS_Buf[600];

 uint8 Flag_Change_RD_Num;//（修改站点编号）
 uint8 Flag_Change_DSC_Ip;////IP修改
 uint8 Flag_Change_InvContents;//修改调查内容
 uint8 Flag_Change_ProCycle;//(处理周期)
 uint8 Flag_Change_DisTime;//跟车百分比鉴别时间

//万集查询设备状态指令	
#define	WJ_CMD_DEVICERESETINFO	0x30		
#define	WJ_CMD_DEVICEPARAM		0x31
#define	WJ_CMD_DEVICEOTHERS		0x32
#define	WJ_CMD_CLEARRESETIFNOBUF	0x34

#define	WJ_CMD_QUERYPARAM 		0x35 //远程获取参数
#define WJ_CMD_SETPARAM			0x36 //远程设置参数
#define WJ_CMD_GETFRAME			0x37 //获取单帧波形 垂直
#define WJ_CMD_GETFRAME2        0x38 //获取单帧波形 倾斜

#define WJ_CMD_GET_THRESHOLD        0x20 //获取阈值
#define WJ_CMD_SET_THRESHOLD        0x21 //设置阈值

#define	WJ_CMD_GETDEVICETIME 		0x39 //获取控制器时间
#define WJ_CMD_SETDEVICETIME		0x3A //设置控制其时间
#define WJ_CMD_GETRDID				0x3B //获取识别码
#define WJ_CMD_SETRDID       		0x3C //设置识别码
#define WJ_CMD_INITALLPARAM			0x3d //初始化所有参数
#define WJ_CMD_RESETDEVICE       	0x3e //重启控制器
#define WJ_CMD_GETLANEDIR           0x40 //远程获取上下行方向
#define WJ_CMD_SETLANEDIR           0x41 //远程设置上下行方向
#define WJ_CMD_GETSD01DATA          0x42 //获取SD卡中的01包数据
#define WJ_CMD_GETSD01DATAERR       0x43 //获取SD卡中的01包数据异常

TRAFFIC_DATA_SEND01  Data_4_Lane[4];

static void SendDevResetInfo(void);

/**************************************************************************************
函数功能：计算自2012.1.1起的给定日期（年月日）是星期几；hong//2012.8.2
**************************************************************************************/
uint8 Function_Week(uint16 *p)
{
   //2012 1 1 星期日
   uint8 week;		  //返回值星期X
   uint8 Month_day[12]={31,28,31,30,31,30,31,31,30,31,30,31};//非闰年月份天数；
   uint8 Month_day_run[12]={31,29,31,30,31,30,31,31,30,31,30,31};//闰年月份天数；
   uint32 sum_day=0;//自2012-1-1计算的天数
   uint8 i=0;
   uint8 num_run=0;	 //已过去的闰年个数
   //普通年能整除4且不能整除100的为闰年。（如2004年就是闰年,1900年不是闰年） 
   //世纪年能整除400的是闰年。(如2000年是闰年，1900年不是闰年) 

   if(p[0]>=2012)
   {
   	 if(p[1]>=1 && p[1]<=12)
	 {
	 	if(p[2]>=1 && p[2]<=31)
		{
			if(p[0]==2012)
			{
				num_run = 0;
			}
			else
			{
			num_run = (p[0]-1 - 2012 )/4 +1;
			}
			if((p[0]%4 == 0 && p[0]%100 !=0) || (p[0]%400 == 0))
			{			   
			   for(i=0;i<p[1]-1;i++)
			      sum_day = Month_day_run[i] +  sum_day;
			   sum_day = (p[0]-2012-num_run)*365 + num_run*366 + sum_day + p[2]; 
			}
			else
			{
			   for(i=0;i<p[1]-1;i++)
			      sum_day = Month_day[i] +  sum_day;
			   sum_day = (p[0]-2012-num_run)*365 + num_run*366 + sum_day + p[2]; 
			}
		}
	 }
   }
   week = (sum_day+6) % 7 ;
   return week;
}
/**************************************************************/
//typedef	struct tagSystemTime
//{
//	uint16	u16Year;				//年
//	uint8	u8Month;				//月
//	uint8	u8Day;					//日
//	uint8	u8Week;					//星期几
//	uint8	u8Hour;					//时
//	uint8	u8Minute;				//分
//	uint8	u8Second;				//秒
//} SystemTime;
/************函数：获取时间编号********************************
函数名称：Get_NUM_From_Time
创建日期：2012-8-14；hong
函数说明：以2012-01-01 00:00:00为时间起点，时间编号为自时间起点至今的总分钟数。
返回值为时间编号；
函数修改：
************************************************************/
uint32 Get_NUM_From_Time(SystemTime  Temp_Time)
{
   uint8 Month_day[12]={31,28,31,30,31,30,31,31,30,31,30,31};//非闰年月份天数；
   uint8 Month_day_run[12]={31,29,31,30,31,30,31,31,30,31,30,31};//闰年月份天数；
   uint32 sum_day=0;//自2012-1-1计算的天数
   uint8 i=0;
   uint8 num_run=0;	 //已过去的闰年个数
   uint32 Num_time;

//	Temp_Time.u8Month
//	Temp_Time.u8Day

	if(Temp_Time.u16Year>=2012)
	{
		if(Temp_Time.u16Year == 2012)
		{
			num_run = 0;
		}
		else
		{
			num_run = (Temp_Time.u16Year-1 - 2012 )/4 +1;
		}
		if((Temp_Time.u16Year%4 == 0 && Temp_Time.u16Year%100 !=0) || (Temp_Time.u16Year%400 == 0))
		{			   
		   for(i=0;i<Temp_Time.u8Month-1;i++)
		      sum_day = Month_day_run[i] +  sum_day;
		   sum_day = (Temp_Time.u16Year-2012-num_run)*365 + num_run*366 + sum_day + Temp_Time.u8Day; 
		}
		else
		{
		   for(i=0;i<Temp_Time.u8Month-1;i++)
		      sum_day = Month_day[i] +  sum_day;
		   sum_day = (Temp_Time.u16Year-2012-num_run)*365 + num_run*366 + sum_day + Temp_Time.u8Day; 
		}		
	}
	Num_time = (sum_day-1)*1440 + Temp_Time.u8Hour * 60 + Temp_Time.u8Minute;	
	return Num_time;
}
/**************************************************************
函数名称：GET_SDaddress_From_TimeNum
创建日期：2012-8-11
函数说明：入口参数为交调时间序号（0-1440之间），
		 返回值为该序号交调01包存储的SD address
		 325440 至2012-8-13总计226天，226*1440 = 325440；
		 现有
**************************************************************/
uint32 GET_SDaddress_From_TimeNum(uint16 number)
{
	 SystemTime Temp_time;
	 uint32 start_num_of_day;
	 uint32 SD_address_of_num;

	 Temp_time = g_sstCurTime;
	 Temp_time.u8Hour = 0;
	 Temp_time.u8Minute = 0;
	 Temp_time.u8Second = 0;
	 start_num_of_day = Get_NUM_From_Time(Temp_time);
	 SD_address_of_num =  SD_Base_address_01 + start_num_of_day - 325440 + number;
	 return SD_address_of_num;
}



/**************************************************************/
/**************************************************************/
//============================================================
//uint8	UartData_Pro(uint8 *p,uint16 len)
//{	
//	uint16 uchCRC,datalen;
//	uint8	CMDNO;
//	uint8 data[500];
//		
//	while(len)
//	{
//		if(p[0]==0xFF)
//		{
//			CMDNO=p[2];
//			datalen=(p[3]<<4+p[4])-8;
//			memcpy(data, p+5, datalen);
//		//	Udata_Pro(data,CMDNO,datalen);
//			len=len-datalen+8;
//		}
//		else
//		{
//			p++;
//			len--;
//		}
//
//	}
//	return 1;
//}


//========================================================
//TCP数据包处理，自动去掉头、尾、自动校验；
//指针  长度
uint8 TcpData_Pro(uint8 *p_uart5buff,uint16 p_len)
{
	uint8 data[500];
	uint16 uchCRC,datalen;
//数据结构：帧头2字节，包长2字节，数据包内容N字节，CRC校验2字节，帧尾2字节
	while(p_len)
	{
		if((p_uart5buff[0] == 0xAA) && (p_uart5buff[1] == 0xAA))
		{
			datalen = p_uart5buff[3];
			datalen = (datalen<<8) + p_uart5buff[2];
			if((p_uart5buff[datalen+6] == 0xEE) && (p_uart5buff[datalen+7] == 0xEE))
			{
			 	uchCRC = p_uart5buff[datalen+5];
				uchCRC = (uchCRC<<8) + p_uart5buff[datalen+4];
				if( uchCRC == CRC16(p_uart5buff+2,datalen+2))	   //CRC校验
				{
					memcpy(data, p_uart5buff+4, datalen);
//====数据交由RD_DataPro处理
					RD_DataPro(data,datalen);	 //定义在下面
					p_len = p_len - (datalen+8);
					p_uart5buff = p_uart5buff+(datalen+8);
				}	
				else
					return 0;	//错误返回		
			}
			else 
				return 0;	//错误返回			 
		}
		else
		{
			p_uart5buff++;
			p_len--;
		}
	}
	return 0;	//错误返回
}
//===========================================================
//接入为包中包含的数据；（去头去尾）
//入口参数为：数据包中数据和数据长度
uint32 Rev0A_count = 0;
void RD_DataPro(uint8 *p_data,uint16 len)
{	
//	uint16 dayinyear;

	if(p_data[0] == 0x0A)	//首位为包类型								//0A包
	{
//		Flag_0ARev = 1;
		//Rx_0A_01_TimeNum = p[1] + (p[2]<<8);	      // 解析时间序号
		OSSemPost(EVENT_0ARev);
		Rev0A_count = Rev0A_count + 1 ;
//		if((p[1] != 0xFF) || (p[2] != 0xFF) );			//校验出错,该如何处理？
	}
	else if( RD_ID_Key((struct RD_data *)p_data,len) )			//判断是否为本RD //设备身份识别码校验函数，正确返回1，错误返回0
	{			  
		switch(p_data[0])
		{
			case 0x02: //02判断与处理
				{ 	
					switch(p_data[17])
					{
						case 0x02:
						case 0x03://设备连接正常
//								Flag_02Rev = 1;
								OSSemPost(EVENT_02Rev);								
								break;
						case 0x09:
//								Flag_08_NotReturn = 1;											
								break;
						default:
							break;
					}
				}break;
			case 0x09: //数据重传与处理
				//解析年、月、日
//==待定
//				Hisyear = p[33]+ (p[34]<<8);
//				Hismonth = p[35];
//				Hisday = p[36];
//				dayinyear = sumday(Hisyear,Hismonth,Hisday);
////===待定				ResendBaseAddress = SdBaseAddress + (dayinyear-1)*1440;	//sd卡相关；；；；
//				//起始序号(需要重传的数据的)
//				DataResendBgnNum = p[37]+(p[38]<<8);
//				//结束序号
//				DataResendEndNum = p[39]+(p[40]<<8);
//				if((DataResendBgnNum>DataResendEndNum)||(DataResendBgnNum>1440)|| (DataResendEndNum>1440))
//				{
//					DataResendCnt = 0;
//					break;
//				}
//				else
//				{
//					DataResendCnt = ((DataResendEndNum+1)- DataResendBgnNum);
//				} 	
				break;

			default:
				{
					if(UserName_Key((struct ASK_03 *)p_data,len))  		//判断用户名和密码
					{
						switch(p_data[0])					  //下位修改指令，
						{
							case 0x03:
								Change_RD_Num((struct ASK_03 *)p_data,len);
								break;
							case 0x04:
								Change_DSC_Ip((struct ASK_04 *)p_data,len);
								break;
							case 0x05:
								Change_Time((struct ASK_05 *)p_data,len);
								break;
							case 0x06: //交通数据调查内容修改指令数据包
								Change_InvContents((struct ASK_06 *)p_data,len);
								break;
							case 0x07: //交通数据处理周期修改指令数据包
								Change_ProCycle((struct ASK_07 *)p_data,len);
								break;
							case 0x0B://跟车百分比鉴别时间修改指令数据包
								Change_DisTime((struct ASK_0B *)p_data,len);
								break;
							default:
								break; 
						}
					}
//					Flag_08_NotReturn = 1;											
				}break;
		}
	}
	memset(p_data,0,len);		 //？？	将p中前len个字节写为0；
}
//=============================================
//数据发送函数，可以自动添加帧头、帧尾、CRC校验、数据长度

uint8 SendDataBuff[512]={0};
uint8 SendData(uint8 *p,uint16 len)
{
	uint8 data[1460];
	uint16 uchCRC;

	//帧头
	data[0] = 0xAA;
	data[1] = 0xAA;
	//帧长
	data[2] = len & 0xFF;
	data[3] = (len>>8) & 0xFF;
	memcpy(data+4, p, len);
	//CRC校验
	uchCRC = CRC16(data+2,len+2);
	data[len+4]	= uchCRC & 0xFF;
	data[len+5]	= (uchCRC>>8) & 0xFF;
	//帧尾
	data[len+6]	= 0xEE;
	data[len+7]	= 0xEE;
	memcpy(SendDataBuff, data, len+8);
	TcpSend_len = len+8; //TCP数据发送长度
	
//	if(!Transport_Way)			//Transport_Way;// 数据传输方式 0：有线网络传输 1：无线网络传输
	//启动发送
//	if(0)//===待定
//	{
//		S_tx_process(0, TcpSend_len);  //直接发送通过0端口
//	}
//	else
//	{
		U5SendBytes(SendDataBuff,TcpSend_len);
//	}
	return 1;		
}
//========================================

//通讯参数初始化函数，每次参数改版后调用该函数或修改相关参数，实时时间要实时更改
void RD_Int08(void)
{
	uint8 i;
//	uint8 buf[30];

	ASK08_data.ASK08.RDHead.Type 		= 0x08;
//==待定
//	i2c_readNByte(FM24V05, TWO_BYTE_SUBA, 0xefff, buf, 30); 
	for(i=0;i<16;i++)
		ASK08_data.ASK08.RDHead.RD_ID[i]	= RDid[i];

	for(i=0;i<15;i++)
	{
		ASK08_data.ASK08.RD_Num[i]	= RDNum[i];					//RD站点编号
	}
	//时间
//==待定
	ASK08_data.ASK08.yearL				= YEAR & 0xFF;
	ASK08_data.ASK08.yearH				= (YEAR >> 8) & 0xFF;
	ASK08_data.ASK08.month				= MONTH;
	ASK08_data.ASK08.day				= DAY;
	ASK08_data.ASK08.hour				= HOUR; 
	ASK08_data.ASK08.min				= MIN;
	ASK08_data.ASK08.sec				= SEC;
	for(i=0;i<4;i++)
//==待定
	ASK08_data.ASK08.DSC_Ip[i]			= Ser_Ip[i];				//DSC IP 
	ASK08_data.ASK08.InvContents		= InvContents; 				//调查内容
	ASK08_data.ASK08.ProCycle			= ProCycle;					//交通数据处理周期
	ASK08_data.ASK08.DisTime			= DisTime;					//跟车百分比鉴别时间 
	ASK08_data.ASK08.ConnectType		= ConnectType;			   	//当前传输方式。01：有线网络传输  02：无线网络传输  
	
	for(i=0;i<8;i++)
	{

//设置经纬度，待定；
//		ASK08_data.ASK08.Longitude[i]  = jingdu_buf[i];
//		ASK08_data.ASK08.Latitude[i] = weidu_buf[i];
	}
	ASK08_data.ASK08.ElevationH = Elevation;

	memcpy(ASK08_data_Modify.ASK08_Modify_65,ASK08_data.ASK08_65,65); //备份08包	 

}

//02格式数据包初始化
void RD_Int02(void)
{
	uint8 i;

	ASK02_data.ASK02.RDHead.Type = 0x02;

	for(i=0;i<16;i++)
		ASK02_data.ASK02.RDHead.RD_ID[i] = RDid[i];

	ASK02_data.ASK02.Ask_Answer	= 0x01;
}

//02格式数据包处理函数
void Ask_Answer(struct ASK_02 *p,uint16 len)
{
	switch(p->Ask_Answer)
	{
		case 0x01:
			break;
		case 0x02:
			break;
		case 0x03:
			break;
		case 0x09:
			break;
	}				
}

//03格式数据包处理函数 （修改站点编号）
void Change_RD_Num(struct ASK_03 *p,uint16 len)
{
	uint8 i;
	
	//将正在生效的08包数据，赋值给修改参数的08包，避免更改两个以上参数时，生效机制出错。
//	memcpy(ASK08_data_Modify.ASK08_Modify_65,ASK08_data.ASK08_65,65);
	for(i=0;i<15;i++)
		ASK08_data_Modify.ASK08_Modify.RD_Num[i] = NewRDNum[i] = p->New_RD_Num[i];
	
	for(i=0;i<15;i++)
		NewRDNum[i]= p->New_RD_Num[i];
	//memcpy(RDNum,NewRDNum,sizeof(NewRDNum));
	//WriteC256(写数据首地址，数据buf,数据长度)；
//	待定；
//	i2c_writeNByte(FM24V05, TWO_BYTE_SUBA, 0xefff, NewRDNum, 15);

	Flag_08_ReturnChangePara = 1;
	Flag_Change_RD_Num = 1;
			
}

//04格式数据包处理函数	//IP修改指令
void Change_DSC_Ip(struct ASK_04 *p,uint16 len)
{
	uint8 i;

	memcpy(ASK08_data_Modify.ASK08_Modify_65,ASK08_data.ASK08_65,65);
	for(i=0;i<4;i++)
		ASK08_data_Modify.ASK08_Modify.DSC_Ip[i] = NewDscIp[i] = p->New_DSC_Ip[i];	


//	待定，
//	i2c_writeNByte(FM24V05, TWO_BYTE_SUBA, (0xefff+15), NewDscIp, 4);  //铁电存储器；
	Flag_08_ReturnChangePara = 1;
	Flag_Change_DSC_Ip = 1;		
}


//05格式数据包处理函数		修改时钟
void Change_Time(struct ASK_05 *p,uint16 len)
{	// 本函数功能;将05包中 的时间写入 ASK08_data.ASK08.中，将时间转化为net_time格式，设置时钟芯片PCF时间，设置实时时钟RTC时间；
//	RTC_TIME net_time;
	SystemTime net_time;
	uint16 YearMonthDay[3]={0};
//typedef	struct tagSystemTime
//{
//	uint16	u16Year;				//年
//	uint8	u8Month;				//月
//	uint8	u8Day;					//日
//	uint8	u8Week;					//星期几
//	uint8	u8Hour;					//时
//	uint8	u8Minute;				//分
//	uint8	u8Second;				//秒
//} SystemTime;

	ASK08_data.ASK08.yearL = p->yearL;
	ASK08_data.ASK08.yearH = p->yearH;
	ASK08_data.ASK08.month = p->month;
	ASK08_data.ASK08.day = p->day;
	ASK08_data.ASK08.hour = p->hour;
	ASK08_data.ASK08.min = p->min;
	ASK08_data.ASK08.sec = p->sec;
//		  2012.8.31 星期三
   if(((p->yearL)+((p->yearH)<<8))>=2000)
   {
		net_time.u16Year = (p->yearL)+((p->yearH)<<8);
		net_time.u8Month = p->month;
		net_time.u8Day = p->day;
		net_time.u8Hour = p->hour;
		net_time.u8Minute = p->min;
		net_time.u8Second =p->sec;
		YearMonthDay[0] =  net_time.u16Year;
		YearMonthDay[1] =  net_time.u8Month;
		YearMonthDay[2] =  net_time.u8Day;

		net_time.u8Week = Function_Week(YearMonthDay);//星期设置；

//  	net_time.u8Week = 1;//星期设置；
//		pcf_setTime(&net_time);
//		rtc_setTime(&net_time);
		SetRTCTime(&net_time);
	}
	 Flag_08_ReturnChangePara = 1;
////	待定，hong 2012.05.03
		MinTotalNew = MinTotalOld = HOUR*60 + MIN;
//
//	//时间实时生效，回复正在生效的08包
//	Flag_08_NotReturn = 1;
}


//06格式数据包处理函数(修改调查内容)
void Change_InvContents(struct ASK_06 *p,uint16 len)
{
//	uint8 buf[2];

	memcpy(ASK08_data_Modify.ASK08_Modify_65,ASK08_data.ASK08_65,65);
	ASK08_data_Modify.ASK08_Modify.InvContents = NewInvContents = p->New_InvContents;
//	buf[0] = NewInvContents;
//写入铁电存储器
//	i2c_writeNByte(FM24V05, TWO_BYTE_SUBA, (0xefff+19), buf, 1);

	Flag_08_ReturnChangePara = 1;
	Flag_Change_InvContents = 1;
}


//07格式数据包处理函数(处理周期)
void Change_ProCycle(struct ASK_07 *p,uint16 len)
{
//	uint8 buf[2];

//	memcpy(ASK08_data_Modify.ASK08_Modify_65,ASK08_data.ASK08_65,65);
//	ASK08_data_Modify.ASK08_Modify.ProCycle = NewProCycle = p->New_ProCycle;
	NewProCycle = p->New_ProCycle;
//	ProCycle = NewProCycle;
//	buf[0] = NewProCycle;
//==待定
//	i2c_writeNByte(FM24V05, TWO_BYTE_SUBA, (0xefff+20), buf, 1);

	Flag_08_ReturnChangePara = 1;
	Flag_Change_ProCycle = 1;
}


//0B格式数据包处理函数
void Change_DisTime(struct ASK_0B *p,uint16 len)
{
//	uint8 buf[2];

//	memcpy(ASK08_data_Modify.ASK08_Modify_65,ASK08_data.ASK08_65,65);

//	ASK08_data_Modify.ASK08_Modify.DisTime = NewDisTime = p->New_DisTime;
	NewDisTime = p->New_DisTime;

//	DisTime = NewDisTime;
//==待定
//	i2c_writeNByte(FM24V05, TWO_BYTE_SUBA, (0xefff+21), buf, 1);

	Flag_08_ReturnChangePara = 1;
	Flag_Change_DisTime = 1;

}

//用户名、密码校验函数，正确返回1，错误返回0
uint8 UserName_Key(struct ASK_03 *p,uint16 len)
{
	uint8 i,OK=1;
	for(i=0;i<8;i++)
	{
		if(p->UserName[i] != UserName[i])
		{
			OK = 0;
			return OK;
		}
	}
	for(i=0;i<8;i++)
	{
		if(p->KeyNum[i] != KeyNum[i])
		{
			OK = 0;
			return OK;
		}
	}
	return OK;
}

//设备身份识别码校验函数，正确返回1，错误返回0
uint8 RD_ID_Key(struct RD_data *p,uint16 len)
{
	uint8 i,OK=1;
	for(i=0;i<16;i++)
	{
		if(p->RD_ID[i] != RDid[i])
		{
			OK = 0;
			return OK;
		}
	}
	return OK;
}

// 更新02包数据
void Update_data02(void)
{
	uint8 k;

	// 读取02包数据
	// 命令字
	Send_data02[0] = 0x02;
	// 设备身份识别码
	for(k=0;k<16;k++)
	{
	   Send_data02[k+1] = RDid[k];
	}
	//更新02包时间
	Send_data02[17] = 0x01;
}
// 更新08包数据
void Update_data08(void)
{
	uint8 k;

	// 读取08包数据
	// 命令字
	Send_data08[0] = 0x08;
	// 设备身份识别码
	for(k=0;k<16;k++)
	{
	   Send_data08[k+1] = RDid[k];
	}
	//更新08包时间
	Send_data08[17] = YEAR & 0xFF;
	Send_data08[18] = (YEAR>> 8) & 0xFF;
	Send_data08[19] = MONTH;
	Send_data08[20] = DAY;
	Send_data08[21] = HOUR; 
	Send_data08[22] = MIN;
	Send_data08[23] = SEC;
	// 数据处理周期
//	if(0 == Flag_08_ReturnChangePara)
//	{
//		Send_data08[24] = ProCycle;
//	}
//	else
//	{
//		Send_data08[24] = NewProCycle;
//	}

	if(0 == Flag_Change_ProCycle)  //20130519
	{
		Send_data08[24] = ProCycle;
	}
	else
	{
		Send_data08[24] = NewProCycle;
	}

    // 调查内容
//	if(0 == Flag_08_ReturnChangePara)
//	{
//		Send_data08[25] = InvContents;
//	}
//	else
//	{
//		 Send_data08[25] = NewInvContents;
//	}

	if(0 == Flag_Change_InvContents)  //20130519
	{
		Send_data08[25] = InvContents;
	}
	else
	{
		 Send_data08[25] = NewInvContents;
	}
	// 服务器IP
//	if(0 == Flag_08_ReturnChangePara)
//	{
//		Send_data08[26] = 10;
//		Send_data08[27] = 252;
//		Send_data08[28] = 0;
//		Send_data08[29] = 39;
//	}
//	else
//	{
//		Send_data08[26] = NewDscIp[0];
//		Send_data08[27] = NewDscIp[1];
//		Send_data08[28] = NewDscIp[2];
//		Send_data08[29] = NewDscIp[3];
//	}

	if (0 == Flag_Change_DSC_Ip)  //20130519
	{
		Send_data08[26] = 222;
		Send_data08[27] = 222;
		Send_data08[28] = 63;
		Send_data08[29] = 99;	
	}
	else
	{
		Send_data08[26] = NewDscIp[0];
		Send_data08[27] = NewDscIp[1];
		Send_data08[28] = NewDscIp[2];
		Send_data08[29] = NewDscIp[3];	
	}
	// 站点编号
	for(k=0;k<15;k++)
	{
//		if(0 == Flag_08_ReturnChangePara)
//		{
//	   		Send_data08[k+30] = RDNum[k];
//		}
//		else
//		{
//			Send_data08[k+30] = NewRDNum[k];
//		}

		if(0 == Flag_Change_RD_Num)		   //20130519
		{
			 Send_data08[k+30] = RDNum[k];
		}
		else
		{
			 Send_data08[k+30] = NewRDNum[k];
		}
	}
	// 鉴别时间
//	if(0 == Flag_08_ReturnChangePara)
//	{
//		Send_data08[45] = DisTime;
//	}
//	else
//	{
//		Send_data08[45] = NewDisTime;
//	}

	if(0 == Flag_Change_DisTime)	 //20130519
	{
		Send_data08[45] = DisTime;
	}
	else
	{
		Send_data08[45] = NewDisTime;
	}
	// 传输方式
	Send_data08[46] = ConnectType;
	// 经度坐标
	for(k=0;k<8;k++)
	{
	//	Send_data08[k+47] = jingdu_buf[k];
	}
	// 纬度
	for(k=0;k<8;k++)
	{
	//	Send_data08[k+55] = weidu_buf[k];
	}
	// 高程
	Send_data08[63] = Elevation&0xFF; 
	Send_data08[64] = (Elevation>>8)&0xFF;
}
//生成01数据包，//并保存在SD卡中。


int32   Save_data_01_process(void)
{
	  uint8 i=0;
	  uint8 j=0;
	  uint8   frame[512];
	  uint8   lane_num=g_sspSetup.u8LaneNum;
	  uint8 stat;
	  uint16  len;
//	  uint32  address;//写入SD卡的地址；
	  uint16 TimeNum_temp;//时间序号，从1-1440
	  uint32 SD_Num_time;
	//  TimeNum_temp = (HOUR+1) * 60 + MIN;
	  TimeNum_temp = (TEMP_HOUR * 60 + TEMP_MIN+1) / ProCycle;     // 修改时间序号为 1-288  20131216

			len = 0;
			frame[len++] = 0x01;
			//设备身份识别码
			for(i=0;i<16;i++)
			{
			    frame[len++] = RDid[i];	
			}
			//站点编号
			for(i=0;i<15;i++)
			{
			    frame[len++] = RDNum[i];		
			}
			//设备硬件错误代码
			frame[len++] = 0x00;
			//调查内容
			frame[len++] = 0x01;
			
			//年
			frame[len++] = TEMP_YEAR;
			frame[len++] = (TEMP_YEAR>>8);
			//月
			frame[len++] = TEMP_MONTH;
			//日
			frame[len++] = TEMP_DAY;
			
			//交通数据处理周期
			frame[len++] = ProCycle;
			//时间序号 低位在前
			frame[len++] = TimeNum_temp&0xff;
			frame[len++] = (TimeNum_temp>>8)&0xff;

			//车道数
			frame[len++] = lane_num;  //42字节
		
			for(i=0;i<lane_num;i++)
			{	
				// 将车道号转换成国标车道号
		 	  	frame[len++] = ChangetoBG_chedao_num(i);  
				
				//跟车百分比
				frame[len++] = g_percent_genche[i];
			
				//平均车头间距
				frame[len++] = g_average_jianju[i]&0xff;
				frame[len++] = (g_average_jianju[i]>>8)&0xff;
			
				//时间占有率
				frame[len++] = g_sum_shijian_share[i];	//时间占有率尚未求；
				//uint32 g_total_veh[4][9]={0}; 
				//***九车型总数；小客车	小货车	大客车	中型货车	大型货车	特大型货车	集装箱车	拖拉机	摩托车
				//六车型（二类协议）：小型车， 中型车，大型车，特大型车，拖拉机，摩托车
				//uint32 g_speed_veh_sum[4][9]={50,50,50,50,50,50,50,50,50};
				//中小客交通量
				for(j=0;j<6;j++)
				{
					frame[len++] = (g_total_veh[i][j])&0xff;  //交通量	 //低位在前
					frame[len++] = ((g_total_veh[i][j])>>8)&0xff; 
					frame[len++] = g_speed_veh_sum[i][j] / g_total_veh[i][j];	 //平均地点车速
				} //6*3=18
			}
			//总字节数161,0xAA。
			frame[510] = (len>>8);//高位
			frame[511] = len;	//低位	
			memcpy(Send_data01_temp,frame,161);//170
		    SD_Num_time	= (Get_NUM_From_Time(g_sstCurTime) - 325440) / 5;
			SD_address_01 = SD_Num_time + SD_Base_address_01;//当前01包存储的SD地址；

/*********将以上变量重新初始化***********/		
		 memset(g_total_veh,0,sizeof(g_total_veh));
		 memset(g_speed_veh_sum,0,sizeof(g_speed_veh_sum));
		 memset(g_total_Lane,0,sizeof(g_total_Lane));
		 memset(g_average_shiju,0,sizeof(g_average_shiju));
		 memset(g_sum_shiju,0,sizeof(g_sum_shiju));
		 memset(g_sum_shijian_share,0,sizeof(g_sum_shijian_share));
		 memset(g_sum_shijian,0,sizeof(g_sum_shijian));
		 memset(g_total_genche,0,sizeof(g_total_genche));
		 memset(g_percent_genche,0,sizeof(g_percent_genche));		 
		 memset(g_average_jianju,0,sizeof(g_average_jianju));
/*********重新初始化***********/	
		 return len;
}

uint8  ChangetoBG_chedao_num( uint8 ln )
{
	  uint8 BG_chedao_num;

	  if (SETUPALIAS.u8LaneNum == 4)	//4车道
	  {
		   if(g_u8LaneDir == 0xBB)    //小号车道 下行 
		   {
			   switch(ln)
			   {
			   	  case 0:
						BG_chedao_num = 31;
						break;
				  case 1:
						BG_chedao_num = 32;
						break;
				  case 2:
						BG_chedao_num = 11;
						break;
				  case 3:
						BG_chedao_num = 12;
						break;
			   }
		   }
		   else	     //其他情况 小号车道  上行
		   {
			   switch(ln)
			   {
			   	  case 0:
						BG_chedao_num = 11;
						break;
				  case 1:
						BG_chedao_num = 12;
						break;
				  case 2:
						BG_chedao_num = 31;
						break;
				  case 3:
						BG_chedao_num = 32;
						break;
			   }		   
		   }
	   }
	   else		//6车道
	   {
		   if(g_u8LaneDir == 0xBB)    //小号车道 下行 
		   {		   
			   switch(ln)
			   {
			   	  case 0:
						BG_chedao_num = 31;
						break;
				  case 1:
						BG_chedao_num = 32;
						break;
				  case 2:
						BG_chedao_num = 33;
						break;
				  case 3:
						BG_chedao_num = 11;
						break;
				  case 4:
				  		BG_chedao_num = 12;
				  		break;
				  case 5:
				  		BG_chedao_num = 13;
				  		break;
			   }
			}
			else
			{
			   switch(ln)
			   {
			   	  case 0:
						BG_chedao_num = 11;
						break;
				  case 1:
						BG_chedao_num = 12;
						break;
				  case 2:
						BG_chedao_num = 13;
						break;
				  case 3:
						BG_chedao_num = 31;
						break;
				  case 4:
				  		BG_chedao_num = 32;
				  		break;
				  case 5:
				  		BG_chedao_num = 33;
				  		break;
			   }			
			}
	   }
	   return 	BG_chedao_num;
}

void JudgeQual(uint8 Qual, uint8 *p)
{
	uint8	tempQual = 0;

	tempQual = Qual;


	switch(tempQual)
	{
		case WJ_CMD_DEVICERESETINFO:   //查询设备重启信息
			SendDevResetInfo();
			break;
		case WJ_CMD_DEVICEPARAM:	   //查询设备配置参数
			SendDevParamInfo();
			break;
		case WJ_CMD_DEVICEOTHERS:	   //查询设备其他情况
			SendDevOtherInfo();
			break;
		case WJ_CMD_CLEARRESETIFNOBUF://清除重启信息
			ClearResetInfoBuf();
			break;
		case WJ_CMD_QUERYPARAM:	  //为参数查询
			SendDevParamInfor2();		
			break;
		case WJ_CMD_SETPARAM:	  //为参数设置
			SetDevParamInfor(p);
			break;
		case WJ_CMD_GETFRAME:	  //获取单帧数据
			Sendframe();
			break;
		case WJ_CMD_GETFRAME2:	  //获取单帧数据
			Sendframe2();
			break;
		case WJ_CMD_GETDEVICETIME:   //获取控制器时间
			GetDeviceTime();
			break;
		case WJ_CMD_SETDEVICETIME:   //设置控制器时间
			SetDeviceTime(p);
			break;
		case WJ_CMD_GETRDID:    //获取设备识别码
			GetRDID();
			break;
		case WJ_CMD_SETRDID:    //设置设备识别码
			SetRDID(p);
			break;
		case WJ_CMD_INITALLPARAM:    //初始化参数
			InitAllParam();
			break;
		case WJ_CMD_RESETDEVICE:    //重启控制器
			ResetDevice();
			break;
		case WJ_CMD_GETLANEDIR:		//获取上下行方向
			GetLaneDir();
			break;
		case WJ_CMD_SETLANEDIR:		//设置上下行方向
			SetLaneDir(p);
			break;
		case WJ_CMD_GETSD01DATA:		//获取SD卡01包数据
//			GetSD01Data(p);
			break;
		case WJ_CMD_GET_THRESHOLD:   //获取阈值
//			GetVehThreshold();
			break;
		case WJ_CMD_SET_THRESHOLD:   //设置阈值
			//SetVehThreshold(p);
			break;
		default :
			break;
	}
		
}


static void SendDevResetInfo(void)
{
	uint8	i = 0;
	uint8	j = 0;
	uint8	u8TmpPos = 0;
	uint8	u8StartPos = 0;
	uint8	u8TmpEntries = 0;
	uint8	u8tempBuf[100] = {0};
	uint16	checksum = 0;
	uint8	u8tempIndex = 0;
	uint32	u32TmpResetTimes = 0;
	CycleBufferStruct  *pTmpResetCycBuf = (CycleBufferStruct *)malloc(sizeof(CycleBufferStruct) + RESETINFO_BUFFERSIZE * sizeof(SystemTime));

	u32TmpResetTimes = g_sspSetup.resetCnt;
	Read256_full(RESETINFOADDR, (uint8 *)pTmpResetCycBuf, sizeof(CycleBufferStruct) + RESETINFO_BUFFERSIZE * sizeof(SystemTime));

	u8tempIndex = 0;
	u8tempBuf[u8tempIndex++] = 0xFF;
	u8tempBuf[u8tempIndex++] = 0xAA;
	u8tempBuf[u8tempIndex++] = WJ_CMD_DEVICERESETINFO;			

	u8tempIndex++;	 	//u8tempBuf[u8tempIndex++] = ;
	u8tempBuf[u8tempIndex++] = RDid[0];
	u8tempBuf[u8tempIndex++] = RDid[1];
	//复位次数
	u8tempBuf[u8tempIndex++] = (u32TmpResetTimes>>24) & 0xFF;
	u8tempBuf[u8tempIndex++] = (u32TmpResetTimes>>16) & 0xFF;
	u8tempBuf[u8tempIndex++] = (u32TmpResetTimes>>8) & 0xFF;
	u8tempBuf[u8tempIndex++] = u32TmpResetTimes & 0xFF;

	//复位时间
	u8TmpEntries = pTmpResetCycBuf->u8CurrentEntries;
	u8TmpPos = pTmpResetCycBuf->u8CurrentPos;
	u8StartPos = (RESETINFO_BUFFERSIZE + u8TmpPos - u8TmpEntries)%RESETINFO_BUFFERSIZE;

	for(i=u8StartPos,j=0; j<u8TmpEntries; i++,j++)	//20130703修改
	{
		i = i % RESETINFO_BUFFERSIZE;
		u8tempBuf[u8tempIndex++] = b2bcd((pTmpResetCycBuf->SysTimeTable[i].u16Year - 2000)&0xFF);
		u8tempBuf[u8tempIndex++] = b2bcd(pTmpResetCycBuf->SysTimeTable[i].u8Month);
		u8tempBuf[u8tempIndex++] = b2bcd(pTmpResetCycBuf->SysTimeTable[i].u8Day);
		u8tempBuf[u8tempIndex++] = b2bcd(pTmpResetCycBuf->SysTimeTable[i].u8Week);
		u8tempBuf[u8tempIndex++] = b2bcd(pTmpResetCycBuf->SysTimeTable[i].u8Hour);
		u8tempBuf[u8tempIndex++] = b2bcd(pTmpResetCycBuf->SysTimeTable[i].u8Minute);
		u8tempBuf[u8tempIndex++] = b2bcd(pTmpResetCycBuf->SysTimeTable[i].u8Second);
	}

	u8tempBuf[3] = u8tempIndex - 4;			  //长度
	checksum = CRCSum(u8tempBuf+4, u8tempIndex-4);
	u8tempBuf[u8tempIndex++] = (checksum>>8)&0xFF;
	u8tempBuf[u8tempIndex++] = checksum&0xFF;
	UART1_SendBuf_full(u8tempBuf, u8tempIndex);
	free(pTmpResetCycBuf);	
}



static void SendDevParamInfo(void)
{
	uint8	u8tempBuf[100] = {0};
	uint16	checksum = 0, i = 0;
	uint8	u8tempIndex = 0;
	SetupParam	TmpDevSetParam;

	memset(&TmpDevSetParam, 0, sizeof(TmpDevSetParam));
	Read256_full(BUF0ADDR,(uint8 *)&TmpDevSetParam, sizeof(TmpDevSetParam));

	u8tempIndex = 0;
	u8tempBuf[u8tempIndex++] = 0xFF;
	u8tempBuf[u8tempIndex++] = 0xAA;
	u8tempBuf[u8tempIndex++] = WJ_CMD_DEVICEPARAM;

	u8tempIndex++;	 	//u8tempBuf[u8tempIndex++] = ;
	u8tempBuf[u8tempIndex++] = RDid[0];			//4-5:
	u8tempBuf[u8tempIndex++] = RDid[1];

	u8tempBuf[u8tempIndex++] = TmpDevSetParam.u8BaudRate;//6；波特率
	u8tempBuf[u8tempIndex++] = TmpDevSetParam.u8DOG;	 //7
	u8tempBuf[u8tempIndex++] = TmpDevSetParam.u8TrafficType; //-8
	u8tempBuf[u8tempIndex++] = (TmpDevSetParam.HeightLaser0>>8)&0xFF; //9-10
	u8tempBuf[u8tempIndex++] = TmpDevSetParam.HeightLaser0 & 0xFF;
	u8tempBuf[u8tempIndex++] = (TmpDevSetParam.HeightLaser1>>8)&0xFF; //9-10
	u8tempBuf[u8tempIndex++] = TmpDevSetParam.HeightLaser1 & 0xFF;
	u8tempBuf[u8tempIndex++] = (TmpDevSetParam.HeightLaser2>>8)&0xFF; //9-10
	u8tempBuf[u8tempIndex++] = TmpDevSetParam.HeightLaser2 & 0xFF;
	u8tempBuf[u8tempIndex++] = (TmpDevSetParam.HeightLaser3>>8)&0xFF; //9-10
	u8tempBuf[u8tempIndex++] = TmpDevSetParam.HeightLaser3 & 0xFF;
//	u8tempBuf[u8tempIndex++] = (TmpDevSetParam.IncHeightLaser>>8) & 0xFF;//
//	u8tempBuf[u8tempIndex++] = TmpDevSetParam.IncHeightLaser & 0xFF;
	u8tempBuf[u8tempIndex++] = (TmpDevSetParam.LaserDistance>>8)&0xFF;//13-14
	u8tempBuf[u8tempIndex++] = TmpDevSetParam.LaserDistance & 0xFF;
	u8tempBuf[u8tempIndex++] = (TmpDevSetParam.n32LaserHorizOff>>8)&0xFF;//15-16
	u8tempBuf[u8tempIndex++] = TmpDevSetParam.n32LaserHorizOff & 0xFF;
	u8tempBuf[u8tempIndex++] = (TmpDevSetParam.u32LaserRoadAngle>>8)&0xFF;		//17-18		 //原寻高起始点，现激光扫描截面偏角
	u8tempBuf[u8tempIndex++] = TmpDevSetParam.u32LaserRoadAngle & 0xFF;
	u8tempBuf[u8tempIndex++] = (TmpDevSetParam.u16VerticalZeroPos0>>8)&0xFF;//19-20
	u8tempBuf[u8tempIndex++] = TmpDevSetParam.u16VerticalZeroPos0 & 0xFF;
	u8tempBuf[u8tempIndex++] = (TmpDevSetParam.u16VerticalZeroPos1>>8)&0xFF;//19-20
	u8tempBuf[u8tempIndex++] = TmpDevSetParam.u16VerticalZeroPos1 & 0xFF;
	u8tempBuf[u8tempIndex++] = (TmpDevSetParam.u16VerticalZeroPos2>>8)&0xFF;//19-20
	u8tempBuf[u8tempIndex++] = TmpDevSetParam.u16VerticalZeroPos2 & 0xFF;
	u8tempBuf[u8tempIndex++] = (TmpDevSetParam.u16VerticalZeroPos3>>8)&0xFF;//19-20
	u8tempBuf[u8tempIndex++] = TmpDevSetParam.u16VerticalZeroPos3 & 0xFF;
//	u8tempBuf[u8tempIndex++] = (TmpDevSetParam.u16InclineZeroPos>>8)&0xFF;//21-22
//	u8tempBuf[u8tempIndex++] = TmpDevSetParam.u16InclineZeroPos & 0xFF;
	u8tempBuf[u8tempIndex++] = (TmpDevSetParam.u16StartPtNum0>>8)&0xFF;// 23-24
	u8tempBuf[u8tempIndex++] = TmpDevSetParam.u16StartPtNum0 & 0xFF;
	u8tempBuf[u8tempIndex++] = (TmpDevSetParam.u16EndPtNum0>>8)&0xFF;  //25-26
	u8tempBuf[u8tempIndex++] = TmpDevSetParam.u16EndPtNum0 & 0xFF;
	u8tempBuf[u8tempIndex++] = (TmpDevSetParam.u16StartPtNum1>>8)&0xFF;// 23-24
	u8tempBuf[u8tempIndex++] = TmpDevSetParam.u16StartPtNum1 & 0xFF;
	u8tempBuf[u8tempIndex++] = (TmpDevSetParam.u16EndPtNum1>>8)&0xFF;  //25-26
	u8tempBuf[u8tempIndex++] = TmpDevSetParam.u16EndPtNum1 & 0xFF;
	u8tempBuf[u8tempIndex++] = (TmpDevSetParam.u16StartPtNum2>>8)&0xFF;// 23-24
	u8tempBuf[u8tempIndex++] = TmpDevSetParam.u16StartPtNum2 & 0xFF;
	u8tempBuf[u8tempIndex++] = (TmpDevSetParam.u16EndPtNum2>>8)&0xFF;  //25-26
	u8tempBuf[u8tempIndex++] = TmpDevSetParam.u16EndPtNum2 & 0xFF;
	u8tempBuf[u8tempIndex++] = (TmpDevSetParam.u16StartPtNum3>>8)&0xFF;// 23-24
	u8tempBuf[u8tempIndex++] = TmpDevSetParam.u16StartPtNum3 & 0xFF;
	u8tempBuf[u8tempIndex++] = (TmpDevSetParam.u16EndPtNum3>>8)&0xFF;  //25-26
	u8tempBuf[u8tempIndex++] = TmpDevSetParam.u16EndPtNum3 & 0xFF;
	u8tempBuf[u8tempIndex++] = (TmpDevSetParam.MedianLeftWide>>8)&0xFF;//27-28
	u8tempBuf[u8tempIndex++] = TmpDevSetParam.MedianLeftWide & 0xFF;
	u8tempBuf[u8tempIndex++] = (TmpDevSetParam.MedianWide>>8)&0xFF;//29-30	 //原隔离带右侧宽度，现隔离带宽度
	u8tempBuf[u8tempIndex++] = TmpDevSetParam.MedianWide & 0xFF;
	u8tempBuf[u8tempIndex++] = 0;//31:车道类型(默认高速)	
	u8tempBuf[u8tempIndex++] = 6;//32:车道数
	u8tempBuf[u8tempIndex++] = (TmpDevSetParam.LaneWide>>8)&0xFF;  //33-34
	u8tempBuf[u8tempIndex++] = TmpDevSetParam.LaneWide & 0xFF;
	//IP
	u8tempBuf[u8tempIndex++] = (TmpDevSetParam.u32LocalIPAddress>>24) & 0xFF;
	u8tempBuf[u8tempIndex++] = (TmpDevSetParam.u32LocalIPAddress>>16) & 0xFF;
	u8tempBuf[u8tempIndex++] = (TmpDevSetParam.u32LocalIPAddress>>8) & 0xFF;
	u8tempBuf[u8tempIndex++] = TmpDevSetParam.u32LocalIPAddress & 0xFF;
	//Port
	u8tempBuf[u8tempIndex++] = (TmpDevSetParam.u32LocalPortNO>>8) & 0xFF;
	u8tempBuf[u8tempIndex++] = TmpDevSetParam.u32LocalPortNO & 0xFF;
	
	u8tempBuf[3] = u8tempIndex - 4;			  //长度   20130703
	checksum = CRCSum(u8tempBuf+4, u8tempIndex-4);
	u8tempBuf[u8tempIndex++] = (checksum>>8)&0xFF;
	u8tempBuf[u8tempIndex++] = checksum&0xFF;
	UART1_SendBuf_full(u8tempBuf, u8tempIndex);
}

void SendDevParamInfor2(void)
{
	uint16	checksum = 0;
	uint8	tempindex = 0;
	uint8	l_u8PCProtocolBuf[150] = {0};
	l_u8PCProtocolBuf[tempindex++] = 0xFF;
	l_u8PCProtocolBuf[tempindex++] = 0xAA;
	l_u8PCProtocolBuf[tempindex++] = WJ_CMD_QUERYPARAM;
	tempindex++;	 	//u8tempBuf[u8tempIndex++] = ;
	memcpy(l_u8PCProtocolBuf+tempindex,SETUPALIAS.au8ProgramVersion,11);		  //版本号
	tempindex += 11;
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u8InstallFlag;         //安装方式 0 侧桩 1 正桩


	Read256_full(BUF0ADDR,(uint8 *)&SETUPALIAS, sizeof(SETUPALIAS));

	
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u8BaudRate;
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u8DOG;
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.HeightLaser0>>24;	   //激光器垂直高度值
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.HeightLaser0>>16)&0xFF;
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.HeightLaser0>>8)&0xFF;                           	  	  
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.HeightLaser0&0xFF;
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.HeightLaser1>>24;	   //激光器垂直高度值
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.HeightLaser1>>16)&0xFF;
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.HeightLaser1>>8)&0xFF;                           	  	  
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.HeightLaser1&0xFF;
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.HeightLaser2>>24;	   //激光器垂直高度值
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.HeightLaser2>>16)&0xFF;
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.HeightLaser2>>8)&0xFF;                           	  	  
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.HeightLaser2&0xFF;
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.HeightLaser3>>24;	   //激光器垂直高度值
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.HeightLaser3>>16)&0xFF;
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.HeightLaser3>>8)&0xFF;                           	  	  
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.HeightLaser3&0xFF;
//	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.IncHeightLaser>>24;		
//	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.IncHeightLaser>>16)&0xFF;
//	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.IncHeightLaser>>8)&0xFF;                              //%车道宽度
//	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.IncHeightLaser&0xFF;

	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.LaserDistance>>24;
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.LaserDistance>>16)&0xFF;
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.LaserDistance>>8)&0xFF;    
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.LaserDistance&0xFF;
	
//	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.Angle12>>24;
//	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.Angle12>>16)&0xFF;
//	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.Angle12>>8)&0xFF;    
//	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.Angle12&0xFF;

	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.LaneWide>>24;
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.LaneWide>>16)&0xFF;
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.LaneWide>>8)&0xFF;    
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.LaneWide&0xFF;

	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.MedianWide>>24;		 //隔离带宽度，原隔离带右边宽度
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.MedianWide>>16)&0xFF;
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.MedianWide>>8)&0xFF;    
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.MedianWide&0xFF;

	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.MedianLeftWide>>24;
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.MedianLeftWide>>16)&0xFF;
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.MedianLeftWide>>8)&0xFF;    
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.MedianLeftWide&0xFF;

//			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.VdirAngle>>24;	   //垂直偏角
//			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.VdirAngle>>16)&0xFF;
//			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.VdirAngle>>8)&0xFF;    
//			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.VdirAngle&0xFF;
//
//			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.IdirAngle>>24;	    //倾斜偏角
//			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.IdirAngle>>16)&0xFF;
//			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.IdirAngle>>8)&0xFF;    
//			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.IdirAngle&0xFF;

	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.resetCnt>>24;
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.resetCnt>>16)&0xFF;
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.resetCnt>>8)&0xFF;    
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.resetCnt&0xFF;

	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32LaserRoadAngle>>24;   //激光扫描截面偏角，（原寻高起始点）
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32LaserRoadAngle>>16)&0xFF;
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32LaserRoadAngle>>8)&0xFF;    
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32LaserRoadAngle&0xFF;
	
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.VerticalLaser_IP>>24;
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.VerticalLaser_IP>>16) & 0xFF;
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.VerticalLaser_IP>>8) & 0xFF;    
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.VerticalLaser_IP & 0xFF;					
	
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.VerticalLaser_Port>>24;
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.VerticalLaser_Port>>16) & 0xFF;
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.VerticalLaser_Port>>8) & 0xFF;    
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.VerticalLaser_Port & 0xFF;
	
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.ParallerLaser_IP1>>24;
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.ParallerLaser_IP1>>16) & 0xFF;
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.ParallerLaser_IP1>>8) & 0xFF;    
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.ParallerLaser_IP1 & 0xFF;					
	
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.ParallerLaser_Port1>>24;
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.ParallerLaser_Port1>>16) & 0xFF;
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.ParallerLaser_Port1>>8) & 0xFF;    
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.ParallerLaser_Port1 & 0xFF;
//2014-12-04
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.ParallerLaser_IP2>>24;
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.ParallerLaser_IP2>>16) & 0xFF;
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.ParallerLaser_IP2>>8) & 0xFF;    
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.ParallerLaser_IP2 & 0xFF;					
	
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.ParallerLaser_Port2>>24;
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.ParallerLaser_Port2>>16) & 0xFF;
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.ParallerLaser_Port2>>8) & 0xFF;    
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.ParallerLaser_Port2 & 0xFF;

		l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.ParallerLaser_IP3>>24;
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.ParallerLaser_IP3>>16) & 0xFF;
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.ParallerLaser_IP3>>8) & 0xFF;    
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.ParallerLaser_IP3 & 0xFF;					
	
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.ParallerLaser_Port3>>24;
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.ParallerLaser_Port3>>16) & 0xFF;
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.ParallerLaser_Port3>>8) & 0xFF;    
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.ParallerLaser_Port3 & 0xFF;
//2014-12-04
//控制器ip参数					
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32LocalIPAddress>>24;
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32LocalIPAddress>>16) & 0xFF;
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32LocalIPAddress>>8) & 0xFF;    
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32LocalIPAddress & 0xFF;
	
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32SubMask>>24;
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32SubMask>>16) & 0xFF;
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32SubMask>>8) & 0xFF;    
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32SubMask & 0xFF;								
	
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32LocalPortNO>>24;
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32LocalPortNO>>16) & 0xFF;
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32LocalPortNO>>8) & 0xFF;    
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32LocalPortNO & 0xFF;					
//网关
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32GatewayIP>>24;
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32GatewayIP>>16) & 0xFF;
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32GatewayIP>>8) & 0xFF;    
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32GatewayIP & 0xFF;
	
//	au8LocalMAC
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.au8LocalMAC[0];
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.au8LocalMAC[1];
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.au8LocalMAC[2];    
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.au8LocalMAC[3];					
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.au8LocalMAC[4];
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.au8LocalMAC[5];																								

//20130424 	1:		
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32Net1_DisconnectNum>>24;
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32Net1_DisconnectNum>>16) & 0xFF;
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32Net1_DisconnectNum>>8) & 0xFF;    
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32Net1_DisconnectNum & 0xFF;
	//2:
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32Net2_DisconnectNum>>24;
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32Net2_DisconnectNum>>16) & 0xFF;
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32Net2_DisconnectNum>>8) & 0xFF;    
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32Net2_DisconnectNum & 0xFF;
	//3:
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32Net1_InvalidRecNum>>24;
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32Net1_InvalidRecNum>>16) & 0xFF;
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32Net1_InvalidRecNum>>8) & 0xFF;    
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32Net1_InvalidRecNum & 0xFF;			
	//4:
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32Net2_InvalidRecNum>>24;
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32Net2_InvalidRecNum>>16) & 0xFF;
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32Net2_InvalidRecNum>>8) & 0xFF;    
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32Net2_InvalidRecNum & 0xFF;			
	//5:
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32DataProcException>>24;	  //处理异常码
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32DataProcException>>16) & 0xFF;
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32DataProcException>>8) & 0xFF;    
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32DataProcException & 0xFF;			
	

	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.n32LaserHorizOff>>24;		  //激光器与行车道距离，原激光器水平偏移
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.n32LaserHorizOff>>16) & 0xFF;
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.n32LaserHorizOff>>8) & 0xFF;    
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.n32LaserHorizOff & 0xFF;
// 垂直
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u16VerticalZeroPos0>>8) & 0xFF;    
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u16VerticalZeroPos0 & 0xFF;
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u16VerticalZeroPos1>>8) & 0xFF;    
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u16VerticalZeroPos1 & 0xFF;
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u16VerticalZeroPos2>>8) & 0xFF;    
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u16VerticalZeroPos2 & 0xFF;
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u16VerticalZeroPos3>>8) & 0xFF;    
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u16VerticalZeroPos3 & 0xFF;			
////倾斜			
//	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u16InclineZeroPos>>8) & 0xFF;    
//	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u16InclineZeroPos & 0xFF;
//起始点			
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u16StartPtNum0>>8) & 0xFF;    
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u16StartPtNum0 & 0xFF;						
//终止点			
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u16EndPtNum0>>8) & 0xFF;    
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u16EndPtNum0 & 0xFF;

	//起始点			
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u16StartPtNum1>>8) & 0xFF;    
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u16StartPtNum1 & 0xFF;						
//终止点			
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u16EndPtNum1>>8) & 0xFF;    
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u16EndPtNum1 & 0xFF;
	//起始点			
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u16StartPtNum2>>8) & 0xFF;    
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u16StartPtNum2 & 0xFF;						
//终止点			
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u16EndPtNum2>>8) & 0xFF;    
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u16EndPtNum2 & 0xFF;
	//起始点			
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u16StartPtNum3>>8) & 0xFF;    
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u16StartPtNum3 & 0xFF;						
//终止点			
	l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u16EndPtNum3>>8) & 0xFF;    
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u16EndPtNum3 & 0xFF;
	
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u8LaserDevType;	 //激光类型 施克 万集
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u8TrafficType;	 //交调 车检器
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u8RoadType;	    // 道路类型 0-- 国道 1-- 高速
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u8LaneNum;	    //车道数 0-- 4车道 1-- 6车道
	l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u8NetType;		//网络连接：0 -- 无线 1 -- 有线

	l_u8PCProtocolBuf[3] = tempindex - 4;			  //长度   20130703
	checksum = CRCSum(l_u8PCProtocolBuf+4, tempindex-4);
	l_u8PCProtocolBuf[tempindex++] = (checksum>>8)&0xFF;
	l_u8PCProtocolBuf[tempindex++] = checksum&0xFF;
	UART1_SendBuf_full(l_u8PCProtocolBuf, tempindex);
}
 void Sendframe()
 {
 
 	uint16	checksum = 0,tempindex = 0;
	uint32	 count = 0, i = 0;  //count是发送的包数 =  831/（0xff-2个帧头，2个帧尾，1个命令号，1个长度，2个RDid，1个帧序号（index = 6） ）
	uint8	l_u8PCProtocolBuf[2048] = {0};
//	uint8  JG_Buff[831]={0};
//		memcpy(JG_Buff,g_au8Two_Buff[g_u32cout_Pro_Two_Buff],831);
	l_u8PCProtocolBuf[tempindex++] = 0xFF;
	l_u8PCProtocolBuf[tempindex++] = 0xAA;
	l_u8PCProtocolBuf[tempindex++] = WJ_CMD_GETFRAME;
	l_u8PCProtocolBuf[tempindex++] = 0x03;	//	 0x33f = 831
	l_u8PCProtocolBuf[tempindex++] = 0x3f;
	l_u8PCProtocolBuf[tempindex++] = RDid[0];			//5`6
	l_u8PCProtocolBuf[tempindex++] = RDid[1];
#ifdef SIM_SOFTWARE	  //接收模拟软件版
	l_u8PCProtocolBuf[0+7] = 2;
	l_u8PCProtocolBuf[1+7] = 2;
	l_u8PCProtocolBuf[2+7] = 2;
	l_u8PCProtocolBuf[3+7] = 2;
    l_u8PCProtocolBuf[42+7]=	g_au8Two_Buff[g_u32cout_Pro_Two_Buff][8];
	l_u8PCProtocolBuf[43+7]=	g_au8Two_Buff[g_u32cout_Pro_Two_Buff][9];
	l_u8PCProtocolBuf[44+7]=	g_au8Two_Buff[g_u32cout_Pro_Two_Buff][10];
	l_u8PCProtocolBuf[45+7]=	g_au8Two_Buff[g_u32cout_Pro_Two_Buff][11];
	l_u8PCProtocolBuf[83+7]=   0x01;	//(p[83]<<8)+p[84];
	l_u8PCProtocolBuf[84+7]=	0x69;
	for(i=85;i<=806;i++)
	{
		l_u8PCProtocolBuf[i+7]= g_au8Two_Buff[g_u32cout_Pro_Two_Buff][i-71];
	}
#else		 //现场版
	memcpy(l_u8PCProtocolBuf+7,g_au8Two_Buff[g_u32cout_Pro_Two_Buff],831);
#endif	
	 tempindex+=831; 
	 checksum = CRCSum(l_u8PCProtocolBuf+5, tempindex-5);
	l_u8PCProtocolBuf[tempindex++] = (checksum>>8)&0xFF;
	l_u8PCProtocolBuf[tempindex++] = checksum&0xFF;
	UART1_SendBuf_full(l_u8PCProtocolBuf, tempindex);
//	 for(count = 0; count < 4 ; count++ )
//	 {
//	 	l_u8PCProtocolBuf[6] = count;
//		tempindex = 7;
//		for(i = 0; i< 246,count*246+i<831; i++)
//		{
//		   l_u8PCProtocolBuf[tempindex++] =  JG_Buff[count*246+i ];
//		}
//		l_u8PCProtocolBuf[3] = tempindex - 4;			  //长度   20130703
//		checksum = CRCSum(l_u8PCProtocolBuf+4, tempindex-4);
//		l_u8PCProtocolBuf[tempindex++] = (checksum>>8)&0xFF;
//		l_u8PCProtocolBuf[tempindex++] = checksum&0xFF;
//		UART1_SendBuf(l_u8PCProtocolBuf, tempindex);
//	 }
 }
  void Sendframe2()
 {
 
 	uint16	checksum = 0,tempindex = 0;
	uint32	 count = 0, i = 0;  //count是发送的包数 =  831/（0xff-2个帧头，2个帧尾，1个命令号，1个长度，2个RDid，1个帧序号（index = 6） ）
	uint8	l_u8PCProtocolBuf[2048] = {0};
//	uint8  JG_Buff[831]={0};
//		memcpy(JG_Buff,g_au8Two_Buff[g_u32cout_Pro_Two_Buff],831);
	l_u8PCProtocolBuf[tempindex++] = 0xFF;
	l_u8PCProtocolBuf[tempindex++] = 0xAA;
	l_u8PCProtocolBuf[tempindex++] = WJ_CMD_GETFRAME2;
	l_u8PCProtocolBuf[tempindex++] = 0x03;	//	 0x33f = 831
	l_u8PCProtocolBuf[tempindex++] = 0x3f;
	l_u8PCProtocolBuf[tempindex++] = RDid[0];			//5`6
	l_u8PCProtocolBuf[tempindex++] = RDid[1];
#ifdef SIM_SOFTWARE	  //接收模拟软件版
	l_u8PCProtocolBuf[0+7] = 2;
	l_u8PCProtocolBuf[1+7] = 2;
	l_u8PCProtocolBuf[2+7] = 2;
	l_u8PCProtocolBuf[3+7] = 2;
    l_u8PCProtocolBuf[42+7]=	g_au8Two_Buff[g_u32cout_Pro_Two_Buff][8];
	l_u8PCProtocolBuf[43+7]=	g_au8Two_Buff[g_u32cout_Pro_Two_Buff][9];
	l_u8PCProtocolBuf[44+7]=	g_au8Two_Buff[g_u32cout_Pro_Two_Buff][10];
	l_u8PCProtocolBuf[45+7]=	g_au8Two_Buff[g_u32cout_Pro_Two_Buff][11];
	l_u8PCProtocolBuf[83+7]=   0x01;	//(p[83]<<8)+p[84];
	l_u8PCProtocolBuf[84+7]=	0x69;
	for(i=85;i<=806;i++)
	{
		l_u8PCProtocolBuf[i+7]= g_au8Two_Buff[g_u32cout_Pro_Two_Buff][i+651];
	}
#else		 //现场版
	memcpy(l_u8PCProtocolBuf+7,g_au8Two_Buff[g_u32cout_Pro_Two_Buff]+831,831);
#endif
	 tempindex+=831; 
	 checksum = CRCSum(l_u8PCProtocolBuf+5, tempindex-5);
	l_u8PCProtocolBuf[tempindex++] = (checksum>>8)&0xFF;
	l_u8PCProtocolBuf[tempindex++] = checksum&0xFF;
	UART1_SendBuf_full(l_u8PCProtocolBuf, tempindex);
//	 for(count = 0; count < 4 ; count++ )
//	 {
//	 	l_u8PCProtocolBuf[6] = count;
//		tempindex = 7;
//		for(i = 0; i< 246,count*246+i<831; i++)
//		{
//		   l_u8PCProtocolBuf[tempindex++] =  JG_Buff[count*246+i ];
//		}
//		l_u8PCProtocolBuf[3] = tempindex - 4;			  //长度   20130703
//		checksum = CRCSum(l_u8PCProtocolBuf+4, tempindex-4);
//		l_u8PCProtocolBuf[tempindex++] = (checksum>>8)&0xFF;
//		l_u8PCProtocolBuf[tempindex++] = checksum&0xFF;
//		UART1_SendBuf(l_u8PCProtocolBuf, tempindex);
//	 }
 }
void SetDevParamInfor(uint8 *pUartDataBuf)
{
	uint8   ret = 0;
	uint16	checksum = 0;
	uint8	tempindex = 0, i = 0 ;
	uint8	l_u8PCProtocolBuf[255] = {0};
	memcpy(l_u8PCProtocolBuf, pUartDataBuf, sizeof(l_u8PCProtocolBuf));
	//跳过版本号设置//
	tempindex += 11;	
	SETUPALIAS.u8InstallFlag = l_u8PCProtocolBuf[tempindex++];	  //安装方式 0 侧桩 1 正桩

	SETUPALIAS.u8BaudRate = l_u8PCProtocolBuf[tempindex++];		//2
	SETUPALIAS.u8DOG = l_u8PCProtocolBuf[tempindex++];			// 3
	SETUPALIAS.HeightLaser0 = (l_u8PCProtocolBuf[tempindex]<<24)		//	   4
							+ (l_u8PCProtocolBuf[tempindex+1]<<16)
							+ (l_u8PCProtocolBuf[tempindex+2]<<8)
							+ l_u8PCProtocolBuf[tempindex+3];
	tempindex += 4;
	SETUPALIAS.HeightLaser1 = (l_u8PCProtocolBuf[tempindex]<<24)		//	   4
							+ (l_u8PCProtocolBuf[tempindex+1]<<16)
							+ (l_u8PCProtocolBuf[tempindex+2]<<8)
							+ l_u8PCProtocolBuf[tempindex+3];
	tempindex += 4;
	SETUPALIAS.HeightLaser2 = (l_u8PCProtocolBuf[tempindex]<<24)		//	   4
							+ (l_u8PCProtocolBuf[tempindex+1]<<16)
							+ (l_u8PCProtocolBuf[tempindex+2]<<8)
							+ l_u8PCProtocolBuf[tempindex+3];
	tempindex += 4;
	SETUPALIAS.HeightLaser3 = (l_u8PCProtocolBuf[tempindex]<<24)		//	   4
							+ (l_u8PCProtocolBuf[tempindex+1]<<16)
							+ (l_u8PCProtocolBuf[tempindex+2]<<8)
							+ l_u8PCProtocolBuf[tempindex+3];
//	tempindex += 4;
//	SETUPALIAS.IncHeightLaser = (l_u8PCProtocolBuf[tempindex]<<24)		//	8
//							+ (l_u8PCProtocolBuf[tempindex+1]<<16)
//							+ (l_u8PCProtocolBuf[tempindex+2]<<8)
//							+ l_u8PCProtocolBuf[tempindex+3];
	tempindex += 4;
	SETUPALIAS.LaserDistance = (l_u8PCProtocolBuf[tempindex]<<24)		   //  12
							+ (l_u8PCProtocolBuf[tempindex+1]<<16)
							+ (l_u8PCProtocolBuf[tempindex+2]<<8)
							+ l_u8PCProtocolBuf[tempindex+3];
//	tempindex += 4;
//	SETUPALIAS.Angle12 = (l_u8PCProtocolBuf[tempindex]<<24)					  // 16
//							+ (l_u8PCProtocolBuf[tempindex+1]<<16)
//							+ (l_u8PCProtocolBuf[tempindex+2]<<8)
//							+ l_u8PCProtocolBuf[tempindex+3]; 					
	tempindex += 4;
	SETUPALIAS.LaneWide = (l_u8PCProtocolBuf[tempindex]<<24)				  // 20
							+ (l_u8PCProtocolBuf[tempindex+1]<<16)
							+ (l_u8PCProtocolBuf[tempindex+2]<<8)
							+ l_u8PCProtocolBuf[tempindex+3];
	tempindex += 4;
	SETUPALIAS.MedianWide = (l_u8PCProtocolBuf[tempindex]<<24)				 //	 24
							+ (l_u8PCProtocolBuf[tempindex+1]<<16)
							+ (l_u8PCProtocolBuf[tempindex+2]<<8)
							+ l_u8PCProtocolBuf[tempindex+3];
	tempindex += 4;
	SETUPALIAS.MedianLeftWide = (l_u8PCProtocolBuf[tempindex]<<24)			  //   28
							+ (l_u8PCProtocolBuf[tempindex+1]<<16)
							+ (l_u8PCProtocolBuf[tempindex+2]<<8)
							+ l_u8PCProtocolBuf[tempindex+3];
	tempindex += 4;
//			SETUPALIAS.VdirAngle = (l_u8PCProtocolBuf[tempindex]<<24)
//									+ (l_u8PCProtocolBuf[tempindex+1]<<16)
//									+ (l_u8PCProtocolBuf[tempindex+2]<<8)
//									+ l_u8PCProtocolBuf[tempindex+3];
//			tempindex += 4;
//			SETUPALIAS.IdirAngle = (l_u8PCProtocolBuf[tempindex]<<24)
//									+ (l_u8PCProtocolBuf[tempindex+1]<<16)
//									+ (l_u8PCProtocolBuf[tempindex+2]<<8)
//									+ l_u8PCProtocolBuf[tempindex+3];
//			tempindex += 4;
	SETUPALIAS.resetCnt = (l_u8PCProtocolBuf[tempindex]<<24)				 //		32
							+ (l_u8PCProtocolBuf[tempindex+1]<<16)
							+ (l_u8PCProtocolBuf[tempindex+2]<<8)
							+ l_u8PCProtocolBuf[tempindex+3];
	tempindex += 4;
	SETUPALIAS.u32LaserRoadAngle = (l_u8PCProtocolBuf[tempindex]<<24)			// 36
							+ (l_u8PCProtocolBuf[tempindex+1]<<16)
							+ (l_u8PCProtocolBuf[tempindex+2]<<8)
							+ l_u8PCProtocolBuf[tempindex+3];

	tempindex += 4;
	SETUPALIAS.VerticalLaser_IP = (l_u8PCProtocolBuf[tempindex]<<24)		   //	40
							+ (l_u8PCProtocolBuf[tempindex+1]<<16)
							+ (l_u8PCProtocolBuf[tempindex+2]<<8)
							+ l_u8PCProtocolBuf[tempindex+3];

	tempindex += 4;
	SETUPALIAS.VerticalLaser_Port = (l_u8PCProtocolBuf[tempindex]<<24)			  //44
							+ (l_u8PCProtocolBuf[tempindex+1]<<16)
							+ (l_u8PCProtocolBuf[tempindex+2]<<8)
							+ l_u8PCProtocolBuf[tempindex+3];

	tempindex += 4;
	SETUPALIAS.ParallerLaser_IP1 = (l_u8PCProtocolBuf[tempindex]<<24)				//	  48
							+ (l_u8PCProtocolBuf[tempindex+1]<<16)
							+ (l_u8PCProtocolBuf[tempindex+2]<<8)
							+ l_u8PCProtocolBuf[tempindex+3];

	tempindex += 4;
	SETUPALIAS.ParallerLaser_Port1 = (l_u8PCProtocolBuf[tempindex]<<24)			//		52
							+ (l_u8PCProtocolBuf[tempindex+1]<<16)
							+ (l_u8PCProtocolBuf[tempindex+2]<<8)
							+ l_u8PCProtocolBuf[tempindex+3];
//2014-12-04
	tempindex += 4;
	SETUPALIAS.ParallerLaser_IP2 = (l_u8PCProtocolBuf[tempindex]<<24)				//	  48
							+ (l_u8PCProtocolBuf[tempindex+1]<<16)
							+ (l_u8PCProtocolBuf[tempindex+2]<<8)
							+ l_u8PCProtocolBuf[tempindex+3];

	tempindex += 4;
	SETUPALIAS.ParallerLaser_Port2 = (l_u8PCProtocolBuf[tempindex]<<24)			//		52
							+ (l_u8PCProtocolBuf[tempindex+1]<<16)
							+ (l_u8PCProtocolBuf[tempindex+2]<<8)
							+ l_u8PCProtocolBuf[tempindex+3];
//2014-12-04
//控制器ip参数
	tempindex += 4;
	SETUPALIAS.u32LocalIPAddress = (l_u8PCProtocolBuf[tempindex]<<24)		   // 56
							+ (l_u8PCProtocolBuf[tempindex+1]<<16)
							+ (l_u8PCProtocolBuf[tempindex+2]<<8)
							+ l_u8PCProtocolBuf[tempindex+3];
	tempindex += 4;
	SETUPALIAS.u32SubMask = (l_u8PCProtocolBuf[tempindex]<<24)					//	60
							+ (l_u8PCProtocolBuf[tempindex+1]<<16)
							+ (l_u8PCProtocolBuf[tempindex+2]<<8)
							+ l_u8PCProtocolBuf[tempindex+3];

	tempindex += 4;
	SETUPALIAS.u32LocalPortNO = (l_u8PCProtocolBuf[tempindex]<<24)				 //		 64
							+ (l_u8PCProtocolBuf[tempindex+1]<<16)
							+ (l_u8PCProtocolBuf[tempindex+2]<<8)
							+ l_u8PCProtocolBuf[tempindex+3];

	tempindex += 4;
	SETUPALIAS.u32GatewayIP = (l_u8PCProtocolBuf[tempindex]<<24)				 //			68
							+ (l_u8PCProtocolBuf[tempindex+1]<<16)
							+ (l_u8PCProtocolBuf[tempindex+2]<<8)
							+ l_u8PCProtocolBuf[tempindex+3];

	tempindex += 4;
	for(i=0; i<6; i++)
	{
		SETUPALIAS.au8LocalMAC[i] = l_u8PCProtocolBuf[tempindex++];
	}
//20130425
//			tempindex += 6;
	SETUPALIAS.u32Net1_DisconnectNum = (l_u8PCProtocolBuf[tempindex]<<24)		   //
							+ (l_u8PCProtocolBuf[tempindex+1]<<16)
							+ (l_u8PCProtocolBuf[tempindex+2]<<8)
							+ l_u8PCProtocolBuf[tempindex+3];

	tempindex += 4;
	SETUPALIAS.u32Net2_DisconnectNum = (l_u8PCProtocolBuf[tempindex]<<24)		   //
							+ (l_u8PCProtocolBuf[tempindex+1]<<16)
							+ (l_u8PCProtocolBuf[tempindex+2]<<8)
							+ l_u8PCProtocolBuf[tempindex+3];

	tempindex += 4;
	SETUPALIAS.u32Net1_InvalidRecNum = (l_u8PCProtocolBuf[tempindex]<<24)		   //
							+ (l_u8PCProtocolBuf[tempindex+1]<<16)
							+ (l_u8PCProtocolBuf[tempindex+2]<<8)
							+ l_u8PCProtocolBuf[tempindex+3];

	tempindex += 4;
	SETUPALIAS.u32Net2_InvalidRecNum = (l_u8PCProtocolBuf[tempindex]<<24)		   //
							+ (l_u8PCProtocolBuf[tempindex+1]<<16)
							+ (l_u8PCProtocolBuf[tempindex+2]<<8)
							+ l_u8PCProtocolBuf[tempindex+3];

	tempindex += 4;
	SETUPALIAS.u32DataProcException = (l_u8PCProtocolBuf[tempindex]<<24)		   //
							+ (l_u8PCProtocolBuf[tempindex+1]<<16)
							+ (l_u8PCProtocolBuf[tempindex+2]<<8)
							+ l_u8PCProtocolBuf[tempindex+3];
	tempindex += 4;

	SETUPALIAS.n32LaserHorizOff = (l_u8PCProtocolBuf[tempindex]<<24)			   //
							+ (l_u8PCProtocolBuf[tempindex+1]<<16)
							+ (l_u8PCProtocolBuf[tempindex+2]<<8)
							+ l_u8PCProtocolBuf[tempindex+3];
    tempindex += 4;
		//u16VerticalZeroPos set//
	SETUPALIAS.u16VerticalZeroPos0 = (l_u8PCProtocolBuf[tempindex]<<8)
			+ l_u8PCProtocolBuf[tempindex+1];
	tempindex += 2;
	SETUPALIAS.u16VerticalZeroPos1 = (l_u8PCProtocolBuf[tempindex]<<8)
			+ l_u8PCProtocolBuf[tempindex+1];
	tempindex += 2;
	SETUPALIAS.u16VerticalZeroPos2 = (l_u8PCProtocolBuf[tempindex]<<8)
			+ l_u8PCProtocolBuf[tempindex+1];
	tempindex += 2;
	SETUPALIAS.u16VerticalZeroPos3 = (l_u8PCProtocolBuf[tempindex]<<8)
			+ l_u8PCProtocolBuf[tempindex+1];
	tempindex += 2;
		//u16VerticalZeroPos set//
//	SETUPALIAS.u16InclineZeroPos = (l_u8PCProtocolBuf[tempindex]<<8)
//			+ l_u8PCProtocolBuf[tempindex+1];
//	tempindex += 2;
		//u16InclineZeroPos set//
	SETUPALIAS.u16StartPtNum0 =(l_u8PCProtocolBuf[tempindex]<<8)
			+ l_u8PCProtocolBuf[tempindex+1];
	tempindex += 2;
		//u16StartPtNum set//
	SETUPALIAS.u16EndPtNum0 = (l_u8PCProtocolBuf[tempindex]<<8)
			+ l_u8PCProtocolBuf[tempindex+1];
	tempindex += 2;

		SETUPALIAS.u16StartPtNum1 =(l_u8PCProtocolBuf[tempindex]<<8)
			+ l_u8PCProtocolBuf[tempindex+1];
	tempindex += 2;
		//u16StartPtNum set//
	SETUPALIAS.u16EndPtNum1 = (l_u8PCProtocolBuf[tempindex]<<8)
			+ l_u8PCProtocolBuf[tempindex+1];
	tempindex += 2;
		SETUPALIAS.u16StartPtNum2 =(l_u8PCProtocolBuf[tempindex]<<8)
			+ l_u8PCProtocolBuf[tempindex+1];
	tempindex += 2;
		//u16StartPtNum set//
	SETUPALIAS.u16EndPtNum2 = (l_u8PCProtocolBuf[tempindex]<<8)
			+ l_u8PCProtocolBuf[tempindex+1];
	tempindex += 2;
		SETUPALIAS.u16StartPtNum3 =(l_u8PCProtocolBuf[tempindex]<<8)
			+ l_u8PCProtocolBuf[tempindex+1];
	tempindex += 2;
		//u16StartPtNum set//
	SETUPALIAS.u16EndPtNum3 = (l_u8PCProtocolBuf[tempindex]<<8)
			+ l_u8PCProtocolBuf[tempindex+1];
	tempindex += 2;

	SETUPALIAS.u8LaserDevType = l_u8PCProtocolBuf[tempindex++];
	SETUPALIAS.u8TrafficType = l_u8PCProtocolBuf[tempindex++];
	SETUPALIAS.u8RoadType = l_u8PCProtocolBuf[tempindex++];	     // 道路类型 0-- 国道 1-- 高速
	SETUPALIAS.u8LaneNum = l_u8PCProtocolBuf[tempindex++];	     //车道数 0-- 4车道 1-- 6车道
	SETUPALIAS.u8NetType = l_u8PCProtocolBuf[tempindex++];       //网络连接：0 -- 无线 1 -- 有线

	AddCrc16((uint8 *)&SETUPALIAS,sizeof(SETUPALIAS)-2);
	tempindex = 0;
	l_u8PCProtocolBuf[tempindex++] = 0xFF;
	l_u8PCProtocolBuf[tempindex++] = 0xAA;
	l_u8PCProtocolBuf[tempindex++] = WJ_CMD_SETPARAM;
	tempindex++;	 	//u8tempBuf[u8tempIndex++] = ;	   //长度
	l_u8PCProtocolBuf[tempindex++] = RDid[0];			//4-5:
	l_u8PCProtocolBuf[tempindex++] = RDid[1];
	
	if(Write256_full(BUF0ADDR, (uint8 *)&SETUPALIAS, sizeof(SETUPALIAS)) && Write256_full(BUF0ADDR_BK, (uint8 *)&SETUPALIAS, sizeof(SETUPALIAS)) )
	{
		ret = 1;
	}
	else
	{
		ret = 0;
	}	
					
	if(ret && 	Write256_full(DEVICECODEADDR, RDid, 2) )		//修改成功
	{		
		l_u8PCProtocolBuf[tempindex++] =1;
		l_u8PCProtocolBuf[3] = tempindex - 4;			  //长度   20130703
		checksum = CRCSum(l_u8PCProtocolBuf+4, tempindex-4);
		l_u8PCProtocolBuf[tempindex++] = (checksum>>8)&0xFF;
		l_u8PCProtocolBuf[tempindex++] = checksum&0xFF;
		UART1_SendBuf_full(l_u8PCProtocolBuf, tempindex);
		OSTimeDly(1000);
		OSSchedLock();
		ResetSystem(); 
	}
	else
	{
		l_u8PCProtocolBuf[tempindex++] =0;
		l_u8PCProtocolBuf[3] = tempindex - 4;			  //长度   20130703
		checksum = CRCSum(l_u8PCProtocolBuf+4, tempindex-4);
		l_u8PCProtocolBuf[tempindex++] = (checksum>>8)&0xFF;
		l_u8PCProtocolBuf[tempindex++] = checksum&0xFF;
		UART1_SendBuf_full(l_u8PCProtocolBuf, tempindex);
	}
	
}

void SendDevOtherInfo(void)
{
	
}

void ClearResetInfoBuf(void)
{
	uint8	u8tempBuf[100] = {0};
	uint16	checksum = 0;
	uint8	u8tempIndex = 0;
	CycleBufferStruct  *pTmpResetCycBuf;	
		
	u8tempIndex = 0;
	u8tempBuf[u8tempIndex++] = 0xFF;
	u8tempBuf[u8tempIndex++] = 0xAA;
	u8tempBuf[u8tempIndex++] = WJ_CMD_CLEARRESETIFNOBUF;
	u8tempIndex++;	 	//u8tempBuf[u8tempIndex++] = ;
	u8tempBuf[u8tempIndex++] = RDid[0];			//4-5:
	u8tempBuf[u8tempIndex++] = RDid[1];		
	

	pTmpResetCycBuf = (CycleBufferStruct *)malloc(sizeof(CycleBufferStruct) + RESETINFO_BUFFERSIZE * sizeof(SystemTime));
	if(pTmpResetCycBuf == NULL)
	{
		pTmpResetCycBuf = (CycleBufferStruct *)malloc(sizeof(CycleBufferStruct) + RESETINFO_BUFFERSIZE * sizeof(SystemTime));	
	}
	memset((uint8 *)pTmpResetCycBuf, 0, sizeof(CycleBufferStruct) + RESETINFO_BUFFERSIZE * sizeof(SystemTime));
	if (TRUE == Write256_full(RESETINFOADDR, (uint8 *)pTmpResetCycBuf, sizeof(CycleBufferStruct) + RESETINFO_BUFFERSIZE * sizeof(SystemTime)))
	{
		u8tempBuf[u8tempIndex++] = 0x01;			 //成功			
	}
	else
	{
		u8tempBuf[u8tempIndex++] = 0x02;			 //失败	
	}
	u8tempBuf[3] = u8tempIndex - 4;			  //长度
	checksum = CRCSum(u8tempBuf+4, u8tempIndex-4);
	u8tempBuf[u8tempIndex++] = (checksum>>8)&0xFF;
	u8tempBuf[u8tempIndex++] = checksum&0xFF;
	UART1_SendBuf_full(u8tempBuf, u8tempIndex);	
				
	free(pTmpResetCycBuf);
	pTmpResetCycBuf = NULL;
}

void GetDeviceTime()
{
	uint16	checksum = 0;
	uint8	tempindex = 0;
	uint8	l_u8PCProtocolBuf[150] = {0};
	tempindex = 4;
	l_u8PCProtocolBuf[tempindex++] = SEC;
	l_u8PCProtocolBuf[tempindex++] = MIN;
	l_u8PCProtocolBuf[tempindex++] = HOUR;
	l_u8PCProtocolBuf[tempindex++] = WEEK;
	l_u8PCProtocolBuf[tempindex++] = DAY;
	l_u8PCProtocolBuf[tempindex++] = MONTH;
	l_u8PCProtocolBuf[tempindex++] = YEAR % 100;
	l_u8PCProtocolBuf[tempindex++] = YEAR / 100;

	l_u8PCProtocolBuf[0] = 0xFF;
	l_u8PCProtocolBuf[1] = 0xAA;
	l_u8PCProtocolBuf[2] = WJ_CMD_GETDEVICETIME;					
	l_u8PCProtocolBuf[3] = tempindex - 4;			  //长度   20130703
	checksum = CRCSum(l_u8PCProtocolBuf+4, tempindex-4);
	l_u8PCProtocolBuf[tempindex++] = (checksum>>8)&0xFF;
	l_u8PCProtocolBuf[tempindex++] = checksum&0xFF;
	UART1_SendBuf_full(l_u8PCProtocolBuf, tempindex);
}
void InitAllParam()
{
	uint8 ret = 0;
	uint8 l_u8PCProtocolBuf[8] ={0xFF,0xAA,0x3d,0x02,0x00,0x01,0x00,0x01}; 
	UART1_SendBuf_full(l_u8PCProtocolBuf, 8);
	JZInit();                    //初始化控制器
	AddCrc16((uint8 *)&g_sspSetup,sizeof(g_sspSetup)-2);	 
//	Write256_full(BUF0ADDR,(uint8 *)&SETUPALIAS, sizeof(SETUPALIAS));
	ret = Write256_full(BUF0ADDR,(uint8 *)&g_sspSetup, sizeof(g_sspSetup));
	if(ret == FALSE)
	{
		OSTimeDly(50);
		ret =  Write256_full(BUF0ADDR, (uint8 *)&g_sspSetup, sizeof(g_sspSetup));
	}

//	OSTimeDly(50);
//	ret = Write256_full(BUF0ADDR_BK, (uint8 *)&SETUPALIAS, sizeof(SETUPALIAS));
//	if(ret == FALSE)
//	{
//		OSTimeDly(50);
//		ret =  Write256_full(BUF0ADDR_BK, (uint8 *)&SETUPALIAS, sizeof(SETUPALIAS));
//	}

	OSTimeDly(1000);
	OSSchedLock();
	ResetSystem();	
}
void ResetDevice()
{
	uint8 l_u8PCProtocolBuf[8] ={0xFF,0xAA,0x3e,0x02,0x00,0x01,0x00,0x01}; 
	UART1_SendBuf_full(l_u8PCProtocolBuf, 8);
	OSTimeDly(1000);
	OSSchedLock();
	ResetSystem();	
}

void SetDeviceTime(uint8 *pUartDataBuf)
{
	uint16	checksum = 0;
	uint8	tempindex = 0, i = 0 ;
	SystemTime dev_time;
	uint8	l_u8PCProtocolBuf[20] = {0};
	memcpy(l_u8PCProtocolBuf, pUartDataBuf, sizeof(l_u8PCProtocolBuf));

	dev_time.u8Second =	l_u8PCProtocolBuf[0];
	dev_time.u8Minute = l_u8PCProtocolBuf[1];
	dev_time.u8Hour = l_u8PCProtocolBuf[2];
	dev_time.u8Week = l_u8PCProtocolBuf[3];
	dev_time.u8Day = l_u8PCProtocolBuf[4];
	dev_time.u8Month = l_u8PCProtocolBuf[5];
	dev_time.u16Year = l_u8PCProtocolBuf[6] + l_u8PCProtocolBuf[7]*100; 

	SetRTCTime(&dev_time);
	//返回成功
	l_u8PCProtocolBuf[0] = 0xFF;
	l_u8PCProtocolBuf[1] = 0xAA;
	l_u8PCProtocolBuf[2] = WJ_CMD_SETDEVICETIME;					
	l_u8PCProtocolBuf[3] = 2;
	l_u8PCProtocolBuf[4] = 0;
	l_u8PCProtocolBuf[5] = 1;
	l_u8PCProtocolBuf[6] = 0;
	l_u8PCProtocolBuf[7] = 1;
	UART1_SendBuf_full(l_u8PCProtocolBuf, 8);
}

void GetRDID()
{
	uint16	checksum = 0;
	uint8	tempindex = 0;
	uint8	l_u8PCProtocolBuf[150] = {0};
	uint8   i=0;
	tempindex = 4;

	for(i=0; i<16; i++)
	{
		l_u8PCProtocolBuf[tempindex++] = RDid[i];	
	}
	for(i=0; i<15; i++)
	{
		l_u8PCProtocolBuf[tempindex++] = RDNum[i];
	}

	l_u8PCProtocolBuf[0] = 0xFF;
	l_u8PCProtocolBuf[1] = 0xAA;
	l_u8PCProtocolBuf[2] = WJ_CMD_GETRDID;					
	l_u8PCProtocolBuf[3] = tempindex - 4;			  //长度   20130703
	checksum = CRCSum(l_u8PCProtocolBuf+4, tempindex-4);
	l_u8PCProtocolBuf[tempindex++] = (checksum>>8)&0xFF;
	l_u8PCProtocolBuf[tempindex++] = checksum&0xFF;
	UART1_SendBuf_full(l_u8PCProtocolBuf, tempindex);
}

void SetRDID(uint8 *pUartDataBuf)
{
	uint16	checksum = 0;
	uint8	tempindex = 0, i = 0 ;
	uint8	l_u8PCProtocolBuf[50] = {0};
	memcpy(l_u8PCProtocolBuf, pUartDataBuf, sizeof(l_u8PCProtocolBuf));

	for(i=0; i<16; i++)
	{
		 RDid[i] = l_u8PCProtocolBuf[tempindex++];	
	}
	for(i=0; i<15; i++)
	{
		RDNum[i] = l_u8PCProtocolBuf[tempindex++];
	}
	Write256_full(DEVICECODEADDR, RDid, 16);  //存铁电
	Write256_full(STATIONNUMADDR, RDNum, 15);  //存铁电

   	//返回成功
	l_u8PCProtocolBuf[0] = 0xFF;
	l_u8PCProtocolBuf[1] = 0xAA;
	l_u8PCProtocolBuf[2] = WJ_CMD_SETRDID;					
	l_u8PCProtocolBuf[3] = 2;
	l_u8PCProtocolBuf[4] = 0;
	l_u8PCProtocolBuf[5] = 1;
	l_u8PCProtocolBuf[6] = 0;
	l_u8PCProtocolBuf[7] = 1;
	UART1_SendBuf_full(l_u8PCProtocolBuf, 8);
}
void GetLaneDir()
{
	uint16	checksum = 0;
	uint8	tempindex = 0;
	uint8	l_u8PCProtocolBuf[150] = {0};
	uint8   i=0;
	tempindex = 4;

	l_u8PCProtocolBuf[tempindex++] = g_u8LaneDir;

	l_u8PCProtocolBuf[0] = 0xFF;
	l_u8PCProtocolBuf[1] = 0xAA;
	l_u8PCProtocolBuf[2] = WJ_CMD_GETLANEDIR;					
	l_u8PCProtocolBuf[3] = tempindex - 4;			  //长度   20130703
	checksum = CRCSum(l_u8PCProtocolBuf+4, tempindex-4);
	l_u8PCProtocolBuf[tempindex++] = (checksum>>8)&0xFF;
	l_u8PCProtocolBuf[tempindex++] = checksum&0xFF;
	UART1_SendBuf_full(l_u8PCProtocolBuf, tempindex);
}

void SetLaneDir(uint8 *pUartDataBuf)
{
	uint16	checksum = 0;
	uint8	tempindex = 0, i = 0 ;
	uint8	l_u8PCProtocolBuf[50] = {0};
	memcpy(l_u8PCProtocolBuf, pUartDataBuf, sizeof(l_u8PCProtocolBuf));

	g_u8LaneDir = l_u8PCProtocolBuf[tempindex];

	Write256_full(LANEDIRADDR, &g_u8LaneDir, 1);  //存铁电

   	//返回成功
	l_u8PCProtocolBuf[0] = 0xFF;
	l_u8PCProtocolBuf[1] = 0xAA;
	l_u8PCProtocolBuf[2] = WJ_CMD_SETLANEDIR;					
	l_u8PCProtocolBuf[3] = 2;
	l_u8PCProtocolBuf[4] = 0;
	l_u8PCProtocolBuf[5] = 1;
	l_u8PCProtocolBuf[6] = 0;
	l_u8PCProtocolBuf[7] = 1;
	UART1_SendBuf_full(l_u8PCProtocolBuf, 8);
}

//void GetSD01Data(uint8 *pUartDataBuf)
//{
//	uint16	checksum = 0;
//	uint8	tempindex = 0 ;
//	uint32  i = 0;
//	uint8	l_u8PCProtocolBuf[50] = {0};
//	uint8   get_buf[512];
//	uint8   get_len = 0;
//	V_TIME  temp_time;
//	uint16   start_num;
//	uint16   end_num;
//	memcpy(l_u8PCProtocolBuf, pUartDataBuf, sizeof(l_u8PCProtocolBuf));
//
//	temp_time.year = l_u8PCProtocolBuf[tempindex++];	//获取年
//	temp_time.month = l_u8PCProtocolBuf[tempindex++];   //获取月
//	temp_time.day = l_u8PCProtocolBuf[tempindex++];	    //获取日
//	start_num = (l_u8PCProtocolBuf[tempindex]<<8)		 //起始时间序号
//			+ l_u8PCProtocolBuf[tempindex+1];
//	tempindex += 2;		   
//	end_num = (l_u8PCProtocolBuf[tempindex]<<8)		     //结束时间序号
//			+ l_u8PCProtocolBuf[tempindex+1];
//	tempindex += 2;	
//	//有效性检验
//	if(start_num <= 288 && end_num <= 288 && start_num <= end_num)
//	{
//		//返回成功
//		l_u8PCProtocolBuf[0] = 0xFF;
//		l_u8PCProtocolBuf[1] = 0xAA;
//		l_u8PCProtocolBuf[2] = WJ_CMD_GETSD01DATA;					
//		l_u8PCProtocolBuf[3] = 2;
//		l_u8PCProtocolBuf[4] = 0;
//		l_u8PCProtocolBuf[5] = 1;
//		l_u8PCProtocolBuf[6] = 0;
//		l_u8PCProtocolBuf[7] = 1;
//		UART1_SendBuf_full(l_u8PCProtocolBuf, 8);  //回应成功
//		OSTimeDly(500);
//
//		for(i=start_num; i<=end_num; i++)
//		{
//			get_len = get01_from_sd(get_buf,&temp_time,i);
//			if(get_len>0)
//			{
//				Send1Data(get_buf,get_len);
//				OSTimeDly(500);    
//			}
//		}		
//	}
//	else
//	{
//		//返回失败
//		l_u8PCProtocolBuf[0] = 0xFF;
//		l_u8PCProtocolBuf[1] = 0xAA;
//		l_u8PCProtocolBuf[2] = WJ_CMD_GETSD01DATAERR;					
//		l_u8PCProtocolBuf[3] = 2;
//		l_u8PCProtocolBuf[4] = 0;
//		l_u8PCProtocolBuf[5] = 1;
//		l_u8PCProtocolBuf[6] = 0;
//		l_u8PCProtocolBuf[7] = 1;
//		UART1_SendBuf_full(l_u8PCProtocolBuf, 8);  //回应失败
//	}	
//}

