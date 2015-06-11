#ifndef _RD_data_H
#define _RD_data_H
#include "config.h"

typedef struct _TRAFFIC_DATA_SEND01
{
	uint32 lane_no;						// 车道号
	uint32 che_num; 					// 车数
	uint32 genche_num;					// 跟车数
	uint32 genchebaifenbi;				// 跟车百分比
	uint32 chetouspace_total; 			// 车头总间距
	uint32 ave_space; 					// 平均车头间距
	uint32 time_occupancy;				// 时间占有率
	uint32 xiaohuo_num;					// 小货交通量
	uint32 xiaohuo_avespeed;			// 小货平均速度
	uint32 xiaohuo_speed_total;			// 小货总速度
	uint32 zhonghuo_num;				// 中货交通量
	uint32 zhonghuo_avespeed;			// 中货平均速度
	uint32 zhonghuo_speed_total;		// 中货总速度
	uint32 dahuo_num;					// 大货交通量
	uint32 dahuo_avespeed;				// 大货平均速度
	uint32 dahuo_speed_total;			// 大货总速度
	uint32 zhongxiaoke_num;				// 中小客交通量
	uint32 zhongxiaoke_avespeed;		// 中小客平均速度
	uint32 zhongxiaoke_speed_total;		// 中小客总速度
	uint32 dake_num;					// 大客交通量
	uint32 dake_avespeed;				// 大客平均速度
	uint32 dake_speed_total;			// 大客总速度
	uint32 tuogua_num;					// 拖挂车交通量
	uint32 tuogua_avespeed;				// 拖挂车平均速度
	uint32 tuogua_speed_total;			// 拖挂车总速度
	uint32 tuolaji_num;					// 拖拉机交通量
	uint32 tuolaji_avespeed;			// 拖拉机平均速度
	uint32 tuolaji_speed_total;			// 拖拉机总速度
	uint32 tedahuo_num;					// 特大货交通量
	uint32 tedahuo_avespeed;			// 特大货平均速度
	uint32 tedahuo_speed_total;			// 特大货总速度
	uint32 moto_num;					// 摩托车交通量
	uint32 moto_avespeed;				// 摩托车平均速度
	uint32 moto_speed_total;			// 摩托车总速度
	uint32 jizhuangxiang_avespeed;	  	// 集装箱车平均速度
	uint32 jizhuangxiang_speed_total; 	// 集装箱车总速度
	uint32 jizhuangxiang_num;		  	// 集装箱车总速度

}TRAFFIC_DATA_SEND01	 ;
extern TRAFFIC_DATA_SEND01  Data_4_Lane[4];
extern uint16 DataResendBgnNum;
extern uint16 DataResendEndNum;
extern uint16 DataResendCnt;
 
extern	uint8 Flag_08_NotReturn;						//08包未返回标志位，为1表示还已经接收到数据，但还没有返回08包
extern uint8 Flag_NetConnect;
extern uint8 Flag_08_ReturnChangePara;
//正在使用的参数
extern uint8 UserName[8]; 					//用户名
extern uint8 KeyNum[8];						//密码
extern uint8 RDid[];						//设备身份识别码
extern uint8 RDNum[15];						//站点编号，15位，不足15位补0
extern uint16 ProCycle;						//交通数据处理周期 
extern uint8 InvContents; 					//调查内容 	
extern uint8 DisTime;						//跟车百分比鉴别时间 

//修改后还未生效的参数
extern uint8 NewUserName[8]; 				//新用户名
extern uint8 NewKeyNum[8];					//新密码
extern uint8 NewRDid[];						//新设备身份识别码
extern uint8 NewRDNum[15];					//新站点编号，15位，不足15位补0
extern uint16 NewProCycle;					//新交通数据处理周期 
extern uint8 NewInvContents; 				//新调查内容 	
extern uint8 NewDisTime;					//新跟车百分比鉴别时间 
extern uint8 NewDscIp[4];					//新服务器IP地址 

extern uint8 ConnectType;					//当前传输方式。01：有线网络传输  02：无线网络传输
extern double Longitude;					//经度
extern double Latitude;						//纬度
extern uint16 Elevation;					//海拔	
//extern uint16 SendFailNumEnd;

extern uint8 Flag_Change_RD_Num;
extern uint8 Flag_Change_DSC_Ip;
extern uint8 Flag_Change_InvContents;
extern uint8 Flag_Change_ProCycle;
extern uint8 Flag_Change_DisTime;
extern uint8	Hisyear;
extern uint8	Hismonth;
extern uint8	Hisday;
extern	union ASK08Data	ASK08_data;
extern	union ASK02Data	ASK02_data;
extern	union ASK08Data_Modify ASK08_data_Modify;
extern	char FlagSentVechicle;
extern	uint8 Flag_ReConnect;
extern	uint8 Flag_08_ReturnChangePara;
extern void Update_data02(void);
extern void Update_data08(void);
extern int32 Save_data_01_process(void);
extern uint8  ChangetoBG_chedao_num( uint8 ln );
extern	void JudgeQual(uint8 Qual, uint8 *p);
extern	void SendDevParamInfo(void);
extern	void SendDevOtherInfo(void);
extern	void ClearResetInfoBuf(void);
extern	void SetDevParamInfor(uint8 *pUartDataBuf);
extern	void SendDevParamInfor2(void);
extern	void Sendframe(void);
extern	void Sendframe2(void);
extern  void GetDeviceTime(void);
extern  void SetDeviceTime(uint8 *pUartDataBuf);
extern  void SetRDID(uint8 *pUartDataBuf);
extern  void GetRDID(void);
extern  void GetLaneDir(void);
extern  void SetLaneDir(uint8 *pUartDataBuf);
//extern  void GetSD01Data(uint8 *pUartDataBuf);
extern  void ResetDevice(void);
extern  void InitAllParam(void);
extern  void SetLimitHeight(uint8 *p);

extern uint8 Flag_08_NotReturn;						//08包未返回标志位，为1表示还已经接收到数据，但还没有返回08包
extern uint8 Flag_02Rev;							//02包收到标志位，收到02包该标志置1
extern uint8 Flag_0ARev;							//08包收到标志位，收到08包该标志置1

extern OS_EVENT *Flag_SendOK;						//数据发送成功信号量，数据发送后释放该信号量，发送数据时请求该信号量
extern OS_EVENT *EVENT_02Rev;						//02包收到信号量，收到时释放信号量，发送01包前先发送02包，再请求该信号量
extern OS_EVENT *EVENT_0ARev;						//0A包收到
extern OS_EVENT *EVENT_11Rev;

//数据包包头 
struct	RD_data	{
				uint8	Type;					//数据包类型 
				uint8	RD_ID[16];				//设备身份识别码，需用ASCII码表示 
				};


//0A格式数据包
struct ASK_0A	{
				uint8	Type;					//数据包类型
				uint8	TimeNumL;				//时间序号低8位
				uint8	TimeNumH;				//时间序号高8位 
				uint8	crcL;					//校验结果信息代码低字节
				uint8	crcH;					//校验结果信息代码高字节
				};


//0x02格式数据包 
struct ASK_02	{
				struct	RD_data	RDHead;
				uint8	Ask_Answer;				//询问与回复信息 
				};

//0x08格式数据包	
struct ASK_08	{
				struct	RD_data	RDHead;
// 				uint16	year;					//年份，低位在前、高位在后
 				uint8	yearL;					//年份，低位在前、高位在后
				uint8	yearH;					//年份，低位在前、高位在后 
				uint8	month; 					//月份 
				uint8	day;					//日 
				uint8	hour;					//小时 
				uint8	min;					//分钟 
				uint8   sec;					//秒 
				uint8	ProCycle;				//交通数据处理周期低字节
				uint8	InvContents; 			//调查内容 
				uint8	DSC_Ip[4];				//DSC IP 
				uint8	RD_Num[15];				//RD站点编号，ASCII码表示，不足部分填0 
				uint8	DisTime;				//跟车百分比鉴别时间 
				uint8	ConnectType;			//当前传输方式。01：有线网络传输  02：无线网络传输
				uint8	Longitude[8];			//经度，这里用数组记录，使用时转换为双精度浮点数
				uint8	Latitude[8];			//纬度，这里用数组记录，使用时转换为双精度浮点数
				uint8	ElevationL;
				uint8	ElevationH;				//高程（海拔），设备安装后填入  	 
				};

//0x08格式数据包	
struct ASK_08_Modify	{
				struct	RD_data	RDHead;
// 				uint16	year;					//年份，低位在前、高位在后
 				uint8	yearL;					//年份，低位在前、高位在后
				uint8	yearH;					//年份，低位在前、高位在后 
				uint8	month; 					//月份 
				uint8	day;					//日 
				uint8	hour;					//小时 
				uint8	min;					//分钟 
				uint8   sec;					//秒 
				uint8	ProCycle;				//交通数据处理周期
				uint8	InvContents; 			//调查内容 
				uint8	DSC_Ip[4];				//DSC IP 
				uint8	RD_Num[15];				//RD站点编号，ASCII码表示，不足部分填0 
				uint8	DisTime;				//跟车百分比鉴别时间 
				uint8	ConnectType;			//当前传输方式。01：有线网络传输  02：无线网络传输
				uint8	Longitude[8];			//经度，这里用数组记录，使用时转换为双精度浮点数
				uint8	Latitude[8];			//纬度，这里用数组记录，使用时转换为双精度浮点数
				uint8	ElevationL;
				uint8	ElevationH;				//高程（海拔），设备安装后填入  	 
				};
				
//03格式数据包
struct ASK_03	{
				struct	RD_data	RDHead;
				uint8	UserName[8];			//用户名，ASCII码
				uint8	KeyNum[8];				//密码
				uint8	New_RD_Num[15];			//新RD站点编号，ASCII码表示，不足部分填0
				};

//04格式数据包
struct ASK_04	{
				struct	RD_data	RDHead;
				uint8	UserName[8];			//用户名，ASCII码
				uint8	KeyNum[8];				//密码
				uint8	New_DSC_Ip[4];			//新DSC IP 
				};

//05格式数据包
struct ASK_05	{
				struct	RD_data	RDHead;
				uint8	UserName[8];			//用户名，ASCII码
				uint8	KeyNum[8];				//密码
 				uint8	yearL;					//年份，低位在前、高位在后
				uint8	yearH;					//年份，低位在前、高位在后 
				uint8	month; 					//月份 
				uint8	day;					//日 
				uint8	hour;					//小时 
				uint8	min;					//分钟 
				uint8   sec;					//秒 
				};


//06格式数据包
struct ASK_06	{
				struct	RD_data	RDHead;
				uint8	UserName[8];			//用户名，ASCII码
				uint8	KeyNum[8];				//密码
				uint8	New_InvContents; 		//新调查内容 
				};

//07格式数据包
struct ASK_07	{
				struct	RD_data	RDHead;
				uint8	UserName[8];			//用户名，ASCII码
				uint8	KeyNum[8];				//密码
				uint8	New_ProCycle;			//新交通数据处理周期
				};

//0B格式数据包
struct ASK_0B	{
				struct	RD_data	RDHead;
				uint8	UserName[8];			//用户名，ASCII码
				uint8	KeyNum[8];				//密码
				uint8	New_DisTime;			//新跟车百分比鉴别时间
				};

//09格式数据包
struct ASK_09	{
				struct	RD_data	RDHead;
				uint8	UserName[8];			//用户名，ASCII码
				uint8	KeyNum[8];				//密码
 				uint8	yearL;					//年份，低位在前、高位在后
				uint8	yearH;					//年份，低位在前、高位在后 
				uint8	month; 					//月份 
				uint8	day;					//日 
				uint8	StartNumL;				//起点时间序号
				uint8	StartNumH;				//起点时间序号
				uint8	EndNumL;				//止点时间序号
				uint8	EndNumH;				//止点时间序号
				};


union ASK08Data	{
		struct ASK_08	ASK08;
		uint8 	ASK08_65[65];
		};						 				//发08格式数据包时使用

union ASK02Data	{
		struct ASK_02	ASK02;
		uint8 	ASK02_18[18];					//发02格式数据包时使用
		};

union ASK08Data_Modify {
		struct ASK_08_Modify	ASK08_Modify;
		uint8 	ASK08_Modify_65[65];
		};						 				//发08格式数据包时使用，刚修改，还未生效的参数

extern uint8 TcpData_Pro(uint8 *p_uart5buff,uint16 p_len);

//uint8 LMS_Data_Pro(uint16 *p,uint16 len);

void RD_DataPro(uint8 *p_data,uint16 len);

extern uint8 SendData(uint8 *p,uint16 len);
		
void RD_Int08(void);
void RD_Int02(void);
//02格式数据包处理函数
void Ask_Answer(struct ASK_02 *p,uint16 len);
void Change_RD_Num(struct ASK_03 *p,uint16 len);
void Change_DSC_Ip(struct ASK_04 *p,uint16 len);
void Change_Time(struct ASK_05 *p,uint16 len);
void Change_InvContents(struct ASK_06 *p,uint16 len);
void Change_ProCycle(struct ASK_07 *p,uint16 len);
void Change_DisTime(struct ASK_0B *p,uint16 len);
//uint16	sumday(uint16 yearL, uint8 month, uint8 day);
extern uint32 SD_Base_address_01;


//用户名、密码校验函数，正确返回1，错误返回0
uint8 UserName_Key(struct ASK_03 *p,uint16 len);
//设备身份识别码校验函数，正确返回1，错误返回0
uint8 RD_ID_Key(struct RD_data *p,uint16 len);




extern	uint16	g_u16MsgPeriod;
#endif
