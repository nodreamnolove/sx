#ifndef __COMMON_H__
#define __COMMON_H__

#include "config.h"

#ifdef	__COMMON_C
#define	CMN_EXT	
#else
#define	CMN_EXT	extern
#endif
/***************************************/
/***************************************/


/***************************************/
/**********任务优先级*******************/
/***************************************/
#define		Task_JG0PRIO		4
#define		Task_JG1PRIO		5
#define 	Task_JG2PRIO        6
#define     Task_JG3PRIO         7
#define		Task_Data_JGPRIO	8
#define		Task_Uart5PRIO		3//hong 调整
#define 	Task_Uart5_SenddataPRIO 10
#define 	Task_SDPRIO 		9//
#define		TaskSvContinuePRIO  12
#define     TaskSendUart1PRIO	11
#define		TASKChecknetPRIO	14
#define		TASK_TiPoPRIO		2           //时钟任务 由13改为2 防止统计丢包 20131216

#define 	Task_TestPRIO       1
//////////////////////////////////////
#define		TASKSTARTPRIO		16
/***************************************/
/**********任务优先级*******************/
/***************************************/

extern	uint16	YEAR;				//年
extern	uint8	MONTH;				//月
extern	uint8	DAY;					//日
extern	uint8	WEEK;					//星期几
extern	uint8	HOUR;					//时
extern	uint8	MIN;				//分
extern	uint8	SEC;				//秒
extern  uint8   YEAR_uint8;//8位年
//临时保存统计包数据时间
extern	uint16	TEMP_YEAR;				//年
extern	uint8	TEMP_MONTH;				//月
extern	uint8	TEMP_DAY;				//日
extern	uint8	TEMP_WEEK;				//星期几
extern	uint8	TEMP_HOUR;				//时
extern	uint8	TEMP_MIN;				//分
extern	uint8	TEMP_SEC;				//秒

extern	unsigned int Ser_Ip[4];
extern	uint16 MinTotalNew;
extern	uint16 MinTotalOld;

CMN_EXT	uint16	AddCrc16( uint8 * p_pu8Ptr,uint16 p_u16Len);		//校验，写CRC
CMN_EXT	uint8	CheckCrc( uint8 * p_pu8Ptr,uint16 p_u16Len);		//校验，不写CRC
CMN_EXT	uint8	bcd(uint32 p_u32Value,uint8 p_u8Pos);				//获取bcd数字

extern uint16 CRCSum(uint8 *data, uint32 length);
extern uint8 Check_CRCSum(uint8 *ptr, uint16 len);

#endif
