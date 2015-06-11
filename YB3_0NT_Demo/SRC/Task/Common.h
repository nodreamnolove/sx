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
/**********�������ȼ�*******************/
/***************************************/
#define		Task_JG0PRIO		4
#define		Task_JG1PRIO		5
#define 	Task_JG2PRIO        6
#define     Task_JG3PRIO         7
#define		Task_Data_JGPRIO	8
#define		Task_Uart5PRIO		3//hong ����
#define 	Task_Uart5_SenddataPRIO 10
#define 	Task_SDPRIO 		9//
#define		TaskSvContinuePRIO  12
#define     TaskSendUart1PRIO	11
#define		TASKChecknetPRIO	14
#define		TASK_TiPoPRIO		2           //ʱ������ ��13��Ϊ2 ��ֹͳ�ƶ��� 20131216

#define 	Task_TestPRIO       1
//////////////////////////////////////
#define		TASKSTARTPRIO		16
/***************************************/
/**********�������ȼ�*******************/
/***************************************/

extern	uint16	YEAR;				//��
extern	uint8	MONTH;				//��
extern	uint8	DAY;					//��
extern	uint8	WEEK;					//���ڼ�
extern	uint8	HOUR;					//ʱ
extern	uint8	MIN;				//��
extern	uint8	SEC;				//��
extern  uint8   YEAR_uint8;//8λ��
//��ʱ����ͳ�ư�����ʱ��
extern	uint16	TEMP_YEAR;				//��
extern	uint8	TEMP_MONTH;				//��
extern	uint8	TEMP_DAY;				//��
extern	uint8	TEMP_WEEK;				//���ڼ�
extern	uint8	TEMP_HOUR;				//ʱ
extern	uint8	TEMP_MIN;				//��
extern	uint8	TEMP_SEC;				//��

extern	unsigned int Ser_Ip[4];
extern	uint16 MinTotalNew;
extern	uint16 MinTotalOld;

CMN_EXT	uint16	AddCrc16( uint8 * p_pu8Ptr,uint16 p_u16Len);		//У�飬дCRC
CMN_EXT	uint8	CheckCrc( uint8 * p_pu8Ptr,uint16 p_u16Len);		//У�飬��дCRC
CMN_EXT	uint8	bcd(uint32 p_u32Value,uint8 p_u8Pos);				//��ȡbcd����

extern uint16 CRCSum(uint8 *data, uint32 length);
extern uint8 Check_CRCSum(uint8 *ptr, uint16 len);

#endif
