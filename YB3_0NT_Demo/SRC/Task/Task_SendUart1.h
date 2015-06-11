#ifndef __TASK_SENDUART1_H__
#define	__TASK_SENDUART1_H__



#include "Common.h"
extern OS_EVENT  *g_Uart1_send;

struct _arg_RenewVehSendInfo
{
	uint16 u16Year;
	uint8 u8Month;
	uint8 u8Day;
	uint32 u32VehIndex;
};

extern uint8  ReadData_FromSD(uint8* const data, uint16* const pDataLen, const uint16 u16PackageSeq, const uint8 u8TimeFlag);
extern uint8  ProcDataPackage(uint8* const pData, const uint16 u16PackageSeq, const uint16 u16DataLen);
extern uint32 GetData_SDAdd(const uint16 u16PackageSeq, const uint8 u8TimeFlag); 
extern void RenewVehSendInfo(struct _arg_RenewVehSendInfo*pstru);
uint8 Stru_Is_Empty(_VehSendInfo *p_VehSendInfo);
#endif
