#include "FW.h"	  
#include "config.h"
#include "TDC256.h"
#include "Task_SD.h"
#include "rd_data.h"
#include "task_data_jg.h"
#include "timer0.h"
#include "task_jg0.h"
#include "task_jg1.h"


union {
uint8 u8[4];
uint32 u32;
}  FW_u8_and_u32;
union {
uint8 u8[2];
uint16 u16;
}  FW_u8_and_u16;
union{
uint8 u8[4];
float f32;
} FW_u8_and_f32;
/**********************************************
�������ƣ�Storage_Write_TDC256
����ʱ�䣺2012-8.15
����˵����������д������
**********************************************/
void Storage_Write_TDC256(uint8 p_coef)
{
////	uint8 write_buf[200];
////	uint32 i=0;
//	switch(p_coef)
//	{
//		case SD_TEST_WRITE:
//			FW_u8_and_u32.u32 = g_u32SD_err_count;
//			WriteC256(0x1200,FW_u8_and_u32.u8,4);
//			FW_u8_and_u32.u32 = g_u32SD_write_count;
//			WriteC256(0x1204,FW_u8_and_u32.u8,4);				
//			break;
//		case TOTAL_COUNT_VEH:
//			FW_u8_and_u32.u32 = g_total_count_Veh;
//			WriteC256(0x1208,FW_u8_and_u32.u8,4);
//			break;
//		case RDID:
//			WriteC256(0x1000,RDid,4);
//			break;
//		case RDNUM:
//			WriteC256(0x1011,RDNum,15);
//			break;
//		case PROCYCLE:
//			FW_u8_and_u16.u16 = ProCycle;
//			WriteC256(0x1020,FW_u8_and_u16.u8,2);
//			break;
//		case DISTIME:
//			FW_u8_and_u16.u16 = DisTime;
//			WriteC256(0x1022,FW_u8_and_u16.u8,2);
//			break;
//		case NEWDSCIP:
//			FW_u8_and_u32.u8[0] = NewDscIp[0];
//			FW_u8_and_u32.u8[1] = NewDscIp[1];
//			FW_u8_and_u32.u8[2] = NewDscIp[2];
//			FW_u8_and_u32.u8[3] = NewDscIp[3];
//			WriteC256(0x1024,FW_u8_and_u32.u8,2);
//	   		break;
//		case STARTANGLE:
//			FW_u8_and_u32.u32 = StartAngle;
//		    WriteC256(0x2000,FW_u8_and_u32.u8,4);
//			break;
//		case ENDANGLE:
//			FW_u8_and_u32.u32 = EndAngle;
//			WriteC256(0x2004,FW_u8_and_u32.u8,4);
//			break;
//		case ANGLE12:
//			FW_u8_and_f32.f32 = Angle12;
//			WriteC256(0x2008,FW_u8_and_f32.u8,4);
//			break;
//		case SIN_ANGLE12:
//			FW_u8_and_f32.f32 = Sin_Angle12;
//			WriteC256(0x200C,FW_u8_and_f32.u8,4);
//			break;
//		case COS_ANGLE12:
//			FW_u8_and_f32.f32 = Cos_Angle12;
//			WriteC256(0x2010,FW_u8_and_f32.u8,4);
//			break;
//		case HEIGHTLASER:
//			FW_u8_and_u32.u32 = HeightLaser;
//			WriteC256(0x2014,FW_u8_and_u32.u8,4);
//			break;
//		case LANEWIDE:
//			FW_u8_and_u32.u32 = LaneWide;
//			WriteC256(0x2018,FW_u8_and_u32.u8,4);
//			break;
////		case:
////			break;
////		case:
////			break;
////		case:
////			break;
//		default: 
//	       break;
		
//	}
}
//////////////////////////////////////////////////////////////////////////////////////////////////
/**********************************************
��������:ReStart_Record
�������ڣ�2012-8.15��hong
����˵������ÿ������ʱ��ȡ��д�������������洢��ַΪ���磺0x0000��
����ֵΪ����������
**********************************************/
uint32 ReStart_Record(void)
{
//	ReadC256(0x0000,FW_u8_and_u32.u8,4);
//	FW_u8_and_u32.u32 = FW_u8_and_u32.u32 + 1;
//	WriteC256(0x0000,FW_u8_and_u32.u8,4);
//	return FW_u8_and_u32.u32;
}
/*********************************************
�������ƣ�Test_Record
�������ڣ�2012-8-15
����˵������¼���ֲ������ݣ�
*********************************************/
//uint32 Timer_cou_JG0=0;
//uint32 Timer_cou_JG1=0;
//uint32 g_u32JG1_cou=0;
//uint32 g_u32JG0_count=0;
uint32 Test_Record_count = 0;

void Test_Record(void)
{
	
//    + Test_Record_count * 32 
//	FW_u8_and_u32.u32 = Timer_cou_JG0;
//	WriteC256(0x0100+ Test_Record_count * 32 ,FW_u8_and_u32.u8,4);
//	FW_u8_and_u32.u32 = Timer_cou_JG1;
//	WriteC256(0x0104+ Test_Record_count * 32 ,FW_u8_and_u32.u8,4);
//	FW_u8_and_u32.u32 = g_u32JG0_count;
//	WriteC256(0x0108+ Test_Record_count * 32 ,FW_u8_and_u32.u8,4);
//	FW_u8_and_u32.u32 = g_u32JG1_cou;
//	WriteC256(0x010C+ Test_Record_count * 32 ,FW_u8_and_u32.u8,4);
//	FW_u8_and_u32.u32 = g_u32count_Pro;
//	WriteC256(0x0110+ Test_Record_count * 32 ,FW_u8_and_u32.u8,4);
//	FW_u8_and_u32.u32 = g_total_count_Veh;
//	WriteC256(0x0114+ Test_Record_count * 32 ,FW_u8_and_u32.u8,4);
//	FW_u8_and_u32.u32 = g_u32SD_write_count;
//	WriteC256(0x0118+ Test_Record_count * 32,FW_u8_and_u32.u8,4);
//	FW_u8_and_u32.u32 = g_u32SD_err_count;
//	WriteC256(0x011C+ Test_Record_count * 32,FW_u8_and_u32.u8,4);
//	Test_Record_count = Test_Record_count + 1;
}
/********************************************************

********************************************************/
void FW_Read_test(uint32 a)
{
//	ReadC256(a,FW_u8_and_u32.u8,4);
}


