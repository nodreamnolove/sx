#include "config.h"
extern void Storage_Write_TDC256(uint8 p_coef);
extern uint32 ReStart_Record(void);
extern void FW_Read_test(uint32 a);

#define SD_TEST_WRITE  		1//SD
#define TOTAL_COUNT_VEH  	2//总车数g_total_count_Veh

#define	RDID				3  //0x1000	RDid设备身份识别码	17
#define RDNUM				4  //0x1011	RDNum站点编号	15
#define PROCYCLE			5  //0x1020	ProCycle调查周期	2
#define DISTIME				6 //0x1022	DisTime跟车百分比鉴别时间	2
#define NEWDSCIP			7 //0x1024	NewDscIp	4


#define STARTANGLE			11	//0x2000	StartAngle开始角度	4
#define ENDANGLE			12	//0x2004	EndAngle终止角度	4
#define	ANGLE12				13 //0x2008	Angle12激光夹角	4
//#define SIN_ANGLE12			14	//0x200C	Sin_Angle12激光夹角sin	4
//#define	COS_ANGLE12			15	//0x2010	Cos_Angle12激光夹角cos	4
#define HEIGHTLASER			16	//0x2014	HeightLaser垂直激光高度	4
#define LANEWIDE			17	//0x2018	LaneWide车道宽度	4










			   


