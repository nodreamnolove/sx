#include "config.h"
extern void Storage_Write_TDC256(uint8 p_coef);
extern uint32 ReStart_Record(void);
extern void FW_Read_test(uint32 a);

#define SD_TEST_WRITE  		1//SD
#define TOTAL_COUNT_VEH  	2//�ܳ���g_total_count_Veh

#define	RDID				3  //0x1000	RDid�豸���ʶ����	17
#define RDNUM				4  //0x1011	RDNumվ����	15
#define PROCYCLE			5  //0x1020	ProCycle��������	2
#define DISTIME				6 //0x1022	DisTime�����ٷֱȼ���ʱ��	2
#define NEWDSCIP			7 //0x1024	NewDscIp	4


#define STARTANGLE			11	//0x2000	StartAngle��ʼ�Ƕ�	4
#define ENDANGLE			12	//0x2004	EndAngle��ֹ�Ƕ�	4
#define	ANGLE12				13 //0x2008	Angle12����н�	4
//#define SIN_ANGLE12			14	//0x200C	Sin_Angle12����н�sin	4
//#define	COS_ANGLE12			15	//0x2010	Cos_Angle12����н�cos	4
#define HEIGHTLASER			16	//0x2014	HeightLaser��ֱ����߶�	4
#define LANEWIDE			17	//0x2018	LaneWide�������	4










			   


