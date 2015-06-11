
#include "config.h"


//#define    MAXDAFEIPTNUM    4
//#define IS_INSIDE(x,y,dx,dy)	( (min(dx,dy) >= max(x,y) || max(dx,dy) <= min(x,y)) ? 0:1 )
//#define max(x,y)	(x > y ? x:y)
//#define min(x,y)	(x > y ? y:x)
//#define MAX_U(x,y)	(abs(x) > abs(y) ? x:y)
//#define MIN_U(x,y)	(abs(x) > abs(y) ? y:x)

extern void Get_Vehicle_Info(void);

void VehModels2(VehicleStruct *pVehicle);

extern int32 Get_shiju(int32 *p,int32 *q);
extern uint8 IsInIncSide(int32 x11, int32 x12, int32 x21, int32 x22);
extern uint8 GetLightVehPattern(VehicleStruct *pVehicle);  //����С�ͳ�����
extern uint8 GetLargeVehPattern(VehicleStruct *pVehicle);   //������ͳ�����
extern uint16 GetLimitValue(int* pg_ZdistanceI,int* pXdata, int len,int startPos, uint8 u8Flag);    //��װ��ʱ��ú��������޸�	u8FlagΪ0��ʾ��ֱ���������ݣ�1��ʾ��б����������

extern int get_vehicle_speed(int m, int Speedline);
extern int get_vehicle_length(int m);


extern void sendTmpData(int *data,int len);

extern void LMS_Data_Process(uint8 *p,uint8 *q);

extern int32  g_ai32Pre_Veh_Info_1_Lane[26];  //����1����ǰһ��������Ϣ��
extern int32  g_ai32Pre_Veh_Info_2_Lane[26];  //����2����ǰһ��������Ϣ��
extern int32  g_ai32Pre_Veh_Info_3_Lane[26];  //����3����ǰһ��������Ϣ��
extern int32  g_ai32Pre_Veh_Info_4_Lane[26];  //����4����ǰһ��������Ϣ��
extern int32  g_ai32Pre_Veh_Info_5_Lane[26];  //����5����ǰһ��������Ϣ��
extern int32  g_ai32Pre_Veh_Info_6_Lane[26];  //����6����ǰһ��������Ϣ��
extern int32  Veh_Info[25]; //��ǰ��������Ϣ

extern uint8 VehInfo_Buffer[19]; //����ͨ��Uart5���ͳ�����Ϣ
	  
							   //�ĳ����ֿ���
extern uint32 g_total_veh[6][9]; //�ų���������С�ͳ�	С����	��ͳ�	���ͻ���	���ͻ���	�ش��ͻ���	��װ�䳵	������	Ħ�г�
extern uint32 g_total_veh_temp[6][9];
extern uint32 g_speed_veh_sum[6][9]; //�ų����ٶ��ۼӣ�С�ͳ�	С����	��ͳ�	���ͻ���	���ͻ���	�ش��ͻ���	��װ�䳵	������	Ħ�г�
extern uint32 g_total_Lane[6];// �ĳ������Գ���������
extern uint32 g_average_shiju[6]; //��������ͷʱ������
extern uint32 g_sum_shiju[6] ;	  //��ռ��ʱ��
extern uint32 g_sum_shijian_share[6]; //ʱ��ռ���� .
extern uint32 g_sum_shijian[6];
extern uint32 g_total_genche[6];//����������
extern uint32 g_percent_genche[6];//�����ٷֱȣ�
extern uint32 g_average_jianju[6]; //ƽ����������ͷ���

extern _Sveh_Que Sveh_Que;
