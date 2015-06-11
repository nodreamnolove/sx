
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
extern uint8 GetLightVehPattern(VehicleStruct *pVehicle);  //计算小型车车型
extern uint8 GetLargeVehPattern(VehicleStruct *pVehicle);   //计算大型车车型
extern uint16 GetLimitValue(int* pg_ZdistanceI,int* pXdata, int len,int startPos, uint8 u8Flag);    //侧装的时候该函数还有修改	u8Flag为0表示垂直激光器数据，1表示倾斜激光器数据

extern int get_vehicle_speed(int m, int Speedline);
extern int get_vehicle_length(int m);


extern void sendTmpData(int *data,int len);

extern void LMS_Data_Process(uint8 *p,uint8 *q);

extern int32  g_ai32Pre_Veh_Info_1_Lane[26];  //保存1车道前一辆车的信息；
extern int32  g_ai32Pre_Veh_Info_2_Lane[26];  //保存2车道前一辆车的信息；
extern int32  g_ai32Pre_Veh_Info_3_Lane[26];  //保存3车道前一辆车的信息；
extern int32  g_ai32Pre_Veh_Info_4_Lane[26];  //保存4车道前一辆车的信息；
extern int32  g_ai32Pre_Veh_Info_5_Lane[26];  //保存5车道前一辆车的信息；
extern int32  g_ai32Pre_Veh_Info_6_Lane[26];  //保存6车道前一辆车的信息；
extern int32  Veh_Info[25]; //当前车辆的信息

extern uint8 VehInfo_Buffer[19]; //用于通过Uart5发送车辆信息
	  
							   //四车道分开；
extern uint32 g_total_veh[6][9]; //九车型总数；小客车	小货车	大客车	中型货车	大型货车	特大型货车	集装箱车	拖拉机	摩托车
extern uint32 g_total_veh_temp[6][9];
extern uint32 g_speed_veh_sum[6][9]; //九车型速度累加；小客车	小货车	大客车	中型货车	大型货车	特大型货车	集装箱车	拖拉机	摩托车
extern uint32 g_total_Lane[6];// 四车道各自车辆总数；
extern uint32 g_average_shiju[6]; //机动车车头时距数据
extern uint32 g_sum_shiju[6] ;	  //总占有时间
extern uint32 g_sum_shijian_share[6]; //时间占有率 .
extern uint32 g_sum_shijian[6];
extern uint32 g_total_genche[6];//跟车总数；
extern uint32 g_percent_genche[6];//跟车百分比；
extern uint32 g_average_jianju[6]; //平均机动车车头间距

extern _Sveh_Que Sveh_Que;
