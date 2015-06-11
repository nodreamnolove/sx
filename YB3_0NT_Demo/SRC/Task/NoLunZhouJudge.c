#include "no_Axle.h"

/*弯板判别车型*/
// 20090520修改：根据秤台修改版本
//               在两轴车（总重和后轴轴重）和六轴车（总重）判别的时候需要用到重量的绝对值，注意弯板采用的是一侧重量（即总重的一半）
//               以下宏定义都变为秤台版本（20090513）的一半
//               #define    V2ALLWTUPPER		1500         // 两轴车总重上界
//               #define    V2ALLWTLOWER		750          // 两轴车总重下界
//               #define    V2AX2WTTHRESHOLD	800          // 两轴车后轴轴重12车判别界线
//               关键参数更改为弯板类型，注意buffer的大小和取值的起始点（buffer为10*10，取值其实点位1）

// Width表示台板宽度，单位是cm

int GetCarM(int *p, int AxleNum, int Width)
{
	return CARMDEFAULT;
}
