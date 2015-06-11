#ifndef _NOAXLE_H
#define _NOAXLE_H
#include "JZGlobal.h"
#define    min_my(a, b)    (((a)<=(b)) ? (a):(b))
#define    abs_my(a)       ((a>=0) ? a:-(a))
// 车型
#define    CARMDEFAULT      0x00000000   // 默认车型
#define    CARM11           0x0000000B
#define    CARM12           0x0000000C
#define    CARM15           0x0000000F
#define    CARM112          0x00000070
#define    CARM122          0x0000007A
#define    CARM114          0x00000072
#define    CARM115          0x00000073
#define    CARM125          0x0000007D
#define    CARM118          0x00000076
#define    CARM119          0x00000077
#define    CARM128          0x00000080
#define    CARM129          0x00000081
#define    CARM155          0x0000009B
#define    CARM1125         0x00000465
#define    CARM149          0x00000095
#define    CARM159          0x0000009F
#define    CARM1129         0x00000469
#define    CARM11151        0x00002B8F
#define    CARM12151        0x00002F77

// 判别界限
#define		V2ALLWTUPPER		1500		// 两轴车总重上界
#define		V2ALLWTLOWER		750			// 两轴车总重下界
#define		V2AX2WTTHRESHOLD	800			// 两轴车后轴轴重12车判别界线

#define		AXGRPDISTHRESHOLD	190 * POINTRATE		// 联轴轴距 1.9m
//#define	AXLELDISB			170 * POINTRATE		// 联轴轴距判别下界 1.7m
//#define	AXLELDISP			210 * POINTRATE		// 联轴轴距判别上界 2.1m

#define		AX1DISTHRESHOLD		270 * POINTRATE		// 第一轴轴距通用阈值 2.7m
#define		AX1DISLOWER			245 * POINTRATE		// 第一轴轴距判别下界,用于判断11X与12X 2.45m
#define		AX1DISUPPER			295 * POINTRATE		// 第一轴轴距判别上界,用于判断11X与12X 2.95m

// 判车型函数
// Width表示台板宽度，单位是cm
int GetCarM(int *p, int AxleNum, int Width);

#endif
