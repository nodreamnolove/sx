#ifndef _NOAXLE_H
#define _NOAXLE_H
#include "JZGlobal.h"
#define    min_my(a, b)    (((a)<=(b)) ? (a):(b))
#define    abs_my(a)       ((a>=0) ? a:-(a))
// ����
#define    CARMDEFAULT      0x00000000   // Ĭ�ϳ���
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

// �б����
#define		V2ALLWTUPPER		1500		// ���ᳵ�����Ͻ�
#define		V2ALLWTLOWER		750			// ���ᳵ�����½�
#define		V2AX2WTTHRESHOLD	800			// ���ᳵ��������12���б����

#define		AXGRPDISTHRESHOLD	190 * POINTRATE		// ������� 1.9m
//#define	AXLELDISB			170 * POINTRATE		// ��������б��½� 1.7m
//#define	AXLELDISP			210 * POINTRATE		// ��������б��Ͻ� 2.1m

#define		AX1DISTHRESHOLD		270 * POINTRATE		// ��һ�����ͨ����ֵ 2.7m
#define		AX1DISLOWER			245 * POINTRATE		// ��һ������б��½�,�����ж�11X��12X 2.45m
#define		AX1DISUPPER			295 * POINTRATE		// ��һ������б��Ͻ�,�����ж�11X��12X 2.95m

// �г��ͺ���
// Width��ʾ̨���ȣ���λ��cm
int GetCarM(int *p, int AxleNum, int Width);

#endif
