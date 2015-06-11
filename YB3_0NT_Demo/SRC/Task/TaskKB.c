/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			TaskKB.C
** Last modified Date:  20110511
** Last Version:		1.0
** Descriptions:		键盘任务
**
**--------------------------------------------------------------------------------------------------------
** Created by:			ZHANG Ye
** Created date:		20110511
** Version:				1.0
** Descriptions:		
**
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
** Version:
** Descriptions:
**
*********************************************************************************************************/
#define	__TASKKB_C
#include "TaskKB.h"

void  TaskRec3(void *tdata)
{
	uint8 err;
	tdata = tdata;               					
	OSTimeDly(10);	
#if	YBVERSION >= 30		//3.0仪表功能
	KeyboardInit();  		
#else	//2.2仪表功能
	//按键初始化
	g_u8KeyValueMapped		= 0;
	g_u32KeyValueOri		= 0;
	g_u32KeyCnt				= 0; 
#endif
	
	while(1)
	{
		OSSemPend(g_psemKey,0,&err);
		switch(g_u32KeyValueOri)
		{
			case 0x76:		//ESC	
				g_u8KeyValueMapped	= KB_ESC;		
				break;
				   
			case 0x5a:		//ENTER
			case 0xE05a:	//小键盘ENTER
				g_u8KeyValueMapped	= KB_ENTER;		
				break;
					   								     
			case 0x49:   	//.
			case 0x71:		//小键盘.
				g_u8KeyValueMapped	= KB_POINT;		
				break;
			
			case 0x45:		//0
			case 0x70:
				g_u8KeyValueMapped	= KB_0; 
				break;	
			case 0x16:		//1
			case 0x69:
				g_u8KeyValueMapped	= KB_1; 
				break;									
			case 0x1e:		//2
			case 0x72:
				g_u8KeyValueMapped	= KB_2; 
				break;				 					
			case 0x26:		//3
			case 0x7a:
				g_u8KeyValueMapped	= KB_3; 
				break;															
			case 0x25:		//4
			case 0x6b:
				g_u8KeyValueMapped	= KB_4;
				break;															
			case 0x2e:		//5
			case 0x73:
				g_u8KeyValueMapped	= KB_5;
				break;
														
			case 0x36:		//6
			case 0x74:
				g_u8KeyValueMapped	= KB_6;
				break;														
			case 0x3d:		//7
			case 0x6c:
				g_u8KeyValueMapped	= KB_7;
				break; 														
			case 0x3e:		//8
			case 0x75:
				g_u8KeyValueMapped	= KB_8;
				break;															
			case 0x46:		//9
			case 0x7d:
				g_u8KeyValueMapped	= KB_9;
				break;
					
			case 0x05:		//F1
				g_u8KeyValueMapped	= KB_F1;
				break;										
			case 0x06:		//F2
				g_u8KeyValueMapped	= KB_F2;
				break;					
			case 0x04:		//F3
				g_u8KeyValueMapped	= KB_F3;
				break;					
			case 0x0c:		//F4
				g_u8KeyValueMapped	= KB_F4;
				break;					
			case 0x03:		//F5
				g_u8KeyValueMapped	= KB_F5;
				break;					
			case 0x0b:		//F6
				g_u8KeyValueMapped	= KB_F6;
				break;
			case 0x83:		//F7
				g_u8KeyValueMapped	= KB_F7;
				break;
			case 0x0a:		//F8
				g_u8KeyValueMapped	= KB_F8;
				break;
//			case 0x01:		//F9
//				g_u8KeyValueMapped	= KB_F9;
//				break;
//			case 0x09:		//F10
//				g_u8KeyValueMapped	= KB_F10;
//				break;
//			case 0x78:		//F11
//				g_u8KeyValueMapped	= KB_F11;
//				break;
//			case 0x07:		//F12
//				g_u8KeyValueMapped	= KB_F12;
//				break;
			
			case 0x66:		//BackSpace
				g_u8KeyValueMapped	= KB_BKSP;
				break;
			
//			case 28:
//				g_u8KeyValueMapped	= KB_A;
//				break;					
//			case 50:
//				g_u8KeyValueMapped	= KB_B;
//				break;				
//			case 33:
//				g_u8KeyValueMapped	= KB_C;
//				break;				
//			case 35:
//				g_u8KeyValueMapped	= KB_D;
//				break;				
//			case 36:
//				g_u8KeyValueMapped	= KB_E;
//				break;				
//			case 43:
//				g_u8KeyValueMapped	= KB_F;
//				break;								
//			case 52:
//				g_u8KeyValueMapped	= KB_G;
//				break;				
//			case 51:
//				g_u8KeyValueMapped	= KB_H;
//				break;				
//			case 67:
//				g_u8KeyValueMapped	= KB_I;
//				break;				
			case 59:
				g_u8KeyValueMapped	= KB_J;
				break;				
//			case 66:
//				g_u8KeyValueMapped	= KB_K;
//				break;				
//			case 75:
//				g_u8KeyValueMapped	= KB_L;
//				break;					
//			case 58:
//				g_u8KeyValueMapped	= KB_M;
//				break;					
//			case 49:
//				g_u8KeyValueMapped	= KB_N;
//				break;				
//			case 68:
//				g_u8KeyValueMapped	= KB_O;
//				break;				
//			case 77:
//				g_u8KeyValueMapped	= KB_P;
//				break;				
//			case 21:
//				g_u8KeyValueMapped	= KB_Q;
//				break;					
//			case 45:
//				g_u8KeyValueMapped	= KB_R;
//				break;
//			case 27:
//				g_u8KeyValueMapped	= KB_S;
//				break;
//			case 44:
//				g_u8KeyValueMapped	= KB_T;
//				break;
//			case 60:
//				g_u8KeyValueMapped	= KB_U;
//				break;
//			case 42:
//				g_u8KeyValueMapped	= KB_V;
//				break;
//			case 29:
//				g_u8KeyValueMapped	= KB_W;
//				break;
//			case 34:
//				g_u8KeyValueMapped	= KB_X;
//				break;
//			case 53:
//				g_u8KeyValueMapped	= KB_Y;
//				break;
//			case 26:
//				g_u8KeyValueMapped	= KB_Z;
//				break;
			default:
				g_u8KeyValueMapped	= 0xff;
				break;
		}   					 
//		OSSemAccept(g_psemScreenRefresh);
//		OSSemPost(g_psemScreenRefresh);
        
        OSTimeDly(5);                

		g_u32KeyValueOri	= 0;
		g_u32KeyCnt			= 0;		 

	}
}




