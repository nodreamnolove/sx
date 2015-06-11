/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			Task_Checknet.C
** Last modified Date:  20120718
** Last Version:		1.0
** Descriptions:		网络连接任务
**
**--------------------------------------------------------------------------------------------------------
** Created by:			Hong XiangYuan
** Created date:		20120718
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
#include "Task_Checknet.h"
#include "WT_Task.h"
#include "W5100.h"
#include "W5100App.h"

#define		SETUPALIAS				g_sspSetup

void  Task_Checknet(void *tdata)	 
{
	uint32   l_u32Linknum;
//    NetInfo  Test;

	tdata	= tdata;		 
////============测试开始=====================================
//	Test.au8SubMask[3]=255;
//	Test.au8SubMask[2]=255;
//	Test.au8SubMask[1]=255;
//	Test.au8SubMask[0]=0;	
//	Test.au8IPAddr[3]=192;
//	Test.au8IPAddr[2]=168;
//	Test.au8IPAddr[1]=0;
//	Test.au8IPAddr[0]=111;
//    Test.au8GatewayIP[3]= 192;
//	Test.au8GatewayIP[2]= 168;
//	Test.au8GatewayIP[1]=  0;
//	Test.au8GatewayIP[0]=  1;
//
//    Test.au8ServerIP1[3]=192; 
//	Test.au8ServerIP1[2]=168; 
//	Test.au8ServerIP1[1]= 0;
//	Test.au8ServerIP1[0]= 2;
//    Test.au8ServerIP2[3]=192; 
//	Test.au8ServerIP2[2]=168; 
//	Test.au8ServerIP2[1]= 0;
//	Test.au8ServerIP2[0]= 3;	
//	
//	Test.u32LocalPortNO=4000;
//
//	Test.u32ServerPortNO1=2110;
//	Test.u32ServerPortNO2=2112;
//	Test.au8MACAddr[0]=0x52;
//	Test.au8MACAddr[1]=0x54;
//	Test.au8MACAddr[2]=0x4c;
//	Test.au8MACAddr[3]=0x19;
//	Test.au8MACAddr[4]=0xf7;
//	Test.au8MACAddr[5]=0x55;
//	InitializeW5100(&Test);
//	memcpy(&g_sniLocal, &Test, 38);
//	InitializeW5100(&g_sniLocal);	
//	//===测试用结束===
//	memcpy(&g_sniLocal, (uint8 *)&SETUPALIAS.VerticalLaser_IP, 38);
//	
//	InitializeW5100(&g_sniLocal);//

	OSTimeDly(1250); 	
	Flag_NetConnect = 1;
	Flag_NetToPC    = 1;
	S0_Data|=S_TRANSMITOK;	
	S1_Data|=S_TRANSMITOK;
	S2_Data|=S_TRANSMITOK;
	S3_Data|=S_TRANSMITOK;
//=================================================
		P3_OUTP_SET = (1 << 9);	 //灭
		P3_OUTP_CLR = (1 << 9);	 //亮
//================ ==================================
	while(1)
	{	
		 
	    if(W5100_LINK)
		{ 		      
			l_u32Linknum++;
			OSTimeDly(5);
			if(l_u32Linknum>100)
			{
				P3_OUTP_SET = (1 << 9);	 //灭 
				l_u32Linknum=0;
				Flag_NetConnect = 0;
				InitializeW5100(&g_sniLocal);
				OSTimeDly(1250);				  //延时不可删除，删除后连接不上 dxin
				l_u32Linknum = 0;
				Flag_NetConnect = 0;			//网络连接
				Flag_NetToPC = 0; 
			}
#ifndef SIM_SOFTWARE
		  	if(((S0_State & S_CONN) == S_CONN))
			{
				Flag_NetConnect=1;			//认定网络连接正常	 						
			}  
			if((S1_State & S_CONN) == S_CONN)
			{
				Flag_NetConnect=1;
			}
			if((S2_State & S_CONN) == S_CONN)
			{
				Flag_NetConnect=1;
			}
			if((S3_State & S_CONN) == S_CONN)
			{
				Flag_NetConnect=1;
			}
#else
			if((S0_State & S_CONN))
			{
				Flag_NetConnect=1;			//认定网络连接正常	 						
			}  
			if((S2_State & S_CONN))
			{
				Flag_NetConnect=1;			//认定网络连接正常	 						
			} 
		 
#endif
		  }
		  else
		  {		
				P3_OUTP_CLR = (1 << 9);	 //亮
				l_u32Linknum=0;
#ifndef SIM_SOFTWARE

				if(((S0_State & S_CONN) == S_CONN) || ((S1_State & S_CONN) == S_CONN)||((S2_State & S_CONN) == S_CONN)||((S3_State & S_CONN) == S_CONN)) 
				{
					Flag_NetConnect	 = 1;
				}
				else
				{
					Flag_NetConnect = 0;
					if(((S0_State & S_CONN) != S_CONN))	 
					{
						SETUPALIAS.u32Net1_DisconnectNum++;
						S0_State = 0;
						SetSocket();
						OSTimeDly(30);				
					}

				   	if(((S1_State & S_CONN) != S_CONN))	 
					{
						SETUPALIAS.u32Net2_DisconnectNum++;
						S1_State = 0;
						SetSocket();
						OSTimeDly(30);				
					} 
					if(((S2_State & S_CONN) != S_CONN))	 
					{
				//		SETUPALIAS.u32Net2_DisconnectNum++;
						S2_State = 0;
						SetSocket();
						OSTimeDly(30);				
					} 
					if(((S3_State & S_CONN) != S_CONN))	 
					{
				//		SETUPALIAS.u32Net2_DisconnectNum++;
						S3_State = 0;
						SetSocket();
						OSTimeDly(30);				
					} 	
				}

#else
 				if(((S0_State & S_CONN) == S_CONN)||(S2_State & S_CONN == S_CONN)) 
				{
					Flag_NetConnect	 = 1;  //
				}
				else
				{
					Flag_NetConnect = 0;
					{
						SETUPALIAS.u32Net1_DisconnectNum++;
						S0_State = 0;
						SetSocket();
						OSTimeDly(30);				
					}
					if(((S2_State & S_CONN) != S_CONN))	 
					{
						SETUPALIAS.u32Net3_DisconnectNum++;
						S2_State = 0;
						SetSocket();
						OSTimeDly(30);				
					}
				}

//			  	if((S2_State & S_CONN) != S_CONN)
//				{							    
//					S2_State = 0;
//					SetSocket();
//					OSTimeDly(3);
//				}
//				else
//				{
//					Flag_NetConnect = 1;
//				}
#endif
//				if((S3_State & S_CONN)== S_CONN)  //用于有线网络连接的端口
//				{
//					Flag_NetToPC = 1;
//				}
//				else
//				{
//					S3_State = 0 ;
//					SetSocket();
//					OSTimeDly(3);
//				}
//					
//				OSTimeDly(100);		

		  }
	}
}
//======================	

//shijian	02 02 02 02 00 00 00 1E 73 4D 4E 20 4C 53 50 73 65 74 64 61 74 65 74 69 6D 65 20 07 DC 04 12 0A 00 00 00 00 00 00 A3 
//uint8 CMD_Settime[39]={0x02,0x02,0x02,0x02,0x00,0x00,0x00,0x1E,0x73,0x4D,0x4E,0x20,0x4c,0x53,0x50,0x73,0x65,0x74,0x64,0x61,0x74,0x65,0x74,
//0x69,0x6D,0x65,0x20,0x07,0xD9,0x02,0x11,0x10,0x22,0x00,0x00,0x00,0x00,0x00,0xA3};
//uint8 CMD_Asktime[18]={0x02,0x02,0x02,0x02,0x00,0x00,0x00,0x09,0x73,0x52,0x4E,0x20,0x53,0x54,0x6C,0x6D,0x73,0x3A};
//02 02 02 02 00 00 00 17 73 4D 4E 20 53 65 74 41 63 63 65 73 73 4D 6F 64 65 20 03 F4 72 47 44 B3
//uint8  CMD_Login[32]={0x02,0x02,0x02,0x02,0x00,0x00,0x00,0x17,0x73,0x4D,0x4E,0x20,0x53,0x65,0x74,0x41,0x63,0x63,
//       0x65,0x73,0x73,0x4D,0x6F,0x64,0x65,0x20,0x03,0xF4,0x72,0x47,0x44,0xB3};//03 authorised client  Password:client
//uint8  CMD_Run[16]={0x02,0x02,0x02,0x02,0x00,0x00,0x00,0x07,0x73,0x4D,0x4E,0x20,0x52,0x75,0x6E,0x19};
//	uint16 RecevieData_Buf[1460];
//	uint8    Flag_NetConnect_00,Flag_NetConnect_01,Flag_NetConnect_02;  
