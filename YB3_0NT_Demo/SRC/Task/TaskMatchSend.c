#include "TaskMatchSend.h"
#include "config.h"
#define MAX_VEH	5
 extern OS_EVENT  *g_sendVeh;
extern uint32 g_u32VehWideHeigh[10][10];  //车辆宽高
extern uint16 g_u16Recvcount;
extern uint16 g_u16DealCount;


extern uint32 g_Veh_Que2[MAX_VEH][5];
extern uint8 g_rInd_2 ;
extern uint8 g_wInd_2 ;
extern uint32 g_Veh_Que3[MAX_VEH][5];
extern uint8 g_rInd_3 ;
extern uint8 g_wInd_3 ;
extern uint32 t0_count2;
extern uint32 g_count;
void Task_Match_Send(void *tdata)
{
	uint8 err;
	uint8 test[3];
	
	uint32 read_add,index;
	uint32 l_temp;
	uint32 l_lane;
	uint8 l_count;
	uint32 l_tempv = 0;
	uint32 vehLen = 0;
	uint8 Sv_buf[60] = {0};
	uint8 U5Buff[20]={0};
	WDTIM_COUNTER	= 1;									/* 喂狗							*/
	
	tdata = tdata;
	while(1)
	{
			OSSemPend(g_sendVeh,0,&err);
			vehLen = 0;
			if(g_u16Recvcount!= g_u16DealCount)
			{
				U5Buff[0] = '$';
				U5Buff[1] = g_u32VehWideHeigh[g_u16DealCount][1]; //车道
				U5Buff[2] = 0;		
				U5Buff[3] = 0;
				U5Buff[4] = 0;		
				U5Buff[5] = 0;
				U5Buff[6] = g_u32VehWideHeigh[g_u16DealCount][2]/10000; //宽
				l_tempv = g_u32VehWideHeigh[g_u16DealCount][2]%10000;
				U5Buff[7] = l_tempv/1000; //宽
				l_tempv = l_tempv%1000;
				U5Buff[8] = l_tempv/100; //宽
				l_tempv = l_tempv%100;
				U5Buff[9] = l_tempv/10; //宽
				U5Buff[10] = g_u32VehWideHeigh[g_u16DealCount][3]/10000;
				l_tempv = g_u32VehWideHeigh[g_u16DealCount][3]%10000;
				U5Buff[11] = l_tempv/1000;
				l_tempv = l_tempv%1000; 
				U5Buff[12] = l_tempv/100; //高
				l_tempv = l_tempv%100;
				U5Buff[13] = l_tempv/10; //
#ifndef SIM_SOFTWARE 
			   	U5Buff[14] = 0x40;// 0x40;
#else
				U5Buff[14] = g_count;// 0x40;
#endif
				l_lane = g_u32VehWideHeigh[g_u16DealCount][1];
				if(l_lane == 1)
				{
					l_count = g_rInd_2;
					while( l_count != g_wInd_2)
					{
						//  g_rInd_2 = (g_rInd_2+1)%5;
						  if( abs(g_u32VehWideHeigh[g_u16DealCount][0]- g_Veh_Que2[l_count][0])<700)
						   {
							//输出长宽高
									//出车	
								vehLen =   g_Veh_Que2[l_count][2]; 
								g_rInd_2 = (l_count+1)%5;					
								break;
							}	
							else
							{
								l_count	 = (l_count+1)%5;	
							}								
					} 
				}
				else if(l_lane == 2)
				{
					l_count = g_rInd_3;	
					while( l_count != g_wInd_3)
					{
						 
						  if( abs(g_u32VehWideHeigh[g_u16DealCount][0]- g_Veh_Que3[l_count][0])<700)//时间在1s以内
						   {
							//输出长宽高
									//出车
								vehLen =   g_Veh_Que3[l_count][2];
								g_rInd_3 = (l_count+1)%5;					
								break;
							}
							else
							{								    
								l_count	 = (l_count+1)%5;   
							}		  						
					}				 
				}
				else
				{
				}
//				if(	vehLen == 0) //随机出数
//				{
//					 if(g_u32VehWideHeigh[g_u16DealCount][3]<1500)
//					 {
//					     vehLen = Myrand(3900,4300);
//					 }
//					 else if(g_u32VehWideHeigh[g_u16DealCount][3]<1700)
//					 {
//					  	vehLen = Myrand(4100,4500);
//					 }
//					 else if(g_u32VehWideHeigh[g_u16DealCount][3]<2000)
//					 {
//					    vehLen = Myrand(4300,5000);
//					 }
//					 else if(g_u32VehWideHeigh[g_u16DealCount][3]<2200)
//					 {
//					    vehLen = Myrand(4000,5500);
//					 }
//					 else if(g_u32VehWideHeigh[g_u16DealCount][3]<2400)
//					 {
//					    vehLen = Myrand(5000,7000);
//					 }
//					 else
//					   vehLen = Myrand(5000,10000);				
//				}			  
			   U5Buff[2] = vehLen/10000;
			   l_tempv = vehLen%10000;		
			   U5Buff[3] = l_tempv/1000;
			   l_tempv = l_tempv%1000;
			   U5Buff[4] = l_tempv/100;
			   l_tempv = l_tempv%100;
			   U5Buff[5] = l_tempv/10;
			   if(g_sspSetup.u32DevID==2)
			   {
			  	 U5Buff[1] = 0x03^U5Buff[1];
			   }	
			   U5SendBytes(U5Buff, 15);
			   memset(g_u32VehWideHeigh[g_u16DealCount],0,10*sizeof(uint32));
			   g_u16DealCount = ( g_u16DealCount + 1 )%10;
			}	
	
	}
}

