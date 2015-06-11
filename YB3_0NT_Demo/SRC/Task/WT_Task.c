/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			WT_Task.C
** Last modified Date:  20110511
** Last Version:		1.0
** Descriptions:		��������
**
**--------------------------------------------------------------------------------------------------------
** Created by:			ZHANG Ye
** Created date:		20110511
** Version:				1.0
** Descriptions:		
**
**--------------------------------------------------------------------------------------------------------
** Modified by:			Hong Xiang Yuan
** Modified date:		20120718
** Version:
** Descriptions:		TaskStart;��������
**																	 									   								 
*********************************************************************************************************/
#define	__JZGLOBAL
#define	__WT_TASK_C
#include "WT_Task.h"
#include "CRC.h"
#include "common.h"
//#include "send_task.h"
#include "UART5.h"
#include "rd_data.h"
#include "Timer0.h"
#include "CMD.h"
#include "sdconfig.h"
#include "sdcommon.h"
#include "sddriver.h"
#include "FAT32APP.h"
#include "JG.h"
#include "rd_data.h"
//#include "Task_TiPo.h"
#include "Task_SendUart1.h"
#include "FW.h"
#include "Protocol.h"
//#include "Task_SD.h"
#include "TDC256.h"
#include "uart1.h"
#include "TaskMatchSend.h"

#define ResetSystem()  			{WDTInit(1),WDTIM_COUNTER =  0x10000000-100;while(1); 	}

uint8 RdToLMSSendData[24] = {0x02,0x02,0x02,0x02,0x00,0x00,0x00,0x0F,0x73,0x52,0x4E,0x20,0x4C,0x4D,0x44,0x73,0x63,0x61,0x6E,0x64,0x61,0x74 
,0x61,0x05};
uint8 LianxuLMSSendData[26]={0x02,0x02,0x02,0x02,0x00,0x00,0x00,0x11,0x73,0x45,0x4E,0x20,0x4C,0x4D,0x44,0x73,0x63,0x61,0x6E,0x64,0x61,0x74 
,0x61,0x20,0x01,0x33};
//02 02 02 02 00 00 00 11 73 45 4E 20 4C 4D 44 73 63 61 6E 64 61 74 61 20 01 33

uint32	sv_write_sd_add = 0;

uint8  Send_data02[18];
uint8  Send_data08[65];
uint8  Send_data01_temp[500]; 

uint32	g_u32JG1_Timermiss_count=0;//����1�ӵ����ݣ�����0����ʱ��ÿ�μ�1��
//uint8 	SD_Buff_Send_VehInfo_Uart1[10][55]={0};
//uint32 	SD_store_count=0;
uint32  SD_pro_count  =0;
uint8   Flag_SD_Init_err   =0;//SD����ʼ��ʧ�ܣ�																	


#define		SETUPALIAS				g_sspSetup			//���ò����ṹ

uint8  	g_u8Flag_wireless=1;
uint8 	Flag_NetConnect;
uint32   g_au32Tempa[5000][4];
//extern OS_EVENT *continue_flag;
uint8   Flag_NetToPC;
OS_EVENT *FW_flag;
//uint8  S_08[73]={0xAA,0xAA,0x49,0x00,0x08,0x30,0x30,0x31,0x32,0x31,0x32,0x31,0x31,0x31,0x30,0x30,0x31,0x30,0x30,0x30,0x31,0xD6,0x07,0x05,0x05,
//0x05,0x05,0x05,0x05,0x01,0xC0,0xA8,0x00,0x6F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x01,
//0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0X6E,0x22,0X58,0X25,0xEE,0xEE};

/*		������������		*/
void	RunStartTask(void)
{
	OSTaskCreate(TaskStart,(void *)0, &TaskStartStk[TASKSTACKSIZE-1], TASKSTARTPRIO);  	
}
/*		��ʼ��		*/

void	JZInit(void)
{
	g_u32count_Pro = 0;
	g_u32cout_Pro_Two_Buff = 0;
	g_u8flag_veh = 0;
	g_total_count_Veh = 0;
//	memset(g_Base_data0_Value,0,sizeof(int32)*POINT_SUM);
//	memset(g_Base_data0_Cnt,0,sizeof(uint8)*POINT_SUM);
//	memset(g_Base_data1_Value,0,sizeof(int32)*POINT_SUM);
//	memset(g_Base_data1_Cnt,0,sizeof(uint8)*POINT_SUM);
	Sin_Angle12= 0.0;// 0.6635;
	Cos_Angle12= 0.0;//0.7482;
//	StartAngle	=	19;
//	EndAngle	=	161;
	g_u8Jg_4_Buff_Count =0;
	g_u32Two_Buff_cout=0;


	SETUPALIAS.J0_Height = 5770;                           //��������ֱ�߶�ֵ
	SETUPALIAS.J1_Height = 5660;                           //��������ֱ�߶�ֵ
	SETUPALIAS.J2_Height = 6050;                           //��������ֱ�߶�ֵ
	SETUPALIAS.J3_Height = 5900;                           //��������ֱ�߶�ֵ	  	  
//	SETUPALIAS.IncHeightLaser = 0;
	SETUPALIAS.LaserDistance = 9670;	                    //������֮�����
//	SETUPALIAS.Angle12 = 45;						    	//��б�������ĽǶ�
	SETUPALIAS.LaneWide = 7700;                              //%�������
	SETUPALIAS.MedianWide = 900;                       ////�����ұ�����
	SETUPALIAS.MedianLeftWide = 950;         	            ////����������� MedianWide- MedianLeftWide < LaserDistance
	SETUPALIAS.u8DOG  = 1;

	SETUPALIAS.resetCnt = 0;  
	SETUPALIAS.u8BaudRate = 5;
	SETUPALIAS.J0_IP = (192<<24)+(168<<16)+(0<<8)+2;
	SETUPALIAS.J0_Port	= 2110;
	SETUPALIAS.J1_IP = (192<<24)+(168<<16)+(0<<8)+3;
	SETUPALIAS.J1_Port = 2112;
	SETUPALIAS.J2_IP = (192<<24)+(168<<16)+(0<<8)+4;
	SETUPALIAS.J2_Port	= 2114;	
	SETUPALIAS.J3_IP = (192<<24)+(168<<16)+(0<<8)+5;
	SETUPALIAS.J3_Port	= 2116;	
	SETUPALIAS.u32LocalIPAddress = (192<<24)+(168<<16)+(0<<8)+111;
	SETUPALIAS.u32SubMask = (255<<24)+(255<<16)+(255<<8)+0;
	SETUPALIAS.u32GatewayIP	= (192<<24)+(168<<16)+(0<<8)+1;
	SETUPALIAS.u32LocalPortNO =  4000;
	SETUPALIAS.au8LocalMAC[0] =	0x52;
 	SETUPALIAS.au8LocalMAC[1] =	0x54;
	SETUPALIAS.au8LocalMAC[2] =	0x4c;
	SETUPALIAS.au8LocalMAC[3] =	0x19;
	SETUPALIAS.au8LocalMAC[4] =	0xf7;
	SETUPALIAS.au8LocalMAC[5] =	0x55;
	
	
	SETUPALIAS.u32Net1_DisconnectNum = 0;
	SETUPALIAS.u32Net2_DisconnectNum = 0;
	SETUPALIAS.u32Net1_InvalidRecNum = 0;
	SETUPALIAS.u32Net2_InvalidRecNum = 0;
//	SETUPALIAS.u32DataProcException = 0;
	memcpy(SETUPALIAS.au8ProgramVersion,JGJD_VERSION,11);

	SETUPALIAS.u8LaserDevType = WJJGDATA;
	SETUPALIAS.u8TrafficType = FIRSTDEVTYPE;//
	SETUPALIAS.u8RoadType = 0;	     // ��·���� 0-- ���� 1-- ����
	SETUPALIAS.u8LaneNum = 4;	     //������
	SETUPALIAS.u8InstallFlag = 0;    //0��װ��1��װ
	SETUPALIAS.u8NetType  = 0;
	SETUPALIAS.u8SDEnable = 0;  //0��SD�����ܴ洢�������ݣ�1��ʾ��

	SETUPALIAS.u16J0ZeroPos = 180;   //Ĭ�ϴ�ֱ������0��λ��		  20130426
	SETUPALIAS.u16J1ZeroPos = 160;   //Ĭ�ϴ�ֱ������0��λ��		  20130426
	SETUPALIAS.u16J2ZeroPos = 175;   //Ĭ�ϴ�ֱ������0��λ��		  20130426
	SETUPALIAS.u16J3ZeroPos = 189;   //Ĭ�ϴ�ֱ������0��λ��		  20130426
//	SETUPALIAS.u16InclineZeroPos  = 180;   //Ĭ����б������0��λ��
	SETUPALIAS.u16J0StartPos      = 50;    //Ĭ����ʼ����
	SETUPALIAS.u16J0EndPos        = SETUPALIAS.u16J0ZeroPos;   //Ĭ����ֹ����
	SETUPALIAS.u16J1StartPos      = SETUPALIAS.u16J1ZeroPos;    //Ĭ����ʼ����
	SETUPALIAS.u16J1EndPos        = 290;   //Ĭ����ֹ����
	SETUPALIAS.u16J2StartPos      = 70;    //Ĭ����ʼ����
	SETUPALIAS.u16J2EndPos        = 170;   //Ĭ����ֹ����
	SETUPALIAS.u16J3StartPos      = 70;    //Ĭ����ʼ����
	SETUPALIAS.u16J3EndPos        = 185;   //Ĭ����ֹ����


}

/* JGCheck_Para() ����:��鼤������Ƿ����������
 * ��������: ��
 * ��������ֵ: �������� ���� FALSE 0
 *				����    ���� TRUE  1
 */
uint8 JGCheck_Para(void)
{
  if( SETUPALIAS.LaneWide == 0)
  {
  	return FALSE;
  }
  if((SETUPALIAS.u16J0ZeroPos <160) || ( SETUPALIAS.u16J0ZeroPos>200))
  {
   	return FALSE;
  }
  if(SETUPALIAS.u16J0EndPos>360)
  {
    return FALSE;  
  }
  /****����ж�����***/

  return TRUE;
}
//void init_SingleVeh_param(void)
//{
//	uint32 head_init,tail_init;
//	uint32 sv_write_Sd_add = 0xc00000;
//	WriteC256(SVWRITESDADD,(uint8*)&sv_write_Sd_add,4);
//	head_init = 0x00;
//	tail_init = 0x00;
////	Set_Que_Cycle(head_init,0);
////	Set_Que_Cycle(tail_init,1);	
//}
/*********************************************************************************************************
** Function name:           TaskStart
**
** Descriptions:            �������񣬳�ʼ��ϵͳ����������������
**
** input parameters:        None
** output parameters:       None
** Returned value:          None
**
** Created by:              ZHANG Ye
** Created Date:            20110511
**--------------------------------------------------------------------------------------------------------
** Modified by:
** Modified date:
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
uint8	g_len = 0;
extern uint32 g_u32count0;
extern uint32 g_u32count1;
extern uint32 g_u32count2;
extern uint32 g_u32count3;
void  TaskStart (void *tdata)
{	
//	uint32	l_u32BR,head_init,tail_init,cnt_init;
//	uint8 wbuf[4] = {0x00,0xc0,0x00,0x00};
//	uint8 rbuf[4] = {0};
//	uint8	err = 0,ReadLen,R_Buf[64],i;	   
//	uint8	u8RecDataStatus = FALSE;	
//	uint8	u8ProtocolBuf[150] = {0};
//	uint8	u8ProtocolDataLen = 0;	   
//	uint8   ret = 0;
//
//	SystemTime	l_sstResetTime;		  //
//#if 1 == TEST_PROBE
//	CycleBufferStruct	*pResetCycBuf;
//#endif
////	_Cycle_Que_Continue Que_Conti_Init;
//
//	uint32	l_u32ResetSource = 0;		  //��λԴ���ͣ�Ϊ0ʱΪ�ⲿ��λ��Ϊ1ʱΪ�ڲ���λ(���Ź���λ)
//	tdata = tdata;
//
//	ThresOrigineDataLow = 30;
//	ThresOrigineDataHigh = 25000;	   //20140211����������
//	ThresVehLow = 300;
//	ThresVehHigh = 6000;
//	ThresVehParallelWide = 3500;
//	ThresVehSingleWide = 1700;
//	g_u32count_Pro = 0;
//	g_u32cout_Pro_Two_Buff = 0;
//	g_u8flag_veh = 0;
//	g_total_count_Veh = 0;
////	memset(g_Base_data0_Value,0,sizeof(int32)*POINT_SUM);
////	memset(g_Base_data0_Cnt,0,sizeof(uint8)*POINT_SUM);
////	memset(g_Base_data1_Value,0,sizeof(int32)*POINT_SUM);
////	memset(g_Base_data1_Cnt,0,sizeof(uint8)*POINT_SUM);
//	Sin_Angle12= 0.0;// 0.6635;
//	Cos_Angle12= 0.0;//0.7482;
////	StartAngle	=	19;
////	EndAngle	=	161;
//	g_u8Jg_4_Buff_Count =0;
//	g_u32Two_Buff_cout=0;  
//
//	g_MedianLeftWide  = 0;
//	g_MedianRightWide = 0;
//	g_MaxLeftWide     = 0;
//	g_MaxRightWide    = 0;
//	g_NearMinWide     = 0;
//	g_NearMaxWide     = 0;
//	g_FarMinWide      = 0;
//	g_FarMaxWide      = 0;
//	g_u8JGFXFlag      = 1;
//
//
//	TargetInit();
//	InitAllIRQ();//��������Ҫ���жϽ��г�ʼ��,��ʱ��I2C,UCOS����ʱ�������г�ʼ��/���������г�ʼ��	 
//
//	ReadC256(BUF0ADDR,(uint8 *)&SETUPALIAS, sizeof(SETUPALIAS));   	
//	if(CheckCrc((uint8 *)&SETUPALIAS,sizeof(SETUPALIAS)-2) == 0 || JGCheck_Para() == 0)
//	{
//		JZInit();
//	}
//		JZInit();
////	else if ((SETUPALIAS.IncHeightLaser != 0) && (SETUPALIAS.HeightLaser != 0))
////	{
////	 	Cos_Angle12 = 1.0*(SETUPALIAS.HeightLaser+SETUPALIAS.LaserDistance)/SETUPALIAS.IncHeightLaser;
////		Sin_Angle12 = sqrt(SETUPALIAS.IncHeightLaser*SETUPALIAS.IncHeightLaser - (SETUPALIAS.HeightLaser+SETUPALIAS.LaserDistance)*(SETUPALIAS.HeightLaser+SETUPALIAS.LaserDistance))/SETUPALIAS.IncHeightLaser;   //sqrt	
////	}
//	SETUPALIAS.resetCnt++;
//	memcpy(SETUPALIAS.au8ProgramVersion,JGJD_VERSION,11);     //��ʼ������汾��
//	AddCrc16((uint8 *)&SETUPALIAS,sizeof(SETUPALIAS)-2);	 
//    WriteC256(BUF0ADDR,(uint8 *)&SETUPALIAS, sizeof(SETUPALIAS));
//
//	WDTInit(1); 
//	l_u32ResetSource = WDTIM_RES&0x01;
//
//#if 1 == TEST_PROBE
////////20130701hyw/////////////////////////////////////////////////////////////////////////
//	g_len = sizeof(CycleBufferStruct) + RESETINFO_BUFFERSIZE * sizeof(SystemTime);
//	pResetCycBuf = (CycleBufferStruct *)malloc(sizeof(CycleBufferStruct) + RESETINFO_BUFFERSIZE * sizeof(SystemTime));
//	if (NULL == pResetCycBuf)
//	{
//		pResetCycBuf = (CycleBufferStruct *)malloc(sizeof(CycleBufferStruct) + RESETINFO_BUFFERSIZE * sizeof(SystemTime));	
//	}
//	memset((uint8 *)pResetCycBuf, 0, sizeof(CycleBufferStruct) + RESETINFO_BUFFERSIZE * sizeof(SystemTime));
//	pResetCycBuf->u8MaxBufSize = RESETINFO_BUFFERSIZE;
//
//	RTC8563Init();	//20130701hyw ʱ������
//	GetRTCTime(&l_sstResetTime); //20130701hyw
//
//   	ReadC256(RESETINFOADDR, (uint8 *)pResetCycBuf, sizeof(CycleBufferStruct) + RESETINFO_BUFFERSIZE * sizeof(SystemTime));
//
//	if(pResetCycBuf->u8MaxBufSize != RESETINFO_BUFFERSIZE
//		|| pResetCycBuf->u8CurrentPos > RESETINFO_BUFFERSIZE
//		|| (pResetCycBuf->u8CurrentEntries) > (pResetCycBuf->u8MaxBufSize))
//	{
//		memset((uint8 *)pResetCycBuf, 0, sizeof(CycleBufferStruct) + RESETINFO_BUFFERSIZE * sizeof(SystemTime));
//		pResetCycBuf->u8MaxBufSize = RESETINFO_BUFFERSIZE;
//		//pResetCycBuf->u8CurrentEntries = 0;
//		//pResetCycBuf->u8CurrentPos = 0;
//	}	
//
//	pResetCycBuf->u8Type[pResetCycBuf->u8CurrentPos] = (l_u32ResetSource+1)& 0xFF;
//	memcpy(&pResetCycBuf->SysTimeTable[pResetCycBuf->u8CurrentPos++], &l_sstResetTime, sizeof(l_sstResetTime));
//	pResetCycBuf->u8CurrentPos %= RESETINFO_BUFFERSIZE;
//
//	if((pResetCycBuf->u8CurrentEntries) >= (pResetCycBuf->u8MaxBufSize))	//���»���������
//	{
//		pResetCycBuf->u8CurrentEntries = pResetCycBuf->u8MaxBufSize;
//	}
//	else
//	{
//		pResetCycBuf->u8CurrentEntries++;
//	}
//
//	SDCardInit();
//	WriteC256(RESETINFOADDR, (uint8 *)pResetCycBuf, sizeof(CycleBufferStruct) + RESETINFO_BUFFERSIZE * sizeof(SystemTime));
//	free(pResetCycBuf);
//	pResetCycBuf = NULL;
////////////////////////////////////////////////////////////////////
//
//#endif
////	head_init = Get_Que_Cycle(0);
////	tail_init = Get_Que_Cycle(1);
////	if((head_init == 0) && (head_init == 0))		//��������һ�γ�ʼ������
////	{
////		init_SingleVeh_param();	
////	}
////	else	
//	{
//	SETUPALIAS.resetCnt++;
//	memcpy(SETUPALIAS.au8ProgramVersion,JGJD_VERSION,11);     //��ʼ������汾��
//	AddCrc16((uint8 *)&SETUPALIAS,sizeof(SETUPALIAS)-2);	 
//    WriteC256(BUF0ADDR,(uint8 *)&SETUPALIAS, sizeof(SETUPALIAS));
//	ReadC256(DEVICECODEADDR,RDid,16);	  //��ȡ�豸ʶ����
//	ReadC256(STATIONNUMADDR,RDNum,15);	  //��ȡվ����
//	}
//    if(RDid[0] == 0)
//	{
//		memcpy(RDid,NewRDid,16);    //Ĭ���豸ʶ����
//		WriteC256(DEVICECODEADDR, RDid, 16);  //������
//	}
//	if(RDNum[0] == 0)
//	{
//		memcpy(RDNum,NewRDNum,15);   //Ĭ��վ����
//		WriteC256(STATIONNUMADDR, RDNum, 15);  //������
//	}
//	ReadC256(LANEDIRADDR,&g_u8LaneDir,1);
//	if(g_u8LaneDir != 0xAA && g_u8LaneDir != 0xBB)
//	{
//		g_u8LaneDir = 0xAA;
//		WriteC256(LANEDIRADDR, &g_u8LaneDir, 1);  //������
//	}
//
//
////��ȡuart1��������ʱ�򣬵�����Ϣ��SD���еĴ洢λ��
////	ReadC256(SVWRITESDADD,(uint8*)&sv_write_sd_add,4);
////
////	limit_length = 0;		//���ó��޲����ĳ�ʼֵ				 	
////	limit_height = 0;
////	limit_width = 0;
//
//
//	
//#ifndef  SIM_SOFTWARE
//	WDTInit(0); 
//#else
//	WDTInit(0); 
//#endif
////	if (g_sspSetup.u8InstallFlag) //��װ��ʽ
////	{	
////	g_MedianLeftWide  =	g_sspSetup.MedianLeftWide;
////	g_MaxLeftWide     =	g_sspSetup.LaneWide*(g_sspSetup.u8LaneNum>>1)+g_sspSetup.MedianLeftWide;
////	g_MedianRightWide =	g_sspSetup.MedianWide;
////	g_MaxRightWide    = g_sspSetup.LaneWide*(g_sspSetup.u8LaneNum>>1)+g_sspSetup.MedianWide;		
////	}
////	else	  //��װ��ʽ
////	{
////	    g_NearMinWide   =  g_sspSetup.n32LaserHorizOff;
////		g_NearMaxWide	=  g_sspSetup.LaneWide*(g_sspSetup.u8LaneNum>>1)+g_sspSetup.n32LaserHorizOff;  //��������������
////		g_FarMinWide    =  g_sspSetup.LaneWide*(g_sspSetup.u8LaneNum>>1)+g_sspSetup.MedianWide+g_sspSetup.n32LaserHorizOff; //Զ��������С����
////	    g_FarMaxWide    =  g_sspSetup.LaneWide*g_sspSetup.u8LaneNum+g_sspSetup.MedianWide+g_sspSetup.n32LaserHorizOff; //Զ������������
////	}
//	g_LaneWide      =  g_sspSetup.LaneWide;
//
////   	if (g_sspSetup.u16StartPtNum0 < g_sspSetup.u16J0ZeroPos && 
////	    g_sspSetup.u16J0EndPos <= g_sspSetup.u16J0ZeroPos)
////	{
////		g_u8JGFXFlag = 0;
////	}
//
////��Ӳ���CPUʹ����
//#if OS_TASK_STAT_EN >0
//	OSStatInit();
//#endif		      	
////	sv_count = 0;
////	memset(sv_frame_data,0,512);		      	
//	//��ʼ�������ź���
////	g_SD_single_Veh = OSSemCreate(0); //�����³�ʱ�����ź�����SD���������ź��������ڴ洢���³���Ϣ��
//	g_Uart1_send = OSSemCreate(0);
//	g_JG0flag	=OSSemCreate(0);
//	g_JG1flag	=OSSemCreate(0);
//    g_JG2flag	=OSSemCreate(0);
//	g_JG3flag	=OSSemCreate(0);
//	g_JG_Pro	=OSSemCreate(0);
//	g_Uart5_Rec	=OSSemCreate(0);
//	FW_flag  =OSSemCreate(0);  
////	EVENT_02Rev	=OSSemCreate(0);//02���յ��ź������յ�ʱ�ͷ��ź���������01��ǰ�ȷ���02������������ź���
////	EVENT_0ARev =OSSemCreate(0); 
////	EVENT_11Rev =OSSemCreate(0); 
////	SD_flag = OSSemCreate(1);	//SD������Դ��־���κ�ʹ�ø���Դ���豸������Ҫ�Ȼ�ȡ��pend������Դ��ʹ����Ϻ�Ҫ�ͷŸ���Դ��post��
//	UART1_flag = OSSemCreate(1);  //UART1����Դ��־	
//   	ReadLen = (uint16)(HSU1_LEVEL & 0xff);
//	   for (i=0; i < ReadLen; i++) 
//	   {
//	    R_Buf[i] = (uint8)(HSU1_RX & 0xff);                       /* �������ݴ�����ջ�����       */
//	   }
//#if	YBVERSION >= 30		//3.0�Ǳ���		
//	InitAllSP(UBR_115200, UBR_115200, UBR_115200);	 //�����д��ڽ��г�ʼ��
//#endif
//
//
//
//
////��ʼ��SD����
//	SDCardInit(); 
//
////����洢		  
////	OSTaskCreate(Task_SD,(void *)0,&TaskSD[TASKSTACKSIZE-1],Task_SDPRIO);	 //TASKSTACKSIZE = 2048
//	OSTaskCreate(Task_Checknet,(void *)0, &TaskChecknet[TASKSTACKSIZE-1], TASKChecknetPRIO);  //�������ݣ�����ʶ
////	OSTaskCreate(Task_TiPo,(void *)0,&TaskTiPo[TASKSTACKSIZE-1],TASK_TiPoPRIO);
//	BeepON();
//	OSTimeDly(50);	
//	BeepOFF();	 
////	OSTaskCreate(Task_Uart5,(void *)0,&TaskUart5[TASKSTACKSIZE-1],Task_Uart5PRIO);
//   
//	OSTaskCreate(Task_JG0,(void *)0,&TaskJG0[TASKSTACKSIZE-1],Task_JG0PRIO);
//	OSTaskCreate(Task_JG1,(void *)0,&TaskJG1[TASKSTACKSIZE-1],Task_JG1PRIO);
//	OSTaskCreate(Task_JG2,(void *)0,&TaskJG2[TASKSTACKSIZE-1],Task_JG2PRIO);
//	OSTaskCreate(Task_JG3,(void *)0,&TaskJG3[TASKSTACKSIZE-1],Task_JG3PRIO);
//
//	OSTaskCreate(Task_Data_JG,(void *)0,&TaskDataJG[TASKSTACKSIZE-1],Task_Data_JGPRIO);
////	OSTaskCreate(Task_Uart5_Senddata,(void *)0,&TaskUart5Senddata[TASKSTACKSIZE-1],Task_Uart5_SenddataPRIO);
//	
////	OSTaskCreate(Task_Test,(void *)0,&TaskTest[TASKSTACKSIZE-1],Task_TestPRIO);
//// 	OSTaskCreate(Task_SendUart1,(void *)0,&TaskSendUart1[TASKSTACKSIZE-1],TaskSendUart1PRIO);
////	OSTaskCreate(Task_Uart1_Senddata,(void *)0,&TaskUart5Senddata[TASKSTACKSIZE-1],Task_Uart5_SenddataPRIO);
////	OSTaskCreate(Task_Match_Send,(void *)0,&TaskMatchSend[TASKSTACKSIZE-1],TaskMatchSendPRIO);
//
//	
//	//��ͨ������������ʱ������
//	BeepON();
//	OSTimeDly(50);	
//	BeepOFF();	 
//	OSTimeDly(50);
//	BeepON();
//	OSTimeDly(50);	
//	BeepOFF();
//	while(1)
//	{
////		OSSemAccept(g_Uart5_Rec, 0, &err);  
//		l_u32BR = 	OSSemAccept(g_Uart5_Rec);
//		if(l_u32BR)
//		{
//	   		u8RecDataStatus = RecComData(u8ProtocolBuf, &u8ProtocolDataLen);
//			if (u8RecDataStatus)
//			{
//				AnalyzeComData(u8ProtocolBuf, &u8ProtocolDataLen);
//			} 			
//		}
//		else
//			OSTimeDly(5);
//
//		WDTIM_COUNTER	= 1;									/* ι��							*/	
//
//	}
//
// 
	uint32	l_u32BR;
//	uint8	err = 0;	   
	uint8	u8RecDataStatus = FALSE;	
	uint8	u8ProtocolBuf[150] = {0};
	uint8	u8ProtocolDataLen = 0;	   
	uint8   ret = 0;
	uint8 U5Buff[15]={0x00};
	SystemTime	l_sstResetTime;		  //

	tdata = tdata;

	ThresOrigineDataLow = 30;//600;		   //20150206
	ThresOrigineDataHigh = 25000;//7000;	   //20140211����������
	ThresOrigineDataLow0 = 600;//600;		   //20150206
	ThresOrigineDataHigh0 = 15000;//8000;	   //20140211����������
	ThresVehLow = 300;//500;
	ThresVehHigh = 6000;//5000;
	ThresVehParallelWide = 3500;
	ThresVehSingleWide = 1700;
	g_u32count_Pro = 0;
	g_u32cout_Pro_Two_Buff = 0;
	g_u8flag_veh = 0;
	g_total_count_Veh = 0; 
	Sin_Angle12= 0.0;// 0.6635;
	Cos_Angle12= 0.0;//0.7482;
	g_u8Jg_4_Buff_Count =0;
	g_u32Two_Buff_cout=0;  

	g_MedianLeftWide  = 0;
	g_MedianRightWide = 0;
	g_MaxLeftWide     = 0;
	g_MaxRightWide    = 0;
	g_NearMinWide     = 0;
	g_NearMaxWide     = 0;
	g_FarMinWide      = 0;
	g_FarMaxWide      = 0;
	g_u8JGFXFlag      = 1;		


	TargetInit();
	InitAllIRQ();//��������Ҫ���жϽ��г�ʼ��,��ʱ��I2C,UCOS����ʱ�������г�ʼ��/���������г�ʼ��	 

	ReadC256(BUF0ADDR,(uint8 *)&SETUPALIAS, sizeof(SETUPALIAS));   	
	if(CheckCrc((uint8 *)&SETUPALIAS,sizeof(SETUPALIAS)-2) == 0 )
	{
		JZInit();
	}
//	JZInit();
	SETUPALIAS.resetCnt++;
	memcpy(SETUPALIAS.au8ProgramVersion,JGJD_VERSION,11);     //��ʼ������汾��
	AddCrc16((uint8 *)&SETUPALIAS,sizeof(SETUPALIAS)-2);	 
    WriteC256(BUF0ADDR,(uint8 *)&SETUPALIAS, sizeof(SETUPALIAS));

	RTC8563Init();	//
	GetRTCTime(&l_sstResetTime); //


#ifndef  SIM_SOFTWARE
	WDTInit(0); 
#else
	WDTInit(0); 
#endif
//��Ӳ���CPUʹ����
#if OS_TASK_STAT_EN >0
	OSStatInit();
#endif		      	
	//��ʼ�������ź���
//	g_SD_single_Veh = OSSemCreate(0); //�����³�ʱ�����ź�����SD���������ź��������ڴ洢���³���Ϣ��
  	g_sendVeh = OSSemCreate(0);
	g_Uart1_send = OSSemCreate(0);
	g_JG0flag	=OSSemCreate(0);
	g_JG1flag	=OSSemCreate(0);
    g_JG2flag	=OSSemCreate(0);
	g_JG3flag	=OSSemCreate(0);
	g_JG_Pro	=OSSemCreate(0);
	g_Uart5_Rec	=OSSemCreate(0);
//	EVENT_02Rev	=OSSemCreate(0);//02���յ��ź������յ�ʱ�ͷ��ź���������01��ǰ�ȷ���02������������ź���
//	EVENT_0ARev =OSSemCreate(0);
//	EVENT_dataRev =OSSemCreate(0);   												
	FW_flag = OSSemCreate(1);	//�������Դ��־���κ�ʹ�ø���Դ���豸������Ҫ�Ȼ�ȡ��pend������Դ��ʹ����Ϻ�Ҫ�ͷŸ���Դ��post��										
//	g_Port3flag =OSSemCreate(0); //hmh
//	Uart1SendResSem = OSSemCreate(1);		//���ڷ�����Դ�ź���	
#if	YBVERSION >= 30		//3.0�Ǳ���		
	InitAllSP(UBR_115200, UBR_115200, UBR_115200);	 //�����д��ڽ��г�ʼ��
#endif




//��ʼ��SD����
//	SDCardInit();



//����洢		  
//	OSTaskCreate(Task_SD,(void *)0,&TaskSD[TASKSTACKSIZE-1],Task_SDPRIO);	 //TASKSTACKSIZE = 2048
	OSTaskCreate(Task_Checknet,(void *)0, &TaskChecknet[TASKSTACKSIZE-1], TASKChecknetPRIO);  //�������ݣ�����ʶ
//	OSTaskCreate(Task_TiPo,(void *)0,&TaskTiPo[TASKSTACKSIZE-1],TASK_TiPoPRIO);
//	OSTaskCreate(Task_Uart5,(void *)0,&TaskUart5[TASKSTACKSIZE-1],Task_Uart5PRIO);
   
	OSTaskCreate(Task_JG0,(void *)0,&TaskJG0[TASKSTACKSIZE-1],Task_JG0PRIO);
	OSTaskCreate(Task_JG1,(void *)0,&TaskJG1[TASKSTACKSIZE-1],Task_JG1PRIO);
	OSTaskCreate(Task_JG2,(void *)0,&TaskJG2[TASKSTACKSIZE-1],Task_JG2PRIO);
	OSTaskCreate(Task_JG3,(void *)0,&TaskJG3[TASKSTACKSIZE-1],Task_JG3PRIO);

	OSTaskCreate(Task_Data_JG,(void *)0,&TaskDataJG[TASKSTACKSIZE-1],Task_Data_JGPRIO);
	OSTaskCreate(Task_Match_Send,(void *)0,&TaskMatchSend[TASKSTACKSIZE-1],TaskMatchSendPRIO);
//	OSTaskCreate(Task_Data_JG,(void *)0,&TaskDataJG[TASKSTACKSIZE-1],Task_Data_JGPRIO);
//	OSTaskCreate(Task_Net,(void*)0,&TaskNetData[TASKSTACKSIZE-1],Task_NetPRIO);
//	OSTaskCreate(Task_Uart5_Senddata,(void *)0,&TaskUart5Senddata[TASKSTACKSIZE-1],Task_Uart5_SenddataPRIO);
	
//	OSTaskCreate(Task_Test,(void *)0,&TaskTest[TASKSTACKSIZE-1],Task_TestPRIO);
// 	OSTaskCreate(Task_SendUart1,(void *)0,&TaskSendUart1[TASKSTACKSIZE-1],TaskSendUart1PRIO);
//	OSTaskCreate(Task_Uart1_Senddata,(void *)0,&TaskUart5Senddata[TASKSTACKSIZE-1],Task_Uart5_SenddataPRIO);

//	P3_OUTP_SET |= 1<<7;
//while(1)
//{
//	P3_OUTP_CLR  |= 1<<7;
//	OSTimeDly(50);
//	P3_OUTP_SET |= 1<<7;
//	OSTimeDly(50);
//	
//}


	//��ͨ������������ʱ������
	BeepON();
	OSTimeDly(50);	
	BeepOFF();	 
	OSTimeDly(50);
	BeepON();
	OSTimeDly(50);	
	BeepOFF();
	while(1)
	{
//		OSSemAccept(g_Uart5_Rec, 0, &err);  
		l_u32BR = 	OSSemAccept(g_Uart5_Rec);
		if(l_u32BR)
		{
	   		u8RecDataStatus = RecComData(u8ProtocolBuf, &u8ProtocolDataLen);
			if (u8RecDataStatus)
			{
				AnalyzeComData(u8ProtocolBuf, &u8ProtocolDataLen);
			} 			
		}
		else
		{
//			OSTimeDly(500);
//			 U5Buff[0]=0xFF;
//			 U5Buff[1]=g_u32count0>>8;	 								   
//			 U5Buff[2]=g_u32count0;
//			 U5Buff[3]=g_u32count1>>8;
//			 U5Buff[4]=g_u32count1;
//			 U5Buff[5]=g_u32count2>>8;
//			 U5Buff[6]=g_u32count2;
//			 U5Buff[7]=g_u32count3>>8;
//			 U5Buff[8]=g_u32count3;		
//			 U5SendBytes(U5Buff,9 );
		 OSTimeDly(10);
		}

		WDTIM_COUNTER	= 1;									/* ι��							*/	

	}



}
/****************************************************/
/****************************************************
��������
hong;
*****************************************************/
uint8 Read256_full(uint16 p_u16Addr, uint8 * p_pu8WriteBuf, uint16 p_u16Len)
{
	uint8 err;
	uint8 ret;
	OSSemPend(FW_flag,5000,&err);
	if(err==0)
	{
		ret = ReadC256(p_u16Addr,p_pu8WriteBuf,p_u16Len);
		OSSemPost(FW_flag);
	}
	return ret;
}
uint8 Write256_full(uint16 p_u16Addr, uint8 * p_pu8WriteBuf, uint16 p_u16Len)
{
	uint8 err;
	uint8 ret; 
	OSSemPend(FW_flag,5000,&err);
	if(err==0)
	{
		ret = WriteC256(p_u16Addr,p_pu8WriteBuf,p_u16Len);
		OSSemPost(FW_flag);
	}
	return ret;
}

