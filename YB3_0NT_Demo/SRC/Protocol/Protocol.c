#include "Protocol.h"
#include "WT_Task.h"
#include "RD_data.H"

#define CMD_QUERYPARAM	1
#define	CMD_SETPARAM	2
#define	CMD_RESETDEVICE	3
#define CMD_INITDEVICE  4

#define CMD_GETDEVICETIME  0x10
#define CMD_SETDEVICETIME  0x11
#define CMD_GETDEVICE_RDID 0X12  // 获取设备识别码和站点编号
#define CMD_SETDEVICE_RDID 0X13	 // 设置设备识别码和站点编号
#define CMD_GET_LANE_DIR   0X20  // 获取设备上下行设置
#define CMD_SET_LANE_DIR   0X21  // 设置设备上下行设置
#define CMD_GET_THRESHOLD  0X14  // 获取阈值
#define CMD_SET_THRESHOLD  0X15  // 设置阈值

#define	WJ_CMD_DEVICERESETINFO		0x30
#define	WJ_CMD_CLEARRESETIFNOBUF	0x31
#define ResetSystem()  			{WDTInit(1),WDTIM_COUNTER =  0x10000000-100;while(1); }
#define		SETUPALIAS				g_sspSetup

#if 1 == TEST_PROBE
static	void SendDevResetInfo(void);
static	void ClearResetInfoBuf(void);
#endif
uint8 RecComData(uint8 *pUartDataBuf, uint8 *plen)
{
	uint8	l_u8tempPCCmdBuf[150] = {0};	//20130424
	uint8	l_u8RecNum = 0;			//接收到的字节数
	static	uint8	l_u8CurrentSizeCnt = 0;	//当前命令有效数据计数
	static	uint16 	l_u16ProtocolFrameLength = 0;//必须是静态变量20130424
	uint8	l_u8tempPFrameLenHi = 0;
	uint8	l_u8tempPFrameLenLo = 0;
	uint8	i = 0;

	l_u8RecNum = U5ReciveByte(l_u8tempPCCmdBuf, 0);

	for(i=0; i<l_u8RecNum; i++)
	{
		if (0 == l_u8CurrentSizeCnt)
		{
		   if (0 == (l_u8tempPCCmdBuf[i]^0xFF))	//order0:帧头1
		   {
		   	  l_u8CurrentSizeCnt = 1;
		   }
		   else
		   {
		   	  l_u8CurrentSizeCnt = 0;
		   }
		}
		else if (1 == l_u8CurrentSizeCnt)
		{
		   if (0 == (l_u8tempPCCmdBuf[i]^0xFF))	//order1:帧头2
		   {
		   	  l_u8CurrentSizeCnt = 2;
		   }
		   else
		   {
		   	  l_u8CurrentSizeCnt = 0;
		   }				
		}
		else if (2 == l_u8CurrentSizeCnt)
		{					
		   if (0 == (l_u8tempPCCmdBuf[i]^0xFF))	 
		   {
		   	  l_u8CurrentSizeCnt = 2;
		   }
		   else	 //order2:命令号
		   {
				*pUartDataBuf = 0xFF;
				*(pUartDataBuf+1) = 0xFF;
				*(pUartDataBuf+2) = l_u8tempPCCmdBuf[i];
				l_u8CurrentSizeCnt++;			   	  
		   }
		}
		else if(3 == l_u8CurrentSizeCnt)
		{
				l_u8tempPFrameLenHi = l_u8tempPCCmdBuf[i];
				*(pUartDataBuf+3) = l_u8tempPFrameLenHi;
				l_u8CurrentSizeCnt++;			
		}
		else if (4 == l_u8CurrentSizeCnt)
		{
			l_u8tempPFrameLenLo = l_u8tempPCCmdBuf[i];
			*(pUartDataBuf+4) = l_u8tempPFrameLenLo;
			l_u16ProtocolFrameLength = (l_u8tempPFrameLenHi<<8) + l_u8tempPFrameLenLo;//协议字节数
			l_u8CurrentSizeCnt++;
		}
		else if ((l_u8CurrentSizeCnt > 4) && (l_u8CurrentSizeCnt < l_u16ProtocolFrameLength))
		{
			*(pUartDataBuf+l_u8CurrentSizeCnt) =  l_u8tempPCCmdBuf[i];
			l_u8CurrentSizeCnt++;
		}

		if (l_u16ProtocolFrameLength == l_u8CurrentSizeCnt && l_u8CurrentSizeCnt>=2)
		{			
			if(CheckCrc(pUartDataBuf,l_u16ProtocolFrameLength-2)==0)//crc校验失败
			{
				l_u8CurrentSizeCnt = 0;
				l_u16ProtocolFrameLength = 0;	 //20130424
				*(pUartDataBuf+2) += 4;
				*(pUartDataBuf+3) = 0;
				*(pUartDataBuf+4) = 7;
				AddCrc16(pUartDataBuf, 5);
				U5SendBytes(pUartDataBuf, 7);
				memset(pUartDataBuf, 0, l_u16ProtocolFrameLength);
				return FALSE;
			}
			else //校验成功
			{
			 	 *plen = l_u16ProtocolFrameLength;
				 l_u16ProtocolFrameLength = 0;	  //20130424
				 l_u8CurrentSizeCnt = 0;
				 return TRUE;
			}	
		}
	}
	return FALSE;
}

void AnalyzeComData(uint8 *pUartDataBuf, uint8 *plen)
{
	uint8	CmdNO = 0;
	uint16	tempindex = 0;
	uint8   ret = 0;
	int		i = 0;
	uint8	l_u8PCProtocolBuf[300] = {0};
	SystemTime dev_time;
	CmdNO = *(pUartDataBuf+2);
	switch(CmdNO)
	{
		case CMD_QUERYPARAM:	  //为参数查询
			
			ret = Read256_full(BUF0ADDR,(uint8 *)&SETUPALIAS, sizeof(SETUPALIAS));
			tempindex = 5;
			memcpy(l_u8PCProtocolBuf+tempindex,SETUPALIAS.au8ProgramVersion,11);	 //版本号
			tempindex += 11;
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u8InstallFlag;         //安装方式 0 侧桩 1 正桩
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u8BaudRate;
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u8DOG;
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.HeightLaser0>>24;	   //激光器垂直高度值
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.HeightLaser0>>16)&0xFF;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.HeightLaser0>>8)&0xFF;                           	  	  
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.HeightLaser0&0xFF; 			
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.HeightLaser1>>24;	   //激光器垂直高度值
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.HeightLaser1>>16)&0xFF;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.HeightLaser1>>8)&0xFF;                           	  	  
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.HeightLaser1&0xFF;
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.HeightLaser2>>24;	   //激光器垂直高度值
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.HeightLaser2>>16)&0xFF;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.HeightLaser2>>8)&0xFF;                           	  	  
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.HeightLaser2&0xFF;
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.HeightLaser3>>24;	   //激光器垂直高度值
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.HeightLaser3>>16)&0xFF;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.HeightLaser3>>8)&0xFF;                           	  	  
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.HeightLaser3&0xFF;
//			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.IncHeightLaser>>24;		
//			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.IncHeightLaser>>16)&0xFF;
//			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.IncHeightLaser>>8)&0xFF;                              //%车道宽度
//			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.IncHeightLaser&0xFF;

			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.LaserDistance>>24;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.LaserDistance>>16)&0xFF;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.LaserDistance>>8)&0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.LaserDistance&0xFF;
			
//			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.Angle12>>24;
//			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.Angle12>>16)&0xFF;
//			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.Angle12>>8)&0xFF;    
//			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.Angle12&0xFF;

			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.LaneWide>>24;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.LaneWide>>16)&0xFF;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.LaneWide>>8)&0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.LaneWide&0xFF;

			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.MedianWide>>24;		 //隔离带宽度，原隔离带右边宽度
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.MedianWide>>16)&0xFF;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.MedianWide>>8)&0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.MedianWide&0xFF;

			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.MedianLeftWide>>24;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.MedianLeftWide>>16)&0xFF;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.MedianLeftWide>>8)&0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.MedianLeftWide&0xFF;
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.resetCnt>>24;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.resetCnt>>16)&0xFF;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.resetCnt>>8)&0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.resetCnt&0xFF;
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32LaserRoadAngle>>24;   //激光扫描截面偏角，（原寻高起始点）
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32LaserRoadAngle>>16)&0xFF;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32LaserRoadAngle>>8)&0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32LaserRoadAngle&0xFF;
			
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.VerticalLaser_IP>>24;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.VerticalLaser_IP>>16) & 0xFF;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.VerticalLaser_IP>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.VerticalLaser_IP & 0xFF;					
			
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.VerticalLaser_Port>>24;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.VerticalLaser_Port>>16) & 0xFF;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.VerticalLaser_Port>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.VerticalLaser_Port & 0xFF;
			
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.ParallerLaser_IP1>>24;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.ParallerLaser_IP1>>16) & 0xFF;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.ParallerLaser_IP1>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.ParallerLaser_IP1 & 0xFF;					
			
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.ParallerLaser_Port1>>24;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.ParallerLaser_Port1>>16) & 0xFF;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.ParallerLaser_Port1>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.ParallerLaser_Port1 & 0xFF;
			//2014-12-04新增
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.ParallerLaser_IP2>>24;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.ParallerLaser_IP2>>16) & 0xFF;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.ParallerLaser_IP2>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.ParallerLaser_IP2 & 0xFF;					
			
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.ParallerLaser_Port2>>24;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.ParallerLaser_Port2>>16) & 0xFF;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.ParallerLaser_Port2>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.ParallerLaser_Port2 & 0xFF;

				l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.ParallerLaser_IP3>>24;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.ParallerLaser_IP3>>16) & 0xFF;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.ParallerLaser_IP3>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.ParallerLaser_IP3 & 0xFF;					
			
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.ParallerLaser_Port3>>24;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.ParallerLaser_Port3>>16) & 0xFF;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.ParallerLaser_Port3>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.ParallerLaser_Port3 & 0xFF;
			//2014-12-04
//控制器ip参数					
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32LocalIPAddress>>24;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32LocalIPAddress>>16) & 0xFF;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32LocalIPAddress>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32LocalIPAddress & 0xFF;
			
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32SubMask>>24;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32SubMask>>16) & 0xFF;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32SubMask>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32SubMask & 0xFF;								
			
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32LocalPortNO>>24;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32LocalPortNO>>16) & 0xFF;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32LocalPortNO>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32LocalPortNO & 0xFF;					
		//网关
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32GatewayIP>>24;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32GatewayIP>>16) & 0xFF;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32GatewayIP>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32GatewayIP & 0xFF;
			
		//	au8LocalMAC
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.au8LocalMAC[0];
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.au8LocalMAC[1];
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.au8LocalMAC[2];    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.au8LocalMAC[3];					
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.au8LocalMAC[4];
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.au8LocalMAC[5];																								

//20130424 	1:		
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32Net1_DisconnectNum>>24;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32Net1_DisconnectNum>>16) & 0xFF;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32Net1_DisconnectNum>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32Net1_DisconnectNum & 0xFF;
			//2:
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32Net2_DisconnectNum>>24;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32Net2_DisconnectNum>>16) & 0xFF;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32Net2_DisconnectNum>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32Net2_DisconnectNum & 0xFF;

				l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32Net3_DisconnectNum>>24;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32Net3_DisconnectNum>>16) & 0xFF;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32Net3_DisconnectNum>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32Net3_DisconnectNum & 0xFF;
			//2:
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32Net4_DisconnectNum>>24;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32Net4_DisconnectNum>>16) & 0xFF;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32Net4_DisconnectNum>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32Net4_DisconnectNum & 0xFF;
			//3:
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32Net1_InvalidRecNum>>24;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32Net1_InvalidRecNum>>16) & 0xFF;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32Net1_InvalidRecNum>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32Net1_InvalidRecNum & 0xFF;			
			//4:
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32Net2_InvalidRecNum>>24;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32Net2_InvalidRecNum>>16) & 0xFF;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32Net2_InvalidRecNum>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32Net2_InvalidRecNum & 0xFF;
			
				l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32Net3_InvalidRecNum>>24;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32Net3_InvalidRecNum>>16) & 0xFF;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32Net3_InvalidRecNum>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32Net3_InvalidRecNum & 0xFF;			
			//4:
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32Net4_InvalidRecNum>>24;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32Net4_InvalidRecNum>>16) & 0xFF;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32Net4_InvalidRecNum>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32Net4_InvalidRecNum & 0xFF;			
			//5:
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32DataProcException>>24;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32DataProcException>>16) & 0xFF;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32DataProcException>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32DataProcException & 0xFF;			
			
//			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32ProgramVersion>>24;
//			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32ProgramVersion>>16) & 0xFF;
//			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32ProgramVersion>>8) & 0xFF;    
//			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32ProgramVersion & 0xFF;

			
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.n32LaserHorizOff>>24;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.n32LaserHorizOff>>16) & 0xFF;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.n32LaserHorizOff>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.n32LaserHorizOff & 0xFF;
// 垂直
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u16VerticalZeroPos0>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u16VerticalZeroPos0 & 0xFF;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u16VerticalZeroPos1>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u16VerticalZeroPos1 & 0xFF;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u16VerticalZeroPos2>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u16VerticalZeroPos2 & 0xFF;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u16VerticalZeroPos3>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u16VerticalZeroPos3 & 0xFF;			
////倾斜			
//			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u16InclineZeroPos>>8) & 0xFF;    
//			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u16InclineZeroPos & 0xFF;
//起始点			
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u16StartPtNum0>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u16StartPtNum0 & 0xFF;						
//终止点			
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u16EndPtNum0>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u16EndPtNum0 & 0xFF;
			//起始点			
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u16StartPtNum1>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u16StartPtNum1 & 0xFF;						
//终止点			
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u16EndPtNum1>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u16EndPtNum1 & 0xFF;
			//起始点			
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u16StartPtNum2>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u16StartPtNum2 & 0xFF;						
//终止点			
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u16EndPtNum2>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u16EndPtNum2 & 0xFF;
			//起始点			
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u16StartPtNum3>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u16StartPtNum3 & 0xFF;						
//终止点			
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u16EndPtNum3>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u16EndPtNum3 & 0xFF;
			
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u8LaserDevType;
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u8TrafficType;
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u8RoadType;	    // 道路类型 0-- 国道 1-- 高速
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u8LaneNum;	    //车道数 
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u8NetType;		//网络连接：0 -- 无线 1 -- 有线
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u8SDEnable;
						
			l_u8PCProtocolBuf[0] = 0xFF;
			l_u8PCProtocolBuf[1] = 0xFF;
			l_u8PCProtocolBuf[2] = CMD_QUERYPARAM;					
			l_u8PCProtocolBuf[3] = ((tempindex + 2)>>8)&0xFF;//协议帧字节数
			l_u8PCProtocolBuf[4] = (tempindex + 2)&0xFF;
//			l_u16CRC = AddCrc16(l_u8PCProtocolBuf,tempindex);
//			l_u8PCProtocolBuf[tempindex++] = (l_u16CRC>>8)&0xFF;//添加CRC校验															  
//			l_u8PCProtocolBuf[tempindex++] = l_u16CRC&0xFF;
//
//			U5SendBytes(l_u8PCProtocolBuf, tempindex + 2);
			AddCrc16(l_u8PCProtocolBuf,	tempindex);
			U5SendBytes(l_u8PCProtocolBuf, tempindex + 2);
			break;
		case CMD_SETPARAM:	  //为参数设置
			memset(l_u8PCProtocolBuf,0,sizeof(l_u8PCProtocolBuf));
			memcpy(l_u8PCProtocolBuf, pUartDataBuf, sizeof(l_u8PCProtocolBuf));
			tempindex = 5;
			//跳过版本号设置//
			tempindex += 11;
			SETUPALIAS.u8InstallFlag = l_u8PCProtocolBuf[tempindex++];	  //安装方式 0 侧桩 1 正桩 
			SETUPALIAS.u8BaudRate = l_u8PCProtocolBuf[tempindex++];
			SETUPALIAS.u8DOG = l_u8PCProtocolBuf[tempindex++];
			SETUPALIAS.HeightLaser0 = (l_u8PCProtocolBuf[tempindex]<<24)
									+ (l_u8PCProtocolBuf[tempindex+1]<<16)
									+ (l_u8PCProtocolBuf[tempindex+2]<<8)
									+ l_u8PCProtocolBuf[tempindex+3];
									tempindex += 4;
		    SETUPALIAS.HeightLaser1 = (l_u8PCProtocolBuf[tempindex]<<24)
									+ (l_u8PCProtocolBuf[tempindex+1]<<16)
									+ (l_u8PCProtocolBuf[tempindex+2]<<8)
									+ l_u8PCProtocolBuf[tempindex+3];
								    tempindex += 4;
			SETUPALIAS.HeightLaser2 = (l_u8PCProtocolBuf[tempindex]<<24)
									+ (l_u8PCProtocolBuf[tempindex+1]<<16)
									+ (l_u8PCProtocolBuf[tempindex+2]<<8)
									+ l_u8PCProtocolBuf[tempindex+3];
									tempindex += 4;
			SETUPALIAS.HeightLaser3 = (l_u8PCProtocolBuf[tempindex]<<24)
									+ (l_u8PCProtocolBuf[tempindex+1]<<16)
									+ (l_u8PCProtocolBuf[tempindex+2]<<8)
									+ l_u8PCProtocolBuf[tempindex+3];								
//			tempindex += 4;
//			SETUPALIAS.IncHeightLaser = (l_u8PCProtocolBuf[tempindex]<<24)
//									+ (l_u8PCProtocolBuf[tempindex+1]<<16)
//									+ (l_u8PCProtocolBuf[tempindex+2]<<8)
//									+ l_u8PCProtocolBuf[tempindex+3];
			tempindex += 4;
			SETUPALIAS.LaserDistance = (l_u8PCProtocolBuf[tempindex]<<24)
									+ (l_u8PCProtocolBuf[tempindex+1]<<16)
									+ (l_u8PCProtocolBuf[tempindex+2]<<8)
									+ l_u8PCProtocolBuf[tempindex+3];
//			tempindex += 4;
//			SETUPALIAS.Angle12 = (l_u8PCProtocolBuf[tempindex]<<24)
//									+ (l_u8PCProtocolBuf[tempindex+1]<<16)
//									+ (l_u8PCProtocolBuf[tempindex+2]<<8)
//									+ l_u8PCProtocolBuf[tempindex+3]; 					
			tempindex += 4;
			SETUPALIAS.LaneWide = (l_u8PCProtocolBuf[tempindex]<<24)
									+ (l_u8PCProtocolBuf[tempindex+1]<<16)
									+ (l_u8PCProtocolBuf[tempindex+2]<<8)
									+ l_u8PCProtocolBuf[tempindex+3];
			tempindex += 4;
			SETUPALIAS.MedianWide = (l_u8PCProtocolBuf[tempindex]<<24)
									+ (l_u8PCProtocolBuf[tempindex+1]<<16)
									+ (l_u8PCProtocolBuf[tempindex+2]<<8)
									+ l_u8PCProtocolBuf[tempindex+3];
			tempindex += 4;
			SETUPALIAS.MedianLeftWide = (l_u8PCProtocolBuf[tempindex]<<24)
									+ (l_u8PCProtocolBuf[tempindex+1]<<16)
									+ (l_u8PCProtocolBuf[tempindex+2]<<8)
									+ l_u8PCProtocolBuf[tempindex+3];

			tempindex += 4;
			SETUPALIAS.resetCnt = (l_u8PCProtocolBuf[tempindex]<<24)
									+ (l_u8PCProtocolBuf[tempindex+1]<<16)
									+ (l_u8PCProtocolBuf[tempindex+2]<<8)
									+ l_u8PCProtocolBuf[tempindex+3];
			tempindex += 4;
			SETUPALIAS.u32LaserRoadAngle = (l_u8PCProtocolBuf[tempindex]<<24)
									+ (l_u8PCProtocolBuf[tempindex+1]<<16)
									+ (l_u8PCProtocolBuf[tempindex+2]<<8)
									+ l_u8PCProtocolBuf[tempindex+3];

			tempindex += 4;
			SETUPALIAS.VerticalLaser_IP = (l_u8PCProtocolBuf[tempindex]<<24)
									+ (l_u8PCProtocolBuf[tempindex+1]<<16)
									+ (l_u8PCProtocolBuf[tempindex+2]<<8)
									+ l_u8PCProtocolBuf[tempindex+3];

			tempindex += 4;
			SETUPALIAS.VerticalLaser_Port = (l_u8PCProtocolBuf[tempindex]<<24)
									+ (l_u8PCProtocolBuf[tempindex+1]<<16)
									+ (l_u8PCProtocolBuf[tempindex+2]<<8)
									+ l_u8PCProtocolBuf[tempindex+3];

			tempindex += 4;
			SETUPALIAS.ParallerLaser_IP1 = (l_u8PCProtocolBuf[tempindex]<<24)
									+ (l_u8PCProtocolBuf[tempindex+1]<<16)
									+ (l_u8PCProtocolBuf[tempindex+2]<<8)
									+ l_u8PCProtocolBuf[tempindex+3];

			tempindex += 4;
			SETUPALIAS.ParallerLaser_Port1 = (l_u8PCProtocolBuf[tempindex]<<24)
									+ (l_u8PCProtocolBuf[tempindex+1]<<16)
									+ (l_u8PCProtocolBuf[tempindex+2]<<8)
									+ l_u8PCProtocolBuf[tempindex+3];
			//2014-12-04
			tempindex += 4;
			SETUPALIAS.ParallerLaser_IP2 = (l_u8PCProtocolBuf[tempindex]<<24)
									+ (l_u8PCProtocolBuf[tempindex+1]<<16)
									+ (l_u8PCProtocolBuf[tempindex+2]<<8)
									+ l_u8PCProtocolBuf[tempindex+3];

			tempindex += 4;
			SETUPALIAS.ParallerLaser_Port2 = (l_u8PCProtocolBuf[tempindex]<<24)
									+ (l_u8PCProtocolBuf[tempindex+1]<<16)
									+ (l_u8PCProtocolBuf[tempindex+2]<<8)
									+ l_u8PCProtocolBuf[tempindex+3];
			//2014-12-04
//控制器ip参数
			tempindex += 4;
			SETUPALIAS.u32LocalIPAddress = (l_u8PCProtocolBuf[tempindex]<<24)
									+ (l_u8PCProtocolBuf[tempindex+1]<<16)
									+ (l_u8PCProtocolBuf[tempindex+2]<<8)
									+ l_u8PCProtocolBuf[tempindex+3];
			tempindex += 4;
			SETUPALIAS.u32SubMask = (l_u8PCProtocolBuf[tempindex]<<24)
									+ (l_u8PCProtocolBuf[tempindex+1]<<16)
									+ (l_u8PCProtocolBuf[tempindex+2]<<8)
									+ l_u8PCProtocolBuf[tempindex+3];

			tempindex += 4;
			SETUPALIAS.u32LocalPortNO = (l_u8PCProtocolBuf[tempindex]<<24)
									+ (l_u8PCProtocolBuf[tempindex+1]<<16)
									+ (l_u8PCProtocolBuf[tempindex+2]<<8)
									+ l_u8PCProtocolBuf[tempindex+3];

			tempindex += 4;
			SETUPALIAS.u32GatewayIP = (l_u8PCProtocolBuf[tempindex]<<24)
									+ (l_u8PCProtocolBuf[tempindex+1]<<16)
									+ (l_u8PCProtocolBuf[tempindex+2]<<8)
									+ l_u8PCProtocolBuf[tempindex+3];

			tempindex += 4;
			for(i=0; i<6; i++)
			{
				SETUPALIAS.au8LocalMAC[i] = l_u8PCProtocolBuf[tempindex++];
			}
//20130425
//			tempindex += 6;
			SETUPALIAS.u32Net1_DisconnectNum = (l_u8PCProtocolBuf[tempindex]<<24)
									+ (l_u8PCProtocolBuf[tempindex+1]<<16)
									+ (l_u8PCProtocolBuf[tempindex+2]<<8)
									+ l_u8PCProtocolBuf[tempindex+3];

			tempindex += 4;
			SETUPALIAS.u32Net2_DisconnectNum = (l_u8PCProtocolBuf[tempindex]<<24)
									+ (l_u8PCProtocolBuf[tempindex+1]<<16)
									+ (l_u8PCProtocolBuf[tempindex+2]<<8)
									+ l_u8PCProtocolBuf[tempindex+3];

			tempindex += 4;
			SETUPALIAS.u32Net1_InvalidRecNum = (l_u8PCProtocolBuf[tempindex]<<24)
									+ (l_u8PCProtocolBuf[tempindex+1]<<16)
									+ (l_u8PCProtocolBuf[tempindex+2]<<8)
									+ l_u8PCProtocolBuf[tempindex+3];

			tempindex += 4;
			SETUPALIAS.u32Net2_InvalidRecNum = (l_u8PCProtocolBuf[tempindex]<<24)
									+ (l_u8PCProtocolBuf[tempindex+1]<<16)
									+ (l_u8PCProtocolBuf[tempindex+2]<<8)
									+ l_u8PCProtocolBuf[tempindex+3];

			tempindex += 4;
			SETUPALIAS.u32DataProcException = (l_u8PCProtocolBuf[tempindex]<<24)
									+ (l_u8PCProtocolBuf[tempindex+1]<<16)
									+ (l_u8PCProtocolBuf[tempindex+2]<<8)
									+ l_u8PCProtocolBuf[tempindex+3];
 			tempindex += 4;

			SETUPALIAS.n32LaserHorizOff = (l_u8PCProtocolBuf[tempindex]<<24)
									+ (l_u8PCProtocolBuf[tempindex+1]<<16)
									+ (l_u8PCProtocolBuf[tempindex+2]<<8)
									+ l_u8PCProtocolBuf[tempindex+3];
		    tempindex += 4;
				//u16VerticalZeroPos set//
			SETUPALIAS.u16VerticalZeroPos0 = (l_u8PCProtocolBuf[tempindex]<<8)
					+ l_u8PCProtocolBuf[tempindex+1];
			tempindex += 2;
			SETUPALIAS.u16VerticalZeroPos1 = (l_u8PCProtocolBuf[tempindex]<<8)
					+ l_u8PCProtocolBuf[tempindex+1];
			tempindex += 2;
			SETUPALIAS.u16VerticalZeroPos2 = (l_u8PCProtocolBuf[tempindex]<<8)
					+ l_u8PCProtocolBuf[tempindex+1];
			tempindex += 2;
			SETUPALIAS.u16VerticalZeroPos3 = (l_u8PCProtocolBuf[tempindex]<<8)
					+ l_u8PCProtocolBuf[tempindex+1];
			tempindex += 2;
				//u16VerticalZeroPos set//
//			SETUPALIAS.u16InclineZeroPos = (l_u8PCProtocolBuf[tempindex]<<8)
//					+ l_u8PCProtocolBuf[tempindex+1];
//			tempindex += 2;
				//u16InclineZeroPos set//
			SETUPALIAS.u16StartPtNum0 =(l_u8PCProtocolBuf[tempindex]<<8)
					+ l_u8PCProtocolBuf[tempindex+1];
			tempindex += 2;
				//u16StartPtNum set//
 			SETUPALIAS.u16EndPtNum0 = (l_u8PCProtocolBuf[tempindex]<<8)
					+ l_u8PCProtocolBuf[tempindex+1];
			tempindex += 2;
			SETUPALIAS.u16StartPtNum1 =(l_u8PCProtocolBuf[tempindex]<<8)
					+ l_u8PCProtocolBuf[tempindex+1];
			tempindex += 2;
				//u16StartPtNum set//
 			SETUPALIAS.u16EndPtNum1= (l_u8PCProtocolBuf[tempindex]<<8)
					+ l_u8PCProtocolBuf[tempindex+1];
			tempindex += 2;
			SETUPALIAS.u16StartPtNum2 =(l_u8PCProtocolBuf[tempindex]<<8)
					+ l_u8PCProtocolBuf[tempindex+1];
			tempindex += 2;
				//u16StartPtNum set//
 			SETUPALIAS.u16EndPtNum2 = (l_u8PCProtocolBuf[tempindex]<<8)
					+ l_u8PCProtocolBuf[tempindex+1];
			tempindex += 2;
			SETUPALIAS.u16StartPtNum3 =(l_u8PCProtocolBuf[tempindex]<<8)
					+ l_u8PCProtocolBuf[tempindex+1];
			tempindex += 2;
				//u16StartPtNum set//
 			SETUPALIAS.u16EndPtNum3 = (l_u8PCProtocolBuf[tempindex]<<8)
					+ l_u8PCProtocolBuf[tempindex+1];
			tempindex += 2;

			SETUPALIAS.u8LaserDevType = l_u8PCProtocolBuf[tempindex++];
			SETUPALIAS.u8TrafficType = l_u8PCProtocolBuf[tempindex++];
			SETUPALIAS.u8RoadType = l_u8PCProtocolBuf[tempindex++];	     // 道路类型 0-- 国道 1-- 高速
			SETUPALIAS.u8LaneNum = l_u8PCProtocolBuf[tempindex++];	     //车道数
			SETUPALIAS.u8NetType = l_u8PCProtocolBuf[tempindex++];       //网络连接：0 -- 无线 1 -- 有线
			SETUPALIAS.u8SDEnable = l_u8PCProtocolBuf[tempindex++];

			AddCrc16((uint8 *)&SETUPALIAS,sizeof(SETUPALIAS)-2);				

			if(Write256_full(BUF0ADDR, (uint8 *)&SETUPALIAS, sizeof(SETUPALIAS)) && Write256_full(BUF0ADDR_BK, (uint8 *)&SETUPALIAS, sizeof(SETUPALIAS)) )
			{
			}
			else
			{
				break;
			}
			
			l_u8PCProtocolBuf[0] = 0xFF;
			l_u8PCProtocolBuf[1] = 0xFF;
			l_u8PCProtocolBuf[2] = CMD_SETPARAM;			
			l_u8PCProtocolBuf[3] = 0;
			l_u8PCProtocolBuf[4] = 7;
			AddCrc16(l_u8PCProtocolBuf, 5);
			U5SendBytes(l_u8PCProtocolBuf, 7);	
			OSTimeDly(1000);
			OSSchedLock();
			ResetSystem(); 
			break;
		case CMD_RESETDEVICE://复位
			l_u8PCProtocolBuf[0] = 0xFF;
			l_u8PCProtocolBuf[1] = 0xFF;
			l_u8PCProtocolBuf[2] = CMD_RESETDEVICE;
			l_u8PCProtocolBuf[3] = 0;
			l_u8PCProtocolBuf[4] = 7;
			AddCrc16(l_u8PCProtocolBuf, 5);
			U5SendBytes(l_u8PCProtocolBuf, 7);
			OSTimeDly(1000);
			OSSchedLock();
			ResetSystem();
			break;
		case CMD_INITDEVICE:             //初始化控制器
			l_u8PCProtocolBuf[0] = 0xFF;
			l_u8PCProtocolBuf[1] = 0xFF;
			l_u8PCProtocolBuf[2] = CMD_INITDEVICE;
			l_u8PCProtocolBuf[3] = 0;
			l_u8PCProtocolBuf[4] = 7;
			AddCrc16(l_u8PCProtocolBuf, 5);
			U5SendBytes(l_u8PCProtocolBuf, 7);
			JZInit();                    //初始化控制器
			AddCrc16((uint8 *)&SETUPALIAS,sizeof(SETUPALIAS)-2);	 
			if(Write256_full(BUF0ADDR, (uint8 *)&SETUPALIAS, sizeof(SETUPALIAS)) && Write256_full(BUF0ADDR_BK, (uint8 *)&SETUPALIAS, sizeof(SETUPALIAS)) )
			{ 
			}
			else
			{
				break;
			}
 

			OSTimeDly(1000);
			OSSchedLock();
			ResetSystem();
			break;
		case CMD_GETDEVICETIME:             //获取控制器时间 20130716
			tempindex = 5;
			l_u8PCProtocolBuf[tempindex++] = SEC;
			l_u8PCProtocolBuf[tempindex++] = MIN;
			l_u8PCProtocolBuf[tempindex++] = HOUR;
			l_u8PCProtocolBuf[tempindex++] = WEEK;
			l_u8PCProtocolBuf[tempindex++] = DAY;
			l_u8PCProtocolBuf[tempindex++] = MONTH;
			l_u8PCProtocolBuf[tempindex++] = YEAR % 100;
			l_u8PCProtocolBuf[tempindex++] = YEAR / 100;

			l_u8PCProtocolBuf[0] = 0xFF;
			l_u8PCProtocolBuf[1] = 0xFF;
			l_u8PCProtocolBuf[2] = CMD_GETDEVICETIME;					
			l_u8PCProtocolBuf[3] = ((tempindex + 2)>>8)&0xFF;//协议帧字节数
			l_u8PCProtocolBuf[4] = (tempindex + 2)&0xFF;
			AddCrc16(l_u8PCProtocolBuf,	tempindex);
			U5SendBytes(l_u8PCProtocolBuf, tempindex + 2);			
			break;
		case CMD_SETDEVICETIME:             //设置控制器时间 20130716			
			memset(l_u8PCProtocolBuf,0,sizeof(l_u8PCProtocolBuf));
			memcpy(l_u8PCProtocolBuf, pUartDataBuf, sizeof(l_u8PCProtocolBuf));
			dev_time.u8Second =	l_u8PCProtocolBuf[5];
			dev_time.u8Minute = l_u8PCProtocolBuf[6];
			dev_time.u8Hour = l_u8PCProtocolBuf[7];
			dev_time.u8Week = l_u8PCProtocolBuf[8];
			dev_time.u8Day = l_u8PCProtocolBuf[9];
			dev_time.u8Month = l_u8PCProtocolBuf[10];
			dev_time.u16Year = l_u8PCProtocolBuf[11] + l_u8PCProtocolBuf[12]*100; 
		
		
			SetRTCTime(&dev_time);		
			break;
		case CMD_GETDEVICE_RDID:             //获取识别码 20130717
			tempindex = 5;
			for(i=0; i<16; i++)
			{
				l_u8PCProtocolBuf[tempindex++] = RDid[i];	
			}
			for(i=0; i<15; i++)
			{
				l_u8PCProtocolBuf[tempindex++] = RDNum[i];
			}

			l_u8PCProtocolBuf[0] = 0xFF;
			l_u8PCProtocolBuf[1] = 0xFF;
			l_u8PCProtocolBuf[2] = CMD_GETDEVICE_RDID;					
			l_u8PCProtocolBuf[3] = ((tempindex + 2)>>8)&0xFF;//协议帧字节数
			l_u8PCProtocolBuf[4] = (tempindex + 2)&0xFF;

			AddCrc16(l_u8PCProtocolBuf,	tempindex);
			U5SendBytes(l_u8PCProtocolBuf, tempindex + 2);			
			break;
		case CMD_SETDEVICE_RDID:             //设置识别码 20130717
			
			memset(l_u8PCProtocolBuf,0,sizeof(l_u8PCProtocolBuf));
			memcpy(l_u8PCProtocolBuf, pUartDataBuf, sizeof(l_u8PCProtocolBuf));

			tempindex = 5;
			for(i=0; i<16; i++)
			{
				 RDid[i] = l_u8PCProtocolBuf[tempindex++];	
			}
			for(i=0; i<15; i++)
			{
				RDNum[i] = l_u8PCProtocolBuf[tempindex++];
			}
			Write256_full(DEVICECODEADDR, RDid, 16);  //存铁电
			Write256_full(STATIONNUMADDR, RDNum, 15);  //存铁电

	
			break;
		case CMD_GET_LANE_DIR:             //获取上下行参数
			tempindex = 5;

			l_u8PCProtocolBuf[tempindex++] = g_u8LaneDir;	

			l_u8PCProtocolBuf[0] = 0xFF;
			l_u8PCProtocolBuf[1] = 0xFF;
			l_u8PCProtocolBuf[2] = CMD_GET_LANE_DIR;					
			l_u8PCProtocolBuf[3] = ((tempindex + 2)>>8)&0xFF;//协议帧字节数
			l_u8PCProtocolBuf[4] = (tempindex + 2)&0xFF;

			AddCrc16(l_u8PCProtocolBuf,	tempindex);
			U5SendBytes(l_u8PCProtocolBuf, tempindex + 2);			
			break;
		case CMD_SET_LANE_DIR:             //设置上下行参数

			memset(l_u8PCProtocolBuf,0,sizeof(l_u8PCProtocolBuf));
			memcpy(l_u8PCProtocolBuf, pUartDataBuf, sizeof(l_u8PCProtocolBuf));
			tempindex = 5;
            g_u8LaneDir = l_u8PCProtocolBuf[tempindex];
				
			Write256_full(LANEDIRADDR, &g_u8LaneDir, 1);  //存铁电
			l_u8PCProtocolBuf[0] = 0xFF;
			l_u8PCProtocolBuf[1] = 0xFF;
			l_u8PCProtocolBuf[2] = CMD_SET_LANE_DIR;
			l_u8PCProtocolBuf[3] = 0;
			l_u8PCProtocolBuf[4] = 7;
			AddCrc16(l_u8PCProtocolBuf, 5);
			U5SendBytes(l_u8PCProtocolBuf, 7);	
			break;
		case  CMD_GET_THRESHOLD:	  //获取阈值
//			tempindex = 5;
//
//			l_u8PCProtocolBuf[tempindex++] = (g_VehTypeThreshold.u16SmallVehThreshold>>8) & 0xFF;
//		    l_u8PCProtocolBuf[tempindex++] = (g_VehTypeThreshold.u16SmallVehThreshold) & 0xFF;
//
//			l_u8PCProtocolBuf[tempindex++] = (g_VehTypeThreshold.u16SuperVehThreshold>>8) & 0xFF;
//		    l_u8PCProtocolBuf[tempindex++] = (g_VehTypeThreshold.u16SuperVehThreshold) & 0xFF;
//
//			l_u8PCProtocolBuf[0] = 0xFF;
//			l_u8PCProtocolBuf[1] = 0xFF;
//			l_u8PCProtocolBuf[2] = CMD_GET_THRESHOLD;					
//			l_u8PCProtocolBuf[3] = ((tempindex + 2)>>8)&0xFF;//??????
//			l_u8PCProtocolBuf[4] = (tempindex + 2)&0xFF;
//
//			AddCrc16(l_u8PCProtocolBuf,	tempindex);
//			U5SendBytes(l_u8PCProtocolBuf, tempindex + 2);				
			break;
		case  CMD_SET_THRESHOLD:
//			memset(l_u8PCProtocolBuf,0,sizeof(l_u8PCProtocolBuf));
//			memcpy(l_u8PCProtocolBuf, pUartDataBuf, sizeof(l_u8PCProtocolBuf));
//			tempindex = 5;
//			g_VehTypeThreshold.u16SmallVehThreshold = (l_u8PCProtocolBuf[tempindex]<<8)+l_u8PCProtocolBuf[tempindex+1];
//			tempindex += 2;	
//			g_VehTypeThreshold.u16SuperVehThreshold = (l_u8PCProtocolBuf[tempindex]<<8)+l_u8PCProtocolBuf[tempindex+1];	
//			Write256_full(BUF2ADDR,(uint8 *)&g_VehTypeThreshold, sizeof(g_VehTypeThreshold));

			break;

#if 1 == TEST_PROBE												//20140311
		case WJ_CMD_DEVICERESETINFO:					
			SendDevResetInfo();										
			break;
		case WJ_CMD_CLEARRESETIFNOBUF:
			ClearResetInfoBuf();			
			break;
#endif
		default:	
			break;
	}
	memset(l_u8PCProtocolBuf, 0, sizeof(l_u8PCProtocolBuf));
	tempindex = 0;			
}
#if 1 == TEST_PROBE
static void SendDevResetInfo(void)
{
	uint8	i = 0;
	uint8	j = 0;
	uint8	u8TmpPos = 0;
	uint8	u8StartPos = 0;
	uint8	u8TmpEntries = 0;
	uint8	u8tempBuf[RESETINFO_BUFFERSIZE*15] = {0};
	uint8	u8tempIndex = 0;
	uint32	u32TmpResetTimes = 0;
	CycleBufferStruct  *pTmpResetCycBuf = (CycleBufferStruct *)malloc(sizeof(CycleBufferStruct) + RESETINFO_BUFFERSIZE * sizeof(SystemTime));

	//u32TmpResetTimes = g_sspSetup.resetCnt;
	ReadC256(RESETINFOADDR, (uint8 *)pTmpResetCycBuf, sizeof(CycleBufferStruct) + RESETINFO_BUFFERSIZE * sizeof(SystemTime));

	u8tempIndex = 0;
	u8tempBuf[u8tempIndex++] = 0xFF;
	u8tempBuf[u8tempIndex++] = 0xFF;
	u8tempBuf[u8tempIndex++] = WJ_CMD_DEVICERESETINFO;
	u8tempIndex += 2;//字节大小字段				

//	//复位次数
//	u8tempBuf[u8tempIndex++] = (u32TmpResetTimes>>24) & 0xFF;
//	u8tempBuf[u8tempIndex++] = (u32TmpResetTimes>>16) & 0xFF;
//	u8tempBuf[u8tempIndex++] = (u32TmpResetTimes>>8) & 0xFF;
//	u8tempBuf[u8tempIndex++] = u32TmpResetTimes & 0xFF;

	//复位时间及复位类型
	u8TmpEntries = pTmpResetCycBuf->u8CurrentEntries;
	u8TmpPos = pTmpResetCycBuf->u8CurrentPos;
	u8StartPos = (RESETINFO_BUFFERSIZE + u8TmpPos - u8TmpEntries)%RESETINFO_BUFFERSIZE;

	for(i=u8StartPos,j=0; j<u8TmpEntries; i++,j++)	//20130703修改
	{
		i = i % RESETINFO_BUFFERSIZE;
		u8tempBuf[u8tempIndex++] = b2bcd(pTmpResetCycBuf->SysTimeTable[i].u16Year/100);
		u8tempBuf[u8tempIndex++] = b2bcd(pTmpResetCycBuf->SysTimeTable[i].u16Year%100);
		u8tempBuf[u8tempIndex++] = b2bcd(pTmpResetCycBuf->SysTimeTable[i].u8Month);
		u8tempBuf[u8tempIndex++] = b2bcd(pTmpResetCycBuf->SysTimeTable[i].u8Day);
		u8tempBuf[u8tempIndex++] = b2bcd(pTmpResetCycBuf->SysTimeTable[i].u8Hour);
		u8tempBuf[u8tempIndex++] = b2bcd(pTmpResetCycBuf->SysTimeTable[i].u8Minute);
		u8tempBuf[u8tempIndex++] = b2bcd(pTmpResetCycBuf->SysTimeTable[i].u8Second);
		u8tempBuf[u8tempIndex++] = pTmpResetCycBuf->u8Type[i];//复位类型1：外部复位 2：看门狗复位
	}
	u8tempBuf[3] = ((u8tempIndex + 2)>>8)&0xFF;//协议帧字节数
	u8tempBuf[4] = (u8tempIndex + 2)&0xFF;
	AddCrc16(u8tempBuf,	u8tempIndex);
	U5SendBytes(u8tempBuf, u8tempIndex + 2);
	free(pTmpResetCycBuf);
	pTmpResetCycBuf = NULL;		
}

void ClearResetInfoBuf(void)
{
	uint8	u8tempBuf[RESETINFO_BUFFERSIZE*15] = {0};
	uint8	u8tempIndex = 0;
	CycleBufferStruct  *pTmpResetCycBuf;	
		
	u8tempIndex = 0;
	u8tempBuf[u8tempIndex++] = 0xFF;
	u8tempBuf[u8tempIndex++] = 0xFF;
	u8tempBuf[u8tempIndex++] = WJ_CMD_CLEARRESETIFNOBUF;		
	u8tempIndex += 2;//字节大小字段	

	pTmpResetCycBuf = (CycleBufferStruct *)malloc(sizeof(CycleBufferStruct) + RESETINFO_BUFFERSIZE * sizeof(SystemTime));
	if(pTmpResetCycBuf == NULL)
	{
		pTmpResetCycBuf = (CycleBufferStruct *)malloc(sizeof(CycleBufferStruct) + RESETINFO_BUFFERSIZE * sizeof(SystemTime));	
	}
	memset((uint8 *)pTmpResetCycBuf, 0, sizeof(CycleBufferStruct) + RESETINFO_BUFFERSIZE * sizeof(SystemTime));
	if (TRUE == WriteC256(RESETINFOADDR, (uint8 *)pTmpResetCycBuf, sizeof(CycleBufferStruct) + RESETINFO_BUFFERSIZE * sizeof(SystemTime)))
	{
		u8tempBuf[u8tempIndex++] = 0x01;			 //成功			
	}
	else
	{
		u8tempBuf[u8tempIndex++] = 0x02;			 //失败	
	}
	u8tempBuf[3] = ((u8tempIndex + 2)>>8)&0xFF;//协议帧字节数
	u8tempBuf[4] = (u8tempIndex + 2)&0xFF;		
	AddCrc16(u8tempBuf,	u8tempIndex);
	U5SendBytes(u8tempBuf, u8tempIndex + 2);				
	free(pTmpResetCycBuf);
	pTmpResetCycBuf = NULL;
}
#endif
