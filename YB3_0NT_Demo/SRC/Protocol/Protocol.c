#include "Protocol.h"
#include "WT_Task.h"
#include "RD_data.H"

#define CMD_QUERYPARAM	1
#define	CMD_SETPARAM	2
#define	CMD_RESETDEVICE	3
#define CMD_INITDEVICE  4

#define CMD_GETDEVICETIME  0x10
#define CMD_SETDEVICETIME  0x11
#define CMD_GETDEVICE_RDID 0X12  // ��ȡ�豸ʶ�����վ����
#define CMD_SETDEVICE_RDID 0X13	 // �����豸ʶ�����վ����
#define CMD_GET_LANE_DIR   0X20  // ��ȡ�豸����������
#define CMD_SET_LANE_DIR   0X21  // �����豸����������
#define CMD_GET_THRESHOLD  0X14  // ��ȡ��ֵ
#define CMD_SET_THRESHOLD  0X15  // ������ֵ

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
	uint8	l_u8RecNum = 0;			//���յ����ֽ���
	static	uint8	l_u8CurrentSizeCnt = 0;	//��ǰ������Ч���ݼ���
	static	uint16 	l_u16ProtocolFrameLength = 0;//�����Ǿ�̬����20130424
	uint8	l_u8tempPFrameLenHi = 0;
	uint8	l_u8tempPFrameLenLo = 0;
	uint8	i = 0;

	l_u8RecNum = U5ReciveByte(l_u8tempPCCmdBuf, 0);

	for(i=0; i<l_u8RecNum; i++)
	{
		if (0 == l_u8CurrentSizeCnt)
		{
		   if (0 == (l_u8tempPCCmdBuf[i]^0xFF))	//order0:֡ͷ1
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
		   if (0 == (l_u8tempPCCmdBuf[i]^0xFF))	//order1:֡ͷ2
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
		   else	 //order2:�����
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
			l_u16ProtocolFrameLength = (l_u8tempPFrameLenHi<<8) + l_u8tempPFrameLenLo;//Э���ֽ���
			l_u8CurrentSizeCnt++;
		}
		else if ((l_u8CurrentSizeCnt > 4) && (l_u8CurrentSizeCnt < l_u16ProtocolFrameLength))
		{
			*(pUartDataBuf+l_u8CurrentSizeCnt) =  l_u8tempPCCmdBuf[i];
			l_u8CurrentSizeCnt++;
		}

		if (l_u16ProtocolFrameLength == l_u8CurrentSizeCnt && l_u8CurrentSizeCnt>=2)
		{			
			if(CheckCrc(pUartDataBuf,l_u16ProtocolFrameLength-2)==0)//crcУ��ʧ��
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
			else //У��ɹ�
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
		case CMD_QUERYPARAM:	  //Ϊ������ѯ
			
			ret = Read256_full(BUF0ADDR,(uint8 *)&SETUPALIAS, sizeof(SETUPALIAS));
			tempindex = 5;
			memcpy(l_u8PCProtocolBuf+tempindex,SETUPALIAS.au8ProgramVersion,11);	 //�汾��
			tempindex += 11;
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u8InstallFlag;         //��װ��ʽ 0 ��׮ 1 ��׮
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32DevID>>24; 	 //�豸ID 4�ֽ�
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32DevID>>16; 
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32DevID>>8; 
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32DevID; 
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u8BaudRate;
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u8DOG;
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u8TrafficType; //�豸����
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u8NetType; 	  //�����ϴ�����
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.resetCnt>>24; //�������� 4�ֽ�
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.resetCnt>>16; //
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.resetCnt>>8; 	  //
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.resetCnt; 	  //
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u8LaserDevType; 	  //�豸����
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u8SDEnable;	 	  //SD��ʹ��
				   //������0�߶�ֵ
		
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.J0_Height>>8)&0xFF;                           	  	  
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.J0_Height&0xFF; 			
			   //������1�߶�ֵ
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.J1_Height>>8)&0xFF;                           	  	  
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.J1_Height&0xFF;
				  //������2�߶�ֵ
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.J2_Height>>8)&0xFF;                           	  	  
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.J2_Height&0xFF;
			   //������3�߶�ֵ			   			
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.J3_Height>>8)&0xFF;                           	  	  
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.J3_Height&0xFF;	   
			 	//������ 0��1��ˮƽ����
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.LaserDistance>>8)&0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.LaserDistance&0xFF;		  
				//����0���ĵ�
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u16J0ZeroPos>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u16J0ZeroPos & 0xFF;
																			//����0��ʼ��			
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u16J0StartPos>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u16J0StartPos & 0xFF;						
																			 //����0��ֹ��
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u16J0EndPos>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u16J0EndPos & 0xFF;
																			 //����1���ĵ�
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u16J1ZeroPos>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u16J1ZeroPos & 0xFF;
																			  	//1��ʼ��
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u16J1StartPos>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u16J1StartPos & 0xFF;						
																				//1��ֹ��
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u16J1EndPos>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u16J1EndPos & 0xFF;
																			   //����2���ĵ�
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u16J2ZeroPos>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u16J2ZeroPos & 0xFF;
																				 //2��ʼ��
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u16J2StartPos>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u16J2StartPos & 0xFF;						
																				//2��ֹ��
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u16J2EndPos>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u16J2EndPos & 0xFF;
																			   //����3���ĵ�
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u16J3ZeroPos>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u16J3ZeroPos & 0xFF;
																				 //3��ʼ��
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u16J3StartPos>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u16J3StartPos & 0xFF;						
																				 //3��ֹ��
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u16J3EndPos>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u16J3EndPos & 0xFF;   			
																		 //�������	 
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.LaneWide>>8)&0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.LaneWide&0xFF;
																	 //����0�೵������
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.MedianWide>>8)&0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.MedianWide&0xFF; 
                                                                   //����1�೵������
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.MedianLeftWide>>8)&0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.MedianLeftWide&0xFF;
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u8LaneNum;	    //������
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u8RoadType;	    // ��·���� 0-- ���� 1-- ����
																		//������ip����					
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32LocalIPAddress>>24;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32LocalIPAddress>>16) & 0xFF;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32LocalIPAddress>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32LocalIPAddress & 0xFF;
																	   //������port
//			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32LocalPortNO>>24;
//			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32LocalPortNO>>16) & 0xFF;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32LocalPortNO>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32LocalPortNO & 0xFF;	

																	   //��������������
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32SubMask>>24;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32SubMask>>16) & 0xFF;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32SubMask>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32SubMask & 0xFF;	
																	  //����������
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32GatewayIP>>24;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32GatewayIP>>16) & 0xFF;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32GatewayIP>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32GatewayIP & 0xFF;
		   															//������mac
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.au8LocalMAC[0];
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.au8LocalMAC[1];
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.au8LocalMAC[2];    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.au8LocalMAC[3];					
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.au8LocalMAC[4];
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.au8LocalMAC[5];
																	//����0IP
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.J0_IP>>24;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.J0_IP>>16) & 0xFF;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.J0_IP>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.J0_IP & 0xFF;					
																	//����0 port
//			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.J0_Port>>24;
//			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.J0_Port>>16) & 0xFF;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.J0_Port>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.J0_Port & 0xFF;
																   //����1 IP
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.J1_IP>>24;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.J1_IP>>16) & 0xFF;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.J1_IP>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.J1_IP & 0xFF;					
																   //����1 port
//			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.J1_Port>>24;
//			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.J1_Port>>16) & 0xFF;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.J1_Port>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.J1_Port & 0xFF;
																  //����2 IP
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.J2_IP>>24;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.J2_IP>>16) & 0xFF;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.J2_IP>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.J2_IP & 0xFF;					
																 //����2 port
//			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.J2_Port>>24;
//			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.J2_Port>>16) & 0xFF;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.J2_Port>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.J2_Port & 0xFF;
																 //����3 IP
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.J3_IP>>24;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.J3_IP>>16) & 0xFF;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.J3_IP>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.J3_IP & 0xFF;					
																 //����3 port
//			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.J3_Port>>24;
//			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.J3_Port>>16) & 0xFF;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.J3_Port>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.J3_Port & 0xFF;
																 //������IP 
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32ServerIP>>24;
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32ServerIP>>16;
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32ServerIP>>8;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32ServerIP;
																 //������ port
//			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u16ServerPort>>24;
//			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u16ServerPort>>16;
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u16ServerPort>>8;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u16ServerPort;
																			   //����1�Ͽ�
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32Net1_DisconnectNum>>24;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32Net1_DisconnectNum>>16) & 0xFF;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32Net1_DisconnectNum>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32Net1_DisconnectNum & 0xFF;
																			  //����1��Ч
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32Net1_InvalidRecNum>>24;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32Net1_InvalidRecNum>>16) & 0xFF;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32Net1_InvalidRecNum>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32Net1_InvalidRecNum & 0xFF;
																			 //����2�Ͽ�
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32Net2_DisconnectNum>>24;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32Net2_DisconnectNum>>16) & 0xFF;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32Net2_DisconnectNum>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32Net2_DisconnectNum & 0xFF;
																			//����2��Ч
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32Net2_InvalidRecNum>>24;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32Net2_InvalidRecNum>>16) & 0xFF;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32Net2_InvalidRecNum>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32Net2_InvalidRecNum & 0xFF;
			//2:															 //����3�Ͽ�
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32Net3_DisconnectNum>>24;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32Net3_DisconnectNum>>16) & 0xFF;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32Net3_DisconnectNum>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32Net3_DisconnectNum & 0xFF;
		
																		 //����3��Ч
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32Net3_InvalidRecNum>>24;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32Net3_InvalidRecNum>>16) & 0xFF;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32Net3_InvalidRecNum>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32Net3_InvalidRecNum & 0xFF;
																		 //����4�Ͽ�
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32Net4_DisconnectNum>>24;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32Net4_DisconnectNum>>16) & 0xFF;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32Net4_DisconnectNum>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32Net4_DisconnectNum & 0xFF; 
																		 //����4��Ч
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32Net4_InvalidRecNum>>24;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32Net4_InvalidRecNum>>16) & 0xFF;
			l_u8PCProtocolBuf[tempindex++] = (SETUPALIAS.u32Net4_InvalidRecNum>>8) & 0xFF;    
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.u32Net4_InvalidRecNum & 0xFF;
			l_u8PCProtocolBuf[tempindex++] = 0x00;//�쳣��	
			l_u8PCProtocolBuf[tempindex++] = 0x00;//�쳣��	
			l_u8PCProtocolBuf[tempindex++] = 0x00;//�쳣��	
			l_u8PCProtocolBuf[tempindex++] = 0x00;//�쳣��		
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.au8ReserveByte[0];// au8ReserveByte3[0]; Ԥ��4�ֽ� 
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.au8ReserveByte[1];//au8ReserveByte3[0]
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.au8ReserveByte[2];//au8ReserveByte3[0]
			l_u8PCProtocolBuf[tempindex++] = SETUPALIAS.au8ReserveByte[3];//au8ReserveByte3[0]
			l_u8PCProtocolBuf[0] = 0xFF;
			l_u8PCProtocolBuf[1] = 0xFF;
			l_u8PCProtocolBuf[2] = CMD_QUERYPARAM;					
			l_u8PCProtocolBuf[3] = ((tempindex + 2)>>8)&0xFF;//Э��֡�ֽ���
			l_u8PCProtocolBuf[4] = (tempindex + 2)&0xFF; 
			AddCrc16(l_u8PCProtocolBuf,	tempindex);
			U5SendBytes(l_u8PCProtocolBuf, tempindex + 2);
			break;
		case CMD_SETPARAM:	  //Ϊ��������
			memset(l_u8PCProtocolBuf,0,sizeof(l_u8PCProtocolBuf));
			memcpy(l_u8PCProtocolBuf, pUartDataBuf, sizeof(l_u8PCProtocolBuf));
			tempindex = 5;
			//�����汾������//
			tempindex += 11;
			SETUPALIAS.u8InstallFlag = l_u8PCProtocolBuf[tempindex++];	  //��װ��ʽ 0 ��׮ 1 ��׮ 
			SETUPALIAS.u32DevID	= 	(l_u8PCProtocolBuf[tempindex]<<24)	  // �豸ID
									+ (l_u8PCProtocolBuf[tempindex+1]<<16)
									+ (l_u8PCProtocolBuf[tempindex+2]<<8)
									+ l_u8PCProtocolBuf[tempindex+3];
									tempindex += 4;
			SETUPALIAS.u8BaudRate = l_u8PCProtocolBuf[tempindex++];	     //������
			SETUPALIAS.u8DOG = l_u8PCProtocolBuf[tempindex++];			 //���Ź�
			SETUPALIAS.u8TrafficType = l_u8PCProtocolBuf[tempindex++];	 //�豸���� 
			SETUPALIAS.u8NetType = l_u8PCProtocolBuf[tempindex++];  	 //�ϴ���ʽ
			SETUPALIAS.resetCnt = (l_u8PCProtocolBuf[tempindex]<<24)	 //��������
									+ (l_u8PCProtocolBuf[tempindex+1]<<16)
									+ (l_u8PCProtocolBuf[tempindex+2]<<8)
									+ l_u8PCProtocolBuf[tempindex+3]; 
									tempindex += 4; 
			SETUPALIAS.u8LaserDevType = l_u8PCProtocolBuf[tempindex++];	 //���������� 
			SETUPALIAS.u8SDEnable = l_u8PCProtocolBuf[tempindex++];	 //SD��ʹ��
			SETUPALIAS.J0_Height = 	 (l_u8PCProtocolBuf[tempindex]<<8) //	������ 0,1,2,3�߶�
									+ l_u8PCProtocolBuf[tempindex+1];
									tempindex += 2;
		    SETUPALIAS.J1_Height = (l_u8PCProtocolBuf[tempindex]<<8)
									+ l_u8PCProtocolBuf[tempindex+1];
								    tempindex += 2;
			SETUPALIAS.J2_Height = (l_u8PCProtocolBuf[tempindex]<<8)
									+ l_u8PCProtocolBuf[tempindex+1];
									tempindex += 2;
			SETUPALIAS.J3_Height =  (l_u8PCProtocolBuf[tempindex]<<8)
									+ l_u8PCProtocolBuf[tempindex+1];
									tempindex += 2;
			SETUPALIAS.LaserDistance = (l_u8PCProtocolBuf[tempindex]<<8)  //������ˮƽ����
									+ l_u8PCProtocolBuf[tempindex+1];
									tempindex += 2;
			SETUPALIAS.u16J0ZeroPos =(l_u8PCProtocolBuf[tempindex]<<8)	  //������ 0 �е� ��� �յ�
									+ l_u8PCProtocolBuf[tempindex+1];
									tempindex += 2;                     
			SETUPALIAS.u16J0StartPos = (l_u8PCProtocolBuf[tempindex]<<8)
									+ (l_u8PCProtocolBuf[tempindex+1]);									 
									tempindex += 2;                      
			SETUPALIAS.u16J0EndPos = (l_u8PCProtocolBuf[tempindex]<<8)
									+ (l_u8PCProtocolBuf[tempindex+1]);								 
									tempindex += 2;	 		
		    SETUPALIAS.u16J1ZeroPos = (l_u8PCProtocolBuf[tempindex]<<8)	  //������ 1 �е� ��� �յ�
									+ (l_u8PCProtocolBuf[tempindex+1]);									
									tempindex += 2;                        
			SETUPALIAS.u16J1StartPos = (l_u8PCProtocolBuf[tempindex]<<8)
									+ (l_u8PCProtocolBuf[tempindex+1]);							
									tempindex += 2;                      
			SETUPALIAS.u16J1EndPos = (l_u8PCProtocolBuf[tempindex]<<8)
									+ (l_u8PCProtocolBuf[tempindex+1]);								 
									tempindex += 2;	  			  
			SETUPALIAS.u16J2ZeroPos = (l_u8PCProtocolBuf[tempindex]<<8)	  //������ 2 �е� ��� �յ�
									+ (l_u8PCProtocolBuf[tempindex+1]);									
									tempindex += 2;                        
			SETUPALIAS.u16J2StartPos = (l_u8PCProtocolBuf[tempindex]<<8)
									+ (l_u8PCProtocolBuf[tempindex+1]);								
									tempindex += 2;                      
			SETUPALIAS.u16J2EndPos = (l_u8PCProtocolBuf[tempindex]<<8)
									+ (l_u8PCProtocolBuf[tempindex+1]);								 
									tempindex += 2;	
		   SETUPALIAS.u16J3ZeroPos = (l_u8PCProtocolBuf[tempindex]<<8)	  //������ 3 �е� ��� �յ�
									+ (l_u8PCProtocolBuf[tempindex+1]);								 
									tempindex += 2;                         
			SETUPALIAS.u16J3StartPos = (l_u8PCProtocolBuf[tempindex]<<8)
									+ (l_u8PCProtocolBuf[tempindex+1]);								 
									tempindex += 2;                      
			SETUPALIAS.u16J3EndPos = (l_u8PCProtocolBuf[tempindex]<<8)
									+ (l_u8PCProtocolBuf[tempindex+1]);
									tempindex += 2;
			SETUPALIAS.LaneWide =  (l_u8PCProtocolBuf[tempindex]<<8)  //���� ���
									+ l_u8PCProtocolBuf[tempindex+1];
			tempindex += 2;
			SETUPALIAS.MedianWide = (l_u8PCProtocolBuf[tempindex]<<8)//0����೵������
									+ (l_u8PCProtocolBuf[tempindex+1]);									
			tempindex += 2;
			SETUPALIAS.MedianLeftWide = (l_u8PCProtocolBuf[tempindex]<<8)
									+ (l_u8PCProtocolBuf[tempindex+1]);
			tempindex += 2;
			SETUPALIAS.u8LaneNum = l_u8PCProtocolBuf[tempindex++];// ������
			SETUPALIAS.u8RoadType = l_u8PCProtocolBuf[tempindex++];// ��������

			SETUPALIAS.u32LocalIPAddress = (l_u8PCProtocolBuf[tempindex]<<24)	//������IP port
									+ (l_u8PCProtocolBuf[tempindex+1]<<16)
									+ (l_u8PCProtocolBuf[tempindex+2]<<8)
									+ l_u8PCProtocolBuf[tempindex+3];

			tempindex += 4;
			SETUPALIAS.u32LocalPortNO = (l_u8PCProtocolBuf[tempindex]<<8)
										+ l_u8PCProtocolBuf[tempindex+1];
			tempindex += 2;
			SETUPALIAS.u32SubMask = (l_u8PCProtocolBuf[tempindex]<<24)		 //��������  ���� mac 
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
			SETUPALIAS.J0_IP = (l_u8PCProtocolBuf[tempindex]<<24)
									+ (l_u8PCProtocolBuf[tempindex+1]<<16)
									+ (l_u8PCProtocolBuf[tempindex+2]<<8)
									+ l_u8PCProtocolBuf[tempindex+3]; 
			tempindex += 4;
			SETUPALIAS.J0_Port = (l_u8PCProtocolBuf[tempindex]<<8)
								+ l_u8PCProtocolBuf[tempindex+1];
			tempindex += 2;
			SETUPALIAS.J1_IP = (l_u8PCProtocolBuf[tempindex]<<24)
									+ (l_u8PCProtocolBuf[tempindex+1]<<16)
									+ (l_u8PCProtocolBuf[tempindex+2]<<8)
									+ l_u8PCProtocolBuf[tempindex+3];
			tempindex += 4;
			SETUPALIAS.J1_Port =  (l_u8PCProtocolBuf[tempindex]<<8)
									+ l_u8PCProtocolBuf[tempindex+1];		
			tempindex += 2;
			SETUPALIAS.J2_IP = (l_u8PCProtocolBuf[tempindex]<<24)
									+ (l_u8PCProtocolBuf[tempindex+1]<<16)
									+ (l_u8PCProtocolBuf[tempindex+2]<<8)
									+ l_u8PCProtocolBuf[tempindex+3];

			tempindex += 4;
			SETUPALIAS.J2_Port = (l_u8PCProtocolBuf[tempindex]<<8)
									+ l_u8PCProtocolBuf[tempindex+1];
			tempindex += 2;	
			SETUPALIAS.J3_IP = (l_u8PCProtocolBuf[tempindex]<<24)
									+ (l_u8PCProtocolBuf[tempindex+1]<<16)
									+ (l_u8PCProtocolBuf[tempindex+2]<<8)
									+ l_u8PCProtocolBuf[tempindex+3];
			tempindex += 4;
			SETUPALIAS.J3_Port =  (l_u8PCProtocolBuf[tempindex]<<8)
									+ l_u8PCProtocolBuf[tempindex+1];
			tempindex += 2;		
			SETUPALIAS.u32ServerIP = (l_u8PCProtocolBuf[tempindex]<<24)
									+ (l_u8PCProtocolBuf[tempindex+1]<<16)
									+ (l_u8PCProtocolBuf[tempindex+2]<<8)
									+ l_u8PCProtocolBuf[tempindex+3];
			tempindex += 4;
			SETUPALIAS.u16ServerPort = (l_u8PCProtocolBuf[tempindex]<<8)
									+ l_u8PCProtocolBuf[tempindex+1];
			tempindex += 2; 
			SETUPALIAS.u32Net1_DisconnectNum = (l_u8PCProtocolBuf[tempindex]<<24)
									+ (l_u8PCProtocolBuf[tempindex+1]<<16)
									+ (l_u8PCProtocolBuf[tempindex+2]<<8)
									+ l_u8PCProtocolBuf[tempindex+3];
			tempindex += 4;
			SETUPALIAS.u32Net1_InvalidRecNum = (l_u8PCProtocolBuf[tempindex]<<24)
									+ (l_u8PCProtocolBuf[tempindex+1]<<16)
									+ (l_u8PCProtocolBuf[tempindex+2]<<8)
									+ l_u8PCProtocolBuf[tempindex+3];
			tempindex += 4;
			SETUPALIAS.u32Net2_DisconnectNum = (l_u8PCProtocolBuf[tempindex]<<24)
									+ (l_u8PCProtocolBuf[tempindex+1]<<16)
									+ (l_u8PCProtocolBuf[tempindex+2]<<8)
									+ l_u8PCProtocolBuf[tempindex+3];
			tempindex += 4;		
			SETUPALIAS.u32Net2_InvalidRecNum = (l_u8PCProtocolBuf[tempindex]<<24)
									+ (l_u8PCProtocolBuf[tempindex+1]<<16)
									+ (l_u8PCProtocolBuf[tempindex+2]<<8)
									+ l_u8PCProtocolBuf[tempindex+3];
			tempindex += 4;
			SETUPALIAS.u32Net3_DisconnectNum = (l_u8PCProtocolBuf[tempindex]<<24)
									+ (l_u8PCProtocolBuf[tempindex+1]<<16)
									+ (l_u8PCProtocolBuf[tempindex+2]<<8)
									+ l_u8PCProtocolBuf[tempindex+3];
			tempindex += 4;		
			SETUPALIAS.u32Net3_InvalidRecNum = (l_u8PCProtocolBuf[tempindex]<<24)
									+ (l_u8PCProtocolBuf[tempindex+1]<<16)
									+ (l_u8PCProtocolBuf[tempindex+2]<<8)
									+ l_u8PCProtocolBuf[tempindex+3];
			tempindex += 4;
			SETUPALIAS.u32Net4_DisconnectNum = (l_u8PCProtocolBuf[tempindex]<<24)
									+ (l_u8PCProtocolBuf[tempindex+1]<<16)
									+ (l_u8PCProtocolBuf[tempindex+2]<<8)
									+ l_u8PCProtocolBuf[tempindex+3];
			tempindex += 4;		
			SETUPALIAS.u32Net4_InvalidRecNum = (l_u8PCProtocolBuf[tempindex]<<24)
									+ (l_u8PCProtocolBuf[tempindex+1]<<16)
									+ (l_u8PCProtocolBuf[tempindex+2]<<8)
									+ l_u8PCProtocolBuf[tempindex+3];
									tempindex += 4;
			SETUPALIAS.u32nonormalNum = (l_u8PCProtocolBuf[tempindex]<<8)
									+ l_u8PCProtocolBuf[tempindex+1];//�쳣��			 	
			tempindex += 4;
			SETUPALIAS.resetCnt = (l_u8PCProtocolBuf[tempindex]<<24)
									+ (l_u8PCProtocolBuf[tempindex+1]<<16)
									+ (l_u8PCProtocolBuf[tempindex+2]<<8)
									+ l_u8PCProtocolBuf[tempindex+3];
			tempindex += 4;
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
		case CMD_RESETDEVICE://��λ
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
		case CMD_INITDEVICE:             //��ʼ��������
			l_u8PCProtocolBuf[0] = 0xFF;
			l_u8PCProtocolBuf[1] = 0xFF;
			l_u8PCProtocolBuf[2] = CMD_INITDEVICE;
			l_u8PCProtocolBuf[3] = 0;
			l_u8PCProtocolBuf[4] = 7;
			AddCrc16(l_u8PCProtocolBuf, 5);
			U5SendBytes(l_u8PCProtocolBuf, 7);
			JZInit();                    //��ʼ��������
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
		case CMD_GETDEVICETIME:             //��ȡ������ʱ�� 20130716
		    GetRTCTime(&g_sstCurTime);
			tempindex = 5;
			l_u8PCProtocolBuf[tempindex++] = g_sstCurTime.u8Second;
			l_u8PCProtocolBuf[tempindex++] = g_sstCurTime.u8Minute;
			l_u8PCProtocolBuf[tempindex++] = g_sstCurTime.u8Hour;
			l_u8PCProtocolBuf[tempindex++] = g_sstCurTime.u8Week;
			l_u8PCProtocolBuf[tempindex++] = g_sstCurTime.u8Day;
			l_u8PCProtocolBuf[tempindex++] = g_sstCurTime.u8Month;
			l_u8PCProtocolBuf[tempindex++] = g_sstCurTime.u16Year % 100;
			l_u8PCProtocolBuf[tempindex++] = g_sstCurTime.u16Year / 100;
			l_u8PCProtocolBuf[0] = 0xFF;
			l_u8PCProtocolBuf[1] = 0xFF;
			l_u8PCProtocolBuf[2] = CMD_GETDEVICETIME;					
			l_u8PCProtocolBuf[3] = ((tempindex + 2)>>8)&0xFF;//Э��֡�ֽ���
			l_u8PCProtocolBuf[4] = (tempindex + 2)&0xFF;
			AddCrc16(l_u8PCProtocolBuf,	tempindex);
			U5SendBytes(l_u8PCProtocolBuf, tempindex + 2);			
			break;
		case CMD_SETDEVICETIME:             //���ÿ�����ʱ�� 20130716			
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
		case CMD_GETDEVICE_RDID:             //��ȡʶ���� 20130717
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
			l_u8PCProtocolBuf[3] = ((tempindex + 2)>>8)&0xFF;//Э��֡�ֽ���
			l_u8PCProtocolBuf[4] = (tempindex + 2)&0xFF;

			AddCrc16(l_u8PCProtocolBuf,	tempindex);
			U5SendBytes(l_u8PCProtocolBuf, tempindex + 2);			
			break;
		case CMD_SETDEVICE_RDID:             //����ʶ���� 20130717
			
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
			Write256_full(DEVICECODEADDR, RDid, 16);  //������
			Write256_full(STATIONNUMADDR, RDNum, 15);  //������	
			break;
		case CMD_GET_LANE_DIR:             //��ȡ�����в���
			tempindex = 5;				  
			l_u8PCProtocolBuf[tempindex++] = g_u8LaneDir;
			l_u8PCProtocolBuf[0] = 0xFF;
			l_u8PCProtocolBuf[1] = 0xFF;
			l_u8PCProtocolBuf[2] = CMD_GET_LANE_DIR;					
			l_u8PCProtocolBuf[3] = ((tempindex + 2)>>8)&0xFF;//Э��֡�ֽ���
			l_u8PCProtocolBuf[4] = (tempindex + 2)&0xFF; 
			AddCrc16(l_u8PCProtocolBuf,	tempindex);
			U5SendBytes(l_u8PCProtocolBuf, tempindex + 2);			
			break;
		case CMD_SET_LANE_DIR:             //���������в���	
			memset(l_u8PCProtocolBuf,0,sizeof(l_u8PCProtocolBuf));
			memcpy(l_u8PCProtocolBuf, pUartDataBuf, sizeof(l_u8PCProtocolBuf));
			tempindex = 5;
            g_u8LaneDir = l_u8PCProtocolBuf[tempindex];	   				
			Write256_full(LANEDIRADDR, &g_u8LaneDir, 1);  //������
			l_u8PCProtocolBuf[0] = 0xFF;
			l_u8PCProtocolBuf[1] = 0xFF;
			l_u8PCProtocolBuf[2] = CMD_SET_LANE_DIR;
			l_u8PCProtocolBuf[3] = 0;
			l_u8PCProtocolBuf[4] = 7;
			AddCrc16(l_u8PCProtocolBuf, 5);
			U5SendBytes(l_u8PCProtocolBuf, 7);	
			break;
		case  CMD_GET_THRESHOLD:	  //��ȡ��ֵ
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
	u8tempIndex += 2;//�ֽڴ�С�ֶ�				

//	//��λ����
//	u8tempBuf[u8tempIndex++] = (u32TmpResetTimes>>24) & 0xFF;
//	u8tempBuf[u8tempIndex++] = (u32TmpResetTimes>>16) & 0xFF;
//	u8tempBuf[u8tempIndex++] = (u32TmpResetTimes>>8) & 0xFF;
//	u8tempBuf[u8tempIndex++] = u32TmpResetTimes & 0xFF;

	//��λʱ�估��λ����
	u8TmpEntries = pTmpResetCycBuf->u8CurrentEntries;
	u8TmpPos = pTmpResetCycBuf->u8CurrentPos;
	u8StartPos = (RESETINFO_BUFFERSIZE + u8TmpPos - u8TmpEntries)%RESETINFO_BUFFERSIZE;

	for(i=u8StartPos,j=0; j<u8TmpEntries; i++,j++)	//20130703�޸�
	{
		i = i % RESETINFO_BUFFERSIZE;
		u8tempBuf[u8tempIndex++] = b2bcd(pTmpResetCycBuf->SysTimeTable[i].u16Year/100);
		u8tempBuf[u8tempIndex++] = b2bcd(pTmpResetCycBuf->SysTimeTable[i].u16Year%100);
		u8tempBuf[u8tempIndex++] = b2bcd(pTmpResetCycBuf->SysTimeTable[i].u8Month);
		u8tempBuf[u8tempIndex++] = b2bcd(pTmpResetCycBuf->SysTimeTable[i].u8Day);
		u8tempBuf[u8tempIndex++] = b2bcd(pTmpResetCycBuf->SysTimeTable[i].u8Hour);
		u8tempBuf[u8tempIndex++] = b2bcd(pTmpResetCycBuf->SysTimeTable[i].u8Minute);
		u8tempBuf[u8tempIndex++] = b2bcd(pTmpResetCycBuf->SysTimeTable[i].u8Second);
		u8tempBuf[u8tempIndex++] = pTmpResetCycBuf->u8Type[i];//��λ����1���ⲿ��λ 2�����Ź���λ
	}
	u8tempBuf[3] = ((u8tempIndex + 2)>>8)&0xFF;//Э��֡�ֽ���
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
	u8tempIndex += 2;//�ֽڴ�С�ֶ�	

	pTmpResetCycBuf = (CycleBufferStruct *)malloc(sizeof(CycleBufferStruct) + RESETINFO_BUFFERSIZE * sizeof(SystemTime));
	if(pTmpResetCycBuf == NULL)
	{
		pTmpResetCycBuf = (CycleBufferStruct *)malloc(sizeof(CycleBufferStruct) + RESETINFO_BUFFERSIZE * sizeof(SystemTime));	
	}
	memset((uint8 *)pTmpResetCycBuf, 0, sizeof(CycleBufferStruct) + RESETINFO_BUFFERSIZE * sizeof(SystemTime));
	if (TRUE == WriteC256(RESETINFOADDR, (uint8 *)pTmpResetCycBuf, sizeof(CycleBufferStruct) + RESETINFO_BUFFERSIZE * sizeof(SystemTime)))
	{
		u8tempBuf[u8tempIndex++] = 0x01;			 //�ɹ�			
	}
	else
	{
		u8tempBuf[u8tempIndex++] = 0x02;			 //ʧ��	
	}
	u8tempBuf[3] = ((u8tempIndex + 2)>>8)&0xFF;//Э��֡�ֽ���
	u8tempBuf[4] = (u8tempIndex + 2)&0xFF;		
	AddCrc16(u8tempBuf,	u8tempIndex);
	U5SendBytes(u8tempBuf, u8tempIndex + 2);				
	free(pTmpResetCycBuf);
	pTmpResetCycBuf = NULL;
}
#endif
