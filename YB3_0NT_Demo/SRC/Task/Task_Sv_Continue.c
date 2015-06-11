#include "Task_Sv_Continue.h"
#include "config.h"

//_Cycle_Que_Continue Cycle_Que_Continue;
extern uint8 stat_tocontinue_flag;
extern _Cycle_Que_Continue cycle_que_continue;

OS_EVENT *continue_flag;	 //续传的资源标志

uint8 b_continue_flag = 0;
void Task_Sv_Continue(void *tdata)
{
	uint8 err;
	uint8 Sv_buf[60] = {0},test[3];
	uint32 read_add,index;
	tdata = tdata;

	WDTIM_COUNTER	= 1;									/* 喂狗							*/

	while(1)
	{

		OSTimeDly(500);  
	//	if(stat_tocontinue_flag == 1)		  //续传标志位为1
		if(1)
		{
			stat_tocontinue_flag = 0;
			 
			while(0 == SD_SvDataAdd_Read(Sv_buf,&read_add,&index) && b_continue_flag == 1)
			{  

				UART1_SendBuf_full(Sv_buf,57);
//				test[0] = index/100 + '0';
//				 test[1] = index/10%10+ '0';
//				 test[2] = index%10+ '0';
//				 UART1_SendBuf(test,3);
//				 UART1_SendBuf("= ",2);
//				 test[0] = Sv_buf[36]/100+ '0';
//				 test[1] = Sv_buf[36]/10%10+ '0';
//				 test[2] = Sv_buf[36]%10+ '0';
//				 UART1_SendBuf(test,3);
//				 UART1_SendBuf("\r\t",2);
	//			 OSTimeDly(2500);
				EVENT_11Rev->OSEventCnt = 0;       
  				OSSemPend(EVENT_11Rev,5000,&err);	//等待11包返回帧


				if(err != OS_NO_ERR)  //没有发送成功
				{					
					g_u8Flag_wireless = 0;				
					Set_Que_Cycle_Continue(read_add,index);	//续传地址回写入续传队列
					break;
				}
				else
				{
					g_u8Flag_wireless = 1;	
				}
			}	 
		}
		else
		{
			OSTimeDly(5);
		}	   
	}
}


uint32 Get_Que_Cycle(uint8 type)
{
	uint8 ReadFWtemp[8];
	uint32 data=0;

	Read256_full(BUF1ADDR+0x16+type*8,ReadFWtemp,8);

   	if((ReadFWtemp[0] = 0xAA) && (ReadFWtemp[1] = 0xAA) && (ReadFWtemp[2] = 0xBB) && (ReadFWtemp[3] = 0xBB)) //block头对了
		data = ReadFWtemp[4]+(ReadFWtemp[5]<<8) + (ReadFWtemp[6]<<16) + (ReadFWtemp[7]<<24);

	return data;	
}
uint8 Set_Que_Cycle(uint32 data,uint8 type)
{
	uint8 WriteFWtemp[8];

	WriteFWtemp[0] = 0xAA;
	WriteFWtemp[1] = 0xAA;
	WriteFWtemp[2] = 0xBB;
	WriteFWtemp[3] = 0xBB;

	WriteFWtemp[4] = (data & 0xFF);
	WriteFWtemp[5] = ((data >> 8) & 0xFF);
	WriteFWtemp[6] = ((data >> 16) & 0xFF);
	WriteFWtemp[7] = ((data >> 24) & 0xFF);

	Write256_full(BUF1ADDR+0x16+type*8,WriteFWtemp,8);

	return 0;
}
uint8  Get_Que_Cycle_Continue(uint32*p_add_conti,uint32 *index)
{
	uint8 ret,err;
	uint8 ReadSDtemp[512] = {0};
	uint16 if_day = 0;
	uint32 x,y,head=0,tail=0,cnt=0;
	uint32 l_add_conti,l_index;

	y = gv_index - 1;
 	head = Get_Que_Cycle(0);
	tail = Get_Que_Cycle(1);
	cnt = (tail - head + SD_CONTINUE_NUM)%SD_CONTINUE_NUM;
	if(cnt == 0)  	//队列空
		return 1;

	full_read_sd(head+SD_CONTINUE_ADD_START,(uint8*)ReadSDtemp);
	if((ReadSDtemp[0] == SD_BLOCK_HEAD[0]) && (ReadSDtemp[1] == SD_BLOCK_HEAD[1]) && 
				(ReadSDtemp[2] == SD_BLOCK_HEAD[2]) && (ReadSDtemp[3] == SD_BLOCK_HEAD[3]))
	{
		*p_add_conti = (ReadSDtemp[4] & 0xFF) + ((ReadSDtemp[5] & 0xFF) << 8) + ((ReadSDtemp[6] & 0xFF) << 16) + ((ReadSDtemp[7] & 0xFF) << 24);
		l_add_conti = *p_add_conti;
		*index = (ReadSDtemp[8] & 0xFF) + ((ReadSDtemp[9] & 0xFF) << 8) + ((ReadSDtemp[10] & 0xFF) << 16) ;
		l_index = *index;
		if_day =  (ReadSDtemp[11] & 0xFF) + ((ReadSDtemp[12] & 0xFF) << 8);

	}
	else
	{
		return 1;
	}
	if(if_day == ((YEAR - 2013) * 372 + (MONTH - 1) * 31 + DAY))	 //同一天
	{		
		x = *index - 1;
		
		if((y/24 - x/24) < 1)	//数据未存储在SD中
		{
			/////////////////////////////////////////
//			Set_Que_Cycle_Continue(l_add_conti,l_index);
//			head = (head + 1)%SD_CONTINUE_NUM;
//			Set_Que_Cycle(head,0);
			/////////////////////////////////////////
			return 1;	
		}
	}
	head = (head + 1)%SD_CONTINUE_NUM;
	Set_Que_Cycle(head,0);

 	return 0;
}

uint8  Set_Que_Cycle_Continue(uint32 save_add,uint32 index)
{
	uint8 ret,err;
	uint8 WriteFWtemp[12];
	uint8 WriteSDtemp[512] = {0};
	uint8 i;
	uint32 tail = 0;

	for(i=0;i<4;i++)
		WriteSDtemp[i] = SD_BLOCK_HEAD[i];

	WriteSDtemp[4] = save_add&0xFF; 	//存储在SD卡中的扇区地址
	WriteSDtemp[5] = (save_add>>8)&0xFF;
	WriteSDtemp[6] = (save_add>>16)&0xFF;	
	WriteSDtemp[7] = (save_add>>24)&0xFF;	
	WriteSDtemp[8] = index&0xFF;		//数据序号
	WriteSDtemp[9] = (index>>8)&0xFF;
	WriteSDtemp[10] = (index>>16)&0xFF;
	WriteSDtemp[11] = IF_SAME_DAY&0xff;	   //天数信息，用于在续传时，判别续传的数据是否是当天数据
	WriteSDtemp[12]	= (IF_SAME_DAY>>8)&0xff;

	OSSemPend(continue_flag,5000,&err);
	if(err==0)				
	{
		tail = Get_Que_Cycle(1);
		full_write_sd(tail+SD_CONTINUE_ADD_START, (uint8*)&WriteSDtemp);	
	    tail = (tail + 1)%SD_CONTINUE_NUM;
		Set_Que_Cycle(tail,1);

		OSSemPost(continue_flag);
	}
	else
	{
		return 0;
	}

	return 0;
	
}


uint8 SD_SvDataAdd_Read(uint8 *data,uint32*p_read_add,uint32 *index)
{
	uint8 ReadFWtemp[512];
	uint8 WriteFWtemp[8];
	uint8 sv_sd_frame[21]={0};
	uint16 crc16;
	//uint32 head_conitnue,tail_continue;	
//	_Cycle_Que_Continue cycle_que_conti;	 
	//uint32 read_add;
	int i;


	if(data == NULL)
	{
		return 2;
	}

	if(1 == Get_Que_Cycle_Continue(p_read_add,index)) 	//队列空
		return 1;


	if (full_read_sd(*p_read_add,ReadFWtemp))
	{
		BeepON();
		OSTimeDly(50);	
		BeepOFF();
		return 2;
	}
															
	if( (ReadFWtemp[0]==SD_BLOCK_HEAD[0])&&(ReadFWtemp[1]==SD_BLOCK_HEAD[1])&&
		(ReadFWtemp[2]==SD_BLOCK_HEAD[2])&&(ReadFWtemp[3]==SD_BLOCK_HEAD[3]) )   //block头对了
	{	
		memcpy(&sv_sd_frame[0],&ReadFWtemp[4+(((*index)-1)%24)*21],21);
		//续传时如果车长宽高超过限制，也不续传
		if(((sv_sd_frame[15] & 0xFF) + (sv_sd_frame[16] & 0xFF) << 8) > limit_length && ((sv_sd_frame[17] & 0xFF) + (sv_sd_frame[18] & 0xFF) << 8) > limit_width &&
				((sv_sd_frame[19] & 0xFF) + (sv_sd_frame[20] & 0xFF) << 8) > limit_height)
			b_continue_flag = 1;
		else
		{
			b_continue_flag = 0;
			return 3;			
		}
		data[0] = 0xAA;
		data[1] = 0xAA;
	
		data[2] = 0x31;			//长度 小端
		data[3] = 0x00;
	
		data[4] = 0x11;
		for(i=0;i<16;i++)
		{
			data[5+i] = RDid[i];
		}
		data[21]=0;
		data[22]=sv_sd_frame[0];
		data[23]=sv_sd_frame[1];
		data[24]=sv_sd_frame[2];
		data[25]=sv_sd_frame[3];
		data[26]=sv_sd_frame[4];
		data[27]=sv_sd_frame[5];
		data[28]=sv_sd_frame[6];
		data[29]=0;
		data[30]=0;
		data[31]=sv_sd_frame[9];
		data[32]=sv_sd_frame[10];
		data[33]=sv_sd_frame[11];
		data[34] = sv_sd_frame[12]; 
		data[35] = sv_sd_frame[13];
		data[36] = sv_sd_frame[14];
	
		//车长
		data[37] =  sv_sd_frame[15];
		data[38] =  sv_sd_frame[16];
		//车宽
		data[39] =  sv_sd_frame[17];
		data[40] =  sv_sd_frame[18];
		//车高
		data[41] =  sv_sd_frame[19];
		data[42] =  sv_sd_frame[20];
	
		data[43] = 0;
		data[44] = 0;
		data[45] = 0;
		data[46] = 0;
		data[47] = 0;
		data[48] = 0;
	
		data[49] = 0;
		data[50] = 0;
	
		data[51] = 0;
		data[52] = 0;
	
		crc16 = crc_16(data,53);
		data[53] = (crc16 & 0xFF);
		data[54] = ((crc16 >> 8) & 0xFF);
	
		data[55] = 0xEE;
		data[56] = 0xEE;
		return 0;
	}
	else
	{
		return 2;
	}
	//}
}

