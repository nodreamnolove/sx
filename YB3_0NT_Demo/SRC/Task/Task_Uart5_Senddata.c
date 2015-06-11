/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			Task_Uart5_Senddata.C
** Last modified Date:  20120718
** Last Version:		1.0
** Descriptions:		���߷�������
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
#include "Task_Uart5_Senddata.h"
#include "WT_Task.h"
#include "RD_data.h"
#include "Common.h"
#include "TDC256.h"

uint32 uart_send_count=0;
uint32 OSRec0A_count = 0;
void FW_fram_save(uint8 *data);
uint8 FW_fram_read(uint8 *data);

void Task_Uart5_Senddata(void *tdata)
{
	uint8 err;
	uint8 l_flag_send01=0;
	uint8 wtemp[2]={0,0};

    tdata =tdata;
	Update_data02(); 
    Update_data08();

	WDTIM_COUNTER	= 1;									/* ι��							*/

	WriteC256(BUF1ADDR,wtemp,2);	//����FIFO

	while(1)
	{
		WDTIM_COUNTER	= 1;									/* ι��							*/
		OSTimeDly(5000);
		WDTIM_COUNTER	= 1;									/* ι��							*/
			
		 //�����豸ʵʱ��ͨ���ݰ���01���ݰ�
		 //SendData();//	���ݷ��ͺ����������Զ����֡ͷ��֡β��CRCУ�顢���ݳ���
		 if(Flag_NetConnect == 1)
		 {
		 	if(MIN%5==0)			 //5����
			{
				Save_data_01_process(); //��

				Update_data02();
			 	SendData( Send_data02,18);//����02����
			 	EVENT_02Rev->OSEventCnt = 0;
			 	OSSemPend(EVENT_02Rev,5000,&err);	// �ȴ�02�����ء� 10s;

				WDTIM_COUNTER	= 1;									/* ι��							*/

			 	if(err == OS_NO_ERR)
			  	{
			  	   g_u8Flag_wireless = 1;
				   Update_data08();
				   SendData(Send_data08,65);   //����08��
				   OSTimeDly(800);
			  	}
				else
				{
					Update_data02();
			 		SendData( Send_data02,18);//����02����
			 		EVENT_02Rev->OSEventCnt = 0;
			 		OSSemPend(EVENT_02Rev,5000,&err);	// �ȴ�02�����ء� 10s;

					WDTIM_COUNTER	= 1;									/* ι��							*/

			 		if(err == OS_NO_ERR)
			  		{
			  		   	g_u8Flag_wireless = 1;
				 		Update_data08();
					   	SendData(Send_data08,65);   //����08��
				   		OSTimeDly(800);
			  		}
					else
					{
						g_u8Flag_wireless = 0;	
					}
				}
				
				if(g_u8Flag_wireless == 0)
				{
					FW_fram_save(Send_data01_temp);		 //�洢��ǰͳ��֡
				}
				else
				{
					SendData(Send_data01_temp,170);
					OSSemPend(EVENT_0ARev,5000,&err);

					WDTIM_COUNTER	= 1;									/* ι��							*/

					if(err == OS_NO_ERR)
					{	
					 	l_flag_send01 = 1;//ȷ�Ϸ��ͳɹ���
					
					}
					else
					{
						SendData(Send_data01_temp,170);
						OSSemPend(EVENT_0ARev,5000,&err);

						WDTIM_COUNTER	= 1;									/* ι��							*/

						if(err == OS_NO_ERR)
						{	
					 		l_flag_send01 = 1;//ȷ�Ϸ��ͳɹ���
						}
						else
						{
						 	l_flag_send01 = 0;//���ξ�δ�ɹ���
						}
					}
					if(l_flag_send01==1)
					{
						while(0==FW_fram_read(Send_data01_temp))	 //ȡ����������	���أ�0����ȷȡ����1�������ѿգ�2����ʱ����Ӧ
						{
							SendData(Send_data01_temp,170);		//����
			  				uart_send_count = uart_send_count +1;
			  				//����01���ݰ���		  //���յ��ظ���0A���ݰ���
			  				OSSemPend(EVENT_0ARev,5000,&err);	//�ȴ�0A�����ء�2500--5s; 10000--20s;

							WDTIM_COUNTER	= 1;									/* ι��							*/

							if(err != OS_NO_ERR)  //û�з��ͳɹ�
							{
								FW_fram_save(Send_data01_temp); //���ȥ
								OSRec0A_count = OSRec0A_count + 1;
								break;
							}
						}	
					 }
					 else
					 {
					 	FW_fram_save(Send_data01_temp);		 //�����˵��Ƿ��ʹ洢��ǰͳ��֡	
					 }
				}
				WDTIM_COUNTER	= 1;									/* ι��							*/	
				OSTimeDly(5000);
				WDTIM_COUNTER	= 1;									/* ι��							*/	
				OSTimeDly(5000);
				WDTIM_COUNTER	= 1;									/* ι��							*/	
				OSTimeDly(5000);
				WDTIM_COUNTER	= 1;									/* ι��							*/	
				OSTimeDly(5000);
				WDTIM_COUNTER	= 1;									/* ι��							*/	
				OSTimeDly(5000);
				WDTIM_COUNTER	= 1;									/* ι��							*/	
				OSTimeDly(5000);
				WDTIM_COUNTER	= 1;									/* ι��							*/	
				OSTimeDly(5000);
				WDTIM_COUNTER	= 1;									/* ι��							*/	
				
			}
			else 
			{
				OSTimeDly(5000);  //10s
			}
		}
	}
}

void FW_fram_save(uint8 *data)
{
	uint8 ReadFWtemp[170];
	uint8 WriteFWtemp[170];
	uint8 head,tail;		  //����ͷ�����ӣ�����β����ӡ�

	ReadC256(BUF1ADDR,ReadFWtemp,2);	  //��ȡ���ƼĴ����е�����
	head=ReadFWtemp[0];
	tail=ReadFWtemp[1];

	if((tail+1)%100==head)	   //������
	{
		return;
	}
	else		              //����δ��
	{
		memcpy(WriteFWtemp,data,170);
		WriteC256(BUF1ADDR+0x200+tail*170,WriteFWtemp,170);							 //д������
		tail=(tail+1)%100;
		WriteC256(BUF1ADDR+0x01,&tail,1);											 //д�����ָ��
	}
}
uint8 FW_fram_read(uint8 *data)
{
	uint8 ReadFWtemp[170];
//	uint8 WriteFWtemp[170];
	uint8 head,tail;		  //����ͷ�����ӣ�����β����ӡ�

	ReadC256(BUF1ADDR,ReadFWtemp,2);	  //��ȡ���ƼĴ����е�����
	head=ReadFWtemp[0];
	tail=ReadFWtemp[1];

	if(tail==head)   //���п�
	{
		return 1;
	}
	else
	{
		ReadC256(BUF1ADDR+0x200+head*170,ReadFWtemp,170);		//���Ӳ���
		head=(head+1)%100;
		WriteC256(BUF1ADDR,&head,1);
		memcpy(data,ReadFWtemp,170);
		return 0;
	}
}
