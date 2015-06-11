/****************************************Copyright (c)****************************************************
**                                     BEIJING  WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			DS18B20.c
** Last modified Date:  20110530
** Last Version:		1.0
** Descriptions:		温度操作驱动
**
**--------------------------------------------------------------------------------------------------------
** Created by:			HE NA
** Created date:		20110530
** Version:				1.0
** Descriptions:		温度
**
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
** Version:
** Descriptions:
**
*********************************************************************************************************/
#define	__DS18B20_C
#include "DS18B20.h"

#define  DQ          (1<<25)	  /*DS18B20*/
#define  DQ_STATE    (1<<10)	  /*DS18B20*/

 /*********************************************************************************************************
** Function name:           Delayus
**
** Descriptions:            延时   Delayus(8)延时1us
**
** input parameters:        us : 延时参数
** output parameters:       None
** Returned value:          None
**
** Created by:              
** Created Date:            
**--------------------------------------------------------------------------------------------------------
** Modified by:
** Modified date:
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void Delayus(uint32 p_ulus)
{
    uint32 i = 0;
   
    for (; p_ulus; p_ulus--) 
	{   
	   for (i = 0; i < 3; i++);
    }	
}
/*********************************************************************************************************
** Function name:           Delayms
**
** Descriptions:            延时
**
** input parameters:        ms : 延时参数
** output parameters:       None
** Returned value:          None
**
** Created by:              
** Created Date:            
**--------------------------------------------------------------------------------------------------------
** Modified by:
** Modified date:
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void Delayms(uint32 p_ulms)
{
    uint32 i = 0;
   
    for (; p_ulms; p_ulms--) 
	{   
	    for (i = 0; i < 6500; i++);
    }	
}

//从 1-wire 总线上读取一个字节
uint8 ReadByte(void)
{
    uint8 i;
    uint8 l_u8Value = 0;
    for(i=8;i>0;i--)
    {
        l_u8Value>>=1;
        P2_DIR_SET = DQ;	  //GPIO设置为输出模式
        P3_OUTP_CLR = DQ;     //输出低电平
        Delayus(8);			  //1

        P2_DIR_CLR = DQ;	  //GPIO设置为输入模式
        Delayus(24); 		  //3
       
        if((P3_INP_STATE&DQ_STATE)!=0) 
		{
			l_u8Value|=0x80;
		}    
        Delayus(500); //60   wait for rest of timeslot
        Delayus(8);	  //1
    }
    return(l_u8Value);
}

//向 1-WIRE 总线上写一个字节
void WriteByte(uint8 p_u8Val)
{
    uint8 i;
	uint8 l_u8Temp;
    for (i=8; i>0; i--) // writes byte, one bit at a time
    {
        P2_DIR_SET = DQ;	  //GPIO设置为输出模式
        P3_OUTP_CLR = DQ;  //输出低电平
        Delayus(40);//  5   pull DQ low to start timeslot

		l_u8Temp = p_u8Val&0x01;
		if(l_u8Temp==0)
		{
			P3_OUTP_CLR = DQ;  //输出低电平
			Delayus(480); //  60   hold value for remainder of timeslot
            P3_OUTP_SET = DQ;  //输出高电平
			Delayus(8);	  //1
		}
		if(l_u8Temp==0x01)
		{
			P3_OUTP_SET = DQ;  //输出高电平
			Delayus(480); // 60   hold value for remainder of timeslot
            P3_OUTP_SET = DQ;  //输出高电平
			Delayus(8);	  //1
		}
        p_u8Val=p_u8Val/2;
     }
     Delayus(440);	//55
}
//复位
uint8 DSReset(void)
{
   uint8 l_u8Presence;

   P2_DIR_SET = DQ;	  //GPIO设置为输出模式
   P3_OUTP_CLR = DQ;  //输出低电平
   Delayus(4000); // leave it low for 480us	  500

   P2_DIR_CLR = DQ;	  //GPIO设置为输入模式
   Delayus(300); // wait for l_u8Presence		  35

   l_u8Presence = P3_INP_STATE & DQ_STATE;



   Delayus(3500); // wait for end of timeslot  465
   return(l_u8Presence); // l_u8Presence signal returned
}   // 0=l_u8Presence, 1 = no part

//读取温度
int ReadTemperature(void)
{
   union{
          uint8 c[2];
          uint32 x;
        }temp;
   DSReset();
   WriteByte(0xCC); // Skip ROM
   WriteByte(0x44); // Start Conversion

   P2_DIR_CLR = DQ;	  //GPIO设置为输入模式
   while((P3_INP_STATE&DQ_STATE)==0);
   DSReset();
   WriteByte(0xCC); // Skip ROM
   WriteByte(0xBE); // Read Scratch Pad	从RAM中读取数据
   
   temp.c[0]=ReadByte();
   temp.c[1]=ReadByte();
   temp.x>>=4;		     //ReadByte得到的值低四位为温度的小数点后的值，应用中只用到整数温度，所以将小数点后数据省去

   return temp.x;
}

void DS18B20Init(void)
{
    DSReset();
    WriteByte(0xCC); // Skip ROM
    WriteByte(0x4E); // 向RAM中写数据
    WriteByte(0x00); // 向RAM中写数据的高地址
    WriteByte(0x00); // 向RAM中写数据的低地址
    WriteByte(0x1F);
}

