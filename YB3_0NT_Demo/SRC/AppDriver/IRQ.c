/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			IRQ.c
** Last modified Date:  2011-03-31
** Last Version:		1.0
** Descriptions:		中断初始化和中断处理函数
**
**--------------------------------------------------------------------------------------------------------
** Created by:			ZHANG Ye
** Created date:		2011-03-31
** Version:				1.0
** Descriptions:		Timer0,UART5,WDT
**
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
** Version:
** Descriptions:
**
*********************************************************************************************************/
#define	IRQ_C
#include "IRQ.h"
#include "sdcommon.h"
#include "sdconfig.h"
#include "Uart1.h"

#define I2C_CLK_100K 1
#define I2C_CLK_400K 4

#define		SETUPALIAS				g_sspSetup			//设置参数结构

/*********************************************************************************************************
** 函数名称:  InitAllIRQ
** 函数功能:  对所有需要的中断进行初始化
** 入口参数:  无
** 出口参数:  无
** 函数说明:
*********************************************************************************************************/
void InitAllIRQ(void)
{
/********************************
* 在此处添加中断初始化调用
*********************************/
//	uint8 status;
	Time0Init();	//定时器 

	I2C1Init(I2C_CLK_100K);	//I2C,默认速度100
	I2C2Init(I2C_CLK_100K);	//I2C,默认速度100
	          
#ifndef	__UCOS				
	PCF8563Init();			//UCOS下在时间任务中初始化
//	KeyboardInit();			//键盘任务中初始化
#endif										   
	StorageInit();
	LCDInit();
	DS18B20Init();
}

/*********************************************************************************************************
** 函数名称:  InitAllSP
** 函数功能:  对所有串口进行初始化
** 入口参数:  p_u32U2,p_u32U3,p_u32U5	串口2、3、5波特率
** 出口参数:  无
** 函数说明:  初始化网口
*********************************************************************************************************/
void InitAllSP(uint32 p_u32U2, uint32 p_u32U3, uint32 p_u32U5)
{								 
	UART2Init(p_u32U2);	//串口2
	UART3Init(p_u32U3);	//串口3
	UART5Init(p_u32U5);	//串口5
	UART1_Init();	  	//串口1波特率为57600

	memcpy(&g_sniLocal, (uint8 *)&SETUPALIAS.u32LocalIPAddress, sizeof(NetInfo)-2);

	InitializeW5100(&g_sniLocal);//
}
