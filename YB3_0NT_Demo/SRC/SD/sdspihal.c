/****************************************Copyright (c)**************************************************
**                               Guangzhou ZLG-MCU Development Co.,LTD.
**                                      graduate school
**                                 http://www.zlgmcu.com
**
**--------------File Info-------------------------------------------------------------------------------
** File name:			sdspihal.c
** Last modified Date:	2007-07-08
** Last Version:		V3.0
** Descriptions:		SD/MMC卡读写软件包: 硬件抽象层 -- SPI模式
**
**
**------------------------------------------------------------------------------------------------------
** Created by:			Ming Yuan Zheng
** Created date:		2005-1-6
** Version:				V1.0
** Descriptions:		The original version
**
**------------------------------------------------------------------------------------------------------
** Modified by:			Ming Yuan Zheng	
** Modified date:		2007-07-08
** Version:				V3.0
** Descriptions:		1. 将SPI_Clk400k()和SPI_ClkToMax()两函数合并为一个函数SdSpiHal_SetMCIClock()
**                      2. 多数函数名以  SdSpiHal_ 作为开头
**						3. 增加对多卡的支持
**------------------------------------------------------------------------------------------------------
** Modified by: 
** Modified date:
** Version:	
** Descriptions: 
**
********************************************************************************************************/


#include "sdconfig.h"

#if (!SDBUS_MODE_EN)

	/***************************************************************************
		
		读写SD/MMC卡的SPI接口函数: SPI 初始化, SPI时钟设置, 发送/接收字节函数	
	
	***************************************************************************/


/*******************************************************************************************************************
** 函数名称: SD_Power()				
**
** 功能描述: 对卡先下电,再上电	
**
** 输　  入: 无
**							  	
** 输 　 出: 无
**
** 返 回 值: 无
********************************************************************************************************************/
void SD_Power(void)
{
	INT32U i;

	SD_POWER_GPIO();
	SD_POWER_OUT();
	SD_POWER_OFF();								/* 关闭 SD 卡电源  turn off power of sd card */
	
	SPI_SCK_GPIO();	
	SPI_SCK_OUT();
	SPI_SCK_CLR();								/* SCK  引脚置低   set SCK to zero */
	
	SPI_MISO_GPIO();
	SPI_MISO_OUT();
	SPI_MISO_CLR();								/* MISO 引脚置低   set MISO to zero */
	
	SPI_MOSI_GPIO();		
	SPI_MOSI_OUT();
	SPI_MOSI_CLR();								/* MOSI 引脚置低   set MOSI to zero */
	
	SPI_CS_GPIO();								
   	SPI_CS_OUT();								
	SPI_CS_CLR();								/* CS 引脚置低	   set CS to zero */
		
	for(i = 0; i < 0x9000; i++);				/* 关闭电源延时    delay after turn off power of sd card */
	SD_POWER_ON();								/* 打开 SD 卡电源  turn on power of sd card */
}


/*******************************************************************************************************************
** 函数名称: SdSpiHal_Initialize()				
**
** 功能描述: 初始化访问卡的硬件条件		
**
** 输　  入: sd_struct *sds:  卡信息结构体, 该函数仅可使用sds->card_posnum成员用于区分卡座编号
**							  	
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
**
** 主要工作: 1. 先给卡下电, 再给卡上电;              2. 设置访问卡的速度小于或等于 400KHz;
** 
**           3. 初始化相关硬件寄存器为适当的状态     4. 初始化SPI控制器
********************************************************************************************************************/
INT8U SdSpiHal_Initialize(sd_struct *sds)
{ 
	SD_Power();									/* 对卡先下电,再上电 */
	
	SPI_INIT();									/* 初始化SPI接口     */	
	
	SD_INSERT_GPIO();
	SD_INSERT_IN();								/* 检测卡完全插入口为输入口 */
	
	SD_WP_GPIO();								
	SD_WP_IN();									/* 写保护检测口为输入口 */	
		   
   	SPI_CS_SET();								/* CS置高 */

 	SdSpiHal_SetMCIClock(sds, SD_RATE_SLOW); 	/* 设置SPI频率小于等于400kHZ */
   
 	S0SPCR = (0x00 << 4) + (0x01 << 5);		    /* 设置SPI接口模式, MSTR = 1,CPOL = 1,CPHA = 0,LSBF=0 */

	return SD_NO_ERR;
}												


/*******************************************************************************************************************
** 函数名称: SdSpiHal_SetMCIClock()				
**
** 功能描述: 设置读写SD/MMC卡时钟		
**
** 输　  入: sd_struct *sds  :  卡信息结构体, 该函数仅可使用sds->card_posnum成员用于区分卡座编号
**           INT32U ClockRate:	要求的时钟速度.  取值SD_RATE_SLOW   时, 设置访问卡速度小于 400KHz		
**			  	                                 取值SD_RATE_NORMAL 时, 设置访问卡速度为卡的最大速度或主控器的最大速度
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
**
** 注    意: 本函数有责任在ClockRate取不同的值时, 计算出相应的 sds->clkval 值, 必须大于0. 
**
********************************************************************************************************************/
void SdSpiHal_SetMCIClock(sd_struct *sds, INT32U ClockRate)
{
   	if (ClockRate == SD_RATE_SLOW)
	{
		S0SPCCR = 128;    							/* 设置SPI时钟分频值为128 */
		sds->host_clkval = Fpclk / 128;
	}
	else if (ClockRate == SD_RATE_NORMAL) 
	{
		S0SPCCR = 8;								/* 设置SPI时钟分频值为8 */
		sds->host_clkval = Fpclk / 8;    				
	}
}


/*******************************************************************************************************************
** 函数名称: SdSpiHal_SendByte()				
**
** 功能描述: 通过SPI控制器发送一个字节到卡		
**
** 输　  入: sd_struct *sds  : 卡信息结构体, 该函数仅可使用sds->card_posnum成员用于区分卡座编号
**			 INT8U byte      : 发送的字节		
**           		
** 输 　 出: 无
**
** 返 回 值: 无
********************************************************************************************************************/
void SdSpiHal_SendByte(sd_struct *sds, INT8U byte)
{
	INT8U temp;

	S0SPDR = byte;							/* 发送数据放入SPI数据寄存器 */
   
	while(0 == (S0SPSR & 0x80));				/* 等待SPIF置位，即等待数据发送完毕 */
												/* wait for SPIF being set, that is, wait for finishing of data being send */
 	temp = S0SPDR;
}


/*******************************************************************************************************************
** 函数名称: SdSpiHal_RecByte()				
**
** 功能描述: 通过SPI控制器从卡接收一个字节		
**
** 输　  入: sd_struct *sds  : 卡信息结构体, 该函数仅可使用sds->card_posnum成员用于区分卡座编号
**           		
** 输 　 出: 无
**
** 返 回 值: 收到的字节
********************************************************************************************************************/
INT8U SdSpiHal_RecByte(sd_struct *sds)
{
	S0SPDR = 0xFF;
   
 	while(0 == (S0SPSR & 0x80));				/* 等待SPIF置位，即等待收到数据 */
												/* wait for SPIF being set, that is, wait for being received data */
	return(S0SPDR); 							/* 读取收到的字节 read the byte received */
}


/*******************************************************************************************************************
** 函数名称: SdSpiHal_CSAssert()				
**
** 功能描述: 通过SPI的CS引脚选择卡		
**
** 输　  入: sd_struct *sds  : 卡信息结构体, 该函数仅可使用sds->card_posnum成员用于区分卡座编号
**           		
** 输 　 出: 无
**
** 返 回 值: 无
********************************************************************************************************************/
void SdSpiHal_CSAssert(sd_struct *sds)
{
	SPI_CS_CLR();			   					/* 片选SPI从机  select the SPI slave */  
}


/*******************************************************************************************************************
** 函数名称: SdSpiHal_CSDeassert()				
**
** 功能描述: 通过SPI的CS引脚不选择卡		
**
** 输　  入: sd_struct *sds  : 卡信息结构体, 该函数仅可使用sds->card_posnum成员用于区分卡座编号
**           		
** 输 　 出: 无
**
** 返 回 值: 无
********************************************************************************************************************/
void SdSpiHal_CSDeassert(sd_struct *sds)
{
	SPI_CS_SET();			    				/* 不片选SPI从机  not select the SPI slave */
}


/*******************************************************************************************************************
** 函数名称: SdHal_CheckCard()				
**
** 功能描述: 检测卡是否完全插入卡座中
**
** 输　  入: sd_struct *sds:  卡信息结构体, 该函数仅可使用sds->card_posnum成员用于区分卡座编号
**	         
** 输 　 出: 无
**
** 返 回 值: 1: 卡完全插入	   0: 卡没有完全插入
********************************************************************************************************************/
INT8U SdHal_CheckCard(sd_struct *sds)
{
	if (SD_INSERT_STATUS() != 0)
		return 0;								/* 未完全插入 not insert entirely */
	else
		return 1;								/* 完全插入 insert entirely */
}


/*******************************************************************************************************************
** 函数名称: SdHal_CheckCardWP()				
**
** 功能描述: 检测卡写保护
**
** 输　  入: sd_struct *sds:  卡信息结构体, 该函数仅可使用sds->card_posnum成员用于区分卡座编号
**	         
** 输 　 出: 无
**
** 返 回 值: 1: 卡已写保护	  0: 卡未写保护
********************************************************************************************************************/
INT8U SdHal_CheckCardWP(sd_struct *sds)
{
	if (SD_WP_STATUS() != 0)
		return 1;								/* 卡写保护 */
	else
		return 0;								/* 卡未写保护 */
}

#endif



