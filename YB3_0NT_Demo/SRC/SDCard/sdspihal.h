/****************************************Copyright (c)**************************************************
**                               Guangzhou ZLG-MCU Development Co.,LTD.
**                                      graduate school
**                                 http://www.zlgmcu.com
**
**--------------File Info-------------------------------------------------------------------------------
** File name:			sdspihal.c
** Last modified Date:	2007-07-12
** Last Version:		V3.0
** Descriptions:		SD/MMC卡读写软件包: 硬件抽象层头文件 -- SPI模式
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


#ifndef __SDSPIHAL_H__
#define __SDSPIHAL_H__


/*
****************************
   SPI 总线模式
****************************
*/

#if (!SDBUS_MODE_EN)

/***********************************************************
LPC2364/6 没有MCI控制器, 所以没办法以SD模式读写SD/MMC卡, 但可以以SPI

模式读写SD/MMC卡. 使用的引脚如下:

 LPC2364/6            卡座引脚    SmartARM2300开发板JP1

P0.15(SCK0)  ----->  CLK/SCK        CLK

P0.17(MISO0) ----->  DAT0/DO        D0      

P0.18(MOSI0) ----->  CMD/DI         CMD

P2.13(用作GPIO) -->  DAT3/CS        D3 

P0.21        ----->  (控制卡供电)   PWR

P2.8         ----->  CARD_INSERT    CD

P3.25        ----->  CARD_WP        WP               

如使用SmartARM2300, 硬件连接如下;

1. 用线连接: P0.15 --> CLK(JP1), P0.17 --> D0(JP1), P0.18 --> CMD(JP1)

2. 短接跳线: D3(JP1),  PWR(JP1),  WP(JP1),  CD(JP1)  

************************************************************/

/*************************************
   LPC2364/6 硬件配置(使用SPI总线) 
**************************************/

/* SCK引脚 */
#define  SPI_SCK				(0x01 << 15)	                // P0.15					
#define  SPI_SCK_GPIO()			PINSEL0 &= ~(0x03U << 30)		/* 设置 SCK 口为GPIO口 */
#define  SPI_SCK_OUT()			IO0DIR |= SPI_SCK				/* 设置 SCK 口为输出口 */
#define	 SPI_SCK_CLR()			IO0CLR = SPI_SCK				/* 置 SCK 为低电平 */	

/* MISO 引脚 */	
#define  SPI_MISO				(0x01 << 17)                    // P0.17						    
#define  SPI_MISO_GPIO()		PINSEL1 &= ~(0x03 << 2)		    /* 设置 MISO 口为GPIO口 */
#define  SPI_MISO_OUT()			IO0DIR |= SPI_MISO				/* 设置 MISO 口为输出口 */
#define	 SPI_MISO_CLR()			IO0CLR = SPI_MISO				/* 置 MISO 为低电平 */

/* MOSI 引脚 */	
#define  SPI_MOSI				(0x01 << 18)                    // P0.18                     
#define  SPI_MOSI_GPIO()		PINSEL1 &= ~(0x03 << 4)	    	/* 设置 MOSI 口为GPIO口 */
#define  SPI_MOSI_OUT()			IO0DIR |= SPI_MOSI				/* 设置 MOSI 口为输出口 */
#define	 SPI_MOSI_CLR()			IO0CLR = SPI_MOSI				/* 置 MISO 为低电平 */

/* P2.13 为 CS 引脚 */
#define  SPI_CS_GPIO()          PINSEL4 &= ~(0x03 << 26);
#define  SPI_CS_OUT()           FIO2DIR |= (0x01 << 13);        /* 设置 CS 口为输出口 */
#define	 SPI_CS_SET()	        FIO2SET |= (0x01 << 13);	    /* 置 CS 为高电平 */
#define  SPI_CS_CLR()           FIO2CLR |= (0x01 << 13);        /* 置 CS 为低电平 */


/* 初始化 IO 口为SPI接口 */
#define  SPI_INIT()				{PINSEL0 &= ~(0x03U << 30); PINSEL1 &= ~((0x03 << 2) + (0x03 << 4));\
                                 PINSEL0 |= (0x03U << 30);  PINSEL1 |= (0x03 << 2) + (0x03 << 4);}  

/* P0.21 电源控制引脚 */
#define  SD_POWER				(0x01 << 21)	
#define  SD_POWER_GPIO()		PINSEL1 &= ~(0x03 << 10)		/* 设置 POWER 口为GPIO口 */
#define  SD_POWER_OUT()			IO0DIR |= SD_POWER				/* 设置 POWER 口为输出口 */
#define  SD_POWER_OFF()			IO0SET |= SD_POWER				/* 置 POWER 为高电平 */
#define  SD_POWER_ON()			IO0CLR |= SD_POWER				/* 置 POWER 为低电平 */


/* P2.8 卡完全插入卡座检测引脚 */
#define  SD_INSERT				(0x01 << 8)		
#define  SD_INSERT_GPIO()		PINSEL4 &= ~(0x03 << 16)		/* 设置 INSERT 口为GPIO口 */	
#define  SD_INSERT_IN()			FIO2DIR &= ~SD_INSERT			/* 设置 INSERT 口为输入口 */	
#define  SD_INSERT_STATUS()  	(FIO2PIN & SD_INSERT)			/* 读取 INSERT 口的状态 */

/* P3.25 卡写保护检测引脚 */
#define  SD_WP					(0x01 << 25)		
#define  SD_WP_GPIO()			PINSEL7 &= ~(0x03 << 18);		/* 设置 WP 口为GPIO口 */	
#define  SD_WP_IN()				FIO3DIR &= ~SD_WP				/* 设置 WP 口为输入口 */	
#define  SD_WP_STATUS()  		(FIO3DIR & SD_WP)				/* 读取 WP 口的状态   */

																
// 检测卡是否完全插入 
extern INT8U SdHal_CheckCard(sd_struct *sds);

// 检测卡是否写保护
extern INT8U SdHal_CheckCardWP(sd_struct *sds);

// 初始化访问卡的硬件初始化
extern INT8U SdSpiHal_Initialize(sd_struct *sds);

// 设置读写SD/MMC卡时钟
extern void SdSpiHal_SetMCIClock(sd_struct *sds, INT32U ClockRate);

// 通过SPI控制器发送一个字节到卡
extern void SdSpiHal_SendByte(sd_struct *sds, INT8U byte);

// 通过SPI控制器从卡接收一个字节
extern INT8U SdSpiHal_RecByte(sd_struct *sds);

// 通过SPI的CS引脚选择卡
extern void SdSpiHal_CSAssert(sd_struct *sds);

// 通过SPI的CS引脚不选择卡	
extern void SdSpiHal_CSDeassert(sd_struct *sds);

#endif


#endif