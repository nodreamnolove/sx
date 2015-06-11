/****************************************Copyright (c)**************************************************
**                               Guangzhou ZLG-MCU Development Co.,LTD.
**                                      graduate school
**                                 http://www.zlgmcu.com
**
**--------------File Info-------------------------------------------------------------------------------
** File name:			sdcmd.h
** Last modified Date:	2005-3-11
** Last Version:		V2.0
** Descriptions:		SD/MMC 读写模块: 物理层 ---- SD/MMC 卡SPI模式支持的命令(头文件) 
**						Soft Packet of SD/MMC Card : commands that SD/MMC card supported 
**						in spi mode(header file)
**------------------------------------------------------------------------------------------------------
** Created by:			Ming Yuan Zheng
** Created date:		2005-1-6
** Version:				V1.0
** Descriptions:		SD/MMC 读写模块: 物理层 ---- SD卡SPI模式支持的命令 
**
**------------------------------------------------------------------------------------------------------
** Modified by:			Ming Yuan Zheng
** Modified date:		2005-3-11
** Version:				V2.0
** Descriptions:		增加了对MMC卡的支持,并使用本模块既可以运行于前后台系统,也可运行于uCOS-II中.
**
**------------------------------------------------------------------------------------------------------
** Modified by:         Ming Yuan Zheng
** Modified date:       2007-5-19
** Version:	            V3.0
** Descriptions:        1. 对原来SPI模式下的函数名进行修改, 改为以 SpiCmd_ 为开头, 数据流函数以 SdSpi_ 为开头
**
**                      2. 增加了对SD总线模式的支持, 函数名以 SdCmd_ 为开头
**
********************************************************************************************************/

#ifndef __SDCMD_H__
#define __SDCMD_H__
#include "sddriver.h"
#include "sdcommon.h"


/*
 *******************************************************
	
		本文件主要是 SD/MMC 卡的相关命令与响应宏定义
		
		包括:  SD总线模式和SPI模式
	
 *******************************************************
*/


/*********************************************************************************
                    
                    SD/MMC 卡 SD总线模式命令及响应

**********************************************************************************/

#if    SDBUS_MODE_EN


/* 命令响应定义 define command's response */
#define R0 		0								/* 用于SD模式 */
#define R1 		1
#define R1B 	2
#define R2 		3
#define R3	 	4
#define R6 		7
#define R7      8 
/*
*******************************

     SD/MMC卡 命令集

*******************************
*/

/******************************** 基本命令集 Basic command set **************************/
/* 复位SD 卡 Reset cards to idle state */
#define CMD0 0
#define CMD0_R R0


/* 读所有SD总线上的SD卡的CID寄存器 asks any device to send their CID numbers on the CMD Line*/
#define  CMD1   	1
#define	 CMD1_R	 	R3

/* 读所有SD总线上的SD卡的CID寄存器 asks any device to send their CID numbers on the CMD Line*/
#define CMD2 2
#define CMD2_R R2

/* 要求设备发送一个新的相对地址 asks the device to publish a new relative adderss */
#define CMD3 3
#define CMD3_R R6

/* 选择高速模式*/
#define	CMD6 6
#define CMD6_R R1

/* 选择/断开选择卡 select/deselect card */
#define	CMD7 7
#define CMD7_R R1

/* type currently unkown, for sdhc use */
#define CMD8 8
//#define CMD8_R 	R7  	//4.17-fan-修改前
#define CMD8_R 	R1 	        //4.17-fan-修改后

/* 读CSD寄存器 Card sends the CSD */
#define CMD9 9
#define CMD9_R R2

/* 读CID寄存器 Card sends CID */
#define CMD10 10
#define CMD10_R R2

/* 停止读多块时的数据传输 Stop a multiple block (stream) read/write operation */
#define CMD12 12
#define CMD12_R R1B

/* 读 Card_Status 寄存器 Get the addressed card's status register */
#define CMD13 13
#define CMD13_R R1

/* 设置设备处于停止状态 set the device to inactive state */
#define CMD15 15
#define CMD15_R R0

/* 设置块的长度 Set the block length */
#define CMD16 16
#define CMD16_R R1

/* 读单块 Read a single block */
#define CMD17 17
#define CMD17_R R1

/* 读多块,直至主机发送CMD12为止 Read multiple blocks until a CMD12 */
#define CMD18 18
#define CMD18_R R1

/***************************** 块写命令集 Block write commands *************************/
/* 写单块 Write a block of the size selected with CMD16 */
#define CMD24 24
#define CMD24_R R1

/* 写多块 Multiple block write until a CMD12 */
#define CMD25 25
#define CMD25_R R1

/* 写CSD寄存器 Program the programmable bits of the CSD */
#define CMD27 27
#define CMD27_R R1

/***************************** 写保护 Write protection *****************************/
/* Set the write protection bit of the addressed group */
#define CMD28 28
#define CMD28_R R1B

/* Clear the write protection bit of the addressed group */
#define CMD29 29
#define CMD29_R R1B

/* Ask the card for the status of the write protection bits */
#define CMD30 30
#define CMD30_R R1

/***************************** 擦除命令 Erase commands *******************************/
/* 设置擦除块的起始地址 Set the address of the first write block to be erased */
#define CMD32 32
#define CMD32_R R1

/* 设置擦除块的终止地址 Set the address of the last write block to be erased */
#define CMD33 33
#define CMD33_R R1


/* 设置擦除块的起始地址(仅适用于MMC卡) */
#define CMD35 35
#define CMD35_R R1

/* 设置擦除块的终止地址(仅适用于MMC卡) */
#define CMD36 36
#define CMD36_R R1


/* 擦除所选择的块 Erase the selected write blocks */
#define CMD38 38
#define CMD38_R R1B

/***************************** 锁卡命令 Lock Card commands ***************************/
/* 设置/复位密码或上锁/解锁卡 Set/reset the password or lock/unlock the card */
#define CMD42 42
#define CMD42_R	R1B
/* Commands from 42 to 54, not defined here */

/***************************** 应用命令 Application-specific commands ****************/
/* 禁止下一个命令为应用命令  Flag that the next command is application-specific */
#define CMD55 55
#define CMD55_R R1

/* 应用命令的通用I/O  General purpose I/O for application-specific commands */
#define CMD56 56
#define CMD56_R R1

/* 读OCR寄存器,只用于SPI模式  Read the OCR (SPI mode only) */
#define CMD58 58
#define CMD58_R R3

/* 使能或禁止 CRC Turn CRC on or off */    //SD卡没有此功能
#define CMD59 59						   //SD卡没有此功能
#define CMD59_R R1						   //SD卡没有此功能

/***************************** 应用命令 Application-specific commands ***************/
#define ACMD6 6
#define ACMD6_R R1

/* 获取 SD Status寄存器 Get the SD card's status */
#define ACMD13 13
#define ACMD13_R R1

/* 得到已写入卡中的块的个数 Get the number of written write blocks (Minus errors ) */
#define ACMD22 22
#define ACMD22_R R1

/* 在写之前,设置预先擦除的块的个数 Set the number of write blocks to be pre-erased before writing */
#define ACMD23 23
#define ACMD23_R R1

/* 读取OCR寄存器 Get the card's OCR (SD mode) */
#define ACMD41 41
#define ACMD41_R R3

/* 连接/断开CD/DATA[3]引脚上的上拉电阻 Connect or disconnect the 50kOhm internal pull-up on CD/DAT[3] */
#define ACMD42 42
#define ACMD42_R R1

/* 读取SCR寄存器 Get the SD configuration register */
#define ACMD51 51
#define ACMD51_R R1



/*
*  SD 总线模式下, Card_Status寄存器相关状态位
*/
#define CARD_STATUS_ACMD_ENABLE		(1 << 5)			/* 仅用于SD卡 */
#define CARD_STATUS_ERASE_RESET		(1 << 13)

/* 卡各种状态掩码 */
#define CARD_STATUS_RDY_DATA		(0x01 << 8)

#define CARD_STATUS_CURRENT_STATE	(0x0F << 9)
#define CARD_STATUS_IDLE            (0x00 << 9)         /* 空闲 */
#define CARD_STATUS_READY          	(0x01 << 9)         /* 接收 */
#define CARD_STATUS_IDENT          	(0x02 << 9)         /* 识别 */
#define CARD_STATUS_STBY          	(0x03 << 9)         /* 待机 */
#define CARD_STATUS_TRAN         	(0x04 << 9)         /* 数据发送 */
#define CARD_STATUS_DATA          	(0x05 << 9)         /* 数据 */
#define CARD_STATUS_REC         	(0x06 << 9)         /* 数据接收 */
#define CARD_STATUS_PRG         	(0x07 << 9)         /* 卡编程 */
#define CARD_STATUS_DIS          	(0x08 << 9)         /* 卡断开 */


/* sdcmd.c文件导出的函数 */
extern INT8U SdCmd_Go_Idle_State(sd_struct *sds);
extern INT8U SD_CheckSDVersion(sd_struct *sds);
extern INT8U SdCmd_Send_OP_Cond(sd_struct *sds);
extern INT8U SdCmd_Send_ACMD(sd_struct *sds);

extern INT8U SdCmd_SD_App_OP_Cond(sd_struct *sds);
extern INT8U SdCmd_All_Send_CID(sd_struct *sds, INT8U cidlen, INT8U *cidbuf);

extern INT8U SdCmd_Set_Relative_Addr(sd_struct *sds);
extern INT8U SdCmd_Send_CSD(sd_struct *sds, INT8U csdlen, INT8U *csdbuf);

extern INT8U SdCmd_Stop_Transmission(sd_struct *sds);
extern INT8U SdCmd_Send_Status(sd_struct *sds, INT32U *status);
extern INT8U SdCmd_Select_Card(sd_struct *sds);

extern INT8U SdCmd_Set_Bus_Width(sd_struct *sds, INT8U bWideBus);
extern INT8U SdCmd_Set_BlockLen(sd_struct *sds, INT32U blockLen);
extern INT8U SdCmd_Read_Single_Block(sd_struct *sds, INT32U blockadddr);
extern INT8U SdCmd_Read_Multiple_Block(sd_struct *sds, INT32U blockadddr);

extern INT8U SdCmd_Deselect_Card(sd_struct *sds, INT32U RCA);
extern INT8U SdCmd_Write_Single_Block(sd_struct *sds, INT32U blockadddr);
extern INT8U SdCmd_Write_Multiple_Block(sd_struct *sds, INT32U blockadddr);

extern INT8U SdCmd_Erase_Wr_Blk_Start(sd_struct *sds, INT32U blockadddr);
extern INT8U SdCmd_Erase_Wr_Blk_End(sd_struct *sds, INT32U blockadddr);
extern INT8U SdCmd_Erase(sd_struct *sds);

#endif // end of SDBUS_MODE_EN









/*********************************************************************************
                    
                    SD/MMC 卡 SPI 总线模式命令及响应

**********************************************************************************/

#if (!SDBUS_MODE_EN)

/* R1和R2高字节错误码 R1 and upper byte of R2 error code */
#define MSK_IDLE          		  0x01
#define MSK_ERASE_RST     		  0x02
#define MSK_ILL_CMD       		  0x04
#define MSK_CRC_ERR       		  0x08
#define MSK_ERASE_SEQ_ERR 		  0x10
#define MSK_ADDR_ERR      		  0x20
#define MSK_PARAM_ERR     		  0x40


/* R2低字节错误码 lower byte of R2 error code */
#define MSK_TOK_ERROR             0x01
#define MSK_TOK_CC_ERROR          0x02
#define MSK_TOK_ECC_FAILED        0x04
#define MSK_TOK_CC_OUTOFRANGE     0x08
#define MSK_TOK_CC_LOCKED         0x10


/* 数据令牌 Data Tokens */
#define SD_TOK_READ_STARTBLOCK    0xFE
#define SD_TOK_WRITE_STARTBLOCK   0xFE
#define SD_TOK_READ_STARTBLOCK_M  0xFE
#define SD_TOK_WRITE_STARTBLOCK_M 0xFC
#define SD_TOK_STOP_MULTI         0xFD

/* 数据响应令牌 Data Response Tokens */
#define SD_RESP_DATA_MSK		  0x0F		//数据响应掩码
#define SD_RESP_DATA_ACCETPTED	  0x05		//数据被接受
#define SD_RESP_DATA_REJECT_CRC	  0x0B      //由于CRC错误而被拒绝
#define SD_RESP_DATA_REJECT_WRITE 0x0D		//由于写错误而被拒绝

/* 等待类型 Wait Type */
#define SD_WAIT_READ			  0x00		//读等待
#define SD_WAIT_WRITE			  0x01		//写等待
#define SD_WAIT_ERASE		 	  0x02		//擦除等待



/* 命令响应定义 define command's response */
#define R1    1
#define R1B   2
#define R2    3
#define R3    4


/*
*********************************************

     SD/MMC卡SPI模式下命令集

*********************************************
*/

/******************************** 基本命令集 Basic command set **************************/
/* 复位SD 卡 Reset cards to idle state */
#define CMD0 0
#define CMD0_R R1

/* 读OCR寄存器 Read the OCR (MMC mode, do not use for SD cards) */
#define CMD1 1
#define CMD1_R R1

/* 选择高速模式*/
#define	CMD6 6
#define CMD6_R R1

/* 读CSD寄存器 Card sends the CSD */
#define CMD9 9
#define CMD9_R R1

/* 读CID寄存器 Card sends CID */
#define CMD10 10
#define CMD10_R R1

/* 停止读多块时的数据传输 Stop a multiple block (stream) read/write operation */
#define CMD12 12
#define CMD12_R R1B

/* 读 Card_Status 寄存器 Get the addressed card's status register */
#define CMD13 13
#define CMD13_R R2

/***************************** 块读命令集 Block read commands **************************/

/* 设置块的长度 Set the block length */
#define CMD16 16
#define CMD16_R R1

/* 读单块 Read a single block */
#define CMD17 17
#define CMD17_R R1

/* 读多块,直至主机发送CMD12为止 Read multiple blocks until a CMD12 */
#define CMD18 18
#define CMD18_R R1

/***************************** 块写命令集 Block write commands *************************/
/* 写单块 Write a block of the size selected with CMD16 */
#define CMD24 24
#define CMD24_R R1

/* 写多块 Multiple block write until a CMD12 */
#define CMD25 25
#define CMD25_R R1

/* 写CSD寄存器 Program the programmable bits of the CSD */
#define CMD27 27
#define CMD27_R R1

/***************************** 写保护 Write protection *****************************/
/* Set the write protection bit of the addressed group */
#define CMD28 28
#define CMD28_R R1B

/* Clear the write protection bit of the addressed group */
#define CMD29 29
#define CMD29_R R1B

/* Ask the card for the status of the write protection bits */
#define CMD30 30
#define CMD30_R R1

/***************************** 擦除命令 Erase commands *******************************/
/* 设置擦除块的起始地址(只用于SD卡) Set the address of the first write block to be erased(only for SD) */
#define CMD32 32
#define CMD32_R R1

/* 设置擦除块的终止地址(只用于SD卡) Set the address of the last write block to be erased(only for SD) */
#define CMD33 33
#define CMD33_R R1

/* 设置擦除块的起始地址(只用于MMC卡) Set the address of the first write block to be erased(only for MMC) */
#define CMD35 35
#define CMD35_R R1

/* 设置擦除块的终止地址(只用于MMC卡) Set the address of the last write block to be erased(only for MMC) */
#define CMD36 36
#define CMD36_R R1

/* 擦除所选择的块 Erase the selected write blocks */
#define CMD38 38
#define CMD38_R R1B

/***************************** 锁卡命令 Lock Card commands ***************************/
/* 设置/复位密码或上锁/解锁卡 Set/reset the password or lock/unlock the card */
#define CMD42 42
#define CMD42_R	R1B
/* Commands from 42 to 54, not defined here */

/***************************** 应用命令 Application-specific commands ****************/
/* 禁止下一个命令为应用命令  Flag that the next command is application-specific */
#define CMD55 55
#define CMD55_R R1

/* 应用命令的通用I/O  General purpose I/O for application-specific commands */
#define CMD56 56
#define CMD56_R R1

/* 读OCR寄存器  Read the OCR (SPI mode only) */
#define CMD58 58
#define CMD58_R R3

/* 使能或禁止 CRC Turn CRC on or off */
#define CMD59 59
#define CMD59_R R1

/***************************** 应用命令 Application-specific commands ***************/
/* 获取 SD Status寄存器 Get the SD card's status */
#define ACMD13 13
#define ACMD13_R R2

/* 得到已写入卡中的块的个数 Get the number of written write blocks (Minus errors ) */
#define ACMD22 22
#define ACMD22_R R1

/* 在写之前,设置预先擦除的块的个数 Set the number of write blocks to be pre-erased before writing */
#define ACMD23 23
#define ACMD23_R R1

/* 读取OCR寄存器 Get the card's OCR (SD mode) */
#define ACMD41 41
#define ACMD41_R R1

/* 连接/断开CD/DATA[3]引脚上的上拉电阻 Connect or disconnect the 50kOhm internal pull-up on CD/DAT[3] */
#define ACMD42 42
#define ACMD42_R R1

/* 读取SCR寄存器 Get the SD configuration register */
#define ACMD51 51
#define ACMD51_R R1


extern INT8U SdSpi_SendCmd(sd_struct *sds, INT8U cmd, INT8U *param, INT8U resptype, INT8U *resp);
extern void SdSpi_PackParam(INT8U *parameter, INT32U value);							// 封装参数

extern INT8U SdSpi_BlockCommand(sd_struct *sds, INT8U cmd, INT8U resptype, INT32U parameter);
 																					    // 块命令
	
extern INT8U SpiCmd_Go_Idle_State(sd_struct *sds);										// 复位SD卡		
extern INT8U SpiCmd_Send_OP_Cond(sd_struct *sds);

extern INT8U SpiCmd_Send_CSD(sd_struct *sds, INT8U csdlen, INT8U *recbuf);			    // 读CSD 			
extern INT8U SpiCmd_Send_CID(sd_struct *sds, INT8U cidlen, INT8U *recbuf);				// 读CID			
extern INT8U SpiCmd_Stop_Transmission(sd_struct *sds);									// 停止传输			

extern INT8U SpiCmd_Send_Status(sd_struct *sds, INT8U len, INT8U *buffer);			    // 读Card Status
extern INT8U SpiCmd_Set_BlockLen(sd_struct *sds, INT32U length);						// 设置块长度

extern INT8U SpiCmd_Read_Single_Block(sd_struct *sds, INT32U blockaddr);				// 读单块
extern INT8U SpiCmd_Read_Multiple_Block(sd_struct *sds, INT32U blockaddr);				// 读多块

extern INT8U SpiCmd_Write_Single_Block(sd_struct *sds, INT32U blockaddr);				// 写单块
extern INT8U SpiCmd_Write_Multiple_Block(sd_struct *sds, INT32U blockaddr);				// 写多块

extern INT8U SpiCmd_Program_CSD(sd_struct *sds, INT8U len, INT8U *buff);				// 写CSD寄存器 

extern INT8U SdCmd_Erase_Wr_Blk_Start(sd_struct *sds, INT32U startblock);			    // 擦块起始地址		
extern INT8U SdCmd_Erase_Wr_Blk_End(sd_struct *sds, INT32U endblock);				    // 擦块终止地址
extern INT8U SdCmd_Erase(sd_struct *sds);											    // 擦除所选的块
    
extern INT8U SpiCmd_ReadOCR(sd_struct *sds, INT32U *ocrbuf);                            // 读OCR         
extern INT8U SpiCmd_CRC_ON_OFF(sd_struct *sds, INT8U bEnable);						    // 使能CRC校验

// 特殊应用命令
extern	INT8U SpiCmd_ReadSD_Status(sd_struct *sds, INT8U sdslen, INT8U *recbuf);	    // 读SD_Status	
extern INT8U SpiCmd_Send_SCR(sd_struct *sds, INT8U scrlen, INT8U *recbuf);			    // 读SCR
extern INT8U SpiCmd_Send_Num_WR_Blcoks(sd_struct *sds, INT32U *blocknum);			    // 得到正确写入块数 


// 数据流函数
extern INT8U SdSpi_ReadRegister(sd_struct *sds, INT32U len, INT8U *recbuf);						// 读寄存器
extern INT8U SdSpi_ReadBlockData(sd_struct *sds, INT32U len, INT8U *recbuf);					// 读块数据
extern INT8U SdSpi_WriteBlockData(sd_struct *sds, INT8U bmulti, INT32U len, INT8U *sendbuf);	// 写块数据
	
// 其它函数
extern void SdSpi_StopMultiToken(sd_struct *sds);					// 停止多块写令牌	 
extern INT8U SdSpi_WaitBusy(sd_struct *sds, INT8U waittype);		// 忙
extern void SdSpi_Delay(sd_struct *sds, INT8U value);		        // 发生SPI时钟


//--------------hena
extern INT8U SdCmd_Speed_Mode(sd_struct *sds);
#endif  // end of !SDBUS_MODE_EN



#endif // end of __SDCMD_H__
























