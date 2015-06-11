
#ifndef __SDCOMMON_H__
#define __SDCOMMON_H__

#ifdef  SDCOMMON_GLOBALS
#define SDCOMMON_EXT  extern
#else
#define SDCOMMON_EXT extern
#endif

/* SD卡信息结构体定义 */ 
/* the information structure variable of SD Card*/
typedef struct SD_STRUCT
{	
	INT32U card_posnum;				/* 卡座号,多卡时使用 */
	INT8U card_type;                /* 卡型 */
	
	INT32U block_num;				/* 卡中块的数量 */
	INT32U block_len;				/* 卡的块长度(单位:字节) */
	INT32U erase_unit;				/* 一次可擦除的块个数 */
	
	INT32U timeout_read;			/* 读块超时时间(单位: 1 clock) */
	INT32U timeout_write;			/* 写块超时时间(单位: 1 clock) */
	INT32U timeout_erase;			/* 擦块超时时间(单位: 1 clock) */
	
	INT32U RCA;                     /* 卡的RCA地址 */
	INT32U host_clkval;             /* 主机时钟值(Hz)  */
	INT32U card_transpeed;          /* 卡数据最大传输速度(Hz) */
	
#if SD_UCOSII_EN
	INT32U timeout_read_os;			/* UCOS-II中,读块超时时间(单位: os tick) */
	INT32U timeout_write_os;		/* UCOS-II中,写块超时时间(单位: os tick) */
	INT32U timeout_erase_os;		/* UCOS-II中,擦块超时时间(单位: os tick) */

	OS_EVENT *pSemSD;	            /* 卡访问权限信号量 */
	OS_EVENT *pSemSdmmcRW;          /* 卡读写等待信号量 */
#endif 	
	
}sd_struct;
SDCOMMON_EXT  sd_struct sd_info;

/* SD/MMC 卡数据块长度 */
#define SD_BLOCKSIZE_NBITS		9  
#define SD_BLOCKSIZE 			(1 << SD_BLOCKSIZE_NBITS)		


/* 等待函数的等待类型 */
#define BUSY_TYPE_READ        0
#define BUSY_TYPE_RDY_DATA    1
#define BUSY_TYPE_PROG        2
#define BUSY_TYPE_ERASE       3      


/* 读写卡时钟: 慢速及标准速度 */
#define   SD_RATE_SLOW		  1
#define   SD_RATE_NORMAL	  2


/*****************************************
   CSD 寄存器相关定义
*****************************************/
/* CSD中一些域的字节位置(高字节在前) */
#define TAAC_POS 				1			//TAAC
#define NSAC_POS				2			//NSAC

#define TRAN_SPEED_POS      	3           //TRAN_SPEED 

#define READ_BL_LEN_POS			5			//READ_BL_LEN

#define C_SIZE_POS1				6			//C_SIZE upper  2-bit
#define C_SIZE_POS2				7			//C_SIZE middle 8-bit
#define C_SIZE_POS3				8			//C_SIZE lower	2-bit

#define C_SIZE_MULT_POS1		9			//C_SIZE_MULT upper 2-bit
#define C_SIZE_MULT_POS2		10			//C_SIZE_MULT lower 1-bit	

#define C_SIZE_HC_POS1		    9			//C_SIZE_MULT upper 2-bit
#define C_SIZE_HC_POS2		   10			//C_SIZE_MULT lower 1-bit	

#define SECTOR_SIZE_POS1		10			//SECTOR_SIZE upper 5-bit
#define SECTOR_SIZE_POS2		11			//SECTOR_SIZE lower 2-bit

#define R2WFACTOR_POS 			12			//R2WFACTOR_POS

/**/
#define ERASE_GRP_SIZE_POS   	10			//MMC卡 ERASE_GRP_SIZE 5-bit
#define ERASE_GRP_MULTI_POS1 	10			//MMC卡 ERASE_GRP_MULTI 2-bit
#define ERASE_GRP_MULTI_POS2 	11			//MMC卡 ERASE_GRP_MULTI 3-bit


//CSD中一些域的掩码
#define TAAC_MSK				0x07		//TAAC 域掩码
#define NSAC_MSK				0x78		//NSAC 域掩码

#define TRAN_SPEED_MSK1    	 	0x07        //TRAN_SPEED 传输速度指数掩码
#define TRAN_SPEED_MSK2     	0x78        //TRAN_SPEED 传输速度尾数 

#define READ_BL_LEN_MSK			0x0F		//READ_BL_LEN 的掩码

#define C_SIZE_MSK1				0x03		//C_SIZE 高2位掩码
#define C_SIZE_MSK3				0xC0		//C_SIZE 低2位掩码

#define C_SIZE_MULT_MSK1 		0x03		//C_SIZE_MULT 的高2位掩码
#define C_SIZE_MULT_MSK2 		0x80		//C_SIZE_MULT 的低2位掩码

#define R2WFACTOR_MSK			0x1C		//R2WFACTOR 掩码

#define SECTOR_SIZE_MSK1		0x3F		//SECTOR_SIZE 的高5位
#define SECTOR_SIZE_MSK2		0x80		//SECTOR_SIZE 的低2位

#define ERASE_GRP_SIZE_MSK 		0x7C		//MMC卡 ERASE_GRP_SIZE 掩码
#define ERASE_GRP_MULTI_MSK1 	0x03		//MMC卡 ERASE_GRP_MULTI 高2位掩码
#define ERASE_GRP_MULTI_MSK2 	0xE0		//MMC卡 ERASE_GRP_NULTI 低3位掩码


#endif

