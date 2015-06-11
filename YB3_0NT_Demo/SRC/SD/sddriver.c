										 /****************************************Copyright (c)**************************************************
**                               Guangzhou ZLG-MCU Development Co.,LTD.
**                                      graduate school
**                                 http://www.zlgmcu.com
**
**--------------File Info-------------------------------------------------------------------------------
** File name:			sddriver.c
** Last modified Date:	2007-5-19
** Last Version:		V2.0
** Descriptions:		SD/MMC卡读写软件包: 提供给用户API函数: 初始化,读,写,擦卡
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
** Modified date:		2005-3-10
** Version:				V2.0
** Descriptions:		增加了对MMC卡的支持,增加了对UCOS-II的支持,使该模块不仅能运行于前后台系统,还可运行
**						于UCOS-II上
**------------------------------------------------------------------------------------------------------
** Modified by:         Ming Yuan Zheng
** Modified date:       2007-5-19
** Version:	            V3.0
** Descriptions:        1. 增加了对SD/MMC卡 SD 总线模式的支持(不同模式下API函数名相同), 各API函数的入口增加了sd_struct结构体, 用于支持多卡读写
**
**                      2. SPI模式下和SD模式下共用 SDMMC_GetCardInfo() 和 SDMMC_CalTimeout()函数
**
**                      3. sd_struct结构体增加了成员多个成员变量, 访问卡的时钟及访问卡的最大时钟改为可动态获取
**
**                      4. SPI 模式下的命令函数使用SpiCmd_ 开头, SD 模式下的命令使用 SdCmd_ 开头
**
**                      5. SPI 模式下与硬件相关的函数使用 SdSpiHal_ 开头, SD 模式下与硬件相关的函数使用 SdHal_ 开头
********************************************************************************************************/


#include "sdconfig.h"
#include "sdcommon.h"
#include "sddriver.h"
#include "config.h"
#include "WT_Task.h"

#define OCR_HC_CCS              0x40000000
#define	GetSDInsertStatus		(P3_INP_STATE & SD_INSERT_BIT)		//SD卡插入状态


/* 超时时间单位表(单位:0.000000001ns) timeout unit table */
const INT32U time_unit[8] = {1000000000,100000000,10000000,
							 1000000,100000,10000,1000,100};

/* 超时时间表 timeout value table */							 
const INT8U time_value[16] = {0,10,12,13,15,20,25,30,
                              35,40,45,50,55,60,70,80};
 
/* 超时时间因数表 timeout factor table */                              
const INT8U r2w_fator[8] = {1,2,4,8,16,32,64,128};

/* 最大传输速度尾数表, 原为小数, 都乘 10 转换为整数 */
const INT8U tr_spval[16] = {0, 10, 12, 13, 15, 20, 25, 30, 35,
                          40, 45, 50, 55, 60, 70, 80};

/* 最大传输速度指数表, 单位 clock, 从100K ~ 100M */
const INT32U tr_spexp[4] = {100000, 1000000, 10000000, 100000000};



static sd_struct sd_info;                          
	/*       
	***************************************************************************************************
		
	     用户API函数:  初始化,读,写,擦 SD/MMC卡
				
	***************************************************************************************************
	*/

void SD_RequestOSSem(sd_struct *sds);
void SD_ReleaseOSSem(sd_struct *sds);
INT8U SDMMC_GetCardInfo(sd_struct *sds);
INT8U SDMMC_CalTimeout(sd_struct *sds);


/*
*****************************************************************************
   本软件包的SD总线模式, 适用于LPC2368, LPC2378 等带用 SD 控制器的微控制器
*****************************************************************************
*/

#if SDBUS_MODE_EN

INT8U SDMMC_ConfigCard(sd_struct *sds);
INT8U SDMMC_IdentifyCard(sd_struct *sds);


//==============================
/*SD卡写单块函数*/
INT8U ReadSDCardBlock(INT32U blockaddr, INT8U *recbuf)
{
	uint8 sdreadstat;

	sdreadstat=SD_ReadBlock(&sd_info, blockaddr, recbuf);
	if(sdreadstat!=0x00)
	{
		OSTimeDly(50);
		sdreadstat=SD_ReadBlock(&sd_info, blockaddr, recbuf);
	}
	if(sdreadstat!=0x00)
	{
		OSTimeDly(50);
		sdreadstat=SD_ReadBlock(&sd_info, blockaddr, recbuf);
	}
	
	return	sdreadstat;
		
}

//==============================
//==============================
/*SD卡读单块函数*/
INT8U WriteSDCardBlock(INT32U blockaddr, INT8U *sendbuf)
{
	uint8 sdwritestat;
	sdwritestat=SD_WriteBlock(&sd_info, blockaddr, sendbuf);
//	if(sdwritestat!=0x00)
//	{
//		OSTimeDly(1);
//		sdwritestat=SD_WriteBlock(&sd_info, blockaddr, sendbuf);
//	}
//	if(sdwritestat!=0x00)
//	{
//		OSTimeDly(1);
//		sdwritestat=SD_WriteBlock(&sd_info, blockaddr, sendbuf);
//	}

	return	sdwritestat;
}

//==============================
/*SD卡初始化函数*/
void SDCardInit(void)
{
	uint8	l_u8Status;
	P2_DIR_CLR = (1<<25);
	
	l_u8Status	= SD_Initialize(&sd_info);                                            /* 初始化SD卡                   */
    
    if (l_u8Status != SD_NO_ERR) 
    {
       //初始化失败，响铃三声提示
		BeepON();
		Delay(500);	
		BeepOFF();
		Delay(1000);
		BeepON();
		Delay(500);	
		BeepOFF();
		Delay(1000);
		BeepON();
		Delay(500);
		BeepOFF();
		Flag_SD_Init_err = 0;
    }
	else
	{
	   	BeepON();
		Delay(500);	
		BeepOFF();
		Flag_SD_Init_err = 1;
	}
	
    micIrqFuncSet(13, 3, (uint32)MCI_Handler);						/* 设置SD/MMC卡控制器的中断向量 */

	if (GetSDInsertStatus != 0)				//检测卡插入状态
		g_u8SDInsert=0;						/* 未完全插入 */	
	else
		g_u8SDInsert=1;						/* 完全插入 */	
}


/*******************************************************************************************************************
** 函数名称: SD_Initialize()				
**
** 功能描述: SD/MMC 卡初始化: 复位卡,识别卡,获取卡相关信息,使卡从识别模式进入数据传输模式 
**
** 输　  入: sd_struct *sds: SD/MMC卡信息结构体 
**	         
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
********************************************************************************************************************/
INT8U SD_Initialize(sd_struct *sds)

{
	INT8U ret, cidbuf[16];
//	INT32U i=0;
	
	if (sds == NULL) return SD_ERR_USER_PARAM;          /* 函数入口参数错误 */  
		
#if SD_UCOSII_EN
	if (sds->pSemSD == NULL)
	{
    	sds->pSemSD = OSSemCreate(1);					/* 创建访问卡信号量 */ 
		if (sds->pSemSD == NULL)					
		 	return SD_ERR_CREATE_SEMSD;	   				
	}
	
	if (sds->pSemSdmmcRW == NULL)
	{
    	sds->pSemSdmmcRW = OSSemCreate(0); 				/* 创建读写擦卡等待信号量 */ 
		if (sds->pSemSdmmcRW == NULL)	   				
		 	return SD_ERR_CREATE_SEMSD;	   				
	}					    
#endif
	SD_RequestOSSem(sds);				   				/* 向 OS 请求获取访问卡信号量 */
	
	if (sds->RCA != 0)  sds->RCA = 0;
	
	SdHal_Initialize(sds);				   			    /* HAL层初始化 */
 	
 	if (!SdHal_CheckCard(sds))			   				/* 检测卡是否完全插入卡座 */
 	{
 		ret = SD_ERR_NO_CARD;
 		goto SDINIT_END;
 	}
 	
	ret = SdCmd_Go_Idle_State(sds);		   				/* CMD0: 复位卡, 使卡进入空闲状态 */
	if (ret != SD_NO_ERR)	
		goto SDINIT_END;

	
	ret = SD_CheckSDVersion(sds);                       /* CMD8:识别卡型，大容量卡或普  */
                                                        /* 通卡                         */

    if (ret == SD_V1)																		
    {																					
        ret = SDMMC_IdentifyCard(sds);                  /* CMD1或ACMD41: 识别卡, 进入就 */
                                                        /* 绪状态                       */
        if (ret != SD_NO_ERR)
        goto SDINIT_END;
    }																					
    else if (ret == SD_V2)
    {
        ret = SD_Ver2ActiveInit(sds);                   /* ACMD41: 识别卡, 进入就绪状态 */
        
        if (ret != SD_NO_ERR)
        {
            goto SDINIT_END;
        }
    }
    else
    {
        ret = SD_ERR_UNKNOWN_CARD;
       
        goto SDINIT_END;
    } 
	  
	ret = SdCmd_All_Send_CID(sds, 16, cidbuf);          /* CMD2: 读CID, 进入识别状态 */
    if (ret != SD_NO_ERR)
	    goto SDINIT_END;
	    
	ret = SdCmd_Set_Relative_Addr(sds);	   				/* CMD3: 获取RCA, 进入待机状态 */
   	if (ret != SD_NO_ERR)	
   		goto SDINIT_END;
 
 
  
    ret = SDMMC_GetCardInfo(sds);		   				/* CMD9: 读取CSD, 获取卡的属性 */
    if (ret != SD_NO_ERR)
    	goto SDINIT_END;

	ret = SDMMC_CalTimeout(sds);		   				/* 计算超时时间值 */
	if (ret != SD_NO_ERR)
	    goto SDINIT_END;


	SdHal_SetMCIClock(sds, SD_RATE_NORMAL);  			/* 设置访问卡的clock为标准clock */		
									    
  	ret = SDMMC_ConfigCard(sds);		   				/* 设置卡相关参数, 处于待机状态   */
    if (ret != SD_NO_ERR)
        goto SDINIT_END;
			
//	ret = SdCmd_Select_Card(sds);                       /* CMD7: 进入传输状态  */
//    if (ret != SD_NO_ERR)
//        goto SDINIT_END;

//	ret = SdCmd_Speed_Mode(sds);
//    if (ret != SD_NO_ERR)
//    	goto SDINIT_END; 
		
	 
SDINIT_END:
	//vicIrqFuncSet(13, 3, (uint32)MCI_Handler);
//	SdCmd_Deselect_Card(sds, 0);
	SD_ReleaseOSSem(sds);	                            /* 释放访问卡信号量 */
	return ret;  	
}


/*******************************************************************************************************************
** 函数名称: SD_ReadBlock()				
**
** 功能描述: 从SD/MMC卡中读出一个数据块
**
** 输　  入: sd_struct *sds  : SD/MMC卡信息结构体 
**           INT32U blockaddr: 以块为单位的块地址, 例如, 卡开始的0 ~ 511字节为块地址0, 512 ~ 1023字节的块地址为1
**	         
** 输 　 出: INT8U *recbuf   : 接收缓冲区,长度固定为 512 字节	
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
********************************************************************************************************************/
INT8U SD_ReadBlock(sd_struct *sds, INT32U blockaddr, INT8U *recbuf)
{
	INT8U ret;

    if ((sds == NULL) || (recbuf == NULL))          
        return SD_ERR_USER_PARAM;                   /* 函数入口参数错误 */ 

	SD_RequestOSSem(sds);							/* 向OS申请访问卡信号量 */
    if (!SdHal_CheckCard(sds))							
    {	
    	ret = SD_ERR_NO_CARD;   					/* 卡没完全插入座卡中 */
		goto SDRD_BLK_END;
	}
	
	if (blockaddr > sds->block_num)	
	{
		ret = SD_ERR_OVER_CARDRANGE;				/* 操作超出卡容量范围 */
	    goto SDRD_BLK_END;
	}
	
	ret = SdCmd_Select_Card(sds);					/* CMD7: 进入传输状态  */
    if (ret != SD_NO_ERR)
    	goto SDRD_BLK_END;
  			
	ret = SdHal_BeforeReadBlock(sds);               /* 发送读命令之前的准备工作 */
	if (ret != SD_NO_ERR)
		goto SDRD_BLK_END;
	
	ret = SdCmd_Read_Single_Block(sds, blockaddr);	/* CMD17: 发送读单块命令 */						
 	if (ret != SD_NO_ERR)
 		goto SDRD_BLK_END;	
	
	ret = SdHal_ReadBlock(sds, recbuf);             /* 从卡接收数据 */
	if (ret != SD_NO_ERR)
		goto SDRD_BLK_END;
  	
SDRD_BLK_END:
    SdCmd_Deselect_Card(sds, 0);				/* CMD7: 退出传输状态 */
	SD_ReleaseOSSem(sds);                           /* 释放访问卡信号量 */
    return ret;						
}


/*******************************************************************************************************************
** 函数名称: SD_ReadMultiBlock()				
**
** 功能描述: 从SD/MMC卡中读出多个数据块
**
** 输　  入: sd_struct *sds  : SD/MMC卡信息结构体 
**           INT32U blockaddr: 以块为单位的块地址
**	         INT32U blocknum : 要读取的块的个数
**
** 输 　 出: INT8U *recbuf   : 接收缓冲区, 长度512 * blocknum 字节	
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
********************************************************************************************************************/
#if SD_ReadMultiBlock_EN
INT8U SD_ReadMultiBlock(sd_struct *sds, INT32U blockaddr, INT32U blocknum, INT8U *recbuf)
{
	INT8U ret;
	INT32U i;
	
    if ((sds == NULL) || (recbuf == NULL)) 
        return SD_ERR_USER_PARAM;                   /* 函数入口参数错误 */	
	
	SD_RequestOSSem(sds);							/* 向OS申请访问卡信号量 */
    if (!SdHal_CheckCard(sds))							
    {	
    	ret = SD_ERR_NO_CARD;   					/* 卡没完全插入卡座中 */
		goto SDRD_MBLK_END;
	}

	if (blockaddr > sds->block_num)	
	{
		ret = SD_ERR_OVER_CARDRANGE;				/* 操作超出卡容量范围 */
	    goto SDRD_MBLK_END;
	}

	ret = SdCmd_Select_Card(sds);					/* CMD7: 进入传输状态  */
    if (ret != SD_NO_ERR)
    	goto SDRD_MBLK_END;
   		
	ret = SdHal_BeforeReadBlock(sds);               /* 发送读命令之前的准备工作 */    
	if (ret != SD_NO_ERR)
		goto SDRD_MBLK_END;
			
	ret = SdCmd_Read_Multiple_Block(sds, blockaddr);/* 发送读多块命令 */						
 	if (ret != SD_NO_ERR)
 		goto SDRD_MBLK_END;
	
    for (i = 0; i < blocknum; i++)
    {		
    	ret = SdHal_ReadBlock(sds, recbuf);         /* 接收来自卡的数据 */
    	if (ret == SD_NO_ERR)
       		recbuf = recbuf + SD_BLOCKSIZE;
    	else
            goto SDRD_MBLK_END; 
        
        if (i < (blocknum - 1))
        {
            ret = SdHal_BeforeReadBlock(sds);       /* 接收来自卡数据之前的准备工作 */  
	        if (ret != SD_NO_ERR)
		        goto SDRD_MBLK_END;      
        }
    }
 
  	ret = SdCmd_Stop_Transmission(sds);             /* 停止传输 */
//  	if (ret != SD_NO_ERR)
//		goto SDRD_MBLK_END;  	

SDRD_MBLK_END:
    SdCmd_Deselect_Card(sds, 0);				/* CMD7: 退出传输状态 */
	SD_ReleaseOSSem(sds);
    return ret;						
}
#endif


/*******************************************************************************************************************
** 函数名称: SD_WriteBlock()				
**
** 功能描述: 向SD/MMC卡中写入一个块	
**
** 输　  入: sd_struct *sds  : SD/MMC卡信息结构体 
**           INT32U blockaddr: 以块为单位的块地址, 例如, 卡开始的0 ~ 511字节为块地址0, 512 ~ 1023字节的块地址为1
**           INT8U *sendbuf  : 发送缓冲区,长度固定为 512 字节	
**	         
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
********************************************************************************************************************/
INT8U SD_WriteBlock(sd_struct *sds, INT32U blockaddr, INT8U *sendbuf)
{
	INT8U ret;
//	uint8 i;

    if ((sds == NULL) || (sendbuf == NULL)) 
        return SD_ERR_USER_PARAM;                    /* 函数入口参数错误 */

	SD_RequestOSSem(sds);							 /* 向OS申请访问卡的信号量 */
    if (!SdHal_CheckCard(sds))							
    {
    	ret = SD_ERR_NO_CARD;   					 /* 卡没完全插入卡座中 */
		goto SDWR_BLK_END;
	}	
	
	if (blockaddr > sds->block_num)	
	{
		ret = SD_ERR_OVER_CARDRANGE;				 /* 操作超出卡容量范围 */
		goto SDWR_BLK_END;
	}
//	
//	if (SdHal_CheckCardWP(sds))
//	{
//		ret = SD_ERR_WRITE_PROTECT;					 /* 卡有写保护 */
//		goto SDWR_BLK_END;
//	}
	
	ret = SdCmd_Select_Card(sds);					 /* CMD7: 进入传输状态  */
    if (ret != SD_NO_ERR)
        goto SDWR_BLK_END;
 	
	ret = SdHal_BeforeWriteBlock(sds);               /* 写数据之前的准备工作 */
	if (ret != SD_NO_ERR)
		goto SDWR_BLK_END;
//	for(i=0;i<20;i++);		
 
	ret = SdCmd_Write_Single_Block(sds, blockaddr);	 /* CMD24, 发送写单块命令 */						
 	if (ret != SD_NO_ERR)
 		goto SDWR_BLK_END;
  
	ret = SdHal_WriteBlock(sds, sendbuf);            /* 往卡中写入数据 */   
	if (ret != SD_NO_ERR)
		goto SDWR_BLK_END;
		
	ret = SdHal_WaitBusy(sds, BUSY_TYPE_PROG);      /* 等待卡编程完成 */
	if (ret != SD_NO_ERR)
        goto SDWR_BLK_END;
           
SDWR_BLK_END:
    SdCmd_Deselect_Card(sds, 0);              /* CMD7: 退出传输状态 */  
	SD_ReleaseOSSem(sds);
	return ret;                                     /* 返回操作结果 */	
}


/*******************************************************************************************************************
** 函数名称: SD_WriteMultiBlock()				
**
** 功能描述: 向SD/MMC卡中写入多个数据块	
**
** 输　  入: sd_struct *sds  : SD/MMC卡信息结构体 
**           INT32U blockaddr: 以块为单位的块地址
**           INT32U blocknum : 要写入的块的个数
**           INT8U *sendbuf  : 发送缓冲区, 长度 512 * blocknum 字节	
**	         
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
********************************************************************************************************************/
#if SD_WriteMultiBlock_EN
INT8U SD_WriteMultiBlock(sd_struct *sds, INT32U blockaddr, INT32U blocknum, INT8U *sendbuf)
{
	INT8U ret;
	INT32U i;

    if ((sds == NULL) || (sendbuf == NULL)) 
        return SD_ERR_USER_PARAM;                       /* 函数入口参数错误 */

	SD_RequestOSSem(sds);                               /* 向OS申请访问卡信号量 */

//    if (!SdHal_CheckCard(sds))							
//    {
//    	ret = SD_ERR_NO_CARD;                           /* 卡没完全插入卡座中 */
//		goto SDWR_MBLK_END;
//	}	
	
//	if (blockaddr > sds->block_num)	
//	{
//		ret = SD_ERR_OVER_CARDRANGE;                    /* 操作超出卡容量范围 */
//		goto SDWR_MBLK_END;
//	}
	
//	if (SdHal_CheckCardWP(sds))
//	{
//		ret = SD_ERR_WRITE_PROTECT;                     /* 卡有写保护 */
//		goto SDWR_MBLK_END;
//	}

	ret = SdCmd_Select_Card(sds);                       /* CMD7: 进入传输状态  */
    if (ret != SD_NO_ERR)
        goto SDWR_MBLK_END;
 	
	ret = SdHal_BeforeWriteBlock(sds);                  /* 发送数据前的准备工作 */
	if (ret != SD_NO_ERR)
		goto SDWR_MBLK_END;		
 
	ret = SdCmd_Write_Multiple_Block(sds, blockaddr);   /* CMD25, 发送写多块命令 */						
 	if (ret != SD_NO_ERR)
 		goto SDWR_MBLK_END;

    for (i = 0; i < blocknum; i++)
    {
		ret = SdHal_WriteBlock(sds, sendbuf);           /* 写入数据 */
 		if (ret == SD_NO_ERR)
 		{	
 			sendbuf = sendbuf + SD_BLOCKSIZE;
 		}
 		else
 		{												/* 写失败  */
			SdCmd_Stop_Transmission(sds);				/* 停止数据传输并等待忙结束 */	
			SdHal_WaitBusy(sds, BUSY_TYPE_PROG);
			goto SDWR_MBLK_END;
		}
		
		ret = SdHal_WaitReadyforData(sds);              /* 等待卡可接收数据 */
        if (ret != SD_NO_ERR)
		   goto SDWR_MBLK_END;	
		
		if (i < (blocknum - 1))
		{		
 		    ret = SdHal_BeforeWriteBlock(sds);          /* 再次准备发送数据 */
	        if (ret != SD_NO_ERR)
		        goto SDWR_MBLK_END;		    
	    }
	}

    ret = SdCmd_Stop_Transmission(sds);                 /* 停止数据传输 */
    if (ret != SD_NO_ERR)
	    goto SDWR_MBLK_END;

    ret = SdHal_WaitBusy(sds, BUSY_TYPE_PROG);          /* 等待卡编程结束 */
    if (ret != SD_NO_ERR)
        goto SDWR_MBLK_END;

SDWR_MBLK_END:
    SdCmd_Deselect_Card(sds, 0);					/* CMD7, 退出传输状态 */  
	SD_ReleaseOSSem(sds);
	return ret;
}
#endif

/*******************************************************************************************************************
** 函数名称: SD_Ver2ActiveInit()
**
** 功能描述: SD2.00 SDHC card识别
**
** 输　  入: sd_struct *sds  : SD/MMC卡信息结构体
**
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
********************************************************************************************************************/
INT8U SD_Ver2ActiveInit(sd_struct *sds)
{
    INT32U resp[4]={0},ret;
    INT32U retry;
    retry = 0;
    do
    {
        ret = SdHal_SendCmd(sds, CMD55, 0x00000000, CMD55_R, resp, 0x01);
                                                                        /* CMD55:下个命令为应用程序命令 */
        if (ret != SD_NO_ERR)
        {
            return ret;
        }
        ret = SdHal_SendCmd(sds, ACMD41, 0x40FF8000, ACMD41_R, resp, 0x01);
                                                                        /* ACMD41:卡发送OCR内容响应     */
        if (ret != SD_NO_ERR)
        {
            return SD_ERR_UNKNOWN_CARD;                                 /* 不可识别卡                   */
        }
        retry++;
        if(retry > SD_IDLE_WAIT_MAX )                                   /* 超时错误                     */
        {
            return SD_ERR_TIMEOUT_WAITIDLE;
        }

    }while ((resp[0] & 0x80000000) == 0);                               /* 等待初始化完成HCS = 1        */

    if ((resp[0] & OCR_HC_CCS) == 0)
    {
        sds->card_type = CARDTYPE_SD;                                   /* 卡型为SD卡                   */
    }
    else
    {
        sds->card_type = CARDTYPE_SDHC;                                 /* 卡型 HCSD 卡                 */
    }

    return SD_NO_ERR;
}
/*******************************************************************************************************************/
/*******************************************************************************************************************
** 函数名称: SDMMC_IdentifyCard()		
**
** 功能描述: 识别卡: SD, MMC, 或不可卡别卡
**
** 输　  入: sd_struct *sds: SD/MMC卡信息结构体 
**	         
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
**
** 注    意: 执行后, 卡型保存在sds->card_type中
********************************************************************************************************************/
INT8U SDMMC_IdentifyCard(sd_struct *sds)
{
	if (SdCmd_Send_OP_Cond(sds) != SD_NO_ERR)			/* CMD1: 命令仅适合于MMC卡 */
	{
		if (SdCmd_SD_App_OP_Cond(sds) == SD_NO_ERR)     /* ACMD41: 读取OCR 寄存器 */
		{                                               
			sds->card_type = CARDTYPE_SD;               /* 卡型为 SD卡 */
		}
		else
		{		
			sds->card_type = CARDTYPE_UNKNOWN;          /* 不可识别的卡 */
			return SD_ERR_UNKNOWN_CARD;						
		}
	}
	else
		sds->card_type = CARDTYPE_MMC;	                /* 卡型为 MMC卡 */
		
	return SD_NO_ERR;		
}


/*******************************************************************************************************************
** 函数名称: SDMMC_ConfigCard()				
**
** 功能描述: 对卡的相关参数进行配置, 如卡总线宽度, 卡数据块大小
**
** 输　  入: sd_struct *sds: SD/MMC卡信息结构体
**	         
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
********************************************************************************************************************/
INT8U SDMMC_ConfigCard(sd_struct *sds)
{
	INT8U ret;
	
	SdHal_OpenDrainCmd(sds, 0);                         /* 禁止CMD线的开漏模式 */
		
    // 下面进入传输状态, 设置卡的相关参数    	
 	ret = SdCmd_Select_Card(sds);			            /* CMD7,进入传输状态   */
    if (ret != SD_NO_ERR)
       return ret;
 
   	if ((sds->card_type == CARDTYPE_SD)||(sds->card_type == CARDTYPE_SDHC))		//20120104	
    {                                                   /* 只有SD卡既可为宽数据总线, 又可为单数据总线 */
#if SD_WIDTHBUS_EN   
		SdHal_EnableMCIBusWidth(sds, 1);
		ret = SdCmd_Set_Bus_Width(sds, 1);    			/* ACMD6设置卡为宽数据总线方式 */
#else
		SdHal_EnableMCIBusWidth(sds, 0);
        ret = SdCmd_Set_Bus_Width(sds, 0);    			/* ACMD6设置卡为单数据总线方式 */
#endif  
    	if (ret != SD_NO_ERR)
    		return ret; 
    }	
    else	   
		SdHal_EnableMCIBusWidth(sds, 0);                /* MMC卡只能为单数据总线方式 */
 	
 	ret = SdCmd_Set_BlockLen(sds, SD_BLOCKSIZE);        /* CMD16, 置块数据大小为: 512字节 */
 	if (ret != SD_NO_ERR)
 	    return ret;
  	
       if (sds->block_len != SD_BLOCKSIZE) {
  		sds->block_num *= (sds->block_len / SD_BLOCKSIZE);
  		sds->block_len = SD_BLOCKSIZE;
  	}

 	SdCmd_Deselect_Card(sds, 0);		            /* CMD7, 退出传输状态 */  
 	return ret;	
}

#endif


/*******************************************************************************************************************
** 函数名称: SDMMC_GetCardInfo()				
**
** 功能描述: 获得SD/MMC卡的信息	
**
** 输　  入: sd_struct *sds: 卡属性结构体
**	         
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
********************************************************************************************************************/
INT8U SDMMC_GetCardInfo(sd_struct *sds)
{
    INT32U tmp,i,j;
    INT8U csdbuf[16],ret;

#if SDBUS_MODE_EN
    ret = SdCmd_Send_CSD(sds, 16, csdbuf);                              /* 读CSD寄存器    read CSD regi */
                                                                        /* ster                         */
#else
    ret = SpiCmd_Send_CSD(sds, 16, csdbuf);
#endif
    if (ret != SD_NO_ERR)
    return ret;

    /* 计算块的最大长度  */                                             /* calculate the size of a sect */
                                                                        /* or                           */
    sds->block_len = 1 << (csdbuf[READ_BL_LEN_POS] & READ_BL_LEN_MSK);  /* (2 ^ READ_BL_LEN)            */

    if (sds->card_type == CARDTYPE_SDHC)                                /* 大容量卡     High Capacity c */
                                                                        /* ard                          */
    {
        /* 计算卡中块的个数 */                                          /* calculate the sector numbers */
                                                                        /* of the SD Card               */
//        sds->block_num = (((csdbuf[C_SIZE_HC_POS1] << 8) +
//        csdbuf[C_SIZE_HC_POS1]) + 1) << 10;                           /* (C_SIZE + 1)* 1K  */	//4.16-修改前

//		  sds->block_num = (((csdbuf[C_SIZE_HC_POS1] << 8) +
//                           csdbuf[C_SIZE_HC_POS1]) + 1) << 9;			//(C_SIZE + 1)* 512   //4.16-fan-第一次修改


		  tmp = csdbuf[7] & 0x0f;
		  tmp = tmp << 16;
	  	  i = csdbuf[8]<< 8;
		  j = csdbuf[9];
		  tmp = tmp | i | j;
		  //tmp = ((csdbuf[7] & 0x0f)<<12 + csdbuf[8]<<4 + csdbuf[9]);
		
		  sds->block_num = 1024* (tmp + 1);								  //4.16-fan-第二次修改
	}



    else
    {

        /* 计算卡中块的个数 */                                          /* calculate the sector numbers */
                                                                        /* of the SD Card               */
        sds->block_num = ((csdbuf[C_SIZE_POS1] & C_SIZE_MSK1) << 10) +
        (csdbuf[C_SIZE_POS2] << 2) +
        ((csdbuf[C_SIZE_POS3] & C_SIZE_MSK3) >> 6) + 1;                 /* (C_SIZE + 1)                 */

        tmp = ((csdbuf[C_SIZE_MULT_POS1] & C_SIZE_MULT_MSK1) << 1) +
        ((csdbuf[C_SIZE_MULT_POS2] & C_SIZE_MULT_MSK2) >> 7) + 2;       /* (C_SIZE_MULT + 2)            */

        /* 获得卡中块的数量 */                                          /* get the block numbers in car */
                                                                        /* d                            */
        sds->block_num = sds->block_num * (1 << tmp);                   /* (C_SIZE + 1) * 2 ^ (C_SIZE_M */
                                                                        /* ULT + 2)                     */
    }
                                                                        /* 计算擦除的单位(单位: 块)     */
    if (sds->card_type == CARDTYPE_MMC)
    {
        tmp  = ((csdbuf[ERASE_GRP_SIZE_POS] & ERASE_GRP_SIZE_MSK) >> 2) + 1;
                                                                        /* (ERASE_GRP_SIZE + 1)         */

                                                                        /* (ERASE_GRP_SIZE + 1) * (ERAS */
                                                                        /* E_GRP_MULTI_MSK1 + 1)        */
        tmp *= ((csdbuf[ERASE_GRP_MULTI_POS1] & ERASE_GRP_MULTI_MSK1) << 3) +
        ((csdbuf[ERASE_GRP_MULTI_POS2] & ERASE_GRP_MULTI_MSK2) >> 5) + 1;
    }
    else                                                                /* calculate the size of sector */
    tmp = ((csdbuf[SECTOR_SIZE_POS1] & SECTOR_SIZE_MSK1) << 1) +
    ((csdbuf[SECTOR_SIZE_POS2] & SECTOR_SIZE_MSK2) >> 7) + 1;           /* SD: SECTOR_SIZE              */

    sds->erase_unit = tmp;                                              /* 擦除单位(块)                 */

    sds->card_transpeed = tr_spexp[csdbuf[TRAN_SPEED_POS] & TRAN_SPEED_MSK1] *
    tr_spval[(csdbuf[TRAN_SPEED_POS] & TRAN_SPEED_MSK2) >> 3] / 10;

    return SD_NO_ERR;                                                   /* 返回执行成功 return perform  */
                                                                        /* sucessfully                  */

}




/*******************************************************************************************************************
** 函数名称: SDMMC_CalTimeout()				
**
** 功能描述: 计算读/写/擦超时时间
**
** 输　  入: INT8U *csdbuf : CSD寄存器内容, 长度为16字节
**	         
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
**
** 注    意: 所有的超时时间都是以一个Clock为单位
********************************************************************************************************************/

INT8U SDMMC_CalTimeout(sd_struct *sds)
{
	INT32U tmp;
	INT8U time_u,time_v,fator;
	INT8U csdbuf[16],ret;
	#if SDBUS_MODE_EN
		ret = SdCmd_Send_CSD(sds, 16, csdbuf);								/* 读CSD寄存器 */
	#else
	    ret = SpiCmd_Send_CSD(sds, 16, csdbuf);
	#endif
		if (ret != SD_NO_ERR)	
			return ret;	
	if (sds->card_type == CARDTYPE_SD) 
	{
	//	INT32U t_100ms = sds->host_clkval / 10;                             /* 100ms 当量, 即多少个clock */
	//	INT32U t_250ms = sds->host_clkval / 4;                              /* 250ms 当量, 即多少个clock */
		INT32U t_100ms = sds->host_clkval / 2;                             /* 100ms 当量, 即多少个clock */
		INT32U t_250ms = sds->host_clkval / 2;                             /* 250ms 当量, 即多少个clock */
					
		sds->timeout_read = t_100ms;								        /* 默认读超时为100ms */
		sds->timeout_write = t_250ms;							            /* 默认写超时为250ms */
		sds->timeout_erase = t_250ms;                                       /* 默认擦超时为250ms */
			
		time_u = (csdbuf[TAAC_POS] & TAAC_MSK);								/* 读超时时间单位 read timeout unit */
		time_v = (csdbuf[TAAC_POS] & NSAC_MSK) >> 3;						/* 读超时时间值   read timeout value */
		fator = (csdbuf[R2WFACTOR_POS] & R2WFACTOR_MSK) >> 2;				/* 读超时时间因数 read timeout factor */
		
		if(time_v == 0)	return SD_ERR_CARD_PARAM;							/* 卡参数有错误 card parameter is error */
		
		tmp = sds->host_clkval * time_value[time_v] / 10 / time_unit[time_u];	/* TAAC * f (单位 unit: clock) */
		tmp = tmp + csdbuf[NSAC_POS] * 100;									    /* TAAC * f + NSAC * 100 (单位 unit: clock) */
		
		/* 计算得到的超时值 the timeout value of being calculated */
		sds->timeout_read = tmp*4;
		sds->timeout_write = tmp * r2w_fator[fator];						/* (TAAC * f + NSAC * 100) * R2WFACTOR (单位 unit:clock)*/
		
		if (sds->card_type == CARDTYPE_SD)
		{
			sds->timeout_read  = (sds->timeout_read * 100);     			/* 实际值为计算值的100倍 */
			sds->timeout_write = (sds->timeout_write * 100);
			if (sds->timeout_read > t_100ms)						        /* 取计算值与默认值中的最小值 */
				sds->timeout_read = t_100ms;
			
			if (sds->timeout_write > t_250ms)
				sds->timeout_write = t_250ms;
		}
		else
		{
			sds->timeout_read  = (sds->timeout_read * 10);      			/* 实际值为计算值的10倍 */
			sds->timeout_write = (sds->timeout_write * 10);
		}
	}
	if (sds->card_type == CARDTYPE_SDHC)	 			//fan-添加，SDHC卡的读写超时时间计算方法不同
	{
//		INT32U t_100ms = (sds->host_clkval /10)*400;    /* 多少个clock */
//		INT32U t_250ms = (sds->host_clkval / 4)*400;
		INT32U t_100ms = 0x1dc230;                      /* 多少个clock */
		INT32U t_250ms = 0x3b8360;
		sds->timeout_read = t_100ms;								        
		sds->timeout_write = t_250ms;							           
		sds->timeout_erase = t_250ms;    
	} 	
	
#if SD_UCOSII_EN
    if (sds->host_clkval == 0)  return SD_ERR_SD_STRUCT;
	sds->timeout_read_os  = OS_TICKS_PER_SEC/10 ; 	//
	sds->timeout_write_os = OS_TICKS_PER_SEC/4 ;	//
	sds->timeout_erase_os = sds->timeout_write_os;						/* (单位 unit: os tick) */

//	sds->timeout_read_os  = sds->timeout_read * OS_TICKS_PER_SEC / sds->host_clkval; 
//	sds->timeout_write_os = sds->timeout_write * OS_TICKS_PER_SEC / sds->host_clkval;
//	sds->timeout_erase_os = sds->timeout_write_os;						/* (单位 unit: os tick) */
#endif	

    /* SPI模式及非UCOS环境下 */
#if	(!SDBUS_MODE_EN)
#if (!SD_UCOSII_EN)
    sds->timeout_read = sds->timeout_read / 8;                         /* SPI总线模式则折算为8clock为单位 */
    sds->timeout_write = sds->timeout_write / 8;
#endif
#endif

	sds->timeout_erase = sds->timeout_write;
	
	return SD_NO_ERR;
}

/*******************************************************************************************************************
** 函数名称: SD_EraseBlock()				
**
** 功能描述: 擦除SD/MMC卡中的块
**
** 输　  入: sd_struct *sds   : 卡属性结构体
**	         INT32U startaddr : 擦卡的起始地址, 地址为以块为单位的块地址
**           INT32U blocknum  : 擦除的块数            
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
**
** 注    意: startaddr 和 blocknum 建议为 sds.erase_unit 的整数倍, 因为有的卡只能以 sds.erase_unit 为单位进行擦除
********************************************************************************************************************/
#if SD_EraseBlock_EN
INT8U SD_EraseBlock(sd_struct *sds, INT32U startaddr, INT32U blocknum)
{
	INT32 tmp;
	INT8U ret;
	
	if (sds == NULL) return SD_ERR_USER_PARAM;                              /* 函数入口参数错误 */
	
	SD_RequestOSSem(sds);                                                   /* 向OS申请访问卡的信号量 */
    if (!SdHal_CheckCard(sds))							
    {
    	ret = SD_ERR_NO_CARD;                                               /* 卡没完全插入卡座中 */	
    	goto SD_ERASE_END;
	}
	
	if ((startaddr + blocknum) > sds->block_num)	
	{
		ret = SD_ERR_OVER_CARDRANGE;	
		goto SD_ERASE_END;                                                  /* 操作超出卡容量范围 */
	}
		
	if (SdHal_CheckCardWP(sds))
	{
	    ret = SD_ERR_WRITE_PROTECT;                                         /* 卡有写保护 */
		goto SD_ERASE_END;						
	}	

#if SDBUS_MODE_EN
	ret = SdCmd_Select_Card(sds);					                        /* CMD7: 进入传输状态  */
    if (ret != SD_NO_ERR)
    	goto SD_ERASE_END;
#endif

	tmp = blocknum - sds->erase_unit;	
	while(tmp >= 0)												            /* 每次擦除sds->erase_unit 块 */
	{	
		ret = SdCmd_Erase_Wr_Blk_Start(sds, startaddr);			            /* 选择起始块地址 */
		if (ret != SD_NO_ERR)
            goto SD_ERASE_END;
		
		ret = SdCmd_Erase_Wr_Blk_End(sds, startaddr + sds->erase_unit - 1);	/* 选择终止块地址 */
		if (ret != SD_NO_ERR)
	        goto SD_ERASE_END;
			
		ret = SdCmd_Erase(sds);							                    /* 擦除所选择的块 */
		if (ret != SD_NO_ERR)
            goto SD_ERASE_END;

		startaddr += sds->erase_unit;								        /* 起始地址递增 */
		blocknum  -= sds->erase_unit;
		tmp = blocknum - sds->erase_unit;
	};		
	
	if (blocknum > 0)											            /* 擦除不够sds->erase_unit的块 */
	{
		ret = SdCmd_Erase_Wr_Blk_Start(sds, startaddr);
		if (ret != SD_NO_ERR)
            goto SD_ERASE_END;
		
		ret = SdCmd_Erase_Wr_Blk_End(sds, startaddr + blocknum - 1);
		if (ret != SD_NO_ERR)
            goto SD_ERASE_END;
			
		ret = SdCmd_Erase(sds);
		if (ret != SD_NO_ERR)
            goto SD_ERASE_END;
	}

SD_ERASE_END:
#if SDBUS_MODE_EN
    SdCmd_Deselect_Card(sds, 0);                                      /* CMD7: 退出传输状态 */  
#endif
	SD_ReleaseOSSem(sds);
	return ret;											                    /* 返回擦除结果 */
}
#endif


/*******************************************************************************************************************
** 函数名称: SD_RequestOSSem()				
**
** 功能描述: 向操作系统申请访问卡的信号量
**
** 输　  入: sd_struct *sds  : SD/MMC卡信息结构体 
**	         
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
********************************************************************************************************************/
void SD_RequestOSSem(sd_struct *sds)
{
#if SD_UCOSII_EN
	INT8U ret;
	
	OSSemPend(sds->pSemSD, 0, &ret);				/* 等待访问卡信号量可用  */
#endif
}


/*******************************************************************************************************************
** 函数名称: SD_ReleaseOSSem()				
**
** 功能描述: 释放访问卡的信号量
**
** 输　  入: sd_struct *sds  : SD/MMC卡信息结构体 
**	         
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
********************************************************************************************************************/
void SD_ReleaseOSSem(sd_struct *sds)
{
#if SD_UCOSII_EN
	OSSemPost(sds->pSemSD);							/* 将访问卡信号量还给操作系统 */
#endif
}


/*******************************************************************************************************************
** 函数名称: SD_GetZLGSDVer()				
**
** 功能描述: 得到ZLG/SD软件包的版本号	
**
** 输　  入: 无 
**	         
** 输 　 出: 无	
**
** 返 回 值: 版本号  Version
**
** 版本记录: V1.0   支持访问SD 卡,只能在前后台系统运行
			 
			 V2.0   支持访问SD和MMC卡,可运行于前后系统或UCOS-II
			 
			 V3.0   支持并整合了 SD/MMC 卡的SD模式和SPI模式, 软件硬件分层更加清晰
********************************************************************************************************************/
INT16U SD_GetZLGSDVer(void)
{
	return 0x0300;						/* 版本号为 3.00 */
}




/*
*****************************************************************************************
   本软件包的 SPI 总线模式, 适用于LPC21xx, LPC22xx, LPC2364/6 等带用SPI控制器的微控制器
*****************************************************************************************
*/

#if (!SDBUS_MODE_EN)

INT8U SdSpi_ActiveInit(sd_struct *sds);
			
/*******************************************************************************************************************
** 函数名称: SD_Initialize()				
**
** 功能描述: SD/MMC 卡初始化: 复位卡,识别卡,获取卡相关信息,使卡进入数据传输模式 
**
** 输　  入: sd_struct *sds: SD/MMC卡信息结构体 
**	         
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
********************************************************************************************************************/			
INT8U SD_Initialize(sd_struct *sds)
{
	INT32U ocr;
	INT8U ret;
	
#if SD_UCOSII_EN
	if (sds->pSemSD == NULL)
	{
    	sds->pSemSD = OSSemCreate(1);				/* 创建访问SD/MMC卡信号量 create SD/MMC semaphore */ 
		if (sds->pSemSD == NULL)					
		 	return SD_ERR_CREATE_SEMSD;
	}				    
#endif	
    
    SD_RequestOSSem(sds);
   	SdSpiHal_Initialize(sds);						/* 初始化读写SD卡的硬件条件 Initialize the hardware that access SD Card */
    
    if (!SdHal_CheckCard(sds))						/* 检查卡是否插入 check weather card is inserted */
    {
    	ret = SD_ERR_NO_CARD;   
    	goto SD_ERR;
    }
         
    SdSpiHal_CSAssert(sds);							/* 1. 置CS为低 assert CS */  
	SdSpi_Delay(sds, 25);						    /* 2. 至少延时 74 clock delay more than 74 clock */
    SdSpiHal_CSDeassert(sds);						/* 3. 置CS为高 dessert CS */
    SdSpi_Delay(sds, 2);							/* 4. 延时2(8 clock) delay 2(8 clock) */
    
    ret = SpiCmd_Go_Idle_State(sds);				/* 5. 发出CMDO命令复位SD卡 send CMD0 command to reset sd card */
    if (ret != SD_NO_ERR)
        goto SD_ERR;									
 	
 	ret = SdSpi_ActiveInit(sds);					/* 6. 激活卡进入初始化过程. active card initialize process */
 	if (ret != SD_NO_ERR)
 		goto SD_ERR;
        
   	ret = SpiCmd_ReadOCR(sds, &ocr);  				/* 7. 读OCR寄存器,查询卡支持的电压值 read OCR register,get the supported voltage */
    if (ret != SD_NO_ERR)
        goto SD_ERR;
    
    if ((ocr & MSK_OCR_33) != MSK_OCR_33)
    { 
        ret = SD_ERR_VOL_NOTSUSP;					/* 不支持3.3V,返回错误码  not support 3.3V,return error code */
    	goto SD_ERR;
    }
    
    SdSpiHal_SetMCIClock(sds, SD_RATE_NORMAL);  	/* 8. 设置访问卡的clock为标准clock */
        
#if SD_CRC_EN        
    ret = SpiCmd_CRC_ON_OFF(sds, 1);				/* 使能CRC校验 enable CRC check */
#else
    ret = SpiCmd_CRC_ON_OFF(sds, 0);				/* 禁止CRC校验 disable CRC check */
#endif   
	if (ret != SD_NO_ERR)  
	  	goto SD_ERR;

    ret = SpiCmd_Set_BlockLen(sds, SD_BLOCKSIZE);	/* 9. 设置块的长度: 512Bytes Set the block length: 512Bytes */
    if (ret != SD_NO_ERR)  
        goto SD_ERR;
        
    ret = SDMMC_GetCardInfo(sds);					/* 10. 读CSD寄存器,获取SD卡信息 read CSD register, get the information of SD card */    
	if (ret != SD_NO_ERR)
		goto SD_ERR;

	ret = SDMMC_CalTimeout(sds);					/* 计算超时时间值 */
	if (ret != SD_NO_ERR)
	    goto SD_ERR;

	SD_ReleaseOSSem(sds);	
	return SD_NO_ERR;								/* 初始化成功 initialize sucessfully */

SD_ERR:	
	SD_ReleaseOSSem(sds);
	return ret;
}


/*******************************************************************************************************************
** 函数名称: SD_ReadBlock()				
**
** 功能描述: SPI模式下, 从SD/MMC卡中读出一个数据块
**
** 输　  入: sd_struct *sds  : SD/MMC卡信息结构体 
**           INT32U blockaddr: 以块为单位的块地址, 例如, 卡开始的0 ~ 511字节为块地址0, 512 ~ 1023字节的块地址为1
**	         
** 输 　 出: INT8U *recbuf   : 接收缓冲区,长度固定为 512 字节	
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
********************************************************************************************************************/
INT8U SD_ReadBlock(sd_struct *sds, INT32U blockaddr, INT8U *recbuf)
{
	INT8U ret; 
	
	SD_RequestOSSem(sds);							    /* 向OS申请访问卡信号量 request semaphore acessed SD/MMC to OS */
    if (!SdHal_CheckCard(sds))							
    {
    	SD_ReleaseOSSem(sds);	    
    	return SD_ERR_NO_CARD;   					    /* 卡没完全插入卡中 card is not inserted entirely */
	}
		
	if (blockaddr > sds->block_num)	
	{
		SD_ReleaseOSSem(sds);	
		return SD_ERR_OVER_CARDRANGE;				    /* 操作超出卡容量范围 operate over the card range */
	}
	
 	ret = SpiCmd_Read_Single_Block(sds, blockaddr);		/* 读单块命令 read single blocks command */						
 	if (ret != SD_NO_ERR)
 	{
 		SD_ReleaseOSSem(sds);									
 		return ret;
 	}
 	
  	ret = SdSpi_ReadBlockData(sds, SD_BLOCKSIZE, recbuf); /* 读出数据 read data from sd card */	
	SD_ReleaseOSSem(sds);								  /* 归还访问卡信号量 return semaphore acessed SD/MMC to OS */
	
	return ret;
}


/*******************************************************************************************************************
** 函数名称: SD_ReadMultiBlock()				
**
** 功能描述: SPI模式下, 从SD/MMC卡中读出多个数据块
**
** 输　  入: sd_struct *sds  : SD/MMC卡信息结构体 
**           INT32U blockaddr: 以块为单位的块地址
**	         INT32U blocknum : 要读取的块的个数
**
** 输 　 出: INT8U *recbuf   : 接收缓冲区, 长度512 * blocknum 字节	
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
********************************************************************************************************************/
#if SD_ReadMultiBlock_EN
INT8U SD_ReadMultiBlock(sd_struct *sds, INT32U blockaddr, INT32U blocknum, INT8U *recbuf)
{
    INT32U i;
    INT8U ret;
       
	SD_RequestOSSem(sds);							    /* 向OS申请访问卡信号量 request semaphore acessed SD/MMC to OS */
    if (!SdHal_CheckCard(sds))							
    {
    	SD_ReleaseOSSem(sds);	
    	return SD_ERR_NO_CARD;   					    /* 卡没完全插入卡中 card is not inserted entirely */
	}
		
	if ((blockaddr + blocknum) > sds->block_num)	
	{
		SD_ReleaseOSSem(sds);	
		return SD_ERR_OVER_CARDRANGE;				    /* 操作超出卡容量范围 operate over the card range */
	}       
    
	ret = SpiCmd_Read_Multiple_Block(sds, blockaddr);   /* 读多块命令 read multiple blocks command */
	if (ret != SD_NO_ERR)
	{	
		SD_ReleaseOSSem(sds);						    /* 归还访问卡信号量 return semaphore acessed SD/MMC to OS */						
		return ret;
    }
    
    for (i = 0; i < blocknum; i++)
    {												    /* 读出数据 read data from SD/MMC card */
    	ret = SdSpi_ReadBlockData(sds, SD_BLOCKSIZE, recbuf);
    	if (ret == SD_NO_ERR)
       		recbuf = recbuf + SD_BLOCKSIZE;
    	else
    	{
    		SD_ReleaseOSSem(sds);
    		return ret;
    	}
    }
 	 
 	ret = SpiCmd_Stop_Transmission(sds);			    /* 结束数据传输 stop transmission operation */ 
 	
 	SD_ReleaseOSSem(sds);
	return ret; 
}

#endif


/*******************************************************************************************************************
** 函数名称: SD_WriteBlock()				
**
** 功能描述: SPI模式下, 向SD/MMC卡中写入一个块	
**
** 输　  入: sd_struct *sds  : SD/MMC卡信息结构体 
**           INT32U blockaddr: 以块为单位的块地址, 例如, 卡开始的0 ~ 511字节为块地址0, 512 ~ 1023字节的块地址为1
**           INT8U *sendbuf  : 发送缓冲区,长度固定为 512 字节	
**	         
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
********************************************************************************************************************/
INT8U SD_WriteBlock(sd_struct *sds, INT32U blockaddr, INT8U *sendbuf)
{
	INT8U ret,tmp[2];
	
	SD_RequestOSSem(sds);										/* 向OS申请访问卡信号量 request semaphore acessed SD/MMC to OS */
    if (!SdHal_CheckCard(sds))						
    {
    	SD_ReleaseOSSem(sds);	
    	return SD_ERR_NO_CARD;   								/* 卡没完全插入卡中 card is not inserted entirely */
	}	
	
	if (blockaddr > sds->block_num)	
	{
		SD_ReleaseOSSem(sds);	
		return SD_ERR_OVER_CARDRANGE;							/* 操作超出卡容量范围 operate over the card range */
	}
	
	if (SdHal_CheckCardWP(sds))
	{
		SD_ReleaseOSSem(sds);
		return SD_ERR_WRITE_PROTECT;							/* 卡有写保护 */
	}
	
	ret = SpiCmd_Write_Single_Block(sds, blockaddr);			/* 写单块命令 write single block */
	if (ret != SD_NO_ERR)
	{	
		SD_ReleaseOSSem(sds);
		return ret;
	}
	
	ret = SdSpi_WriteBlockData(sds, 0, SD_BLOCKSIZE, sendbuf);	/* 写入数据 write data */
 	if (ret == SD_NO_ERR)										/* 读Card Status寄存器, 检查写入是否成功 */
 	{															/* read Card Status register to check write wheather sucessfully */
 		ret = SpiCmd_Send_Status(sds, 2, tmp);
 		if (ret != SD_NO_ERR)
 		{
 			SD_ReleaseOSSem(sds);
 			return ret;											/* 读寄存器失败 read register fail */
		}

 		if((tmp[0] != 0) || (tmp[1] != 0))
 		{
 			SD_ReleaseOSSem(sds);
			ret = SD_ERR_WRITE_BLK; 			     			/* 响应指示写失败 response indicate write fail */
 		}
 	}
  
    SD_ReleaseOSSem(sds);
 	return ret;													/* 返回写入结果 return the result of writing */									
}


/*******************************************************************************************************************
** 函数名称: SD_WriteMultiBlock()				
**
** 功能描述: SPI模式下, 向SD/MMC卡中写入多个数据块	
**
** 输　  入: sd_struct *sds  : SD/MMC卡信息结构体 
**           INT32U blockaddr: 以块为单位的块地址
**           INT32U blocknum : 要写入的块的个数
**           INT8U *sendbuf  : 发送缓冲区, 长度 512 * blocknum 字节	
**	         
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
********************************************************************************************************************/
#if SD_WriteMultiBlock_EN
INT8U SD_WriteMultiBlock(sd_struct *sds, INT32U blockaddr, INT32U blocknum, INT8U *sendbuf)
{
	INT32U i;
	INT8U ret,tmp[2];
	
	SD_RequestOSSem(sds);										/* 向OS申请访问卡信号量 request semaphore acessed SD/MMC to OS */
    if (!SdHal_CheckCard(sds))							
    {
    	SD_ReleaseOSSem(sds);	
    	return SD_ERR_NO_CARD;   								/* 卡没完全插入卡中 card is not inserted entirely */
	}
		
	if ((blockaddr + blocknum) > sds->block_num)	
	{
		SD_ReleaseOSSem(sds);	
		return SD_ERR_OVER_CARDRANGE;							/* 操作超出卡容量范围 operate over the card range */
	}
		
	if (SdHal_CheckCardWP(sds))
	{
		SD_ReleaseOSSem(sds);
		return SD_ERR_WRITE_PROTECT;							/* 卡有写保护 */
	}
	
	ret = SpiCmd_Write_Multiple_Block(sds, blockaddr);			/* 写多块命令 write multiple blocks command */
	if (ret != SD_NO_ERR)
	{
		SD_ReleaseOSSem(sds);
		return ret;
	}
	
    for (i = 0; i < blocknum; i++)
    {															
 		ret = SdSpi_WriteBlockData(sds, 1, SD_BLOCKSIZE, sendbuf);	/* 写入数据 write data */
 		if (ret == SD_NO_ERR)	
 			sendbuf = sendbuf + SD_BLOCKSIZE;
 		else
 		{														/* 写失败 write fail */
			SpiCmd_Stop_Transmission(sds);						/* 停止数据传输 stop data transmission */	
			SdSpi_WaitBusy(sds, SD_WAIT_WRITE);					/* 等待 waiting */
			SD_ReleaseOSSem(sds);
			return ret;
		}
	}
   
    SdSpi_StopMultiToken(sds);									/* 发送数据停止令牌 send data stop token */
      
    ret = SdSpi_WaitBusy(sds, SD_WAIT_WRITE);					/* 等待写入的完成 wait for finishing writing */
    if (ret != SD_NO_ERR)
    {
    	SD_ReleaseOSSem(sds);
    	return SD_ERR_TIMEOUT_WRITE;
    }
    
    if (sds->card_type == CARDTYPE_SD)
    {
   		ret = SpiCmd_Send_Num_WR_Blcoks(sds, &i);				/* 读正确写入的块数 read the blocks that be written correctly */
   		if (ret != SD_NO_ERR)
   		{
   			SD_ReleaseOSSem(sds);
   		  	return ret;
   		}
   		if(i != blocknum)
			ret =  SD_ERR_WRITE_BLKNUMS;						/* 正确写入块数错误 the blocks that be written correctly is error */
   	}
   	else
   	{
   	 	ret = SpiCmd_Send_Status(sds, 2, tmp);
 		if (ret != SD_NO_ERR)
 		{
 			SD_ReleaseOSSem(sds);
 			return ret;											/* 读寄存器失败 read register fail */
		}
 		if((tmp[0] != 0) || (tmp[1] != 0))
			ret = SD_ERR_WRITE_BLK; 			     			/* 响应指示写失败 response indicate write fail */
   	}
   	 	
   	SD_ReleaseOSSem(sds);   	     
	return ret;													/* 返回写入成功 return write sucessfully */			
}
#endif


	/*****************************************************************
	
	    		下面为子程序
	 
	*****************************************************************/

/*******************************************************************************************************************
** 函数名称: SdSpi_ActiveInit()				
**
** 功能描述: SPI模式下, 激活卡,并获得卡型	
**
** 输　  入: sd_struct *sds  : SD/MMC卡信息结构体 
**	         
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
**
** 函数说明: SpiCmd_Send_OP_Cond()不断重复发送CMD0到卡，直到响应R1的Bit0(Idle)位为0，表示卡内部初始化处理完成。
		     当响应的Idle位为0时，SD卡就完全进入SPI模式了。当然重复发送命令CMD1是有次数限制的，
		     最大次数为宏定义SD_IDLE_WAIT_MAX.
********************************************************************************************************************/
INT8U SdSpi_ActiveInit(sd_struct *sds)
{
	INT8U param[4] = {0,0,0,0},resp[5],ret;
	
    ret = SpiCmd_Send_OP_Cond(sds);                         /* 激活卡进入初始化过程, 使卡退出Idle状态 */
    if (ret != SD_NO_ERR)
        return ret;    
    	
    ret = SdSpi_SendCmd(sds, CMD55, param, CMD55_R, resp);
    if (ret != SD_NO_ERR)
       	return ret;
       	
    ret = SdSpi_SendCmd(sds, ACMD41, param, ACMD41_R, resp);/* 激活内部初始化命令 active card to initialize process internal */
    if (ret != SD_NO_ERR)	
    	return SD_ERR_UNKNOWN_CARD;
    
    if ((resp[0] & 0xFE) == 0)
      	sds->card_type = CARDTYPE_SD;						/* 是SD卡 the card is SD card */	
    else	
    	sds->card_type = CARDTYPE_MMC;						/* 是MMC卡 the card is MMC card */

	return SD_NO_ERR;
}

#endif  //end of #if (!SDBUS_MODE_EN)

















