/****************************************Copyright (c)**************************************************
**                               Guangzhou ZLG-MCU Development Co.,LTD.
**                                      graduate school
**                                 http://www.zlgmcu.com
**
**--------------File Info-------------------------------------------------------------------------------
** File name:			sdcmd.c
** Last modified Date:	2007-5-19
** Last Version:		V2.0
** Descriptions:		SD/MMC 软件包: SD/MMC卡SPI模式和SD模式支持的命令 
**						
**
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

#include "sdconfig.h"


			/*********************************************

			 本文件实现 SD/MMC 卡 SD模式下的各 CMD 和 ACMD

			**********************************************/

#if SDBUS_MODE_EN

static void word_to_byte(INT32U value, INT8U *pdst);

#define SEND_IF_COND            0x000001AA
#define OCR_WITHOUT_BUSY		0x00FF8000
#define OCR_WITH_BUSY			0x80000000


/*************
    CMD 0 
*************/
/*******************************************************************************************************************
** 函数名称: SdCmd_Go_Idle_State()				
**
** 功能描述: CMD0, 复位所有卡, 使卡进入Idle状态		
**
** 输　  入: sd_struct *sds  : SD/MMC卡信息结构体 
**	
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
********************************************************************************************************************/
INT8U SdCmd_Go_Idle_State(sd_struct *sds)
{
	return (SdHal_SendCmd(sds, CMD0, 0, CMD0_R, 0, 0));			/* 发送 CMD0 并取得响应 */
}


/*************
    CMD 1 
*************/
/*******************************************************************************************************************
** 函数名称: SdCmd_Send_OP_Cond()				
**
** 功能描述: CMD1(仅用于MMC卡), 使卡退出Idle状态, 进入Ready状态(通过判断MMC卡的OCR寄存器值)		
**
** 输　  入: sd_struct *sds  : SD/MMC卡信息结构体 
**	
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
********************************************************************************************************************/
INT8U SdCmd_Send_OP_Cond(sd_struct *sds)
{
	INT32U OCRVal;
	INT8U ret;
	INT32U count = 0;

 	do 
    {	
		ret = SdHal_SendCmd(sds, CMD1, OCR_WITHOUT_BUSY, CMD1_R, &OCRVal, 1);	/* 发送 CMD1 并取得响应 */
		if (ret != SD_NO_ERR)
			return ret;

        if (count >= SD_IDLE_WAIT_MAX)
            return SD_ERR_TIMEOUT_WAITIDLE;									    /* 超时,返回错误码 */
     	
      	count ++;

    }while ((OCRVal & OCR_WITH_BUSY) == 0x00000000);
	
    if ((OCRVal & MSK_OCR_33) != MSK_OCR_33)
        return SD_ERR_VOL_NOTSUSP;											    /* 卡不支3.2V ~ 3.3V电压 */			
	
	return SD_NO_ERR;
}

/*************
    CMD 8
*************/
/*********************************************************************************************************
** 函数名称: SD_CheckSDVersion()
**
** 功能描述: 识别卡版本
**
** 输　  入: sd_struct *sds: SD/MMC卡信息结构体
**
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
**
** 注    意: 执行后, 卡型保存在sds->card_type中
*********************************************************************************************************/
INT8U SD_CheckSDVersion(sd_struct *sds)
{
    INT8U  ret;
    INT32U resp[4];

    ret = SdHal_SendCmd(sds, CMD8, SEND_IF_COND, CMD8_R, resp, 1);
    
    
    if (ret != SD_NO_ERR) {
        return ret;                                                   /* 卡型为version 1              */
    } 
    if ((resp[0] & SEND_IF_COND) == SEND_IF_COND)
        return (0x01);                                                   /* 卡型为version 1              */

    else
        return (0x00);                                                   /* 卡型为version 2              */
   
}
/*************
    CMD 55 
*************/
/*******************************************************************************************************************
** 函数名称: SdCmd_Send_ACMD()				
**
** 功能描述: CMD55(仅用于SD卡), 在发送所有 ACMDxx 命令之前, 必须先发送本命令		
**
** 输　  入: sd_struct *sds  : SD/MMC卡信息结构体 
**	
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
**
** 注	 意: 在未使用 Set_Relative_Addr 命令取得卡的RCA地址之前, sds->RCA为0.
********************************************************************************************************************/
INT8U SdCmd_Send_ACMD(sd_struct *sds)
{
	INT32U argument;	
	INT32U devstatus = 0;
	INT8U ret;

	if (sds->card_type == CARDTYPE_SD)									/* 根据卡型取不同的参数 */
		argument = sds->RCA;											
	else
		argument = 0x00000000;	
				  
	ret = SdHal_SendCmd(sds, CMD55, argument, CMD55_R, &devstatus, 1);	/* 发送 CMD55 并取得响应 */	
	if (ret != SD_NO_ERR)
		return ret;
		
	if (devstatus & CARD_STATUS_ACMD_ENABLE)						
		return SD_NO_ERR;											    /* 支持 CMD55 命令 */
		
	return SD_ERR_CMD_NOTSUSP;										    /* 不支持该命令 */
}


/*************
    ACMD 41 
*************/
/*******************************************************************************************************************
** 函数名称: SdCmd_SD_App_OP_Cond()				
**
** 功能描述: ACMD41(仅用于SD卡), 使卡退出Idle状态, 并进入Ready状态(通过判断SD卡的OCR寄存器值)		
**
** 输　  入: sd_struct *sds  : SD/MMC卡信息结构体
**           
**	
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
**
** 注	 意: 在未使用 Set_Relative_Addr 命令取得卡的RCA地址之前, sds->RCA为0.
********************************************************************************************************************/
INT8U SdCmd_SD_App_OP_Cond(sd_struct *sds)
{
	INT8U ret;
	INT32U OCRVal, count = 0;

 	do 
    {	
       	ret = SdCmd_Send_ACMD(sds);                                               /* 先发送 CMD55 命令 */
		if (ret != SD_NO_ERR)
			return ret;
           
		ret = SdHal_SendCmd(sds, ACMD41, OCR_WITHOUT_BUSY, ACMD41_R, &OCRVal, 1); /* 发送 ACMD41 并取得响应 */
		if (ret != SD_NO_ERR)
			return ret;

        if (count >= SD_IDLE_WAIT_MAX)
            return SD_ERR_TIMEOUT_WAITIDLE;									      /* 超时,返回错误码 */
     	
      	count ++;

    }while ((OCRVal & OCR_WITH_BUSY) == 0x00000000);
	
    if ((OCRVal & MSK_OCR_33) != MSK_OCR_33)
        return SD_ERR_VOL_NOTSUSP;											      /* 卡不支3.2V ~ 3.3V电压 */			
	
	return SD_NO_ERR;	
}


/*************
    CMD 2
*************/
/*******************************************************************************************************************
** 函数名称: SdCmd_All_Send_CID()				
**
** 功能描述: 获取所有卡的CID寄存器内容	
**
** 输　  入: sd_struct *sds  : SD/MMC卡信息结构体
**           INT8U cidlen    : CID 寄存器长度, 固定为16字节
**	
** 输 　 出: INT8U *cidbuf   : CID 寄存器内容, 长度为16字节
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
**
** 注	 意: 该命令的Argument域不必关心
********************************************************************************************************************/
INT8U SdCmd_All_Send_CID(sd_struct *sds, INT8U cidlen, INT8U *cidbuf)
{
	INT8U ret;
	INT32U resp[4];
	
	if ((cidlen < 16) || (cidbuf == 0))
	    return SD_ERR_USER_PARAM;
		
	ret = SdHal_SendCmd(sds, CMD2, 0, CMD2_R, resp, 4);
	if (ret != SD_NO_ERR)
		return ret;	
	
	word_to_byte(*resp, cidbuf);                            /* 从响应内容中取得CID寄存器内容 */
	word_to_byte(*(resp + 1), cidbuf + 4);
	word_to_byte(*(resp + 2), cidbuf + 8);
	word_to_byte(*(resp + 3), cidbuf + 12);
			
	return SD_NO_ERR;
}


/*****************************
    CMD 3: 设置卡的地址 RCA 
******************************/
/*******************************************************************************************************************
** 函数名称: SdCmd_Set_Relative_Addr()				
**
** 功能描述: CMD3, 获取卡的RCA地址, 然后保存到sds结构体中		
**
** 输　  入: sd_struct *sds  : SD/MMC卡信息结构体
**	
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
**
** 注	 意: 该函数将给 sds->RCA 成员赋值
********************************************************************************************************************/
INT8U SdCmd_Set_Relative_Addr(sd_struct *sds)
{	
	INT32U argument;
    INT32U resp;
    INT8U ret;

    /* 如果是一张SD卡, 使用SET_RELATIVE_ADDR用于从卡获取地址并在RCA中使用该值,
       如果是一张MMC卡, 使用默认的RCA地址 0x00010000. */
    if (sds->card_type == CARDTYPE_SD || sds->card_type == CARDTYPE_SDHC)
    {
        argument = 0;
    }
    else
    {                                                                   /* 如果是未知的或MMC卡, 固定RCA */
                                                                        /* 地址                         */
        argument = 0x00010000;
    }

    ret = SdHal_SendCmd(sds, CMD3, argument, CMD3_R, &resp, 1);
    if (ret != 0)
    return ret;

    if (sds->card_type == CARDTYPE_SD || sds->card_type == CARDTYPE_SDHC)
    {
                                                                        /* 卡应处于 IDENT 状态          */
        if ((resp & CARD_STATUS_CURRENT_STATE) == CARD_STATUS_IDENT)
        sds->RCA = resp & 0xFFFF0000;
        else
        return SD_ERR_CURRENT_STATE;
    }
    else
    sds->RCA = argument;

    return SD_NO_ERR;
}


/********************************************
    CMD 9, 仅在CMD3发送后才可发送, 获取 CSD 
*********************************************/
/*******************************************************************************************************************
** 函数名称: SdCmd_Send_CSD()				
**
** 功能描述: CMD9, 获取卡的CSD寄存器值		
**
** 输　  入: sd_struct *sds  : SD/MMC卡信息结构体
**           INT8U csdlen    : CSD 寄存器长度
**	
** 输 　 出: INT8U *csdbuf   : CSD寄存器内容, bit127 在csdbuf[0] 
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
**
** 注	 意: 该命令需要 sds->RCA 作为命令参数
********************************************************************************************************************/
INT8U SdCmd_Send_CSD(sd_struct *sds, INT8U csdlen, INT8U *csdbuf)
{	
	INT8U ret;
	INT32U resp[4];
	
	if ((csdlen < 16) || (csdbuf == 0))
	    return SD_ERR_USER_PARAM;	
	
	ret = SdHal_SendCmd(sds, CMD9, sds->RCA, CMD9_R, resp, 4);
	if (ret != SD_NO_ERR)
		return ret;
		
	word_to_byte(*resp, csdbuf);                            /* 从响应内容中取得CSD寄存器内容 */
	word_to_byte(*(resp + 1), csdbuf + 4);
	word_to_byte(*(resp + 2), csdbuf + 8);
	word_to_byte(*(resp + 3), csdbuf + 12);
	
	return SD_NO_ERR;
}


/**********************************************************
    CMD 7: 选择卡, 使卡在 Stand-by 与 Transfer 之间切换
***********************************************************/
/*******************************************************************************************************************
** 函数名称: SdCmd_Select_Card()				
**
** 功能描述: CMD7, 使卡在 Stand-by 与 Transfer 之间切换	
**
** 输　  入: sd_struct *sds  : SD/MMC卡信息结构体
**	        
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
**
** 注	 意: 该命令需要 sds->RCA 作为命令参数
********************************************************************************************************************/
INT8U SdCmd_Select_Card(sd_struct *sds)
{
	INT32U resp;
	INT8U ret;

	ret = SdHal_SendCmd(sds, CMD7, sds->RCA, CMD7_R, &resp, 1);
	if (ret != SD_NO_ERR)
		return ret;
	
	return SD_NO_ERR;
}


/**********************************************************
    CMD 7: 取消选择卡, 使卡在 Stand-by 与 Transfer 之间切换
***********************************************************/
/*******************************************************************************************************************
** 函数名称: SdCmd_Select_Card()				
**
** 功能描述: CMD7, 使卡在 Stand-by 与 Transfer 之间切换	
**
** 输　  入: sd_struct *sds  : SD/MMC卡信息结构体
**	         INT32U RCA      : 卡的RCA地址
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
**
** 注	 意: 该命令的命令参数为RCA, 取值为0
********************************************************************************************************************/
INT8U SdCmd_Deselect_Card(sd_struct *sds, INT32U RCA)
{
	INT32U resp;
	INT8U ret;

	ret = SdHal_SendCmd(sds, CMD7, RCA, CMD0_R, &resp, 1);
	if (ret != SD_NO_ERR)
		return ret;
	
	return SD_NO_ERR;
}


/***********************************
  CMD12: 停止传输Stop Transmission
************************************/
/*******************************************************************************************************************
** 函数名称: SdCmd_Send_Stop()				
**
** 功能描述: CMD12, 停止传输
**
** 输　  入: sd_struct *sds  : SD/MMC卡信息结构体
**	
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
**
** 注	 意: 该命令的argument域不必关心, 当主机调用该函数停止发送多块数据到卡时, 还需等待CARD_Status寄存器的
             READY_FOR_DATA位置1
********************************************************************************************************************/
INT8U SdCmd_Stop_Transmission(sd_struct *sds)
{
	INT8U ret;
	INT32U resp;
	
	ret = SdHal_SendCmd(sds, CMD12, 0, CMD12_R, &resp, 1);
	if (ret != SD_NO_ERR)
		return ret;
		
	return SD_NO_ERR;
}


/******************************
    CMD 13: 读取卡当前状态
*******************************/
/*******************************************************************************************************************
** 函数名称: SdCmd_Send_Status()				
**
** 功能描述: CMD13, 读取卡当前状态, 当卡进入数据传输模式后(即从识别模式进入该模式后), 就可以使用本函数		
**
** 输　  入: sd_struct *sds  : SD/MMC卡信息结构体
**	
** 输 　 出: INT32U *status  : 卡状态
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
**
** 注	 意: 该命令需要 sds->RCA 作为命令参数
********************************************************************************************************************/
INT8U SdCmd_Send_Status(sd_struct *sds, INT32U *status)
{
	INT32U resp;
	INT8U ret;
	
	ret = SdHal_SendCmd(sds, CMD13, sds->RCA, CMD13_R, &resp, 1);
	if (ret != SD_NO_ERR)
		return ret;
	
	*status = resp;
	return SD_NO_ERR;
}

		
/*********************************************
    CMD 15: 使卡进入非活动状态
*********************************************/
/*******************************************************************************************************************
** 函数名称: SdCmd_Go_Inactive_State()				
**
** 功能描述: CMD15, 使卡进入非活动状态	
**
** 输　  入: sd_struct *sds  : SD/MMC卡信息结构体
**	
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
**
** 注	 意: 该命令需要 sds->RCA 作为命令参数
********************************************************************************************************************/
INT8U SdCmd_Go_Inactive_State(sd_struct *sds)
{
	INT32U resp;
	INT8U ret;

	ret = SdHal_SendCmd(sds, CMD15, sds->RCA, CMD15_R, &resp, 1);
	if (ret != SD_NO_ERR)
		return ret;
	
	return SD_NO_ERR;	
}


/*********************************************
    CMD 16: 设置块长度, 必须发送CMD7后发送
*********************************************/
/*******************************************************************************************************************
** 函数名称: SdCmd_Set_BlockLen()				
**
** 功能描述: CMD16, 设置卡的块长度	
**
** 输　  入: sd_struct *sds  : SD/MMC卡信息结构体
**           INT32U blockLen : 卡块的长度
**	
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
********************************************************************************************************************/
INT8U SdCmd_Set_BlockLen(sd_struct *sds, INT32U blockLen)
{
	INT32U resp;
	INT8U ret;

	ret = SdHal_SendCmd(sds, CMD16, blockLen, CMD16_R, &resp, 1);
	if (ret != SD_NO_ERR)
		return ret;
		
	/* bit 9 through 12 should be in transfer state now. bit 8 is ready. */
	if ((resp & CARD_STATUS_CURRENT_STATE) != CARD_STATUS_TRAN)	 
	    return SD_ERR_CURRENT_STATE;
	 
	if (!(resp & CARD_STATUS_RDY_DATA))	     
	    return SD_ERR_READY_FOR_DATA;
	
	return SD_NO_ERR;	
}


/**************************************
	ACMD6: 设置总线宽度, 只适合于SD卡
***************************************/
/*******************************************************************************************************************
** 函数名称: SdCmd_Set_Bus_Width()				
**
** 功能描述: ACMD6, 设置卡传输数据的总线宽度(对于SD卡, 可以为1bit, 也可以为4bits)
**			 对于MMC卡, 本命令无效			
**
** 输　  入: sd_struct *sds:  SD/MMC卡信息结构体
**           INT8U bWideBus:  是否使能宽总线(4bit data bus).
**							  1:  使能       0: 禁止
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
********************************************************************************************************************/
INT8U SdCmd_Set_Bus_Width(sd_struct *sds, INT8U bWideBus)
{
	INT32U argument;
	INT32U resp;
	INT8U ret;
	
	if (bWideBus > 0)
		argument = 0x02;		// '02' = 4bits data bus
	else
		argument = 0x0;			// '00'	= 1bit data bus

    ret = SdCmd_Send_ACMD(sds);
	if (ret != SD_NO_ERR)
		return ret;

	ret = SdHal_SendCmd(sds, ACMD6, argument, ACMD6_R, &resp, 1);
	if (ret != SD_NO_ERR)
		return ret;
	
	/* bit 9 through 12 should be in transfer state now. bit 8 is ready. */
	if ((resp & CARD_STATUS_CURRENT_STATE) != CARD_STATUS_TRAN)	 
	    return SD_ERR_CURRENT_STATE;
	 
	if (!(resp & CARD_STATUS_RDY_DATA))	     
	    return SD_ERR_READY_FOR_DATA;
	
	return SD_NO_ERR;		
}


/***********************************
  CMD17: 发送读单块命令
************************************/
/*******************************************************************************************************************
** 函数名称: SdCmd_Read_Single_Block()				
**
** 功能描述: CMD17, 发送读单块命令
**
** 输　  入: sd_struct *sds:  SD/MMC卡信息结构体
**           INT32U blockadddr:  块地址. 0 ~ 511字节取值为1. 512 ~ 1023字节取值为2
**	
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
********************************************************************************************************************/
INT8U SdCmd_Read_Single_Block(sd_struct *sds, INT32U blockadddr)
{
	INT8U ret;
    INT32U resp;
    /*
     * Data address is in byte units in a Standard Capacity SD Memory Card and in block (512 Byte)
     * units in a High Capacity SDMemory Card.
     */
    if (sds->card_type != CARDTYPE_SDHC)
    {
        blockadddr = blockadddr << SD_BLOCKSIZE_NBITS;
    }
    ret = SdHal_SendCmd(sds, CMD17, blockadddr, CMD17_R, &resp, 1);
    if (ret != SD_NO_ERR)
    return ret;

                                                                        /* bit 9 through 12 should be i */
                                                                        /* n transfer                   */
                                                                        /* state now. bit 8 is ready.   */
    if ((resp & CARD_STATUS_CURRENT_STATE) != CARD_STATUS_TRAN)
    return SD_ERR_CURRENT_STATE;

    if (!(resp & CARD_STATUS_RDY_DATA))
    return SD_ERR_READY_FOR_DATA;

    return SD_NO_ERR;
}


/***********************************
  CMD18: 发送读多块命令
************************************/
/*******************************************************************************************************************
** 函数名称: SdCmd_Read_Multiple_Block()				
**
** 功能描述: CMD18, 发送读多块命令
**
** 输　  入: sd_struct *sds:  SD/MMC卡信息结构体
**           INT32U blockadddr:  块地址
**	
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
********************************************************************************************************************/
INT8U SdCmd_Read_Multiple_Block(sd_struct *sds, INT32U blockadddr)
{
	INT8U ret;
    INT32U resp;
    /*
     * Data address is in byte units in a Standard Capacity SD Memory Card and in block (512 Byte)
     * units in a High Capacity SDMemory Card.
     */
    if (sds->card_type != CARDTYPE_SDHC)
    {
        blockadddr = blockadddr << SD_BLOCKSIZE_NBITS;
    }

    ret = SdHal_SendCmd(sds, CMD18, blockadddr, CMD18_R, &resp, 1);
    if (ret != SD_NO_ERR)
    return ret;

                                                                        /* bit 9 through 12 should be i */
                                                                        /* n transfer                   */
                                                                        /* state now. bit 8 is ready.   */
    if ((resp & CARD_STATUS_CURRENT_STATE) != CARD_STATUS_TRAN)
    return SD_ERR_CURRENT_STATE;

    if (!(resp & CARD_STATUS_RDY_DATA))
    return SD_ERR_READY_FOR_DATA;

    return SD_NO_ERR;
}


/***********************************
  CMD24: 发送写单块命令
************************************/
/*******************************************************************************************************************
** 函数名称: SdCmd_Write_Single_Block()				
**
** 功能描述: CMD24, 发送写单块命令
**
** 输　  入: sd_struct *sds:  SD/MMC卡信息结构体
**           INT32U blockadddr:  块地址
**	
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
********************************************************************************************************************/
INT8U SdCmd_Write_Single_Block(sd_struct *sds, INT32U blockadddr)
{
	INT8U ret;
    INT32U resp;
    /*
     * Data address is in byte units in a Standard Capacity SD Memory Card and in block (512 Byte)
     * units in a High Capacity SDMemory Card.
     */
    if (sds->card_type != CARDTYPE_SDHC)
    {
        blockadddr = blockadddr << SD_BLOCKSIZE_NBITS;
    }

    ret = SdHal_SendCmd(sds, CMD24, blockadddr, CMD24_R, &resp, 1);
    if (ret != SD_NO_ERR)
    return ret;

                                                                        /* bit 9 through 12 should be i */
                                                                        /* n transfer                   */
                                                                        /* state now. bit 8 is ready.   */
    if ((resp & CARD_STATUS_CURRENT_STATE) != CARD_STATUS_TRAN)
    return SD_ERR_CURRENT_STATE;

    if (!(resp & CARD_STATUS_RDY_DATA))
    return SD_ERR_READY_FOR_DATA;

    return SD_NO_ERR;	
}


/***********************************
  CMD25: 发送写多块命令
************************************/
/*******************************************************************************************************************
** 函数名称: SdCmd_Write_Multiple_Block()				
**
** 功能描述: CMD25, 发送写多块命令
**
** 输　  入: sd_struct *sds:  SD/MMC卡信息结构体
**           INT32U blockadddr:  块地址
**	
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
********************************************************************************************************************/
INT8U SdCmd_Write_Multiple_Block(sd_struct *sds, INT32U blockadddr)
{
	INT8U ret;
    INT32U resp;

    /*
     * Data address is in byte units in a Standard Capacity SD Memory Card and in block (512 Byte)
     * units in a High Capacity SDMemory Card.
     */
    if (sds->card_type != CARDTYPE_SDHC)
    {
        blockadddr = blockadddr << SD_BLOCKSIZE_NBITS;
    }

    ret = SdHal_SendCmd(sds, CMD25, blockadddr, CMD25_R, &resp, 1);
    if (ret != SD_NO_ERR)
    return ret;

                                                                        /* bit 9 through 12 should be i */
                                                                        /* n transfer                   */
                                                                        /* state now. bit 8 is ready.   */
    if ((resp & CARD_STATUS_CURRENT_STATE) != CARD_STATUS_TRAN)
    return SD_ERR_CURRENT_STATE;

    if (!(resp & CARD_STATUS_RDY_DATA))
    return SD_ERR_READY_FOR_DATA;

    return SD_NO_ERR;
}



/***********************************
  CMD32或CMD35: 设置擦卡起始地址
************************************/
/*******************************************************************************************************************
** 函数名称: SdCmd_Erase_Wr_Blk_Start()				
**
** 功能描述: CMD32或CMD35, 设置擦卡起始地址
**
** 输　  入: sd_struct *sds:  SD/MMC卡信息结构体
**           INT32U blockadddr:  块地址
**	
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
********************************************************************************************************************/
INT8U SdCmd_Erase_Wr_Blk_Start(sd_struct *sds, INT32U blockadddr)
{
	INT8U ret;
    INT32U resp;

    /*
     * Data address is in byte units in a Standard Capacity SD Memory Card and in block (512 Byte)
     * units in a High Capacity SDMemory Card.
     */
    if (sds->card_type != CARDTYPE_SDHC)
    {
        blockadddr = blockadddr << SD_BLOCKSIZE_NBITS;
    }

    if (sds->card_type == CARDTYPE_SD || sds->card_type == CARDTYPE_SDHC)
    ret = SdHal_SendCmd(sds, CMD32, blockadddr, CMD32_R, &resp, 1);
    else
    ret = SdHal_SendCmd(sds, CMD35, blockadddr, CMD35_R, &resp, 1);

    if (ret != SD_NO_ERR)
    return ret;

                                                                        /* bit 9 through 12 should be i */
                                                                        /* n transfer                   */
                                                                        /* state now. bit 8 is ready.   */
    if ((resp & CARD_STATUS_CURRENT_STATE) != CARD_STATUS_TRAN)
    return SD_ERR_CURRENT_STATE;

    if (!(resp & CARD_STATUS_RDY_DATA))
    return SD_ERR_READY_FOR_DATA;

    return SD_NO_ERR;		
}


/***********************************
  CMD33或CMD36: 设置擦卡终止地址
************************************/
/*******************************************************************************************************************
** 函数名称: SdCmd_Erase_Wr_Blk_End()				
**
** 功能描述: CMD33或CMD36, 设置擦卡终止地址
**
** 输　  入: sd_struct *sds:  SD/MMC卡信息结构体
**           INT32U blockadddr:  块地址
**	
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
********************************************************************************************************************/
INT8U SdCmd_Erase_Wr_Blk_End(sd_struct *sds, INT32U blockadddr)
{
	INT8U ret;
    INT32U resp;

    /*
     * Data address is in byte units in a Standard Capacity SD Memory Card and in block (512 Byte)
     * units in a High Capacity SDMemory Card.
     */
    if (sds->card_type != CARDTYPE_SDHC)
    {
        blockadddr = blockadddr << SD_BLOCKSIZE_NBITS;
    }

    if (sds->card_type == CARDTYPE_SD || sds->card_type == CARDTYPE_SDHC)
    ret = SdHal_SendCmd(sds, CMD33, blockadddr, CMD33_R, &resp, 1);
    else
    ret = SdHal_SendCmd(sds, CMD36, blockadddr, CMD36_R, &resp, 1);

    if (ret != SD_NO_ERR)
    return ret;

                                                                        /* bit 9 through 12 should be i */
                                                                        /* n transfer                   */
                                                                        /* state now. bit 8 is ready.   */
    if ((resp & CARD_STATUS_CURRENT_STATE) != CARD_STATUS_TRAN)
    return SD_ERR_CURRENT_STATE;

    if (!(resp & CARD_STATUS_RDY_DATA))
    return SD_ERR_READY_FOR_DATA;

    return SD_NO_ERR;	
}


/***********************************
  CMD38: 发送擦卡命令
************************************/
/*******************************************************************************************************************
** 函数名称: SdCmd_Erase()				
**
** 功能描述: CMD38, 擦除所选择指定的块
**
** 输　  入: sd_struct *sds:  SD/MMC卡信息结构体
**
**	
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
**
** 注    意: 在执行本函数之前, 必须执行SdCmd_Erase_Wr_Blk_Start()和SdCmd_Erase_Wr_Blk_End()
********************************************************************************************************************/
INT8U SdCmd_Erase(sd_struct *sds)
{
	INT8U ret;
	INT32U i,resp;
		
	ret = SdHal_SendCmd(sds, CMD38, 0, CMD38_R, &resp, 1);
	if (ret != SD_NO_ERR)
		return ret;

	for(i=0;i<0x1000;i++);										/* 延时一段时间,以便擦除完成 */
	
    ret = SdHal_WaitBusy(sds, BUSY_TYPE_ERASE);                 /* 等待卡编程结束 */
	return ret;		
}


// 将32位字转换为字节函数
static void word_to_byte(INT32U value, INT8U *pdst)
{
    pdst[0] = (INT8U)(value >> 24);
    pdst[1] = (INT8U)(value >> 16);
    pdst[2] = (INT8U)(value >> 8);
    pdst[3] = (INT8U)(value);
}

#endif



/*
************************************************
	
	 下面为SD/MMC卡的SPI命令

************************************************
*/

#if (!SDBUS_MODE_EN)

/*******************************************************************************************************************
** 函数名称: SdSpi_SendCmd()				
**
** 功能描述: SPI模式下, 发送一个SD/MMC命令, 并获得响应			
**
** 输　  入: sd_struct *sds:  卡信息结构体, 仅使用sds->card_posnum成员用于区分卡座
**
**			 INT8U cmd:       命令索引(命令的[5:0]位)
**
**		     INT32U argument: 命令参数, 长度为4字节
**
**			 INT8U resptype : 响应类型 
**							  	
** 输 　 出: INT8U *resp	: 响应内容, 长度为1-5字节, 响应为R3时, R1 在 resp[4], OCR内容的bit31在 resp[3].
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
********************************************************************************************************************/
INT8U SdSpi_SendCmd(sd_struct *sds, INT8U cmd, INT8U *param, INT8U resptype, INT8U *resp)
{
	INT32 i,rlen;
	INT8U tmp;
    
	SdSpiHal_CSAssert(sds);
   
    SdSpiHal_SendByte(sds, (cmd & 0x3F) | 0x40);	 /* 发送命令头和命令字 send command header and word */
    
    for (i = 3; i >= 0; i--)
        SdSpiHal_SendByte(sds, param[i]);			 /* 发送参数 send parameters */

#if SD_CRC_EN
	tmp = SD_GetCmdByte6((cmd & 0x3F) | 0x40, param);
	SdSpiHal_SendByte(sds, tmp);
#else
    SdSpiHal_SendByte(sds, 0x95);					 /* CRC校验码,只用于第1个命令 CRC,only used for the first command */
#endif 
    
    rlen = 0;
    switch (resptype)								 /* 根据不同的命令,得到不同的响应长度 */
    {												 /* according various command,get the various response length */
  		case R1:
   	 	case R1B: rlen = 1;  break;
       		 
    	case R2:  rlen = 2;	 break;
       		 
   		case R3:  rlen = 5;	 break;
       		 
    	default:  SdSpiHal_SendByte(sds, 0xFF);	
      		      SdSpiHal_CSDeassert(sds);						 
        	      return SD_ERR_CMD_RESPTYPE;		 /* 返回命令响应类型错误 return error of command response type */
    		      break;
    }
    
    i = 0;				
    do 												 /* 等待响应,响应的开始位为0 */
    {												 /* Wait for a response,a response is a start bit(zero) */ 
        tmp = SdSpiHal_RecByte(sds);
        i++;
    }while (((tmp & 0x80) != 0) && (i < SD_CMD_TIMEOUT));
    
    if (i >= SD_CMD_TIMEOUT)
    {												 
        SdSpiHal_CSDeassert(sds);
        return SD_ERR_CMD_TIMEOUT;					 /* 返回命令超时 return response timeout of command */
    }
    
    for (i = rlen - 1; i >= 0; i--)
    {
        resp[i] = tmp;
        tmp = SdSpiHal_RecByte(sds);				 /* 循环的最后发送8clock  at the last recycle,clock out 8 clock */
    }
      
    SdSpiHal_CSDeassert(sds);
    return SD_NO_ERR;								 /* 返回执行成功 return perform sucessfully */
}


/*******************************************************************************************************************
** 函数名称: SdSpi_PackParam()				
**
** 功能描述: 将32位的参数转为字节形式	
**
** 输　  入: INT8U *parameter: 字节参数缓冲区
**           INT32U value    : 32位参数	
**
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
********************************************************************************************************************/
void SdSpi_PackParam(INT8U *parameter, INT32U value)
{
    parameter[3] = (INT8U)(value >> 24);
    parameter[2] = (INT8U)(value >> 16);
    parameter[1] = (INT8U)(value >> 8);
    parameter[0] = (INT8U)(value);
}


/*******************************************************************************************************************
** 函数名称: SdSpi_BlockCommand()				
**
** 功能描述: 块命令	
**
** 输　  入: sd_struct *sds  : SD/MMC卡信息结构体 
**           INT8U cmd	     : 命令字
**           INT8U resptype  : 响应类型
**           INT32U parameter: 块操作参数
**
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
********************************************************************************************************************/
INT8U SdSpi_BlockCommand(sd_struct *sds, INT8U cmd, INT8U resptype, INT32U parameter)
{
	INT8U param[4],resp,ret;
	
	parameter <<= SD_BLOCKSIZE_NBITS;					 /* 调整地址:左移9位 adjust address: move 9 bits left */

	SdSpi_PackParam(param, parameter);					 /* 将参数转化为字节形式 change the parameter to bytes form */	

	ret = SdSpi_SendCmd(sds, cmd, param, resptype, &resp);
	if (ret != SD_NO_ERR)
	   	 return ret;							 		 /* 结束数据传输失败 stop transmission operation fail */
	
	if (resp != 0)
		 return SD_ERR_CMD_RESP;		 				 /* 响应错误 response is error */
		 
	return SD_NO_ERR;
}

	
/*************
    CMD 0 
*************/
/*******************************************************************************************************************
** 函数名称: SpiCmd_Go_Idle_State()				
**
** 功能描述: CMD0, 复位所选择的卡, 使卡进入Idle状态		
**
** 输　  入: sd_struct *sds  : SD/MMC卡信息结构体 
**	
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
********************************************************************************************************************/
INT8U SpiCmd_Go_Idle_State(sd_struct *sds)
{
	INT8U param[4] = {0,0,0,0},resp;
	
    return (SdSpi_SendCmd(sds, CMD0, param, CMD0_R, &resp));	/* 复位命令 command that reset card */
}


/*************
    CMD 1 
*************/
/*******************************************************************************************************************
** 函数名称: SpiCmd_Send_OP_Cond()				
**
** 功能描述: CMD1, 使卡退出Idle状态(判断响应 R1 的 bit0 位)		
**
** 输　  入: sd_struct *sds  : SD/MMC卡信息结构体 
**	
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
********************************************************************************************************************/
INT8U SpiCmd_Send_OP_Cond(sd_struct *sds)
{
	INT8U param[4] = {0,0,0,0},resp[5],ret;
	INT32U i = 0;
	
 	do 
    {														/* 发出CMD1, 查询卡的状态, send CMD1 to poll card status */
        ret = SdSpi_SendCmd(sds, CMD1, param, CMD1_R, resp);
        if (ret != SD_NO_ERR)
       		return ret;
        i ++;
    }while (((resp[0] & MSK_IDLE) == MSK_IDLE) && (i <= SD_IDLE_WAIT_MAX));
    														/* 如果响应R1的最低位Idle位为1,则继续循环 */
    														/* if response R1 Idle bit is 1,continue recycle */    
    if (i >= SD_IDLE_WAIT_MAX)
        return SD_ERR_TIMEOUT_WAITIDLE;						/* 超时,返回错误 time out,return error */		
	
	return SD_NO_ERR;
}


/*************
    ACMD 41 
*************/
/*******************************************************************************************************************
** 函数名称: SpiCmd_SD_App_OP_Cond()				
**
** 功能描述: ACMD41(仅用于SD卡), 激活卡进入初始化过程	
**
** 输　  入: sd_struct *sds  : SD/MMC卡信息结构体
**           
**	
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
**
** 注	 意: 
********************************************************************************************************************/
INT8U SpiCmd_SD_App_OP_Cond(sd_struct *sds)
{
    INT8U param[4] = {0,0,0,0},resp[5],ret;

    ret = SdSpi_SendCmd(sds, CMD55, param, CMD55_R, resp);
    if (ret != SD_NO_ERR)
       	return ret;
       	
    ret = SdSpi_SendCmd(sds, ACMD41, param, ACMD41_R, resp);/* 激活内部初始化命令 active card to initialize process internal */
    if (ret != SD_NO_ERR)	
    	return ret;
		
	return SD_NO_ERR;	
}


/*************
    CMD 9 
*************/
/*******************************************************************************************************************
** 函数名称: SpiCmd_Send_CSD()				
**
** 功能描述: CMD9, 获取所选择的卡的CSD寄存器内容		
**
** 输　  入: sd_struct *sds  : SD/MMC卡信息结构体
**           INT8U csdlen    : CSD 寄存器长度
**	
** 输 　 出: INT8U *csdbuf   : CSD寄存器内容, bit127 在csdbuf[0] 
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
**
** 注	 意: 
********************************************************************************************************************/
INT8U SpiCmd_Send_CSD(sd_struct *sds, INT8U csdlen, INT8U *recbuf)
{
    INT8U param[4] = {0,0,0,0},resp,ret;
  
    ret = SdSpi_SendCmd(sds, CMD9, param, CMD9_R, &resp);		/* 读CSD寄存器命令 command that read CSD register */
    if (ret != SD_NO_ERR) 									
        return ret;									
  
    if (resp != 0)
        return SD_ERR_CMD_RESP;								    /* 响应错误 response is error */
    
	return (SdSpi_ReadRegister(sds, csdlen, recbuf));           /* 读取寄存器内容 */
}


/*************
    CMD10
*************/
/*******************************************************************************************************************
** 函数名称: SpiCmd_Send_CID()				
**
** 功能描述: 读取所选择的卡的CID寄存器值		
**
** 输　  入: sd_struct *sds  : SD/MMC卡信息结构体
**           INT8U cidlen    : CID 寄存器长度, 固定为16字节
**	
** 输 　 出: INT8U *cidbuf   : CID 寄存器内容, 长度为16字节
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
**
** 注	 意: 
********************************************************************************************************************/
INT8U SpiCmd_Send_CID(sd_struct *sds, INT8U cidlen, INT8U *recbuf)
{
    INT8U param[4] = {0,0,0,0},resp,ret;
 
    ret = SdSpi_SendCmd(sds, CMD10, param, CMD10_R, &resp);		/* 读CID寄存器命令 command that read CID register */
    if ( ret != SD_NO_ERR)
   		return ret;			  									
   
    if (resp != 0)
        return SD_ERR_CMD_RESP;									/* 响应错误 response is error */
      
  	return (SdSpi_ReadRegister(sds, cidlen, recbuf));
}


/***********************************
  CMD12: 停止传输Stop Transmission
************************************/
/*******************************************************************************************************************
** 函数名称: SdCmd_Send_Stop()				
**
** 功能描述: CMD12, 停止数据传输命令
**
** 输　  入: sd_struct *sds  : SD/MMC卡信息结构体
**	
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
**
** 注	 意: 该命令的argument域不必关心, 当主机需要停止读多个数据块或停止写多个数据块时调用该函数
********************************************************************************************************************/
INT8U SpiCmd_Stop_Transmission(sd_struct *sds)
{
	INT8U param[4] = {0,0,0,0},resp;
	
 	return (SdSpi_SendCmd(sds, CMD12, param, CMD12_R, &resp));		/* 结束数据传输命令失败 stop transmission command fail */
}



/*********
  CMD13
**********/
/*******************************************************************************************************************
** 函数名称: SpiCmd_Send_Status()				
**
** 功能描述: 读SD/MMC卡的 Card Status 寄存器
**
** 输　  入: sd_struct *sds  : SD/MMC卡信息结构体
**	         INT8U len       : 寄存器长度(固定为2)  
**
** 输 　 出: INT8U *recbuf : 接收缓冲区
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
**
** 注	 意: 
********************************************************************************************************************/
INT8U SpiCmd_Send_Status(sd_struct *sds, INT8U len, INT8U *buffer)
{
    INT8U param[4] = {0,0,0,0};

    return (SdSpi_SendCmd(sds, CMD13, param, CMD13_R, buffer)); 	/* 读 Card Status 寄存器 */
}


/*********
  CMD16 
**********/
/*******************************************************************************************************************
** 函数名称: SpiCmd_Set_BlockLen()				
**
** 功能描述: 设置卡的数据块长度
**
** 输　  入: sd_struct *sds  : SD/MMC卡信息结构体
**	         INT32U length	 : 数据块的长度值  
**
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
**
** 注	 意: 通常卡的一个读数据块长度与一个写数据块长度一样
********************************************************************************************************************/
INT8U SpiCmd_Set_BlockLen(sd_struct *sds, INT32U length)
{
	INT8U param[4],resp,ret;
  
    SdSpi_PackParam(param, length);					/* 将参数转化为字节形式 change the parameter to bytes form */
          												
    ret = SdSpi_SendCmd(sds, CMD16, param, CMD16_R, &resp);
    if (ret != SD_NO_ERR)
 		return ret;									/* 设置块的长度为length失败 set the length of block to length fail */
	
	if (resp != 0)
    	return SD_ERR_CMD_RESP;			   			/* 响应错误 response is error */
    
    return SD_NO_ERR; 								/* 返回执行成功 return perform sucessfully */			
}


/*********
  CMD17 
**********/
/*******************************************************************************************************************
** 函数名称: SpiCmd_Read_Single_Block()				
**
** 功能描述: 读单块命令
**
** 输　  入: sd_struct *sds  : SD/MMC卡信息结构体
**	         INT32U blockaddr: 块地址  
**
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
**
** 注	 意: blockaddr是以块为单位的块地址
********************************************************************************************************************/
INT8U SpiCmd_Read_Single_Block(sd_struct *sds, INT32U blockaddr)
{
	return (SdSpi_BlockCommand(sds, CMD17, CMD17_R, blockaddr)); /* 读单块命令 command that read single block */
}


/*********
  CMD18 
**********/
/*******************************************************************************************************************
** 函数名称: SpiCmd_Read_Single_Block()				
**
** 功能描述: 读多块命令
**
** 输　  入: sd_struct *sds  : SD/MMC卡信息结构体
**	         INT32U blockaddr: 块地址  
**
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
**
** 注	 意: blockaddr是以块为单位的块地址
********************************************************************************************************************/
INT8U SpiCmd_Read_Multiple_Block(sd_struct *sds, INT32U blockaddr)
{
	return (SdSpi_BlockCommand(sds, CMD18, CMD18_R, blockaddr)); /* 读多块命令 command that read multiple block */
}


/*********
  CMD24 
**********/
/*******************************************************************************************************************
** 函数名称: SpiCmd_Write_Single_Block()				
**
** 功能描述: 写单块命令
**
** 输　  入: sd_struct *sds  : SD/MMC卡信息结构体
**	         INT32U blockaddr: 块地址  
**
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
**
** 注	 意: blockaddr是以块为单位的块地址
********************************************************************************************************************/
INT8U SpiCmd_Write_Single_Block(sd_struct *sds, INT32U blockaddr)
{
	return (SdSpi_BlockCommand(sds, CMD24, CMD24_R, blockaddr)); /* 写单块命令 command that write single block */
}


/*********
  CMD25 
**********/
/*******************************************************************************************************************
** 函数名称: SpiCmd_Write_Multiple_Block()				
**
** 功能描述: 写多块命令
**
** 输　  入: sd_struct *sds  : SD/MMC卡信息结构体
**	         INT32U blockaddr: 块地址  
**
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
**
** 注	 意: blockaddr是以块为单位的块地址
********************************************************************************************************************/
INT8U SpiCmd_Write_Multiple_Block(sd_struct *sds, INT32U blockaddr)
{
	return (SdSpi_BlockCommand(sds, CMD25, CMD25_R, blockaddr)); /* 写多块命令 command that write multiple block */
}


/*********
  CMD25 
**********/
/*******************************************************************************************************************
** 函数名称: SpiCmd_Program_CSD()				
**
** 功能描述: 写CSD寄存器
**
** 输　  入: sd_struct *sds  : SD/MMC卡信息结构体
**	         INT8U *buff   	 : CSD寄存器内容  
**           INT8U len	  	 : CSD寄存器长度  
** 
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
**
** 注	 意: 
********************************************************************************************************************/
INT8U SpiCmd_Program_CSD(sd_struct *sds, INT8U len, INT8U *buff)
{
	INT8U param[4] = {0,0,0,0},resp,ret;
	
	if (len != 16) return SD_ERR_USER_PARAM;

	ret = SdSpi_SendCmd(sds, CMD27, param, CMD27_R, &resp); 	/* 发送写CSD寄存器命令 send command that write CSD */
	if (ret != SD_NO_ERR)
		return ret;
		        
    if (resp != 0)    
        return SD_ERR_CMD_RESP;
		
	buff[15] = (SdSpi_GetCRC7(buff, 15) << 1) + 0x01;  	        /* 计算CSD中的crc 位域 calculate crc field in CSD */
		
	return(SdSpi_WriteBlockData(sds, 0, 16, buff));
}


/********************************************************************************************************************
** 函数名称: INT8U SD_GetCRC7()						Name:	  INT8U SD_GetCRC7()
** 功能描述: 计算CRC7								Function: calculate crc7
** 输　  入: INT8U *pSource: 数据					Input:    INT8U *pSource: data
			 INT16U len    : 数据长度						  INT16U len   : data length
** 输 　 出: CRC7码									Output:	  CRC7 code
*********************************************************************************************************************/
INT8U SdSpi_GetCRC7(INT8U *pSource, INT16U len)
{
	INT8U i = 0, j;
	INT8U reg = 0;
	
	do
	{
	    for (j = 0; j < 8; j++)
	    {
			reg <<= 1;
			reg ^= ((((pSource[i] << j) ^ reg) & 0x80) ? 0x9 : 0);
	    }
	    
	    i++;
	    
	}while(i < len);
	
	return reg;
}	
	

#if SD_EraseBlock_EN
/***************
  CMD32或CMD35 
***************/
/*******************************************************************************************************************
** 函数名称: SdCmd_Erase_Wr_Blk_Start()				
**
** 功能描述: 设置块擦除起始地址
**
** 输　  入: sd_struct *sds    : SD/MMC卡信息结构体
**	         INT32U startblock : 块地址
** 
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
**
** 注	 意: startblock是以块为单位的块地址
********************************************************************************************************************/
INT8U SdCmd_Erase_Wr_Blk_Start(sd_struct *sds, INT32U startblock)
{
	if (sds->card_type == CARDTYPE_SD)
		return (SdSpi_BlockCommand(sds, CMD32, CMD32_R, startblock));	/* 发送擦除起始块地址 send the start block address of erasing operation */
	else
		return (SdSpi_BlockCommand(sds, CMD35, CMD35_R, startblock));	/* 发送擦除起始块地址 send the start block address of erasing operation */
}


/*****************
  CMD33 或 CMD36
*****************/
/*******************************************************************************************************************
** 函数名称: SdCmd_Erase_Wr_Blk_End()				
**
** 功能描述: 设置块擦除终止地址
**
** 输　  入: sd_struct *sds    : SD/MMC卡信息结构体
**	         INT32U endblock   : 块地址
** 
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
**
** 注	 意: startblock是以块为单位的块地址
********************************************************************************************************************/
INT8U SdCmd_Erase_Wr_Blk_End(sd_struct *sds, INT32U endblock)
{
	if (sds->card_type == CARDTYPE_SD)
		return (SdSpi_BlockCommand(sds, CMD33, CMD33_R, endblock));     /* 发送擦除终止块地址 send the end block address of erasing operation */
	else
		return (SdSpi_BlockCommand(sds, CMD36, CMD36_R, endblock));     /* 发送擦除终止块地址 send the end block address of erasing operation */
}


/*****************
  CMD38
*****************/
/*******************************************************************************************************************
** 函数名称: SdCmd_Erase()				
**
** 功能描述: 擦除已选中的块
**
** 输　  入: sd_struct *sds    : SD/MMC卡信息结构体
** 
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
**
** 注	 意: startblock是以块为单位的块地址
********************************************************************************************************************/
INT8U SdCmd_Erase(sd_struct *sds)
{
	INT8U param[4],resp,tmp;
	
	SdSpi_PackParam(param, 0);
	
	tmp = SdSpi_SendCmd(sds, CMD38, param, CMD38_R, &resp);	 	    /* 擦除所选择的块  erase blocks selected */
	if (tmp != SD_NO_ERR)
		return tmp;							 	
	
	if (SdSpi_WaitBusy(sds, SD_WAIT_ERASE) != SD_NO_ERR)			/* 等待擦除完成 wait for finishing erasing */
		return SD_ERR_TIMEOUT_ERASE;
	else
		return SD_NO_ERR;									
}	
#endif


/*****************
  CMD58
*****************/
/*******************************************************************************************************************
** 函数名称: SpiCmd_ReadOCR()				
**
** 功能描述: 读所选择的卡的操作条件寄存器OCR
**
** 输　  入: sd_struct *sds   : SD/MMC卡信息结构体
**
** 输 　 出: INT32U *ocrbuf   : OCR寄存器内容, 长度为1 
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
********************************************************************************************************************/
INT8U SpiCmd_ReadOCR(sd_struct *sds, INT32U *ocrbuf)
{
    INT8U param[4] = {0,0,0,0},resp[5],tmp;

    tmp = SdSpi_SendCmd(sds, CMD58, param, CMD58_R, resp);	/* 读 OCR 寄存器命令 */
    if (tmp != SD_NO_ERR)								/* read OCR register command */
    	return tmp;		 										
    												
    if (resp[4] != 0)
        return SD_ERR_CMD_RESP;			 				/* 响应错误 response is error */
          
    *ocrbuf = (resp[3] << 24) + (resp[2] << 16) +
              (resp[1] << 8) + resp[0];                 /* 复制OCR寄存器内容到接收缓冲区 */
            
    return SD_NO_ERR;
}


/********
  CMD59
*********/
/*******************************************************************************************************************
** 函数名称: SpiCmd_CRC_ON_OFF()				
**
** 功能描述: SPI模式下, 使能主机与卡之间的数据传输是否使用CRC校验
**
** 输　  入: sd_struct *sds   : SD/MMC卡信息结构体
**           INT8U bEnable    : 1: 使能;  0: 禁止 
**
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
********************************************************************************************************************/
#if SD_CRC_EN
INT8U SpiCmd_CRC_ON_OFF(sd_struct *sds, INT8U bEnable)
{
	INT8U param[4],resp,ret;
		
	if (bEnable == 1)
		param[0] = 1;											/* 使能crc enable crc */
	else
		param[0] = 0; 											/* 禁止crc disalbe crc */

	ret = SdSpi_SendCmd(sds, CMD59, param, CMD59_R, &resp);		/* "使能/禁止CRC"命令 enable/disable crc command */
	if (ret != SD_NO_ERR)
        return ret;							 
    
    if (resp != 0)    
       	return SD_ERR_CMD_RESP;									/* 响应错误 response is error */		
 	
	return SD_NO_ERR;
}
#endif


/********
  ACMD13
*********/
/*******************************************************************************************************************
** 函数名称: SpiCmd_ReadSD_Status()				
**
** 功能描述: 读SD卡的SD Status 寄存器(MMC卡没有该命令)
**
** 输　  入: sd_struct *sds   : SD/MMC卡信息结构体
**           INT8U sdslen  	  : 寄存器长度(固定为64) 
**
** 输 　 出: INT8U *recbuf 	  : 接收缓冲区
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
**
** 注    意: 只有SD卡才有SD Status 寄存器
********************************************************************************************************************/
INT8U SpiCmd_ReadSD_Status(sd_struct *sds, INT8U sdslen, INT8U *recbuf)
{
    INT8U param[4] = {0,0,0,0},resp[2],ret;
    
    ret = SdSpi_SendCmd(sds, CMD55, param, CMD55_R, resp);		/* 后续命令为一个应用命令 */
    if (ret != SD_NO_ERR)
    	return ret;											    /* command that the followed commnad is a specific application */
    												 
    if (resp[0] != 0)
        return SD_ERR_CMD_RESP;								    /* 响应错误 response is error */
    
    ret = SdSpi_SendCmd(sds, ACMD13, param, ACMD13_R, resp);	/* 读 SD_Status 命令 */
    if (ret != SD_NO_ERR)
    	return ret;											    /* command that read SD_Status register */
   												
    if ((resp[0] != 0) || (resp[1] != 0))
        return SD_ERR_CMD_RESP;								    /* 响应错误 response is error */
        
	return (SdSpi_ReadBlockData(sds, sdslen, recbuf));			/* 读出寄存器内容 read the content of the register */
}


/********
  ACMD51
*********/
/*******************************************************************************************************************
** 函数名称: SpiCmd_Send_SCR()				
**
** 功能描述: 读SD卡的 SCR 寄存器(MMC卡没有该命令)
**
** 输　  入: sd_struct *sds   : SD/MMC卡信息结构体
**           INT8U sdslen  	  : 寄存器长度(固定为64) 
**
** 输 　 出: INT8U *recbuf 	  : 接收缓冲区
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
**
** 注    意: MMC卡没有该寄存器
********************************************************************************************************************/
INT8U SpiCmd_Send_SCR(sd_struct *sds, INT8U scrlen, INT8U *recbuf)
{
    INT8U param[4] = {0,0,0,0},resp,ret;
    
    ret = SdSpi_SendCmd(sds, CMD55, param, CMD55_R, &resp);		/* 后续命令为一个应用命令 */
    if (ret != SD_NO_ERR)								        /* command that the followed commnad is a specific application */
    	return ret;													
    												 
    if (resp != 0)
        return SD_ERR_CMD_RESP;							        /* 响应错误 response is error */
    
    ret = SdSpi_SendCmd(sds, ACMD51, param, ACMD51_R, &resp);   /* 发送读 SD Status 命令*/
    if (ret != SD_NO_ERR)								        /* command that read SD Status register */
   		return ret;													
				    															
    if (resp != 0)
        return SD_ERR_CMD_RESP;						 	        /* 响应错误 response is error */
        
	return (SdSpi_ReadBlockData(sds, scrlen, recbuf));	 		/* 读出寄存器内容 read the content of the register */
}


/********
  ACMD22
*********/
/*******************************************************************************************************************
** 函数名称: SpiCmd_Send_Num_WR_Blcoks()				
**
** 功能描述: 得到正确写入的块数
**
** 输　  入: sd_struct *sds   : SD/MMC卡信息结构体
**
** 输 　 出: INT32U *blocknum : 返回的块数, 缓冲区长度固定为1
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
**
** 注    意: MMC卡没有该命令
********************************************************************************************************************/
#if SD_WriteMultiBlock_EN
INT8U SpiCmd_Send_Num_WR_Blcoks(sd_struct *sds, INT32U *blocknum)
{
    INT8U tmp[4] = {0,0,0,0},resp,ret;
  
    ret = SdSpi_SendCmd(sds, CMD55, tmp, CMD55_R, &resp);	 /* 后续命令为一个应用命令 */
    if (ret != SD_NO_ERR) 								     /* command that the followed commnad is a specific application */
    	return ret;
    	 
    if (resp != 0)
    	return SD_ERR_CMD_RESP;    	
 											
   	ret = SdSpi_SendCmd(sds, ACMD22, tmp, ACMD22_R, &resp); /* 读取正确写入的块数命令 */
   	if (ret != SD_NO_ERR)								    /* command that read the numbers of block written correctly */
   		return ret;											    
   		 														
	if (resp != 0)
    	return SD_ERR_CMD_RESP;							    /* 响应错误 response is error */
    		
    ret = SdSpi_ReadBlockData(sds, 4, tmp);				    /* 读块数 read the numbvers of block */
    if (ret != SD_NO_ERR)
    	return ret;
    	
    *blocknum = (tmp[0] << 24) + (tmp[1] << 16) + (tmp[2] << 8) + tmp[3];	
    													    /* 转换为32位 change to 32 bits */

	return SD_NO_ERR;    								    /* 返回执行成功 return perform sucessfully */		
}
#endif



		/*********************************************************
		
		    			下面为一些数据传输函数
		
		**********************************************************/

/*******************************************************************************************************************
** 函数名称: SdSpi_ReadRegister()				
**
** 功能描述: SPI模式下, 读SD/MMC卡寄存器内容
**
** 输　  入: sd_struct *sds   : SD/MMC卡信息结构体
**           INT32U len	      : 数据字节长度
**
** 输 　 出: INT8U *recbuf    : 接收缓冲区
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
**
** 注    意: 该函数只适应用于读寄存器时使用, 不适合于读数据块
********************************************************************************************************************/
INT8U SdSpi_ReadRegister(sd_struct *sds, INT32U len, INT8U *recbuf)
{	
	INT32U i = 0;
	INT8U resp;

    SdSpiHal_CSAssert(sds);
    do{    											/* 等待数据起始令牌 wait for data start token */
        resp = SdSpiHal_RecByte(sds);
    	i++;
    }while((resp == 0xFF) && (i < SD_READREG_TIMEOUT));    
    
    if (i >= SD_READREG_TIMEOUT)
    {
    	SdSpiHal_CSDeassert(sds);
    	return SD_ERR_TIMEOUT_READ;					/* 超时, 返回错误 timeout, return error */
  	}
  	
   	if (resp != SD_TOK_READ_STARTBLOCK)				
   	{												/* 不是收到数据起始令牌 not receive data start token */
		recbuf[0] = resp;							
		i = 1;										/* 还有len - 1个字节要接收 still len - 1 bytes will be received */
   	}
   	else
   		i = 0;										/* 收到数据起始令牌,还有len个字节要接收 received data start token,still len bytes will be received */
   	  	
    for (; i < len; i++)
   		recbuf[i] = SdSpiHal_RecByte(sds);			/* 接收数据 receive data */
   		
    i = SdSpiHal_RecByte(sds);								
    i = (i << 8) + SdSpiHal_RecByte(sds);   		/* 读取16位CRC get 16-bit CRC */	

#if SD_CRC_EN 
   	if (i != SD_GetCRC16(recbuf, len))
   	{												/* CRC校验错误 CRC check is error */
   		SdSpiHal_SendByte(sds, 0xFF);
   		SdSpiHal_CSDeassert(sds);		
  		return SD_ERR_DATA_CRC16;					/* 返回RCR16错误  return error of CRC16 */				
  	}    
#endif   
  
    SdSpiHal_SendByte(sds, 0xFF);					/* 返回之前发送8个clock  clock out 8 clk before return */
    SdSpiHal_CSDeassert(sds);	
    	
	return SD_NO_ERR;
}	


#define  SD_UCOSII_SMALLWAIT  256

/*******************************************************************************************************************
** 函数名称: SdSpi_ReadBlockData()				
**
** 功能描述: SPI模式下, 从卡中读取数据块
**
** 输　  入: sd_struct *sds   : SD/MMC卡信息结构体
**           INT32U len	      : 数据字节长度
**
** 输 　 出: INT8U *recbuf    : 接收缓冲区
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
**
** 注    意: 该函数只适于读数据块, 并不适合于读卡寄存器内容
********************************************************************************************************************/
INT8U SdSpi_ReadBlockData(sd_struct *sds, INT32U len, INT8U *recbuf)
{
    INT8U tmp;
    INT32U i = 0,timeout;
    
#if SD_UCOSII_EN    
    timeout = SD_UCOSII_SMALLWAIT;						/* 很少的等待时间 small wait time */
#else
	timeout = sds->timeout_read;						/* 等待接收数据开始令牌最长时间 wait time that receive data start token */
#endif  
    
    SdSpiHal_CSAssert(sds);    
    do
    { 											    	/* 等待接收数据开始令牌0xFE  wait for receiving data start token 0xFE */
        tmp = SdSpiHal_RecByte(sds);
        i++;
    }while((tmp == 0xFF) && (i < timeout));
	
#if SD_UCOSII_EN    
    if (i >= timeout)
   	{													/* 继续等待(挂起任务) continue to wait(suspend task) */
   		timeout = sds->timeout_read;
   		i = 0;
	    do
    	{
        	OSTimeDly(1);								/* 挂起该任务1 tick(suspend task 1 tick) */
        	tmp = SdSpiHal_RecByte(sds);
       	 	i++;
   		}while((tmp == 0xFF) && (i < timeout));
	}
#endif

	if (i >= timeout)
	{
		SdSpiHal_CSDeassert(sds);
       	return SD_ERR_TIMEOUT_READ;						/* 返回读超时错误码  return error timeout error code of reading */
	}
	
	if (tmp != SD_TOK_READ_STARTBLOCK)					/* 块读开始令牌错误 read start block token is error */
	{
		SdSpiHal_SendByte(sds, 0xFF);
		SdSpiHal_CSDeassert(sds);
		return SD_ERR_DATA_START_TOK;
	}
	
	for (i = 0; i < len; i++)
   		recbuf[i] = SdSpiHal_RecByte(sds);				/* 接收数据 receive data */
   		
    i = SdSpiHal_RecByte(sds);							
    i = (i << 8) + SdSpiHal_RecByte(sds);    			/* 读取16位CRC get 16-bit CRC */	

#if SD_CRC_EN 
   	if (i != SD_GetCRC16(recbuf, len))
   	{	
   		SdSpiHal_SendByte(sds, 0xFF); 					/* CRC校验错误 CRC check is error */
   		SdSpiHal_CSDeassert(sds);		
  		return SD_ERR_DATA_CRC16;						/* 返回RCR16错误  return error of CRC16 */				
  	}    
#endif   

	SdSpiHal_SendByte(sds, 0xFF); 
	SdSpiHal_CSDeassert(sds);

  	return SD_NO_ERR;									/* 返回函数执行成功 return function perform sucessfully */
}


/*******************************************************************************************************************
** 函数名称: SdSpi_WriteBlockData()				
**
** 功能描述: SPI模式下, 向卡写数据块
**
** 输　  入: sd_struct *sds   : SD/MMC卡信息结构体
**           INT8U bmulti     : 是否为多块操作1:是0:否
**           INT32U len	      : 数据字节长度
**           INT8U *sendbuf   : 发送缓冲区
**
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
**
** 注    意: 
********************************************************************************************************************/
INT8U SdSpi_WriteBlockData(sd_struct *sds, INT8U bmulti, INT32U len, INT8U *sendbuf)
{
	INT16U i;
	INT8U tmp;

	SdSpiHal_CSAssert(sds);
		
    SdSpiHal_SendByte(sds, 0xFF);							/* 开始发送数据之前发送8个clock clock out 8 clk before start */
    
    if (bmulti == 1)
        SdSpiHal_SendByte(sds, SD_TOK_WRITE_STARTBLOCK_M);	/* 写多块开始令牌 start token of write multi blocks */
	else
		SdSpiHal_SendByte(sds, SD_TOK_WRITE_STARTBLOCK);	/* 写单块开始令牌 start token of write single block */

	for (i = 0; i < len; i++)
        SdSpiHal_SendByte(sds, sendbuf[i]);					/* 发送数据 send data */

#if SD_CRC_EN	
	i = SD_GetCRC16(sendbuf,len);							/* 计算CRC16 calculate CRC16 */
#endif

	SdSpiHal_SendByte(sds, (i >> 8) & 0xFF);
	SdSpiHal_SendByte(sds, i & 0xFF); 						/* 发送CRC16校验码 send CRC16 check code */
			    
	tmp = SdSpiHal_RecByte(sds);
  	if ((tmp & SD_RESP_DATA_MSK) != SD_RESP_DATA_ACCETPTED)	
  	{		
   		SdSpiHal_SendByte(sds, 0xFF);						/* 返回之前发送8个clock  clock out 8 clk before return */
   		SdSpiHal_CSDeassert(sds);
   		return SD_ERR_DATA_RESP;							/* 数据响应错误 data response error */
    }
        
    SdSpiHal_CSDeassert(sds);
     		
    if (SdSpi_WaitBusy(sds, SD_WAIT_WRITE) != SD_NO_ERR)			
    	return SD_ERR_TIMEOUT_WRITE;						/* 写入超时 write time out */
    else
    	return SD_NO_ERR; 									/* 写入正确 write right */
}


/*******************************************************************************************************************
** 函数名称: SdSpi_StopMultiToken()				
**
** 功能描述: SPI模式下, 发送多块写停止令牌
**
** 输　  入: sd_struct *sds   : SD/MMC卡信息结构体
**
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
**
** 注    意: 
********************************************************************************************************************/
void SdSpi_StopMultiToken(sd_struct *sds)
{
	SdSpiHal_CSAssert(sds);
	
	SdSpiHal_SendByte(sds, 0xFF);							/* 先发送8个clock send 8 clock first */
	SdSpiHal_SendByte(sds, SD_TOK_STOP_MULTI);				/* 发送停止数据传输令牌 send stop transmission token */
	SdSpiHal_RecByte(sds);
	
    SdSpiHal_CSDeassert(sds);
}


/*******************************************************************************************************************
** 函数名称: SdSpi_WaitBusy()				
**
** 功能描述: SPI模式下, 查询SD卡是否处于忙状态
**
** 输　  入: sd_struct *sds   : SD/MMC卡信息结构体
**           INT32U waittype  : 超时类型, 可取值: SD_WAIT_WRITE 或 SD_WAIT_ERASE
**	
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
********************************************************************************************************************/
INT8U SdSpi_WaitBusy(sd_struct *sds, INT8U waittype)
{
    INT32U timeout, i = 0;
    INT8U tmp;
    
  	if (waittype == SD_WAIT_WRITE)
  		timeout = sds->timeout_write;				/* 等待类型为写操作 wait type is write operation */
  	else
  		timeout = sds->timeout_erase;   			/* 等待类型为擦除操作 wait type is erase operation */
    	
#if SD_UCOSII_EN
        timeout = SD_UCOSII_SMALLWAIT;				/* 很少的等待时间 small wait time */
#endif
   
	SdSpiHal_CSAssert(sds);
   	do
   	{ 												/* 等待忙结束 wait for being busy end */
        tmp = SdSpiHal_RecByte(sds);
        i++;
    }while ((tmp != 0xFF) && (i < timeout));		/* 忙时收到的值为0 always receive 0 when card is busy */    

#if SD_UCOSII_EN
	if (i >= timeout)
	{												/* 很少等待后卡仍忙, after small wait, card is still busy */
  		if (waittype == SD_WAIT_WRITE)
  			timeout = sds->timeout_write;
  		else
  			timeout = sds->timeout_erase;   
			
		i = 0;
   		do
   		{ 	
   			OSTimeDly(1);							/* 操作系统挂起1 tick  OS suspend 1 tick */
       		tmp = SdSpiHal_RecByte(sds);
       		i++;
    	}while ((tmp != 0xFF) && (i < timeout));	/* 忙时收到的值为0 always receive 0 when card is busy */    
	}
#endif	

	if(i < timeout) 
		tmp = SD_NO_ERR;							/* 返回0,表示没超时 return 0 indicate that operation is not time out */
	else 
		tmp = SD_ERR_TIMEOUT_WAIT;					/* 返回错误码,表示超时 return error code indicate that operation is time out */

	SdSpiHal_SendByte(sds, 0xFF);
	SdSpiHal_CSDeassert(sds);							
	return tmp;										/* 返回执行结果 */
}


/*******************************************************************************************************************
** 函数名称: SdSpi_Delay()				
**
** 功能描述: SPI模式下, SPI总线延时, 以8个clock为单位
**
** 输　  入: sd_struct *sds : SD/MMC卡信息结构体
**           INT8U value    : 延时值,不超过255
**	
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
********************************************************************************************************************/
void SdSpi_Delay(sd_struct *sds, INT8U value)
{
    INT8U i;

    for (i = 0; i < value; i++)
        SdSpiHal_SendByte(sds, 0xFF);				/* 发送0xFF clock out 0xFF */
}


#endif 




