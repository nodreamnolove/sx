/****************************************Copyright (c)**************************************************
**                               Guangzhou ZLG-MCU Development Co.,LTD.
**                                      graduate school
**                                 http://www.zlgmcu.com
**
**--------------File Info-------------------------------------------------------------------------------
** File name:			sdhal.c
** Last modified Date:	2007-5-20
** Last Version:		V2.0
** Descriptions:		SD/MMC卡读写模块: 硬件抽象层 ---- SD总线硬件相关的操作函数
**
**
**------------------------------------------------------------------------------------------------------
** Created by:			Ming Yuan Zheng
** Created date:		2005-1-6
** Version:				V1.0
** Descriptions:		The original version
**
**------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
** Version:				
** Descriptions:		
**
**------------------------------------------------------------------------------------------------------
** Modified by: 
** Modified date:
** Version:	
** Descriptions: 
**
********************************************************************************************************/

#include "sdconfig.h"


#if SDBUS_MODE_EN

#if LPC23xx_MCI_DMA_EN
INT32U *src_addr; 
INT32U *dest_addr;
#endif
 

/* mci_struct 结构体成员 RxTxFlag 取值 */
#define  MCI_RESET_FLAG             0xFF
#define  MCI_CUR_REC                0x01
#define  MCI_CUR_SEND               0x02

/* MCI数据传输结构体 */
typedef struct MCI_STRUCT
{
    INT8U CurTran;              // 当前数据传输类型, 可为MCI_CUR_REC或MCI_CUR_SEND
    INT8U RxTxFlag;             // 数据接收/发送标志, 用于标识接收/发送执行结果

    INT8U *pRecBuf;             // 接收数据缓冲区指针
    INT8U *pSendBuf;            // 发送数据缓冲区指针
    INT32U RecCnt;              // 接收数据计数器
    INT32U SendCnt;             // 发送数据计数器
    
    sd_struct *psds;            // 当前sd_struct结构体指针
}mci_struct;

volatile mci_struct lpc_mci_struct;    // mci_struct结构体变量
volatile mci_struct *lpc_mci = &lpc_mci_struct;


void SD_PowerOff(void);
void MCI_Delay(INT16U val);
void MCI_SendCmd(INT32U CmdIndex, INT32U Argument, INT32U ExpectResp, INT32U AllowTimeout );
INT8U MCI_GetCmdResp(INT32U ExpectCmdData, INT32U ExpectResp, INT8U resplen, INT32U *CmdResp);

	/**********************************************
		
		硬件抽象层接口函数实现文件
	
	***********************************************/

/*********************************************************************************************************
** 函数名称: SdHal_Initialize()				
**
** 功能描述: 初始化访问卡的硬件条件		
**
** 输　  入: sd_struct *sds:  卡信息结构体
**							  	
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
**
** 主要工作: 1. 先给卡下电, 再给卡上电;     2. 设置访问卡的速度小于或等于 400KHz;
**
**           3. 使 CMD 线处于开漏状态;      4. 初始化相关硬件寄存器为适当的状态
*********************************************************************************************************/
INT8U SdHal_Initialize(sd_struct *sds)
{
    // 卡下电并延时
    SD_PowerOff(); 
	MCI_Delay(1);				            

	// 卡上电并初始化MCI相关引脚
	SD_POWER_ON();

    MS_CTRL = 0x222;                                                    /* 使能SD卡时钟                 */ 
	           
    MCIPower = 0x02;
	MCI_Delay(1);                                                       /* 延时                         */ 
    MCIPower |= 0x01;
    MCIPower |= (1 << 6 );	                                            /* CMD线处于开漏状态            */
  	
	// 设置访问卡时钟小于 400KHz
	SdHal_SetMCIClock(sds, SD_RATE_NORMAL); 				           //SD_RATE_SLOW
    SdHal_EnableMCIBusWidth(sds,1);
	// 清空 MCI 相关寄存器		
    MCICommand  = 0;						    
	MCIClear    = 0x7FF;						                       /* 清空MCI状态寄存器             */
	MCIDataCtrl = 0;						
    MCIMask0    = 0;							
    MCIMask1    = 0;                                                   /* 屏蔽所有 MCI 中断             */
    
	return SD_NO_ERR;
}												


/*******************************************************************************************************************
** 函数名称: SD_PowerOff()				
**
** 功能描述: 给卡下电, 将	
**
** 输　  入: 无
**							  	
** 输 　 出: 无
**
** 返 回 值: 无
********************************************************************************************************************/
void SD_PowerOff(void)
{

	SD_POWER_OFF();							/* 关闭 SD 卡电源  turn off power of sd card */
	

	MCIPower = 0x00;                        /* 配置电源控制寄存器 */
    MCIClock = 0;
    MCIClock &= ~(1 << 8);                  /* 关闭MCICLK时钟 */	
}


/*******************************************************************************************************************
** 函数名称: SdHal_OpenDrainCmd()				
**
** 功能描述: 设置 CMD 信号线的开漏状态 	
**
** 输　  入: sd_struct *sds  : 卡信息结构体
**           INT8U bOpenDrain: > 0: CMD 信号线开漏;  = 0: CMD 信号线处于非开漏 
**          							  	
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
**
** 主要工作: 1. 初始化相关硬件寄存器;  2. 先给卡下电, 再上电;  3. 设置访问卡的速度小于或等于 400KHz
**
**           4. 使 CMD 线处于开漏状态
********************************************************************************************************************/
void SdHal_OpenDrainCmd(sd_struct *sds, INT8U bOpenDrain)
{
    if (bOpenDrain)
        MCIPower |= (1 << 6 );	            /* CMD线处于开漏状态 */ 
            
    else
        MCIPower &= ~(1 << 6 );	            /* CMD线处于非开漏状态 */
              
}


/*******************************************************************************************************************
** 函数名称: SdHal_SendCmd()				
**
** 功能描述: 发送一个SD/MMC命令, 并获得响应			
**
** 输　  入: sd_struct *sds:  卡信息结构体, 仅使用sds->card_posnum成员用于区分卡座
			 INT8U cmd:       命令索引(命令的[5:0]位)
**		     INT32U argument: 命令参数
**			 INT8U resptype:  响应类型, 取值可以为R0(即不需要响应), R1, R1B, R2, R3, R6 
**			 INT8U resplen:   响应长度, 
**                            当响应为R0时, 取值为0(对应resp指针也可为空).
**                            当响应为R1,R1B,R3,R6时, 取值4; 
**                            当响应为R2时, 取值为16(R2的bit[127:0]).
**							  	
** 输 　 出: INT32U *resp:    响应内容, bit127在resp[0]的bit32
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
********************************************************************************************************************/
INT8U SdHal_SendCmd(sd_struct *sds, INT8U cmd, INT32U argument, INT8U resptype, INT32U *resp, INT8U resplen)
{
    INT32U ExpectResp, i;
    INT8U ret;
    
    if (resptype == R0)
    {
        ExpectResp = EXPECT_NO_RESP;                        /* 卡无须响应本命令 */
    } 
    else if ((resptype == R1) || (resptype == R1B) || (resptype == R3) || (resptype == R6))
    {
        ExpectResp = EXPECT_SHORT_RESP;                     /* 短响应 */
    }  
    else if (resptype == R2)
    {
        ExpectResp = EXPECT_LONG_RESP;                      /* 长响应 */
    } 
    else 
    {
        return SD_ERR_CMD_RESPTYPE;							/* 错误的响应类型 */
    }
                
    MCI_SendCmd(cmd, argument, ExpectResp, 0);				/* 发送命令并获取响应 */
    ret = MCI_GetCmdResp(cmd, ExpectResp, resplen, resp);
    if (ret != SD_NO_ERR)
        return ret;
       
    for (i = 0; i < 0x200; i++);	                        /* 延时, 确保 MCICommand 寄存器在规定时间内不再次被写入 */
  
    return ret;
}


/*******************************************************************************************************************
** 函数名称: SdHal_EnableMCIBusWidth()				
**
** 功能描述: 使能SD/MMC主控器是否使能宽总线		
**
** 输　  入: sd_struct *sds: 卡信息结构体, 仅使用sds->card_posnum成员用于区分卡座
**           INT32U bWidth : > 0:  使能宽总线;     0: 禁止宽总线     
**							  	
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
********************************************************************************************************************/
void SdHal_EnableMCIBusWidth(sd_struct *sds, INT32U bWidth)
{
	if (bWidth != 0)
		MCIClock |= (1 << 11);

	else
	    MCIClock &= ~(1 << 11);
}


/*******************************************************************************************************************
** 函数名称: SdHal_SetMCIClock()				
**
** 功能描述: 设置读写SD/MMC卡时钟		
**
** 输　  入: sd_struct *sds  :  卡信息结构体
**           INT32U ClockRate:	要求的时钟速度.  取值SD_RATE_SLOW   时, 设置访问卡速度小于 400KHz		
**			  	                                 取值SD_RATE_NORMAL 时, 设置访问卡速度为卡的最大速度或主控器的最大速度
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
**
** 注    意: 本函数有责任在ClockRate取不同的值时, 计算出相应的 sds->clkval 值, 必须大于0. 
**
********************************************************************************************************************/
void SdHal_SetMCIClock(sd_struct *sds, INT32U ClockRate)
{
    INT32U i;
    INT32 div = 0;
     
	if (ClockRate == SD_RATE_SLOW)
	{
		div = MCLKDIV_SLOW;	                            /* 设置时钟小于等于400KHz */
		sds->host_clkval = Fhclk / (div + 1) / 2;     /* MCI 总线周期频率 = MCLK / (2 * (div + 1)) */
	}
	else if (ClockRate == SD_RATE_NORMAL) 
	{
	    div = MCLKDIV_NORMAL;	
	        		               
		sds->host_clkval = Fhclk / (div + 1) / 2;       /* MCI 总线周期频率 = MCLK / (2 * (div + 1)) */
	}
	
	MCIClock &= ~0xFF;
	MCIClock = (1 << 8) | (1 << 9) | div;
	for (i = 0; i < 0x10; i++);	                        /* 在下次写MCIClock前, 延时 3MCLK + 2PCLK */
}


/*******************************************************************************************************************
** 函数名称: SdHal_CheckCard()				
**
** 功能描述: 检测卡是否完全插入卡座中
**
** 输　  入: sd_struct *sds:  卡信息结构体
**	         
** 输 　 出: 无
**
** 返 回 值: 1: 卡完全插入	   0: 卡没有完全插入
********************************************************************************************************************/
INT8U SdHal_CheckCard(sd_struct *sds)
{
	if (SD_INSERT_STATUS() != 0)
		return 0;								/* 未完全插入 */
	else
		return 1;								/* 完全插入 */
}


/*******************************************************************************************************************
** 函数名称: SdHal_CheckCardWP()				
**
** 功能描述: 检测卡写保护
**
** 输　  入: sd_struct *sds:  卡信息结构体
**	         
** 输 　 出: 无
**
** 返 回 值: 1: 卡已写保护	  0: 卡未写保护
********************************************************************************************************************/
INT8U SdHal_CheckCardWP(sd_struct *sds)
{
	if (SD_WP_STATUS() != 0)
		return 1;								/* 写保护 */
	else
		return 0;								/* 未写保护 */
}



/*******************************************************************************************************************
** 函数名称: SdHal_BeforeReadBlock()				
**
** 功能描述: 在发送读单块命令之前, 给硬件控制器做一些事情的机会
**
** 输　  入: sd_struct *sds  :  卡信息结构体 
**	         
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
********************************************************************************************************************/
INT8U SdHal_BeforeReadBlock(sd_struct *sds)
{	
    MCIMask0 |= (DATA_RX_INT_MASK) | (FIFO_INT_MASK);	    /* 仅使能接收中断 */
	MCIMask1 = MCIMask0;

	MCIClear = 0x7FF;							            /* 清空清零寄存器的所有标志 */
	MCIDataCtrl = 0;							            /* 数据控制寄存器清0  */
			 
	MCIDataTimer = sds->timeout_read;                       /* 读数据块超时值 */
	MCIDataLength = SD_BLOCKSIZE;				            /* 块长度 */
    MCIFifoCnt = 0;                                         /* 清空接收FIFO计数器*/
    
    lpc_mci->RecCnt = 0;                                     /* 初始化mci_struct结构体全局变量lpc_mci */
    lpc_mci->RxTxFlag = MCI_RESET_FLAG;
    lpc_mci->psds = sds;    
    lpc_mci->CurTran = MCI_CUR_REC; 
         
	return SD_NO_ERR;
}


/*******************************************************************************************************************
** 函数名称: SdHal_ReadBlock()				
**
** 功能描述: 在发送读单块命令之后, 由硬件控制器实现读取数据
**
** 输　  入: sd_struct *sds  :  卡信息结构体 
**	         
** 输 　 出: INT8U *recbuf: 接收缓冲区, 大小为512字节
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
********************************************************************************************************************/
INT8U SdHal_ReadBlock(sd_struct *sds, INT8U *recbuf)
{
	INT32U DataCtrl = 0;
	INT8U ret;
			
	lpc_mci->pRecBuf = recbuf;	

#if LPC23xx_MCI_DMA_EN
    // DMA 控制器初始化
	DMA_Init(1, P2M);                                                           
	DMACC1Configuration |= 0x10001 | (0x04 << 1) | (0x00 << 6) | (0x06 << 11);
	
	// 设置数据控制寄存器: 读,块传输,DMA传输,块长度
	DataCtrl |= ((1 << 0) | (1 << 1) | (1 << 3) | (SD_BLOCKSIZE_NBITS << 4));	    
#else
    // 设置数据控制寄存器: 读,块传输,块长度
	DataCtrl |= ((1 << 0) | (1 << 1) | (SD_BLOCKSIZE_NBITS << 4));
#endif
	
#if SD_UCOSII_EN
    OS_ENTER_CRITICAL();
    MCIDataCtrl = DataCtrl;						                /* 启动数据传输 */
    OSSemPend(sds->pSemSdmmcRW, sds->timeout_read_os, &ret);    /* 进入等待状态 */
    if (ret == OS_NO_ERR)   
        ret = lpc_mci->RxTxFlag;                                 /* 返回数据接收结果 */                            
    else if (ret == OS_TIMEOUT)
        ret = SD_ERR_TIMEOUT_READ;                              /* 接收数据超时 */            
    else
        ret = SD_ERR_WAIT_WRSEM;                                /* 等待信号量失败 */
    
    OS_EXIT_CRITICAL();
#else	
    MCIDataCtrl = DataCtrl;                                     /* 启动数据传输 */
	do
	{                                                           /* 等待数据接收 */
	    if (lpc_mci->RxTxFlag == SD_NO_ERR)
	        break;                                              /* 数据接收成功 */
	}while (lpc_mci->RxTxFlag == MCI_RESET_FLAG);
	
	ret = lpc_mci->RxTxFlag;                                     /* 返回接收结果 */
#endif	

	MCIMask0 &= ~((DATA_RX_INT_MASK) | (FIFO_INT_MASK));        /* 仅禁止接收中断 */
	MCIMask1 = MCIMask0;  	

#if LPC23xx_MCI_DMA_EN
    src_addr = (INT32U *)DMA_SRC;                               /* 取得 DMA 源地址 */         
	dest_addr = (INT32U *)DMA_DST;                              /* 取得 DMA 目标地址 */    
    if (ret == SD_NO_ERR)
        memcpy(recbuf, dest_addr, 512);                         /* 返回接收到的数据 */
#endif

    return ret;
}



/*******************************************************************************************************************
** 函数名称: SdHal_BeforeWriteBlock()				
**
** 功能描述: 在发送写单块命令之前, 给硬件控制器做一些事情的机会
**
** 输　  入: 无 
**	         
** 输 　 出: INT8U *recbuf: 接收缓冲区, 大小为512字节
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
********************************************************************************************************************/
INT8U SdHal_BeforeWriteBlock(sd_struct *sds)
{
	MCIMask0 = DATA_TX_INT_MASK | (FIFO_INT_MASK);	      /* 仅使能TX中断 */
	MCIMask1 = MCIMask0; 

	MCIClear = 0x7FF;							          /* 清空清零寄存器的所有标志 */
	MCIDataCtrl = 0;							          /* 数据控制寄存器清0 */
		 
	MCIDataTimer = sds->timeout_write;                    /* 写入写数据超时值 */
	MCIDataLength = SD_BLOCKSIZE;				          /* 数据块的长度 */
    MCIFifoCnt = 0;  
    
    lpc_mci->psds = sds;                                   /* 初始化lpc_mci结构体变量成员 */  
    lpc_mci->RxTxFlag = MCI_RESET_FLAG; 
    lpc_mci->SendCnt = 0;
    lpc_mci->CurTran = MCI_CUR_SEND;   
	return SD_NO_ERR;
}


/*******************************************************************************************************************
** 函数名称: SdHal_WriteBlock()				
**
** 功能描述: 在发送写单块命令之后, 由硬件控制器向卡发送数据
**
** 输　  入: 无 
**	         
** 输 　 出: INT8U *recbuf: 接收缓冲区, 大小为512字节
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
********************************************************************************************************************/
INT8U SdHal_WriteBlock(sd_struct *sds, INT8U *sendbuf)
{
	INT32U DataCtrl = 0;
	INT8U ret;
	
#if LPC23xx_MCI_DMA_EN
    src_addr = (INT32U *)DMA_SRC;                           
	dest_addr = (INT32U *)DMA_DST;
	//memcpy(sendbuf, src_addr, 512);
	memcpy(src_addr, sendbuf, 512);                              /* 复制要写入的数据到DMA源地址 */

	DMA_Init(0, M2P);                                            /* DMA控制器初始化 */
	DMACC0Configuration |= 0x00001 | (0x00 << 1) | (0x04 << 6) | (0x05 << 11);
	
	/* 设置数据控制寄存器: 写, 块传输, DMA, 数据长度 */
	DataCtrl |= ((1 << 0) | (1 << 3) | (SD_BLOCKSIZE_NBITS << 4));               
#else
 	lpc_mci->pSendBuf = sendbuf;
 	
 	/* 设置数据控制寄存器: 读, 块传输, 数据长度 */
 	DataCtrl |= ((1 << 0) | (SD_BLOCKSIZE_NBITS << 4));
#endif 	


#if SD_UCOSII_EN
    OS_ENTER_CRITICAL();
    MCIDataCtrl = DataCtrl;                                      /* 启动数据传输 */
    OSSemPend(sds->pSemSdmmcRW, sds->timeout_write_os, &ret);    /* 进入等待状态 */
    if (ret == OS_NO_ERR)
	    ret = lpc_mci->RxTxFlag;                                  /* 返回写块数据结果 */
    else if (ret == OS_TIMEOUT)
        ret = SD_ERR_TIMEOUT_WRITE;                              /* 返回等待超时 */
    else
        ret = SD_ERR_WAIT_WRSEM;                                 /* OSSemPend()执行失败 */
    
    OS_EXIT_CRITICAL();
#else
    MCIDataCtrl = DataCtrl;                                      /* 启动数据传输 */
	do
	{
	    if (lpc_mci->RxTxFlag == SD_NO_ERR)
	        break;                                               /* 发送数据成功 */   
	}while (lpc_mci->RxTxFlag == MCI_RESET_FLAG);
	
	ret = lpc_mci->RxTxFlag;                                      /* 返回发送数据执行结果 */
#endif

	MCIMask0 &= ~(DATA_TX_INT_MASK | FIFO_INT_MASK);	         /* 仅禁止TX中断 */
	MCIMask1 = MCIMask0;  	
  
	return ret;
}


/*******************************************************************************************************************
** 函数名称: SDMMC_WaitReadyforData()				
**
** 功能描述: 等待卡可接收数据(等待卡中有空的接收缓冲区), 即等待卡状态寄存器的 bit8 为1, 
**
** 输　  入: sd_struct *sds  : SD/MMC卡信息结构体 
**	         
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
********************************************************************************************************************/
INT8U SdHal_WaitReadyforData(sd_struct *sds)
{    
    return (SdHal_WaitBusy(sds, BUSY_TYPE_RDY_DATA));
}


#define   SD_UCOSII_SMALLWAIT		256				            /* 运行于UCOS-II时的小等待时间(避免OSTimeDly(1)降低系统性能) */
#define   CMD13_CLK_NUM             160                         /* 一个CMD13大约需要160个clk */   

/*******************************************************************************************************************
** 函数名称: SdHal_WaitBusy()				
**
** 功能描述: 等待卡编程或擦除结束, 即等待卡状态由编程状态变为传输状态: prg --> tran
**
** 输　  入: sd_struct *sds  : SD/MMC卡信息结构体 
**	         INT32U BusyType : 等待忙类型, 取值为: SD_WAIT_WRITE 和 SD_WAIT_ERASE
** 输 　 出: 无
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
********************************************************************************************************************/
INT8U SdHal_WaitBusy(sd_struct *sds, INT32U busytype)
{
    INT8U ret;
    INT32U status, timeout = 0, cnt = 0;
    INT32U stmask = CARD_STATUS_CURRENT_STATE;                
    INT32U stwait = CARD_STATUS_PRG;                            /* 等待时, 卡处于编程状态 */
    
    if (busytype == BUSY_TYPE_RDY_DATA)
    {
        timeout = sds->timeout_write / CMD13_CLK_NUM;           /* 写超时等待时间 */
        stmask = CARD_STATUS_RDY_DATA;
        stwait = 0;                                             /* 等待时, 卡的接收缓冲区未就绪 */
    }
    else if (busytype == BUSY_TYPE_PROG)
        timeout = sds->timeout_write / CMD13_CLK_NUM;           /* 擦除超时等待时间 */
    else if (busytype == BUSY_TYPE_ERASE)
        timeout = sds->timeout_erase / CMD13_CLK_NUM;
            
#if SD_UCOSII_EN 
    timeout = SD_UCOSII_SMALLWAIT;                              /* 先进行快速查询, 再每隔1 os tick查询 */
#endif

    do	
	{	
		ret = SdCmd_Send_Status(sds, &status);                  /* 读取卡的状态 */
		if (ret != SD_NO_ERR)
			return ret;
			
	    if (!SdHal_CheckCard(sds))                              /* 检测卡是否拔出 */
 		    return SD_ERR_NO_CARD; 	
			
		cnt++;			
	}while(((status & stmask) == stwait) && (cnt < timeout));

#if SD_UCOSII_EN
	if (cnt >= timeout)
	{                                                           /* 很少等待后卡仍忙 */
  		if ((busytype == BUSY_TYPE_RDY_DATA) || (busytype == BUSY_TYPE_PROG))
  			timeout = sds->timeout_write_os;
  		else
  			timeout = sds->timeout_erase_os;   
			
		cnt = 0;
   		do
   		{ 	
   			OSTimeDly(1);                                       /* 操作系统挂起1 tick */
   			
		    ret = SdCmd_Send_Status(sds, &status);              /* 读取卡的状态 */
		    if (ret != SD_NO_ERR)
		    	return ret;
			
	        if (!SdHal_CheckCard(sds))                          /* 检测卡是否拔出 */
 		        return SD_ERR_NO_CARD; 	   			
   			
   			cnt++;
    	}while(((status & stmask) == stwait) && (cnt < timeout));
	}
#endif	
	
	return SD_NO_ERR;
}


/*******************************************************************************************************************
** 函数名称: MCI_Delay()				
**
** 功能描述: 延时函数	
**
** 输　  入: INT16U val:  延时值, 1大约相当于5ms
**							  	
** 输 　 出: 无
**
** 返 回 值: 无 
********************************************************************************************************************/
void MCI_Delay(INT16U val)
{
#if !SD_UCOSII_EN
    INT16U i,j;
    for (i = 0; i < val; i++)
        for (j = 0; j < 0xC000; j++);
#else
    OSTimeDly(val); 
#endif   
}


/*******************************************************************************************************************
** 函数名称: MCI_SendCmd()				
**
** 功能描述: 发送命令给卡, 但不获取响应
**
** 输　  入: INT32U CmdIndex    : 命令索引
**           INT32U Argument    : 命令参数
**           INT32U ExpectResp  : 期望得到的回复 
**           INT32U AllowTimeout: 是否允许超时
**	         
** 输 　 出: 无
**
** 返 回 值: 无
********************************************************************************************************************/
void MCI_SendCmd(INT32U CmdIndex, INT32U Argument, INT32U ExpectResp, INT32U AllowTimeout)
{
    INT32U CmdData = 0, i;
	INT32U CmdStatus;
		
	while ((CmdStatus = (INT32U)MCIStatus) & MCI_CMD_ACTIVE)    /* 命令正在处理中 */
	{
		MCICommand = 0;                                 
		MCIClear = CmdStatus | MCI_CMD_ACTIVE;          /* 清除相关状态 */
		for(i = 0; i < 0x20; i++);
	}	
		
	CmdData |= (CmdIndex & 0x3F);	                    /* 命令索引只有bit0 ~ bit5 有效 */
	if (ExpectResp == EXPECT_NO_RESP)			        /* 无响应类型 */
	{
		CmdData &= ~((1 << 6) | (1 << 7));		        /* 清除短响应和长响应位 */
	}
	else if (ExpectResp == EXPECT_SHORT_RESP)	        /* 希望得到短响应 */
	{
		CmdData |= (1 << 6);
	}
	else if (ExpectResp == EXPECT_LONG_RESP)	        /* 希望得到长响应 */
	{
		CmdData |= (1 << 6) | (1 << 7);
	}

	if (AllowTimeout)			                        /* 是否允许超时中断 */
		CmdData |= (1 << 8);
	else
		CmdData &= ~(1 << 8);

	CmdData |= (1 << 10);		                        /* 使能发送命令 */
	MCIArgument = Argument;	                            /* 命令参数     */
	MCICommand = CmdData;                               /* 启动命令发送 */
}


/*******************************************************************************************************************
** 函数名称: MCI_GetCmdResp()				
**
** 功能描述: 从卡获取响应, 该函数与MCI_SendCmd()成对使用
**
** 输　  入: INT32U ExpectCmdData: SD/MMC 命令码
**           INT32U ExpectResp   : 希望的响应, 取值为: EXPECT_NO_RESP, EXPECT_SHORT_RESP, EXPECT_LONG_RESP
**           INT8U  resplen      : 响应长度
**           
**	         
** 输 　 出: INT32U *CmdResp     : 响应内容, 长度为resplen
**
** 返 回 值: 0:   正确    >0:   错误码, 见 sddriver.h 文件
********************************************************************************************************************/
INT8U MCI_GetCmdResp(INT32U ExpectCmdData, INT32U ExpectResp, INT8U resplen, INT32U *CmdResp)
{
	INT32U LastCmdIndex;
	INT32U CmdRespStatus = 0;

	while (1)
	{                                                       /* 不断循环等待卡的响应 */
		CmdRespStatus =  (INT32U)MCIStatus;
		if (CmdRespStatus & (MCI_CMD_TIMEOUT))
		{                                                   /* 超时错误, 超时周期固定为64个MCICLK时钟周期 */
		    MCIClear = CmdRespStatus | MCI_CMD_TIMEOUT;
			MCICommand = 0;                                 /* 清空命令寄存器及参数寄存器 */
			MCIArgument = 0xFFFFFFFF;
			return SD_ERR_CMD_TIMEOUT;                      /* 返回响应命令超时错误 */
		}
		
		if (CmdRespStatus & MCI_CMD_CRC_FAIL)
		{                                                   /* CRC校验失败错误 */
			MCIClear = CmdRespStatus | MCI_CMD_CRC_FAIL;
			LastCmdIndex =  (INT32U)MCICommand & 0x003F;
			if ((LastCmdIndex == SEND_OP_COND) || (LastCmdIndex == SEND_APP_OP_COND) 
					|| (LastCmdIndex == STOP_TRANSMISSION))
			{                                               /* 忽略命令 SEND_OP_COND和STOP_TRANSMISSION的CRC校验 */
				MCICommand = 0;
				MCIArgument = 0xFFFFFFFF;	 
				break;
			}
			else
				return SD_ERR_CMD_RESPCRC;                  /* 返回响应命令CRC校验失败错误 */
		}
		else if (CmdRespStatus & MCI_CMD_RESP_END)
		{
			MCIClear = CmdRespStatus | MCI_CMD_RESP_END;
			break;	                                        /* 响应已成功接收到,退出循环 */
		}
		else if ((CmdRespStatus & MCI_CMD_SENT) && (ExpectResp == EXPECT_NO_RESP))
		{                                                   /* 该命令无须响应 */
			MCIClear = MCI_CMD_SENT;
			return SD_NO_ERR;                               /* 返回命令*/
		}		
	}
	
	if ((MCIRespCmd & 0x3F) != ExpectCmdData)
	{
	    /* 响应为非R1, 而是R2或R3, 则无须检查响应命令是否与发送的相符合 */
		if ((ExpectCmdData != SEND_OP_COND) && (ExpectCmdData != SEND_APP_OP_COND) 
			 && (ExpectCmdData != ALL_SEND_CID) && (ExpectCmdData != SEND_CSD))
		{
			return SD_ERR_CMD_RESP;                         /* 返回响应错误 */
		}
	}

    /* 下面读取短响应内容及长响应内容 */
	if ((ExpectResp == EXPECT_SHORT_RESP) && (resplen == 1))
	{                                                       
   		*CmdResp =  (INT32U)MCIResponse0;                            /* 读取短响应内容 */
	}
	else if ((ExpectResp ==  (INT32U)EXPECT_LONG_RESP) && (resplen == 4))
	{
		*CmdResp =  (INT32U)MCIResponse0;                            /* 读取长响应内容 */
		*(CmdResp+1) =  (INT32U)MCIResponse1;
		*(CmdResp+2) =  (INT32U)MCIResponse2;
		*(CmdResp+3) =  (INT32U)MCIResponse3;
	}	
	
	return SD_NO_ERR;	                                    /* 响应正确. */
}



/**********************************

 下面部分为 MCI 中断处理相关函数

***********************************/


/*******************************************************************************************************************
** 函数名称: MCI_DataEndProcess()				
**
** 功能描述: MCI 数据结束处理函数
********************************************************************************************************************/
void MCI_DataEndProcess(void)
{
#if LPC23xx_MCI_DMA_EN
    if (lpc_mci->CurTran == MCI_CUR_SEND)
    {
        while ( !(DMACRawIntTCStatus & 0x01) );
	    DMACIntTCClear |= 0x01;
		lpc_mci->RxTxFlag = SD_NO_ERR;                       /* DMA 接收块数据成功 */
    }
    else
    {
        while ( !(DMACRawIntTCStatus & 0x02) );
		DMACIntTCClear |= 0x02;
		lpc_mci->RxTxFlag = SD_NO_ERR;                       /* DMA 发送块数据成功 */
    }
    
    #if SD_UCOSII_EN
	    OSSemPost(lpc_mci->psds->pSemSdmmcRW);               /* 使等待函数就绪 */
    #endif	

#else
   
    MCIClear |= MCI_DATA_BLK_END;
    if ((lpc_mci->SendCnt >= SD_BLOCKSIZE) || 
        (lpc_mci->RecCnt >= SD_BLOCKSIZE))	   
	{                                                       /* 块数据已收/发成功 */
	    lpc_mci->RxTxFlag = SD_NO_ERR;
	    #if SD_UCOSII_EN
	    	OSSemPost(lpc_mci->psds->pSemSdmmcRW);
		#endif
	} 

#endif	    
	MCIDataCtrl = 0;			                            /* 数据控制寄存器清0  */
}


/*******************************************************************************************************************
** 函数名称: MCI_TxActiveProcess()				
**
** 功能描述: MCI 发送数据处理函数
********************************************************************************************************************/
void MCI_TxActiveProcess(void)
{
	INT32U DataWord, blockSize,i;
	
	MCIClear |= MCI_TX_ACTIVE;
    if (lpc_mci->SendCnt < SD_BLOCKSIZE )
	{
	    blockSize = 0;
		if (MCIStatus & MCI_TX_FIFO_EMPTY)                  /* FIFO全空 */   
	        blockSize = FIFO_SIZE * 4;                      
		else if (MCIStatus & MCI_TX_HALF_EMPTY)             /* FIFO半空 */
		   	blockSize = (FIFO_SIZE / 2) * 4; 
		else if (!(MCIStatus & MCI_TX_FIFO_FULL))           /* FIFO全满 */
	     	blockSize = 1 * 4; 
		   
		for (i = 0; i < blockSize; i += 4)
    	{                                                   /* 循环向FIFO中写入数据 */
    		DataWord = ((INT32U)lpc_mci->pSendBuf[lpc_mci->SendCnt]) + 
    		           (((INT32U)lpc_mci->pSendBuf[lpc_mci->SendCnt + 1]) << 8) +
    		           (((INT32U)lpc_mci->pSendBuf[lpc_mci->SendCnt + 2]) << 16) +
    		           (((INT32U)lpc_mci->pSendBuf[lpc_mci->SendCnt + 3]) << 24);
    		MCIFIFO = DataWord;                             /* 向FIFO写入一个32位的数据 */
    		lpc_mci->SendCnt += 4;
    		if (lpc_mci->SendCnt >= (SD_BLOCKSIZE))          
     			break;	                                    /* 已发送完一个数据块 */
    	}     	    
    }
}


/*******************************************************************************************************************
** 函数名称: MCI_RxActiveProcess()				
**
** 功能描述: MCI 接收数据处理函数
********************************************************************************************************************/
void MCI_RxActiveProcess(void)
{
	INT32U DataWord, blockSize, i;
	INT32U Status;
	
    if (lpc_mci->RecCnt < SD_BLOCKSIZE )
	{
	    blockSize = 0;
	    if (!(MCIStatus & MCI_RX_FIFO_FULL))                 /* FIFO全满 */          
	   	  	blockSize = FIFO_SIZE * 4; 
	   	else if (MCIStatus & MCI_RX_HALF_FULL)               /* FIFO半满 */
	   		blockSize = (FIFO_SIZE/2) * 4; 
	   	else if (MCIStatus & MCI_RX_FIFO_EMPTY)              /* FIFO全空 */
	   		blockSize = 1 * 4; 
	   	
	   	for ( i = 0; i < blockSize; i += 4 )
	   	{                                                    /* 循环从FIFO中读取数据 */
		    while (!((Status =  (INT32U)MCIStatus) & MCI_RX_DATA_AVAIL))
		    {
		        if ((Status & MCI_RX_OVERRUN) != 0)          /* 判断是否发生FIFO下溢错误 */
		        {
		            MCIDataCtrl = 0;
		            MCIClear    = 0x7FF;
		            MCIMask0    = 0;
		            MCIMask1    = 0;
		            lpc_mci->RxTxFlag = SD_ERR_HARDWARE;
		            return;
		        }
		    }
		    DataWord =  (INT32U)MCIFIFO;                              /* 从FIFO中读取一个32位的数据 */
		    lpc_mci->pRecBuf[lpc_mci->RecCnt] = DataWord & 0xFF;
		    lpc_mci->pRecBuf[lpc_mci->RecCnt + 1] = (uint8)(DataWord >> 8) & 0xFF; 
		    lpc_mci->pRecBuf[lpc_mci->RecCnt + 2] = (DataWord >> 16) & 0xFF;
		    lpc_mci->pRecBuf[lpc_mci->RecCnt + 3] = (DataWord >> 24) & 0xFF;
		    lpc_mci->RecCnt += 4;                             /* 计数器计数 */
            if (lpc_mci->RecCnt >=  (INT32U)(SD_BLOCKSIZE))
     			break;		    
	    }
	}

}


/*******************************************************************************************************************
** 函数名称: MCI_DataInterruptProcess()				
**
** 功能描述: 这部分是处理读写卡的关键部分. 对于FIFO中断没有处理, 本函数主要使用 TX_ACTIVE 和 RX_ACTIVE 处理块读写.
********************************************************************************************************************/
void MCI_DataInterruptProcess( void )
{
	INT32U Status  =  (INT32U)MCIStatus;

 	if (Status &  MCI_DATA_END)                              /* 数据结束中断 */
 		MCIClear = Status | MCI_DATA_END;
	
	if (Status & MCI_DATA_BLK_END)                           /* 数据块结束中断 */
	{
	    MCI_DataEndProcess();
	    return;
	}
		
#if !LPC23xx_MCI_DMA_EN	
	if (MCIStatus & MCI_TX_ACTIVE)
        MCI_TxActiveProcess();                               /* 发送数据中断处理 */

	if (MCIStatus & MCI_RX_ACTIVE)
        MCI_RxActiveProcess();                               /* 接收数据中断处理 */
#endif        
}


/*******************************************************************************************************************
** 函数名称: MCI_DataErrorProcess()				
**
** 功能描述: MCI 错误处理函数: 当MCI有错误产生时, 该函数被调用
********************************************************************************************************************/
void MCI_DataErrorProcess( void )
{
	INT32U Status =  (INT32U)MCIStatus;
	
    if (Status &  (INT32U)MCI_DATA_CRC_FAIL)
		lpc_mci->RxTxFlag = SD_ERR_DATA_CRC16;                /* 收/发数据块时发生CRC校验错误 */         

	if (Status & MCI_DATA_TIMEOUT)
	{
        if (lpc_mci->CurTran == MCI_CUR_SEND)
            lpc_mci->RxTxFlag = SD_ERR_TIMEOUT_WRITE;         /* 发送数据超时错误 */
        else    
            lpc_mci->RxTxFlag = SD_ERR_TIMEOUT_READ;          /* 接收数据超时错误 */
    }
    	
	if (Status & MCI_TX_UNDERRUN)                            /* 发送数据时FIFO下溢错误 */
	    lpc_mci->RxTxFlag = SD_ERR_HARDWARE;

	if (Status & MCI_RX_OVERRUN)                             /* 接收数据时FIFO上溢错误 */
	    lpc_mci->RxTxFlag = SD_ERR_HARDWARE;

	if ( Status &  MCI_START_BIT_ERR )                       /* 宽总线模式下所有数据信号线上没有检测到起始位 */
	    lpc_mci->RxTxFlag = SD_ERR_DATA_STARTBIT;
	
#if SD_UCOSII_EN
	OSSemPost(lpc_mci->psds->pSemSdmmcRW);                    /* 使等待函数就绪 */
#endif
}


/*******************************************************************************************************************
** 函数名称: MCI_Exception	或 MCI_Handler		
**
** 功能描述: MCI 中断处理函数
********************************************************************************************************************/

void   MCI_Exception(void)
{	   
    INT32U Status =  (INT32U)MCIStatus;                              /* 读取MCI当前状态 */

    if (Status & CMD_INT_MASK)
	{                                                       /* 命令中断处理 */
		MCIClear = Status & CMD_INT_MASK;
	}
	else if (MCIStatus & DATA_ERR_INT_MASK)
	{                                                       /* 数据错误中断处理 */
		MCI_DataErrorProcess();                 
		MCIClear |= DATA_ERR_INT_MASK;                      /* 清除数据错误中断位 */
	}
	else if (MCIStatus & DATA_INT_MASK)
	{                                                       /* 数据中断处理 */
		MCI_DataInterruptProcess();
		//MCIClear |= DATA_INT_MASK;
	}
    else if (MCIStatus & FIFO_INT_MASK)
	{
		;
	}
	
	MIC_SR |= 13;
}



/*************************
LPC23xx DMA 初始化函数
**************************/

#if LPC23xx_MCI_DMA_EN

/*******************************************************************************************************************
** 函数名称: DMA_Init()				
**
** 功能描述: LPC23xx DMA控制器初始化
**
** 输　  入: INT32U ChannelNum: DMA 控制器通道号 
**	         INT32U DMAMode:    DMA 模式
** 输 　 出: 无
**
** 返 回 值: TRUE:   正确;    FALSE:   错误
********************************************************************************************************************/
INT32U DMA_Init(INT32U ChannelNum, INT32U DMAMode)
{
	/* USB RAM is used for test.
	Please note, Ethernet has its own SRAM, but GPDMA can't access
	that. GPDMA can access USB SRAM and IRAM. Ethernet DMA controller can 
	access both IRAM and Ethernet SRAM. */
	if ( ChannelNum == 0 )
	{
		DMACIntErrClr = 0x01;   
		if ( DMAMode == M2M )
		{
			/* Ch0 is set for M2M tranfer from AHB1 to AHB2 SRAM */
			DMACC0SrcAddr = DMA_SRC;
			DMACC0DestAddr = DMA_DST;
			/* Terminal Count Int enable */
			DMACC0Control = (DMA_SIZE & 0x0FFF) | (0x04 << 12) | (0x04 << 15) 
				| (0x02 << 18) | (0x02 << 21) | (1 << 26) | (1 << 27) | 0x80000000;	
		}
		else if ( DMAMode == M2P )
		{
			/* Ch0 set for M2P transfer from mempry to MCI FIFO. */
			DMACC0SrcAddr = DMA_SRC;
			DMACC0DestAddr = DMA_MCIFIFO;
			/* The burst size is set to 8, the size is 8 bit too. */
			/* Terminal Count Int enable */
			DMACC0Control = (DMA_SIZE & 0x0FFF) | (0x01 << 12) | (0x02 << 15)
				| (0x02 << 18) | (0x02 << 21) | (1 << 26) | 0x80000000;
		}
		else if ( DMAMode == P2M )
		{
			/* Ch0 set for P2M transfer from MCI FIFO to memory. */
			DMACC0SrcAddr = DMA_MCIFIFO;
			DMACC0DestAddr = DMA_DST;
			/* The burst size is set to 8, the size is 8 bit too. */
			/* Terminal Count Int enable */
			DMACC0Control = (DMA_SIZE & 0x0FFF) | (0x04 << 12) | (0x04 << 15)
				| (0x02 << 18) | (0x02 << 21) | (1 << 27) | 0x80000000;
		}
		else
		{
			return ( FALSE );
		}
	}
	else if ( ChannelNum == 1 )
	{   
		DMACIntErrClr = 0x02;
		if ( DMAMode == M2M )
		{
			/* Ch1 is set for M2M tranfer */
			DMACC1SrcAddr = DMA_SRC;
			DMACC1DestAddr = DMA_DST;
			/* Terminal Count Int enable */
			DMACC1Control = (DMA_SIZE & 0x0FFF) | (0x04 << 12) | (0x04 << 15) 
				| (0x02 << 18) | (0x02 << 21) | (1 << 26) | (1 << 27) | 0x80000000;	
		}
		else if ( DMAMode == M2P )
		{
			/* Ch1 set for M2P transfer from mempry to MCI FIFO. */
			DMACC1SrcAddr = DMA_SRC;
			DMACC1DestAddr = DMA_MCIFIFO;
			/* The burst size is set to 8, the size is 8 bit too. */
			/* Terminal Count Int enable */
			DMACC1Control = (DMA_SIZE & 0x0FFF) | (0x04 << 12) | (0x04 << 15)
				| (0x02 << 18) | (0x02 << 21) | (1 << 26) | 0x80000000;
		}
		else if ( DMAMode == P2M )
		{
			/* Ch1 set for P2M transfer from MCI_FIFO to memory. */
			DMACC1SrcAddr = DMA_MCIFIFO;
			DMACC1DestAddr = DMA_DST;
			/* The burst size is set to 8, the size is 8 bit too. */
			/* Terminal Count Int enable */
			DMACC1Control = (DMA_SIZE & 0x0FFF) | (0x02 << 12) | (0x02 << 15)
				| (0x02 << 18) | (0x02 << 21) | (1 << 27) | 0x80000000;
		}
		else
		{
			return ( FALSE );
		}
	}
	else
	{
		return ( FALSE );
	}
	
	DMACConfiguration = 0x01;	/* Enable DMA channels, little endian */
	while ( !(DMACConfiguration & 0x01) );    
    return (TRUE);
}

#endif



#endif




