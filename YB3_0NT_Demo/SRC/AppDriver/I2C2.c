/****************************************Copyright (c)****************************************************
**                         Guangzhou ZHIYUAN electronics Co.,LTD.                               
**                                     
**                               http://www.embedtools.com
**
**--------------File Info---------------------------------------------------------------------------------
** File name:           I2CINT.c
** Last modified Date:  2009-02-03
** Last Version:        1.00
** Descriptions:        for lpc3200
**--------------------------------------------------------------------------------------------------------
** Created by:          Lilingyu
** Created date:        2009-02-03
** Version:             1.00
** Descriptions:        The original version
*********************************************************************************************************/
#define	__I2C2_C
#include "I2C2.h"

#define I2C_CLK_100K 1
#define I2C_CLK_400K 4

#define  _BIT(x)     (1 << (x))


static	void	I2C2_Excetion(void);		//中断处理

static	uint8	Wait_I2c_End(uint32  dly);

/**********************************************************************
* i2c_txrx register definitions
**********************************************************************/
#define I2C_START_FLAG    _BIT(8)                                       /* generate a START before      */
                                                                        /* this B                       */
#define I2C_STOP_FLAG     _BIT(9)                                       /* generate a STOP after this B */

/*********************************************************************************************************
* i2c_stat register definitions
*********************************************************************************************************/
#define I2C_TDI     _BIT(0)                                             /* Transaction Done Interrupt   */
#define I2C_AFI     _BIT(1)                                             /* Arbitration Failure Interrupt*/
#define I2C_NAI     _BIT(2)                                             /* No Acknowledge Interrupt     */
#define I2C_DRMI    _BIT(3)                                             /* Master Data Request Interrupt*/
#define I2C_DRSI    _BIT(4)                                             /* Slave Data Request Interrupt */
#define I2C_ACTIVE  _BIT(5)                                             /* Busy bus indicator           */
#define I2C_SCL     _BIT(6)                                             /* The current SCL signal value */
#define I2C_SDA     _BIT(7)                                             /* The current SDA signal value */
#define I2C_RFF     _BIT(8)                                             /* Receive FIFO Full            */
#define I2C_RFE     _BIT(9)                                             /* Receive FIFO Empty           */
#define I2C_TFF     _BIT(10)                                            /* Transmit FIFO Full           */
#define I2C_TFE     _BIT(11)                                            /* Transmit FIFO Empty          */
#define I2C_TFFS    _BIT(12)                                            /* Slave Transmit FIFO Full     */
#define I2C_TFES    _BIT(13)                                            /* Slave Transmit FIFO Empty    */

/*********************************************************************************************************
* i2c_ctrl register definitions
*********************************************************************************************************/
#define I2C_TDIE    _BIT(0)                                             /* Transaction Done Int Enable  */
#define I2C_AFIE    _BIT(1)                                             /* Arbitration Failure Int Ena  */
#define I2C_NAIE    _BIT(2)                                             /* No Acknowledge Int Enable    */
#define I2C_DRMIE   _BIT(3)                                             /* Master Data Request Int Ena  */
#define I2C_DRSIE   _BIT(4)                                             /* Slave Data Request Int Ena   */
#define I2C_RFFIE   _BIT(5)                                             /* Receive FIFO Full Int Ena    */
#define I2C_RFDAIE  _BIT(6)                                             /* Rcv Data Available Int Ena   */
#define I2C_TFFIE   _BIT(7)                                             /* Trnsmt FIFO Not Full Int Ena */
#define I2C_RESET   _BIT(8)                                             /* Soft Reset                   */
#define I2C_SEVEN   _BIT(9)                                             /* Seven-bit slave address      */
#define I2C_TFFSIE  _BIT(10)                                            /* Slave Trnsmt FIFO Not Full IE*/

/*********************************************************************************************************
* 执行结果
*********************************************************************************************************/
#define      I2C_BEGIN                0x01
#define   I2C_DONE               0x02
#define   I2C_NAK                  0x03
#define   I2C_ARF                0x04
#define   I2C_ERR               0xFF

/*********************************************************************************************************
* I2C操作码
*********************************************************************************************************/
#define   I2C_CMD_WR_EX_ADDR    0x01
#define   I2C_CMD_WR_DATA       0x02
#define   I2C_CMD_RD_DATA       0x03

#define    ONE_BYTE_SUBA    1
#define TWO_BYTE_SUBA    2
#define X_ADD_8_SUBA    3

/*********************************************************************************************************
定义用于和I2C中断传递信息的全局变量 
*********************************************************************************************************/
volatile uint16    I2C2_sla;                                             /* I2C器件地址                  */
volatile uint8     I2C2_Status;                                          /* I2C状态标志                  */
volatile uint8     *I2C2_recvbuf;                                        /* 读出数据存储地址             */
volatile uint8     *I2C2_sendbuf;                                        /* 写入数据存储地址             */
volatile uint32 	I2C2_recvnum;                                        /* 读出数据个数                 */
volatile uint32 	I2C2_sendnum;                                        /* 写入数据个数                 */
volatile uint32 	I2C2_recvNullnum;                                    /* 需要读出的数据个数           */


/****************************************************************************************************************
* 函数名称：I2CInit()
* 函数功能：I2C 初始化
* 入口参数：i2c_clk        I2C时钟可选参数:I2C_CLK_100K,I2C_CLK_400K
* 返回参数：TURE | FALSE
*****************************************************************************************************************/
uint8 I2C2Init(uint8 i2c_clk)
{
    I2CCLK_CTRL = 0x03;                                                 /* 使能I2C                      */
    I2C2_CTRL = (1 << 8);                                               /* Rset I2C                     */
    while ((I2C2_CTRL & (1 << 8)) != 0);								/*复位后该位为复位值为0*/
    
    if (i2c_clk  ==  I2C_CLK_100K){                                     /* 设置I2C时钟为100K            */
        I2C2_CLKHI    = 520;                                            /* HCLK = 104M                  */
        I2C2_CLKLO    = 520;
    }else if (i2c_clk  ==  I2C_CLK_400K){                               /* 设置I2C时钟为400K            */
        I2C2_CLKHI    = 96;
        I2C2_CLKLO    = 166;
    }else{
        return FALSE;
    }
    return((uint8)sic1IrqFuncSet(18, 2, (unsigned int)I2C2_Excetion));   /* I2C1_INT:19 I2C2_INT:18      */
}


/****************************************************************************************************************
* 函数名称：Wait_I2c_End
* 函数功能：软件延时，提供给I2C总线等待超时使用
* 入口参数：dly        延时参数，值越大，延时越久
* 出口参数：TURE | FALSE
*****************************************************************************************************************/
uint8  Wait_I2c_End(uint32  dly)
{
    uint32  i;
    if (I2C2_Status == I2C_DONE)
    return (TRUE);
    for (; dly > 0; dly--)
        for (i = 0; i < 50000; i++){
            if (I2C2_Status == I2C_DONE) return (TRUE);
        }
    return (FALSE);
}

/**********************************************************************************************************
** 函数名称 ：I2C_ReadNByte
** 函数功能 ：从有子地址器件任意地址开始读取N字节数据
** 入口参数 ：sla            器件从地址
**            suba_type      子地址结构    1－单字节地址    2－双字节地址    n－多字节地址
**            suba           器件内部子地址
**            *s             将要写入的数据的指针
**            num            将要写入的数据的个数
** 出口参数 ：TURE | FALSE
**********************************************************************************************************/
uint8 I2C2_ReadNByte (uint16 sla, uint8 suba_type, uint32 suba, uint8 *tp, uint32 num)
{
    uint8 err;
    err = TRUE;
    
    while (I2C2_STAT & I2C_ACTIVE);                                     /* I2C总线是否空闲？            */
    I2C2_STAT = 0x3;                                                    /* 清AFI、TDI标志               */
    I2C2_CTRL = (1 << 8);                                               /* Rset i2c                     */
    while ((I2C2_CTRL & (1 << 8)) != 0);                                /* 等待复位                     */
    
    if (num > 0)                                                        /* 判断num个数的合法性          */
    {
        I2C2_Status = I2C_BEGIN;                                         /* I2C状态设置                  */
        if (suba_type){                                                 /* 是否为有子地址读操作         */
            /*
             * 发送子地址 
             */
            I2C2_sendnum = suba_type;                                    /* 子地址字节个数               */
            I2C2_sendbuf = (uint8*) & suba;
            I2C2_TX = (sla & 0xfe) | I2C_START_FLAG;                    /* 发送器件地址启动I2C总线      */
            I2C2_CTRL = (I2C_TFFIE |                                    /* 发送FIFO未满中断             */
                         I2C_NAIE  |                                    /* 发送无应答中断               */
                         I2C_AFIE  |                                    /* 仲裁失败中断                 */
                         I2C_TDIE );                                    /* 处理完成中断                 */
            err  &= Wait_I2c_End(10);
            /*
             * 读数据 
             */
            I2C2_recvNullnum = num;
            I2C2_recvnum = 0;
            I2C2_recvbuf = tp;
            I2C2_Status = I2C_BEGIN;
            while (I2C2_STAT & I2C_TFF);                                /* 发送FIFO是否未满             */
            I2C2_TX = ((sla|0x01) | I2C_START_FLAG);                    /* 再次启动I2C总线              */
            I2C2_CTRL |= (I2C_RFDAIE|                                   /* 使能I2C读数据有效中断        */
            I2C_TFFIE);                                                 /* 使能发送FIFO未满中断         */
        }else{                                                          /* 无子地址读                   */
            I2C2_recvNullnum = num;
            I2C2_recvnum = 0;
            I2C2_recvbuf = tp;
            while (I2C2_STAT & I2C_TFF);                                /* 发送FIFO是否未满             */
            I2C_TX = sla|I2C_START_FLAG;
            I2C2_CTRL = (I2C_RFDAIE |                                   /* 接收数据可用中断             */
            I2C_TFFIE  |                                                /* 发送FIFO未满中断             */
            I2C_NAIE   |                                                /* 发送无应答中断               */
            I2C_AFIE   |                                                /* 仲裁失败中断                 */
            I2C_TDIE );                                                 /* 处理完成中断                 */
        }
        while ((I2C2_STAT & 0x01)== 0);                                /* 等待I2C传输结束              */
        return err;
    }
    return (FALSE);
}

/*********************************************************************************************************
** 函数名称 ：I2C_WriteNByte
** 函数功能 ：向有子地址器件写入N字节数据
** 入口参数 ：sla         器件从地址
**            suba_type   子地址结构    1－单字节地址    3－8+X结构    2－双字节地址
**            suba        器件内部物理地址
**            *s          将要写入的数据的指针
**            num         将要写入的数据的个数
** 出口参数 ：TRUE        操作成功
**            FALSE       操作失败
*********************************************************************************************************/
uint8 I2C2_WriteNByte(uint16 sla, uint8 suba_type, uint32 suba, uint8 *tp, uint32 num)
{
    uint8 err;
    err = TRUE;
    
    while (I2C2_STAT & I2C_ACTIVE);                                     /* I2C总线是否空闲？            */
    I2C2_STAT = 0x3;                                                    /* 清AFI、TDI标志               */
    I2C2_CTRL = (1 << 8);                                               /* Rset i2c                     */
    while ((I2C2_CTRL & (1 << 8)) != 0);                                /* 等待复位                     */
    
    if (num > 0)                                                        /* 判断num个数的合法性          */
    {
        I2C2_Status = I2C_BEGIN;                                         /* 设置I2C状态                  */
        if (suba_type)
        {                                                 /* 是否为有子地址写操作         */
            /*
             * 发送器件从地址 
             */
            I2C2_sendnum  = suba_type;
            I2C2_sendbuf  = (uint8*) &suba;
            I2C2_TX = (sla & 0xfe) | I2C_START_FLAG;                    /* 发送器件地址启动I2C总线      */
            I2C2_CTRL = (I2C_TFFIE |                                    /* 发送FIFO未满中断             */
                         I2C_NAIE  |                                    /* 发送无应答中断               */
                         I2C_AFIE  |                                    /* 仲裁失败中断                 */
                         I2C_TDIE );                                    /* 处理完成中断                 */
            err  &= Wait_I2c_End(10);
            /*
             * 发送数据 
             */
            I2C2_sendbuf  = tp;
            I2C2_sendnum  = num-1;
            I2C2_Status = I2C_BEGIN;
            if (num  ==  1){
                while (I2C2_STAT & I2C_TFF);                            /* 发送FIFO是否满               */
                I2C2_TX = ((*I2C2_sendbuf)|I2C_STOP_FLAG);
            }
            else{
                I2C2_CTRL |= I2C_TFFIE;                                 /* 使能发送FIFO未满中断         */
                                                                        /* 再次发送数据                 */
                err  &= Wait_I2c_End(20);
                while (I2C2_STAT & I2C_TFF);                            /* 发送FIFO是否未满             */
                I2C2_TX = ((*I2C2_sendbuf)|I2C_STOP_FLAG);               /* 发送最后一个数据与停止位     */
            }
        }
        else{                                                           /* 无子地址写                   */
            I2C2_CTRL = (I2C_TFFIE |                                    /* 发送FIFO未满中断             */
                         I2C_NAIE  |                                    /* 发送无应答中断               */
                         I2C_AFIE  |                                    /* 仲裁失败中断                 */
                         I2C_TDIE );                                    /* 处理完成中断                 */
            I2C2_sendnum = num-1;
            I2C2_sendbuf = tp;
            I2C2_TX = sla|I2C_START_FLAG;                               /* 发送器件地址启动I2C总线      */
            err  &= Wait_I2c_End(20);
            while (I2C2_STAT & I2C_TFF);                                /* 发送FIFO是否未满             */
            I2C2_TX = ((*I2C2_sendbuf)|I2C_STOP_FLAG);                   /* 发送最后一个数据与停止位     */
        }
        return err;
    }
    return FALSE;
}

/**********************************************************************************************************
** 函数名称 ：I2C_Excetion
** 函数名次 ：硬件I2C中断服务程序
** 入口参数 ：无
** 出口参数 ：无
** 说    明 ：无
**********************************************************************************************************/
void I2C2_Excetion(void)
{
    uint32 sts;
    uint32 bsend = 0;
    
    while ((I2C2_STAT & (1 << 9))  ==  0){                              /* 接收FIFO不为空               */
        *I2C2_recvbuf++ =I2C2_RX;                                        /* 读取数据                     */
        I2C2_recvnum++;
    }
    while ((bsend<I2C2_sendnum) && ((I2C2_STAT & (1 << 10))== 0)){      /* 发送FIFO未满且仍有数据等待   */
                                                                        /* 发送                         */
        I2C2_TX = (uint32)*I2C2_sendbuf++;
        bsend++;
    }
    I2C2_sendnum -= bsend;
    while ((I2C2_recvNullnum > 0) && ((I2C2_STAT & (1 << 10))== 0)){    /* 发送FIFO未满且仍有数据需要   */
                                                                        /* 接收                         */
        if (I2C2_recvNullnum  ==  1){                                    /* 是否为最后一个需要接收的数据 */
            I2C2_TX = (0xff|I2C_STOP_FLAG);                             /* 发送停止位                   */
        }else{
            I2C2_TX = 0xff;
        }
        I2C2_recvNullnum--;
    }
    if ((I2C2_sendnum + I2C2_recvNullnum)== 0){                          /* 如果没有数据需要发送         */
        I2C2_CTRL  &= ~I2C_TFFIE;                                      /* 禁止发送FIFO未满中断         */
        I2C2_Status = I2C_DONE;
    }
    sts = I2C2_STAT;                                                    /* 更新I2C状态                  */
    if ((sts & (1 << 5)) != 0){

    }else if ((sts & (1 << 2))  !=  0){                                 /* 无应答                       */
        I2C2_Status = I2C_NAK;
        I2C2_CTRL = 0x00;                                               /* 禁止中断                     */
    }else if ((sts & (1 << 0)) != 0){                                   /* 已发送停止位                 */
        I2C2_Status = I2C_DONE;
        while ((I2C2_STAT & (1 << 9))== 0){                            /* 接收FIFO未空                 */
            *I2C2_recvbuf++ = I2C_RX;
            I2C2_recvnum++;
        }
        I2C2_CTRL = 0;                                                  /* 禁止中断                     */
    }else{
        I2C2_Status = I2C_ERR;
        I2C2_CTRL = 0;
    }
}
