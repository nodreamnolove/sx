/****************************************Copyright (c)****************************************************
**                         Guangzhou ZHIYUAN electronics Co.,LTD.                               
**                                     
**                               http://www.embedtools.com
**
**--------------File Info---------------------------------------------------------------------------------
** File Name:           config.h
** Last modified Date:  2004-09-17
** Last Version:        1.0
** Descriptions:        User Configurable File
**
**--------------------------------------------------------------------------------------------------------
** Created By:          Chenmingji
** Created date:        2004-09-17
** Version: 1.0
** Descriptions:        First version
**
**--------------------------------------------------------------------------------------------------------
** Modified by:  		LinEnqiang
** Modified date: 		2007-12-15
** Version: 1.0
** Descriptions: 		For LPC3200
**
*********************************************************************************************************/
//#include "Uart5.h"
//#include "crc.h"
//#include "cmd.h"
//#include "LCDApp.h"

#ifndef __CONFIG_H
#define __CONFIG_H

#ifdef  __cplusplus
extern "C" {
#endif

#ifndef TRUE
#define TRUE  1
#endif                                                                  /*  TRUE                        */

#ifndef FALSE
#define FALSE 0
#endif                                                                  /*  FALSE                       */

typedef unsigned char  uint8;                                           /*  无符号8位整型变量           */
typedef signed   char  int8;                                            /*  有符号8位整型变量           */
typedef unsigned short uint16;                                          /*  无符号16位整型变量          */
typedef signed   short int16;                                           /*  有符号16位整型变量          */
typedef unsigned int   uint32;                                          /*  无符号32位整型变量          */
typedef signed   int   int32;                                           /*  有符号32位整型变量          */
typedef float          fp32;                                            /*  单精度浮点数（32位长度）    */
typedef double         fp64;                                            /*  双精度浮点数（64位长度）    */

typedef unsigned char  INT8U;                                           /*  无符号8位整型变量           */
typedef signed   char  INT8S;                                           /*  有符号8位整型变量           */
typedef unsigned short INT16U;                                          /*  无符号16位整型变量          */
typedef signed   short INT16S;                                          /*  有符号16位整型变量          */
typedef unsigned int   INT32U;                                          /*  无符号32位整型变量          */
typedef signed   int   INT32S;                                          /*  有符号32位整型变量          */
typedef float          FP32;                                            /*  单精度浮点数（32位长度）    */
typedef double         FP64;                                            /*  双精度浮点数（64位长度）    */

/*********************************************************************************************************
  ARM的特殊代码
  ARM specital code
*********************************************************************************************************/
#include    "LPC3200.h"
#include    "target.h"
#include    "vicControl.h"

/*********************************************************************************************************
  应用程序配置
  Application Program Configurations
*********************************************************************************************************/
#include    <stdio.h>
#include    <rt_sys.h>
#include    <ctype.h>
#include    <stdlib.h>
#include    <setjmp.h>
#include    <rt_misc.h>	  
#include	<string.h>
#include	<math.h>

#define		YBVERSION	30
#define		TEST_PROBE	1		//20131127
#include	"DEFConfig.h"

#define		UBR_4800	4800		//串口波特率
#define		UBR_9600	9600		//串口波特率
#define		UBR_19200	19200		//串口波特率
#define		UBR_38400	38400		//串口波特率
#define		UBR_57600	57600		//串口波特率
#define		UBR_115200	115200		//串口波特率
#define		UBR_230400	230400		//串口波特率
#define		UBR_460800	460800		//串口波特率

#define		UARTDLM				0                                                
#define		UARTDLL				1

#include	"Includes.h"
#include	"JZGlobal.h"
#include	"GPIO.h"
#include    "jg.h"


/*********************************************************************************************************
  本例子的配置
  Configuration of the example
*********************************************************************************************************/
#define	USE_USB			0
#define	USE_PLL397      0


#define Fosc            13000000					                    /*	OSC = 13.0MHz               */

#define Fuclk	        48000000    

#define Fcclk           (Fosc * 16)                                     /* 不高于208MHz                 */

#define Fcco            (Fcclk * 1)                                     /* 
                                                                         * 必须在156MHz 到320MHz 范围内， 
                                                                         * Fcco必须是Fcclk的1、2、4、8、
                                                                         * 16倍
                                                                         */

#define Fhclk           (Fcclk / 2)					                    /* 
                                                                         * Fhclk 通常是Fcclk 的 2 分频
                                                                         * 但可以在与Fcclk 相同的频率或 
                                                                         * Fcclk 的 4 分频下运行。
                                                                         * Fhclk频率设置不应高于104MHz 
                                                                         * 或低于Fosc
                                                                         */

#define Fpclk	        (Fcclk / 16)

#define Fdclk	        (Fhclk * 0)                        	            /* 
                                                                         * 如果使用DDR SDRAM,该时钟必须
                                                                         * 编程为 2 倍 Fhclk频率,且Fhclk
                                                                         * 为Fcclk的2、4分频,否则为,0
                                                                         */
                                                                         
#define Fsclk	        (Fcclk / 8)                                     /* SD卡时钟,基于Fcclk的1~15分频 */


#if (USE_PLL397 == 1)&&(USE_USB == 1)
   #error  "USB couldn't be used together with PLL397"                        
#endif
#if Fcclk > 208000000
   #error  "Fcclk couldn't be higher than 208MHz"                        
#endif

/*
 * 判断FCCO参数是否正确
 */
#if !((Fcco / Fcclk == 1) || (Fcco / Fcclk == 2) ||   \
      (Fcco / Fcclk == 4) || (Fcco / Fcclk == 8) ||   \
      (Fcco / Fcclk == 16)) 
     #error  "The parameter Fcco is incorrect! "  
#endif
      
       
      

#ifdef __cplusplus
    }
#endif                                                                  /*  __cplusplus                 */
#endif                                                                  /*  __CONFIG_H                  */ 
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
