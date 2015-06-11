;/****************************************Copyright (c)***************************************************
**                         Guangzhou ZHIYUAN electronics Co.,LTD.                               
**                                     
**                               http://www.embedtools.com
;**
;**--------------File Info--------------------------------------------------------------------------------
;** File name: 			Startup.s
;** Last modified Date:  
;** Last Version: 		
;** Descriptions: 		The start up codes for LPC2100, including the initializing codes for the entry 
;**                     point of exceptions and the stacks of user tasks. Every project should have a 
;**                     independent copy of this file for related modifications
;**-------------------------------------------------------------------------------------------------------
;** Created by: 		Chenmingji
;** Created date:   	2004-02-02
;** Version:			1.0
;** Descriptions: 		The original version
;**-------------------------------------------------------------------------------------------------------
;** Modified by: 		LinEnqiang
;** Modified date:		2008-12-12	
;** Version:			1.01
;** Descriptions: 		增加中断管理支持 for LPC3200
;**
;********************************************************************************************************/
                                INCLUDE     LPC3200.INC                 ;/*  引入头文件                 */
                                                                        ;/*  定义堆栈的大小             */
SVC_STACK_LEGTH     EQU         0
FIQ_STACK_LEGTH     EQU         10
IRQ_STACK_LEGTH     EQU         9 * 8                                   ;/*  每层嵌套需要8个字堆栈，允许*/
                                                                        ;/*  9嵌套层                    */
ABT_STACK_LEGTH     EQU         0
UND_STACK_LEGTH     EQU         0

NoInt               EQU         0x80                                    ;/*  禁止 IRQ 中断宏定义        */
NoFIQ		        EQU	        0x40                                    ;/*  禁止 FIQ 中断宏定义        */

USR32Mode           EQU         0x10                                    ;/*  用户模式宏定义             */
SVC32Mode           EQU         0x13                                    ;/*  管理模式宏定义             */
SYS32Mode           EQU         0x1f                                    ;/*  系统模式宏定义             */
IRQ32Mode           EQU         0x12                                    ;/*  IRQ模式宏定义              */
FIQ32Mode           EQU         0x11                                    ;/*  FIQ模式宏定义              */

SNorFlahMW_08       EQU         0x13579BD0                              ;/*  外部存储器 8位宽度有效字   */
SNorFlahMW_16       EQU         0x13579BD1                              ;/*  外部存储器16位宽度有效字   */
SNorFlahMW_32       EQU         0x13579BD2                              ;/*  外部存储器32位宽度有效字   */


VFP_EN_BIT      	EQU     	(1<<30)         ; VFP Enable Bit

;/********************************************************************************************************
; The imported labels    
; 引入的外部标号在这声明
;********************************************************************************************************/
    IMPORT  __main                                                      ;/*  C语言主程序入口            */        
    IMPORT  vicControl
    
    IMPORT  IRQ_Handler
    IMPORT  FIQ_Handler  
    IMPORT  TargetResetInit                                             ;/*  目标板基本初始化           */    
    IMPORT	SoftwareInterrupt    
    IMPORT __use_two_region_memory
    IMPORT __use_no_semihosting_swi
    
;/********************************************************************************************************
; The emported labels    
; 给外部使用的标号在这声明
;********************************************************************************************************/

    EXPORT  Reset    
	EXPORT  StackUsr   
	EXPORT  InitStack
	EXPORT  bottom_of_heap  
	EXPORT  SoftwareInterruptAdd
	EXPORT  __user_initial_stackheap
	
    CODE32
	PRESERVE8
    AREA    vectors,CODE,READONLY
ENTRY
Reset              ;/*  中断向量表                 */
                

      LDR         PC, ResetAddr
        LDR     	PC, UndefinedAddr
        LDR    	 	PC, SWI_Addr
        LDR     	PC, PrefetchAddr
        LDR     	PC, DataAbortAddr
        DCD         0
        LDR     	PC, IRQ_Addr
        LDR     	PC, FIQ_Addr

ResetAddr           DCD     ResetInit
UndefinedAddr       DCD     Undefined
SWI_Addr            DCD     SoftwareInterruptAdd
PrefetchAddr        DCD     PrefetchAbort
DataAbortAddr       DCD     DataAbort
Nouse               DCD     0
IRQ_Addr            DCD     IRQ_Handler
FIQ_Addr            DCD     FIQ_Handler

Undefined
        B       .	  
PrefetchAbort
        B       .
DataAbort
        B       .          
                
SoftwareInterruptAdd 
        CMP     R0, #0x100
        BLO     SoftwareInterrupt
        B       vicControl    

;/********************************************************************************************************
;** Function name:          InitStack
;**
;** Descriptions:           初始化堆栈
;**
;** input parameters:       None
;** Returned value:         None
;**
;** Created by:             Chenmingji
;** Created Date:           2004/02/02
;**-------------------------------------------------------------------------------------------------------
;** Modified by:            
;** Modified date:          
;**-------------------------------------------------------------------------------------------------------
;********************************************************************************************************/
InitStack    
        MOV     R0, LR                
;Build the SVC stack
;设置管理模式堆栈
        MSR     CPSR_c, #0xd3		
        LDR     SP, StackSvc
;Build the IRQ stack
;设置中断模式堆栈
        MSR     CPSR_c, #0xd2
        LDR     SP, StackIrq
;Build the FIQ stack	
;设置快速中断模式堆栈
        MSR     CPSR_c, #0xd1
        LDR     SP, StackFiq
;Build the DATAABORT stack
;设置中止模式堆栈
        MSR     CPSR_c, #0xd7
        LDR     SP, StackAbt
;Build the UDF stack
;设置未定义模式堆栈
        MSR     CPSR_c, #0xdb
        LDR     SP, StackUnd
;Build the SYS stack
;设置系统模式堆栈
        MSR     CPSR_c, #0xd0
        LDR     SP, =StackUsr              
        BX		R0

;/********************************************************************************************************
;** Function name:          ResetInit
;**
;** Descriptions:           复位处理
;**
;** input parameters:       None
;** Returned value:         None
;**
;** Created by:             Chenmingji
;** Created Date:           2004/02/02
;**-------------------------------------------------------------------------------------------------------
;** Modified by:            
;** Modified date:          
;**-------------------------------------------------------------------------------------------------------
;********************************************************************************************************/
ResetInit  
     	IF :DEF: __FLASHMODE		;/* 如果在SDRam中运行，不需要操作缓存			*/
		B	_SDRam		;/* 跳转至_SDRam			*/
		ENDIF

        ; /*
        ;  * 关闭内部看门狗时钟
        ;  */
		LDR   R0, =TIMCLK_CTRL
		MOV   R1, #0
		STR   R1, [R0]

		; Enable VFP
  		IF :DEF: __VFPENABLE
		
		MOV     R1, #VFP_EN_BIT ; Enable VFP
        FMXR    FPEXC, R1
  
		ENDIF
		
        ; /*
        ;  * 关闭MMU、Cache和写缓冲
        ;  */
        MRC   p15, 0, r0, c1, c0, 0         
        BIC   r0 ,r0 ,#1                                                ;/*  清零M位，禁止MMU           */
        BIC   r0 ,r0 ,#4                                                ;/*  清零D cache，禁止数据Cache */
        BIC   r0 ,r0 ,#0x1000                                           ;/*  清零I CACHE，禁止指令Cache */  
        MCR   p15, 0, r0, c1, c0 ,0
        
        MOV   r0, #0
        MCR   p15, 0, r0, c7, c7, 0                                     ;/*  cache无效                  */

        ; Invalidate I cache
		mov     r0, #0
		mcr     p15, 0, r0, c7, c5, 0
       
        ; Enable I and D caches and write buffer
		mrc     p15, 0, r0, c1, c0, 0
		orr     r0, r0, #(1 :SHL: 12)           ; I-cache
		orr     r0, r0, #(1 :SHL: 2)            ; D-cache
		orr     r0, r0, #(1 :SHL: 3)            ; write buffer
		mcr     p15, 0, r0, c1, c0, 0 

_SDRam       
		BL      InitStack               		                        ;/*  初始化堆栈                 */
		BL      TargetResetInit         		                        ;/*  目标板基本初始化           */       

		B       __main                                                  ;/*  跳转到c语言入口            */
                    		                            
;/********************************************************************************************************
;** Function name:          __user_initial_stackheap
;**
;** Descriptions:           库函数初始化堆和栈，不能删除
;**
;** input parameters:       参考库函数手册
;** Returned value:         参考库函数手册
;**
;** Created by:             Chenmingji
;** Created Date:           2004/02/02
;**-------------------------------------------------------------------------------------------------------
;** Modified by:            
;** Modified date:          
;**-------------------------------------------------------------------------------------------------------
;********************************************************************************************************/
__user_initial_stackheap    
    LDR   R0, =bottom_of_heap		
    ;LDR   R1, =StackUsr			
    LDR   R2, =top_of_heap		
    LDR   R3, =bottom_of_Stacks	      
    BX		LR

    EXPORT _sys_exit
_sys_exit
    B       .

    EXPORT __rt_div0
    EXPORT fputc
    EXPORT fgetc
    EXPORT _sys_close
    EXPORT _sys_write
    EXPORT _sys_read
    EXPORT _sys_istty
    EXPORT _sys_seek
    EXPORT _sys_ensure
    EXPORT _sys_flen
    EXPORT _sys_tmpnam
    EXPORT _sys_command_string

__rt_div0
fputc
fgetc
_sys_close
_sys_write
_sys_read
_sys_istty
_sys_seek
_sys_ensure
_sys_flen
_sys_tmpnam
_sys_command_string
    NOP	
    MOV     R0, #0
    BX		LR		    
              
    LTORG
StackSvc           DCD     SvcStackSpace  + (SVC_STACK_LEGTH * 4)
StackIrq           DCD     IrqStackSpace  + (IRQ_STACK_LEGTH * 4)
StackFiq           DCD     FiqStackSpace  + (FIQ_STACK_LEGTH * 4)
StackAbt           DCD     AbtStackSpace  + (ABT_STACK_LEGTH * 4)
StackUnd           DCD     UndtStackSpace + (UND_STACK_LEGTH * 4)
    
        AREA    MyStacks, DATA, NOINIT, ALIGN = 2;                      ;/*  分配堆栈空间               */    
        
SvcStackSpace      SPACE   SVC_STACK_LEGTH * 4                          ;/*  管理模式堆栈空间           */
IrqStackSpace      SPACE   IRQ_STACK_LEGTH * 4                          ;/*  中断模式堆栈空间           */
FiqStackSpace      SPACE   FIQ_STACK_LEGTH * 4                          ;/*  快速中断模式堆栈空间       */
AbtStackSpace      SPACE   ABT_STACK_LEGTH * 4                          ;/*  中止义模式堆栈空间         */
UndtStackSpace     SPACE   UND_STACK_LEGTH * 4                          ;/*  未定义模式堆栈             */   

        AREA    HeapBottom, DATA, NOINIT
bottom_of_heap    	SPACE   1

        AREA    StackBottom, DATA, NOINIT
bottom_of_Stacks    SPACE   1

        AREA    HeapTop, DATA, NOINIT
top_of_heap

        AREA    StacksTop, DATA, NOINIT
StackUsr  
    END
    
;/********************************************************************************************************
;**                            End Of File
;********************************************************************************************************/
