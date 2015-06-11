;定义系统模式堆栈的大小
SVC_STACK_LEGTH     EQU         32

NoInt       EQU 0x80

USR32Mode   EQU 0x10
SVC32Mode   EQU 0x13
SYS32Mode   EQU 0x1f
IRQ32Mode   EQU 0x12
FIQ32Mode   EQU 0x11

;T_bit用于检测进入异常前cpu是否处于THUMB状态
T_bit               EQU         0x20

    CODE32
	PRESERVE8
    AREA    |subr|, CODE, READONLY

			;IMPORT	vicControl
            IMPORT  OSTCBCur                    ;指向当前任务TCB的指针
            IMPORT  OSTCBHighRdy                ;指向将要运行的任务TCB的指针
            IMPORT  OSPrioCur                   ;当前任务的优先级
            IMPORT  OSPrioHighRdy               ;将要运行的任务的优先级
            IMPORT  OSTaskSwHook                ;任务切换的钩子函数
            IMPORT  OSRunning                   ;uC/OS-II运行标志

            IMPORT  OsEnterSum                  ;关中断计数器（关中断信号量）
            ;IMPORT  SWI_Exception               ;软中断异常处理程序
            IMPORT  OSIntNesting        
            
			IMPORT  _OSFunctionAddr
            IMPORT  _UsrFunctionAddr

            IMPORT  _TaskIsARM
            IMPORT  _TaskIsTHUMB
            
            EXPORT  __OSStartHighRdy            
            EXPORT  OSIntCtxSw                  ;中断退出时的入口，参见startup.s中的IRQ_Handler
            EXPORT  SoftwareInterrupt           ;软中断入口


;/*********************************************************************************************************
;** 函数名称: SoftwareInterrupt
;** 功能描述: 软件中断，用于提供一些系统服务，功能参考os_cpu_c.c文件
;** 输　入:   依功能而定
;** 输　出 :  依功能而定
;** 全局变量: 无
;** 调用模块: SWI_Exception
;** 
;** 作　者: 
;**-------------------------------------------------------------------------------------------------------
;** 修　改: 
;**-------------------------------------------------------------------------------------------------------
;** 修　改: 
;**-------------------------------------------------------------------------------------------------------
;** 修　改: 
;**-------------------------------------------------------------------------------------------------------
;********************************************************************************************************/

;软件中断
SoftwareInterrupt
        CMP     R0, #13
        LDRLO   PC, [PC, R0, LSL #2]
        MOVS    PC, LR

SwiFunction
        DCD     TASK_SW                 ;0
        DCD     ENTER_CRITICAL          ;1
        DCD     EXIT_CRITICAL           ;2
        DCD     ISRBegin                ;3
        DCD     ChangeToSYSMode         ;4
        DCD     ChangeToUSRMode         ;5
        DCD     _OSStartHighRdy			;6
        DCD     TaskIsARM               ;7
        DCD     TaskIsTHUMB             ;8
        DCD     OSISRNeedSwap           ;9
        DCD     GetOSFunctionAddr       ;10
        DCD     GetUsrFunctionAddr      ;11
        DCD     EnableIRQ              	;12

TASK_SW
        MRS     R3, SPSR                        ;保存任务的CPSR
        MOV     R2, LR                          ;保存任务的PC
        
        MSR     CPSR_c, #(NoInt :OR: SYS32Mode)    ;切换到系统模式
        STMFD   SP!, {R2}                       ;保存PC到堆栈
        STMFD   SP!, {R0-R12, LR}               ;保存R0-R12,LR到堆栈
                                                ;因为R0~R3没有保存有用数据，所以可以这样做
        B       OSIntCtxSw_0                    ;真正进行任务切换

ENTER_CRITICAL
                                                ;OsEnterSum++
        LDR     R1, =OsEnterSum
        LDRB    R2, [R1]
        ADD     R2, R2, #1
        STRB    R2, [R1]
                                                ;关中断
        MRS     R0, SPSR
        ORR     R0, R0, #NoInt
        MSR     SPSR_c, R0
        MOVS    PC, LR

EXIT_CRITICAL
                                                ;OsEnterSum--
        LDR     R1, =OsEnterSum	 
        LDRB    R2, [R1]		; //LDR{cond}B{T} Rd，<地址>    指令加载指定地址的字节数据到Rd的的最低字节中（Rd的高24位清零）；
        SUB     R2, R2, #1	;//减法
        STRB    R2, [R1]	;//STR{cond}B{T} Rd， <地址>   指令存储Rd中的最低字节数据到指定的地址单元中。
                                                ;if(OsEnterSum == 0) 开中断;
        CMP     R2, #0
        MRSEQ   R0, SPSR
        BICEQ   R0, R0, #NoInt
        MSREQ   SPSR_c, R0
        MOVS    PC, LR

ISRBegin
                                                ;OSIntNesting++
        LDR     R1, =OSIntNesting
        LDRB    R2, [R1]
        ADD     R2, R2, #1
        STRB    R2, [R1]
        MOVS    PC, LR

ChangeToSYSMode
                                                ;切换到系统模式
        MRS     R0, SPSR
        BIC     R0, R0, #0x1f
        ORR     R0, R0, #SYS32Mode    
        MSR     SPSR_c, R0
        MOVS    PC, LR

ChangeToUSRMode
                                                ;切换到用户模式
        MRS     R0, SPSR
        BIC     R0, R0, #0x1f
        ORR     R0, R0, #USR32Mode    
        MSR     SPSR_c, R0
        MOVS    PC, LR

_OSStartHighRdy
        B       __OSStartHighRdy

TaskIsARM
        MSR     CPSR_c, #(NoInt :OR: SYS32Mode)    ; 切换到系统模式
        MOV     R0, R1
        BL      _TaskIsARM
        MSR     CPSR_c, #(NoInt :OR: SVC32Mode)    ; 切换回管理模式
        MOVS    PC, LR

TaskIsTHUMB
        MSR     CPSR_c, #(NoInt :OR: SYS32Mode)    ; 切换到系统模式
        MOV     R0, R1
        BL      _TaskIsTHUMB
        MSR     CPSR_c, #(NoInt :OR: SVC32Mode)    ; 切换回管理模式
        MOVS    PC, LR

OSISRNeedSwap
        LDR     R1, =OSTCBHighRdy
        LDR     R2, =OSTCBCur
        CMP     R1, R2
        MOVEQ   R0, #0
        MOVNE   R0, #1
        MOVS    PC, LR

GetOSFunctionAddr
        LDR     R1, =_OSFunctionAddr
        LDR     R0, [R1,R0,LSL #2]
        MOVS    PC, LR
        
GetUsrFunctionAddr
        LDR     R1, =_UsrFunctionAddr
        LDR     R0, [R1,R0,LSL #2]
        MOVS    PC, LR

EnableIRQ
        MRS		R0, SPSR
        BIC		R0, R0, #NoInt
        MSR		SPSR_c, R0
        MOVS    PC, LR

;/*********************************************************************************************************
;** 函数名称: OSIntCtxSw
;** 功能描述: 中断退出时的入口
;** 输　入:   R3    :当前任务的状态寄存器CPSR（即SPSR的值）
;**           R4-R12:当前任务的R4-R11
;**           当前处理器模式的堆栈结构（出栈次序）：R0-R3、R12、PC（当前任务的）
;** 输　出 :  无
;** 全局变量: OSPrioCur,OSPrioHighRdy,OSPrioCur,OSPrioHighRdy
;** 调用模块: 无
;** 
;**-------------------------------------------------------------------------------------------------------
;********************************************************************************************************/
OSIntCtxSw
                                                    ;下面为保存任务环境
        LDR     R2, [SP, #20]                       ;获取PC
        LDR     R12, [SP, #16]                      ;获取R12
        MRS     R0, CPSR

        MSR     CPSR_c, #(NoInt :OR: SYS32Mode)
        MOV     R1, LR
        STMFD   SP!, {R1-R2}                        ;保存LR,PC
        STMFD   SP!, {R4-R12}                       ;保存R4-R12

        MSR     CPSR_c, R0
        LDMFD   SP!, {R4-R7}                        ;获取R0-R3
        ADD     SP, SP, #8                          ;出栈R12,PC
        
        MSR     CPSR_c, #(NoInt :OR: SYS32Mode)
        STMFD   SP!, {R4-R7}                        ;保存R0-R3
        
OSIntCtxSw_0        
        LDR     R1, =OsEnterSum                     ;获取OsEnterSum
        LDR     R2, [R1]
        STMFD   SP!, {R2, R3}                       ;保存CPSR,OsEnterSum

                                                    ;保存当前任务堆栈指针到当前任务的TCB
        LDR     R1, =OSTCBCur
        LDR     R1, [R1]
        STR     SP, [R1]

        BL      OSTaskSwHook                        ;调用钩子函数
                                                    ;OSPrioCur <= OSPrioHighRdy
        LDR     R4, =OSPrioCur
        LDR     R5, =OSPrioHighRdy
        LDRB    R6, [R5]
        STRB    R6, [R4]
                                                    ;OSTCBCur <= OSTCBHighRdy
        LDR     R6, =OSTCBHighRdy
        LDR     R6, [R6]
        LDR     R4, =OSTCBCur
        STR     R6, [R4]
OSIntCtxSw_1
                                                    ;获取新任务堆栈指针
        LDR     R4, [R6]
        ADD     SP, R4, #68                         ;17寄存器CPSR,OsEnterSum,R0-R12,LR,SP
        LDR     LR, [SP, #-8]
        MSR     CPSR_c, #(NoInt :OR: SVC32Mode)        ;进入管理模式
        MOV     SP, R4                              ;设置堆栈指针

        LDMFD   SP!, {R4, R5}                       ;CPSR,OsEnterSum
                                                    ;恢复新任务的OsEnterSum
        LDR     R3, =OsEnterSum
        STR     R4, [R3]
    
        MSR     SPSR_cxsf, R5                       ;恢复CPSR
        LDMFD   SP!, {R0-R12, LR, PC }^             ;运行新任务

;/*********************************************************************************************************
;** 函数名称: __OSStartHighRdy
;** 功能描述: uC/OS-II启动时使用OSStartHighRdy运行第一个任务,
;**           OSStartHighRdy会调用__OSStartHighRdy
;** 输　入:   无
;** 输　出 :  无
;** 全局变量: OSRunning,OSTCBCur,OSTCBHighRdy,OsEnterSum
;** 调用模块: OSTaskSwHook
;** 
;** 作　者: 
;** 日　期: 2003年6月5日
;**-------------------------------------------------------------------------------------------------------
;** 修　改: 
;** 日　期: 2003年6月13日
;**-------------------------------------------------------------------------------------------------------
;********************************************************************************************************/

__OSStartHighRdy
        MSR     CPSR_c, #(NoInt :OR: SYS32Mode)
                                                ;告诉uC/OS-II自身已经运行
        LDR     R4, =OSRunning
        MOV     R5, #1
        STRB    R5, [R4]

        BL      OSTaskSwHook                    ;调用钩子函数

        LDR     R6, =OSTCBHighRdy
        LDR     R6, [R6]
        B       OSIntCtxSw_1

        AREA    SWIStacks, DATA, NOINIT,ALIGN=2
SvcStackSpace      SPACE   SVC_STACK_LEGTH * 4  ;管理模式堆栈空间

    END
;/*********************************************************************************************************
;**                            End Of File
;********************************************************************************************************/
