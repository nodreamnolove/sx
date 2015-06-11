;/****************************************Copyright (c)***************************************************
;**                         Guangzhou ZHIYUAN electronics Co.,LTD.                               
;**                                     
;**                               http://www.embedtools.com
;**
;**--------------File Info-------------------------------------------------------------------------------
;** File name: 			vicControl.s
;** Last modified Date: 2008-12-12
;** Last Version: 		1.0
;** Descriptions: 		Provide VIC_Control
;**------------------------------------------------------------------------------------------------------
;** Created by: 		LinEnqiang
;** Created date:   	2008-12-12
;** Version:			1.0
;** Descriptions: 		The original version
;**
;**------------------------------------------------------------------------------------------------------
;** Modified by: 		
;** Modified date:		
;** Version:			
;** Descriptions:       
;********************************************************************************************************/
                                 INCLUDE     LPC3200.INC                ; Include the head file 引入头文件

;/********************************************************************************************************
; 宏定义
;********************************************************************************************************/

NoInt               EQU 0x80
NoFIQ		        EQU	0x40
SVC32Mode           EQU 0x13
SYS32Mode           EQU 0x1f
IRQ32Mode           EQU 0x12
VIC_STACK_LEGTH     EQU 0x60                                            ; VIC中断服务向量表堆栈大小

		    IMPORT  FIQ_Exception
		    IMPORT  OSIntCtxSw                      ;任务切换函数
	        IMPORT  OSIntExit                       ;中断退出函数
	        IMPORT  OSTCBCur
	        IMPORT  OSTCBHighRdy
	        IMPORT  OSIntNesting                    ;中断嵌套计数器 
	 		IMPORT  OsEnterSum

			EXPORT  vicInitial
	 		EXPORT  IRQ_Handler
	 		EXPORT  FIQ_Handler
	 		EXPORT  vicControl  
	 			 	
	CODE32
	PRESERVE8
    AREA    |RUNFIRST|, CODE, READONLY     

;/********************************************************************************************************
;** Function name:           vicInitial
;** Descriptions:            VIC管理初始化
;** input parameters:        none
;** output parameters:       none
;** Returned value:          none
;** Created by:              LinEnqiang
;** Created Date:            2008-12-12
;**-------------------------------------------------------------------------------------------------------
;** Modified by:
;** Modified date:
;**-------------------------------------------------------------------------------------------------------
;********************************************************************************************************/
vicInitial	
        STMFD   SP!, {r0-r10}
        		
		LDR     R2, =MIC_ER
		MOV     R3, #3
		ORR     R3, R3, R3, LSL #30
		STR	    R3, [R2] 
		
		
		MOV     R3, #0				
		LDR     R2, =MIC_APR
		STR	    R3, [R2]		
		LDR     R2, =MIC_ATR
		STR	    R3, [R2]
				
		LDR     R2, =SIC1_ER
		STR	    R3, [R2]		
		LDR     R2, =SIC1_ITR		
		STR	    R3, [R2]		
		LDR     R2, =SIC1_APR
		STR	    R3, [R2]		
		LDR     R2, =SIC1_ATR
		STR	    R3, [R2]
		
		LDR     R2, =SIC2_ER
		STR	    R3, [R2]        
        LDR     R2, =SIC2_ITR		
		STR	    R3, [R2]		
		LDR     R2, =SIC2_APR
		STR	    R3, [R2]		
		LDR     R2, =SIC2_ATR
		STR	    R3, [R2]
		
		LDR     R12,=VIC_STACK_LEGTH
		MOV     R12,R12, LSR #5
		LDR     R0, =VectStackSpace	
		
		MOV     R1, #0
        MOV     R2, #0
        MOV     R3, #0
        MOV     R4, #0
        MOV     R5, #0
        MOV     R6, #0
        MOV     R7, #0
        MOV     R8, #0	
0		
		CMP     R12,#0
		BLS     %F1
		
        STMIA   R0!, {R1-R8}
        STMIA   R0!, {R1-R8}
        STMIA   R0!, {R1-R8}
        STMIA   R0!, {R1-R8}
        
        SUB     R12,R12,#1
        B       %B0 
1        
		LDR     R0, =VectStackSpace
		LDR     R1, =IRQ_SCI1_Func
		LDR     R2, =IRQ_SCI2_Func
		STMIA   R0!, {R1-R2}
		
		LDMFD   SP!, {r0-r10}
		BX      LR 

;/********************************************************************************************************
;** Function name:           FIQ_Handler
;** Descriptions:            FIQ 中断处理
;** input parameters:        none
;** output parameters:       none
;** Returned value:          none
;** Created by:              LinEnqiang
;** Created Date:            2008-12-12
;**-------------------------------------------------------------------------------------------------------
;** Modified by:
;** Modified date:
;**-------------------------------------------------------------------------------------------------------
;********************************************************************************************************/
FIQ_Handler
        STMFD   SP!, {R0-R3, LR}        
        LDR     R0, =FIQ_Exception	                                    ; FIQ中断处理
        ADD     LR, PC, #1
        BX      R0
    CODE16
        BX      PC
        NOP
    CODE32
        LDMFD   SP!, {R0-R3, LR}
        SUBS    PC,  LR,  #4                

;/********************************************************************************************************
;** Function name:           IRQ_SCI1_Func
;** Descriptions:            中断处理函数
;** input parameters:        none
;** output parameters:       none
;** Returned value:          none
;** Created by:              LinEnqiang
;** Created Date:            2008-12-12
;**-------------------------------------------------------------------------------------------------------
;** Modified by:
;** Modified date:
;**-------------------------------------------------------------------------------------------------------
;********************************************************************************************************/
IRQ_SCI1_Func
        STMFD   SP!, {R4-R6, LR}           						
        LDR     R4, =VectStackSpace
        MOV     R0, #1
        ADD     R4, R4, R0, LSL #7
0        
        LDR     R2, =SIC1_SR
        LDR     R2, [R2]
        CMP     R2, #0
        LDMEQFD SP!, {R4-R6, PC}
                
        MOV     R5, #31
        MOV     R6, #1
1        
        CMP     R5, #0
        BLT     %B0
        
        LDR     R2, =SIC1_SR
        LDR     R2, [R2]  
        TST     R2, R6, LSL R5        
        SUBEQ   R5, R5,#1
        BEQ     %B1
        
        LDR     R2, =SIC1_RSR
        MOV     R1, R6, LSL R5
        STR     R1, [R2]
        
        LDR     R0, [R4, R5, LSL #2]
        SUB     R5, R5,#1
        CMP     R0, #0                       
        BEQ     %B1
                
        ADD     LR, PC, #1
        BX      R0
    CODE16
        BX      PC
        NOP
    CODE32     
        B       %B1
        LDMFD   SP!, {R4-R6, PC}
        
;/********************************************************************************************************
;** Function name:           IRQ_SCI2_Func
;** Descriptions:            中断处理函数
;** input parameters:        none
;** output parameters:       none
;** Returned value:          none
;** Created by:              LinEnqiang
;** Created Date:            2008-12-12
;**-------------------------------------------------------------------------------------------------------
;** Modified by:
;** Modified date:
;**-------------------------------------------------------------------------------------------------------
;********************************************************************************************************/
IRQ_SCI2_Func        
        STMFD   SP!, {R4-R6, LR}           						
        LDR     R4, =VectStackSpace
        MOV     R0, #2
        ADD     R4, R4, R0,LSL #7
0        
        LDR     R2, =SIC2_SR
        LDR     R2, [R2]
        CMP     R2, #0
        LDMEQFD SP!, {R4-R6, PC}
        
        MOV     R5, #31
        MOV     R6, #1
1        
        CMP     R5, #0
        BLT     %B0
        
        LDR     R2, =SIC2_SR
        LDR     R2, [R2]  
        TST     R2, R6, LSL R5        
        SUBEQ   R5, R5,#1
        BEQ     %B1
        
        LDR     R2, =SIC2_RSR
        MOV     R1, R6, LSL R5
        STR     R1, [R2]
        
        LDR     R0, [R4, R5, LSL #2]
        SUB     R5, R5,#1
        CMP     R0, #0                       
        BEQ     %B1
                
        ADD     LR, PC, #1
        BX      R0
    CODE16
        BX      PC
        NOP
    CODE32     
        B       %B1                 
        LDMFD   SP!, {R4-R6, PC}
        
;/********************************************************************************************************
;** Function name:           IRQ_Handler
;** Descriptions:            中断处理函数
;** input parameters:        none
;** output parameters:       none
;** Returned value:          none
;** Created by:              LinEnqiang
;** Created Date:            2008-12-12
;**-------------------------------------------------------------------------------------------------------
;** Modified by:
;** Modified date:
;**-------------------------------------------------------------------------------------------------------
;********************************************************************************************************/
IRQ_Handler 
        SUB     LR, LR, #4                      						;  计算返回地址
        STMFD   SP!, {R0-R3, R12, LR}           						;  保存任务环境
        MRS     R3, SPSR                        						;  保存状态
        STMFD   SP, {R3,LR}^                    						;  保存SPSR和用户状态的SP,                                               							
        
		;
		LDR     R2,  =OSIntNesting              ; OSIntNesting++
        LDRB    R1, [R2]
        ADD     R1, R1, #1
        STRB    R1, [R2]
		;

		NOP
        SUB     SP, SP, #4*2

        MSR     CPSR_c, #(NoInt :OR: SYS32Mode)    						;  切换到系统模式 
        
        STMFD   SP!, {R4-R6} 
        LDR     R4, =VectStackSpace
0        
        LDR     R2, =MIC_SR
        LDR     R2, [R2]
        CMP     R2, #0        
        BEQ     %F2
        
        MOV     R5, #29
        MOV     R6, #1
        
        MVN     R0, #3
        TST     R2, R0
        MOVEQ   R5, #1 
1        
        CMP     R5, #0
        BLT     %B0
        
        LDR     R2, =MIC_SR
        LDR     R2, [R2]  
        TST     R2, R6, LSL R5        
        SUBEQ   R5, R5, #1
        BEQ     %B1
        
        LDR     R2, =MIC_RSR
        MOV     R1, R6, LSL R5
        STR     R1, [R2]
        
        LDR     R0, [R4, R5, LSL #2]
        SUB     R5, R5,#1
        CMP     R0, #0                       
        BEQ     %B1
         
        ADD     LR, PC, #1
        BX      R0
    CODE16
        BX      PC
        NOP
    CODE32     
        B       %B1                 
2
        LDMFD   SP!, {R4-R6}

		LDR     R2, =OsEnterSum                 ; OsEnterSum,使OSIntExit退出时中断关闭
        MOV     R1, #1
        STR     R1, [R2]

        BL      OSIntExit

        LDR     R2, =OsEnterSum                 ; 因为中断服务程序要退出，所以OsEnterSum=0
        MOV     R1, #0
        STR     R1, [R2]

        MSR     CPSR_c, #(NoInt :OR: IRQ32Mode)    						;  切换回irq模式
        LDMFD   SP, {R3,LR}^                    						;  恢复SPSR和用户状态的SP                                                				
        MSR     SPSR_cxsf, R3
        ADD     SP, SP, #4*2                    

		LDR     R0, =OSTCBHighRdy
        LDR     R0, [R0]
        LDR     R1, =OSTCBCur
        LDR     R1, [R1]
        CMP     R0, R1

        LDMEQFD SP!, {R0-R3, R12, PC}^          ; 不进行任务切换
        LDR     PC, =OSIntCtxSw                 ; 进行任务切换
		
;/********************************************************************************************************
;** Function name:           vicControl
;** Descriptions:            软件中断，用于提供VIC管理服务
;** input parameters:        依功能而定
;** output parameters:       依功能而定
;** Returned value:          依功能而定
;** Created by:              LinEnqiang
;** Created Date:            2008/04/30
;**-------------------------------------------------------------------------------------------------------
;** Modified by:
;** Modified date:
;**-------------------------------------------------------------------------------------------------------
;********************************************************************************************************/
vicControl         
        SUB     R0, R0, #0x100
        CMP     R0, #0x12
        LDRLO   PC, [PC, R0, LSL #2]
        MOVS    PC, LR
                
vicControlFuncAdd
        DCD     SetmicIrqFunc                                           ; 0
        DCD     ClrmicIrqFunc                                           ; 1
        DCD     EnablemicIrq                                            ; 2
        DCD     DisablemicIrq                                           ; 3
        DCD     SetmicFiq                                               ; 4
        DCD     ClrmicFiq                                               ; 5 

        DCD     Setsic1IrqFunc                                          ; 6 
        DCD     Clrsic1IrqFunc                                          ; 7
        DCD     Enablesic1Irq                                           ; 8
        DCD     Disablesic1Irq                                          ; 9
        DCD     Setsic1Fiq                                              ; 10
        DCD     Clrsic1Fiq                                              ; 11 

        DCD     Setsic2IrqFunc                                          ; 12
        DCD     Clrsic2IrqFunc                                          ; 13
        DCD     Enablesic2Irq                                           ; 14
        DCD     Disablesic2Irq                                          ; 15
        DCD     Setsic2Fiq                                              ; 16
        DCD     Clrsic2Fiq                                              ; 17 

;/*********************************************************************************************************
;** Function name:           SetmicIrqFunc
;** Descriptions:            设置所选外设的中断触发类型、中断服务函数地址，并使能中断
;** input parameters:        R0:         外设对应的中断通道号
;**                          R1:         中断触发类型
;**                          R2:         中断服务函数地址
;** output parameters:       none
;** Returned value:          1:          成功
;**                          0:          失败
;** Created by:              LinEnqiang
;** Created Date:            2008-12-12
;**-------------------------------------------------------------------------------------------------------
;** Modified by:
;** Modified date:
;**-------------------------------------------------------------------------------------------------------
;*********************************************************************************************************/

SetmicIrqFunc        
        CMP     R1, #32                                                 ;  if (通道号 >=32) return FALSE
        MOVCS   R0, #0
        MOVCSS  PC, LR
        
        CMP     R2, #4                                                  ;  if (触发类型 >=4) return FALSE
        MOVCS   R0, #0 
        MOVCSS  PC, LR 
       
        CMP     R3, #0                                                  ;  if (处理函数 ==0) return FALSE
        MOVEQ   R0, #0
        MOVEQS  PC, LR
		     
 		MSR     CPSR_c, #(NoFIQ :OR: NoInt :OR: SYS32Mode)
        STMFD   SP!, {R2, R3}
		MOV     R3, #1                                                  
        MOV     R3, R3, LSL R1 
       
        LDR     R0, =MIC_ITR                                            ;  if (Enable) return FALSE
        LDR     R2, [R0]
        ANDS    R2, R2, R3
        BNE     SetmicIrqFunc_j
        
        LDR     R0, =VectStackSpace                                     ;  if (IRQ已经使能) return FALSE
        LDR     R2, [R0, R1, LSL #2]
        CMP     R2, #0         
        
SetmicIrqFunc_j        
        LDMFD   SP!, {R2, R3}
        MSR     CPSR_c, #(NoFIQ :OR: NoInt :OR: SVC32Mode)
        MOVNE   R0, #0
        MOVNES  PC, LR 
        
        STR     R3, [R0, R1, LSL #2]
        
        LDR     R0, =MIC_APR
        LDR     R3, [R0]
        MOV     R0, #1
        BIC     R3, R3, R0,LSL R1  
        TST     R2, #1
        ORRNE   R3, R3, R0,LSL R1
        LDR     R0, =MIC_APR
        STR     R3, [R0]     
        
        LDR     R0, =MIC_ATR
        LDR     R3, [R0]
        MOV     R0, #1
        BIC     R3, R3, R0,LSL R1  
        TST     R2, #2
        ORREQ   R3, R3, R0,LSL R1    
        LDR     R0, =MIC_ATR
        STR     R3, [R0]  

        LDR     R0, =MIC_ER
        LDR     R2, [R0]
        MOV     R3, #1
        ORR     R2, R2, R3,LSL R1 
        STR     R2, [R0]
                                              
        MOV     R0, #1
        MOVS    PC, LR   
;/*********************************************************************************************************
;** Function name:           ClrmicIrqFunc
;** Descriptions:            清除所选外设的IRQ资源
;** input parameters:        R0:        外设对应的中断通道号
;** output parameters:       none
;** Returned value:          1:         成功
;**                          0:         失败
;** Created by:              LinEnqiang
;** Created Date:            2008-12-12
;**-------------------------------------------------------------------------------------------------------
;** Modified by:
;** Modified date:
;**-------------------------------------------------------------------------------------------------------
;*********************************************************************************************************/
ClrmicIrqFunc        
        CMP     R1, #32 						                        ;  if (通道号 >=32) return FALSE
        MOVCS   R0, #0
		MOVCSS  PC, LR
		
		LDR     R0, =MIC_ITR      			                            ;  if (FIQ) return FALSE
        LDR     R2, [R0]
        MOV     R3, #1
        TST     R2, R3, LSL R1         
        MOVNE   R0, #0
		MOVNES  PC, LR
        
        LDR     R0, =VectStackSpace				                        ;  if (IRQ wasnt Set) return FALSE
        LDR     R2, [R0, R1, LSL #2] 
        CMP     R2, #0
        MOVEQ   R0, #0
		MOVEQS  PC, LR
		
        MSR     CPSR_c, #(NoFIQ :OR: NoInt :OR: SVC32Mode)        
		LDR     R0, =MIC_ER       			                            ;  Disable IRQ
        LDR     R2, [R0]
        BIC     R2, R2, R3,LSL R1
        STR     R2, [R0]  
  
        LDR     R0, =VectStackSpace   			                        ;  Clear VectStackSpace
        MOV     R3, #0
        STR     R3, [R0, R1, LSL #2]        
        MOV     R0, #1        
		MOVS    PC, LR
;/*********************************************************************************************************
;** Function name:           EnablemicIrq
;** Descriptions:            使能相应外设的中断
;** input parameters:        R0:         外设对应的中断通道号
;** output parameters:       none
;** Returned value:          1:          成功
;**                          0:          失败
;** Created by:              LinEnqiang
;** Created Date:            2008-12-12
;**-------------------------------------------------------------------------------------------------------
;** Modified by:
;** Modified date:
;**-------------------------------------------------------------------------------------------------------
;*********************************************************************************************************/
EnablemicIrq        
        CMP     R1, #32                                                 ;  if (通道号 >=32) return FALSE
        MOVCS   R0, #0
        MOVCSS  PC, LR 
        
        LDR     R0, =VectStackSpace				                        ;  if (IRQ wasnt Set) return FALSE
        LDR     R2, [R0, R1, LSL #2] 
        CMP     R2, #0
        MOVEQ   R0, #0
		MOVEQS  PC, LR
		
		MSR     CPSR_c, #(NoFIQ :OR: NoInt :OR: SVC32Mode)
		LDR     R0, =MIC_ER
        LDR     R2, [R0]
        MOV     R3, #1
        ORR     R2, R2, R3,LSL R1 
        STR     R2, [R0]
        MOV     R0, #1
        MOVS    PC, LR                

;/*********************************************************************************************************
;** Function name:           DisablemicIrq
;** Descriptions:            禁止相应外设的中断
;** input parameters:        uiChannel:  外设对应的中断通道号
;** output parameters:       none
;** Returned value:          1:          成功
;**                          0:          失败
;** Created by:              LinEnqiang
;** Created Date:            2008-12-12
;**-------------------------------------------------------------------------------------------------------
;** Modified by:
;** Modified date:
;**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
DisablemicIrq       
        CMP     R1, #32                                                 ;  if (通道号 >=32) return FALSE
        MOVCS   R0, #0
        MOVCSS  PC, LR 
        
        LDR     R0, =VectStackSpace                                     ;  if (未加载) return FALSE
        LDR     R2, [R0, R1, LSL #2]
        CMP     R2, #0
        MOVEQ   R0, #0
        MOVEQS  PC, LR
        
        MSR     CPSR_c, #(NoFIQ :OR: NoInt :OR: SVC32Mode)
        
        LDR     R0, =MIC_ER
        LDR     R2, [R0]
        MOV     R3, #1
        BIC     R2, R2, R3,LSL R1         
        STR     R2, [R0]
        
        MOV     R0, #1
        MOVS    PC, LR
;/*********************************************************************************************************
;** Function name:           SetmicFiq
;** Descriptions:            设置并使能所选中断通道号为FIQ中断
;** input parameters:        R0:        外设对应的中断通道号
;** output parameters:       none
;** Returned value:          1:          成功
;**                          0:          失败
;** Created by:              LinEnqiang
;** Created Date:            2008-12-12
;**-------------------------------------------------------------------------------------------------------
;** Modified by:
;** Modified date:
;**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
SetmicFiq
       	MSR     CPSR_c, #(NoFIQ :OR: NoInt :OR: SVC32Mode)
       	
        CMP     R1, #32							                        ;  if (通道号 >=32) return FALSE
        MOVCS   R0, #0
		MOVCSS  PC, LR
        
        LDR     R0, =VectStackSpace				                        ;  if (IRQ已加载) return FALSE
        LDR     R3, [R0, R1, LSL #2] 
        CMP     R3, #0
        MOVNE   R0, #0
		MOVNES  PC, LR
        
        CMP     R2, #4                                                  ;  if (触发类型 >=4) return FALSE
        MOVCS   R0, #0 
        MOVCSS  PC, LR 

        LDR     R0, =MIC_APR
        LDR     R3, [R0]
        MOV     R0, #1
        BIC     R3, R3, R0,LSL R1  
        TST     R2, #1
        ORRNE   R3, R3, R0,LSL R1
        LDR     R0, =MIC_APR
        STR     R3, [R0]     
        
        LDR     R0, =MIC_ATR
        LDR     R3, [R0]
        MOV     R0, #1
        BIC     R3, R3, R0,LSL R1  
        TST     R2, #2
        ORREQ   R3, R3, R0,LSL R1        
        LDR     R0, =MIC_ATR
        STR     R3, [R0]
        
        LDR     R0, =MIC_ITR				
        LDR     R2, [R0]
        MOV     R3, #1
        ORR     R2, R2, R3,LSL R1 
        STR     R2, [R0]
        
        LDR     R0, =MIC_ER
        LDR     R2, [R0]
        ORR     R2, R2, R3,LSL R1 
        STR     R2, [R0]
        MOV     R0, #1
        MOVS    PC, LR
;/*********************************************************************************************************
;** Function name:           ClrmicFiq
;** Descriptions:            清除所选中断通道号的FIQ中断
;** input parameters:        R0:         外设对应的中断通道号
;** output parameters:       none
;** Returned value:          1:          成功
;**                          0:          失败;
;** Created by:              LinEnqiang
;** Created Date:            2008-12-12
;**-------------------------------------------------------------------------------------------------------
;** Modified by:
;** Modified date:
;**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/

ClrmicFiq
		MSR     CPSR_c, #(NoFIQ :OR: NoInt :OR: SVC32Mode)
		
        CMP     R1, #32							                        ;  if (通道号 >=32) return FALSE
        MOVCS   R0, #0
		MOVCSS  PC, LR       
             
        LDR     R0, =MIC_ITR				                            ;  if(FIQ未使能)return FALSE
        LDR     R2, [R0]
        MOV     R3, #1
        ANDS    R3, R2, R3,LSL R1   
        MOVEQ   R0, #0
		MOVEQS  PC, LR         
       
        LDR     R0, =MIC_ER
        LDR     R2, [R0]
        BIC     R2, R2, R3
        STR     R2, [R0]  						                        ;  Disable FIQ
        
        LDR     R0, =MIC_ITR
        LDR     R2, [R0]
        BIC     R2, R2,R3
        STR     R2, [R0]  							       
        
        MOV     R0, #1
        MOVS    PC, LR
                    
;/*********************************************************************************************************
;** Function name:           Setsic1IrqFunc
;** Descriptions:            设置所选外设的中断触发类型、中断服务函数地址，并使能中断
;** input parameters:        R0:         外设对应的中断通道号
;**                          R1:         中断触发类型
;**                          R2:         中断服务函数地址
;** output parameters:       none
;** Returned value:          1:          成功
;**                          0:          失败
;** Created by:              LinEnqiang
;** Created Date:            2008-12-12
;**-------------------------------------------------------------------------------------------------------
;** Modified by:
;** Modified date:
;**-------------------------------------------------------------------------------------------------------
;*********************************************************************************************************/
Setsic1IrqFunc
        CMP     R1, #32                                                 ;  if (通道号 >=32) return FALSE
        MOVCS   R0, #0
        MOVCSS  PC, LR
        
        CMP     R2, #4                                                  ;  if (触发类型 >=4) return FALSE
        MOVCS   R0, #0 
        MOVCSS  PC, LR 
       
        CMP     R3, #0                                                  ;  if (处理函数 ==0) return FALSE
        MOVEQ   R0, #0
        MOVEQS  PC, LR
		     
		MSR     CPSR_c, #(NoFIQ :OR: NoInt :OR: SYS32Mode)
        STMFD   SP!, {R2, R3}
		MOV     R3, #1                                                  
        MOV     R3, R3, LSL R1 
       
        LDR     R0, =SIC1_ITR                                            ;  if (Enable) return FALSE
        LDR     R2, [R0]
        ANDS    R2, R2, R3
        BNE     Setsic1IrqFunc_j
        
        LDR     R0, =VectStackSpace                                     ;  if (IRQ已经使能) return FALSE
        MOV     R2, #1
        ADD     R0, R0, R2,LSL #7
        
        LDR     R2, [R0, R1, LSL #2]
        CMP     R2, #0         
Setsic1IrqFunc_j
        LDMFD   SP!, {R2, R3}
        MSR     CPSR_c, #(NoFIQ :OR: NoInt :OR: SVC32Mode)
        MOVNE   R0, #0
        MOVNES  PC, LR 
        
        STR     R3, [R0, R1,LSL #2]
        
        LDR     R0, =SIC1_APR
        LDR     R3, [R0]
        MOV     R0, #1
        BIC     R3, R3, R0,LSL R1  
        TST     R2, #1
        ORRNE   R3, R3, R0,LSL R1
        LDR     R0, =SIC1_APR
        STR     R3, [R0]     
        
        LDR     R0, =SIC1_ATR
        LDR     R3, [R0]
        MOV     R0, #1
        BIC     R3, R3, R0,LSL R1  
        TST     R2, #2
        ORREQ   R3, R3, R0,LSL R1      
        LDR     R0, =SIC1_ATR
        STR     R3, [R0]  

        LDR     R0, =SIC1_ER
        LDR     R2, [R0]
        MOV     R3, #1
        ORR     R2, R2, R3,LSL R1 
        STR     R2, [R0]
                                              
        MOV     R0, #1
        MOVS    PC, LR   
;/*********************************************************************************************************
;** Function name:           Clrsic1IrqFunc
;** Descriptions:            清除所选外设的IRQ资源
;** input parameters:        R0:        外设对应的中断通道号
;** output parameters:       none
;** Returned value:          1:         成功
;**                          0:         失败
;** Created by:              LinEnqiang
;** Created Date:            2008-12-12
;**-------------------------------------------------------------------------------------------------------
;** Modified by:
;** Modified date:
;**-------------------------------------------------------------------------------------------------------
;*********************************************************************************************************/
Clrsic1IrqFunc
        CMP     R1, #32 						                        ;  if (通道号 >=32) return FALSE
        MOVCS   R0, #0
		MOVCSS  PC, LR
		
		LDR     R0, =SIC1_ITR      			                            ;  if (FIQ) return FALSE
        LDR     R2, [R0]
        MOV     R3, #1
        TST     R2, R3, LSL R1         
        MOVNE   R0, #0
		MOVNES  PC, LR
        
        LDR     R0, =VectStackSpace				                        ;  if (IRQ wasnt Set) return FALSE
        MOV     R2, #1
        ADD     R0, R0,R2,LSL #7
 
        LDR     R2, [R0, R1, LSL #2] 
        CMP     R2, #0
        MOVEQ   R0, #0
		MOVEQS  PC, LR
		
        MSR     CPSR_c, #(NoFIQ :OR: NoInt :OR: SVC32Mode)        
		LDR     R0, =SIC1_ER       			                            ;  Disable IRQ
        LDR     R2, [R0]
        BIC     R2, R2, R3,LSL R1
        STR     R2, [R0]  
  
        LDR     R0, =VectStackSpace   			                        ;  Clear VectStackSpace
        MOV     R2, #1
        ADD     R0, R0, R2,LSL #7
 
        MOV     R3, #0
        STR     R3, [R0, R1, LSL #2]        
        MOV     R0, #1        
		MOVS    PC, LR
;/*********************************************************************************************************
;** Function name:           Enablesic1Irq
;** Descriptions:            使能相应外设的中断
;** input parameters:        R0:         外设对应的中断通道号
;** output parameters:       none
;** Returned value:          1:          成功
;**                          0:          失败
;** Created by:              LinEnqiang
;** Created Date:            2008-12-12
;**-------------------------------------------------------------------------------------------------------
;** Modified by:
;** Modified date:
;**-------------------------------------------------------------------------------------------------------
;*********************************************************************************************************/
Enablesic1Irq
        CMP     R1, #32                                                 ;  if (通道号 >=32) return FALSE
        MOVCS   R0, #0
        MOVCSS  PC, LR 
        
        LDR     R0, =VectStackSpace				                        ;  if (IRQ wasnt Set) return FALSE
        MOV     R2, #1
        ADD     R0, R0, R2,LSL #7
 
        LDR     R2, [R0, R1, LSL #2] 
        CMP     R2, #0
        MOVEQ   R0, #0
		MOVEQS  PC, LR
		
		MSR     CPSR_c, #(NoFIQ :OR: NoInt :OR: SVC32Mode)
		LDR     R0, =SIC1_ER
        LDR     R2, [R0]
        MOV     R3, #1
        ORR     R2, R2, R3,LSL R1 
        STR     R2, [R0]
        MOV     R0, #1
        MOVS    PC, LR                

;/*********************************************************************************************************
;** Function name:           Disablesic1Irq
;** Descriptions:            禁止相应外设的中断
;** input parameters:        uiChannel:  外设对应的中断通道号
;** output parameters:       none
;** Returned value:          1:          成功
;**                          0:          失败
;** Created by:              LinEnqiang
;** Created Date:            2008-12-12
;**-------------------------------------------------------------------------------------------------------
;** Modified by:
;** Modified date:
;**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
Disablesic1Irq
        CMP     R1, #32                                                 ;  if (通道号 >=32) return FALSE
        MOVCS   R0, #0
        MOVCSS  PC, LR 
        
        LDR     R0, =VectStackSpace                                     ;  if (未加载) return FALSE
        MOV     R2, #1
        ADD     R0, R0, R2,LSL #7
 
        LDR     R2, [R0, R1,LSL #2]
        CMP     R2, #0
        MOVEQ   R0, #0
        MOVEQS  PC, LR
        
        MSR     CPSR_c, #(NoFIQ :OR: NoInt :OR: SVC32Mode)
        
        LDR     R0, =SIC1_ER
        LDR     R2, [R0]
        MOV     R3, #1
        BIC     R2, R2, R3,LSL R1         
        STR     R2, [R0]
        
        MOV     R0, #1
        MOVS    PC, LR
;/*********************************************************************************************************
;** Function name:           Setsic1Fiq
;** Descriptions:            设置并使能所选中断通道号为FIQ中断
;** input parameters:        R0:        外设对应的中断通道号
;** output parameters:       none
;** Returned value:          1:          成功
;**                          0:          失败
;** Created by:              LinEnqiang
;** Created Date:            2008-12-12
;**-------------------------------------------------------------------------------------------------------
;** Modified by:
;** Modified date:
;**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
Setsic1Fiq
       	MSR     CPSR_c, #(NoFIQ :OR: NoInt :OR: SVC32Mode)
       	
        CMP     R1, #32							                        ;  if (通道号 >=32) return FALSE
        MOVCS   R0, #0
		MOVCSS  PC, LR
        
        LDR     R0, =VectStackSpace				                        ;  if (IRQ已加载) return FALSE
        MOV     R3, #1
        ADD     R0, R0,R3,LSL #7
        
        LDR     R3, [R0, R1,LSL #2] 
        CMP     R3, #0
        MOVNE   R0, #0
		MOVNES  PC, LR
        
        CMP     R2, #4                                                  ;  if (触发类型 >=4) return FALSE
        MOVCS   R0, #0 
        MOVCSS  PC, LR 

        LDR     R0, =SIC1_APR
        LDR     R3, [R0]
        MOV     R0, #1
        BIC     R3, R3, R0,LSL R1  
        TST     R2, #1
        ORRNE   R3, R3, R0,LSL R1
        LDR     R0, =SIC1_APR
        STR     R3, [R0]     
        
        LDR     R0, =SIC1_ATR
        LDR     R3, [R0]
        MOV     R0, #1
        BIC     R3, R3, R0,LSL R1  
        TST     R2, #2
        ORREQ   R3, R3, R0,LSL R1        
        LDR     R0, =SIC1_ATR
        STR     R3, [R0]   
        
        LDR     R0, =SIC1_ITR				
        LDR     R2, [R0]
        MOV     R3, #1
        ORR     R2, R2, R3,LSL R1 
        STR     R2, [R0]
        
        LDR     R0, =SIC1_ER
        LDR     R2, [R0]
        ORR     R2, R2, R3,LSL R1 
        STR     R2, [R0]
        MOV     R0, #1
        MOVS    PC, LR
;/*********************************************************************************************************
;** Function name:           Clrsic1Fiq
;** Descriptions:            清除所选中断通道号的FIQ中断
;** input parameters:        R0:         外设对应的中断通道号
;** output parameters:       none
;** Returned value:          1:          成功
;**                          0:          失败;
;** Created by:              LinEnqiang
;** Created Date:            2008-12-12
;**-------------------------------------------------------------------------------------------------------
;** Modified by:
;** Modified date:
;**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/

Clrsic1Fiq
		MSR     CPSR_c, #(NoFIQ :OR: NoInt :OR: SVC32Mode)
		
        CMP     R1, #32							                        ;  if (通道号 >=32) return FALSE
        MOVCS   R0, #0
		MOVCSS  PC, LR       
             
        LDR     R0, =SIC1_ITR				                            ;  if(FIQ未使能)return FALSE
        LDR     R2, [R0]
        MOV     R3, #1
        ANDS    R3, R2, R3,LSL R1   
        MOVEQ   R0, #0
		MOVEQS  PC, LR         
       
        LDR     R0, =SIC1_ER
        LDR     R2, [R0]
        BIC     R2, R2, R3
        STR     R2, [R0]  						                        ;  Disable FIQ
        
        LDR     R0, =SIC1_ITR
        LDR     R2, [R0]
        BIC     R2, R2, R3
        STR     R2, [R0]  							       
        
        MOV     R0, #1
        MOVS    PC, LR     
        
;/*********************************************************************************************************
;** Function name:           Setsic2IrqFunc
;** Descriptions:            设置所选外设的中断触发类型、中断服务函数地址，并使能中断
;** input parameters:        R0:         外设对应的中断通道号
;**                          R1:         中断触发类型
;**                          R2:         中断服务函数地址
;** output parameters:       none
;** Returned value:          1:          成功
;**                          0:          失败
;** Created by:              LinEnqiang
;** Created Date:            2008-12-12
;**-------------------------------------------------------------------------------------------------------
;** Modified by:
;** Modified date:
;**-------------------------------------------------------------------------------------------------------
;*********************************************************************************************************/
Setsic2IrqFunc
        CMP     R1, #32                                                 ;  if (通道号 >=32) return FALSE
        MOVCS   R0, #0
        MOVCSS  PC, LR
        
        CMP     R2, #4                                                  ;  if (触发类型 >=4) return FALSE
        MOVCS   R0, #0 
        MOVCSS  PC, LR 
       
        CMP     R3, #0                                                  ;  if (处理函数 ==0) return FALSE
        MOVEQ   R0, #0
        MOVEQS  PC, LR
             
         MSR     CPSR_c, #(NoFIQ :OR: NoInt :OR: SYS32Mode)
        STMFD   SP!, {R2, R3}
        MOV     R3, #1                                                  
        MOV     R3, R3, LSL R1 
       
        LDR     R0, =SIC2_ITR                                           ;  if (Enable) return FALSE
        LDR     R2, [R0]
        ANDS    R2, R2, R3
        BNE     Setsic2IrqFunc_j
        
        LDR     R0, =VectStackSpace                                     ;  if (IRQ已经使能) return FALSE
        MOV     R2, #2
        ADD     R0, R0,R2,LSL #7
 
        LDR     R2, [R0, R1, LSL #2]
        CMP     R2, #0         
Setsic2IrqFunc_j
        LDMFD   SP!, {R2, R3}
        MSR     CPSR_c, #(NoFIQ :OR: NoInt :OR: SVC32Mode)
        MOVNE   R0, #0
        MOVNES  PC, LR 
        
        STR     R3, [R0, R1, LSL #2]
        
        LDR     R0, =SIC2_APR
        LDR     R3, [R0]
        MOV     R0, #1
        BIC     R3, R3, R0,LSL R1  
        TST     R2, #1
        ORRNE   R3, R3, R0,LSL R1
        LDR     R0, =SIC2_APR
        STR     R3, [R0]     
        
        LDR     R0, =MIC_ATR
        LDR     R3, [R0]
        MOV     R0, #1
        BIC     R3, R3, R0,LSL R1  
        TST     R2, #2
        ORREQ   R3, R3, R0,LSL R1                
        LDR     R0, =SIC2_ATR
        STR     R3, [R0]  

        LDR     R0, =SIC2_ER
        LDR     R2, [R0]
        MOV     R3, #1
        ORR     R2, R2, R3,LSL R1 
        STR     R2, [R0]
                                              
        MOV     R0, #1
        MOVS    PC, LR   
;/*********************************************************************************************************
;** Function name:           Clrsic2IrqFunc
;** Descriptions:            清除所选外设的IRQ资源
;** input parameters:        R0:        外设对应的中断通道号
;** output parameters:       none
;** Returned value:          1:         成功
;**                          0:         失败
;** Created by:              LinEnqiang
;** Created Date:            2008-12-12
;**-------------------------------------------------------------------------------------------------------
;** Modified by:
;** Modified date:
;**-------------------------------------------------------------------------------------------------------
;*********************************************************************************************************/
Clrsic2IrqFunc
        CMP     R1, #32 						                        ;  if (通道号 >=32) return FALSE
        MOVCS   R0, #0
		MOVCSS  PC, LR
		
		LDR     R0, =SIC2_ITR      			                            ;  if (FIQ) return FALSE
        LDR     R2, [R0]
        MOV     R3, #1
        TST     R2, R3, LSL R1         
        MOVNE   R0, #0
		MOVNES  PC, LR
        
        LDR     R0, =VectStackSpace				                        ;  if (IRQ wasnt Set) return FALSE
        MOV     R2, #2
        ADD     R0, R0, R2,LSL #7
        
        LDR     R2, [R0, R1, LSL #2] 
        CMP     R2, #0
        MOVEQ   R0, #0
		MOVEQS  PC, LR
		
        MSR     CPSR_c, #(NoFIQ :OR: NoInt :OR: SVC32Mode)        
		LDR     R0, =SIC2_ER       			                            ;  Disable IRQ
        LDR     R2, [R0]
        BIC     R2, R2, R3,LSL R1
        STR     R2, [R0]  
  
        LDR     R0, =VectStackSpace   			                        ;  Clear VectStackSpace
        MOV     R2, #2
        ADD     R0, R0, R2,LSL #7
        
        MOV     R2, #0
        STR     R2, [R0, R1, LSL #2]        
        MOV     R0, #1        
		MOVS    PC, LR
;/*********************************************************************************************************
;** Function name:           Enablesic2Irq
;** Descriptions:            使能相应外设的中断
;** input parameters:        R0:         外设对应的中断通道号
;** output parameters:       none
;** Returned value:          1:          成功
;**                          0:          失败
;** Created by:              LinEnqiang
;** Created Date:            2008-12-12
;**-------------------------------------------------------------------------------------------------------
;** Modified by:
;** Modified date:
;**-------------------------------------------------------------------------------------------------------
;*********************************************************************************************************/
Enablesic2Irq
        CMP     R1, #32                                                 ;  if (通道号 >=32) return FALSE
        MOVCS   R0, #0
        MOVCSS  PC, LR 
        
        LDR     R0, =VectStackSpace				                        ;  if (IRQ wasnt Set) return FALSE
        MOV     R2, #2
        ADD     R0, R0, R2,LSL #7
        
        LDR     R2, [R0, R1, LSL #2] 
        CMP     R2, #0
        MOVEQ   R0, #0
		MOVEQS  PC, LR
		
		MSR     CPSR_c, #(NoFIQ :OR: NoInt :OR: SVC32Mode)
		LDR     R0, =SIC2_ER
        LDR     R2, [R0]
        MOV     R3, #1
        ORR     R2, R2, R3,LSL R1 
        STR     R2, [R0]
        MOV     R0, #1
        MOVS    PC, LR                

;/*********************************************************************************************************
;** Function name:           Disablesic2Irq
;** Descriptions:            禁止相应外设的中断
;** input parameters:        uiChannel:  外设对应的中断通道号
;** output parameters:       none
;** Returned value:          1:          成功
;**                          0:          失败
;** Created by:              LinEnqiang
;** Created Date:            2008-12-12
;**-------------------------------------------------------------------------------------------------------
;** Modified by:
;** Modified date:
;**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
Disablesic2Irq
        CMP     R1, #32                                                 ;  if (通道号 >=32) return FALSE
        MOVCS   R0, #0
        MOVCSS  PC, LR 
        
        LDR     R0, =VectStackSpace                                     ;  if (未加载) return FALSE
        MOV     R2, #2
        ADD     R0, R0, R2,LSL #7
        
        LDR     R2, [R0, R1, LSL #2]
        CMP     R2, #0
        MOVEQ   R0, #0
        MOVEQS  PC, LR
        
        MSR     CPSR_c, #(NoFIQ :OR: NoInt :OR: SVC32Mode)
        
        LDR     R0, =SIC2_ER
        LDR     R2, [R0]
        MOV     R3, #1
        BIC     R2, R2, R3,LSL R1         
        STR     R2, [R0]
        
        MOV     R0, #1
        MOVS    PC, LR
;/*********************************************************************************************************
;** Function name:           Setsic2Fiq
;** Descriptions:            设置并使能所选中断通道号为FIQ中断
;** input parameters:        R0:        外设对应的中断通道号
;** output parameters:       none
;** Returned value:          1:          成功
;**                          0:          失败
;** Created by:              LinEnqiang
;** Created Date:            2008-12-12
;**-------------------------------------------------------------------------------------------------------
;** Modified by:
;** Modified date:
;**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
Setsic2Fiq
       	MSR     CPSR_c, #(NoFIQ :OR: NoInt :OR: SVC32Mode)
       	
        CMP     R1, #32							                        ;  if (通道号 >=32) return FALSE
        MOVCS   R0, #0
		MOVCSS  PC, LR
        
        LDR     R0, =VectStackSpace				                        ;  if (IRQ已加载) return FALSE
        MOV     R3, #2
        ADD     R0, R0, R3,LSL #7
        
        LDR     R3, [R0, R1, LSL #2] 
        CMP     R3, #0
        MOVNE   R0, #0
		MOVNES  PC, LR
        
        CMP     R2, #4                                                  ;  if (触发类型 >=4) return FALSE
        MOVCS   R0, #0 
        MOVCSS  PC, LR 

        LDR     R0, =SIC2_APR
        LDR     R3, [R0]
        MOV     R0, #1
        BIC     R3, R3, R0,LSL R1  
        TST     R2, #1
        ORRNE   R3, R3, R0,LSL R1
        LDR     R0, =SIC2_APR
        STR     R3, [R0]     
        
        LDR     R0, =SIC2_ATR
        LDR     R3, [R0]
        MOV     R0, #1
        BIC     R3, R3, R0,LSL R1  
        TST     R2, #2
        ORREQ   R3, R3, R0,LSL R1        
        LDR     R0, =SIC2_ATR
        STR     R3, [R0] 
        
        LDR     R0, =SIC2_ITR				
        LDR     R2, [R0]
        MOV     R3, #1
        ORR     R2, R2, R3,LSL R1 
        STR     R2, [R0]
        
        LDR     R0, =SIC2_ER
        LDR     R2, [R0]
        ORR     R2, R2, R3,LSL R1 
        STR     R2, [R0]
        MOV     R0, #1
        MOVS    PC, LR
;/*********************************************************************************************************
;** Function name:           Clrsic2Fiq
;** Descriptions:            清除所选中断通道号的FIQ中断
;** input parameters:        R0:         外设对应的中断通道号
;** output parameters:       none
;** Returned value:          1:          成功
;**                          0:          失败;
;** Created by:              LinEnqiang
;** Created Date:            2008-12-12
;**-------------------------------------------------------------------------------------------------------
;** Modified by:
;** Modified date:
;**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/

Clrsic2Fiq
		MSR     CPSR_c, #(NoFIQ :OR: NoInt :OR: SVC32Mode)
		
        CMP     R1, #32							                        ;  if (通道号 >=32) return FALSE
        MOVCS   R0, #0
		MOVCSS  PC, LR       
             
        LDR     R0, =SIC2_ITR				                            ;  if(FIQ未使能)return FALSE
        LDR     R2, [R0]
        MOV     R3, #1
        ANDS    R3, R2, R3,LSL R1   
        MOVEQ   R0, #0
		MOVEQS  PC, LR         
       
        LDR     R0, =SIC2_ER
        LDR     R2, [R0]
        BIC     R2, R2, R3
        STR     R2, [R0]  						                        ;  Disable FIQ
        
        LDR     R0, =SIC2_ITR
        LDR     R2, [R0]
        BIC     R2, R2, R3
        STR     R2, [R0]  							       
        
        MOV     R0, #1
        MOVS    PC, LR             
                    
    AREA    VectStacks, DATA, NOINIT, ALIGN = 2;                        ;/*  分配堆栈空间               */    
        
VectStackSpace  SPACE   VIC_STACK_LEGTH * 4                             ;/*  中断服务向量表堆栈空间     */
                 
	
    END
;/********************************************************************************************************
;	End Of File
;********************************************************************************************************/