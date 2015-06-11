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
                                 INCLUDE     LPC3200.INC                ; Include the head file ����ͷ�ļ�

;/********************************************************************************************************
; �궨��
;********************************************************************************************************/

NoInt               EQU 0x80
NoFIQ		        EQU	0x40
SVC32Mode           EQU 0x13
SYS32Mode           EQU 0x1f
IRQ32Mode           EQU 0x12
VIC_STACK_LEGTH     EQU 0x60                                            ; VIC�жϷ����������ջ��С

		    IMPORT  FIQ_Exception
		    IMPORT  OSIntCtxSw                      ;�����л�����
	        IMPORT  OSIntExit                       ;�ж��˳�����
	        IMPORT  OSTCBCur
	        IMPORT  OSTCBHighRdy
	        IMPORT  OSIntNesting                    ;�ж�Ƕ�׼����� 
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
;** Descriptions:            VIC�����ʼ��
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
;** Descriptions:            FIQ �жϴ���
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
        LDR     R0, =FIQ_Exception	                                    ; FIQ�жϴ���
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
;** Descriptions:            �жϴ�����
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
;** Descriptions:            �жϴ�����
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
;** Descriptions:            �жϴ�����
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
        SUB     LR, LR, #4                      						;  ���㷵�ص�ַ
        STMFD   SP!, {R0-R3, R12, LR}           						;  �������񻷾�
        MRS     R3, SPSR                        						;  ����״̬
        STMFD   SP, {R3,LR}^                    						;  ����SPSR���û�״̬��SP,                                               							
        
		;
		LDR     R2,  =OSIntNesting              ; OSIntNesting++
        LDRB    R1, [R2]
        ADD     R1, R1, #1
        STRB    R1, [R2]
		;

		NOP
        SUB     SP, SP, #4*2

        MSR     CPSR_c, #(NoInt :OR: SYS32Mode)    						;  �л���ϵͳģʽ 
        
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

		LDR     R2, =OsEnterSum                 ; OsEnterSum,ʹOSIntExit�˳�ʱ�жϹر�
        MOV     R1, #1
        STR     R1, [R2]

        BL      OSIntExit

        LDR     R2, =OsEnterSum                 ; ��Ϊ�жϷ������Ҫ�˳�������OsEnterSum=0
        MOV     R1, #0
        STR     R1, [R2]

        MSR     CPSR_c, #(NoInt :OR: IRQ32Mode)    						;  �л���irqģʽ
        LDMFD   SP, {R3,LR}^                    						;  �ָ�SPSR���û�״̬��SP                                                				
        MSR     SPSR_cxsf, R3
        ADD     SP, SP, #4*2                    

		LDR     R0, =OSTCBHighRdy
        LDR     R0, [R0]
        LDR     R1, =OSTCBCur
        LDR     R1, [R1]
        CMP     R0, R1

        LDMEQFD SP!, {R0-R3, R12, PC}^          ; �����������л�
        LDR     PC, =OSIntCtxSw                 ; ���������л�
		
;/********************************************************************************************************
;** Function name:           vicControl
;** Descriptions:            ����жϣ������ṩVIC�������
;** input parameters:        �����ܶ���
;** output parameters:       �����ܶ���
;** Returned value:          �����ܶ���
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
;** Descriptions:            ������ѡ������жϴ������͡��жϷ�������ַ����ʹ���ж�
;** input parameters:        R0:         �����Ӧ���ж�ͨ����
;**                          R1:         �жϴ�������
;**                          R2:         �жϷ�������ַ
;** output parameters:       none
;** Returned value:          1:          �ɹ�
;**                          0:          ʧ��
;** Created by:              LinEnqiang
;** Created Date:            2008-12-12
;**-------------------------------------------------------------------------------------------------------
;** Modified by:
;** Modified date:
;**-------------------------------------------------------------------------------------------------------
;*********************************************************************************************************/

SetmicIrqFunc        
        CMP     R1, #32                                                 ;  if (ͨ���� >=32) return FALSE
        MOVCS   R0, #0
        MOVCSS  PC, LR
        
        CMP     R2, #4                                                  ;  if (�������� >=4) return FALSE
        MOVCS   R0, #0 
        MOVCSS  PC, LR 
       
        CMP     R3, #0                                                  ;  if (������ ==0) return FALSE
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
        
        LDR     R0, =VectStackSpace                                     ;  if (IRQ�Ѿ�ʹ��) return FALSE
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
;** Descriptions:            �����ѡ�����IRQ��Դ
;** input parameters:        R0:        �����Ӧ���ж�ͨ����
;** output parameters:       none
;** Returned value:          1:         �ɹ�
;**                          0:         ʧ��
;** Created by:              LinEnqiang
;** Created Date:            2008-12-12
;**-------------------------------------------------------------------------------------------------------
;** Modified by:
;** Modified date:
;**-------------------------------------------------------------------------------------------------------
;*********************************************************************************************************/
ClrmicIrqFunc        
        CMP     R1, #32 						                        ;  if (ͨ���� >=32) return FALSE
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
;** Descriptions:            ʹ����Ӧ������ж�
;** input parameters:        R0:         �����Ӧ���ж�ͨ����
;** output parameters:       none
;** Returned value:          1:          �ɹ�
;**                          0:          ʧ��
;** Created by:              LinEnqiang
;** Created Date:            2008-12-12
;**-------------------------------------------------------------------------------------------------------
;** Modified by:
;** Modified date:
;**-------------------------------------------------------------------------------------------------------
;*********************************************************************************************************/
EnablemicIrq        
        CMP     R1, #32                                                 ;  if (ͨ���� >=32) return FALSE
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
;** Descriptions:            ��ֹ��Ӧ������ж�
;** input parameters:        uiChannel:  �����Ӧ���ж�ͨ����
;** output parameters:       none
;** Returned value:          1:          �ɹ�
;**                          0:          ʧ��
;** Created by:              LinEnqiang
;** Created Date:            2008-12-12
;**-------------------------------------------------------------------------------------------------------
;** Modified by:
;** Modified date:
;**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
DisablemicIrq       
        CMP     R1, #32                                                 ;  if (ͨ���� >=32) return FALSE
        MOVCS   R0, #0
        MOVCSS  PC, LR 
        
        LDR     R0, =VectStackSpace                                     ;  if (δ����) return FALSE
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
;** Descriptions:            ���ò�ʹ����ѡ�ж�ͨ����ΪFIQ�ж�
;** input parameters:        R0:        �����Ӧ���ж�ͨ����
;** output parameters:       none
;** Returned value:          1:          �ɹ�
;**                          0:          ʧ��
;** Created by:              LinEnqiang
;** Created Date:            2008-12-12
;**-------------------------------------------------------------------------------------------------------
;** Modified by:
;** Modified date:
;**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
SetmicFiq
       	MSR     CPSR_c, #(NoFIQ :OR: NoInt :OR: SVC32Mode)
       	
        CMP     R1, #32							                        ;  if (ͨ���� >=32) return FALSE
        MOVCS   R0, #0
		MOVCSS  PC, LR
        
        LDR     R0, =VectStackSpace				                        ;  if (IRQ�Ѽ���) return FALSE
        LDR     R3, [R0, R1, LSL #2] 
        CMP     R3, #0
        MOVNE   R0, #0
		MOVNES  PC, LR
        
        CMP     R2, #4                                                  ;  if (�������� >=4) return FALSE
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
;** Descriptions:            �����ѡ�ж�ͨ���ŵ�FIQ�ж�
;** input parameters:        R0:         �����Ӧ���ж�ͨ����
;** output parameters:       none
;** Returned value:          1:          �ɹ�
;**                          0:          ʧ��;
;** Created by:              LinEnqiang
;** Created Date:            2008-12-12
;**-------------------------------------------------------------------------------------------------------
;** Modified by:
;** Modified date:
;**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/

ClrmicFiq
		MSR     CPSR_c, #(NoFIQ :OR: NoInt :OR: SVC32Mode)
		
        CMP     R1, #32							                        ;  if (ͨ���� >=32) return FALSE
        MOVCS   R0, #0
		MOVCSS  PC, LR       
             
        LDR     R0, =MIC_ITR				                            ;  if(FIQδʹ��)return FALSE
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
;** Descriptions:            ������ѡ������жϴ������͡��жϷ�������ַ����ʹ���ж�
;** input parameters:        R0:         �����Ӧ���ж�ͨ����
;**                          R1:         �жϴ�������
;**                          R2:         �жϷ�������ַ
;** output parameters:       none
;** Returned value:          1:          �ɹ�
;**                          0:          ʧ��
;** Created by:              LinEnqiang
;** Created Date:            2008-12-12
;**-------------------------------------------------------------------------------------------------------
;** Modified by:
;** Modified date:
;**-------------------------------------------------------------------------------------------------------
;*********************************************************************************************************/
Setsic1IrqFunc
        CMP     R1, #32                                                 ;  if (ͨ���� >=32) return FALSE
        MOVCS   R0, #0
        MOVCSS  PC, LR
        
        CMP     R2, #4                                                  ;  if (�������� >=4) return FALSE
        MOVCS   R0, #0 
        MOVCSS  PC, LR 
       
        CMP     R3, #0                                                  ;  if (������ ==0) return FALSE
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
        
        LDR     R0, =VectStackSpace                                     ;  if (IRQ�Ѿ�ʹ��) return FALSE
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
;** Descriptions:            �����ѡ�����IRQ��Դ
;** input parameters:        R0:        �����Ӧ���ж�ͨ����
;** output parameters:       none
;** Returned value:          1:         �ɹ�
;**                          0:         ʧ��
;** Created by:              LinEnqiang
;** Created Date:            2008-12-12
;**-------------------------------------------------------------------------------------------------------
;** Modified by:
;** Modified date:
;**-------------------------------------------------------------------------------------------------------
;*********************************************************************************************************/
Clrsic1IrqFunc
        CMP     R1, #32 						                        ;  if (ͨ���� >=32) return FALSE
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
;** Descriptions:            ʹ����Ӧ������ж�
;** input parameters:        R0:         �����Ӧ���ж�ͨ����
;** output parameters:       none
;** Returned value:          1:          �ɹ�
;**                          0:          ʧ��
;** Created by:              LinEnqiang
;** Created Date:            2008-12-12
;**-------------------------------------------------------------------------------------------------------
;** Modified by:
;** Modified date:
;**-------------------------------------------------------------------------------------------------------
;*********************************************************************************************************/
Enablesic1Irq
        CMP     R1, #32                                                 ;  if (ͨ���� >=32) return FALSE
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
;** Descriptions:            ��ֹ��Ӧ������ж�
;** input parameters:        uiChannel:  �����Ӧ���ж�ͨ����
;** output parameters:       none
;** Returned value:          1:          �ɹ�
;**                          0:          ʧ��
;** Created by:              LinEnqiang
;** Created Date:            2008-12-12
;**-------------------------------------------------------------------------------------------------------
;** Modified by:
;** Modified date:
;**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
Disablesic1Irq
        CMP     R1, #32                                                 ;  if (ͨ���� >=32) return FALSE
        MOVCS   R0, #0
        MOVCSS  PC, LR 
        
        LDR     R0, =VectStackSpace                                     ;  if (δ����) return FALSE
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
;** Descriptions:            ���ò�ʹ����ѡ�ж�ͨ����ΪFIQ�ж�
;** input parameters:        R0:        �����Ӧ���ж�ͨ����
;** output parameters:       none
;** Returned value:          1:          �ɹ�
;**                          0:          ʧ��
;** Created by:              LinEnqiang
;** Created Date:            2008-12-12
;**-------------------------------------------------------------------------------------------------------
;** Modified by:
;** Modified date:
;**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
Setsic1Fiq
       	MSR     CPSR_c, #(NoFIQ :OR: NoInt :OR: SVC32Mode)
       	
        CMP     R1, #32							                        ;  if (ͨ���� >=32) return FALSE
        MOVCS   R0, #0
		MOVCSS  PC, LR
        
        LDR     R0, =VectStackSpace				                        ;  if (IRQ�Ѽ���) return FALSE
        MOV     R3, #1
        ADD     R0, R0,R3,LSL #7
        
        LDR     R3, [R0, R1,LSL #2] 
        CMP     R3, #0
        MOVNE   R0, #0
		MOVNES  PC, LR
        
        CMP     R2, #4                                                  ;  if (�������� >=4) return FALSE
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
;** Descriptions:            �����ѡ�ж�ͨ���ŵ�FIQ�ж�
;** input parameters:        R0:         �����Ӧ���ж�ͨ����
;** output parameters:       none
;** Returned value:          1:          �ɹ�
;**                          0:          ʧ��;
;** Created by:              LinEnqiang
;** Created Date:            2008-12-12
;**-------------------------------------------------------------------------------------------------------
;** Modified by:
;** Modified date:
;**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/

Clrsic1Fiq
		MSR     CPSR_c, #(NoFIQ :OR: NoInt :OR: SVC32Mode)
		
        CMP     R1, #32							                        ;  if (ͨ���� >=32) return FALSE
        MOVCS   R0, #0
		MOVCSS  PC, LR       
             
        LDR     R0, =SIC1_ITR				                            ;  if(FIQδʹ��)return FALSE
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
;** Descriptions:            ������ѡ������жϴ������͡��жϷ�������ַ����ʹ���ж�
;** input parameters:        R0:         �����Ӧ���ж�ͨ����
;**                          R1:         �жϴ�������
;**                          R2:         �жϷ�������ַ
;** output parameters:       none
;** Returned value:          1:          �ɹ�
;**                          0:          ʧ��
;** Created by:              LinEnqiang
;** Created Date:            2008-12-12
;**-------------------------------------------------------------------------------------------------------
;** Modified by:
;** Modified date:
;**-------------------------------------------------------------------------------------------------------
;*********************************************************************************************************/
Setsic2IrqFunc
        CMP     R1, #32                                                 ;  if (ͨ���� >=32) return FALSE
        MOVCS   R0, #0
        MOVCSS  PC, LR
        
        CMP     R2, #4                                                  ;  if (�������� >=4) return FALSE
        MOVCS   R0, #0 
        MOVCSS  PC, LR 
       
        CMP     R3, #0                                                  ;  if (������ ==0) return FALSE
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
        
        LDR     R0, =VectStackSpace                                     ;  if (IRQ�Ѿ�ʹ��) return FALSE
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
;** Descriptions:            �����ѡ�����IRQ��Դ
;** input parameters:        R0:        �����Ӧ���ж�ͨ����
;** output parameters:       none
;** Returned value:          1:         �ɹ�
;**                          0:         ʧ��
;** Created by:              LinEnqiang
;** Created Date:            2008-12-12
;**-------------------------------------------------------------------------------------------------------
;** Modified by:
;** Modified date:
;**-------------------------------------------------------------------------------------------------------
;*********************************************************************************************************/
Clrsic2IrqFunc
        CMP     R1, #32 						                        ;  if (ͨ���� >=32) return FALSE
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
;** Descriptions:            ʹ����Ӧ������ж�
;** input parameters:        R0:         �����Ӧ���ж�ͨ����
;** output parameters:       none
;** Returned value:          1:          �ɹ�
;**                          0:          ʧ��
;** Created by:              LinEnqiang
;** Created Date:            2008-12-12
;**-------------------------------------------------------------------------------------------------------
;** Modified by:
;** Modified date:
;**-------------------------------------------------------------------------------------------------------
;*********************************************************************************************************/
Enablesic2Irq
        CMP     R1, #32                                                 ;  if (ͨ���� >=32) return FALSE
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
;** Descriptions:            ��ֹ��Ӧ������ж�
;** input parameters:        uiChannel:  �����Ӧ���ж�ͨ����
;** output parameters:       none
;** Returned value:          1:          �ɹ�
;**                          0:          ʧ��
;** Created by:              LinEnqiang
;** Created Date:            2008-12-12
;**-------------------------------------------------------------------------------------------------------
;** Modified by:
;** Modified date:
;**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
Disablesic2Irq
        CMP     R1, #32                                                 ;  if (ͨ���� >=32) return FALSE
        MOVCS   R0, #0
        MOVCSS  PC, LR 
        
        LDR     R0, =VectStackSpace                                     ;  if (δ����) return FALSE
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
;** Descriptions:            ���ò�ʹ����ѡ�ж�ͨ����ΪFIQ�ж�
;** input parameters:        R0:        �����Ӧ���ж�ͨ����
;** output parameters:       none
;** Returned value:          1:          �ɹ�
;**                          0:          ʧ��
;** Created by:              LinEnqiang
;** Created Date:            2008-12-12
;**-------------------------------------------------------------------------------------------------------
;** Modified by:
;** Modified date:
;**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
Setsic2Fiq
       	MSR     CPSR_c, #(NoFIQ :OR: NoInt :OR: SVC32Mode)
       	
        CMP     R1, #32							                        ;  if (ͨ���� >=32) return FALSE
        MOVCS   R0, #0
		MOVCSS  PC, LR
        
        LDR     R0, =VectStackSpace				                        ;  if (IRQ�Ѽ���) return FALSE
        MOV     R3, #2
        ADD     R0, R0, R3,LSL #7
        
        LDR     R3, [R0, R1, LSL #2] 
        CMP     R3, #0
        MOVNE   R0, #0
		MOVNES  PC, LR
        
        CMP     R2, #4                                                  ;  if (�������� >=4) return FALSE
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
;** Descriptions:            �����ѡ�ж�ͨ���ŵ�FIQ�ж�
;** input parameters:        R0:         �����Ӧ���ж�ͨ����
;** output parameters:       none
;** Returned value:          1:          �ɹ�
;**                          0:          ʧ��;
;** Created by:              LinEnqiang
;** Created Date:            2008-12-12
;**-------------------------------------------------------------------------------------------------------
;** Modified by:
;** Modified date:
;**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/

Clrsic2Fiq
		MSR     CPSR_c, #(NoFIQ :OR: NoInt :OR: SVC32Mode)
		
        CMP     R1, #32							                        ;  if (ͨ���� >=32) return FALSE
        MOVCS   R0, #0
		MOVCSS  PC, LR       
             
        LDR     R0, =SIC2_ITR				                            ;  if(FIQδʹ��)return FALSE
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
                    
    AREA    VectStacks, DATA, NOINIT, ALIGN = 2;                        ;/*  �����ջ�ռ�               */    
        
VectStackSpace  SPACE   VIC_STACK_LEGTH * 4                             ;/*  �жϷ����������ջ�ռ�     */
                 
	
    END
;/********************************************************************************************************
;	End Of File
;********************************************************************************************************/