;����ϵͳģʽ��ջ�Ĵ�С
SVC_STACK_LEGTH     EQU         32

NoInt       EQU 0x80

USR32Mode   EQU 0x10
SVC32Mode   EQU 0x13
SYS32Mode   EQU 0x1f
IRQ32Mode   EQU 0x12
FIQ32Mode   EQU 0x11

;T_bit���ڼ������쳣ǰcpu�Ƿ���THUMB״̬
T_bit               EQU         0x20

    CODE32
	PRESERVE8
    AREA    |subr|, CODE, READONLY

			;IMPORT	vicControl
            IMPORT  OSTCBCur                    ;ָ��ǰ����TCB��ָ��
            IMPORT  OSTCBHighRdy                ;ָ��Ҫ���е�����TCB��ָ��
            IMPORT  OSPrioCur                   ;��ǰ��������ȼ�
            IMPORT  OSPrioHighRdy               ;��Ҫ���е���������ȼ�
            IMPORT  OSTaskSwHook                ;�����л��Ĺ��Ӻ���
            IMPORT  OSRunning                   ;uC/OS-II���б�־

            IMPORT  OsEnterSum                  ;���жϼ����������ж��ź�����
            ;IMPORT  SWI_Exception               ;���ж��쳣�������
            IMPORT  OSIntNesting        
            
			IMPORT  _OSFunctionAddr
            IMPORT  _UsrFunctionAddr

            IMPORT  _TaskIsARM
            IMPORT  _TaskIsTHUMB
            
            EXPORT  __OSStartHighRdy            
            EXPORT  OSIntCtxSw                  ;�ж��˳�ʱ����ڣ��μ�startup.s�е�IRQ_Handler
            EXPORT  SoftwareInterrupt           ;���ж����


;/*********************************************************************************************************
;** ��������: SoftwareInterrupt
;** ��������: ����жϣ������ṩһЩϵͳ���񣬹��ܲο�os_cpu_c.c�ļ�
;** �䡡��:   �����ܶ���
;** �䡡�� :  �����ܶ���
;** ȫ�ֱ���: ��
;** ����ģ��: SWI_Exception
;** 
;** ������: 
;**-------------------------------------------------------------------------------------------------------
;** �ޡ���: 
;**-------------------------------------------------------------------------------------------------------
;** �ޡ���: 
;**-------------------------------------------------------------------------------------------------------
;** �ޡ���: 
;**-------------------------------------------------------------------------------------------------------
;********************************************************************************************************/

;����ж�
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
        MRS     R3, SPSR                        ;���������CPSR
        MOV     R2, LR                          ;���������PC
        
        MSR     CPSR_c, #(NoInt :OR: SYS32Mode)    ;�л���ϵͳģʽ
        STMFD   SP!, {R2}                       ;����PC����ջ
        STMFD   SP!, {R0-R12, LR}               ;����R0-R12,LR����ջ
                                                ;��ΪR0~R3û�б����������ݣ����Կ���������
        B       OSIntCtxSw_0                    ;�������������л�

ENTER_CRITICAL
                                                ;OsEnterSum++
        LDR     R1, =OsEnterSum
        LDRB    R2, [R1]
        ADD     R2, R2, #1
        STRB    R2, [R1]
                                                ;���ж�
        MRS     R0, SPSR
        ORR     R0, R0, #NoInt
        MSR     SPSR_c, R0
        MOVS    PC, LR

EXIT_CRITICAL
                                                ;OsEnterSum--
        LDR     R1, =OsEnterSum	 
        LDRB    R2, [R1]		; //LDR{cond}B{T} Rd��<��ַ>    ָ�����ָ����ַ���ֽ����ݵ�Rd�ĵ�����ֽ��У�Rd�ĸ�24λ���㣩��
        SUB     R2, R2, #1	;//����
        STRB    R2, [R1]	;//STR{cond}B{T} Rd�� <��ַ>   ָ��洢Rd�е�����ֽ����ݵ�ָ���ĵ�ַ��Ԫ�С�
                                                ;if(OsEnterSum == 0) ���ж�;
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
                                                ;�л���ϵͳģʽ
        MRS     R0, SPSR
        BIC     R0, R0, #0x1f
        ORR     R0, R0, #SYS32Mode    
        MSR     SPSR_c, R0
        MOVS    PC, LR

ChangeToUSRMode
                                                ;�л����û�ģʽ
        MRS     R0, SPSR
        BIC     R0, R0, #0x1f
        ORR     R0, R0, #USR32Mode    
        MSR     SPSR_c, R0
        MOVS    PC, LR

_OSStartHighRdy
        B       __OSStartHighRdy

TaskIsARM
        MSR     CPSR_c, #(NoInt :OR: SYS32Mode)    ; �л���ϵͳģʽ
        MOV     R0, R1
        BL      _TaskIsARM
        MSR     CPSR_c, #(NoInt :OR: SVC32Mode)    ; �л��ع���ģʽ
        MOVS    PC, LR

TaskIsTHUMB
        MSR     CPSR_c, #(NoInt :OR: SYS32Mode)    ; �л���ϵͳģʽ
        MOV     R0, R1
        BL      _TaskIsTHUMB
        MSR     CPSR_c, #(NoInt :OR: SVC32Mode)    ; �л��ع���ģʽ
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
;** ��������: OSIntCtxSw
;** ��������: �ж��˳�ʱ�����
;** �䡡��:   R3    :��ǰ�����״̬�Ĵ���CPSR����SPSR��ֵ��
;**           R4-R12:��ǰ�����R4-R11
;**           ��ǰ������ģʽ�Ķ�ջ�ṹ����ջ���򣩣�R0-R3��R12��PC����ǰ����ģ�
;** �䡡�� :  ��
;** ȫ�ֱ���: OSPrioCur,OSPrioHighRdy,OSPrioCur,OSPrioHighRdy
;** ����ģ��: ��
;** 
;**-------------------------------------------------------------------------------------------------------
;********************************************************************************************************/
OSIntCtxSw
                                                    ;����Ϊ�������񻷾�
        LDR     R2, [SP, #20]                       ;��ȡPC
        LDR     R12, [SP, #16]                      ;��ȡR12
        MRS     R0, CPSR

        MSR     CPSR_c, #(NoInt :OR: SYS32Mode)
        MOV     R1, LR
        STMFD   SP!, {R1-R2}                        ;����LR,PC
        STMFD   SP!, {R4-R12}                       ;����R4-R12

        MSR     CPSR_c, R0
        LDMFD   SP!, {R4-R7}                        ;��ȡR0-R3
        ADD     SP, SP, #8                          ;��ջR12,PC
        
        MSR     CPSR_c, #(NoInt :OR: SYS32Mode)
        STMFD   SP!, {R4-R7}                        ;����R0-R3
        
OSIntCtxSw_0        
        LDR     R1, =OsEnterSum                     ;��ȡOsEnterSum
        LDR     R2, [R1]
        STMFD   SP!, {R2, R3}                       ;����CPSR,OsEnterSum

                                                    ;���浱ǰ�����ջָ�뵽��ǰ�����TCB
        LDR     R1, =OSTCBCur
        LDR     R1, [R1]
        STR     SP, [R1]

        BL      OSTaskSwHook                        ;���ù��Ӻ���
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
                                                    ;��ȡ�������ջָ��
        LDR     R4, [R6]
        ADD     SP, R4, #68                         ;17�Ĵ���CPSR,OsEnterSum,R0-R12,LR,SP
        LDR     LR, [SP, #-8]
        MSR     CPSR_c, #(NoInt :OR: SVC32Mode)        ;�������ģʽ
        MOV     SP, R4                              ;���ö�ջָ��

        LDMFD   SP!, {R4, R5}                       ;CPSR,OsEnterSum
                                                    ;�ָ��������OsEnterSum
        LDR     R3, =OsEnterSum
        STR     R4, [R3]
    
        MSR     SPSR_cxsf, R5                       ;�ָ�CPSR
        LDMFD   SP!, {R0-R12, LR, PC }^             ;����������

;/*********************************************************************************************************
;** ��������: __OSStartHighRdy
;** ��������: uC/OS-II����ʱʹ��OSStartHighRdy���е�һ������,
;**           OSStartHighRdy�����__OSStartHighRdy
;** �䡡��:   ��
;** �䡡�� :  ��
;** ȫ�ֱ���: OSRunning,OSTCBCur,OSTCBHighRdy,OsEnterSum
;** ����ģ��: OSTaskSwHook
;** 
;** ������: 
;** �ա���: 2003��6��5��
;**-------------------------------------------------------------------------------------------------------
;** �ޡ���: 
;** �ա���: 2003��6��13��
;**-------------------------------------------------------------------------------------------------------
;********************************************************************************************************/

__OSStartHighRdy
        MSR     CPSR_c, #(NoInt :OR: SYS32Mode)
                                                ;����uC/OS-II�����Ѿ�����
        LDR     R4, =OSRunning
        MOV     R5, #1
        STRB    R5, [R4]

        BL      OSTaskSwHook                    ;���ù��Ӻ���

        LDR     R6, =OSTCBHighRdy
        LDR     R6, [R6]
        B       OSIntCtxSw_1

        AREA    SWIStacks, DATA, NOINIT,ALIGN=2
SvcStackSpace      SPACE   SVC_STACK_LEGTH * 4  ;����ģʽ��ջ�ռ�

    END
;/*********************************************************************************************************
;**                            End Of File
;********************************************************************************************************/
