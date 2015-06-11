/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			NANDFlash.h
** Last modified Date:  20110419
** Last Version:		1.0
** Descriptions:		NANDFlash����,ֻ����������������ʹ��
**
**--------------------------------------------------------------------------------------------------------
** Created by:			ZHANG Ye
** Created date:		20110419
** Version:				1.0
** Descriptions:		
**
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
** Version:
** Descriptions:
**
*********************************************************************************************************/
#define	__NANDFLASH_C
#include "NANDFlash.h"
/*
* NAND Controller Buffer Definition
*/
#define MLC_DATA				(*(volatile unsigned long  *)(0x200B0000))
#define MLC_DATAX(x)			(*(volatile unsigned long  *)(0x200A8000+(x<<2)))

/*
* NAND Flash Commands
*/
#define NAND_CMD_READ0			0x00				/* Read mode (1) command        */
#define NAND_CMD_READ1			0x01				/* Read mode (2) command        */
#define NAND_CMD_PAGEPROG		0x10				/* Auto program command         */
#define NAND_CMD_READSTART		0x30				/* Read start command           */
#define NAND_CMD_READ2			0x50				/* Read mode (3) command        */
#define NAND_CMD_ERASE1ST		0x60				/* Auto block erase 1-st command*/
#define NAND_CMD_STATUS			0x70				/* Status read (1) command      */
#define NAND_CMD_STATUS_MULTI	0x71				/* Status read (2) command      */
#define NAND_CMD_SDIN			0x80				/* Serial data input command    */
#define NAND_CMD_READID			0x90				/* ID read (1) command          */
#define NAND_CMD_ERASE2ND		0xD0				/* Auto block erase 2-nd command*/
#define NAND_CMD_RESET			0xFF				/* Reset command                */


static	uint8	SetPageAddr(uint32 p_u32PageAddr);		//����ҳ��ַ
static	uint8	SetBlockAddr(uint32 p_u32BlockAddr);	//���ÿ��ַ
static	void	DelayNS(uint32 idly);					//��ʱ��ר��

/*********************************************************************************************************
** Function name:		NandInit
** Descriptions:		NandFlash��������ʼ��
**
** input parameters:	None
** output parameters:	None
** Returned value:		None
**
** Created by:			ZHANG Ye
** Created Date:		20110419
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void NandInit(void)
{
	FLASHCLK_CTRL = 0x00000002;                                         /* ����NAND Flashʱ��           */
    MLC_CEH       = 0;                                                  
    MLC_CMD       = NAND_CMD_RESET;                                     /* ��λNAND Flash               */
    while (!(MLC_ISR & 0x01));

    MLC_LOCK_PR     = 0xA25E;                                           /* ���� MLC_TIME_REG�Ĵ���      */
    
    MLC_TIME_REG    = (5 << 0)                                          /* ����NAND Flashʱ�����       */
                     |(4 << 4)                                         
                     |(5 << 8)                                         
                     |(15 << 12)                                        
                     |(5 << 16)                                        
                     |(7  << 19)                                        
                     |(5 << 24);                                       
    
    MLC_LOCK_PR     = 0xA25E;                                           /* ���� MLC_ICR�Ĵ���           */    
    MLC_ICR         = (0 << 0)                                          /*  8bit                        */
                     |(1 << 1)                                          
                     |(1 << 2)                                          
                     |(0 << 3);                                         /*  ���� WP                     */
    while (!(MLC_ISR & 0x01));
    MLC_CEH  = 1;

	return;
}

/*********************************************************************************************************
** Function name:		SetPageAddr
** Descriptions:		����ҳ��ַ
**
** input parameters:	p_u32PageAddr ҳ��ַ
** output parameters:	None
** Returned value:		�������	TRUE:�ɹ��� FALSE��ʧ��
**
** Created by:			ZHANG Ye
** Created Date:		20110419
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
uint8  SetPageAddr(uint32 p_u32PageAddr)
{
	MLC_ADDR = 0;				/*  ҳ��ʼ�ֽ�                   */
	MLC_ADDR = 0;
	/*  
	* ����ҳ��ַ                
	*/
	MLC_ADDR = (uint8)(p_u32PageAddr);
	MLC_ADDR = (uint8)(p_u32PageAddr >> 8);
	MLC_ADDR = (uint8)(p_u32PageAddr >> 16)&0x1f;		//�����ܹ�������2M
	
	return TRUE;                                                         
}

/*********************************************************************************************************
** Function name:		SetBlockAddr
** Descriptions:		����ҳ��ַ
**
** input parameters:	p_u32BlockAddr ���ַ
** output parameters:	None
** Returned value:		�������	TRUE:�ɹ��� FALSE��ʧ��
**
** Created by:			ZHANG Ye
** Created Date:		20110419
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
uint8  SetBlockAddr(uint32 p_u32BlockAddr)
{
	MLC_ADDR = (uint8)(p_u32BlockAddr & 0xff);
    MLC_ADDR = (uint8)(p_u32BlockAddr >> 8);
    MLC_ADDR = (uint8)(p_u32BlockAddr >> 16)&0x1f;
     
    return TRUE;                                                            
}

/*********************************************************************************************************
** Function name:		EraseNandBlock
** Descriptions:		����ָ��Block����
**
** input parameters:	p_u32BlockAddress	���ַ
** output parameters:	None
** Returned value:		�������	TRUE:�ɹ��� FALSE��ʧ��
**
** Created by:			ZHANG Ye
** Created Date:		20110419
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
uint8 EraseNandBlock(uint32 p_u32BlockAddress)
{
	while (!(MLC_ISR & 0x01));					/* �ȴ�NAND Flash����           */
	MLC_CMD  = NAND_CMD_ERASE1ST;
	SetBlockAddr(p_u32BlockAddress);
	MLC_CMD  = NAND_CMD_ERASE2ND;
	
	return TRUE;  
}

/*********************************************************************************************************
** Function name:		DelayNS
** Descriptions:		�����ʱ
**
** input parameters:	idly	
** output parameters:	None
** Returned value:		None
**
** Created by:			ZHANG Ye
** Created Date:		20110419
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void  DelayNS(unsigned int  idly)
{  
	unsigned int  i;
	for(; idly>0; idly--){
		for(i=0; i<5000; i++);
	}
}

/*********************************************************************************************************
** Function name:		ReadNandPage
** Descriptions:		��Flash��ָ����ҳ
**
** input parameters:	p_u32Address	���NOR�׵�ַ��ƫ�Ƶ�ַ0~0x1fffff
**						p_u32ReadBytes	��ȡ�ֽ���
**						p_pu8Data		����ָ��
** output parameters:	None
** Returned value:		�������	TRUE:�ɹ��� FALSE��ʧ��
**
** Created by:			ZHANG Ye
** Created Date:		20110419
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
uint8 ReadNandPage(uint32 p_u32Address, uint32 p_u32ReadBytes, uint8 * p_pu8Data)
{
	unsigned int j, i, l_u32Cnt;
	unsigned long *ptr_ul	= (unsigned long *)p_pu8Data;
	MLC_CEH       = 0; 
	l_u32Cnt	= 0;
	if(p_pu8Data == NULL)
	    return(FALSE);
	    
	while (!(MLC_ISR & 0x01));   								/* �ȴ�NAND Flash����           */
	MLC_CMD = NAND_CMD_READ0;
	SetPageAddr(p_u32Address);   								/* ����ҳ��ַ                   */
	while (!(MLC_ISR & 0x01));   								/* �ȴ�NAND Flash����           */
	
	while (!(MLC_ISR & 0x01));      							/* �ȴ�NAND Flash����           */
	
	MLC_CMD = NAND_CMD_READSTART;
	
	for(j=0;j<4;j++) {   
	    MLC_ECC_AUTO_DEC_REG = 0x00;							/* ʹ���Զ�����                 */
	    while (!(MLC_ISR & 0x02));           					/* �ȴ����������               */
	    while (!(MLC_ISR & 0x04));  							/* �ȴ�ECCУ�����              */
	    if (MLC_ISR & 0x40)
	        ;                          							/* ����������                 */
	        
	    DelayNS(10);
	    
	    for (i=0; i<((512)>>2); i++) { 							/* ��ȡ����                     */
	        *ptr_ul++ =  MLC_DATAX(i);							/* MLC_DATAX(i);                */
	        l_u32Cnt ++;
	        if((l_u32Cnt << 2) >= p_u32ReadBytes)					/*  ��ȡ����������˳�          */
	            break; 
	    }
	    if((l_u32Cnt << 2) >= p_u32ReadBytes)
	        break;
	} 
	
	MLC_CEH		= 1; 
	return TRUE; 
}

/*********************************************************************************************************
** Function name:		ProgramNandPage
** Descriptions:		дFlash��ҳ
**
** input parameters:	p_u32Address	���NAND�׵�ַ��ƫ�Ƶ�ַ0~0x1fffff
**						p_u32WriteNumber	��ȡ�ֽ���
** output parameters:	p_u16Data		����ָ��
** Returned value:		�������	TRUE:�ɹ��� FALSE��ʧ��
**
** Created by:			ZHANG Ye
** Created Date:		20110419
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
uint8 ProgramNandPage(uint32 p_u32Address, uint32 p_u32WriteBytes, uint8 * p_pu8Data)
{
	unsigned int j, i, l_u32Cnt;
    unsigned long *ptr_ul = (unsigned long *)p_pu8Data;
    
    MLC_CEH		= 0; 
	l_u32Cnt	= 0;

    while (!(MLC_ISR & 0x01));                                          /* �ȴ�NAND Flash����           */
    MLC_CMD = NAND_CMD_SDIN;
    SetPageAddr(p_u32Address);
    
    for(j=0;j<4;j++) {
        MLC_ECC_ENC_REG = 0;
        for (i=0; i<(512>>2); i++)  {                                   /* װ�����ݵ���������           */
            MLC_DATAX(i) = *ptr_ul++;
            l_u32Cnt ++;
        }

        for (i=(512>>2); i<(520>>2); i++)                               /* װ����չ����������           */
            MLC_DATAX(i) = 0xffffffff;
        
        MLC_ECC_AUTO_ENC_REG = 0x10;
        while (!(MLC_ISR & 0x02));                                      /* �ȴ����������               */
        
        if(l_u32Cnt >= p_u32WriteBytes)
            break;
    }
    MLC_CMD		= NAND_CMD_PAGEPROG;

    MLC_CEH		= 1; 

    return TRUE;
}

