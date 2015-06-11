/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			NORFlash.c
** Last modified Date:  20110419
** Last Version:		1.0
** Descriptions:		NORFlash驱动
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
#define	__NORFLASH_C
#include "NORFlash.h"
				 
#define  FLASH_ADDR     0xe0000000                                      // NOR Flash 接在BANK0上
#define  GetAddr(addr)	(volatile uint16  *)(FLASH_ADDR|(addr << 1))	//相对地址转换为绝对地址

/*********************************************************************************************************
** Function name:		EraseNORFlash
** Descriptions:		擦除所有NORFlash数据
**
** input parameters:	None
** output parameters:	None
** Returned value:		操作结果	TRUE:成功； FALSE：失败
**
** Created by:			ZHANG Ye
** Created Date:		20110419
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
uint8 EraseNORFlash(void)
{
	volatile	uint16 *	l_pu16Temp;
				uint16		l_u16Temp1;
				uint16		l_u16Temp2;
	
    l_pu16Temp		= GetAddr(0x555);
    l_pu16Temp[0]	= 0xaaaa;						/* 第一个周期,地址0x5555,数据0xAA*/
    l_pu16Temp		= GetAddr(0x2aa);
    l_pu16Temp[0]	= 0x5555;						/* 第二个周期,地址0x2aaa,数据0x55*/
    l_pu16Temp		= GetAddr(0x555);
    l_pu16Temp[0]	= 0x8080;						/* 第三个周期,地址0x5555,数据0x80*/
    l_pu16Temp		= GetAddr(0x555);
    l_pu16Temp[0]	= 0xaaaa;						/* 第四个周期,地址0x5555,数据0xAA*/
    l_pu16Temp		= GetAddr(0x2aa);
    l_pu16Temp[0]	= 0x5555;						/* 第五个周期,地址0x2aaa,数据0x55*/
    l_pu16Temp		= GetAddr(0x555);
    l_pu16Temp[0]	= 0x1010;						/* 第六个周期,地址0x5555,数据0x10*/

    /* 
     *  等待操作完成 (若擦除操作没有完成，每次读操作DQ6会跳变) 
     */
    while (1)				
    {  
        l_u16Temp1	= *l_pu16Temp;
        l_u16Temp2	= *l_pu16Temp;
        if (l_u16Temp1 == l_u16Temp2)
        {  
            if (l_u16Temp1 != 0xffff)
                return(FALSE);
            else
                return(TRUE);
        }
    }
}

/*********************************************************************************************************
** Function name:		EraseBlock
** Descriptions:		擦除Block数据
**
** input parameters:	p_u32BlockID
** output parameters:	None
** Returned value:		操作结果	TRUE:成功； FALSE：失败
**
** Created by:			ZHANG Ye
** Created Date:		20110419
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
uint8 EraseBlock(uint32 p_u32BlockID)
{
	volatile	uint16 *	l_pu16Temp;
				uint16		l_u16Temp1;
				uint16		l_u16Temp2;
	
    l_pu16Temp		= GetAddr(0x555);
    l_pu16Temp[0]	= 0xaaaa;						/* 第一个周期,地址0x5555,数据0xAA*/
    l_pu16Temp		= GetAddr(0x2aa);
    l_pu16Temp[0]	= 0x5555;						/* 第二个周期,地址0x2aaa,数据0x55*/
    l_pu16Temp		= GetAddr(0x555);
    l_pu16Temp[0]	= 0x8080;						/* 第三个周期,地址0x5555,数据0x80*/
    l_pu16Temp		= GetAddr(0x555);
    l_pu16Temp[0]	= 0xaaaa;						/* 第四个周期,地址0x5555,数据0xAA*/
    l_pu16Temp		= GetAddr(0x2aa);
    l_pu16Temp[0]	= 0x5555;						/* 第五个周期,地址0x2aaa,数据0x55*/
    l_pu16Temp		= GetAddr(p_u32BlockID>>1);
#ifdef	__ZLG
    l_pu16Temp[0]	= 0x5050;						/* 第六个周期,块地址,数据0x50  */
#else				//非周立功
	l_pu16Temp[0]	= 0x3030;						/* 第六个周期,块地址,数据0x30  */
#endif
    
    /* 
     *  等待操作完成 (若擦除操作没有完成，每次读操作DQ6会跳变) 
     */
    while (1)				
    {  
        l_u16Temp1	= *l_pu16Temp;
        l_u16Temp2	= *l_pu16Temp;
        if (l_u16Temp1 == l_u16Temp2)
        {  
            if (l_u16Temp1 != 0xffff)
                return(FALSE);
            else
                return(TRUE);
        }
    }
}

/*********************************************************************************************************
** Function name:		EraseSector
** Descriptions:		擦除Sector数据
**
** input parameters:	p_u32SectorAddr
** output parameters:	None
** Returned value:		操作结果	TRUE:成功； FALSE：失败
**
** Created by:			ZHANG Ye
** Created Date:		20110419
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
uint8 EraseSector(uint32 p_u32SectorAddr)
{
	volatile	uint16 *	l_pu16Temp;
				uint16		l_u16Temp1;
				uint16		l_u16Temp2;
	
    l_pu16Temp		= GetAddr(0x555);
    l_pu16Temp[0]	= 0xaaaa;						/* 第一个周期,地址0x5555,数据0xAA*/
    l_pu16Temp		= GetAddr(0x2aa);
    l_pu16Temp[0]	= 0x5555;						/* 第二个周期,地址0x2aaa,数据0x55*/
    l_pu16Temp		= GetAddr(0x555);
    l_pu16Temp[0]	= 0x8080;						/* 第三个周期,地址0x5555,数据0x80*/
    l_pu16Temp		= GetAddr(0x555);
    l_pu16Temp[0]	= 0xaaaa;						/* 第四个周期,地址0x5555,数据0xAA*/
    l_pu16Temp		= GetAddr(0x2aa);
    l_pu16Temp[0]	= 0x5555;						/* 第五个周期,地址0x2aaa,数据0x55*/
    l_pu16Temp		= GetAddr(p_u32SectorAddr>>1);
#ifdef	__ZLG
    l_pu16Temp[0]	= 0x3030;						/* 第六个周期,块地址,数据0x30  */
#else				//非周立功
	l_pu16Temp[0]	= 0x5050;						/* 第六个周期,块地址,数据0x50  */
#endif
    /* 
     *  等待操作完成 (若擦除操作没有完成，每次读操作DQ6会跳变) 
     */
    while (1)				
    {  
        l_u16Temp1	= *l_pu16Temp;
        l_u16Temp2	= *l_pu16Temp;
        if (l_u16Temp1 == l_u16Temp2)
        {  
            if (l_u16Temp1 != 0xffff)
                return(FALSE);
            else
                return(TRUE);
        }
    }
}

/*********************************************************************************************************
** Function name:		WordProgram
** Descriptions:		写WORD
**
** input parameters:	p_u32Address	地址
**						p_u16Data		数据
** output parameters:	None
** Returned value:		操作结果	TRUE:成功； FALSE：失败
**
** Created by:			ZHANG Ye
** Created Date:		20110419
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
uint8 WordProgram(uint32 p_u32Address, uint16 p_u16Data)
{
	volatile	uint16 *	l_pu16Temp;
				uint16		l_u16Temp1;
				uint16		l_u16Temp2;
	
	l_pu16Temp		= GetAddr(0x555);
	l_pu16Temp[0]	= 0xaaaa;						/* 第一个周期,地址0x5555,数据0xAA*/
	l_pu16Temp		= GetAddr(0x2aa);
	l_pu16Temp[0]	= 0x5555;						/* 第二个周期,地址0x2aaa,数据0x55*/
	l_pu16Temp		= GetAddr(0x555);
	l_pu16Temp[0]	= 0xa0a0;						/* 第三个周期,地址0x5555,数据0x80*/
	
	l_pu16Temp		= (uint16 *)(FLASH_ADDR|(p_u32Address & 0x1FFFFF));
	*l_pu16Temp		= p_u16Data;					/* 第四个周期,地址,数据  */
    
	/* 
	 *  等待操作完成 (若擦除操作没有完成，每次读操作DQ6会跳变) 
	 */
	while (1)				
	{  
		l_u16Temp1	= *l_pu16Temp;
		l_u16Temp2	= *l_pu16Temp;
		if (l_u16Temp1 == l_u16Temp2)
		{  
			if (l_u16Temp1 != p_u16Data)
				return(FALSE);
			else
				return(TRUE);
		}
	}
}

/*********************************************************************************************************
** Function name:		ReadNORFlash
** Descriptions:		读Flash中指定位置数据，每四位读取
**
** input parameters:	p_u32Address	相对NOR首地址的偏移地址0~0x1fffff
**						p_u32ReadBytes	读取字节数
**						p_pu8Data		数据指针
** output parameters:	None
** Returned value:		操作结果	TRUE:成功； FALSE：失败
**
** Created by:			ZHANG Ye
** Created Date:		20110419
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
uint8 ReadNORFlash(uint32 p_u32Address, uint32 p_u32ReadBytes, uint8 * p_pu8Data)
{
	uint32		i;
	uint32 *	l_pu32Data;

	l_pu32Data	= (uint32 *)p_pu8Data;

	for (i = 0; i < p_u32ReadBytes; i += 4)
	{
		//读NORFlash
		*(l_pu32Data ++)	= *(uint32 *)(FLASH_ADDR|((p_u32Address+i) & 0x1FFFFF));
	}
	return	TRUE;
}

/*********************************************************************************************************
** Function name:		WriteNORFlash
** Descriptions:		写Flash中指定位置数据
**
** input parameters:	p_u32Address	相对NOR首地址的偏移地址0~0x1fffff
**						p_u32WriteNumber	读取字节数
** output parameters:	p_u16Data		数据指针
** Returned value:		操作结果	TRUE:成功； FALSE：失败
**
** Created by:			ZHANG Ye
** Created Date:		20110419
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
uint8 WriteNORFlash(uint32 p_u32Address, uint32 p_u32WriteNumber, uint8 * p_pu8Data)
{
	uint32		i;
	uint16 *	l_pu16Data;
	uint8		l_u8Res;
	
	l_u8Res		= TRUE;

#ifdef	__UCOS		//在UCOS中，写NORFlash时关中断，防止写Flash过程中被打断
	OS_ENTER_CRITICAL();
#endif

	l_pu16Data	= (uint16 *)p_pu8Data;
	
//	EraseSector(p_u32Address);		//擦除一个扇区，否则不能对这个空间进行写操作，空间为2K	
	
	for (i = 0; i < p_u32WriteNumber; i += 2)
	{
		while(FALSE == WordProgram(p_u32Address + i, *(l_pu16Data + (i >> 1))));		//将数据写入NORFlash
	}
	
#ifdef	__UCOS
	OS_EXIT_CRITICAL();
#endif

	return	l_u8Res;
}

