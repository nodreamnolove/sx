/****************************************Copyright (c)****************************************************
**                                     BEIJING  WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			DS2460.c
** Last modified Date:  2011-04-13
** Last Version:		1.0
** Descriptions:		2460/ESAM相关函数
**
**--------------------------------------------------------------------------------------------------------
** Created by:			ZHANG Ye
** Created date:		20110413
** Version:				1.0
** Descriptions:		2460/ESAM
**
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
** Version:
** Descriptions:
**
*********************************************************************************************************/
#define	__DS2460_C
#include "DS2460.h"
#include "I2C1.h"
#include "SHA.h"

/****************************************************
**
**   DS2460相关变量定义
**
****************************************************/
#define		DS2460_ADDR_W		0x80
#define		DS2460_ADDR_R		0x81

#define		MACOUTPUT			0		//是否使用读缓存的方式输出MAC:1,是；0，否

uint8	getMK(void);
uint8	WriteCSK(uint8 * p_au8CSK);
uint8	generateCSK(void);
uint8	generateRND(void);
uint8	ComputeArmMAC(void);

struct	EEPInfo
{
	//总共112个Byte
	uint8	Flag;			//1		标记
							//0000 0000	从左起：
							//第1~4位：	是否合法，0110表示合法
							//其他位：	保留
	uint16	UpdateTimes;	//2		刷程序次数
	

	
}	m_eepInfo;

uint8	m_au8MK[8];				//MK
uint8	m_au8SN[8];				//ROM SN
uint8	m_au8CSK[8];			//CSK
uint8	m_Input_buffer[64];		//输入缓冲器的输入buffer
uint8	m_au8MACARM[20];		//生成的MAC值
uint8	m_u8ACKFlag;			//应答标志：0表示无应答，1表示有应答

uint8	m_u8ChkResult;			//校验结果，0表示通过，1表示不通过
uint8	m_au8RND[64];			//随机数

#if	MACOUTPUT == 0				//不允许直接读MAC OUTput buffer
uint8	m_u8Command	= 0x8C;		//0x8C	10001100	选择E-SECRET1作为输入，MAC OUTput buffer作为输出
#else							//允许直接读MAC OUTput buffer
uint8	m_au8MAC2460[20];		//2460的MAC值
uint8	m_u8Command	= 0x94;		//0x94	10010100	选择E-SECRET2作为输入，MAC OUTput buffer作为输出
#endif

uint8	m_au8InputBuffer1[64] = {0x3c,0xe2,0x99,0x18,		//没用	  
                0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,
                0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x10,0x11,0x12,0x13,
				0x14,0x15,0x16,0x17,0x18,0x19,0x1A,0x1B,0x1C,0x1D,
				0x1E,0x1F,0x20,0x21,0x22,0x23,0x24,0x25,0x26,0x27,
				0x28,0x29,0x2A,0x2B,0x00,0x00,0x00,0x36,0x2C,0x2D,
				0x2E,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0xB8};


#ifdef	USEMODE
/*********************************************************************************************************
** Function name:		CheckESAM
** Descriptions:		ESAM校验
**
** input parameters:	None
** output parameters:	None
** Returned value:		校验结果：	1(>0)，	失败
**									0，		成功
** Created by:			ZHANG Ye
** Created Date:		20110413
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
uint8 CheckESAM(void)
{
#if	MACOUTPUT > 0				//允许直接读MAC OUTput buffer
	uint8	i;
#endif


	//获取SN,MK
	ReadSN(m_au8SN);
	getMK();
	OSTimeDly(1);	//10ms

	//生成CSK
	generateCSK();
	OSTimeDly(1);	//10ms

	//生成随机数
	generateRND();
	OSTimeDly(1);	//10ms

	//计算MAC
	ComputeArmMAC();
	OSTimeDly(1);	//10ms

	//使2460计算MAC
	I2C1_WriteNByte(DS2460_ADDR_W, 1, 0x00, m_au8RND, 64);
	OSTimeDly(1);	//10ms

	//写入命令码启动计算MAC
	I2C1_WriteNByte(DS2460_ADDR_W, 1, 0x5C, &m_u8Command, 1);
	OSTimeDly(1);	//10ms

#if	MACOUTPUT == 0				//不允许直接读MAC OUTput buffer
	
	//向I2C写MAC
	I2C1_WriteNByte(DS2460_ADDR_W, 1, 0x5E, m_au8MACARM, 20);
	OSTimeDly(1);	//10ms
			
	//根据第20位是否响应来判断验证是否通过
	if (m_u8ACKFlag == 0)	//无应答，失败
		return	1;
	else					//有应答，通过
		return	0;		

#else							//允许直接读MAC OUTput buffer
	I2C_ReadNByte(DS2460_ADDR_R, 1, 0x40, m_au8MAC2460, 20);
	OSTimeDly(1);	//10ms

	for (i=0; i<20; i++)
	{
		if (m_au8MAC2460[i] != m_au8MACARM[i])
			return	1;
	}
	return	0;
	
#endif		
	
}

#endif

/*********************************************************************************************************
** Function name:		getMK
** Descriptions:		获取MK
**
** input parameters:	None
** output parameters:	None
** Returned value:		操作结果：	1(>0)，	失败
**									0，		成功
** Created by:			ZHANG Ye
** Created Date:		20110413
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
uint8 getMK(void)
{
	uint8 *	l_pu8Temp;
	l_pu8Temp = m_au8MK;

	//临时
	*(l_pu8Temp++)	= 0x11;
	*(l_pu8Temp++)	= 0x22;
	*(l_pu8Temp++)	= 0x33;
	*(l_pu8Temp++)	= 0x44;
	*(l_pu8Temp++)	= 0x55;
	*(l_pu8Temp++)	= 0x66;
	*(l_pu8Temp++)	= 0x77;
	*(l_pu8Temp++)	= 0x88;
	l_pu8Temp	= 0;
	return	0;
}

/*********************************************************************************************************
** Function name:		ComputeArmMAC
** Descriptions:		计算Arm的MAC
**
** input parameters:	None
** output parameters:	None
** Returned value:		操作结果：	1(>0)，	失败
**									0，		成功
** Created by:			ZHANG Ye
** Created Date:		20110413
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
uint8 ComputeArmMAC(void)
{
	uint8	l_u8Cnt;
	long	A, B, C, D, E;

	//此处应使用时间
	srand(0);
	
	for(l_u8Cnt=0; l_u8Cnt<4; l_u8Cnt++)
	{
		m_au8RND[l_u8Cnt] = m_au8CSK[l_u8Cnt];		//密钥低四位
	}
	for(l_u8Cnt=48; l_u8Cnt<52; l_u8Cnt++)		
	{
		m_au8RND[l_u8Cnt] = m_au8CSK[l_u8Cnt-44];	//密钥高四位
	}
	
	ComputeSHAEE(m_au8RND, &A, &B, &C, &D, &E);

	m_au8MACARM[0]	= (E)&0xff;
	m_au8MACARM[1]	= (E>>8)&0xff;
	m_au8MACARM[2]	= (E>>16)&0xff;
	m_au8MACARM[3]	= (E>>24)&0xff;
	m_au8MACARM[4]	= (D)&0xff;
	m_au8MACARM[5]	= (D>>8)&0xff;
	m_au8MACARM[6]	= (D>>16)&0xff;
	m_au8MACARM[7]	= (D>>24)&0xff;
	m_au8MACARM[8]	= (C)&0xff;
	m_au8MACARM[9]	= (C>>8)&0xff;
	m_au8MACARM[10]	= (C>>16)&0xff;
	m_au8MACARM[11]	= (C>>24)&0xff;
	m_au8MACARM[12]	= (B)&0xff;
	m_au8MACARM[13]	= (B>>8)&0xff;
	m_au8MACARM[14]	= (B>>16)&0xff;
	m_au8MACARM[15]	= (B>>24)&0xff;
	m_au8MACARM[16]	= (A)&0xff;
	m_au8MACARM[17]	= (A>>8)&0xff;
	m_au8MACARM[18]	= (A>>16)&0xff;
	m_au8MACARM[19]	= (A>>24)&0xff;	

	return	0;
}

/*********************************************************************************************************
** Function name:		generateRND
** Descriptions:		生成随机数Buf
**
** input parameters:	None
** output parameters:	None
** Returned value:		操作结果：	1(>0)，	失败
**									0，		成功
** Created by:			ZHANG Ye
** Created Date:		20110413
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
uint8 generateRND(void)
{
	uint8	l_u8Cnt;

	//此处应使用时间
	srand(0);
	
	for(l_u8Cnt=0; l_u8Cnt<4; l_u8Cnt++)
	{
		m_au8RND[l_u8Cnt] = 0x00;				//会被替换，该4位没有要求
	}
	for(l_u8Cnt =4; l_u8Cnt<48; l_u8Cnt++)
	{
		m_au8RND[l_u8Cnt] = rand() & 0xff;		//随机数
	}
	for(l_u8Cnt=48; l_u8Cnt<52; l_u8Cnt++)		
	{
		m_au8RND[l_u8Cnt] = 0x00;				//会被替换，该4位没有要求
	}
	for(l_u8Cnt =52; l_u8Cnt<55; l_u8Cnt++)
	{
		m_au8RND[l_u8Cnt] = rand() & 0xff;		//随机数
	}
	m_au8RND[55] = 0x80;
	for(l_u8Cnt=56; l_u8Cnt<62;l_u8Cnt++)
	{
		m_au8RND[l_u8Cnt] = 0x0;
	}
	m_au8RND[62]=0x01;
	m_au8RND[63]=0xB8;

	return	0;
}

/*********************************************************************************************************
** Function name:		generateCSK
** Descriptions:		根据MK和SN生成CSK
**
** input parameters:	None
** output parameters:	None
** Returned value:		操作结果：	1(>0)，	失败
**									0，		成功
** Created by:			ZHANG Ye
** Created Date:		20110413
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
uint8 generateCSK(void)
{
	uint8	l_u8Cnt;
	long	A, B, C, D, E;
	uint8 *	l_au8Temp;

	//生成BUF
	for(l_u8Cnt=0; l_u8Cnt<4; l_u8Cnt++)
	{
		m_au8InputBuffer1[l_u8Cnt] = *(m_au8SN+l_u8Cnt);	  
	}
	for(l_u8Cnt =4; l_u8Cnt<48; l_u8Cnt++)
	{
		m_au8InputBuffer1[l_u8Cnt] = *(m_au8MK+(l_u8Cnt%4));
	}
	for(l_u8Cnt=48; l_u8Cnt<52; l_u8Cnt++)		
	{
		m_au8InputBuffer1[l_u8Cnt] = *(m_au8SN+l_u8Cnt-44);	
	}
	for(l_u8Cnt =52; l_u8Cnt<55; l_u8Cnt++)
	{
		m_au8InputBuffer1[l_u8Cnt] = *(m_au8MK+(l_u8Cnt%4)+4);
	}
	m_au8InputBuffer1[55] = 0x80;
	for(l_u8Cnt=56; l_u8Cnt<62;l_u8Cnt++)
	{
		m_au8InputBuffer1[l_u8Cnt] = 0x0;
	}
	m_au8InputBuffer1[62]=0x01;
	m_au8InputBuffer1[63]=0xB8;
	
	//计算得出密钥	
	ComputeSHAEE(m_au8InputBuffer1, &A, &B, &C, &D, &E);

	l_au8Temp	= m_au8CSK;

	*(l_au8Temp++)	= (E)&0xff;
	*(l_au8Temp++)	= (E>>8)&0xff;
	*(l_au8Temp++)	= (E>>16)&0xff;
	*(l_au8Temp++)	= (E>>24)&0xff;
	*(l_au8Temp++)	= (D)&0xff;
	*(l_au8Temp++)	= (D>>8)&0xff;
	*(l_au8Temp++)	= (D>>16)&0xff;
	*(l_au8Temp++)	= (D>>24)&0xff;

	l_au8Temp	= 0;

	return	0;
}

/*********************************************************************************************************
** Function name:		WriteCSK
** Descriptions:		写CSK
**
** input parameters:	p_au8CSK
** output parameters:	None
** Returned value:		操作结果：	1(>0)，	失败
**									0，		成功
** Created by:			ZHANG Ye
** Created Date:		20110413
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
uint8 WriteCSK(uint8 * p_au8CSK)
{
#if	MACOUTPUT == 0				//不允许直接读MAC OUTput buffer
	I2C1_WriteNByte(DS2460_ADDR_W, 1, 0x60, p_au8CSK, 8);   //E_SECRET1 HIDE=YES，MAC输出不可读
#else							//允许直接读MAC OUTput buffer
	I2C1_WriteNByte(DS2460_ADDR_W, 1, 0x68, p_au8CSK, 8);   //E_SECRET2 HIDE=NO，MAC输出可读
#endif

	OSTimeDly(1);	//10ms
	return	0;
}

#ifdef	PUBMODE

/*********************************************************************************************************
** Function name:		BurnCSK
** Descriptions:		烧写CSK
**
** input parameters:	None
** output parameters:	None
** Returned value:		操作结果：	1(>0)，	失败
**									0，		成功
** Created by:			ZHANG Ye
** Created Date:		20110413
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
uint8 BurnCSK(void)
{
	//获取SN,MK
	ReadSN(m_au8SN);
	getMK();
	OSTimeDly(1);	//10ms

	//生成CSK
	generateCSK();
	OSTimeDly(1);	//10ms

	return	WriteCSK(m_au8CSK);
}

#endif

/*********************************************************************************************************
** Function name:		ReadSN
** Descriptions:		读取设备SN
**
** input parameters:	None
** output parameters:	p_au8SN
** Returned value:		操作结果：	1(>0)，	失败
**									0，		成功
** Created by:			ZHANG Ye
** Created Date:		20110413
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
uint8 ReadSN(uint8 * p_au8SN)
{
	I2C1_ReadNByte(DS2460_ADDR_R, 1, 0xF0, p_au8SN, 8);
	Delay(10);		//延时10ms
	return	0;
}

#if (defined USEMODE) || (defined UPDATEMODE)

/*********************************************************************************************************
** Function name:		ReadEEPROM
** Descriptions:		读取EEPROM所有112Byte数据
**
** input parameters:	None
** output parameters:	p_au8EEPROM
** Returned value:		操作结果：	1(>0)，	失败
**									0，		成功
** Created by:			ZHANG Ye
** Created Date:		20110413
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
ESAM_EXT	uint8 ReadEEPROM(uint8 * p_au8EEPROM)
{
	I2C1_ReadNByte(DS2460_ADDR_R, 1, 0x80, p_au8EEPROM, 112);
	Delay(10);		//延时10ms
	return	0;
}

#endif

#if (defined PUBMODE) || (defined UPDATEMODE)

/*********************************************************************************************************
** Function name:		WriteEEPROM
** Descriptions:		向EEPROM写入所有112Byte数据
**
** input parameters:	p_au8EEPROM
** output parameters:	None
** Returned value:		操作结果：	1(>0)，	失败
**									0，		成功
** Created by:			ZHANG Ye
** Created Date:		20110413
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
ESAM_EXT	uint8 WriteEEPROM(uint8 * p_au8EEPROM)
{
	I2C1_WriteNByte(DS2460_ADDR_W, 1, 0x80, p_au8EEPROM, 112);
	Delay(10);		//延时10ms
	return	0;
}

#endif
