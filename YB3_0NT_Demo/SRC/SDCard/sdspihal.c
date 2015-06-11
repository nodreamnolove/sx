/****************************************Copyright (c)**************************************************
**                               Guangzhou ZLG-MCU Development Co.,LTD.
**                                      graduate school
**                                 http://www.zlgmcu.com
**
**--------------File Info-------------------------------------------------------------------------------
** File name:			sdspihal.c
** Last modified Date:	2007-07-08
** Last Version:		V3.0
** Descriptions:		SD/MMC����д������: Ӳ������� -- SPIģʽ
**
**
**------------------------------------------------------------------------------------------------------
** Created by:			Ming Yuan Zheng
** Created date:		2005-1-6
** Version:				V1.0
** Descriptions:		The original version
**
**------------------------------------------------------------------------------------------------------
** Modified by:			Ming Yuan Zheng	
** Modified date:		2007-07-08
** Version:				V3.0
** Descriptions:		1. ��SPI_Clk400k()��SPI_ClkToMax()�������ϲ�Ϊһ������SdSpiHal_SetMCIClock()
**                      2. ������������  SdSpiHal_ ��Ϊ��ͷ
**						3. ���ӶԶ࿨��֧��
**------------------------------------------------------------------------------------------------------
** Modified by: 
** Modified date:
** Version:	
** Descriptions: 
**
********************************************************************************************************/


#include "sdconfig.h"

#if (!SDBUS_MODE_EN)

	/***************************************************************************
		
		��дSD/MMC����SPI�ӿں���: SPI ��ʼ��, SPIʱ������, ����/�����ֽں���	
	
	***************************************************************************/


/*******************************************************************************************************************
** ��������: SD_Power()				
**
** ��������: �Կ����µ�,���ϵ�	
**
** �䡡  ��: ��
**							  	
** �� �� ��: ��
**
** �� �� ֵ: ��
********************************************************************************************************************/
void SD_Power(void)
{
	INT32U i;

	SD_POWER_GPIO();
	SD_POWER_OUT();
	SD_POWER_OFF();								/* �ر� SD ����Դ  turn off power of sd card */
	
	SPI_SCK_GPIO();	
	SPI_SCK_OUT();
	SPI_SCK_CLR();								/* SCK  �����õ�   set SCK to zero */
	
	SPI_MISO_GPIO();
	SPI_MISO_OUT();
	SPI_MISO_CLR();								/* MISO �����õ�   set MISO to zero */
	
	SPI_MOSI_GPIO();		
	SPI_MOSI_OUT();
	SPI_MOSI_CLR();								/* MOSI �����õ�   set MOSI to zero */
	
	SPI_CS_GPIO();								
   	SPI_CS_OUT();								
	SPI_CS_CLR();								/* CS �����õ�	   set CS to zero */
		
	for(i = 0; i < 0x9000; i++);				/* �رյ�Դ��ʱ    delay after turn off power of sd card */
	SD_POWER_ON();								/* �� SD ����Դ  turn on power of sd card */
}


/*******************************************************************************************************************
** ��������: SdSpiHal_Initialize()				
**
** ��������: ��ʼ�����ʿ���Ӳ������		
**
** �䡡  ��: sd_struct *sds:  ����Ϣ�ṹ��, �ú�������ʹ��sds->card_posnum��Ա�������ֿ������
**							  	
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
**
** ��Ҫ����: 1. �ȸ����µ�, �ٸ����ϵ�;              2. ���÷��ʿ����ٶ�С�ڻ���� 400KHz;
** 
**           3. ��ʼ�����Ӳ���Ĵ���Ϊ�ʵ���״̬     4. ��ʼ��SPI������
********************************************************************************************************************/
INT8U SdSpiHal_Initialize(sd_struct *sds)
{ 
	SD_Power();									/* �Կ����µ�,���ϵ� */
	
	SPI_INIT();									/* ��ʼ��SPI�ӿ�     */	
	
	SD_INSERT_GPIO();
	SD_INSERT_IN();								/* ��⿨��ȫ�����Ϊ����� */
	
	SD_WP_GPIO();								
	SD_WP_IN();									/* д��������Ϊ����� */	
		   
   	SPI_CS_SET();								/* CS�ø� */

 	SdSpiHal_SetMCIClock(sds, SD_RATE_SLOW); 	/* ����SPIƵ��С�ڵ���400kHZ */
   
 	S0SPCR = (0x00 << 4) + (0x01 << 5);		    /* ����SPI�ӿ�ģʽ, MSTR = 1,CPOL = 1,CPHA = 0,LSBF=0 */

	return SD_NO_ERR;
}												


/*******************************************************************************************************************
** ��������: SdSpiHal_SetMCIClock()				
**
** ��������: ���ö�дSD/MMC��ʱ��		
**
** �䡡  ��: sd_struct *sds  :  ����Ϣ�ṹ��, �ú�������ʹ��sds->card_posnum��Ա�������ֿ������
**           INT32U ClockRate:	Ҫ���ʱ���ٶ�.  ȡֵSD_RATE_SLOW   ʱ, ���÷��ʿ��ٶ�С�� 400KHz		
**			  	                                 ȡֵSD_RATE_NORMAL ʱ, ���÷��ʿ��ٶ�Ϊ��������ٶȻ�������������ٶ�
** �� �� ��: ��
**
** �� �� ֵ: 0:   ��ȷ    >0:   ������, �� sddriver.h �ļ�
**
** ע    ��: ��������������ClockRateȡ��ͬ��ֵʱ, �������Ӧ�� sds->clkval ֵ, �������0. 
**
********************************************************************************************************************/
void SdSpiHal_SetMCIClock(sd_struct *sds, INT32U ClockRate)
{
   	if (ClockRate == SD_RATE_SLOW)
	{
		S0SPCCR = 128;    							/* ����SPIʱ�ӷ�ƵֵΪ128 */
		sds->host_clkval = Fpclk / 128;
	}
	else if (ClockRate == SD_RATE_NORMAL) 
	{
		S0SPCCR = 8;								/* ����SPIʱ�ӷ�ƵֵΪ8 */
		sds->host_clkval = Fpclk / 8;    				
	}
}


/*******************************************************************************************************************
** ��������: SdSpiHal_SendByte()				
**
** ��������: ͨ��SPI����������һ���ֽڵ���		
**
** �䡡  ��: sd_struct *sds  : ����Ϣ�ṹ��, �ú�������ʹ��sds->card_posnum��Ա�������ֿ������
**			 INT8U byte      : ���͵��ֽ�		
**           		
** �� �� ��: ��
**
** �� �� ֵ: ��
********************************************************************************************************************/
void SdSpiHal_SendByte(sd_struct *sds, INT8U byte)
{
	INT8U temp;

	S0SPDR = byte;							/* �������ݷ���SPI���ݼĴ��� */
   
	while(0 == (S0SPSR & 0x80));				/* �ȴ�SPIF��λ�����ȴ����ݷ������ */
												/* wait for SPIF being set, that is, wait for finishing of data being send */
 	temp = S0SPDR;
}


/*******************************************************************************************************************
** ��������: SdSpiHal_RecByte()				
**
** ��������: ͨ��SPI�������ӿ�����һ���ֽ�		
**
** �䡡  ��: sd_struct *sds  : ����Ϣ�ṹ��, �ú�������ʹ��sds->card_posnum��Ա�������ֿ������
**           		
** �� �� ��: ��
**
** �� �� ֵ: �յ����ֽ�
********************************************************************************************************************/
INT8U SdSpiHal_RecByte(sd_struct *sds)
{
	S0SPDR = 0xFF;
   
 	while(0 == (S0SPSR & 0x80));				/* �ȴ�SPIF��λ�����ȴ��յ����� */
												/* wait for SPIF being set, that is, wait for being received data */
	return(S0SPDR); 							/* ��ȡ�յ����ֽ� read the byte received */
}


/*******************************************************************************************************************
** ��������: SdSpiHal_CSAssert()				
**
** ��������: ͨ��SPI��CS����ѡ��		
**
** �䡡  ��: sd_struct *sds  : ����Ϣ�ṹ��, �ú�������ʹ��sds->card_posnum��Ա�������ֿ������
**           		
** �� �� ��: ��
**
** �� �� ֵ: ��
********************************************************************************************************************/
void SdSpiHal_CSAssert(sd_struct *sds)
{
	SPI_CS_CLR();			   					/* ƬѡSPI�ӻ�  select the SPI slave */  
}


/*******************************************************************************************************************
** ��������: SdSpiHal_CSDeassert()				
**
** ��������: ͨ��SPI��CS���Ų�ѡ��		
**
** �䡡  ��: sd_struct *sds  : ����Ϣ�ṹ��, �ú�������ʹ��sds->card_posnum��Ա�������ֿ������
**           		
** �� �� ��: ��
**
** �� �� ֵ: ��
********************************************************************************************************************/
void SdSpiHal_CSDeassert(sd_struct *sds)
{
	SPI_CS_SET();			    				/* ��ƬѡSPI�ӻ�  not select the SPI slave */
}


/*******************************************************************************************************************
** ��������: SdHal_CheckCard()				
**
** ��������: ��⿨�Ƿ���ȫ���뿨����
**
** �䡡  ��: sd_struct *sds:  ����Ϣ�ṹ��, �ú�������ʹ��sds->card_posnum��Ա�������ֿ������
**	         
** �� �� ��: ��
**
** �� �� ֵ: 1: ����ȫ����	   0: ��û����ȫ����
********************************************************************************************************************/
INT8U SdHal_CheckCard(sd_struct *sds)
{
	if (SD_INSERT_STATUS() != 0)
		return 0;								/* δ��ȫ���� not insert entirely */
	else
		return 1;								/* ��ȫ���� insert entirely */
}


/*******************************************************************************************************************
** ��������: SdHal_CheckCardWP()				
**
** ��������: ��⿨д����
**
** �䡡  ��: sd_struct *sds:  ����Ϣ�ṹ��, �ú�������ʹ��sds->card_posnum��Ա�������ֿ������
**	         
** �� �� ��: ��
**
** �� �� ֵ: 1: ����д����	  0: ��δд����
********************************************************************************************************************/
INT8U SdHal_CheckCardWP(sd_struct *sds)
{
	if (SD_WP_STATUS() != 0)
		return 1;								/* ��д���� */
	else
		return 0;								/* ��δд���� */
}

#endif


