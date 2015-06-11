/****************************************Copyright (c)****************************************************
**                         Guangzhou ZHIYUAN electronics Co.,LTD.                               
**                                     
**                               http://www.embedtools.com
**
**--------------File Info---------------------------------------------------------------------------------
** File name:           PCF8563.c
** Last modified Date:  20110413
** Last Version:        1.00
** Descriptions:        for lpc3200
**--------------------------------------------------------------------------------------------------------
** Created by:          ZHANG Ye
** Created date:        20110413
** Version:             1.00
** Descriptions:        The original version
*********************************************************************************************************/
#define	__PCF8563_C
#include "PCF8563.h"

#define		PCF8563ADRW		0xA2	//写
#define		PCF8563ADRR		0xA3	//读

#define		BCD_to_HEC(b)	(((b>>4)*10)+(b&0x0f))
#define		HEC_to_BCD(h)	(((h/10)<<4)|(h%10))

#define		StopRTC()		RTC_CTRL	|= 0x40
#define		StartRTC()		RTC_CTRL	&= 0xbf


uint16 m_u16TotalDays[12]		= {0,31,59,90,120,151,181,212,243,273,304,334};		/* 非闰年月份日期表				*/
uint16 m_u16TotalDaysLeap[12]	= {0,31,60,91,121,152,182,213,244,274,305,335};		/* 闰年月份日期表				*/

uint16 m_u16MonthDays[12]		= {31,28,31,30,31,30,31,31,30,31,30,31};			/* 非闰年月份日期表				*/
uint16 m_u16MonthDaysLeap[12]	= {31,29,31,30,31,30,31,31,30,31,30,31};			/* 闰年月份日期表				*/

static	uint8	SetRTC(RTCDateTime * p_RdtTime);
static	uint8	GET8563(RTCDateTime * p_RdtTime);
static	uint8	SET8563(RTCDateTime * p_RdtTime);

/*********************************************************************************************************
** Function name:		RTC8563Init
** Descriptions:		RTC与8563初始化
** input parameters:	none 
** output parameters:	none
**
** Created by:			ZHANG Ye		  
** Created Date:		20110503	  
**-------------------------------------------------------------------------------------------------------
** Modified by:			ZHANG Ye	
** Modified date:		20110518	
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void RTC8563Init(void)
{
	uint8 temp;

    StopRTC();
	//启动8563
    temp = 0;
    I2C2_WriteNByte(PCF8563ADRW, 1, 0x00, &temp, 1);	

	//启动RTC
	RTC_CTRL   = 0x10;					/* 复位计数器   硬件复位        */
    RTC_CTRL   = 0x00;					/* 停止计数器一直复位           */
	
	Synchro8563();						//同步
	StartRTC();
}

/********************************************************************************************************
* 名    称：SetRTC
* 功    能：设置RTC
* 入口参数：p_RdtTime    指向时间结构体的指针
* 出口参数：FALSE or TRUE
*********************************************************************************************************/
uint8 SetRTC(RTCDateTime * p_RdtTime)
{
	uint32	l_u32SecCnt;
	uint32	l_u32Temp;
	RTCDateTime	l_rdtTemp;

	l_u32SecCnt	= 0;

	l_rdtTemp.u16Year		= p_RdtTime -> u16Year;  
	l_rdtTemp.u8Month		= p_RdtTime -> u8Month;
	l_rdtTemp.u8Day			= p_RdtTime -> u8Day;
	l_rdtTemp.u8Week		= p_RdtTime -> u8Week;
	l_rdtTemp.u8Hour		= p_RdtTime -> u8Hour;
	l_rdtTemp.u8Minute		= p_RdtTime -> u8Minute;
	l_rdtTemp.u8Second		= p_RdtTime -> u8Second;    

	StopRTC();                     		/* 设置时间时先关闭计数器       */
    
	if (l_rdtTemp.u16Year > 2099) 						/* 设置年不能超过99年，			*/
	{											    
		l_rdtTemp.u16Year =  2099;						/* 否则取99年					*/
    }
	else if (l_rdtTemp.u16Year < 2000)
	{
		l_rdtTemp.u16Year =  2000;
	}
    
    l_u32Temp 		=  l_rdtTemp.u16Year - 2000;
    l_u32SecCnt		=  (l_u32Temp * 365 + (l_u32Temp / 4));   		/* 通过年份计算天数             */
    if (l_u32Temp % 4) 												/* 非闰年的时候补偿天数         */
	{
        l_u32SecCnt += 1;
    }                                          						

    if (l_rdtTemp.u8Month > 12) 										/* 设置月不能超过12月,否则取12	*/
	{										
        l_rdtTemp.u8Month	= 12;
    }
   
    if (l_u32Temp % 4) 													/* 非闰年，月份天数				*/
	{                                                        
        l_u32Temp	=  (uint8)(l_rdtTemp.u8Month - 1);
        l_u32SecCnt += (uint16)m_u16TotalDays[l_u32Temp]; 
		
		l_u32Temp	= (uint16)m_u16MonthDays[(uint8)(l_rdtTemp.u8Month - 1)];
		if(l_rdtTemp.u8Day < l_u32Temp)
		{                                                        
        	l_u32Temp	= l_rdtTemp.u8Day;
		}
    }
    else 																/* 闰年，月份天数				*/
	{                                                              
        l_u32Temp	=  (uint8)(l_rdtTemp.u8Month - 1);
        l_u32SecCnt += (uint16)m_u16TotalDaysLeap[l_u32Temp];

		l_u32Temp	= (uint16)m_u16MonthDaysLeap[(uint8)(l_rdtTemp.u8Month - 1)];
		if(l_rdtTemp.u8Day < l_u32Temp)
		{                                                        
        	l_u32Temp	= l_rdtTemp.u8Day;
		}
    }

    l_u32Temp	= (uint8)(l_u32Temp - 1);
    l_u32SecCnt += l_u32Temp;               							/* 以上得距离2000-01- 01的天数  */
    l_u32SecCnt	*= 86400;                    							/* 一天的秒数为86400秒          */

    if (l_rdtTemp.u8Hour > 23) 											/* 设置小时不能超过23小时，		*/
	{																	/* 否则23小时					*/
        l_rdtTemp.u8Hour	= 23;											
    }
    
    l_u32Temp	= l_rdtTemp.u8Hour;
    l_u32SecCnt	+= l_u32Temp * 3600;                     				/* 一小时3600秒                 */
    
    if (l_rdtTemp.u8Minute > 59) 											/* 设置分钟不能超过59分，		*/
	{																	/* 否则取59分					*/
        l_rdtTemp.u8Minute	=  59;												
    }
    
    l_u32Temp	= l_rdtTemp.u8Minute;
    l_u32SecCnt += l_u32Temp * 60;   									/* 一分钟60秒                   */

    if (l_rdtTemp.u8Second > 59) 										/* 设置秒不能超过59秒，			*/
	{																	/* 否则取59秒					*/
        l_rdtTemp.u8Second	= 59;												
    }
    
    l_u32Temp	= l_rdtTemp.u8Second;
    l_u32SecCnt += l_u32Temp;

	
	RTC_UCOUNT	= l_u32SecCnt;                       				/* 将计算得到的值赋给计数器     */
    RTC_DCOUNT	= (uint32)(0xffffffff - l_u32SecCnt);     			/* 递增计数器和递减计数器包含   */
    
    StartRTC();
    return	TRUE;
}

/********************************************************************************************************
* 名    称：SET8563
* 功    能：设置PCF8563
* 入口参数：p_RdtTime    指向时间结构体的指针
* 出口参数：FALSE or TRUE
*********************************************************************************************************/
uint8 SET8563(RTCDateTime * p_RdtTime)
{
	uint8 temp[7];
	uint8 u16Year;
    
	temp[0] = HEC_to_BCD(p_RdtTime->u8Second);
	temp[1] = HEC_to_BCD(p_RdtTime->u8Minute);
	temp[2] = HEC_to_BCD(p_RdtTime->u8Hour);
	temp[3] = HEC_to_BCD(p_RdtTime->u8Day);
	temp[4] = HEC_to_BCD(p_RdtTime->u8Week);
	temp[5] = HEC_to_BCD(p_RdtTime->u8Month);
    
	if (p_RdtTime->u16Year >= 2000){
		temp[5] |= 0x80;
		u16Year = p_RdtTime->u16Year - 2000;
		temp[6] = HEC_to_BCD(u16Year);
	}else{
		u16Year = p_RdtTime->u16Year - 1900;
		temp[6] = HEC_to_BCD(u16Year);
	}
	return	I2C2_WriteNByte(PCF8563ADRW, 1, 0x02, temp, 7);
}

/********************************************************************************************************
* 名    称：GET8563
* 功    能：读取PCF8563
* 入口参数：p_RdtTime
* 出口参数：FALSE or TRUE
*********************************************************************************************************/
uint8 GET8563(RTCDateTime * p_RdtTime)
{
	uint8 temp[7];
    
    if (I2C2_ReadNByte(PCF8563ADRR, 1, 0x02, temp, 7)==FALSE) {
        return FALSE;
    }
    
    p_RdtTime -> u8Second	= BCD_to_HEC((temp[0]&0x7f));
    p_RdtTime -> u8Minute	= BCD_to_HEC((temp[1]&0x7f));
    p_RdtTime -> u8Hour		= BCD_to_HEC((temp[2]&0x3f));
    p_RdtTime -> u8Day		= BCD_to_HEC((temp[3]&0x3f));
    p_RdtTime -> u8Week		= BCD_to_HEC((temp[4]&0x07));
    
    if (temp[5]&0x80){
        p_RdtTime -> u8Month	= BCD_to_HEC((temp[5]&0x1f));
        p_RdtTime -> u16Year	= BCD_to_HEC(temp[6])+2000;
    }else{
        p_RdtTime -> u8Month	= BCD_to_HEC((temp[5]&0x1f));
        p_RdtTime -> u16Year	= BCD_to_HEC(temp[6])+1900;
    }
    return TRUE;
}

/********************************************************************************************************
* 名    称：Synchro8563
* 功    能：从8563中读取时间，并设置RTC
* 入口参数：none
* 出口参数：FALSE or TRUE
*********************************************************************************************************/
void Synchro8563(void)
{
	RTCDateTime		l_rdtTemp;

	GET8563(&l_rdtTemp);		//读取8563

	SetRTC(&l_rdtTemp);				//将读出的时间保持到RTC
}

/********************************************************************************************************
* 名    称：SetRTCTime
* 功    能：设置RTC时间
* 入口参数：p_pstTime
* 出口参数：无
*********************************************************************************************************/
void SetRTCTime(SystemTime * p_pstTime)
{ 
	SET8563(p_pstTime);		//设置8563
	SetRTC(p_pstTime);		//设置RTC
}


/********************************************************************************************************
* 名    称：GetRTCTime
* 功    能：获取RTC时间
* 入口参数：p_pstTime
* 出口参数：无
*********************************************************************************************************/
void GetRTCTime(SystemTime * p_pstTime)
{
	uint32	l_u32SecCnt	= 0;
	uint32	l_u32Temp;
	uint16	l_u16Days;
	uint32	l_u32WeekTemp;

	uint32	i;
	l_u32WeekTemp	= 6;		//2000-01-01 星期六
    l_u32SecCnt		= (uint32)RTC_UCOUNT;
	
	l_u32WeekTemp	+= (l_u32SecCnt / 86400);
	p_pstTime -> u8Week	= l_u32WeekTemp % 7;

    l_u32Temp		= (uint8)(l_u32SecCnt / 126230400);         /* 4年，三非闰年加一闰年的总共  */
                                                      			/* 秒数为126230400秒            */
    l_u32Temp		= (uint8)(l_u32Temp * 4);
    l_u32SecCnt		= l_u32SecCnt % 126230400;

    if (l_u32SecCnt >= 31622400) 
	{
        l_u32Temp ++;                                 			/* 第一年闰年，秒数为31622400   */
        l_u32SecCnt	= l_u32SecCnt - 31622400;
        if (l_u32SecCnt >= 31536000) 
		{
            l_u32Temp ++;                            			/* 第二年非闰年，秒数为31536000 */
            l_u32SecCnt	= l_u32SecCnt - 31536000;
        } 
		if (l_u32SecCnt >= 31536000) 
		{
            l_u32Temp ++;                                   	/* 第三年非闰年，秒数为31536000 */
            l_u32SecCnt = l_u32SecCnt-31536000;
        }
    }

    p_pstTime->u16Year	= l_u32Temp + 2000;                    		/* 以上步骤计算得到当前年份     */

	l_u16Days		= l_u32SecCnt / 86400;
	l_u32SecCnt		= l_u32SecCnt % 86400;

    if (l_u32Temp % 4) 
	{
        for (i = 11; i > 0; i--) 
		{
            if (l_u16Days >= m_u16TotalDays[i]) 
			{
				l_u16Days	-= m_u16TotalDays[i];
                break;
            }
        }
    }
    else 
	{                                                   		/* 闰年，月份表为Table1         */
        for (i = 11; i > 0; i--) 
		{
            if (l_u16Days >= m_u16TotalDaysLeap[i]) 
			{
				l_u16Days	-= m_u16TotalDaysLeap[i];
                break;
            }
        }
    }
    i	+=1 ;
    p_pstTime->u8Month		= i;                             			/* 计算得到月份数               */

    i	= l_u16Days + 1;
    p_pstTime->u8Day   	= i;                                			/* 每天86400秒，取得天数        */

    i	= (uint8)(l_u32SecCnt / 3600);
    p_pstTime->u8Hour		= i;                                        	/* 每小时3600秒，取得小时数     */

    l_u32SecCnt		=  l_u32SecCnt % 3600;
    i	= (uint8)(l_u32SecCnt / 60);
    p_pstTime->u8Minute	= i;                                     		/* 取得分数                     */

    l_u32SecCnt		= l_u32SecCnt % 60;
    p_pstTime->u8Second	= (uint8)l_u32SecCnt;                       	/* 取得秒数                     */

}
