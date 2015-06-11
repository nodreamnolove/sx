#ifndef  __DS2460_H
#define  __DS2460_H

#include "Config.h"

#ifdef __DS2460_C
#define ESAM_EXT
#else
#define ESAM_EXT  extern
#endif

/*********************************************************************************************************
** 宏定义说明：
**		在系统设置中定义，PUBMOD为发行模式，USEMODE为正常程序模式，UPDATEMODE为刷程序模式
**		默认为USEMODE	
*********************************************************************************************************/

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
*********************************************************************************************************/
ESAM_EXT	uint8 CheckESAM(void);

#endif

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
*********************************************************************************************************/
ESAM_EXT	uint8 BurnCSK(void);

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
*********************************************************************************************************/
ESAM_EXT	uint8 ReadSN(uint8 * p_au8SN);

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
*********************************************************************************************************/
ESAM_EXT	uint8 ReadEEPROM(uint8 * p_au8EEPROM);

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
*********************************************************************************************************/
ESAM_EXT	uint8 WriteEEPROM(uint8 * p_au8EEPROM);

#endif

#endif		//__DS2460_H
