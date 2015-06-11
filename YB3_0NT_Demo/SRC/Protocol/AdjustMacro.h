/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			AdjustMacro.h
** Last modified Date:  20110531
** Last Version:		1.0
** Descriptions:		调校协议命令宏定义
**
**--------------------------------------------------------------------------------------------------------
** Created by:			ZHANG Ye
** Created date:		20110531
** Version:				1.0
** Descriptions:		调校协议命令宏定义
**
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
** Version:
** Descriptions:
**
*********************************************************************************************************/
#ifndef	__ADJUSTMACRO_H
#define	__ADJUSTMACRO_H

//命令帧
//	0xFF 00 [LEN] [COMM] [PARAM1] [PARAM2] [PARAM3] [VALUE1] [VALUE2] [VALUE3] [VALUE4] [CRC1] [CRC2]

//命令宏定义	COMM
#define		ADJ_SAVE				0xA1
#define		ADJ_GETALLPARAM			0xA2
#define		ADJ_MODIFYPARAM			0xA3	//暂时取消该命令
#define		ADJ_GETNONEWEIGHT		0xA4		//读取所有非重量修正参数
#define		ADJ_UPDATENONEWEIGHT	0xA5		//更新所有非重量修正参数
  
#define		ADJ_GETSTATICWEIGHT		0xA6		//读取所有静态修正参数
#define		ADJ_UPDATESTATICWEIGHT	0xA7		//更新所有静态修正参数

#define		ADJ_GETMOTIONWEIGHT		0xA8		//读取所有动态修正参数
#define		ADJ_UPDATEMOTIONWEIGHT	0xA9		//更新所有动态修正参数

#define		ADJ_UPDATEALL		0xA0		//更新所有SETUP

#define		ADJ_READVEHNUM		0x04		//读缓存车数
#define		ADJ_DEVICESTATUS	0x05		//设备状态
#define		ADJ_DELETEFIRST		0x03		//删首车
#define		ADJ_DELETELAST		0x06		//删尾车	
#define		ADJ_DELETENO		0x12		//删除指定车辆
#define		ADJ_SYNCHRONY		0x07		//同步
#define		ADJ_RESEND			0x0A		//重发
#define		ADJ_CAPTURE			0x13		//抓拍
#define		ADJ_ManualShouwei	0x14		//强制收尾
#define		ADJ_LOOPTRIGGER		0x21		//线圈触发

#define		ADJ_TIME			0x3A		//系统校时
#define		ADJ_INITWITHCACHE	0x55		//保留缓存初始化	
#define		ADJ_INITNOCACHE		0x54		//不保留缓存初始化

#define		ADJ_SENDAXLE		0x01		//发送轴数据
#define		ADJ_SENDAXLEGRP		0x00		//发送轴组数据

#define		ADJ_SLAVESTART		0x09		//从机上电	 
#define		ADJ_QINGLING		0x32		//清零

//修改参数时，参数1的参数类型

#define		PRM_ZERO			0x01		//零点
#define		PRM_GAIN			0x02		//增益
#define		PRM_STATICMODIFY	0x03		//静态修正
#define		PRM_SPEEDMODIFY		0x04		//速度修正
	
#define		PRM_PWD				0x05		//密码
#define		PRM_FULL			0x06		//量程
#define		PRM_PODU			0x07		//坡度 
#define		PRM_STATICSCALE		0x08		//静态分度
#define		PRM_MOTIONSCALE		0x09		//动态分度
#define		PRM_GENZONG			0x0A		//零点跟踪
#define		PRM_COMMODE			0x0B		//命令模式
#define		PRM_BAUDRATE		0x0C		//波特率
#define		PRM_DOG				0x0D		//看门狗	

//一般不修改时间
#define		PRM_YEAR			0x0E		//修改时间，年
#define		PRM_MONTH			0x0F		//修改时间，月
#define		PRM_DAY				0x10		//修改时间，日

#define		PRM_PLATWIDTH		0x11		//台面宽度
#define		PRM_FANGXIANG		0x12		//方向使能
#define		PRM_DIAODIAN		0x13		//掉电保护标识	
#define		PRM_CAPTURE			0x14		//抓拍使能
#define		PRM_LOOP			0x15		//线圈触发使能

#define		PRM_VEHTOTALM		0x16		//车型整体修正
#define		PRM_VEHSPEEDM		0x17		//车型速度修正
#define		PRM_PROTOCOL		0x18		//协议
													   
#define		PRM_BUFSIZE			0x19		//缓存车数
#define		PRM_LUNZHOUERR		0x1A		//轮轴报错使能
#define		PRM_LUNZHOUPROG		0x1B		//无轮轴程序使能 
#define		PRM_SENDWAVE		0x1C		//发波形使能


#endif	//__ADJUSTMACRO_H
