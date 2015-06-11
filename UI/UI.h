/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			UI.h
** Last modified Date:  20110505
** Last Version:		1.0
** Descriptions:		程序所有界面函数
**
**--------------------------------------------------------------------------------------------------------
** Created by:			ZHANG Ye
** Created date:		20110505
** Version:				1.0
** Descriptions:		The original version
**
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
** Version:
** Descriptions:
**
*********************************************************************************************************/
#ifndef	__UI_H
#define	__UI_H

#include "Config.h"

#ifdef	__UI_C
#define		UI_EXT
#include "Common.h"
#include "TDC256.h"

#include "KBMacro.h"
#include "UIOperation.h"
#include "LCDApp.h"

#if	YBVERSION >= 30		//3.0仪表功能				  
#include "StorageApp.h"
#else	//2.2
#include "ISP.h"
#endif	//#if	YBVERSION >= 30		//3.0仪表功能

#else		//__UI_C
#define		UI_EXT		extern
#endif

UI_EXT	void	SaveParams(void);			//保存参数设置

UI_EXT	void	UIStartUp(void);			//启动界面

UI_EXT	void	UIGeneral(void);			//通用过车界面
UI_EXT	void	UIValidate(void);			//刷程序后验证界面
UI_EXT	void	UISN(void);					//SN码界面

UI_EXT	void	UIBDRoot(void);				//标定根界面

UI_EXT	void	UIBDMain(uint8 p_u8Motion);	//动静态标定界面
//UI_EXT	void	UIBDStatic(void);			//静态标定界面
UI_EXT	void	UISystemInit(void);			//系统初始化界面

UI_EXT	void	UIBDWanBanChoose(uint8 p_u8Motion);				//弯板标定通道选择界面
UI_EXT	void	UIBDWanBan(uint8 p_u8CID, uint8 p_u8Motion);	//弯板标定界面
UI_EXT	void	UIBDChengTai(uint8 p_u8CID, uint8 p_u8Motion);	//称台标定界面	   
UI_EXT	void	UIBDChengTaiAll(uint8 p_u8Motion);				//称台标定界面:所有台板加和值，台板号选择0
UI_EXT	void	UIBDChengtaiChoose(uint8 p_u8Motion);			//秤台台板选择界面 
UI_EXT	void	UIBDScale(uint8 p_u8Motion);					//设置分度

UI_EXT	void	UIBDGenZong(void);			//零点跟踪使能界面
UI_EXT	void	UIBDPoDu(void);				//坡度修正界面
UI_EXT	void	UIBDLunZhou(void);			//无轮轴程序使能界面
#if	SENDWAVEENABLE > 0		//使能发波形
UI_EXT	void	UIBDSendWave(void);			//发送波形使能界面
#endif
UI_EXT	void	UIBDFullRange(void);		//设置最大量程 
UI_EXT	void	UIBDStaticModify(void);		//设置静态修正界面，按照传感器重量修正
UI_EXT	void	UIBDChooseMotion(void);		//动态修正选择界面，点修or线修
UI_EXT	void	UIBDLineModify(void);		//线性修正界面
UI_EXT	void	UIBDPointModify(void);		//点修正界面
UI_EXT	void	UIBDChooseVehPos(void);		//车型速度修正位置选择
//UI_EXT	void	UIBDVehSpeedModifyAB(void);	//车型速度修正AB板
//UI_EXT	void	UIBDVehSpeedModifyBC(void);	//车型速度修正BC板
//UI_EXT	void	UIBDVehSpeedModifyGap(void);//车型速度修正压缝 
//UI_EXT	void	UIBDVehModify(void);		//车型整体修正
//UI_EXT	void	UIBDVehGap(void);			//车型整体压缝修正
UI_EXT	void	UIBDChooseVeh(uint8 p_u8Pos);	//选择车型界面，根据压缝位置不同，查看的数据不同
UI_EXT	void	UIBDVSModifyParam(uint8 p_u8Pos, uint8 p_u8Veh);	//参数值界面，根据压缝位置和车型


UI_EXT	void	UICommonSet(void);			//普通设置界面，111
UI_EXT	void	UIViewSetting(void);		//查看设置参数界面，222
UI_EXT	void	UIViewModify(void);			//查看速度修正界面，333
UI_EXT	void	UIViewAuthor(void);			//查看作者信息界面，888
UI_EXT	void	UIViewThreshold(void);		//阈值参数，8968
#if	YBVERSION >= 30		//3.0仪表功能
UI_EXT	void	UIViewIPInfo(void);			//查看IP信息，444
UI_EXT	void	UIViewStartUpTime(void);	//查看启动时间信息，8494
#endif													
		
UI_EXT	void	UIF3Code(void);				//查看F3代码界面
UI_EXT	void	UIF4Code(void);				//查看F4代码界面
UI_EXT	void	UIF5Code(void);				//查看F5代码界面

UI_EXT	void	UISetBaudRate(void);		//设置波特率界面
UI_EXT	void	UISetTime(void);			//设置时间界面
UI_EXT	void	UISetCommandMode(void);		//设置命令模式界面
UI_EXT	void	UISetForwardEnable(void);	//设置方向使能界面
UI_EXT	void	UISetProtocol(void);		//协议界面
UI_EXT	void	UISetLoop(void);			//线圈使能界面
//UI_EXT	void	UISetPassword(void);		//修改密码界面
UI_EXT	void	UISetCapture(void);			//设置抓拍使能界面
UI_EXT	void	UISetPlat(void);			//设置台面宽度界面
UI_EXT	void	UISetVehicleCache(void);	//设置车辆缓存
UI_EXT	void	UISetDiaodian(void);		//设置掉电保护界面
UI_EXT	void	UISetDog(void);				//设置看门狗界面
UI_EXT	void	UISetLunZhouEnable(void);	//设置轮轴故障显示使能界面
#if	YBVERSION >= 30		//3.0仪表功能
UI_EXT	void	UISetValueParamIP(char * p_pcName, uint8 * p_pu8IP);	//设置IP
#endif


//UI_EXT	void	UIBDStaticWanBan(uint8	p_u8CID);	//弯板静态标定界面，用于静态型批
//UI_EXT	void	UIBDStaticChengTai(uint8 p_u8CID);	//称台静态标定界面，用于静态型批  
//UI_EXT	void	UIBDStaticChengtaiChoose(void);
//UI_EXT	void	UIBDStaticScale(void);				//设置静态分度 
//UI_EXT	void	UIBDStaticWanBanChoose(void);		//弯板标定通道选择界面

#endif		//__UI_C
