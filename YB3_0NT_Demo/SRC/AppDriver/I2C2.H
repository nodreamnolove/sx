/*
*********************************************************************************************************
** 文件名称 ： I2C2.h
** 功能说明 ： LPC2200系列芯片硬件I2C软件包。
** 使用说明 ： 主程序要配置好I2C总线接口(I2C引脚功能和I2C中断，并已使能I2C主模式)
*********************************************************************************************************
*/

#ifndef  __I2C2_H
#define  __I2C2_H

#include "config.h"

#ifdef __I2C2_C
#define I2C2_EXT
#else
#define I2C2_EXT  extern
#endif

I2C2_EXT	uint8	I2C2Init(uint8 i2c_clk);
I2C2_EXT	uint8	I2C2_ReadNByte (uint16 sla, uint8 suba_type, uint32 suba, uint8 *tp, uint32 num);
I2C2_EXT	uint8	I2C2_WriteNByte(uint16 sla, uint8 suba_type, uint32 suba, uint8 *tp, uint32 num);

#endif


