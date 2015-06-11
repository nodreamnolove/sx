/*
*********************************************************************************************************
** �ļ����� �� I2C1.h
** ����˵�� �� LPC2200ϵ��оƬӲ��I2C��������
** ʹ��˵�� �� ������Ҫ���ú�I2C���߽ӿ�(I2C���Ź��ܺ�I2C�жϣ�����ʹ��I2C��ģʽ)
*********************************************************************************************************
*/

#ifndef  __I2C1_H
#define  __I2C1_H

#include "config.h"

#ifdef __I2C1_C
#define I2C1_EXT
#else
#define I2C1_EXT  extern
#endif

I2C1_EXT	uint8	I2C1Init(uint8 i2c_clk);
I2C1_EXT	uint8	I2C1_ReadNByte (uint16 sla, uint8 suba_type, uint32 suba, uint8 *tp, uint32 num);
I2C1_EXT	uint8	I2C1_WriteNByte(uint16 sla, uint8 suba_type, uint32 suba, uint8 *tp, uint32 num);


#endif