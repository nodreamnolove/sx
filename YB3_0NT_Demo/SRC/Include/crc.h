#ifndef _CRC_H
#define _CRC_H

#include "config.h"

#define CRC_OK      0
#define CRC_ERROR   1

extern uint16 crc_16(uint8 *ptr, uint16 len);

extern void crc_create(uint8 *ptr, uint16 len);
extern uint8 crc_check(uint8 *ptr, uint16 len);
extern unsigned short CRC16(unsigned char *puchMsg, unsigned short usDataLen);

#endif
