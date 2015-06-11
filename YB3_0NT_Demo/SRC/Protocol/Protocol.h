#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__
#include "config.h"
#include "Common.h"
#include "Uart5.h"
#include "TDC256.h"
#include "WDT.h"


extern uint8 RecComData(uint8 *pUartDataBuf, uint8 *plen);
extern void AnalyzeComData(uint8 *pUartDataBuf, uint8 *plen);
#endif
