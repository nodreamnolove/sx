#include "config.h"
extern uint8 Rcv_Buf[2048];
extern void UART1_SendBuf (uint8  *RcvBufPt,  uint32  Snd_Len);
extern uint8 UART1_SendBuf_full (uint8  *RcvBufPt,  uint32  Snd_Len);		//独占串口方式发送数据  2014-6-3 mjh
extern void UART1_Init (void);

extern OS_EVENT *UART1_flag;
