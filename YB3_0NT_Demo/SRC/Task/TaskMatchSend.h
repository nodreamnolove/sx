#ifndef __SV_CONTINUE__
#define __SV_CONTINUE__

#include "config.h"
#include "WT_Task.h"
#include "Task_SD.h"
#include "TDC256.h"
#include "rd_data.h"
////////////////////////////

typedef struct _Cycle_Que
{
	uint32 head;
	uint32 tail;
	uint32 Que_Cnt;
}_Cycle_Que_Continue;

extern uint32 Get_Que_Cycle(uint8 type);
//extern uint8 Set_Que_Cycle(uint32 data,uint8 type);
extern uint8 Get_Que_Cycle_Continue(uint32*p_add_conti,uint32 *index);
//extern uint8 Set_Que_Cycle_Continue(uint32 save_add,uint32 index);
extern uint8 SD_SvDataAdd_Read(uint8 *data,uint32*p_read_add,uint32 *index);

#endif
