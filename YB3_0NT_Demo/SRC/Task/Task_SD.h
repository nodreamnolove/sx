#ifndef __TASK_SD_H__
#define __TASK_SD_H__



#include "Common.h"
#include "W5100App.h"
#include "JZGlobal.h"

#define  SD_SINGLE_START0	 	0x002000
//#define  SD_SINGLE_START1	 	0x281800
//#define  SD_SINGLE_START2	 	0x501000
//#define  SD_SINGLE_START3	 	0x780800
#define  SD_SINGLE_BLOCK_NUM  	0x27F800
#define  SD_STAT_START		  	0xA00000		 //5G
#define	 SD_STAT_BLOCK_NUM		0x400000		 //2G

#define  SD_STAT_ADD_START      0x900000    //???????????????	//4G地址处
#define  SD_STAT_ADD_NUM        0x80000   //???????????????		//256M

/*****************************UART1出单车SD卡相关宏定义*****************************************/
#define SD_STAT1_ADD_START	0xd00000						//一天的头一辆车的起始地址

#define SD_CONTINUE_ADD_START 0xd80000	
#define SD_CONTINUE_NUM		   50000


/*****************************UART1出单车SD卡相关宏定义*****************************************/
		

typedef struct
{
	uint8 year;	    //?ê′ó2013?ê?aê?
	uint8 month;
	uint8 day;
	uint8 hour;
	uint8 minute;
	uint8 second;
}V_TIME;

typedef struct 
{
	uint16 year;
	uint8 month;
	uint8 day;
	uint8 hour;
	uint8 minute;
	uint8 second;
}V16_TIME;
typedef struct
{
	V_TIME time_info;
	uint32 start_num;
	uint32 end_num;
}STAT_SEND_INFO;

typedef struct
{
	V_TIME s_time;
	V_TIME e_time;
}SV_SEND_INFO;
//extern uint32 test_MCIerr_count;


//extern uint32 gv_index;

extern uint8  SD_BLOCK_HEAD[4];
extern uint32 g_u32SD_err_count;
extern uint32 g_u32SD_write_count;

extern STAT_SEND_INFO stat_send_fifo[100];
extern int stat_send_fifohead;
extern int stat_send_fifoend;

extern SV_SEND_INFO sv_send_fifo[100];
extern int sv_send_fifohead;
extern int sv_send_fifotail;


extern OS_EVENT *FW_flag;
extern OS_EVENT *SD_flag;



extern uint32 gv_index;
extern uint8 frame11_buf[57];
//
extern V_TIME sv_time_lock;


//extern uint8 RD_num_buf[16];
//extern uint8 flag_03change;
//extern uint8 DSC_ip_buf[6];
//extern uint8 flag_04change;
//extern uint8 InvContents_buf[1];
//extern uint8 flag_06change;
//extern uint8 ProCycle_buf[1];
//extern uint8 flag_07change;
//extern uint8 DisTime_buf[1];
//extern uint8 flag_0Bchange;


uint8 full_read_sd(uint32 sd_add, uint8 *buf);
uint8 full_write_sd(uint32 sd_add, uint8 *buf);
void save01_to_sd(uint8 * buf,int32 len);
uint32 save11_to_sd(uint8 *buf,int32 len);	//by mjh 20140616
void add_to_net_send_stat(uint16 year,uint8 month,uint8 day,uint32 start_num,uint32 end_num);
void Task_Net_Send(void *tdata);
//void save_v_tosd(uint8 * buf);
void SD_write_v(uint8 *c_v_buf,uint32 l_min_indexfrom2013);	
void sv_read_sd(uint8 *sv_tbuf,uint32 temp_minute_indexfrom2013);
uint8 get_lane(uint8 olane); //3μμàó3é?
void gen_sv_fram(uint8 *buf);
//void gen2_sv_fram(uint8 *buf,uint8 num);

uint8 Read256_full(uint16 p_u16Addr, uint8 * p_pu8WriteBuf, uint16 p_u16Len);
uint8 Write256_full(uint16 p_u16Addr, uint8 * p_pu8WriteBuf, uint16 p_u16Len);

//uint32 get_min_index(V_TIME *ptime);
//
//void savefwParam(uint8 type,uint8 * buf);
//void Set_para();

#endif
