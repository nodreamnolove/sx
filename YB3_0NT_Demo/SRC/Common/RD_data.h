#ifndef _RD_data_H
#define _RD_data_H
#include "config.h"

typedef struct _TRAFFIC_DATA_SEND01
{
	uint32 lane_no;						// ������
	uint32 che_num; 					// ����
	uint32 genche_num;					// ������
	uint32 genchebaifenbi;				// �����ٷֱ�
	uint32 chetouspace_total; 			// ��ͷ�ܼ��
	uint32 ave_space; 					// ƽ����ͷ���
	uint32 time_occupancy;				// ʱ��ռ����
	uint32 xiaohuo_num;					// С����ͨ��
	uint32 xiaohuo_avespeed;			// С��ƽ���ٶ�
	uint32 xiaohuo_speed_total;			// С�����ٶ�
	uint32 zhonghuo_num;				// �л���ͨ��
	uint32 zhonghuo_avespeed;			// �л�ƽ���ٶ�
	uint32 zhonghuo_speed_total;		// �л����ٶ�
	uint32 dahuo_num;					// �����ͨ��
	uint32 dahuo_avespeed;				// ���ƽ���ٶ�
	uint32 dahuo_speed_total;			// ������ٶ�
	uint32 zhongxiaoke_num;				// ��С�ͽ�ͨ��
	uint32 zhongxiaoke_avespeed;		// ��С��ƽ���ٶ�
	uint32 zhongxiaoke_speed_total;		// ��С�����ٶ�
	uint32 dake_num;					// ��ͽ�ͨ��
	uint32 dake_avespeed;				// ���ƽ���ٶ�
	uint32 dake_speed_total;			// ������ٶ�
	uint32 tuogua_num;					// �Ϲҳ���ͨ��
	uint32 tuogua_avespeed;				// �Ϲҳ�ƽ���ٶ�
	uint32 tuogua_speed_total;			// �Ϲҳ����ٶ�
	uint32 tuolaji_num;					// ��������ͨ��
	uint32 tuolaji_avespeed;			// ������ƽ���ٶ�
	uint32 tuolaji_speed_total;			// ���������ٶ�
	uint32 tedahuo_num;					// �ش����ͨ��
	uint32 tedahuo_avespeed;			// �ش��ƽ���ٶ�
	uint32 tedahuo_speed_total;			// �ش�����ٶ�
	uint32 moto_num;					// Ħ�г���ͨ��
	uint32 moto_avespeed;				// Ħ�г�ƽ���ٶ�
	uint32 moto_speed_total;			// Ħ�г����ٶ�
	uint32 jizhuangxiang_avespeed;	  	// ��װ�䳵ƽ���ٶ�
	uint32 jizhuangxiang_speed_total; 	// ��װ�䳵���ٶ�
	uint32 jizhuangxiang_num;		  	// ��װ�䳵���ٶ�

}TRAFFIC_DATA_SEND01	 ;
extern TRAFFIC_DATA_SEND01  Data_4_Lane[4];
extern uint16 DataResendBgnNum;
extern uint16 DataResendEndNum;
extern uint16 DataResendCnt;
 
extern	uint8 Flag_08_NotReturn;						//08��δ���ر�־λ��Ϊ1��ʾ���Ѿ����յ����ݣ�����û�з���08��
extern uint8 Flag_NetConnect;
extern uint8 Flag_08_ReturnChangePara;
//����ʹ�õĲ���
extern uint8 UserName[8]; 					//�û���
extern uint8 KeyNum[8];						//����
extern uint8 RDid[];						//�豸���ʶ����
extern uint8 RDNum[15];						//վ���ţ�15λ������15λ��0
extern uint16 ProCycle;						//��ͨ���ݴ������� 
extern uint8 InvContents; 					//�������� 	
extern uint8 DisTime;						//�����ٷֱȼ���ʱ�� 

//�޸ĺ�δ��Ч�Ĳ���
extern uint8 NewUserName[8]; 				//���û���
extern uint8 NewKeyNum[8];					//������
extern uint8 NewRDid[];						//���豸���ʶ����
extern uint8 NewRDNum[15];					//��վ���ţ�15λ������15λ��0
extern uint16 NewProCycle;					//�½�ͨ���ݴ������� 
extern uint8 NewInvContents; 				//�µ������� 	
extern uint8 NewDisTime;					//�¸����ٷֱȼ���ʱ�� 
extern uint8 NewDscIp[4];					//�·�����IP��ַ 

extern uint8 ConnectType;					//��ǰ���䷽ʽ��01���������紫��  02���������紫��
extern double Longitude;					//����
extern double Latitude;						//γ��
extern uint16 Elevation;					//����	
//extern uint16 SendFailNumEnd;

extern uint8 Flag_Change_RD_Num;
extern uint8 Flag_Change_DSC_Ip;
extern uint8 Flag_Change_InvContents;
extern uint8 Flag_Change_ProCycle;
extern uint8 Flag_Change_DisTime;
extern uint8	Hisyear;
extern uint8	Hismonth;
extern uint8	Hisday;
extern	union ASK08Data	ASK08_data;
extern	union ASK02Data	ASK02_data;
extern	union ASK08Data_Modify ASK08_data_Modify;
extern	char FlagSentVechicle;
extern	uint8 Flag_ReConnect;
extern	uint8 Flag_08_ReturnChangePara;
extern void Update_data02(void);
extern void Update_data08(void);
extern int32 Save_data_01_process(void);
extern uint8  ChangetoBG_chedao_num( uint8 ln );
extern	void JudgeQual(uint8 Qual, uint8 *p);
extern	void SendDevParamInfo(void);
extern	void SendDevOtherInfo(void);
extern	void ClearResetInfoBuf(void);
extern	void SetDevParamInfor(uint8 *pUartDataBuf);
extern	void SendDevParamInfor2(void);
extern	void Sendframe(void);
extern	void Sendframe2(void);
extern  void GetDeviceTime(void);
extern  void SetDeviceTime(uint8 *pUartDataBuf);
extern  void SetRDID(uint8 *pUartDataBuf);
extern  void GetRDID(void);
extern  void GetLaneDir(void);
extern  void SetLaneDir(uint8 *pUartDataBuf);
//extern  void GetSD01Data(uint8 *pUartDataBuf);
extern  void ResetDevice(void);
extern  void InitAllParam(void);
extern  void SetLimitHeight(uint8 *p);

extern uint8 Flag_08_NotReturn;						//08��δ���ر�־λ��Ϊ1��ʾ���Ѿ����յ����ݣ�����û�з���08��
extern uint8 Flag_02Rev;							//02���յ���־λ���յ�02���ñ�־��1
extern uint8 Flag_0ARev;							//08���յ���־λ���յ�08���ñ�־��1

extern OS_EVENT *Flag_SendOK;						//���ݷ��ͳɹ��ź��������ݷ��ͺ��ͷŸ��ź�������������ʱ������ź���
extern OS_EVENT *EVENT_02Rev;						//02���յ��ź������յ�ʱ�ͷ��ź���������01��ǰ�ȷ���02������������ź���
extern OS_EVENT *EVENT_0ARev;						//0A���յ�
extern OS_EVENT *EVENT_11Rev;

//���ݰ���ͷ 
struct	RD_data	{
				uint8	Type;					//���ݰ����� 
				uint8	RD_ID[16];				//�豸���ʶ���룬����ASCII���ʾ 
				};


//0A��ʽ���ݰ�
struct ASK_0A	{
				uint8	Type;					//���ݰ�����
				uint8	TimeNumL;				//ʱ����ŵ�8λ
				uint8	TimeNumH;				//ʱ����Ÿ�8λ 
				uint8	crcL;					//У������Ϣ������ֽ�
				uint8	crcH;					//У������Ϣ������ֽ�
				};


//0x02��ʽ���ݰ� 
struct ASK_02	{
				struct	RD_data	RDHead;
				uint8	Ask_Answer;				//ѯ����ظ���Ϣ 
				};

//0x08��ʽ���ݰ�	
struct ASK_08	{
				struct	RD_data	RDHead;
// 				uint16	year;					//��ݣ���λ��ǰ����λ�ں�
 				uint8	yearL;					//��ݣ���λ��ǰ����λ�ں�
				uint8	yearH;					//��ݣ���λ��ǰ����λ�ں� 
				uint8	month; 					//�·� 
				uint8	day;					//�� 
				uint8	hour;					//Сʱ 
				uint8	min;					//���� 
				uint8   sec;					//�� 
				uint8	ProCycle;				//��ͨ���ݴ������ڵ��ֽ�
				uint8	InvContents; 			//�������� 
				uint8	DSC_Ip[4];				//DSC IP 
				uint8	RD_Num[15];				//RDվ���ţ�ASCII���ʾ�����㲿����0 
				uint8	DisTime;				//�����ٷֱȼ���ʱ�� 
				uint8	ConnectType;			//��ǰ���䷽ʽ��01���������紫��  02���������紫��
				uint8	Longitude[8];			//���ȣ������������¼��ʹ��ʱת��Ϊ˫���ȸ�����
				uint8	Latitude[8];			//γ�ȣ������������¼��ʹ��ʱת��Ϊ˫���ȸ�����
				uint8	ElevationL;
				uint8	ElevationH;				//�̣߳����Σ����豸��װ������  	 
				};

//0x08��ʽ���ݰ�	
struct ASK_08_Modify	{
				struct	RD_data	RDHead;
// 				uint16	year;					//��ݣ���λ��ǰ����λ�ں�
 				uint8	yearL;					//��ݣ���λ��ǰ����λ�ں�
				uint8	yearH;					//��ݣ���λ��ǰ����λ�ں� 
				uint8	month; 					//�·� 
				uint8	day;					//�� 
				uint8	hour;					//Сʱ 
				uint8	min;					//���� 
				uint8   sec;					//�� 
				uint8	ProCycle;				//��ͨ���ݴ�������
				uint8	InvContents; 			//�������� 
				uint8	DSC_Ip[4];				//DSC IP 
				uint8	RD_Num[15];				//RDվ���ţ�ASCII���ʾ�����㲿����0 
				uint8	DisTime;				//�����ٷֱȼ���ʱ�� 
				uint8	ConnectType;			//��ǰ���䷽ʽ��01���������紫��  02���������紫��
				uint8	Longitude[8];			//���ȣ������������¼��ʹ��ʱת��Ϊ˫���ȸ�����
				uint8	Latitude[8];			//γ�ȣ������������¼��ʹ��ʱת��Ϊ˫���ȸ�����
				uint8	ElevationL;
				uint8	ElevationH;				//�̣߳����Σ����豸��װ������  	 
				};
				
//03��ʽ���ݰ�
struct ASK_03	{
				struct	RD_data	RDHead;
				uint8	UserName[8];			//�û�����ASCII��
				uint8	KeyNum[8];				//����
				uint8	New_RD_Num[15];			//��RDվ���ţ�ASCII���ʾ�����㲿����0
				};

//04��ʽ���ݰ�
struct ASK_04	{
				struct	RD_data	RDHead;
				uint8	UserName[8];			//�û�����ASCII��
				uint8	KeyNum[8];				//����
				uint8	New_DSC_Ip[4];			//��DSC IP 
				};

//05��ʽ���ݰ�
struct ASK_05	{
				struct	RD_data	RDHead;
				uint8	UserName[8];			//�û�����ASCII��
				uint8	KeyNum[8];				//����
 				uint8	yearL;					//��ݣ���λ��ǰ����λ�ں�
				uint8	yearH;					//��ݣ���λ��ǰ����λ�ں� 
				uint8	month; 					//�·� 
				uint8	day;					//�� 
				uint8	hour;					//Сʱ 
				uint8	min;					//���� 
				uint8   sec;					//�� 
				};


//06��ʽ���ݰ�
struct ASK_06	{
				struct	RD_data	RDHead;
				uint8	UserName[8];			//�û�����ASCII��
				uint8	KeyNum[8];				//����
				uint8	New_InvContents; 		//�µ������� 
				};

//07��ʽ���ݰ�
struct ASK_07	{
				struct	RD_data	RDHead;
				uint8	UserName[8];			//�û�����ASCII��
				uint8	KeyNum[8];				//����
				uint8	New_ProCycle;			//�½�ͨ���ݴ�������
				};

//0B��ʽ���ݰ�
struct ASK_0B	{
				struct	RD_data	RDHead;
				uint8	UserName[8];			//�û�����ASCII��
				uint8	KeyNum[8];				//����
				uint8	New_DisTime;			//�¸����ٷֱȼ���ʱ��
				};

//09��ʽ���ݰ�
struct ASK_09	{
				struct	RD_data	RDHead;
				uint8	UserName[8];			//�û�����ASCII��
				uint8	KeyNum[8];				//����
 				uint8	yearL;					//��ݣ���λ��ǰ����λ�ں�
				uint8	yearH;					//��ݣ���λ��ǰ����λ�ں� 
				uint8	month; 					//�·� 
				uint8	day;					//�� 
				uint8	StartNumL;				//���ʱ�����
				uint8	StartNumH;				//���ʱ�����
				uint8	EndNumL;				//ֹ��ʱ�����
				uint8	EndNumH;				//ֹ��ʱ�����
				};


union ASK08Data	{
		struct ASK_08	ASK08;
		uint8 	ASK08_65[65];
		};						 				//��08��ʽ���ݰ�ʱʹ��

union ASK02Data	{
		struct ASK_02	ASK02;
		uint8 	ASK02_18[18];					//��02��ʽ���ݰ�ʱʹ��
		};

union ASK08Data_Modify {
		struct ASK_08_Modify	ASK08_Modify;
		uint8 	ASK08_Modify_65[65];
		};						 				//��08��ʽ���ݰ�ʱʹ�ã����޸ģ���δ��Ч�Ĳ���

extern uint8 TcpData_Pro(uint8 *p_uart5buff,uint16 p_len);

//uint8 LMS_Data_Pro(uint16 *p,uint16 len);

void RD_DataPro(uint8 *p_data,uint16 len);

extern uint8 SendData(uint8 *p,uint16 len);
		
void RD_Int08(void);
void RD_Int02(void);
//02��ʽ���ݰ�������
void Ask_Answer(struct ASK_02 *p,uint16 len);
void Change_RD_Num(struct ASK_03 *p,uint16 len);
void Change_DSC_Ip(struct ASK_04 *p,uint16 len);
void Change_Time(struct ASK_05 *p,uint16 len);
void Change_InvContents(struct ASK_06 *p,uint16 len);
void Change_ProCycle(struct ASK_07 *p,uint16 len);
void Change_DisTime(struct ASK_0B *p,uint16 len);
//uint16	sumday(uint16 yearL, uint8 month, uint8 day);
extern uint32 SD_Base_address_01;


//�û���������У�麯������ȷ����1�����󷵻�0
uint8 UserName_Key(struct ASK_03 *p,uint16 len);
//�豸���ʶ����У�麯������ȷ����1�����󷵻�0
uint8 RD_ID_Key(struct RD_data *p,uint16 len);




extern	uint16	g_u16MsgPeriod;
#endif
