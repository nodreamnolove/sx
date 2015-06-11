/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			FAT32.c
** Last modified Date:  2011-05-04
** Last Version:		1.0
** Descriptions:		FAT32������غ���
**
*********************************************************************************************************/
#include "sdconfig.h"
#include "FAT32.H"
#include "FAT32App.h"
#define  SDCOMMON_GLOBALS

//------------------------------------------------------------------------
#define	SEC_Size				512
#define	MBR_Sector				0				//���Ե�ַ
#define	FAT_Sector				0				//�߼���ַ
//-------------------------------------------------------------------------
static	uint8 		BUFFER[SEC_Size];
static	uint8 		PB_RelativeSector=0;
static	uint16   	BPB_BytesPerSec;						//ÿ�������ֽ���
static	uint8 		BPB_SecPerClus;							//ÿ���ص�������
static	uint16  	BPB_RsvdSecCnt;							//����������
static	uint8 		BPB_NumFATs;							//FAT ������Ŀ
static	uint16  	BPB_RootEntCnt;							//����FAT32�����������Ϊ0
static	uint16  	BPB_TotSec16;
static	uint16  	BPB_FATSz16;							//FATռ�õ�sectors
static	uint32  	BPB_HiddSec;
static	uint32  	BPB_TotSec32;
static	uint32  	BPB_FATsz32;							//ÿ��FAT ռ�õ�������,���FAT32
static	uint32  	BPB_RootClus;							//��Ŀ¼���ڵ�һ�صĴغ�

static	uint32  	TheLastFAT=0;

//static	sd_struct	sd_info;					//SD����Ϣ
//-------------------------------------------------------------------------
uint8  ReadBlock(uint32  LBA)						//���Ե�ַ��һ������
{					
//	if(SD_ReadBlock(&sd_info, LBA,BUFFER)!=0)return SD_FAIL;

	if(ReadSDCardBlock(LBA,BUFFER)!=0)return SD_FAIL;
	return SD_SUCC;
}
//-------------------------------------------------------------------------
uint8  WriteBlock(uint32  LBA)					//���Ե�ַдһ������
{

//	if(SD_WriteBlock(&sd_info, LBA, BUFFER)!=0)return SD_FAIL;
	if(WriteSDCardBlock(LBA, BUFFER)!=0)return SD_FAIL;
	return SD_SUCC;
}
//-------------------------------------------------------------------------
uint8  ReadFatBlock(uint32  Add)					//�߼���ַ��һ������
{
//	return SD_ReadBlock(&sd_info, Add+PB_RelativeSector,BUFFER);
    return ReadSDCardBlock(Add+PB_RelativeSector,BUFFER);
}
//-------------------------------------------------------------------------
uint8  WriteFatBlock(uint32  Add)			    	//�߼���ַдһ������
{
//	return SD_WriteBlock(&sd_info, Add+PB_RelativeSector,BUFFER);
    return WriteSDCardBlock(Add+PB_RelativeSector,BUFFER);
}
//-------------------------------------------------------------------------
void CopyBytes(uint8  *ps,uint8  *pd,uint16  size){	//�ڴ濽��
	for(;size;size--)*pd++=*ps++;
}
//-------------------------------------------------------------------------
uint8  IsEqual(uint8  *pa,uint8  *pb,uint8  size){	//�ڴ�Ƚ�
	for(;size;size--)if(*pa++!=*pb++)return 0;
	return 1;
}
//-------------------------------------------------------------------------
void EmptyBytes(uint8  *pd,uint16  size){			//�ڴ����
	for(;size;size--)*pd++ =0;
}
//-------------------------------------------------------------------------
uint8  ReadMBR(void){									//��ȡMBR���ݽṹ
/*	uint8  ok;
	uint16  i;
	FAT_MBR * MBR;
	
	ok=ReadBlock(MBR_Sector);
	for(i=0;i<446;i++)
		MBR->MBR_mbr[i] = BUFFER[i];
	for(i=0;i<4;i++)
	{
	    MBR->MBR_pb[i].PB_BootIndicator	= BUFFER[446+i*16];
		MBR->MBR_pb[i].PB_StartHead		= BUFFER[446+i*16+1];
		
		MBR->MBR_pb[i].PB_StartSector	= BUFFER[446+i*16+3];
		MBR->MBR_pb[i].PB_StartSector	= MBR->MBR_pb[i].PB_StartSector*256 + BUFFER[446+i*16+2];
		
		MBR->MBR_pb[i].PB_SystemID		= BUFFER[446+i*16+4];
		MBR->MBR_pb[i].PB_EndHead		= BUFFER[446+i*16+5];
		
		MBR->MBR_pb[i].PB_EndSector		= BUFFER[446+i*16+7];
		MBR->MBR_pb[i].PB_EndSector		= MBR->MBR_pb[i].PB_EndSector*256 + BUFFER[446+i*16+6];
		
		MBR->MBR_pb[i].PB_RelativeSector= BUFFER[446+i*16+11];
		MBR->MBR_pb[i].PB_RelativeSector= MBR->MBR_pb[i].PB_RelativeSector*256 + BUFFER[446+i*16+10];
		MBR->MBR_pb[i].PB_RelativeSector= MBR->MBR_pb[i].PB_RelativeSector*256 + BUFFER[446+i*16+9];
		MBR->MBR_pb[i].PB_RelativeSector= MBR->MBR_pb[i].PB_RelativeSector*256 + BUFFER[446+i*16+8];
		
		MBR->MBR_pb[i].PB_TotalSector	= BUFFER[446+i*16+15];	
		MBR->MBR_pb[i].PB_TotalSector	= MBR->MBR_pb[i].PB_TotalSector*256 + BUFFER[446+i*16+14];
		MBR->MBR_pb[i].PB_TotalSector	= MBR->MBR_pb[i].PB_TotalSector*256 + BUFFER[446+i*16+13];
		MBR->MBR_pb[i].PB_TotalSector	= MBR->MBR_pb[i].PB_TotalSector*256 + BUFFER[446+i*16+12];
	}
	
	MBR->MBR_Signature = BUFFER[511];
	MBR->MBR_Signature = MBR->MBR_Signature *256 +BUFFER[510];
	
	if(ok==SD_FAIL)return SD_FAIL;
	if(MBR->MBR_Signature!=0xAA55)return SD_FAIL;		//����Ч��־
		
	//��ȡ����
	//PB_RelativeSector=MBR->MBR_pb[0].PB_RelativeSector;//���߼���ַ����Ե�ַ��ƫ��
	PB_RelativeSector=0;	*/
	return SD_SUCC;
}
//-------------------------------------------------------------------------
uint8  ReadBPB(void){									//��ȡBPB���ݽṹ
	uint8  ok;
	FAT_BPB * BPB=(FAT_BPB*)BUFFER;
	ok=ReadFatBlock(FAT_Sector);
	if(ok==SD_FAIL)return SD_FAIL;
	
	//��ȡ����
	BPB_BytesPerSec = BUFFER[12]*256 + BUFFER[11];	//ofs:11.ÿ�����ֽ�����
	BPB_SecPerClus = BUFFER[13];					//ofs:13.ÿ��������
	BPB_RsvdSecCnt = BUFFER[14] + BUFFER[15]*256;	//ofs:14.��������������DBR ��FAT ����������
	BPB_NumFATs = BUFFER[16];						//ofs:16.FAT �ĸ�����
	BPB_RootEntCnt = BUFFER[17] + BUFFER[18]*256;	//ofs:17.��Ŀ¼����
	BPB_TotSec16 = BUFFER[19] + BUFFER[20]*256;		//ofs:19.������������(<32M ʱ��)
	BPB_FATSz16 = BUFFER[22] + BUFFER[23]*256;		//ofs:22.ÿ��FAT ռ����������
	BPB_HiddSec = BUFFER[28] + BUFFER[29]*256 + BUFFER[30]*256*256 + BUFFER[31]*256*256*256;//ofs:28.��������������MBR ��DBR ����������							//����������
	BPB_TotSec32 = BUFFER[32] + BUFFER[33]*256 + BUFFER[34]*256*256 + BUFFER[35]*256*256*256;//ofs:32.������������(>=32M ʱ��)��
	BPB_FATsz32 = BUFFER[36] + BUFFER[37]*256 + BUFFER[38]*256*256 + BUFFER[39]*256*256*256; //ofs:36.ÿ��FAT ռ�õ�������,���FAT32
	BPB_RootClus= BUFFER[44] + BUFFER[45]*256 + BUFFER[46]*256*256 + BUFFER[47]*256*256*256; //ofs:44.��Ŀ¼���ڵ�һ�صĴغ�
	return SD_SUCC;
}
//-------------------------------------------------------------------------
uint32  DirStartSec(void){							//��ȡ��Ŀ¼��ʼ������
	uint32 t_RootFlieClus;
	t_RootFlieClus = GetRootFileSec();	
	return BPB_RsvdSecCnt+BPB_NumFATs*BPB_FATsz32+(t_RootFlieClus-2)*BPB_SecPerClus;
}
//-------------------------------------------------------------------------
uint16  GetDirSecCount(void){						//Ŀ¼��ռ�õ�������
	return BPB_RootEntCnt*32/BPB_BytesPerSec;
}
//-------------------------------------------------------------------------
uint32  DataStartSec(void){							//��ȡ��������ʼ������
	return BPB_RsvdSecCnt+BPB_NumFATs*BPB_FATsz32+BPB_RootEntCnt*32/BPB_BytesPerSec;
}
//-------------------------------------------------------------------------
uint32  ClusConvLBA(uint16  ClusID){				//��ȡһ���صĿ�ʼ����
	return DataStartSec()+BPB_SecPerClus*(ClusID-2);
}
//-------------------------------------------------------------------------
uint32  ReadFAT(uint32  Index)						//��ȡ�ļ��������ָ����
{						
	uint32  *RAM=(uint32*)BUFFER;
	uint32  SecID;
	
	SecID=BPB_RsvdSecCnt+Index/128;
	ReadFatBlock(SecID);
	return RAM[Index%128];
}
//-------------------------------------------------------------------------
void WriteFAT(uint32  Index,uint32  Value){			//д�ļ��������ָ����
	uint32  *RAM=(uint32*)BUFFER;
	uint32  SecID;
	
	SecID=BPB_RsvdSecCnt+Index/128;
	ReadFatBlock(SecID);
	RAM[Index%128]=Value;
	WriteFatBlock(SecID);
	WriteFatBlock(SecID+BPB_FATsz32);				//����FAT��
}
//-------------------------------------------------------------------------
uint16  GetEmptyDIR(void){							//��ȡ��Ŀ¼�п���ʹ�õ�һ��
	uint32  i,DirSecCut,DirStart,m,ID=0;
	uint32 t_NextRootFileClus = 0;
	uint8 j;
	uint16 SD_temp;
	uint8 SD_ZeroBuffer[512];

	for(SD_temp=0;SD_temp<512;SD_temp++)
	{
		SD_ZeroBuffer[SD_temp] = 0x00;	
	}
	DirStart=DirStartSec();

	for(i=0;i<8;i++)
	{
		ReadFatBlock(DirStart+i);
		for(m=0;m<16;m++)
		{
			if(BUFFER[m*32]==0)return ID;
			if(BUFFER[m*32]==0xe5)return ID;
			ID++;
			if(ID == 128)								//������ǰ��Ŀ¼���޿��õ�ַ
			{
				ID = 0;
				t_NextRootFileClus = GetNextFAT();		//�ҳ���һ��Ϊ�յ�FAT��ַ
				WriteFAT(GetRootFileSec(),t_NextRootFileClus);
				WriteFAT(t_NextRootFileClus,0x0fffffff);
				//���µ�ROOTFILE�� ���Ƚ���Ŀ¼����
				DirStart=DirStartSec();					//��ȡ�·���Ĵصĵ�ַ
				for(j=0;j<8;j++)
				{
					 WriteSDCardBlock(DirStart+j,SD_ZeroBuffer);		//����cluster ���� sector one by one.
				}
				
				return ID;
			}
		}
	}
	//if(ReadFAT(BPB_RootClus) != 0x0fffffff)		//Ŀ¼������һ���ص����
	return ID;
}
//-------------------------------------------------------------------------
//��ú��ļ�����Ӧ��Ŀ¼
uint8  GetFileID(uint8  Name[11],DIR *ID,uint16  *pIndex){
	uint32   i,DirSecCut,DirStart,m;
	
	//DirSecCut = GetDirSecCount();
	DirSecCut=128;									//��ϵͳ�������1024��Ŀ¼������			
	DirStart = DirStartSec();
	for(i=0,*pIndex=0;i<DirSecCut;i++){
		ReadFatBlock(DirStart+i);
		for(m=0;m<16;m++){
			if(IsEqual(Name,(uint8 *)&((DIR*)&BUFFER[m*32])->FileName,11))
			{
				*ID = *((DIR*)&BUFFER[m*32]);
				ID->FileFatClusHI 	= BUFFER[m*32+21]*256 + BUFFER[m*32+20];
				ID->FilePosit.Start = BUFFER[m*32+27]*256 + BUFFER[m*32+26];
				ID->FilePosit.Size  = BUFFER[m*32+31]*256*256*256 + BUFFER[m*32+30]*256*256 + BUFFER[m*32+29]*256 + BUFFER[m*32+28];
  				return 1; 						//�ҵ���Ӧ��Ŀ¼��,����1.
			}
			(*pIndex)++;
		}
	}
	return 0; 									//û���ҵ���Ӧ��Ŀ¼��,����0.
}
//-------------------------------------------------------------------------
//uint32  GetNextFAT(void)
//{							//��ȡһ���յ�FAT��
//	uint32  FAT_Count,i;
//	FAT_Count=BPB_FATsz32*128; 						//FAT��������
//	for(i=TheLastFAT+1;i<FAT_Count;i++)
//	{
//		if(ReadFAT(i)==0)
//		{
//		    TheLastFAT = i;		//��ס���һ�εĿ�FAT�����ٱ������������ϵͳ�ٶ�
//		    return i;
//		}
//	}
//	return 0;
//}

//�Ż�Ѱ����һ����FAT��--����һ�������е�FAT��ֵ��һ�Σ�ԭʼ�ķ�����ÿ��Ѱ�ҿ�FAT����FAT������һ�ζ�������ʱ
uint32  GetNextFAT(void)		   
{
	uint32  FAT_Count,i;
	uint32  SecIDNextFat;
	uint32  *RAMNextFat=(uint32*)BUFFER;

	FAT_Count=BPB_FATsz32*128; 						//FAT��������
	i = TheLastFAT+1;	
	SecIDNextFat=BPB_RsvdSecCnt+i/128;
	ReadFatBlock(SecIDNextFat);
	for(i=TheLastFAT+1;i<FAT_Count;i++)
	{
	   if(i%128 != 0)
	   {  
	      if(RAMNextFat[i%128]==0)
		  {
		      TheLastFAT = i;		//��ס���һ�εĿ�FAT�����ٱ������������ϵͳ�ٶ�
		      return i;		  
		  }	   
	   }
	   else
	   {
	   	  SecIDNextFat++;
		  ReadFatBlock(SecIDNextFat);
		  if(RAMNextFat[i%128]==0)
		  {
		      TheLastFAT = i;		//��ס���һ�εĿ�FAT�����ٱ������������ϵͳ�ٶ�
		      return i;		  
		  }
	   }
	}
	return 0;    //FAT���������޿�FAT

}

uint32 GetRootFileSec(void)
{
	uint32 t_RootFileSec;
	uint32 FAT_Count;

	t_RootFileSec = 0x02;								//Ĭ�������¸�Ŀ¼�Ĵؿ�ʼ��ַΪ0x22;
	FAT_Count = BPB_FATsz32*128; 						//FAT��������

	while(ReadFAT(t_RootFileSec) != 0x0FFFFFFF)
	{
		t_RootFileSec = ReadFAT(t_RootFileSec);
	}

	return t_RootFileSec;			
}

//-------------------------------------------------------------------------
void ReadDIR(uint16  Index, DIR* Value){			//��ȡ��Ŀ¼��ָ����
	uint32  LBA = DirStartSec()+Index/16;			//ÿ��������16��Ŀ¼��
	ReadFatBlock(LBA);
	CopyBytes((uint8 *)&BUFFER[(Index%16)*32],(uint8 *)Value,32);
	Value->FileFatClusHI = BUFFER[(Index%16)*32+21]*256 + BUFFER[(Index%16)*32+20];
	Value->FilePosit.Start = BUFFER[(Index%16)*32+27]*256 + BUFFER[(Index%16)*32+26];
	Value->FilePosit.Size  = BUFFER[(Index%16)*32+31]*256*256*256 + BUFFER[(Index%16)*32+30]*256*256 + BUFFER[(Index%16)*32+29]*256 + BUFFER[(Index%16)*32+28];
}
//-------------------------------------------------------------------------
void WriteDIR(uint16  Index, DIR* Value){			//д��Ŀ¼��ָ����
	uint32  LBA = DirStartSec()+Index/16;
	ReadFatBlock(LBA);
	CopyBytes((uint8 *)Value,(uint8 *)&BUFFER[(Index%16)*32],32);
	BUFFER[(Index%16)*32+20] = Value->FileFatClusHI&0x00ff;
	BUFFER[(Index%16)*32+21] = (Value->FileFatClusHI&0xff00)>>8;
	BUFFER[(Index%16)*32+26] = Value->FilePosit.Start&0x00ff;
	BUFFER[(Index%16)*32+27] = (Value->FilePosit.Start&0xff00)>>8;
	BUFFER[(Index%16)*32+28] = Value->FilePosit.Size&0x000000ff;
	BUFFER[(Index%16)*32+29] = (Value->FilePosit.Size&0x0000ff00)>>8;
	BUFFER[(Index%16)*32+30] = (Value->FilePosit.Size&0x00ff0000)>>16;
	BUFFER[(Index%16)*32+31] = (Value->FilePosit.Size&0xff000000)>>24;
	WriteFatBlock(LBA);
}
//-------------------------------------------------------------------------
void CopyFAT(void){						//�����ļ������,ʹ��ͱ���һ��
	uint32  i;

	for(i=0;i<BPB_FATsz32;i++){
		ReadFatBlock(BPB_RsvdSecCnt+i);
		WriteFatBlock(BPB_RsvdSecCnt+BPB_FATsz32+i);
	}
}
//-------------------------------------------------------------------------
uint8  CreateFile(uint8  *Name,uint32  Size){	//����һ�����ļ�
	uint32  ClusID, ClusNum, ClusNext, i;
	uint16  dirIndex;
	DIR FileDir;
	
	if(GetFileID(Name,&FileDir,&dirIndex)==1)return SD_FAIL;	//�ļ��Ѵ���
	
	ClusNum=Size/(BPB_SecPerClus*BPB_BytesPerSec)+1;
	
	EmptyBytes((uint8 *)&FileDir,sizeof(DIR));
	CopyBytes(Name,(uint8 *)&FileDir.FileName,11);
	
	ClusID=GetNextFAT();
	FileDir.FilePosit.Size  = Size;
	FileDir.FileFatClusHI	= ClusID>>16;
	FileDir.FilePosit.Start = ClusID;
	
	for(i=0;i<ClusNum-1;i++)
	{
		//WriteFAT(ClusID,0x0fffffff);
		ClusNext=GetNextFAT();
		WriteFAT(ClusID,ClusNext);
		ClusID=ClusNext;
	}
	WriteFAT(ClusID, 0x0fffffff);
	
	WriteDIR(GetEmptyDIR(),&FileDir);
	
	//CopyFAT();
	return SD_SUCC;
}
//-------------------------------------------------------------------------
//���ļ�
uint8  ReadFile(uint8  Name[11],uint32  Start,uint32  Len,uint8  *p)
{
	uint32  BytePerClus,ClusID,m;
	uint16  dirIndex;
	uint32  LBA;
	uint8 	 i;
	DIR      FileDir;
	
	if(GetFileID(Name,&FileDir,&dirIndex)==0)return SD_FAIL;//�ļ�������
	
	BytePerClus=BPB_SecPerClus*BPB_BytesPerSec;		//ÿ�ص��ֽ���	
	m=Start/BytePerClus;					//���㿪ʼλ�ð����Ĵ���
	ClusID=FileDir.FilePosit.Start + FileDir.FileFatClusHI*256*256;	//�ļ��Ŀ�ʼ�غ�
	for(i=0;i<m;i++)ClusID=ReadFAT(ClusID);			//���㿪ʼλ�����ڴصĴغ�	
	i=(Start%BytePerClus)/BPB_BytesPerSec;			//���㿪ʼλ�����������Ĵ���ƫ��
	LBA=ClusConvLBA(ClusID)+i;				//���㿪ʼλ�õ��߼�������
	m=(Start%BytePerClus)%BPB_BytesPerSec;			//���㿪ʼλ����������ƫ��

READ:
	for(;i<BPB_SecPerClus;i++){
		ReadFatBlock(LBA++);
		for(;m<BPB_BytesPerSec;m++){
			*p++=BUFFER[m];
			if(--Len==0)return SD_SUCC;			//�����ȡ��ɾ��˳�
		}
		m=0;
	}
	i=0;
	ClusID=ReadFAT(ClusID);							//��һ�شغ�
	LBA=ClusConvLBA(ClusID);
	goto READ;
}
//-------------------------------------------------------------------------
//д�ļ�
uint8  WriteFile(uint8  Name[11],uint32  Start,uint32  Len,uint8  *p)
{
	uint16  BytePerClus,ClusID,m,dirIndex;
	uint32  LBA;
	uint8 	 i;
	DIR      FileDir;
	
	if(GetFileID(Name,&FileDir,&dirIndex)==0)return SD_FAIL;//�ļ�������
	
	BytePerClus=BPB_SecPerClus*BPB_BytesPerSec;		// ÿ�ص��ֽ���	
	m=Start/BytePerClus;					        //���㿪ʼλ�ð����Ĵ���
	ClusID=FileDir.FilePosit.Start + FileDir.FileFatClusHI*256*256;	//�ļ��Ŀ�ʼ�غ�
	for(i=0;i<m;i++)ClusID=ReadFAT(ClusID);			//���㿪ʼλ�����ڴصĴغ�	
	i=(Start%BytePerClus)/BPB_BytesPerSec;			//���㿪ʼλ�����������Ĵ���ƫ��
	LBA=ClusConvLBA(ClusID)+i;				        //���㿪ʼλ�õ��߼�������
	m=(Start%BytePerClus)%BPB_BytesPerSec;			//���㿪ʼλ����������ƫ��

WRITE:
	for(;i<BPB_SecPerClus;i++)
	{
		ReadFatBlock(LBA);
		for(;m<BPB_BytesPerSec;m++)
		{
			BUFFER[m]=*p++;
			if(--Len==0)
			{							            //�����ȡ��ɾ��˳�
				WriteFatBlock(LBA);					//��д����
				return SD_SUCC;				
			}
		}
		m=0;
		WriteFatBlock(LBA++);						//��д����
	}
	i=0;
	ClusID=ReadFAT(ClusID);							//��һ�شغ�
	LBA=ClusConvLBA(ClusID);
	goto WRITE;
}
//-------------------------------------------------------------------------
uint8  InitFat16(void){							//��ʼ��FAT16�ı���
	unsigned char i;
	
	i=ReadMBR();
	if(i==SD_FAIL)return SD_FAIL;
	
	i=ReadBPB();	
	if(i==SD_FAIL)return SD_FAIL;

	return SD_SUCC;
	
}
//-------------------------------------------------------------------------
//ɾ���ļ�
uint8  EreaseFile(uint8  Name[11]){
	uint32  ClusID,ClusNext,i;
	uint16  dirIndex;
	DIR FileDir;
	
	if(GetFileID(Name,&FileDir,&dirIndex)==0)return SD_FAIL;	//�ļ�������
	ClusID=FileDir.FilePosit.Start + FileDir.FileFatClusHI*256*256;	//�ļ��Ŀ�ʼ�غ�
	
EREASEFAT:
	if((ClusNext=ReadFAT(ClusID))!=0x0fffffff){		//ɾ��FAT���е�����
		WriteFAT(ClusID,0x00000000);
		ClusID=ClusNext;
	}else{
		WriteFAT(ClusID,0x00000000);
		goto EREASEFATEND;
	}
	goto EREASEFAT;
EREASEFATEND:
	
	FileDir.FileName.NAME[0]=0xe5;					//ɾ��Dir�е��ļ���¼
	WriteDIR(dirIndex,&FileDir);
	CopyFAT();										//FAT2<-FAT1
	return SD_SUCC;
}
//-------------------------------------------------------------------------