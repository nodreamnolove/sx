/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			FAT32.h
** Last modified Date:  2011-05-04
** Last Version:		1.0
** Descriptions:		FAT32驱动声明文件
**
*********************************************************************************************************/
#ifndef __FAT32_H__
#define __FAT32_H__

#ifdef	__FAT32_C
#define	FAT32_EXT		
#else
#define	FAT32_EXT	extern
#endif
//-------------------------------------------------------------------------
typedef struct{
	uint8		PB_BootIndicator;
	uint8		PB_StartHead;
	uint16	    PB_StartSector;
	uint8		PB_SystemID;
	uint8		PB_EndHead;
	uint16	    PB_EndSector;
	uint32	    PB_RelativeSector;
	uint32   	PB_TotalSector;		
} FAT_PB;
//-------------------------------------------------------------------------
typedef struct{
	uint8		MBR_mbr[446];			
	FAT_PB		MBR_pb[4];
	uint16	MBR_Signature;
} FAT_MBR;
//-------------------------------------------------------------------------
typedef struct{
	uint8		BS_jmpBoot[3];
	uint8		BS_OEMName[8];
	uint16	    BPB_BytesPerSec;
	uint8		BPB_SecPerClus;
	uint16  	BPB_RsvdSecCnt;
	uint8		BPB_NumFATs;
	uint16  	BPB_RootEntCnt;
	uint16   	BPB_TotSec16;
	uint8		BPB_Media;
	uint16  	BPB_FATSz16;
	uint16  	BPB_SecPerTrk;
	uint16  	BPB_NumHeads;
	uint32  	BPB_HiddSec;
	uint32  	BPB_TotSec32;
	uint8		BS_DrvNum;
	uint8		BS_Reservedl;
	uint8		BS_BootSig;
	uint32	    BS_VolID;
	uint8		BS_VolLab[11];
	uint8		BS_FilSysType[8];
	uint8		ExecutableCode[448];
	uint8		ExecutableMarker[2];
} FAT_BPB;
//-------------------------------------------------------------------------
typedef struct{
	uint8		NAME[8];
	uint8		TYPE[3];
} FILE_NAME;
//-------------------------------------------------------------------------
typedef struct{
	uint16  	Start;
	uint32  	Size;
} FILE_POSIT;
//-------------------------------------------------------------------------
typedef struct{
	FILE_NAME	FileName;
	uint8		FileAttrib;
	uint8		FileNtres;
	uint8		FileCrtTimeTeenth;
	uint8		FileCrtTime[2];
	uint8		FileDate[2];
	uint8		FileLastAccDate[2];
	uint16  	FileFatClusHI;
	uint8   	FileUpdateTime[2];
	uint8		FileUpdateData[2];
	FILE_POSIT	FilePosit;
} DIR;
//-------------------------------------------------------------------------
typedef struct{
	uint16  	ClusID;
	uint16  	SecOfClus;
	uint16  	ByteOfSec;
} DATA_POSIT;
//-------------------------------------------------------------------------
#define	SD_SUCC				0
#define	SD_FAIL				1

FAT32_EXT	uint32  	BPB_RootClus;							//根目录所在第一簇的簇号
FAT32_EXT	uint32  	BPB_FATsz32;							//每个FAT 占用的扇区数,针对FAT32
FAT32_EXT	uint16  	BPB_RsvdSecCnt;							//保留扇区数




uint32 GetRootFileSec(void);

uint8  ReadBlock(uint32   LBA);						//绝对地址读一个扇区
uint8  WriteBlock(uint32   LBA);					//绝对地址写一个扇区
uint8  ReadFatBlock(uint32   Add);					//逻辑地址读一个扇区
uint8  WriteFatBlock(uint32   Add);			    	//逻辑地址写一个扇区

void   CopyBytes(uint8  *ps,uint8  *pd,uint16  size);	//内存拷贝
uint8  IsEqual(uint8  *pa,uint8  *pb,uint8  size);	    //内存比较
void   EmptyBytes(uint8  *pd,uint16  size);			    //内存清空

uint8  ReadMBR(void);								         //读取MBR数据结构
uint8  ReadBPB(void);								         //读取BPB数据结构
uint32  DirStartSec(void);							         //获取根目录开始扇区号
uint16  GetDirSecCount(void);						         //目录项占用的扇区数
uint32  DataStartSec(void);							         //获取数据区开始扇区号
uint32  ClusConvLBA(uint16  ClusID);				         //获取一个簇的开始扇区
uint32  ReadFAT(uint32  Index);						         //读取文件分配表的指定项
void   WriteFAT(uint32  Index,uint32  Value);		         //写文件分配表的指定项
uint16  GetEmptyDIR(void);							         //获取根目录中可以使用的一项
uint8  GetFileID(uint8  Name[11],DIR *ID,uint16  *pIndex);	 //获得和文件名对应的目录
uint32  GetNextFAT(void);							         //获取一个空的FAT项
void ReadDIR(uint16  Index, DIR* Value);			         //读取根目录的指定项
void WriteDIR(uint16  Index, DIR* Value);			         //写根目录的指定项
void CopyFAT(void);									         //复制文件分配表,使其和备份一致

uint8  CreateFile(uint8  *Name,uint32  Size);	                        //创建一个空文件
uint8  ReadFile(uint8  Name[11],uint32  Start,uint32  Len,uint8  *p);	//读文件
uint8  WriteFile(uint8  Name[11],uint32  Start,uint32  Len,uint8  *p);	//写文件
uint8  EreaseFile(uint8  Name[11]);					                    //删除文件

uint8  InitFat16(void);							    //初始化FAT16的变量


 
 #endif

