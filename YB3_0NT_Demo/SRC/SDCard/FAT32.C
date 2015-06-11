/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			FAT32.c
** Last modified Date:  2011-05-04
** Last Version:		1.0
** Descriptions:		FAT32驱动相关函数
**
*********************************************************************************************************/ 
#define	__FAT32_C	  
#define SDCOMMON_GLOBALS
#include "FAT32.H"	

//------------------------------------------------------------------------
#define		SECSIZE			512
#define		MBRSIZE			0				//绝对地址
#define		FATSECTOR		0				//逻辑地址
//-------------------------------------------------------------------------
static	uint8 		BUFFER[SECSIZE];
static	uint8 		PB_RelativeSector=0;
static	uint16   	BPB_BytesPerSec;						//每个扇区字节数
static	uint8 		BPB_SecPerClus;							//每个簇的扇区数
static	uint8 		BPB_NumFATs;							//FAT 表的数目
static	uint16  	BPB_RootEntCnt;							//对于FAT32，此区域必须为0
static	uint16  	BPB_TotSec16;
static	uint16  	BPB_FATSz16;							//FAT占用的sectors
static	uint32  	BPB_HiddSec;
static	uint32  	BPB_TotSec32;
static	uint32  	TheLastFAT=0;

//static	sd_struct	sd_info;					//SD卡信息
//-------------------------------------------------------------------------
uint8 ReadBlock(uint32 LBA)						//绝对地址读一个扇区
{					
	if(ReadSDCardBlock(LBA,BUFFER)!=0)return SD_FAIL;
	return SD_SUCC;
}

//-------------------------------------------------------------------------
uint8  WriteBlock(uint32  LBA)					//绝对地址写一个扇区
{
	if(WriteSDCardBlock(LBA, BUFFER)!=0)
		return SD_FAIL;

	return SD_SUCC;
}
//-------------------------------------------------------------------------
uint8 ReadFatBlock(uint32 Add)					//逻辑地址读一个扇区
{
    return ReadSDCardBlock(Add+PB_RelativeSector,BUFFER);
}

//-------------------------------------------------------------------------
uint8 WriteFatBlock(uint32 Add)			    	//逻辑地址写一个扇区
{
    return WriteSDCardBlock(Add+PB_RelativeSector,BUFFER);
}

//-------------------------------------------------------------------------
void CopyBytes(uint8 *ps,uint8 *pd,uint16 size)	//内存拷贝
{	
	for(;size;size--)
		*pd++	= *ps++;
}

//-------------------------------------------------------------------------
uint8 IsEqual(uint8 *pa, uint8 *pb,uint8 size)	//内存比较
{	
	for(;size;size--)
		if(*pa++!=*pb++)
			return 0;
	return 1;
}

//-------------------------------------------------------------------------
void EmptyBytes(uint8  *pd,uint16  size)		//内存清空
{	
	for(;size;size--)
		*pd++	= 0;
}

//-------------------------------------------------------------------------
uint8  ReadMBR(void)						//读取MBR数据结构
{									
	return SD_SUCC;
}

//-------------------------------------------------------------------------
uint8 ReadBPB(void)									//读取BPB数据结构
{	
	uint8	l_u8ReadResult;
	FAT_BPB * BPB	=(FAT_BPB*)BUFFER;
	l_u8ReadResult	= ReadFatBlock(FATSECTOR);
	if(l_u8ReadResult == SD_FAIL)
		return	SD_FAIL;
	
	//获取参数
	BPB_BytesPerSec	= BPB -> BPB_BytesPerSec;
	BPB_SecPerClus	= BPB -> BPB_SecPerClus;
	BPB_RsvdSecCnt	= BPB -> BPB_RsvdSecCnt;
	BPB_NumFATs		= BPB -> BPB_NumFATs;
	BPB_RootEntCnt	= BPB -> BPB_RootEntCnt;
	BPB_TotSec16	= BPB -> BPB_BytesPerSec;
	BPB_FATSz16		= BPB -> BPB_FATSz16;
	BPB_HiddSec		= BPB -> BPB_HiddSec;
	BPB_TotSec32	= BPB -> BPB_TotSec32;
	BPB_FATsz32		= *(uint32 *)(BUFFER + 36);	//ofs:36.每个FAT 占用的扇区数,针对FAT32
	BPB_RootClus	= *(uint32 *)(BUFFER + 44);	//ofs:44.根目录所在第一簇的簇号
	
//	BPB_BytesPerSec	= (BUFFER[12]<<8) + BUFFER[11];	//ofs:11.每扇区字节数。
//	BPB_SecPerClus	= BUFFER[13];					//ofs:13.每簇扇区数
//	BPB_RsvdSecCnt	= BUFFER[14] + (BUFFER[15]<<8);	//ofs:14.保留扇区数，从DBR 到FAT 的扇区数。
//	BPB_NumFATs		= BUFFER[16];					//ofs:16.FAT 的个数。
//	BPB_RootEntCnt	= BUFFER[17] + (BUFFER[18]<<8);	//ofs:17.根目录项数
//	BPB_TotSec16	= BUFFER[19] + (BUFFER[20]<<8);	//ofs:19.分区总扇区数(<32M 时用)
//	BPB_FATSz16		= BUFFER[22] + (BUFFER[23]<<8);	//ofs:22.每个FAT 占的扇区数。
//	BPB_HiddSec		= BUFFER[28] + (BUFFER[29]<<8) + (BUFFER[30]<<16) + (BUFFER[31]<<24);	//ofs:28.隐藏扇区数，从MBR 到DBR 的扇区数。							//保留扇区数
//	BPB_TotSec32	= BUFFER[32] + (BUFFER[33]<<8) + (BUFFER[34]<<16) + (BUFFER[35]<<24);	//ofs:32.分区总扇区数(>=32M 时用)。
//	BPB_FATsz32		= BUFFER[36] + (BUFFER[37]<<8) + (BUFFER[38]<<16) + (BUFFER[39]<<24);	//ofs:36.每个FAT 占用的扇区数,针对FAT32
//	BPB_RootClus	= BUFFER[44] + (BUFFER[45]<<8) + (BUFFER[46]<<16) + (BUFFER[47]<<24);	//ofs:44.根目录所在第一簇的簇号
	return SD_SUCC;
}

//-------------------------------------------------------------------------
uint32 DirStartSec(void)							//获取根目录开始扇区号
{	
	uint32 t_RootFlieClus;
	t_RootFlieClus	= GetRootFileSec();	
	return BPB_RsvdSecCnt+BPB_NumFATs*BPB_FATsz32+(t_RootFlieClus-2)*BPB_SecPerClus;
}

//-------------------------------------------------------------------------
uint16 GetDirSecCount(void)						//目录项占用的扇区数
{	
	return BPB_RootEntCnt*32/BPB_BytesPerSec;
}

//-------------------------------------------------------------------------
uint32 DataStartSec(void)							//获取数据区开始扇区号
{	
	return BPB_RsvdSecCnt+BPB_NumFATs*BPB_FATsz32+BPB_RootEntCnt*32/BPB_BytesPerSec;
}
//-------------------------------------------------------------------------
uint32 ClusConvLBA(uint16 ClusID)				//获取一个簇的开始扇区
{	
	return DataStartSec()+BPB_SecPerClus*(ClusID-2);
}

//-------------------------------------------------------------------------
uint32 ReadFAT(uint32 Index)						//读取文件分配表的指定项
{						
	uint32 *	RAM		= (uint32*)BUFFER;
	uint32		SecID;
	
	SecID	= BPB_RsvdSecCnt + Index/128;
	ReadFatBlock(SecID);
	return RAM[Index%128];
}

//-------------------------------------------------------------------------
void WriteFAT(uint32 Index, uint32 Value)			//写文件分配表的指定项
{	
	uint32 *	RAM	= (uint32*)BUFFER;
	uint32  	SecID;
	
	SecID	= BPB_RsvdSecCnt+Index/128;
	ReadFatBlock(SecID);
	RAM[Index%128]	= Value;
	WriteFatBlock(SecID);
	WriteFatBlock(SecID + BPB_FATsz32);				//复制FAT表
}

//-------------------------------------------------------------------------
uint16  GetEmptyDIR(void)							//获取根目录中可以使用的一项
{	
	uint32	i,DirStart,m,ID=0;
	uint32	t_NextRootFileClus = 0;
	uint8	j;
	uint16	SD_temp;
	uint8	SD_ZeroBuffer[512];

	for(SD_temp=0;SD_temp<512;SD_temp++)
	{
		SD_ZeroBuffer[SD_temp] = 0x00;	
	}
	DirStart	= DirStartSec();

	for(i=0;i<8;i++)
	{
		ReadFatBlock(DirStart+i);
		for(m=0;m<16;m++)
		{
			if(BUFFER[m*32]==0)
				return ID;
			if(BUFFER[m*32]==0xe5)
				return ID;
			ID++;
			if(ID == 128)								//表明当前根目录簇无可用地址
			{
				ID = 0;
				t_NextRootFileClus = GetNextFAT();		//找出下一个为空的FAT地址
				WriteFAT(GetRootFileSec(),t_NextRootFileClus);
				WriteFAT(t_NextRootFileClus,0x0fffffff);
				//开新的ROOTFILE簇 首先将此目录清零
				DirStart=DirStartSec();					//获取新分配的簇的地址
				for(j=0;j<8;j++)
				{
					 WriteSDCardBlock(DirStart+j,SD_ZeroBuffer);		//将此cluster 清零 sector one by one.
				}
				
				return ID;
			}
		}
	}
	return ID;
}

//-------------------------------------------------------------------------
//获得和文件名对应的目录
uint8 GetFileID(uint8 Name[11],DIR * ID,uint16 * pIndex)
{
	uint32	i,DirSecCut,DirStart,m;
	
	DirSecCut	= 128;									//本系统允许最多1024个目录项扇区			
	DirStart	= DirStartSec();
	for(i=0,*pIndex=0;i<DirSecCut;i++)
	{
		ReadFatBlock(DirStart+i);
		for(m=0;m<16;m++){
			if(IsEqual(Name,(uint8 *)&((DIR*)&BUFFER[m*32])->FileName,11))
			{
				*ID = *((DIR*)&BUFFER[m*32]);
				ID->FileFatClusHI	= (BUFFER[m*32+21]<<8) + BUFFER[m*32+20];
				ID->FilePosit.Start	= (BUFFER[m*32+27]<<8) + BUFFER[m*32+26];
				ID->FilePosit.Size	= (BUFFER[m*32+31]<<24) + (BUFFER[m*32+30]<<16) + (BUFFER[m*32+29]<<8) + BUFFER[m*32+28];
  				return 1; 						//找到对应的目录项,返回1.
			}
			(*pIndex)++;
		}
	}
	return 0; 									//没有找到对应的目录项,返回0.
}

//优化寻找下一个空FAT表--对于一个扇区中的FAT表值读一次，原始的方法是每次寻找空FAT都对FAT表进行一次读操作费时
uint32 GetNextFAT(void)		   
{
	uint32		FAT_Count,i;
	uint32		SecIDNextFat;
	uint32 *	RAMNextFat	= (uint32*)BUFFER;

	FAT_Count	= BPB_FATsz32*128; 						//FAT表总项数
	i	= TheLastFAT + 1;	
	SecIDNextFat	= BPB_RsvdSecCnt+i/128;
	ReadFatBlock(SecIDNextFat);
	for(i=TheLastFAT+1;i<FAT_Count;i++)
	{
	   if(i%128 != 0)
	   {  
	      if(RAMNextFat[i%128]==0)
		  {
		      TheLastFAT = i;		//记住最后一次的空FAT，减少遍历次数，提高系统速度
		      return i;		  
		  }	   
	   }
	   else
	   {
	   	  SecIDNextFat++;
		  ReadFatBlock(SecIDNextFat);
		  if(RAMNextFat[i%128]==0)
		  {
		      TheLastFAT = i;		//记住最后一次的空FAT，减少遍历次数，提高系统速度
		      return i;		  
		  }
	   }
	}
	return 0;    //FAT表已满，无空FAT
}

uint32 GetRootFileSec(void)
{
	uint32	t_RootFileSec;
//	uint32	FAT_Count;

	t_RootFileSec	= 0x02;								//默认条件下根目录的簇开始地址为0x22;
//	FAT_Count		= BPB_FATsz32*128; 					//FAT表总项数

	while(ReadFAT(t_RootFileSec) != 0x0FFFFFFF)
	{
		t_RootFileSec	= ReadFAT(t_RootFileSec);
	}

	return t_RootFileSec;			
}

//-------------------------------------------------------------------------
void ReadDIR(uint16  Index, DIR* Value)			//读取根目录的指定项
{	
	uint32  LBA = DirStartSec()+Index/16;			//每个扇区有16个目录项
	ReadFatBlock(LBA);
	CopyBytes((uint8 *)&BUFFER[(Index%16)*32],(uint8 *)Value,32);
	Value->FileFatClusHI	= (BUFFER[(Index%16)*32+21]<<8) + BUFFER[(Index%16)*32+20];
	Value->FilePosit.Start	= (BUFFER[(Index%16)*32+27]<<8) + BUFFER[(Index%16)*32+26];
	Value->FilePosit.Size	= (BUFFER[(Index%16)*32+31]<<24) + (BUFFER[(Index%16)*32+30]<<16) + (BUFFER[(Index%16)*32+29]<<8) + BUFFER[(Index%16)*32+28];
}

//-------------------------------------------------------------------------
void WriteDIR(uint16 Index, DIR * Value)			//写根目录的指定项
{
	uint32	LBA	= DirStartSec()+Index/16;
	ReadFatBlock(LBA);
	CopyBytes((uint8 *)Value,(uint8 *)&BUFFER[(Index%16)*32],32);
	BUFFER[(Index%16)*32+20]	= Value->FileFatClusHI&0x00ff;
	BUFFER[(Index%16)*32+21]	= (Value->FileFatClusHI&0xff00)>>8;
	BUFFER[(Index%16)*32+26]	= Value->FilePosit.Start&0x00ff;
	BUFFER[(Index%16)*32+27]	= (Value->FilePosit.Start&0xff00)>>8;
	BUFFER[(Index%16)*32+28]	= Value->FilePosit.Size&0x000000ff;
	BUFFER[(Index%16)*32+29]	= (Value->FilePosit.Size&0x0000ff00)>>8;
	BUFFER[(Index%16)*32+30]	= (Value->FilePosit.Size&0x00ff0000)>>16;
	BUFFER[(Index%16)*32+31]	= (Value->FilePosit.Size&0xff000000)>>24;
	WriteFatBlock(LBA);
}

//-------------------------------------------------------------------------
void CopyFAT(void)						//复制文件分配表,使其和备份一致
{
	uint32  i;

	for(i=0;i<BPB_FATsz32;i++)
	{
		ReadFatBlock(BPB_RsvdSecCnt+i);
		WriteFatBlock(BPB_RsvdSecCnt+BPB_FATsz32+i);
	}
}

//-------------------------------------------------------------------------
uint8 CreateFile(uint8 * Name, uint32 Size)	//创建一个空文件
{
	uint32	ClusID, ClusNum, ClusNext, i;
	uint16	dirIndex;
	DIR		FileDir;
	
	if(GetFileID(Name,&FileDir,&dirIndex)==1)	 		
		return SD_FAIL;	//文件已存在
	
	ClusNum	= Size/(BPB_SecPerClus*BPB_BytesPerSec)+1;
	
	EmptyBytes((uint8 *)&FileDir,sizeof(DIR));
	CopyBytes(Name,(uint8 *)&FileDir.FileName,11);
	
	ClusID	= GetNextFAT();
	FileDir.FilePosit.Size	= Size;
	FileDir.FileFatClusHI	= ClusID>>16;
	FileDir.FilePosit.Start	= ClusID;
	
	for(i=0;i<ClusNum-1;i++)
	{
		ClusNext=GetNextFAT();
		WriteFAT(ClusID,ClusNext);
		ClusID=ClusNext;
	}
	WriteFAT(ClusID, 0x0fffffff);
	
	WriteDIR(GetEmptyDIR(),&FileDir);
	
	return SD_SUCC;
}
//-------------------------------------------------------------------------
//读文件
uint8 ReadFile(uint8 Name[11], uint32 Start, uint32 Len,uint8 * p)
{
	uint32	BytePerClus,ClusID,m;
	uint16	dirIndex;
	uint32	LBA;
	uint8	i;
	DIR		FileDir;
	
	if(GetFileID(Name,&FileDir,&dirIndex)==0)
		return SD_FAIL;//文件不存在
	
	BytePerClus	= BPB_SecPerClus*BPB_BytesPerSec;		//每簇的字节数	
	m	= Start/BytePerClus;							//计算开始位置包含的簇数
	ClusID		= FileDir.FilePosit.Start + (FileDir.FileFatClusHI<<16);	//文件的开始簇号
	for(i=0;i<m;i++)					//计算开始位置所在簇的簇号
		ClusID	= ReadFAT(ClusID);				
	i	= (Start%BytePerClus)/BPB_BytesPerSec;			//计算开始位置所在扇区的簇内偏移
	LBA	= ClusConvLBA(ClusID)+i;						//计算开始位置的逻辑扇区号
	m	= (Start%BytePerClus)%BPB_BytesPerSec;			//计算开始位置在扇区内偏移

	while (1)
	{
		for(;i<BPB_SecPerClus;i++)
		{
			ReadFatBlock(LBA++);
			for(;m<BPB_BytesPerSec;m++)
			{
				*p++=BUFFER[m];
				if(--Len == 0)
					return SD_SUCC;			//如果读取完成就退出
			}
			m=0;
		}
		i=0;
		ClusID=ReadFAT(ClusID);							//下一簇簇号
		LBA=ClusConvLBA(ClusID);
	}
}
//-------------------------------------------------------------------------
//写文件
uint8 WriteFile(uint8 Name[11], uint32 Start, uint32 Len, uint8 * p)
{
	uint16	BytePerClus,ClusID,m,dirIndex;
	uint32	LBA;
	uint8	i;
	DIR		FileDir;
	
	if(GetFileID(Name,&FileDir,&dirIndex)==0)		//文件不存在
		return SD_FAIL;
	
	BytePerClus	= BPB_SecPerClus*BPB_BytesPerSec;		// 每簇的字节数	
	m	= Start/BytePerClus;					        //计算开始位置包含的簇数
	ClusID	= FileDir.FilePosit.Start + (FileDir.FileFatClusHI<<16);	//文件的开始簇号
	for(i=0;i<m;i++)				  				//计算开始位置所在簇的簇号
		ClusID=ReadFAT(ClusID);		
			
	i	= (Start%BytePerClus)/BPB_BytesPerSec;			//计算开始位置所在扇区的簇内偏移
	LBA	= ClusConvLBA(ClusID)+i;				        //计算开始位置的逻辑扇区号
	m	= (Start%BytePerClus)%BPB_BytesPerSec;			//计算开始位置在扇区内偏移

	while (1)
	{
		for(;i<BPB_SecPerClus;i++)
		{
			ReadFatBlock(LBA);
			for(;m<BPB_BytesPerSec;m++)
			{
				BUFFER[m]=*p++;
				if(--Len==0)
				{							            //如果读取完成就退出
					WriteFatBlock(LBA);					//回写扇区
					return SD_SUCC;				
				}
			}
			m=0;
			WriteFatBlock(LBA++);						//回写扇区
		}
		i=0;
		ClusID=ReadFAT(ClusID);							//下一簇簇号
		LBA=ClusConvLBA(ClusID);
	}
}
//-------------------------------------------------------------------------
uint8 InitFat32(void)							//初始化FAT16的变量
{
	unsigned char i;
	
	i=ReadMBR();
	if(i==SD_FAIL)return SD_FAIL;
	
	i=ReadBPB();	
	if(i==SD_FAIL)return SD_FAIL;

	return SD_SUCC;
	
}

//-------------------------------------------------------------------------
//删除文件
uint8 EreaseFile(uint8 Name[11])
{
	uint32	ClusID,ClusNext;
	uint16	dirIndex;
	DIR		FileDir;
	
	if(GetFileID(Name,&FileDir,&dirIndex)==0)		//文件不存在
		return SD_FAIL;

	ClusID	= FileDir.FilePosit.Start + (FileDir.FileFatClusHI<<16);	//文件的开始簇号
	
	while (1)
	{
		if((ClusNext=ReadFAT(ClusID))!=0x0fffffff)	//删除FAT表中的链表
		{		
			WriteFAT(ClusID,0x00000000);
			ClusID=ClusNext;
		}
		else
		{
			WriteFAT(ClusID,0x00000000);
			break;
		}
	}
	
	FileDir.FileName.NAME[0]=0xe5;					//删除Dir中的文件记录
	WriteDIR(dirIndex,&FileDir);
	CopyFAT();										//FAT2<-FAT1
	return SD_SUCC;
}
