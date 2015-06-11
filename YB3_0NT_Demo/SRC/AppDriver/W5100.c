/********************************************************************************
    本软件包括5个部分：
    	1. W5100初始化
    	2. W5100的Socket初始化
    	3. Socket连接
    	   如果Socket设置为TCP服务器模式，则调用Socket_Listen()函数,W5100处于侦听状态，直到远程客户端与它连接。
    	   如果Socket设置为TCP客户端模式，则调用Socket_Connect()函数，
    	                                  每调用一次Socket_Connect(s)函数，产生一次连接，
    	                                  如果连接不成功，则产生超时中断，然后可以再调用该函数进行连接。
    	   如果Socket设置为UDP模式,则调用Socket_UDP函数
    	4. Socket数据接收和发送
    	5. W5100中断处理

    置W5100为服务器模式的调用过程：W5100_Init()-->Socket_Init(s)-->Socket_Listen(s)，设置过程即完成，等待客户端的连接。
    置W5100为客户端模式的调用过程：W5100_Init()-->Socket_Init(s)-->Socket_Connect(s)，设置过程即完成，并与远程服务器连接。
    置W5100为UDP模式的调用过程：W5100_Init()-->Socket_Init(s)-->Socket_UDP(s)，设置过程即完成，可以与远程主机UDP通信。

    W5100产生的连接成功、终止连接、接收数据、发送数据、超时等事件，都可以从中断状态中获得。
********************************************************************************/
#define	__W5100_C
#include "W5100.h"
#include "WT_Task.h"
uint32	DARec;

uint32	u32Interrupt_Num = 0;         //记录W5100中断，连续的中断次数	

// 延时1ms
void W5100_Delay_1MS(uint32 m)
{
	uint32 i;

	for(; m != 0; m--)	
 		for (i=0; i<8300; i++);
}

/*****************************************************************
程序名：Read_W5100
输入: 地址
输出: 无
返回: 读取的数据
说明：从W5100指定的地址读取一个字节
*****************************************************************/
uint8 Read_W5100(uint32 addr)
{
	unsigned char i;

	volatile unsigned char *ip;
	
	ip = (volatile unsigned char *)addr;
	i = *ip;
	
	return i;
}

/*****************************************************************
程序名：Write_W5100
输入: 地址，字节数据
输出: 无
返回: 无
说明：将一个字节写入W5100指定的地址
*****************************************************************/
void Write_W5100(uint32 addr, uint8 dat)
{
	volatile unsigned char *ip;

	ip = (volatile unsigned char *)addr;
	*ip = dat;
}

/*------------------------------------------------------------------------------
						W5100初始化函数
在使用W5100之前，对W5100初始化
------------------------------------------------------------------------------*/
void W5100_Init(void)
{
	unsigned char i;

	Write_W5100(W5100_MODE,MODE_RST);		/*软复位W5100*/

	W5100_Delay_1MS(100);						/*延时100ms，自己定义该函数*/

	/*设置网关(Gateway)的IP地址，4字节 */
	/*使用网关可以使通信突破子网的局限，通过网关可以访问到其它子网或进入Internet*/
	for(i=0;i<4;i++)
		Write_W5100(W5100_GAR+i,Gateway_IP[i]);			/*Gateway_IP为4字节unsigned char数组,自己定义*/

	/*设置子网掩码(MASK)值，4字节。子网掩码用于子网运算*/
	for(i=0;i<4;i++)
		Write_W5100(W5100_SUBR+i,Sub_Mask[i]);			/*SUB_MASK为4字节unsigned char数组,自己定义*/

	/*设置物理地址，6字节，用于唯一标识网络设备的物理地址值
	该地址值需要到IEEE申请，按照OUI的规定，前3个字节为厂商代码，后三个字节为产品序号
	如果自己定义物理地址，注意第一个字节必须为偶数*/
	
	for(i=0;i<6;i++)
		Write_W5100(W5100_SHAR+i,Phy_Addr[i]);			/*PHY_ADDR6字节unsigned char数组,自己定义*/

	/*设置本机的IP地址，4个字节
	注意，网关IP必须与本机IP属于同一个子网，否则本机将无法找到网关*/
	for(i=0;i<4;i++)
		Write_W5100(W5100_SIPR+i,IP_Addr[i]);			/*IP_ADDR为4字节unsigned char数组,自己定义*/

	/*设置发送缓冲区和接收缓冲区的大小，参考W5100数据手册*/
 	Write_W5100(W5100_RMSR,0x55);		/*Socket Rx memory size=2k*/

//	Write_W5100(W5100_RMSR,0xAA);		/*Socket Rx memory size=2k*/
	Write_W5100(W5100_TMSR,0x55);		/*Socket Tx mempry size=2k*/

	/* 设置重试时间，默认为2000(200ms) *///当设置为200ms时，实测为190~200ms之间，故将其适当扩大，设为250ms by wqf 20100930
	Write_W5100(W5100_RTR,0x09);
	Write_W5100(W5100_RTR+1,0xC4);

	/* 设置重试次数，默认为8次 */
	Write_W5100(W5100_RCR,8);

	/* 启动中断，参考W5100数据手册确定自己需要的中断类型
	IMR_CONFLICT是IP地址冲突异常中断
	IMR_UNREACH是UDP通信时，地址无法到达的异常中断
	其它是Socket事件中断，根据需要添加 */
	Write_W5100(W5100_IMR,(IMR_CONFLICT|IMR_UNREACH|IMR_S0_INT|IMR_S1_INT|IMR_S2_INT|IMR_S3_INT));
}
/****************************************************************************
                            Detect Gateway
input:  	None
Output: 	None
Return: 	if fail to detect gateway, return FALSE
		if detect the gateway, return TRUE
****************************************************************************/
uint8 Detect_Gateway(void)
{
	unsigned char i;

	Write_W5100((W5100_S0_MR),S_MR_TCP);		/*设置socket0为TCP模式*/

	Write_W5100((W5100_S0_CR),S_CR_OPEN);		/*打开socket0*/

	if(Read_W5100(W5100_S0_SSR)!=S_SSR_INIT)
	{
		Write_W5100((W5100_S0_CR),S_CR_CLOSE);	/*打开不成功，关闭Socket，然后返回*/
		return FALSE;
	}

	/*检查网关及获取网关的物理地址*/
	for(i=0;i<4;i++)
		Write_W5100((W5100_S0_DIPR+i),IP_Addr[i]+1);	/*向目的地址寄存器写入与本机IP不同的IP值*/

	Write_W5100((W5100_S0_CR),S_CR_CONNECT);		/*打开socket0的TCP连接*/

	W5100_Delay_1MS(20);						/* 延时20ms */

	i=Read_W5100(W5100_S0_DHAR);			/*读取目的主机的物理地址，该地址就是网关地址*/

	Write_W5100((W5100_S0_CR),S_CR_CLOSE);			/*关闭socket0*/

	if(i==0xff)
	{
		/**********没有找到网关服务器，或没有与网关服务器成功连接***********/
		/**********              自己添加处理代码                ***********/
		return FALSE;
	}
	return TRUE;
}

/******************************************************************************
                           Socket处理, 其它3个Socket的处理可参照此程序
*****************************************************************************

						Socket初始化
如果成功则返回true, 否则返回false
-----------------------------------------------------------------------------*/
void Socket_Init(SOCKET s)
{
	unsigned char i;

	/*设置分片长度，参考W5100数据手册，该值可以不修改*/
	Write_W5100((W5100_S0_MSS+s*0x100),0x05);		/*最大分片字节数=1460*/
	Write_W5100((W5100_S0_MSS+s*0x100+1),0xb4);

	/* Set Socket Port number */
	switch(s)
	{
		case 0:
			Write_W5100(W5100_S0_PORT,S0_Port[0]);	/* Set Local Socket Port number */
			Write_W5100(W5100_S0_PORT+1,S0_Port[1]);

			Write_W5100(W5100_S0_DPORT,S0_DPort[0]);	/* Set Destination port number */
			Write_W5100(W5100_S0_DPORT+1,S0_DPort[1]);

			for(i=0;i<4;i++)
				Write_W5100(W5100_S0_DIPR+i,S0_DIP[i]);	/* Set Destination IP Address */
			break;
		case 1:
			Write_W5100(W5100_S1_PORT,S1_Port[0]);	/* Set Local Socket Port number */
			Write_W5100(W5100_S1_PORT+1,S1_Port[1]);

			Write_W5100(W5100_S1_DPORT,S1_DPort[0]);	/* Set Destination port number */
			Write_W5100(W5100_S1_DPORT+1,S1_DPort[1]);

			for(i=0;i<4;i++)
				Write_W5100(W5100_S1_DIPR+i,S1_DIP[i]);	/* Set Destination IP Address */
			break;
		case 2:
			Write_W5100(W5100_S2_PORT,S2_Port[0]);	/* Set Local Socket Port number */
			Write_W5100(W5100_S2_PORT+1,S2_Port[1]);

			Write_W5100(W5100_S2_DPORT,S2_DPort[0]);	/* Set Destination port number */
			Write_W5100(W5100_S2_DPORT+1,S2_DPort[1]);

			for(i=0;i<4;i++)
				Write_W5100(W5100_S2_DIPR+i,S2_DIP[i]);	/* Set Destination IP Address */
			break;
		case 3:
			Write_W5100(W5100_S3_PORT,S3_Port[0]);	/* Set Local Socket Port number */
			Write_W5100(W5100_S3_PORT+1,S3_Port[1]);

			Write_W5100(W5100_S3_DPORT,S3_DPort[0]);	/* Set Destination port number */
			Write_W5100(W5100_S3_DPORT+1,S3_DPort[1]);

			for(i=0;i<4;i++)
				Write_W5100(W5100_S3_DIPR+i,S3_DIP[i]);	/* Set Destination IP Address */
			break;
		default:
			break;
	}
}
/*-----------------------------------------------------------------------------
                           设置Socket为客户端与远程服务器连接
当本机Socket工作在客户端模式时，引用该程序，与远程服务器建立连接
如果设置成功则返回true，否则返回false
如果启动连接后出现超时中断，则与服务器连接失败，需要重新调用该程序连接
该程序每调用一次，就与服务器产生一次连接
------------------------------------------------------------------------------*/
uint8 Socket_Connect(SOCKET s)
{
	Write_W5100((W5100_S0_MR+s*0x100), S_MR_TCP);		/*设置socket为TCP模式 */
	Write_W5100((W5100_S0_CR+s*0x100), S_CR_OPEN);		/*打开Socket*/

	if(Read_W5100(W5100_S0_SSR+s*0x100)!=S_SSR_INIT)
	{
		Write_W5100(W5100_S0_CR+s*0x100,S_CR_CLOSE);	/*打开不成功，关闭Socket，然后返回*/
		return FALSE;
	}

	Write_W5100((W5100_S0_CR+s*0x100),S_CR_CONNECT);		/*设置Socket为Connect模式*/

	return TRUE;

	/*至此完成了Socket的打开连接工作，至于它是否与远程服务器建立连接，则需要等待Socket中断，
	以判断Socket的连接是否成功。参考W5100数据手册的Socket中断状态*/
}

/*-----------------------------------------------------------------------------
                           设置Socket作为服务器等待远程主机的连接
当本机Socket工作在服务器模式时，引用该程序，等等远程主机的连接
如果设置成功则返回true, 否则返回false
该程序只调用一次，就使W5100设置为服务器模式
-----------------------------------------------------------------------------*/
uint8 Socket_Listen(SOCKET s)
{
	Write_W5100((W5100_S0_MR+s*0x100), S_MR_TCP);		/*设置socket为TCP模式 */
	Write_W5100((W5100_S0_CR+s*0x100), S_CR_OPEN);		/*打开Socket*/

	if(Read_W5100(W5100_S0_SSR+s*0x100)!=S_SSR_INIT)
	{
		Write_W5100((W5100_S0_CR+s*0x100),S_CR_CLOSE);	/*打开不成功，关闭Socket，然后返回*/
		return FALSE;
	}

	Write_W5100((W5100_S0_CR+s*0x100), S_CR_LISTEN);		/*设置Socket为侦听模式*/

	if(Read_W5100(W5100_S0_SSR+s*0x100)!=S_SSR_LISTEN)
	{
		Write_W5100((W5100_S0_CR+s*0x100), S_CR_CLOSE);		/*设置不成功，关闭Socket，然后返回*/
		return FALSE;
	}

	return TRUE;

	/*至此完成了Socket的打开和设置侦听工作，至于远程客户端是否与它建立连接，则需要等待Socket中断，
	以判断Socket的连接是否成功。参考W5100数据手册的Socket中断状态
	在服务器侦听模式不需要设置目的IP和目的端口号*/
}

/*-----------------------------------------------------------------------------
					设置Socket为UDP模式
如果Socket工作在UDP模式，引用该程序。在UDP模式下，Socket通信不需要建立连接
如果设置成功则返回true, 否则返回false
该程序只调用一次，就使W5100设置为UDP模式
-----------------------------------------------------------------------------*/
uint8 Socket_UDP(SOCKET s)
{
	Write_W5100((W5100_S0_MR+s*0x100), S_MR_UDP);		/*设置Socket为UDP模式*/
	Write_W5100((W5100_S0_CR+s*0x100), S_CR_OPEN);		/*打开Socket*/

	if(Read_W5100(W5100_S0_SSR+s*0x100)!=S_SSR_UDP)
	{
		Write_W5100((W5100_S0_CR+s*0x100), S_CR_CLOSE);	/*打开不成功，关闭Socket，然后返回*/
		return FALSE;
	}
	else
		return TRUE;

	/*至此完成了Socket的打开和UDP模式设置，在这种模式下它不需要与远程主机建立连接
	因为Socket不需要建立连接，所以在发送数据前都可以设置目的主机IP和目的Socket的端口号
	如果目的主机IP和目的Socket的端口号是固定的，在运行过程中没有改变，那么也可以在这里设置*/
}


/******************************************************************************
                              处理Socket接收和发送的数据
******************************************************************************/
/*-----------------------------------------------------------------------------
如果Socket产生接收数据的中断，则引用该程序进行处理
该程序将Socket的接收到的数据缓存到Rx_buffer数组中，并返回接收的数据字节数
-----------------------------------------------------------------------------*/
uint16 S_rx_process(SOCKET s)
{
	unsigned int i,j;
	unsigned short rx_size,rx_offset;



	/*读取接收数据的字节数*/
	rx_size=Read_W5100(W5100_S0_RX_RSR+s*0x100);
	rx_size*=256;
	rx_size+=Read_W5100(W5100_S0_RX_RSR+s*0x100+1);

	/*读取接收缓冲区的偏移量*/
	rx_offset=Read_W5100(W5100_S0_RX_RR+s*0x100);
	rx_offset*=256;
	rx_offset+=Read_W5100(W5100_S0_RX_RR+s*0x100+1);

	i=rx_offset/S_RX_SIZE;				/*计算实际的物理偏移量，S0_RX_SIZE需要在前面#define中定义*/
								/*注意S_RX_SIZE的值在W5100_Init()函数的W5100_RMSR中确定*/
	rx_offset=rx_offset-i*S_RX_SIZE;

	j=W5100_RX+s*S_RX_SIZE+rx_offset;		/*实际物理地址为W5100_RX+rx_offset*/
	for(i=0;i<rx_size;i++)
	{
		if(rx_offset>=S_RX_SIZE)
		{
			j=W5100_RX+s*S_RX_SIZE;
			rx_offset=0;
		}
		Rx_Buffer[i+s*Max_Size]=Read_W5100(j);		/*将数据缓存到Rx_buffer数组中*/
//		Rx_Buffer[DARec+i]=Read_W5100(j);		 //hong
		j++;
		rx_offset++;
	}
		
//		DARec=DARec+rx_size;


	/*计算下一次偏移量*/
	rx_offset=Read_W5100(W5100_S0_RX_RR+s*0x100);
	rx_offset*=256;
	rx_offset+=Read_W5100(W5100_S0_RX_RR+s*0x100+1);

	rx_offset+=rx_size;
	Write_W5100((W5100_S0_RX_RR+s*0x100), (rx_offset/256));
	Write_W5100((W5100_S0_RX_RR+s*0x100+1), rx_offset);

	Write_W5100((W5100_S0_CR+s*0x100), S_CR_RECV);			/*设置RECV命令，等等下一次接收*/

	return rx_size;								/*返回接收的数据字节数*/
}

/*-----------------------------------------------------------------------------
如果要通过Socket发送数据，则引用该程序
要发送的数据缓存在Tx_buffer中, size则是要发送的字节长度
-----------------------------------------------------------------------------*/
uint8 S_tx_process(SOCKET s,uint8 *buf, uint32 size)
{
	unsigned int i,j;
	unsigned short tx_free_size,tx_offset;

	/*读取缓冲区剩余的长度*/
	tx_free_size=Read_W5100(W5100_S0_TX_FSR+s*0x100);
	tx_free_size*=256;
	tx_free_size+=Read_W5100(W5100_S0_TX_FSR+s*0x100+1);
	if(tx_free_size<size)						/*如果剩余的字节长度小于发送字节长度,则返回*/
		return FALSE;

	/*读取发送缓冲区的偏移量*/
	tx_offset=Read_W5100(W5100_S0_TX_WR+s*0x100);
	tx_offset*=256;
	tx_offset+=Read_W5100(W5100_S0_TX_WR+s*0x100+1);

	i=tx_offset/S_TX_SIZE;					/*计算实际的物理偏移量，S0_TX_SIZE需要在前面#define中定义*/
									/*注意S0_TX_SIZE的值在W5100_Init()函数的W5100_TMSR中确定*/
	tx_offset=tx_offset-i*S_TX_SIZE;
	j=W5100_TX+s*S_TX_SIZE+tx_offset;		/*实际物理地址为W5100_TX+tx_offset*/

	for(i=0;i<size;i++)
	{
		if(tx_offset>=S_TX_SIZE)
		{
			j=W5100_TX+s*S_TX_SIZE;
			tx_offset=0;
		}
		Write_W5100(j,buf[i]);						/*将Tx_buffer缓冲区中的数据写入到发送缓冲区*/
		j++;
		tx_offset++;
	}

	/*计算下一次的偏移量*/
	tx_offset=Read_W5100(W5100_S0_TX_WR+s*0x100);
	tx_offset*=256;
	tx_offset+=Read_W5100(W5100_S0_TX_WR+s*0x100+1);


	tx_offset+=size;
	Write_W5100((W5100_S0_TX_WR+s*0x100),(tx_offset/256));
	Write_W5100((W5100_S0_TX_WR+s*0x100+1),tx_offset);

	Write_W5100((W5100_S0_CR+s*0x100), S_CR_SEND);			/*设置SEND命令,启动发送*/

	return TRUE;								/*返回成功*/
}


/******************************************************************************
					W5100中断处理程序框架
******************************************************************************/
void W5100_Interrupt_Process(void)
{
	unsigned char i,j;

	W5100_Interrupt=0;

	i=Read_W5100(W5100_IR);
LOOP:
	Write_W5100(W5100_IR, (i&0xf0));					/*回写清除中断标志*/

	
	if((i & IR_CONFLICT) == IR_CONFLICT)	 	/*IP地址冲突异常处理，自己添加代码*/
	{

	}

	if((i & IR_UNREACH) == IR_UNREACH)			/*UDP模式下地址无法到达异常处理，自己添加代码*/

	{
	}

	/* Socket事件处理 */
	if((i & IR_S0_INT) == IR_S0_INT)
	{
		j=Read_W5100(W5100_S0_IR);
		Write_W5100(W5100_S0_IR, j);		/* 回写清中断标志 */

		if(j&S_IR_CON)				/* 在TCP模式下,Socket0成功连接 */
		{
			S0_State|=S_CONN;
		}
		if(j&S_IR_DISCON)				/* 在TCP模式下Socket断开连接处理，自己添加代码 */
		{
			Write_W5100(W5100_S0_CR, S_CR_CLOSE);		/* 关闭端口，等待重新打开连接 */   
			S0_State=0;
		}
		if(j&S_IR_SENDOK)				/* Socket0数据发送完成，可以再次启动S_tx_process()函数发送数据 */
		{
			S0_Data|=S_TRANSMITOK;
		}
		if(j&S_IR_RECV)				/* Socket接收到数据，可以启动S_rx_process()函数 */
		{
			S0_Data|=S_RECEIVE;
			OSSemPost(g_JG0flag);	   //发送信号量
		}
		if(j&S_IR_TIMEOUT)			/* Socket连接或数据传输超时处理 */
		{
			Write_W5100(W5100_S0_CR, S_CR_CLOSE);		/* 关闭端口，等待重新打开连接 */
			S0_State=0;
		}
	}

	/* Socket1事件处理 */
	if((i&IR_S1_INT)==IR_S1_INT)
	{
		j=Read_W5100(W5100_S1_IR);
		Write_W5100(W5100_S1_IR, j);		/* 回写清中断标志 */

		if(j&S_IR_CON)				/* 在TCP模式下,Socket1成功连接 */
		{
			S1_State|=S_CONN;
		}
		if(j&S_IR_DISCON)		/* 在TCP模式下Socket1断开连接处理，自己添加代码 */
		{
			Write_W5100(W5100_S1_CR, S_CR_CLOSE);		/* 关闭端口，等待重新打开连接 */
			S1_State=0;
		}
		if(j&S_IR_SENDOK)		/* Socket1数据发送完成，可以再次启动S_tx_process()函数发送数据 */
		{
			S1_Data|=S_TRANSMITOK;
		}
		if(j&S_IR_RECV)	  	/* Socket1接收到数据，可以启动S_rx_process()函数 */
		{
			S1_Data|=S_RECEIVE;
			OSSemPost(g_JG1flag);  //发送信号量
		}
		if(j&S_IR_TIMEOUT)	/* Socket1连接或数据传输超时处理 */
		{
			Write_W5100(W5100_S1_CR, S_CR_CLOSE);		/*关闭端口，等待重新打开连接 */
			S1_State=0;
		}
	}

	/* Socket2事件处理 */
	if((i&IR_S2_INT)==IR_S2_INT)
	{
		j=Read_W5100(W5100_S2_IR);
		Write_W5100(W5100_S2_IR, j);		/*回写清中断标志 */

		if(j&S_IR_CON)		/* 在TCP模式下,Socket2成功连接 */
		{
			S2_State|=S_CONN;
		}
		if(j&S_IR_DISCON)		/* 在TCP模式下Socket2断开连接处理，自己添加代码 */
		{
			Write_W5100(W5100_S2_CR, S_CR_CLOSE);		/* 关闭端口，等待重新打开连接 */
			S2_State=0;
		}
		if(j&S_IR_SENDOK)		/* Socket2数据发送完成，可以再次启动S_tx_process()函数发送数据 */
		{
			S2_Data|=S_TRANSMITOK;
		}
		if(j&S_IR_RECV)		/* Socket2接收到数据，可以启动S_rx_process()函数 */
		{
			S2_Data|=S_RECEIVE;
			OSSemPost(g_JG2flag);	   //发送信号量
		}
		if(j&S_IR_TIMEOUT)	/* Socket2连接或数据传输超时处理 */
		{
			Write_W5100(W5100_S2_CR, S_CR_CLOSE);		/*关闭端口，等待重新打开连接 */
			S2_State=0;
		}
	}

	/* Socket3事件处理 */
	if((i&IR_S3_INT)==IR_S3_INT)
	{
		j=Read_W5100(W5100_S3_IR);
		Write_W5100(W5100_S3_IR, j);		/* 回写清中断标志 */

		if(j&S_IR_CON)		/* 在TCP模式下,Socket3成功连接 */
		{
			S3_State|=S_CONN;
		}
		if(j&S_IR_DISCON)		/* 在TCP模式下Socket3断开连接处理，自己添加代码 */
		{
			Write_W5100(W5100_S3_CR, S_CR_CLOSE);		/* 关闭端口，等待重新打开连接 */
//			Write_W5100(W5100_S3_CR, S_CR_LISTEN);	  			
			S3_State=0;
		}
		if(j&S_IR_SENDOK)				/* Socket3数据发送完成，可以再次启动S_tx_process()函数发送数据 */
		{
			S3_Data|=S_TRANSMITOK;
		}
		if(j&S_IR_RECV)		/* Socket3接收到数据，可以启动S_rx_process()函数 */
		{
			S3_Data|=S_RECEIVE;
//			g_UART1Cnt = S_rx_process(3);			
		    OSSemPost(g_JG3flag);
		}
		if(j&S_IR_TIMEOUT)	/* Socket3连接或数据传输超时处理 */
		{
			Write_W5100(W5100_S3_CR, S_CR_CLOSE);		/*关闭端口，等待重新打开连接 */
			S3_State=0;
		}
	}

	i=Read_W5100(W5100_IR);
	if(i)
	{
		u32Interrupt_Num++;
		goto LOOP;
	}

}
