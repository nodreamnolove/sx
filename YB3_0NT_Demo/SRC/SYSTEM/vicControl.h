/****************************************Copyright (c)****************************************************
**                            Guangzhou ZHIYUAN electronics Co.,LTD.
**                                      
**                                 http://www.embedtools.com
**
**--------------File Info---------------------------------------------------------------------------------
** File name:               vicControl.h
** Latest modified Date:    2008-12-12
** Latest Version:          1.0
** Descriptions:            VIC控制代码
**
**--------------------------------------------------------------------------------------------------------
** Created by:              LinEnqiang
** Created date:            2008-12-12
** Version:                 1.0
** Descriptions:            The original version
**
**--------------------------------------------------------------------------------------------------------
** Modified by:             
** Modified date:           
** Version:                 
** Descriptions:            
**
*********************************************************************************************************/

#ifndef _VIC_CONTROL_H_
#define _VIC_CONTROL_H_

#ifdef __cplusplus
    extern "C" {
#endif
                                                                  /*  __cplusplus                 */
/*********************************************************************************************************
  中断触发类型值 宏定义
*********************************************************************************************************/
#define INT_LEVEL_TYPE_LOW      0x02                                    /*  低电平触发中断模式          */
#define INT_LEVEL_TYPE_HIGH     0x03                                    /*  高电平触发中断模式          */

#define INT_EDGE_TYPE_DOWN      0x00                                    /*  下降沿触发中断模式          */
#define INT_EDGE_TYPE_UP        0x01                                    /*  上升沿触发中断模式          */


/*********************************************************************************************************
  IRQ、FIQ 中断 全局开关 管理
*********************************************************************************************************/

/*********************************************************************************************************
** Function name:           swiHandle1
** Descriptions:            SWI函数声明
** input parameters:        iHandle: 用于区分功能                          
** output parameters:       根据功能决定
** Returned value:          根据功能决定
*********************************************************************************************************/
__swi(0x00) void swiHandle1(int iHandle);

/*********************************************************************************************************
** Function name:           IRQDisable
** Descriptions:            关闭全局 IRQ 中断
** input parameters:        none
** output parameters:       none
** Returned value:          none                          
*********************************************************************************************************/
__inline unsigned int IRQDisable() 
{
	swiHandle1(1);		   
	return 0;
}
/*********************************************************************************************************
** Function name:           IRQEnable
** Descriptions:            打开全局 IRQ 中断
** input parameters:        none
** output parameters:       none
** Returned value:          none                          
*********************************************************************************************************/
__inline unsigned int IRQEnable() 
{
	swiHandle1(12); 
	return 0;
}

/*********************************************************************************************************
  通用外设 IRQ、FIQ 中断管理
*********************************************************************************************************/
/*********************************************************************************************************
** Function name:           swiHandle
** Descriptions:            SWI函数声明
** input parameters:        iHandle: 用于区分功能
**                          其他:    根据功能决定
** output parameters:       根据功能决定
** Returned value:          根据功能决定
*********************************************************************************************************/
__swi(0x00) unsigned int swiHandle (int iHandle, unsigned int, unsigned int, unsigned int);

/*********************************************************************************************************
** Function name:           micIrqFuncSet
** Descriptions:            设置所选外设的中断触发类型、中断服务函数地址，并使能中断
** input parameters:        uiChannel:  外设对应的中断通道号
**                          uiType:     中断触发类型 -- 0:下降沿 1:上升沿 2:低电平 3:高电平
**                          uiFuncAddr: 中断服务函数地址
** output parameters:       none
** Returned value:          1:          成功
**                          0:          失败
*********************************************************************************************************/
__inline unsigned int micIrqFuncSet (unsigned int uiChannel,
                                     unsigned int uiType,
                                     unsigned int uiFuncAddr)
{
    return swiHandle(0x100, uiChannel, uiType, uiFuncAddr);
}

/*********************************************************************************************************
** Function name:           micIrqFuncClr
** Descriptions:            清除所选外设的IRQ资源
** input parameters:        uiChannel:  外设对应的中断通道号
** output parameters:       none
** Returned value:          1:          成功
**                          0:          失败
*********************************************************************************************************/
__inline unsigned int micIrqFuncClr (unsigned int uiChannel)
{
    return swiHandle(0x101, uiChannel ,0 ,0);
}

/*********************************************************************************************************
** Function name:           micIrqEnable
** Descriptions:            使能相应外设的中断
** input parameters:        uiChannel:  外设对应的中断通道号
** output parameters:       none
** Returned value:          1:          成功
**                          0:          失败
*********************************************************************************************************/
__inline unsigned int micIrqEnable (unsigned int uiChannel)
{
    return swiHandle(0x102, uiChannel, 0, 0);
}

/*********************************************************************************************************
** Function name:           micIrqDisable
** Descriptions:            禁止相应外设的中断
** input parameters:        uiChannel:  外设对应的中断通道号
** output parameters:       none
** Returned value:          1:          成功
**                          0:          失败
*********************************************************************************************************/
__inline unsigned int micIrqDisable (unsigned int uiChannel)
{
    return swiHandle(0x103, uiChannel, 0, 0);
}
/*********************************************************************************************************
** Function name:           micFiqSet
** Descriptions:            设置并使能所选中断通道号为FIQ中断
** input parameters:        uiChannel:  外设对应的中断通道号
**                          uiType:     中断触发类型 -- 0:下降沿 1:上升沿 2:低电平 3:高电平
** output parameters:       none
** Returned value:          1:          成功
**                          0:          失败
*********************************************************************************************************/
__inline unsigned int micFiqSet (unsigned int uiChannel, unsigned int uiType)
{
    return swiHandle(0x104, uiChannel, uiType, 0);
}

/*********************************************************************************************************
** Function name:           micFiqClr
** Descriptions:            清除所选中断通道号的FIQ中断
** input parameters:        uiChannel:  外设对应的中断通道号
** output parameters:       none
** Returned value:          1:          成功
**                          0:          失败
*********************************************************************************************************/
__inline unsigned int micFiqClr (unsigned int uiChannel)
{
    return swiHandle(0x105, uiChannel, 0, 0);
}

/*********************************************************************************************************
** Function name:           sic1IrqFuncSet
** Descriptions:            设置所选外设的中断触发类型、中断服务函数地址，并使能中断
** input parameters:        uiChannel:  外设对应的中断通道号
**                          uiType:     中断触发类型 -- 0:下降沿 1:上升沿 2:低电平 3:高电平
**                          uiFuncAddr: 中断服务函数地址
** output parameters:       none
** Returned value:          1:          成功
**                          0:          失败
*********************************************************************************************************/
__inline unsigned int sic1IrqFuncSet (unsigned int uiChannel,
                                      unsigned int uiType,
                                      unsigned int uiFuncAddr)
{
    return swiHandle(0x106, uiChannel, uiType, uiFuncAddr);
}

/*********************************************************************************************************
** Function name:           vicIrqFuncClr
** Descriptions:            清除所选外设的IRQ资源
** input parameters:        uiChannel:  外设对应的中断通道号
** output parameters:       none
** Returned value:          1:          成功
**                          0:          失败
*********************************************************************************************************/
__inline unsigned int sic1IrqFuncClr (unsigned int uiChannel)
{
    return swiHandle(0x107, uiChannel ,0 ,0);
}

/*********************************************************************************************************
** Function name:           vicIrqEnable
** Descriptions:            使能相应外设的中断
** input parameters:        uiChannel:  外设对应的中断通道号
** output parameters:       none
** Returned value:          1:          成功
**                          0:          失败
*********************************************************************************************************/
__inline unsigned int sic1IrqEnable (unsigned int uiChannel)
{
    return swiHandle(0x108, uiChannel, 0, 0);
}

/*********************************************************************************************************
** Function name:           vicIrqDisable
** Descriptions:            禁止相应外设的中断
** input parameters:        uiChannel:  外设对应的中断通道号
** output parameters:       none
** Returned value:          1:          成功
**                          0:          失败
*********************************************************************************************************/
__inline unsigned int sic1IrqDisable (unsigned int uiChannel)
{
    return swiHandle(0x109, uiChannel, 0, 0);
}

/*********************************************************************************************************
** Function name:           vicFiqSet
** Descriptions:            设置并使能所选中断通道号为FIQ中断
** input parameters:        uiChannel:  外设对应的中断通道号
**                          uiType:     中断触发类型 -- 0:下降沿 1:上升沿 2:低电平 3:高电平
** output parameters:       none
** Returned value:          1:          成功
**                          0:          失败
*********************************************************************************************************/
__inline unsigned int sic1FiqSet (unsigned int uiChannel, unsigned int uiType)
{
    return swiHandle(0x10A, uiChannel, uiType, 0);
}

/*********************************************************************************************************
** Function name:           vicFiqClr
** Descriptions:            清除所选中断通道号的FIQ中断
** input parameters:        uiChannel:  外设对应的中断通道号
** output parameters:       none
** Returned value:          1:          成功
**                          0:          失败
*********************************************************************************************************/
__inline unsigned int sic1FiqClr (unsigned int uiChannel)
{
    return swiHandle(0x10B, uiChannel, 0, 0);
}

/*********************************************************************************************************
** Function name:           sic2IrqFuncSet
** Descriptions:            设置所选外设的中断触发类型、中断服务函数地址，并使能中断
** input parameters:        uiChannel:  外设对应的中断通道号
**                          uiType:     中断触发类型 -- 0:下降沿 1:上升沿 2:低电平 3:高电平
**                          uiFuncAddr: 中断服务函数地址
** output parameters:       none
** Returned value:          1:          成功
**                          0:          失败
*********************************************************************************************************/
__inline unsigned int sic2IrqFuncSet (unsigned int uiChannel,
                                      unsigned int uiType,
                                      unsigned int uiFuncAddr)
{
    return swiHandle(0x10C, uiChannel, uiType, uiFuncAddr);
}

/*********************************************************************************************************
** Function name:           vicIrqFuncClr
** Descriptions:            清除所选外设的IRQ资源
** input parameters:        uiChannel:  外设对应的中断通道号
** output parameters:       none
** Returned value:          1:          成功
**                          0:          失败
*********************************************************************************************************/
__inline unsigned int sic2IrqFuncClr (unsigned int uiChannel)
{
    return swiHandle(0x10D, uiChannel ,0 ,0);
}

/*********************************************************************************************************
** Function name:           vicIrqEnable
** Descriptions:            使能相应外设的中断
** input parameters:        uiChannel:  外设对应的中断通道号
** output parameters:       none
** Returned value:          1:          成功
**                          0:          失败
*********************************************************************************************************/
__inline unsigned int sic2IrqEnable (unsigned int uiChannel)
{
    return swiHandle(0x10E, uiChannel, 0, 0);
}

/*********************************************************************************************************
** Function name:           vicIrqDisable
** Descriptions:            禁止相应外设的中断
** input parameters:        uiChannel:  外设对应的中断通道号
** output parameters:       none
** Returned value:          1:          成功
**                          0:          失败
*********************************************************************************************************/
__inline unsigned int sic2IrqDisable (unsigned int uiChannel)
{
    return swiHandle(0x10F, uiChannel, 0, 0);
}

/*********************************************************************************************************
** Function name:           vicFiqSet
** Descriptions:            设置并使能所选中断通道号为FIQ中断
** input parameters:        uiChannel:  外设对应的中断通道号
**                          uiType:     中断触发类型 -- 0:下降沿 1:上升沿 2:低电平 3:高电平
** output parameters:       none
** Returned value:          1:          成功
**                          0:          失败
*********************************************************************************************************/
__inline unsigned int sic2FiqSet (unsigned int uiChannel, unsigned int uiType)
{
    return swiHandle(0x110, uiChannel, uiType, 0);
}

/*********************************************************************************************************
** Function name:           vicFiqClr
** Descriptions:            清除所选中断通道号的FIQ中断
** input parameters:        uiChannel:  外设对应的中断通道号
** output parameters:       none
** Returned value:          1:          成功
**                          0:          失败
*********************************************************************************************************/
__inline unsigned int sic2FiqClr (unsigned int uiChannel)
{
    return swiHandle(0x111, uiChannel, 0, 0);
}

#ifdef __cplusplus
    }
#endif                                                                  /*  __cplusplus                 */

#endif                                                                  /*  _VIC_CONTROL_H_             */
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/

