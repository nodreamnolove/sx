/****************************************Copyright (c)****************************************************
**                         Guangzhou ZHIYUAN electronics Co.,LTD.                               
**                                     
**                               http://www.embedtools.com
**
**--------------File Info---------------------------------------------------------------------------------
** File name:           LPC32x0.h
** Last modified Date:  2008-09-19
** Last Version:        1.0
** Descriptions:        header file of LPC32x0 user may modify it as needed
**--------------------------------------------------------------------------------------------------------
** Created by:          Chenmingji
** Created date:        2004-02-02
** Version:             1.0
** Descriptions:        The original version
**
**--------------------------------------------------------------------------------------------------------
** Modified by:         Wangfaqiang 
** Modified date:       2008-09-19
** Version:            
** Descriptions: 
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
#ifndef __LPC32X0_H
#define __LPC32X0_H

    /*
     * System Control 
     */
#define     SYSCTRL_BASE_ADDR	    0x40004000
#define     BOOT_MAP	            (*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0x14))

    /* 
     * Clock and Power Control 
     */
#define     CLK_PM_BASE_ADDR	    0x40004000
#define     PWR_CTRL	            (*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0x44))
#define     OSC_CTRL	            (*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0x4C))
#define     SYSCLK_CTRL	            (*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0x50))
#define     PLL397_CTRL	            (*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0x48))	
#define     HCLKPLL_CTRL	        (*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0x58))
#define     HCLKDIV_CTRL	        (*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0x40))
#define     TEST_CLK	            (*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0xA4))
#define     AUTOCLK_CTRL	        (*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0xEC))

#define     START_ER_INT	        (*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0x20))
#define     START_ER_PIN	        (*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0x30))
#define     START_RSR_INT	        (*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0x24))
#define     START_RSR_PIN	        (*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0x34))
#define     START_SR_INT	        (*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0x28))
#define     START_SR_PIN	        (*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0x38))
#define     START_APR_INT	        (*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0x2C))
#define     START_APR_PIN	        (*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0x3C))

#define     DMACLK_CTRL	            (*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0xE8))
#define     USBDIV_CTRL             (*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0x1C)) 
#define     USBCLK_CTRL	            (*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0x64))
#define     MS_CTRL		            (*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0x80))
#define     TIMCLK_CTRL	            (*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0xBC))
#define     SPI_CTRL	            (*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0xC4))
#define     FLASHCLK_CTRL	        (*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0xC8))
#define     MACCLK_CTRL	            (*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0x90))
#define     LCDCLK_CTRL	            (*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0x54))
#define     I2S_CTRL	            (*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0x7c))
#define     SSP_CTRL	            (*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0x78))
#define     I2CCLK_CTRL	            (*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0xAC))
#define     TIMCLK_CTRL1            (*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0xC0))
#define     ADCLK_CTRL	            (*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0xB4))
#define     ADCLK_CTRL1	            (*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0x60))
#define     KEYCLK_CTRL	            (*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0xB0))
#define     PWMCLK_CTRL	            (*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0xB8))
#define     UARTCLK_CTRL	        (*(volatile unsigned long *)(CLK_PM_BASE_ADDR + 0xE4))

    /* 
     * Interrupt Controller 
     */
#define     MIC_BASE_ADDR	        0x40008000
#define     MIC_ER		            (*(volatile unsigned long *)(MIC_BASE_ADDR + 0x00))
#define     MIC_RSR		            (*(volatile unsigned long *)(MIC_BASE_ADDR + 0x04))
#define     MIC_SR		            (*(volatile unsigned long *)(MIC_BASE_ADDR + 0x08))
#define     MIC_APR		            (*(volatile unsigned long *)(MIC_BASE_ADDR + 0x0C))	
#define     MIC_ATR		            (*(volatile unsigned long *)(MIC_BASE_ADDR + 0x10))
#define     MIC_ITR		            (*(volatile unsigned long *)(MIC_BASE_ADDR + 0x14))

#define     SIC1_BASE_ADDR	        0x4000C000
#define     SIC1_ER		            (*(volatile unsigned long *)(SIC1_BASE_ADDR + 0x00))
#define     SIC1_RSR	            (*(volatile unsigned long *)(SIC1_BASE_ADDR + 0x04))
#define     SIC1_SR		            (*(volatile unsigned long *)(SIC1_BASE_ADDR + 0x08))
#define     SIC1_APR	            (*(volatile unsigned long *)(SIC1_BASE_ADDR + 0x0C))
#define     SIC1_ATR	            (*(volatile unsigned long *)(SIC1_BASE_ADDR + 0x10))
#define     SIC1_ITR	            (*(volatile unsigned long *)(SIC1_BASE_ADDR + 0x14))

#define     SIC2_BASE_ADDR	        0x40010000
#define     SIC2_ER		            (*(volatile unsigned long *)(SIC2_BASE_ADDR + 0x00))
#define     SIC2_RSR	            (*(volatile unsigned long *)(SIC2_BASE_ADDR + 0x04))
#define     SIC2_SR		            (*(volatile unsigned long *)(SIC2_BASE_ADDR + 0x08))
#define     SIC2_APR	            (*(volatile unsigned long *)(SIC2_BASE_ADDR + 0x0C))
#define     SIC2_ATR	            (*(volatile unsigned long *)(SIC2_BASE_ADDR + 0x10))
#define     SIC2_ITR	            (*(volatile unsigned long *)(SIC2_BASE_ADDR + 0x14))

#define     SWI_BASE_ADDR	        0x40004000
#define     SW_INT		            (*(volatile unsigned long *)(SWI_BASE_ADDR + 0xA8))

    /* 
     * DMA  Control register  
     */ 
#define     DMA_GENERAL_BASE_ADDR   0x31000000
#define     DMACIntStat             (*(volatile unsigned long *)(DMA_GENERAL_BASE_ADDR + 0x00))
#define     DMACIntTCStat           (*(volatile unsigned long *)(DMA_GENERAL_BASE_ADDR + 0x04))
#define     DMACIntTCClear          (*(volatile unsigned long *)(DMA_GENERAL_BASE_ADDR + 0x08))
#define     DMACIntErrStat          (*(volatile unsigned long *)(DMA_GENERAL_BASE_ADDR + 0x0C))
#define     DMACIntErrClr           (*(volatile unsigned long *)(DMA_GENERAL_BASE_ADDR + 0x10))
#define     DMACRawIntTCStat        (*(volatile unsigned long *)(DMA_GENERAL_BASE_ADDR + 0x14))
#define     DMACRawIntErrStat       (*(volatile unsigned long *)(DMA_GENERAL_BASE_ADDR + 0x18))
#define     DMACEnbldChns           (*(volatile unsigned long *)(DMA_GENERAL_BASE_ADDR + 0x1C))
#define     DMACSoftBReq            (*(volatile unsigned long *)(DMA_GENERAL_BASE_ADDR + 0x20))
#define     DMACSoftSReq            (*(volatile unsigned long *)(DMA_GENERAL_BASE_ADDR + 0x24))
#define     DMACSoftLBReq           (*(volatile unsigned long *)(DMA_GENERAL_BASE_ADDR + 0x28))
#define     DMACSoftLSReq           (*(volatile unsigned long *)(DMA_GENERAL_BASE_ADDR + 0x2C))
#define     DMACConfig              (*(volatile unsigned long *)(DMA_GENERAL_BASE_ADDR + 0x30))
   
    /* 
     * DMA channel 0 
     */
#define     DMA_CHAN0_BASE_ADDR     0x31000100
#define     DMACC0SrcAddr           (*(volatile unsigned long *)(DMA_CHAN0_BASE_ADDR + 0x00))
#define     DMACC0DestAddr          (*(volatile unsigned long *)(DMA_CHAN0_BASE_ADDR + 0x04))
#define     DMACC0LLI               (*(volatile unsigned long *)(DMA_CHAN0_BASE_ADDR + 0x08))
#define     DMACC0Control           (*(volatile unsigned long *)(DMA_CHAN0_BASE_ADDR + 0x0C))
#define     DMACC0Config            (*(volatile unsigned long *)(DMA_CHAN0_BASE_ADDR + 0x10))
   
    /* 
     * DMA channel 1 
     */
#define     DMA_CHAN1_BASE_ADDR     0x31000120
#define     DMACC1SrcAddr           (*(volatile unsigned long *)(DMA_CHAN1_BASE_ADDR + 0x00))
#define     DMACC1DestAddr          (*(volatile unsigned long *)(DMA_CHAN1_BASE_ADDR + 0x04))
#define     DMACC1LLI               (*(volatile unsigned long *)(DMA_CHAN1_BASE_ADDR + 0x08))
#define     DMACC1Control           (*(volatile unsigned long *)(DMA_CHAN1_BASE_ADDR + 0x0C))
#define     DMACC1Config            (*(volatile unsigned long *)(DMA_CHAN1_BASE_ADDR + 0x10))
   
    /* 
     * DMA channel 2 
     */
#define     DMA_CHAN2_BASE_ADDR     0x31000140
#define     DMACC2SrcAddr           (*(volatile unsigned long *)(DMA_CHAN2_BASE_ADDR + 0x00))
#define     DMACC2DestAddr          (*(volatile unsigned long *)(DMA_CHAN2_BASE_ADDR + 0x04))
#define     DMACC2LLI               (*(volatile unsigned long *)(DMA_CHAN2_BASE_ADDR + 0x08))
#define     DMACC2Control           (*(volatile unsigned long *)(DMA_CHAN2_BASE_ADDR + 0x0C))
#define     DMACC2Config            (*(volatile unsigned long *)(DMA_CHAN2_BASE_ADDR + 0x10))
  
    /* 
     * DMA channel 3 
     */
#define     DMA_CHAN3_BASE_ADDR     0x31000160
#define     DMACC3SrcAddr           (*(volatile unsigned long *)(DMA_CHAN3_BASE_ADDR + 0x00))
#define     DMACC3DestAddr          (*(volatile unsigned long *)(DMA_CHAN3_BASE_ADDR + 0x04))
#define     DMACC3LLI               (*(volatile unsigned long *)(DMA_CHAN3_BASE_ADDR + 0x08))
#define     DMACC3Control           (*(volatile unsigned long *)(DMA_CHAN3_BASE_ADDR + 0x0C))
#define     DMACC3Config            (*(volatile unsigned long *)(DMA_CHAN3_BASE_ADDR + 0x10))
  
    /* 
     * DMA channel 4 
     */
#define     DMA_CHAN4_BASE_ADDR     0x31000180
#define     DMACC4SrcAddr           (*(volatile unsigned long *)(DMA_CHAN4_BASE_ADDR + 0x00))
#define     DMACC4DestAddr          (*(volatile unsigned long *)(DMA_CHAN4_BASE_ADDR + 0x04))
#define     DMACC4LLI               (*(volatile unsigned long *)(DMA_CHAN4_BASE_ADDR + 0x08))
#define     DMACC4Control           (*(volatile unsigned long *)(DMA_CHAN4_BASE_ADDR + 0x0C))
#define     DMACC4Config            (*(volatile unsigned long *)(DMA_CHAN4_BASE_ADDR + 0x10))
    
    /* 
     * DMA channel 5 
     */
#define     DMA_CHAN5_BASE_ADDR     0x310001A0
#define     DMACC5SrcAddr           (*(volatile unsigned long *)(DMA_CHAN5_BASE_ADDR + 0x00))
#define     DMACC5DestAddr          (*(volatile unsigned long *)(DMA_CHAN5_BASE_ADDR + 0x04))
#define     DMACC5LLI               (*(volatile unsigned long *)(DMA_CHAN5_BASE_ADDR + 0x08))
#define     DMACC5Control           (*(volatile unsigned long *)(DMA_CHAN5_BASE_ADDR + 0x0C))
#define     DMACC5Config            (*(volatile unsigned long *)(DMA_CHAN5_BASE_ADDR + 0x10))
    
    /* 
     * DMA channel 6 
     */
#define     DMA_CHAN6_BASE_ADDR     0x310001C0
#define     DMACC6SrcAddr           (*(volatile unsigned long *)(DMA_CHAN6_BASE_ADDR + 0x00))
#define     DMACC6DestAddr          (*(volatile unsigned long *)(DMA_CHAN6_BASE_ADDR + 0x04))
#define     DMACC6LLI               (*(volatile unsigned long *)(DMA_CHAN6_BASE_ADDR + 0x08))
#define     DMACC6Control           (*(volatile unsigned long *)(DMA_CHAN6_BASE_ADDR + 0x0C))
#define     DMACC6Config            (*(volatile unsigned long *)(DMA_CHAN6_BASE_ADDR + 0x10))
  
    /* 
     * DMA channel 7 
     */
#define     DMA_CHAN7_BASE_ADDR     0x310001E0
#define     DMACC7SrcAddr           (*(volatile unsigned long *)(DMA_CHAN7_BASE_ADDR + 0x00))
#define     DMACC7DestAddr          (*(volatile unsigned long *)(DMA_CHAN7_BASE_ADDR + 0x04))
#define     DMACC7LLI               (*(volatile unsigned long *)(DMA_CHAN7_BASE_ADDR + 0x08))
#define     DMACC7Control           (*(volatile unsigned long *)(DMA_CHAN7_BASE_ADDR + 0x0C))
#define     DMACC7Config            (*(volatile unsigned long *)(DMA_CHAN7_BASE_ADDR + 0x10))

    /* 
     * SDRAM Control register 
     */
#define     SDRAM_CLK_BASE_ADDR	    0x40004000
#define     SDRAMCLK_CTRL	        (*(volatile unsigned long *)(SDRAM_CLK_BASE_ADDR + 0x68))

#define     SDRAM_BASE_ADDR		    0x31080000
#define     EMCControl	            (*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x00))
#define     EMCStatus	            (*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x04))
#define     EMCConfig	            (*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x08))
#define     EMCDynamicControl	    (*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x20))
#define     EMCDynamicRefresh	    (*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x24))
#define     EMCDynamicReadConfig	(*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x28))
#define     EMCDynamicRP	        (*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x30))
#define     EMCDynamicRAS	        (*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x34))
#define     EMCDynamicSREX	        (*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x38))
#define     EMCDynamicWR	        (*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x44))
#define     EMCDynamicRC	        (*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x48))
#define     EMCDynamicRFC	        (*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x4C))
#define     EMCDynamicXSR	        (*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x50))
#define     EMCDynamicRRD	        (*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x54))
#define     EMCDynamicMRD	        (*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x58))
#define     EMCDynamicCDLR	        (*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x5C))
#define     EMCStaticExtendedWait   (*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x80))

#define     EMCDynamicConfig0	    (*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x100))
#define     EMCDynamicRasCas0	    (*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x104))

#define     EMCDynamicConfig1	    (*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x120))
#define     EMCDynamicRasCas1	    (*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x124))

#define     EMCAHBControl0	        (*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x400))
#define     EMCAHBStatus0	        (*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x404))
#define     EMCAHBTimeout0	        (*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x408))

#define     EMCAHBControl2	        (*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x440))
#define     EMCAHBStatus2	        (*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x444))
#define     EMCAHBTimeout2	        (*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x448))

#define     EMCAHBControl3	        (*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x460))
#define     EMCAHBStatus3	        (*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x464))
#define     EMCAHBTimeout3	        (*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x468))

#define     EMCAHBControl4	        (*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x480))
#define     EMCAHBStatus4	        (*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x484))
#define     EMCAHBTimeout4	        (*(volatile unsigned long *)(SDRAM_BASE_ADDR + 0x488))

#define     DDR_BASE_ADDR	        0x40004000
#define     DDR_LAP_NOM	            (*(volatile unsigned long *)(DDR_BASE_ADDR + 0x6C))
#define     DDR_LAP_COUNT	        (*(volatile unsigned long *)(DDR_BASE_ADDR + 0x70))
#define     DDR_CAL_DELAY	        (*(volatile unsigned long *)(DDR_BASE_ADDR + 0x74))

    /* 
     * EMC  static               
     */
#define     EMC_STATIC_BASE_ADDR    0x31080200
#define     EMCStaticConfig0        (*(volatile unsigned long *)(EMC_STATIC_BASE_ADDR + 0x00))
#define     EMCStaticWaitWen0       (*(volatile unsigned long *)(EMC_STATIC_BASE_ADDR + 0x04))
#define     EMCStaticWaitOen0       (*(volatile unsigned long *)(EMC_STATIC_BASE_ADDR + 0x08))
#define     EMCStaticWaitRd0        (*(volatile unsigned long *)(EMC_STATIC_BASE_ADDR + 0x0C))
#define     EMCStaticWaitPage0      (*(volatile unsigned long *)(EMC_STATIC_BASE_ADDR + 0x10))
#define     EMCStaticWaitWr0        (*(volatile unsigned long *)(EMC_STATIC_BASE_ADDR + 0x14))
#define     EMCStaticWaitTurn0      (*(volatile unsigned long *)(EMC_STATIC_BASE_ADDR + 0x18))

#define     EMCStaticConfig1        (*(volatile unsigned long *)(EMC_STATIC_BASE_ADDR + 0x20))
#define     EMCStaticWaitWen1       (*(volatile unsigned long *)(EMC_STATIC_BASE_ADDR + 0x24))
#define     EMCStaticWaitOen1       (*(volatile unsigned long *)(EMC_STATIC_BASE_ADDR + 0x28))
#define     EMCStaticWaitRd1        (*(volatile unsigned long *)(EMC_STATIC_BASE_ADDR + 0x2C))
#define     EMCStaticWaitPage1      (*(volatile unsigned long *)(EMC_STATIC_BASE_ADDR + 0x30))
#define     EMCStaticWaitWr1        (*(volatile unsigned long *)(EMC_STATIC_BASE_ADDR + 0x34))
#define     EMCStaticWaitTurn1      (*(volatile unsigned long *)(EMC_STATIC_BASE_ADDR + 0x38))

#define     EMCStaticConfig2        (*(volatile unsigned long *)(EMC_STATIC_BASE_ADDR + 0x40))
#define     EMCStaticWaitWen2       (*(volatile unsigned long *)(EMC_STATIC_BASE_ADDR + 0x44))
#define     EMCStaticWaitOen2       (*(volatile unsigned long *)(EMC_STATIC_BASE_ADDR + 0x48))
#define     EMCStaticWaitRd2        (*(volatile unsigned long *)(EMC_STATIC_BASE_ADDR + 0x4C))
#define     EMCStaticWaitPage2      (*(volatile unsigned long *)(EMC_STATIC_BASE_ADDR + 0x50))
#define     EMCStaticWaitWr2        (*(volatile unsigned long *)(EMC_STATIC_BASE_ADDR + 0x54))
#define     EMCStaticWaitTurn2      (*(volatile unsigned long *)(EMC_STATIC_BASE_ADDR + 0x58))

#define     EMCStaticConfig3        (*(volatile unsigned long *)(EMC_STATIC_BASE_ADDR + 0x60))
#define     EMCStaticWaitWen3       (*(volatile unsigned long *)(EMC_STATIC_BASE_ADDR + 0x64))
#define     EMCStaticWaitOen3       (*(volatile unsigned long *)(EMC_STATIC_BASE_ADDR + 0x68))
#define     EMCStaticWaitRd3        (*(volatile unsigned long *)(EMC_STATIC_BASE_ADDR + 0x6C))
#define     EMCStaticWaitPage3      (*(volatile unsigned long *)(EMC_STATIC_BASE_ADDR + 0x70))
#define     EMCStaticWaitWr3        (*(volatile unsigned long *)(EMC_STATIC_BASE_ADDR + 0x74))
#define     EMCStaticWaitTurn3      (*(volatile unsigned long *)(EMC_STATIC_BASE_ADDR + 0x78))

    /* 
     * Multiple level NAND Flash 
     */
#define     MLC_BASE_ADDR	        0x200B8000
#define     MLC_CMD		            (*(volatile unsigned long *)(MLC_BASE_ADDR + 0x00))
#define     MLC_ADDR	            (*(volatile unsigned long *)(MLC_BASE_ADDR + 0x04))
#define     MLC_ECC_ENC_REG	        (*(volatile unsigned long *)(MLC_BASE_ADDR + 0x08))
#define     MLC_ECC_DEC_REG	        (*(volatile unsigned long *)(MLC_BASE_ADDR + 0x0C))
#define     MLC_ECC_AUTO_ENC_REG	(*(volatile unsigned long *)(MLC_BASE_ADDR + 0x10))
#define     MLC_ECC_AUTO_DEC_REG	(*(volatile unsigned long *)(MLC_BASE_ADDR + 0x14))
#define     MLC_RPR		            (*(volatile unsigned long *)(MLC_BASE_ADDR + 0x18))
#define     MLC_WPR		            (*(volatile unsigned long *)(MLC_BASE_ADDR + 0x1C))
#define     MLC_RUBP	            (*(volatile unsigned long *)(MLC_BASE_ADDR + 0x20))
#define     MLC_ROBP	            (*(volatile unsigned long *)(MLC_BASE_ADDR + 0x24))
#define     MLC_SW_WP_ADD_LOW	    (*(volatile unsigned long *)(MLC_BASE_ADDR + 0x28))
#define     MLC_SW_WP_ADD_HIG	    (*(volatile unsigned long *)(MLC_BASE_ADDR + 0x2C))
#define     MLC_ICR		            (*(volatile unsigned long *)(MLC_BASE_ADDR + 0x30))
#define     MLC_TIME_REG	        (*(volatile unsigned long *)(MLC_BASE_ADDR + 0x34))
#define     MLC_IRQ_MR	            (*(volatile unsigned long *)(MLC_BASE_ADDR + 0x38))
#define     MLC_IRQ_SR	            (*(volatile unsigned long *)(MLC_BASE_ADDR + 0x3C))
#define     MLC_LOCK_PR	            (*(volatile unsigned long *)(MLC_BASE_ADDR + 0x44))
#define     MLC_ISR		            (*(volatile unsigned long *)(MLC_BASE_ADDR + 0x48))
#define     MLC_CEH		            (*(volatile unsigned long *)(MLC_BASE_ADDR + 0x4C))
    
    /* 
     * multiple level NAND Flash BUFF 
     */
#define     MLC_BUFF                (*(volatile unsigned long *)(0x200A8000))
#define     MLC_DATA                (*(volatile unsigned long *)(0x200B0000))

    /* 
     * Single level NAND Flash 
     */
#define     SLC_BASE_ADDR	        0x20020000
#define     SLC_DATA	            (*(volatile unsigned long *)(SLC_BASE_ADDR + 0x00))
#define     SLC_ADDR	            (*(volatile unsigned long *)(SLC_BASE_ADDR + 0x04))
#define     SLC_CMD		            (*(volatile unsigned long *)(SLC_BASE_ADDR + 0x08))
#define     SLC_STOP	            (*(volatile unsigned long *)(SLC_BASE_ADDR + 0x0C))
#define     SLC_CTRL	            (*(volatile unsigned long *)(SLC_BASE_ADDR + 0x10))
#define     SLC_CFG		            (*(volatile unsigned long *)(SLC_BASE_ADDR + 0x14))
#define     SLC_STAT	            (*(volatile unsigned long *)(SLC_BASE_ADDR + 0x18))
#define     SLC_INT_STAT	        (*(volatile unsigned long *)(SLC_BASE_ADDR + 0x1C))
#define     SLC_IEN		            (*(volatile unsigned long *)(SLC_BASE_ADDR + 0x20))
#define     SLC_ISR		            (*(volatile unsigned long *)(SLC_BASE_ADDR + 0x24))
#define     SLC_ICR		            (*(volatile unsigned long *)(SLC_BASE_ADDR + 0x28))
#define     SLC_TAC		            (*(volatile unsigned long *)(SLC_BASE_ADDR + 0x2C))
#define     SLC_TC                  (*(volatile unsigned long *)(SLC_BASE_ADDR + 0x30))
#define     SLC_ECC	                (*(volatile unsigned long *)(SLC_BASE_ADDR + 0x34))
#define     SLC_DMA_DATA            (*(volatile unsigned long *)(SLC_BASE_ADDR + 0x38))

    /* 
     * LCD Controller  Registers 
     */
#define     LCD_BASE_ADDR           0x31040000
#define     LCD_TIMH                (*(volatile unsigned long *)(LCD_BASE_ADDR + 0x000))
#define     LCD_TIMV                (*(volatile unsigned long *)(LCD_BASE_ADDR + 0x004))
#define     LCD_POL                 (*(volatile unsigned long *)(LCD_BASE_ADDR + 0x008))
#define     LCD_LE                  (*(volatile unsigned long *)(LCD_BASE_ADDR + 0x00C))
#define     LCD_UPBASE              (*(volatile unsigned long *)(LCD_BASE_ADDR + 0x010))
#define     LCD_LPBASE              (*(volatile unsigned long *)(LCD_BASE_ADDR + 0x014))
#define     LCD_CTRL                (*(volatile unsigned long *)(LCD_BASE_ADDR + 0x018))
#define     LCD_INTMSK              (*(volatile unsigned long *)(LCD_BASE_ADDR + 0x01C))
#define     LCD_INTRAW              (*(volatile unsigned long *)(LCD_BASE_ADDR + 0x020))
#define     LCD_INTSTAT             (*(volatile unsigned long *)(LCD_BASE_ADDR + 0x024))
#define     LCD_INTCLR              (*(volatile unsigned long *)(LCD_BASE_ADDR + 0x028))
#define     LCD_UPCURR              (*(volatile unsigned long *)(LCD_BASE_ADDR + 0x02C))
#define     LCD_LPCURR              (*(volatile unsigned long *)(LCD_BASE_ADDR + 0x030))
    
    /* 
     * LCD PAL 0x31040200 -> 0x310403FC 
     */
#define     LCD_PAL                 (*(volatile unsigned long *)(LCD_BASE_ADDR + 0x200))
    
    /* 
     * LCD IMG 0x31040800 -> 0x31040BFC 
     */ 
#define     CRSR_IMG                (*(volatile unsigned long *)(LCD_BASE_ADDR + 0x800))
#define     CRSR_CTRL               (*(volatile unsigned long *)(LCD_BASE_ADDR + 0xC00)) 
#define     CRSR_CFG                (*(volatile unsigned long *)(LCD_BASE_ADDR + 0xC04))
#define     CRSR_PAL0               (*(volatile unsigned long *)(LCD_BASE_ADDR + 0xC08))
#define     CRSR_PAL1               (*(volatile unsigned long *)(LCD_BASE_ADDR + 0xC0C))
#define     CRSR_XY                 (*(volatile unsigned long *)(LCD_BASE_ADDR + 0xC10))
#define     CRSR_CLIP               (*(volatile unsigned long *)(LCD_BASE_ADDR + 0xC14))
#define     CRSR_INTMSK             (*(volatile unsigned long *)(LCD_BASE_ADDR + 0xC20))
#define     CRSR_INTCLR             (*(volatile unsigned long *)(LCD_BASE_ADDR + 0xC24))
#define     CRSR_INTRAW             (*(volatile unsigned long *)(LCD_BASE_ADDR + 0xC28))
#define     CRSR_INTSTAT            (*(volatile unsigned long *)(LCD_BASE_ADDR + 0xC2C))

    /* 
     * TOUCH Screen Register 
     */
#define     TOUCH_BASE_ADDR         0x40048000
#define     ADC_STAT                (*(volatile unsigned long *)(TOUCH_BASE_ADDR + 0x00)) 
#define     ADC_SELECT              (*(volatile unsigned long *)(TOUCH_BASE_ADDR + 0x04)) 
#define     ADC_CTRL                (*(volatile unsigned long *)(TOUCH_BASE_ADDR + 0x08)) 
#define     TSC_SAMPLE_FIFO         (*(volatile unsigned long *)(TOUCH_BASE_ADDR + 0x0c))         
#define     TSC_DTR                 (*(volatile unsigned long *)(TOUCH_BASE_ADDR + 0x10)) 
#define     TSC_RTR                 (*(volatile unsigned long *)(TOUCH_BASE_ADDR + 0x14)) 
#define     TSC_UTR                 (*(volatile unsigned long *)(TOUCH_BASE_ADDR + 0x18)) 
#define     TSC_TTR                 (*(volatile unsigned long *)(TOUCH_BASE_ADDR + 0x1C)) 
#define     TSC_DXP                 (*(volatile unsigned long *)(TOUCH_BASE_ADDR + 0x20)) 
#define     TSC_MIN_X               (*(volatile unsigned long *)(TOUCH_BASE_ADDR + 0x24))   
#define     TSC_MAX_X               (*(volatile unsigned long *)(TOUCH_BASE_ADDR + 0x28))  
#define     TSC_MIN_Y               (*(volatile unsigned long *)(TOUCH_BASE_ADDR + 0x2C))  
#define     TSC_MAX_Y               (*(volatile unsigned long *)(TOUCH_BASE_ADDR + 0x30))   
#define     TSC_AUX_UTR             (*(volatile unsigned long *)(TOUCH_BASE_ADDR + 0x34))    
#define     TSC_AUX_MIN             (*(volatile unsigned long *)(TOUCH_BASE_ADDR + 0x38))    
#define     TSC_AUX_MAX             (*(volatile unsigned long *)(TOUCH_BASE_ADDR + 0x3C))     
#define     TSC_AUX_VALUE           (*(volatile unsigned long *)(TOUCH_BASE_ADDR + 0x44))      
#define     ADC_VALUE               (*(volatile unsigned long *)(TOUCH_BASE_ADDR + 0x48))   

    /* 
     * Keyboard Scan controller interface 
     */
#define     KS_BASE_ADDR		    0x40050000
#define     KS_DEB		            (*(volatile unsigned long *)(KS_BASE_ADDR + 0x00))
#define     KS_STATE_COND	        (*(volatile unsigned long *)(KS_BASE_ADDR + 0x04))
#define     KS_IRQ		            (*(volatile unsigned long *)(KS_BASE_ADDR + 0x08))
#define     KS_SCAN_CTL	            (*(volatile unsigned long *)(KS_BASE_ADDR + 0x0C))
#define     KS_FAST_TST	            (*(volatile unsigned long *)(KS_BASE_ADDR + 0x10))
#define     KS_MATRIX_DIM        	(*(volatile unsigned long *)(KS_BASE_ADDR + 0x14))
#define     KS_DATA0	            (*(volatile unsigned long *)(KS_BASE_ADDR + 0x40))
#define     KS_DATA1	            (*(volatile unsigned long *)(KS_BASE_ADDR + 0x44))
#define     KS_DATA2	            (*(volatile unsigned long *)(KS_BASE_ADDR + 0x48))
#define     KS_DATA3	            (*(volatile unsigned long *)(KS_BASE_ADDR + 0x4C))
#define     KS_DATA4            	(*(volatile unsigned long *)(KS_BASE_ADDR + 0x50))
#define     KS_DATA5	            (*(volatile unsigned long *)(KS_BASE_ADDR + 0x54))
#define     KS_DATA6	            (*(volatile unsigned long *)(KS_BASE_ADDR + 0x58))
#define     KS_DATA7             	(*(volatile unsigned long *)(KS_BASE_ADDR + 0x5C))

    /* 
     * Ethernet controller  registers   
     */
#define     MAC_BASE_ADDR		    0x31060000 
#define     MAC_MAC1                (*(volatile unsigned long *)(MAC_BASE_ADDR + 0x000)) 
#define     MAC_MAC2                (*(volatile unsigned long *)(MAC_BASE_ADDR + 0x004)) 
#define     MAC_IPGT                (*(volatile unsigned long *)(MAC_BASE_ADDR + 0x008)) 
#define     MAC_IPGR                (*(volatile unsigned long *)(MAC_BASE_ADDR + 0x00C))
#define     MAC_CLRT                (*(volatile unsigned long *)(MAC_BASE_ADDR + 0x010))
#define     MAC_MAXF                (*(volatile unsigned long *)(MAC_BASE_ADDR + 0x014))
#define     MAC_SUPP                (*(volatile unsigned long *)(MAC_BASE_ADDR + 0x018))
#define     MAC_TEST                (*(volatile unsigned long *)(MAC_BASE_ADDR + 0x01C))
#define     MAC_MCFG                (*(volatile unsigned long *)(MAC_BASE_ADDR + 0x020))
#define     MAC_MCMD                (*(volatile unsigned long *)(MAC_BASE_ADDR + 0x024))
#define     MAC_MADR                (*(volatile unsigned long *)(MAC_BASE_ADDR + 0x028))
#define     MAC_MWTD                (*(volatile unsigned long *)(MAC_BASE_ADDR + 0x02C))
#define     MAC_MRDD                (*(volatile unsigned long *)(MAC_BASE_ADDR + 0x030))
#define     MAC_MIND                (*(volatile unsigned long *)(MAC_BASE_ADDR + 0x034))

#define     MAC_SA0                 (*(volatile unsigned long *)(MAC_BASE_ADDR + 0x040)) 
#define     MAC_SA1                 (*(volatile unsigned long *)(MAC_BASE_ADDR + 0x044)) 
#define     MAC_SA2                 (*(volatile unsigned long *)(MAC_BASE_ADDR + 0x048)) 

#define     MAC_COMMAND             (*(volatile unsigned long *)(MAC_BASE_ADDR + 0x100)) 
#define     MAC_STATUS              (*(volatile unsigned long *)(MAC_BASE_ADDR + 0x104)) 
#define     MAC_RXDESCRIPTOR        (*(volatile unsigned long *)(MAC_BASE_ADDR + 0x108)) 
#define     MAC_RXSTATUS            (*(volatile unsigned long *)(MAC_BASE_ADDR + 0x10C)) 
#define     MAC_RXDESCRIPTORNUM     (*(volatile unsigned long *)(MAC_BASE_ADDR + 0x110)) 
#define     MAC_RXPRODUCEINDEX      (*(volatile unsigned long *)(MAC_BASE_ADDR + 0x114)) 
#define     MAC_RXCONSUMEINDEX      (*(volatile unsigned long *)(MAC_BASE_ADDR + 0x118)) 
#define     MAC_TXDESCRIPTOR        (*(volatile unsigned long *)(MAC_BASE_ADDR + 0x11C)) 
#define     MAC_TXSTATUS            (*(volatile unsigned long *)(MAC_BASE_ADDR + 0x120)) 
#define     MAC_TXDESCRIPTORNUM     (*(volatile unsigned long *)(MAC_BASE_ADDR + 0x124)) 
#define     MAC_TXPRODUCEINDEX      (*(volatile unsigned long *)(MAC_BASE_ADDR + 0x128)) 
#define     MAC_TXCONSUMEINDEX      (*(volatile unsigned long *)(MAC_BASE_ADDR + 0x12C)) 

#define     MAC_TSV0                (*(volatile unsigned long *)(MAC_BASE_ADDR + 0x158)) 
#define     MAC_TSV1                (*(volatile unsigned long *)(MAC_BASE_ADDR + 0x15C)) 
#define     MAC_RSV                 (*(volatile unsigned long *)(MAC_BASE_ADDR + 0x160)) 

#define     MAC_FLOWCONTROLCNT      (*(volatile unsigned long *)(MAC_BASE_ADDR + 0x170)) 
#define     MAC_FLOWCONTROLSTS      (*(volatile unsigned long *)(MAC_BASE_ADDR + 0x174)) 

#define     MAC_RXFILTERCTRL        (*(volatile unsigned long *)(MAC_BASE_ADDR + 0x200))
#define     MAC_RXFILTERWOLSTS      (*(volatile unsigned long *)(MAC_BASE_ADDR + 0x204))
#define     MAC_RXFILTERWOLCLR      (*(volatile unsigned long *)(MAC_BASE_ADDR + 0x208))

#define     MAC_HASHFILTERL         (*(volatile unsigned long *)(MAC_BASE_ADDR + 0x210))
#define     MAC_HASHFILTERH         (*(volatile unsigned long *)(MAC_BASE_ADDR + 0x214))

#define     MAC_INTSTATUS           (*(volatile unsigned long *)(MAC_BASE_ADDR + 0xFE0))
#define     MAC_INTENABLE           (*(volatile unsigned long *)(MAC_BASE_ADDR + 0xFE4))
#define     MAC_INTCLEAR            (*(volatile unsigned long *)(MAC_BASE_ADDR + 0xFE8))
#define     MAC_INTSET              (*(volatile unsigned long *)(MAC_BASE_ADDR + 0xFEC))

#define     MAC_POWERDOWN           (*(volatile unsigned long *)(MAC_BASE_ADDR + 0xFF4))
#define     MAC_MODULEID            (*(volatile unsigned long *)(MAC_BASE_ADDR + 0xFFC))

    /* 
     * USB Device 
     */
#define     USB_BASE_ADDR		    0x31020200

#define     DEV_INT_STAT            (*(volatile unsigned long *)(USB_BASE_ADDR + 0x00))
#define     DEV_INT_EN              (*(volatile unsigned long *)(USB_BASE_ADDR + 0x04))
#define     DEV_INT_CLR             (*(volatile unsigned long *)(USB_BASE_ADDR + 0x08))
#define     DEV_INT_SET             (*(volatile unsigned long *)(USB_BASE_ADDR + 0x0C))
#define     DEV_INT_PRIO            (*(volatile unsigned long *)(USB_BASE_ADDR + 0x2C))
#define     EP_INT_STAT             (*(volatile unsigned long *)(USB_BASE_ADDR + 0x30))
#define     EP_INT_EN               (*(volatile unsigned long *)(USB_BASE_ADDR + 0x34))
#define     EP_INT_CLR              (*(volatile unsigned long *)(USB_BASE_ADDR + 0x38))
#define     EP_INT_SET              (*(volatile unsigned long *)(USB_BASE_ADDR + 0x3C))
#define     EP_INT_PRIO             (*(volatile unsigned long *)(USB_BASE_ADDR + 0x40))
#define     REALIZE_EP              (*(volatile unsigned long *)(USB_BASE_ADDR + 0x44))
#define     EP_INDEX                (*(volatile unsigned long *)(USB_BASE_ADDR + 0x48))
#define     MAXPACKET_SIZE          (*(volatile unsigned long *)(USB_BASE_ADDR + 0x4C))
#define     CMD_CODE                (*(volatile unsigned long *)(USB_BASE_ADDR + 0x10))
#define     CMD_DATA                (*(volatile unsigned long *)(USB_BASE_ADDR + 0x14))
#define     RX_DATA                 (*(volatile unsigned long *)(USB_BASE_ADDR + 0x18))
#define     TX_DATA                 (*(volatile unsigned long *)(USB_BASE_ADDR + 0x1C))
#define     RX_PLENGTH              (*(volatile unsigned long *)(USB_BASE_ADDR + 0x20))
#define     TX_PLENGTH              (*(volatile unsigned long *)(USB_BASE_ADDR + 0x24))
#define     USB_CTRL                (*(volatile unsigned long *)(USB_BASE_ADDR + 0x28))
#define     DC_REVISION             (*(volatile unsigned long *)(USB_BASE_ADDR + 0x7C))

    /* 
     * USB DMA Registers 
     */
#define     DMA_REQ_STAT            (*(volatile unsigned long *)(USB_BASE_ADDR + 0x50))
#define     DMA_REQ_CLR             (*(volatile unsigned long *)(USB_BASE_ADDR + 0x54))
#define     DMA_REQ_SET             (*(volatile unsigned long *)(USB_BASE_ADDR + 0x58))
#define     UDCA_HEAD               (*(volatile unsigned long *)(USB_BASE_ADDR + 0x80))
#define     EP_DMA_STAT             (*(volatile unsigned long *)(USB_BASE_ADDR + 0x84))
#define     EP_DMA_EN               (*(volatile unsigned long *)(USB_BASE_ADDR + 0x88))
#define     EP_DMA_DIS              (*(volatile unsigned long *)(USB_BASE_ADDR + 0x8C))
#define     DMA_INT_STAT            (*(volatile unsigned long *)(USB_BASE_ADDR + 0x90))
#define     DMA_INT_EN              (*(volatile unsigned long *)(USB_BASE_ADDR + 0x94))
#define     EOT_INT_STAT            (*(volatile unsigned long *)(USB_BASE_ADDR + 0xA0))
#define     EOT_INT_CLR             (*(volatile unsigned long *)(USB_BASE_ADDR + 0xA4))
#define     EOT_INT_SET             (*(volatile unsigned long *)(USB_BASE_ADDR + 0xA8))
#define     NDD_REQ_INT_STAT        (*(volatile unsigned long *)(USB_BASE_ADDR + 0xAC))
#define     NDD_REQ_INT_CLR         (*(volatile unsigned long *)(USB_BASE_ADDR + 0xB0))
#define     NDD_REQ_INT_SET         (*(volatile unsigned long *)(USB_BASE_ADDR + 0xB4))
#define     SYS_ERR_INT_STAT        (*(volatile unsigned long *)(USB_BASE_ADDR + 0xB8))
#define     SYS_ERR_INT_CLR         (*(volatile unsigned long *)(USB_BASE_ADDR + 0xBC))
#define     SYS_ERR_INT_SET         (*(volatile unsigned long *)(USB_BASE_ADDR + 0xC0))
#define     MODULE_ID               (*(volatile unsigned long *)(USB_BASE_ADDR + 0xFC))

    /* 
     * USB Host Controller 
     */
#define     USBH_BASE_ADDR	        0x31020000
#define     HcRevision	            (*(volatile unsigned long *)(USBH_BASE_ADDR + 0x00))
#define     HcControl	            (*(volatile unsigned long *)(USBH_BASE_ADDR + 0x04))
#define     HcCommandStatus	        (*(volatile unsigned long *)(USBH_BASE_ADDR + 0x08))
#define     HcInterruptStatus	    (*(volatile unsigned long *)(USBH_BASE_ADDR + 0x0C))
#define     HcInterruptEnable	    (*(volatile unsigned long *)(USBH_BASE_ADDR + 0x10))
#define     HcInterruptDisable	    (*(volatile unsigned long *)(USBH_BASE_ADDR + 0x14))
#define     HcHCCA		            (*(volatile unsigned long *)(USBH_BASE_ADDR + 0x18))
#define     HcPeriodCurrentED	    (*(volatile unsigned long *)(USBH_BASE_ADDR + 0x1C))
#define     HcControlHeadED	        (*(volatile unsigned long *)(USBH_BASE_ADDR + 0x20))
#define     HcControlCurrentED	    (*(volatile unsigned long *)(USBH_BASE_ADDR + 0x24))
#define     HcBulkHeadED	        (*(volatile unsigned long *)(USBH_BASE_ADDR + 0x28))
#define     HcBulkCurrentED         (*(volatile unsigned long *)(USBH_BASE_ADDR + 0x2C))
#define     HcDoneHead	            (*(volatile unsigned long *)(USBH_BASE_ADDR + 0x30))
#define     HcFmInterval	        (*(volatile unsigned long *)(USBH_BASE_ADDR + 0x34))
#define     HcFmRemaining	        (*(volatile unsigned long *)(USBH_BASE_ADDR + 0x38))
#define     HcFmNumber	            (*(volatile unsigned long *)(USBH_BASE_ADDR + 0x3C))
#define     HcPeriodicStart	        (*(volatile unsigned long *)(USBH_BASE_ADDR + 0x40))
#define     HcLSThreshold	        (*(volatile unsigned long *)(USBH_BASE_ADDR + 0x44))
#define     HcRhDescriptorA	        (*(volatile unsigned long *)(USBH_BASE_ADDR + 0x48))
#define     HcRhDescriptorB	        (*(volatile unsigned long *)(USBH_BASE_ADDR + 0x4C))
#define     HcRhStatus	            (*(volatile unsigned long *)(USBH_BASE_ADDR + 0x50))
#define     HcRhPortStatus1	        (*(volatile unsigned long *)(USBH_BASE_ADDR + 0x54))
#define     HcRhPortStatus2	        (*(volatile unsigned long *)(USBH_BASE_ADDR + 0x58))
#define     HcModuleID	            (*(volatile unsigned long *)(USBH_BASE_ADDR + 0x5C))

    /* 
     * USB OTG 
     */
#define     USB_OTG_BASE_ADDR	    0x31020000
#define     OTG_INT_STATUS	        (*(volatile unsigned long *)(USB_OTG_BASE_ADDR + 0x100))
#define     OTG_INT_ENABLE	        (*(volatile unsigned long *)(USB_OTG_BASE_ADDR + 0x104))
#define     OTG_INT_SET             (*(volatile unsigned long *)(USB_OTG_BASE_ADDR + 0x108))
#define     OTG_INT_CLEAR	        (*(volatile unsigned long *)(USB_OTG_BASE_ADDR + 0x10C))
#define     OTG_STATUS   	        (*(volatile unsigned long *)(USB_OTG_BASE_ADDR + 0x110))
#define     OTG_TIMER	            (*(volatile unsigned long *)(USB_OTG_BASE_ADDR + 0x114))
#define     OTG_CLOCK_CONTROL	    (*(volatile unsigned long *)(USB_OTG_BASE_ADDR + 0xFF4))
#define     OTG_CLOCK_STATUS	    (*(volatile unsigned long *)(USB_OTG_BASE_ADDR + 0xFF8))
    
    /* 
     * USB OTG I2C register 
     */
#define     OTG_I2C_BASE_ADDR	    0x31020300
#define     I2C_RX		            (*(volatile unsigned long *)(OTG_I2C_BASE_ADDR + 0x00))
#define     I2C_TX		            (*(volatile unsigned long *)(OTG_I2C_BASE_ADDR + 0x00))
#define     I2C_STS		            (*(volatile unsigned long *)(OTG_I2C_BASE_ADDR + 0x04))
#define     I2C_CTL  	            (*(volatile unsigned long *)(OTG_I2C_BASE_ADDR + 0x08))
#define     I2C_CLKHI	            (*(volatile unsigned long *)(OTG_I2C_BASE_ADDR + 0x0C))
#define     I2C_CLKLO	            (*(volatile unsigned long *)(OTG_I2C_BASE_ADDR + 0x10))

    /* 
     * SD Card Interface 
     */
#define     MCI_BASE_ADDR	        0x20098000
#define     MCIPower                (*(volatile unsigned long *)(MCI_BASE_ADDR + 0x00))
#define     MCIClock                (*(volatile unsigned long *)(MCI_BASE_ADDR + 0x04))
#define     MCIArgument             (*(volatile unsigned long *)(MCI_BASE_ADDR + 0x08))
#define     MCICommand              (*(volatile unsigned long *)(MCI_BASE_ADDR + 0x0C))
#define     MCIRespCmd              (*(volatile unsigned long *)(MCI_BASE_ADDR + 0x10))
#define     MCIResponse0            (*(volatile unsigned long *)(MCI_BASE_ADDR + 0x14))
#define     MCIResponse1            (*(volatile unsigned long *)(MCI_BASE_ADDR + 0x18))
#define     MCIResponse2            (*(volatile unsigned long *)(MCI_BASE_ADDR + 0x1C))
#define     MCIResponse3            (*(volatile unsigned long *)(MCI_BASE_ADDR + 0x20))
#define     MCIDataTimer            (*(volatile unsigned long *)(MCI_BASE_ADDR + 0x24))
#define     MCIDataLength           (*(volatile unsigned long *)(MCI_BASE_ADDR + 0x28))
#define     MCIDataCtrl             (*(volatile unsigned long *)(MCI_BASE_ADDR + 0x2C))
#define     MCIDataCnt              (*(volatile unsigned long *)(MCI_BASE_ADDR + 0x30))
#define     MCIStatus               (*(volatile unsigned long *)(MCI_BASE_ADDR + 0x34))
#define     MCIClear                (*(volatile unsigned long *)(MCI_BASE_ADDR + 0x38))
#define     MCIMask0                (*(volatile unsigned long *)(MCI_BASE_ADDR + 0x3C))
#define     MCIMask1                (*(volatile unsigned long *)(MCI_BASE_ADDR + 0x40))
#define     MCIFifoCnt              (*(volatile unsigned long *)(MCI_BASE_ADDR + 0x48))
#define     MCIFIFO                 (*(volatile unsigned long *)(MCI_BASE_ADDR + 0x80))
    
    /*
     * Note: for FIFO register, the addr. is 0x20098080 through 0x200980BC 
     */
#define     SD_FIFO		            (*(volatile unsigned long *)(SD_BASE_ADDR + 0x80))


    /* 
     * Universal Asynchronous Receiver Transmitter (Standard UART3,4,5,6) 
     */
#define     UART3_BASE_ADDR		    0x40080000
#define     U3RBR                   (*(volatile unsigned long *)(UART3_BASE_ADDR + 0x00))
#define     U3THR                   (*(volatile unsigned long *)(UART3_BASE_ADDR + 0x00))
#define     U3DLL                   (*(volatile unsigned long *)(UART3_BASE_ADDR + 0x00))
#define     U3DLM                   (*(volatile unsigned long *)(UART3_BASE_ADDR + 0x04))
#define     U3IER                   (*(volatile unsigned long *)(UART3_BASE_ADDR + 0x04))
#define     U3IIR                   (*(volatile unsigned long *)(UART3_BASE_ADDR + 0x08))
#define     U3FCR                   (*(volatile unsigned long *)(UART3_BASE_ADDR + 0x08))
#define     U3LCR                   (*(volatile unsigned long *)(UART3_BASE_ADDR + 0x0C))
#define     U3LSR                   (*(volatile unsigned long *)(UART3_BASE_ADDR + 0x14))
#define     U3RXLEV                 (*(volatile unsigned long *)(UART3_BASE_ADDR + 0x1C))

#define     UART4_BASE_ADDR		    0x40088000
#define     U4RBR                   (*(volatile unsigned long *)(UART4_BASE_ADDR + 0x00))
#define     U4THR                   (*(volatile unsigned long *)(UART4_BASE_ADDR + 0x00))
#define     U4DLL                   (*(volatile unsigned long *)(UART4_BASE_ADDR + 0x00))
#define     U4DLM                   (*(volatile unsigned long *)(UART4_BASE_ADDR + 0x04))
#define     U4IER                   (*(volatile unsigned long *)(UART4_BASE_ADDR + 0x04))
#define     U4IIR                   (*(volatile unsigned long *)(UART4_BASE_ADDR + 0x08))
#define     U4FCR                   (*(volatile unsigned long *)(UART4_BASE_ADDR + 0x08))
#define     U4LCR                   (*(volatile unsigned long *)(UART4_BASE_ADDR + 0x0C))
#define     U4LSR                   (*(volatile unsigned long *)(UART4_BASE_ADDR + 0x14))
#define     U4RXLEV                 (*(volatile unsigned long *)(UART4_BASE_ADDR + 0x1C))

#define     UART5_BASE_ADDR		    0x40090000
#define     U5RBR                   (*(volatile unsigned long *)(UART5_BASE_ADDR + 0x00))
#define     U5THR                   (*(volatile unsigned long *)(UART5_BASE_ADDR + 0x00))
#define     U5DLL                   (*(volatile unsigned long *)(UART5_BASE_ADDR + 0x00))
#define     U5DLM                   (*(volatile unsigned long *)(UART5_BASE_ADDR + 0x04))
#define     U5IER                   (*(volatile unsigned long *)(UART5_BASE_ADDR + 0x04))
#define     U5IIR                   (*(volatile unsigned long *)(UART5_BASE_ADDR + 0x08))
#define     U5FCR                   (*(volatile unsigned long *)(UART5_BASE_ADDR + 0x08))
#define     U5LCR                   (*(volatile unsigned long *)(UART5_BASE_ADDR + 0x0C))
#define     U5LSR                   (*(volatile unsigned long *)(UART5_BASE_ADDR + 0x14))
#define     U5RXLEV                 (*(volatile unsigned long *)(UART5_BASE_ADDR + 0x1C))

#define     UART6_BASE_ADDR		    0x40098000
#define     U6RBR                   (*(volatile unsigned long *)(UART6_BASE_ADDR + 0x00))
#define     U6THR                   (*(volatile unsigned long *)(UART6_BASE_ADDR + 0x00))
#define     U6DLL                   (*(volatile unsigned long *)(UART6_BASE_ADDR + 0x00))
#define     U6DLM                   (*(volatile unsigned long *)(UART6_BASE_ADDR + 0x04))
#define     U6IER                   (*(volatile unsigned long *)(UART6_BASE_ADDR + 0x04))
#define     U6IIR                   (*(volatile unsigned long *)(UART6_BASE_ADDR + 0x08))
#define     U6FCR                   (*(volatile unsigned long *)(UART6_BASE_ADDR + 0x08))
#define     U6LCR                   (*(volatile unsigned long *)(UART6_BASE_ADDR + 0x0C))
#define     U6LSR                   (*(volatile unsigned long *)(UART6_BASE_ADDR + 0x14))
#define     U6RXLEV                 (*(volatile unsigned long *)(UART6_BASE_ADDR + 0x1C))

#define     UART_CLK_BASE_ADDR	    0x40004000
#define     U3CLK                   (*(volatile unsigned long *)(UART_CLK_BASE_ADDR + 0xD0))
#define     U4CLK                   (*(volatile unsigned long *)(UART_CLK_BASE_ADDR + 0xD4))
#define     U5CLK                   (*(volatile unsigned long *)(UART_CLK_BASE_ADDR + 0xD8))
#define     U6CLK                   (*(volatile unsigned long *)(UART_CLK_BASE_ADDR + 0xDC))
#define     IRDACLK                 (*(volatile unsigned long *)(UART_CLK_BASE_ADDR + 0xE0))

#define     UART_CTRL_BASE_ADDR	    0x40054000
#define     UART_CTRL               (*(volatile unsigned long *)(UART_CTRL_BASE_ADDR + 0x00))
#define     UART_CLKMODE            (*(volatile unsigned long *)(UART_CTRL_BASE_ADDR + 0x04))
#define     UART_LOOP               (*(volatile unsigned long *)(UART_CTRL_BASE_ADDR + 0x08))

    /* 
     * High-speed Universal Asynchronous Receiver Transmitter (UART1,2,7) 
     */
#define     HS_UART1_BASE_ADDR		0x40014000
#define     HSU1_RX                 (*(volatile unsigned long *)(HS_UART1_BASE_ADDR + 0x00))
#define     HSU1_TX                 (*(volatile unsigned long *)(HS_UART1_BASE_ADDR + 0x00))
#define     HSU1_LEVEL              (*(volatile unsigned long *)(HS_UART1_BASE_ADDR + 0x04))
#define     HSU1_IIR                (*(volatile unsigned long *)(HS_UART1_BASE_ADDR + 0x08))
#define     HSU1_CTRL               (*(volatile unsigned long *)(HS_UART1_BASE_ADDR + 0x0C))
#define     HSU1_RATE               (*(volatile unsigned long *)(HS_UART1_BASE_ADDR + 0x10))

#define     HS_UART2_BASE_ADDR		0x40018000
#define     HSU2_RX                 (*(volatile unsigned long *)(HS_UART2_BASE_ADDR + 0x00))
#define     HSU2_TX                 (*(volatile unsigned long *)(HS_UART2_BASE_ADDR + 0x00))
#define     HSU2_LEVEL              (*(volatile unsigned long *)(HS_UART2_BASE_ADDR + 0x04))
#define     HSU2_IIR                (*(volatile unsigned long *)(HS_UART2_BASE_ADDR + 0x08))
#define     HSU2_CTRL               (*(volatile unsigned long *)(HS_UART2_BASE_ADDR + 0x0C))
#define     HSU2_RATE               (*(volatile unsigned long *)(HS_UART2_BASE_ADDR + 0x10))

#define     HS_UART7_BASE_ADDR		0x4001C000
#define     HSU7_RX                 (*(volatile unsigned long *)(HS_UART7_BASE_ADDR + 0x00))
#define     HSU7_TX                 (*(volatile unsigned long *)(HS_UART7_BASE_ADDR + 0x00))
#define     HSU7_LEVEL              (*(volatile unsigned long *)(HS_UART7_BASE_ADDR + 0x04))
#define     HSU7_IIR                (*(volatile unsigned long *)(HS_UART7_BASE_ADDR + 0x08))
#define     HSU7_CTRL               (*(volatile unsigned long *)(HS_UART7_BASE_ADDR + 0x0C))
#define     HSU7_RATE               (*(volatile unsigned long *)(HS_UART7_BASE_ADDR + 0x10))

    /* 
     * SPI Interface 
     */
#define     SPI1_BASE_ADDR	        0x20088000
#define     SPI1_GLOBAL             (*(volatile unsigned long *)(SPI1_BASE_ADDR + 0x000))
#define     SPI1_CON	            (*(volatile unsigned long *)(SPI1_BASE_ADDR + 0x004))
#define     SPI1_FRM	            (*(volatile unsigned long *)(SPI1_BASE_ADDR + 0x008))
#define     SPI1_IER	            (*(volatile unsigned long *)(SPI1_BASE_ADDR + 0x00C))
#define     SPI1_STAT	            (*(volatile unsigned long *)(SPI1_BASE_ADDR + 0x010))
#define     SPI1_DAT	            (*(volatile unsigned long *)(SPI1_BASE_ADDR + 0x014))
#define     SPI1_TIM_CTRL	        (*(volatile unsigned long *)(SPI1_BASE_ADDR + 0x400))
#define     SPI1_TIM_COUNT	        (*(volatile unsigned long *)(SPI1_BASE_ADDR + 0x404))
#define     SPI1_TIM_STAT	        (*(volatile unsigned long *)(SPI1_BASE_ADDR + 0x408))

#define     SPI2_BASE_ADDR	        0x20090000
#define     SPI2_GLOBAL	            (*(volatile unsigned long *)(SPI2_BASE_ADDR + 0x000))
#define     SPI2_CON	            (*(volatile unsigned long *)(SPI2_BASE_ADDR + 0x004))
#define     SPI2_FRM	            (*(volatile unsigned long *)(SPI2_BASE_ADDR + 0x008))
#define     SPI2_IER	            (*(volatile unsigned long *)(SPI2_BASE_ADDR + 0x00C))
#define     SPI2_STAT	            (*(volatile unsigned long *)(SPI2_BASE_ADDR + 0x010))
#define     SPI2_DAT	            (*(volatile unsigned long *)(SPI2_BASE_ADDR + 0x014))
#define     SPI2_TIM_CTRL	        (*(volatile unsigned long *)(SPI2_BASE_ADDR + 0x400))
#define     SPI2_TIM_COUNT	        (*(volatile unsigned long *)(SPI2_BASE_ADDR + 0x404))
#define     SPI2_TIM_STAT	        (*(volatile unsigned long *)(SPI2_BASE_ADDR + 0x408))

    /* 
     * SSP Interface 
     */
#define     SSP0_BASE_ADDR	        0x20084000
#define     SSP0CR0                 (*(volatile unsigned long *)(SSP0_BASE_ADDR + 0x000))
#define     SSP0CR1  	            (*(volatile unsigned long *)(SSP0_BASE_ADDR + 0x004))
#define     SSP0DR   	            (*(volatile unsigned long *)(SSP0_BASE_ADDR + 0x008))
#define     SSP0SR  	            (*(volatile unsigned long *)(SSP0_BASE_ADDR + 0x00C))
#define     SSP0CPSR	            (*(volatile unsigned long *)(SSP0_BASE_ADDR + 0x010))
#define     SSP0IMSC	            (*(volatile unsigned long *)(SSP0_BASE_ADDR + 0x014))
#define     SSP0RIS             	(*(volatile unsigned long *)(SSP0_BASE_ADDR + 0x018))
#define     SSP0MIS     	        (*(volatile unsigned long *)(SSP0_BASE_ADDR + 0x01C))
#define     SSP0ICR             	(*(volatile unsigned long *)(SSP0_BASE_ADDR + 0x020))
#define     SSP0DMACR     	        (*(volatile unsigned long *)(SSP0_BASE_ADDR + 0x024))

#define     SSP1_BASE_ADDR	        0x2008C000
#define     SSP1CR0                 (*(volatile unsigned long *)(SSP1_BASE_ADDR + 0x000))
#define     SSP1CR1  	            (*(volatile unsigned long *)(SSP1_BASE_ADDR + 0x004))
#define     SSP1DR   	            (*(volatile unsigned long *)(SSP1_BASE_ADDR + 0x008))
#define     SSP1SR  	            (*(volatile unsigned long *)(SSP1_BASE_ADDR + 0x00C))
#define     SSP1CPSR	            (*(volatile unsigned long *)(SSP1_BASE_ADDR + 0x010))
#define     SSP1IMSC                (*(volatile unsigned long *)(SSP1_BASE_ADDR + 0x014))
#define     SSP1RIS              	(*(volatile unsigned long *)(SSP1_BASE_ADDR + 0x018))
#define     SSP1MIS     	        (*(volatile unsigned long *)(SSP1_BASE_ADDR + 0x01C))
#define     SSP1ICR                 (*(volatile unsigned long *)(SSP1_BASE_ADDR + 0x020))
#define     SSP1DMACR     	        (*(volatile unsigned long *)(SSP1_BASE_ADDR + 0x024))

    /* 
     * I2C Interface 1 
     */
#define     I2C1_BASE_ADDR		    0x400A0000
#define     I2C1_RX		            (*(volatile unsigned long *)(I2C1_BASE_ADDR + 0x00))
#define     I2C1_TX		            (*(volatile unsigned long *)(I2C1_BASE_ADDR + 0x00))
#define     I2C1_STAT	            (*(volatile unsigned long *)(I2C1_BASE_ADDR + 0x04))
#define     I2C1_CTRL	            (*(volatile unsigned long *)(I2C1_BASE_ADDR + 0x08))
#define     I2C1_CLKHI	            (*(volatile unsigned long *)(I2C1_BASE_ADDR + 0x0C))
#define     I2C1_CLKLO	            (*(volatile unsigned long *)(I2C1_BASE_ADDR + 0x10))
#define     I2C1_ADR	            (*(volatile unsigned long *)(I2C1_BASE_ADDR + 0x14))
#define     I2C1_RXFL	            (*(volatile unsigned long *)(I2C1_BASE_ADDR + 0x18))
#define     I2C1_TXFL	            (*(volatile unsigned long *)(I2C1_BASE_ADDR + 0x1C))
#define     I2C1_RXB	            (*(volatile unsigned long *)(I2C1_BASE_ADDR + 0x20))
#define     I2C1_TXB	            (*(volatile unsigned long *)(I2C1_BASE_ADDR + 0x24))
#define     I2C1_S_TX	            (*(volatile unsigned long *)(I2C1_BASE_ADDR + 0x28))
#define     I2C1_S_TXFL	            (*(volatile unsigned long *)(I2C1_BASE_ADDR + 0x2C))

    /* 
     * I2C Interface 2 
     */
#define     I2C2_BASE_ADDR		    0x400A8000
#define     I2C2_RX		            (*(volatile unsigned long *)(I2C2_BASE_ADDR + 0x00))
#define     I2C2_TX		            (*(volatile unsigned long *)(I2C2_BASE_ADDR + 0x00))
#define     I2C2_STAT	            (*(volatile unsigned long *)(I2C2_BASE_ADDR + 0x04))
#define     I2C2_CTRL	            (*(volatile unsigned long *)(I2C2_BASE_ADDR + 0x08))
#define     I2C2_CLKHI	            (*(volatile unsigned long *)(I2C2_BASE_ADDR + 0x0C))
#define     I2C2_CLKLO	            (*(volatile unsigned long *)(I2C2_BASE_ADDR + 0x10))
#define     I2C2_ADR	            (*(volatile unsigned long *)(I2C2_BASE_ADDR + 0x14))
#define     I2C2_RXFL	            (*(volatile unsigned long *)(I2C2_BASE_ADDR + 0x18))
#define     I2C2_TXFL	            (*(volatile unsigned long *)(I2C2_BASE_ADDR + 0x1C))
#define     I2C2_RXB	            (*(volatile unsigned long *)(I2C2_BASE_ADDR + 0x20))
#define     I2C2_TXB	            (*(volatile unsigned long *)(I2C2_BASE_ADDR + 0x24))
#define     I2C2_S_TX	            (*(volatile unsigned long *)(I2C2_BASE_ADDR + 0x28))
#define     I2C2_S_TXFL	            (*(volatile unsigned long *)(I2C2_BASE_ADDR + 0x2C)) 

    /*  
     * I2S  
     */
#define     I2S0_BASE_ADDR          0x20094000
#define     I2S0DAO                 (*(volatile unsigned long *)(I2S1_BASE_ADDR + 0x00))
#define     I2S0DAI                 (*(volatile unsigned long *)(I2S1_BASE_ADDR + 0x04))
#define     I2S0TXFIFO              (*(volatile unsigned long *)(I2S1_BASE_ADDR + 0x08))
#define     I2S0RXFIFO              (*(volatile unsigned long *)(I2S1_BASE_ADDR + 0x0C))
#define     I2S0STATE               (*(volatile unsigned long *)(I2S1_BASE_ADDR + 0x10))
#define     I2S0DMA0                (*(volatile unsigned long *)(I2S1_BASE_ADDR + 0x14))
#define     I2S0DMA1                (*(volatile unsigned long *)(I2S1_BASE_ADDR + 0x18))
#define     I2S0IRQ                 (*(volatile unsigned long *)(I2S1_BASE_ADDR + 0x1C))
#define     I2S0TXRATE              (*(volatile unsigned long *)(I2S1_BASE_ADDR + 0x20))
#define     I2S0RXRATE              (*(volatile unsigned long *)(I2S1_BASE_ADDR + 0x24))

#define     I2S1_BASE_ADDR          0x2009C000
#define     I2S1DAO                 (*(volatile unsigned long *)(I2S1_BASE_ADDR + 0x00))
#define     I2S1DAI                 (*(volatile unsigned long *)(I2S1_BASE_ADDR + 0x04))
#define     I2S1TXFIFO              (*(volatile unsigned long *)(I2S1_BASE_ADDR + 0x08))
#define     I2S1RXFIFO              (*(volatile unsigned long *)(I2S1_BASE_ADDR + 0x0C))
#define     I2S1STATE               (*(volatile unsigned long *)(I2S1_BASE_ADDR + 0x10))
#define     I2S1DMA0                (*(volatile unsigned long *)(I2S1_BASE_ADDR + 0x14))
#define     I2S1DMA1                (*(volatile unsigned long *)(I2S1_BASE_ADDR + 0x18))
#define     I2S1IRQ                 (*(volatile unsigned long *)(I2S1_BASE_ADDR + 0x1C))
#define     I2S1TXRATE              (*(volatile unsigned long *)(I2S1_BASE_ADDR + 0x20))
#define     I2S1RXRATE              (*(volatile unsigned long *)(I2S1_BASE_ADDR + 0x24)) 

    /* 
     * Standard timers 0/1/2/3        
     */
#define     TIMER0_BASE_ADDR        0x40044000
#define     T0IR                    (*(volatile unsigned long *)(TIMER0_BASE_ADDR + 0x000))
#define     T0TCR                   (*(volatile unsigned long *)(TIMER0_BASE_ADDR + 0x004))
#define     T0TC                    (*(volatile unsigned long *)(TIMER0_BASE_ADDR + 0x008))
#define     T0PR                    (*(volatile unsigned long *)(TIMER0_BASE_ADDR + 0x00C))
#define     T0PC                    (*(volatile unsigned long *)(TIMER0_BASE_ADDR + 0x010))
#define     T0MCR                   (*(volatile unsigned long *)(TIMER0_BASE_ADDR + 0x014))
#define     T0MR0                   (*(volatile unsigned long *)(TIMER0_BASE_ADDR + 0x018))
#define     T0MR1                   (*(volatile unsigned long *)(TIMER0_BASE_ADDR + 0x01C))
#define     T0MR2                   (*(volatile unsigned long *)(TIMER0_BASE_ADDR + 0x020))
#define     T0MR3                   (*(volatile unsigned long *)(TIMER0_BASE_ADDR + 0x024))
#define     T0CCR                   (*(volatile unsigned long *)(TIMER0_BASE_ADDR + 0x028))
#define     T0CR0                   (*(volatile unsigned long *)(TIMER0_BASE_ADDR + 0x02C))
#define     T0CR1                   (*(volatile unsigned long *)(TIMER0_BASE_ADDR + 0x030))
#define     T0CR2                   (*(volatile unsigned long *)(TIMER0_BASE_ADDR + 0x034))
#define     T0CR3                   (*(volatile unsigned long *)(TIMER0_BASE_ADDR + 0x038))
#define     T0EMR                   (*(volatile unsigned long *)(TIMER0_BASE_ADDR + 0x03C))
#define     T0CTCR                  (*(volatile unsigned long *)(TIMER0_BASE_ADDR + 0x070))


#define     TIMER1_BASE_ADDR         0x4004C000
#define     T1IR                    (*(volatile unsigned long *)(TIMER1_BASE_ADDR + 0x000))
#define     T1TCR                   (*(volatile unsigned long *)(TIMER1_BASE_ADDR + 0x004))
#define     T1TC                    (*(volatile unsigned long *)(TIMER1_BASE_ADDR + 0x008))
#define     T1PR                    (*(volatile unsigned long *)(TIMER1_BASE_ADDR + 0x00C))
#define     T1PC                    (*(volatile unsigned long *)(TIMER1_BASE_ADDR + 0x010))
#define     T1MCR                   (*(volatile unsigned long *)(TIMER1_BASE_ADDR + 0x014))
#define     T1MR0                   (*(volatile unsigned long *)(TIMER1_BASE_ADDR + 0x018))
#define     T1MR1                   (*(volatile unsigned long *)(TIMER1_BASE_ADDR + 0x01C))
#define     T1MR2                   (*(volatile unsigned long *)(TIMER1_BASE_ADDR + 0x020))
#define     T1MR3                   (*(volatile unsigned long *)(TIMER1_BASE_ADDR + 0x024))
#define     T1CCR                   (*(volatile unsigned long *)(TIMER1_BASE_ADDR + 0x028))
#define     T1CR0                   (*(volatile unsigned long *)(TIMER1_BASE_ADDR + 0x02C))
#define     T1CR1                   (*(volatile unsigned long *)(TIMER1_BASE_ADDR + 0x030))
#define     T1CR2                   (*(volatile unsigned long *)(TIMER1_BASE_ADDR + 0x034))
#define     T1CR3                   (*(volatile unsigned long *)(TIMER1_BASE_ADDR + 0x038))
#define     T1EMR                   (*(volatile unsigned long *)(TIMER1_BASE_ADDR + 0x03C))
#define     T1CTCR                  (*(volatile unsigned long *)(TIMER1_BASE_ADDR + 0x070))

#define     TIMER2_BASE_ADDR        0x40058000
#define     T2IR                    (*(volatile unsigned long *)(TIMER2_BASE_ADDR + 0x000))
#define     T2TCR                   (*(volatile unsigned long *)(TIMER2_BASE_ADDR + 0x004))
#define     T2TC                    (*(volatile unsigned long *)(TIMER2_BASE_ADDR + 0x008))
#define     T2PR                    (*(volatile unsigned long *)(TIMER2_BASE_ADDR + 0x00C))
#define     T2PC                    (*(volatile unsigned long *)(TIMER2_BASE_ADDR + 0x010))
#define     T2MCR                   (*(volatile unsigned long *)(TIMER2_BASE_ADDR + 0x014))
#define     T2MR0                   (*(volatile unsigned long *)(TIMER2_BASE_ADDR + 0x018))
#define     T2MR1                   (*(volatile unsigned long *)(TIMER2_BASE_ADDR + 0x01C))
#define     T2MR2                   (*(volatile unsigned long *)(TIMER2_BASE_ADDR + 0x020))
#define     T2MR3                   (*(volatile unsigned long *)(TIMER2_BASE_ADDR + 0x024))
#define     T2CCR                   (*(volatile unsigned long *)(TIMER2_BASE_ADDR + 0x028))
#define     T2CR0                   (*(volatile unsigned long *)(TIMER2_BASE_ADDR + 0x02C))
#define     T2CR1                   (*(volatile unsigned long *)(TIMER2_BASE_ADDR + 0x030))
#define     T2CR2                   (*(volatile unsigned long *)(TIMER2_BASE_ADDR + 0x034))
#define     T2CR3                   (*(volatile unsigned long *)(TIMER2_BASE_ADDR + 0x038))
#define     T2EMR                   (*(volatile unsigned long *)(TIMER2_BASE_ADDR + 0x03C))
#define     T2CTCR                  (*(volatile unsigned long *)(TIMER2_BASE_ADDR + 0x070))

#define     TIMER3_BASE_ADDR         0x40060000
#define     T3IR                    (*(volatile unsigned long *)(TIMER3_BASE_ADDR + 0x000))
#define     T3TCR                   (*(volatile unsigned long *)(TIMER3_BASE_ADDR + 0x004))
#define     T3TC                    (*(volatile unsigned long *)(TIMER3_BASE_ADDR + 0x008))
#define     T3PR                    (*(volatile unsigned long *)(TIMER3_BASE_ADDR + 0x00C))
#define     T3PC                    (*(volatile unsigned long *)(TIMER3_BASE_ADDR + 0x010))
#define     T3MCR                   (*(volatile unsigned long *)(TIMER3_BASE_ADDR + 0x014))
#define     T3MR0                   (*(volatile unsigned long *)(TIMER3_BASE_ADDR + 0x018))
#define     T3MR1                   (*(volatile unsigned long *)(TIMER3_BASE_ADDR + 0x01C))
#define     T3MR2                   (*(volatile unsigned long *)(TIMER3_BASE_ADDR + 0x020))
#define     T3MR3                   (*(volatile unsigned long *)(TIMER3_BASE_ADDR + 0x024))
#define     T3CCR                   (*(volatile unsigned long *)(TIMER3_BASE_ADDR + 0x028))
#define     T3CR0                   (*(volatile unsigned long *)(TIMER3_BASE_ADDR + 0x02C))
#define     T3CR1                   (*(volatile unsigned long *)(TIMER3_BASE_ADDR + 0x030))
#define     T3CR2                   (*(volatile unsigned long *)(TIMER3_BASE_ADDR + 0x034))
#define     T3CR3                   (*(volatile unsigned long *)(TIMER3_BASE_ADDR + 0x038))
#define     T3EMR                   (*(volatile unsigned long *)(TIMER3_BASE_ADDR + 0x03C))
#define     T3CTCR                  (*(volatile unsigned long *)(TIMER3_BASE_ADDR + 0x070))


    /* 
     * High-speed Timer  
     */
#define     HSTIM_BASE_ADDR		    0x40038000
#define     HSTIM_INT	            (*(volatile unsigned long *)(HSTIM_BASE_ADDR + 0x00))
#define     HSTIM_CTRL	            (*(volatile unsigned long *)(HSTIM_BASE_ADDR + 0x04))
#define     HSTIM_COUNTER	        (*(volatile unsigned long *)(HSTIM_BASE_ADDR + 0x08))
#define     HSTIM_PMATCH	        (*(volatile unsigned long *)(HSTIM_BASE_ADDR + 0x0C))
#define     HSTIM_PCOUNT	        (*(volatile unsigned long *)(HSTIM_BASE_ADDR + 0x10))
#define     HSTIM_MCTRL	            (*(volatile unsigned long *)(HSTIM_BASE_ADDR + 0x14))
#define     HSTIM_MATCH0	        (*(volatile unsigned long *)(HSTIM_BASE_ADDR + 0x18))
#define     HSTIM_MATCH1	        (*(volatile unsigned long *)(HSTIM_BASE_ADDR + 0x1C))
#define     HSTIM_MATCH2	        (*(volatile unsigned long *)(HSTIM_BASE_ADDR + 0x20))
#define     HSTIM_CCR	            (*(volatile unsigned long *)(HSTIM_BASE_ADDR + 0x28))
#define     HSTIM_CR0	            (*(volatile unsigned long *)(HSTIM_BASE_ADDR + 0x2C))
#define     HSTIM_CR1	            (*(volatile unsigned long *)(HSTIM_BASE_ADDR + 0x30))


    /* 
     * Millisecond Timer 
     */
#define     MSTIM_BASE_ADDR		    0x40034000
#define     MSTIM_INT	            (*(volatile unsigned long *)(MSTIM_BASE_ADDR + 0x00))
#define     MSTIM_CTRL	            (*(volatile unsigned long *)(MSTIM_BASE_ADDR + 0x04))
#define     MSTIM_COUNTER	        (*(volatile unsigned long *)(MSTIM_BASE_ADDR + 0x08))
#define     MSTIM_MCTRL	            (*(volatile unsigned long *)(MSTIM_BASE_ADDR + 0x14))
#define     MSTIM_MATCH0	        (*(volatile unsigned long *)(MSTIM_BASE_ADDR + 0x18))
#define     MSTIM_MATCH1	        (*(volatile unsigned long *)(MSTIM_BASE_ADDR + 0x1C))


    /* 
     * Real Time Clock 
     */
#define     RTC_BASE_ADDR		    0x40024000
#define     RTC_UCOUNT	            (*(volatile unsigned long *)(RTC_BASE_ADDR + 0x00))
#define     RTC_DCOUNT	            (*(volatile unsigned long *)(RTC_BASE_ADDR + 0x04))
#define     RTC_MATCH0	            (*(volatile unsigned long *)(RTC_BASE_ADDR + 0x08))
#define     RTC_MATCH1	            (*(volatile unsigned long *)(RTC_BASE_ADDR + 0x0C))
#define     RTC_CTRL	            (*(volatile unsigned long *)(RTC_BASE_ADDR + 0x10))
#define     RTC_INTSTAT             (*(volatile unsigned long *)(RTC_BASE_ADDR + 0x14))
#define     RTC_KEY		            (*(volatile unsigned long *)(RTC_BASE_ADDR + 0x18))
#define     RTC_SRAM	            (*(volatile unsigned long *)(RTC_BASE_ADDR + 0x80))


    /* 
     * Watchdog timer 
     */
#define     WDTIM_BASE_ADDR		    0x4003C000
#define     WDTIM_INT	            (*(volatile unsigned long *)(WDTIM_BASE_ADDR + 0x00))
#define     WDTIM_CTRL	            (*(volatile unsigned long *)(WDTIM_BASE_ADDR + 0x04))
#define     WDTIM_COUNTER           (*(volatile unsigned long *)(WDTIM_BASE_ADDR + 0x08))
#define     WDTIM_MCTRL	            (*(volatile unsigned long *)(WDTIM_BASE_ADDR + 0x0C))
#define     WDTIM_MATCH0	        (*(volatile unsigned long *)(WDTIM_BASE_ADDR + 0x10))
#define     WDTIM_EMR	            (*(volatile unsigned long *)(WDTIM_BASE_ADDR + 0x14))
#define     WDTIM_PULSE	            (*(volatile unsigned long *)(WDTIM_BASE_ADDR + 0x18))
#define     WDTIM_RES	            (*(volatile unsigned long *)(WDTIM_BASE_ADDR + 0x1C))

    /* 
     * motor controller registers 
     */
#define     MOTOR_BASE_ADDR         0x400E8000
#define     MCCON                   (*(volatile unsigned long *)(MOTOR_BASE_ADDR + 0x00))
#define     MCCON_SET               (*(volatile unsigned long *)(MOTOR_BASE_ADDR + 0x04))    
#define     MCCON_CLR               (*(volatile unsigned long *)(MOTOR_BASE_ADDR + 0x08))    
#define     MCCAPCON                (*(volatile unsigned long *)(MOTOR_BASE_ADDR + 0x0C))   
#define     MCCAPCON_SET            (*(volatile unsigned long *)(MOTOR_BASE_ADDR + 0x10))       
#define     MCCAPCON_CLR            (*(volatile unsigned long *)(MOTOR_BASE_ADDR + 0x14))      
#define     MCTC0                   (*(volatile unsigned long *)(MOTOR_BASE_ADDR + 0x18))
#define     MCTC1                   (*(volatile unsigned long *)(MOTOR_BASE_ADDR + 0x1C))
#define     MCTC2                   (*(volatile unsigned long *)(MOTOR_BASE_ADDR + 0x20))
#define     MCLIM0                  (*(volatile unsigned long *)(MOTOR_BASE_ADDR + 0x24)) 
#define     MCLIM1                  (*(volatile unsigned long *)(MOTOR_BASE_ADDR + 0x28)) 
#define     MCLIM2                  (*(volatile unsigned long *)(MOTOR_BASE_ADDR + 0x2C)) 
#define     MCMAT0                  (*(volatile unsigned long *)(MOTOR_BASE_ADDR + 0x30))
#define     MCMAT1                  (*(volatile unsigned long *)(MOTOR_BASE_ADDR + 0x34)) 
#define     MCMAT2                  (*(volatile unsigned long *)(MOTOR_BASE_ADDR + 0x38)) 
#define     MCDT                    (*(volatile unsigned long *)(MOTOR_BASE_ADDR + 0x3C))
#define     MCCP                    (*(volatile unsigned long *)(MOTOR_BASE_ADDR + 0x40)) 
#define     MCCAP0                  (*(volatile unsigned long *)(MOTOR_BASE_ADDR + 0x44))
#define     MCCAP1                  (*(volatile unsigned long *)(MOTOR_BASE_ADDR + 0x48)) 
#define     MCCAP2                  (*(volatile unsigned long *)(MOTOR_BASE_ADDR + 0x4C))
#define     MCINTEN                 (*(volatile unsigned long *)(MOTOR_BASE_ADDR + 0x50)) 
#define     MCINTEN_SET             (*(volatile unsigned long *)(MOTOR_BASE_ADDR + 0x54))      
#define     MCINTEN_CLR             (*(volatile unsigned long *)(MOTOR_BASE_ADDR + 0x58))     
#define     MCINTFLAG               (*(volatile unsigned long *)(MOTOR_BASE_ADDR + 0x68))    
#define     MCINTFLAG_SET           (*(volatile unsigned long *)(MOTOR_BASE_ADDR + 0x6C))        
#define     MCINTFLAG_CLR           (*(volatile unsigned long *)(MOTOR_BASE_ADDR + 0x70))        
#define     MCCAP_CLR               (*(volatile unsigned long *)(MOTOR_BASE_ADDR + 0x74))   

    /* 
     * PWM controller registers    
     */
#define     PWM_BASE_ADDR           0x4005C000
#define     PWM1_CTRL               (*(volatile unsigned long *)(PWM_BASE_ADDR + 0x00))                  
#define     PWM2_CTRL               (*(volatile unsigned long *)(PWM_BASE_ADDR + 0x04))                 

    /* 
     * IO  Controller registers 
     */
#define     MUX_BASE_ADDR           0x40028000
#define     P_MUX_SET               (*(volatile unsigned long *)(MUX_BASE_ADDR + 0x100))
#define     P_MUX_CLR               (*(volatile unsigned long *)(MUX_BASE_ADDR + 0x104))
#define     P_MUX_STATE             (*(volatile unsigned long *)(MUX_BASE_ADDR + 0x108))
#define     P0_MUX_SET              (*(volatile unsigned long *)(MUX_BASE_ADDR + 0x120))
#define     P0_MUX_CLR              (*(volatile unsigned long *)(MUX_BASE_ADDR + 0x124))
#define     P0_MUX_STATE            (*(volatile unsigned long *)(MUX_BASE_ADDR + 0x128))
#define     P1_MUX_SET              (*(volatile unsigned long *)(MUX_BASE_ADDR + 0x130))
#define     P1_MUX_CLR              (*(volatile unsigned long *)(MUX_BASE_ADDR + 0x134))
#define     P1_MUX_STATE            (*(volatile unsigned long *)(MUX_BASE_ADDR + 0x138))
#define     P2_MUX_SET              (*(volatile unsigned long *)(MUX_BASE_ADDR + 0x028))
#define     P2_MUX_CLR              (*(volatile unsigned long *)(MUX_BASE_ADDR + 0x02C))
#define     P2_MUX_STATE            (*(volatile unsigned long *)(MUX_BASE_ADDR + 0x030))
#define     P3_MUX_SET              (*(volatile unsigned long *)(MUX_BASE_ADDR + 0x110))
#define     P3_MUX_CLR              (*(volatile unsigned long *)(MUX_BASE_ADDR + 0x114))
#define     P3_MUX_STATE            (*(volatile unsigned long *)(MUX_BASE_ADDR + 0x118))

#define     P0_BASE_ADDR            0x40028040
#define     P0_INP_STATE            (*(volatile unsigned long *)(P0_BASE_ADDR + 0x00))
#define     P0_OUTP_SET             (*(volatile unsigned long *)(P0_BASE_ADDR + 0x04))
#define     P0_OUTP_CLR             (*(volatile unsigned long *)(P0_BASE_ADDR + 0x08))
#define     P0_OUTP_STATE           (*(volatile unsigned long *)(P0_BASE_ADDR + 0x0C))
#define     P0_DIR_SET              (*(volatile unsigned long *)(P0_BASE_ADDR + 0x10))
#define     P0_DIR_CLR              (*(volatile unsigned long *)(P0_BASE_ADDR + 0x14))
#define     P0_DIR_STATE            (*(volatile unsigned long *)(P0_BASE_ADDR + 0x18))

#define     P1_BASE_ADDR            0x40028060
#define     P1_INP_STATE            (*(volatile unsigned long *)(P1_BASE_ADDR + 0x00))
#define     P1_OUTP_SET             (*(volatile unsigned long *)(P1_BASE_ADDR + 0x04))
#define     P1_OUTP_CLR             (*(volatile unsigned long *)(P1_BASE_ADDR + 0x08))
#define     P1_OUTP_STATE           (*(volatile unsigned long *)(P1_BASE_ADDR + 0x0C))
#define     P1_DIR_SET              (*(volatile unsigned long *)(P1_BASE_ADDR + 0x10))
#define     P1_DIR_CLR              (*(volatile unsigned long *)(P1_BASE_ADDR + 0x14))
#define     P1_DIR_STATE            (*(volatile unsigned long *)(P1_BASE_ADDR + 0x18))

#define     P2_BASE_ADDR            0x40028010
#define     P2_INP_STATE            (*(volatile unsigned long *)(P2_BASE_ADDR + 0x0C))
#define     P2_OUTP_SET             (*(volatile unsigned long *)(P2_BASE_ADDR + 0x10))
#define     P2_OUTP_CLR             (*(volatile unsigned long *)(P2_BASE_ADDR + 0x14))
#define     P2_DIR_SET              (*(volatile unsigned long *)(P2_BASE_ADDR + 0x00))
#define     P2_DIR_CLR              (*(volatile unsigned long *)(P2_BASE_ADDR + 0x04))
#define     P2_DIR_STATE            (*(volatile unsigned long *)(P2_BASE_ADDR + 0x08))

#define     P3_BASE_ADDR            0x40028000
#define     P3_INP_STATE            (*(volatile unsigned long *)(P3_BASE_ADDR + 0x00))
#define     P3_OUTP_SET             (*(volatile unsigned long *)(P3_BASE_ADDR + 0x04))
#define     P3_OUTP_CLR             (*(volatile unsigned long *)(P3_BASE_ADDR + 0x08))
#define     P3_OUTP_STATE           (*(volatile unsigned long *)(P3_BASE_ADDR + 0x0C))

#endif                                                                  /* __LPC3250_H                  */

