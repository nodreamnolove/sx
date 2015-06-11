;/****************************************Copyright (c***************************************************
;**                         Guangzhou ZHIYUAN electronics Co.,LTD.                               
;**                                     
;**                               http://www.embedtools.com
;**
;**--------------File Info--------------------------------------------------------------------------------
;** File name: 		    LPC2400.inc
;** Last modified Date: 2008-04-29 
;** Last Version: 		
;** Descriptions: 		
;**-------------------------------------------------------------------------------------------------------
;** Created by: 		LinEnqiang
;** Created date:   	2008-12-15
;** Version:			1.0
;** Descriptions: 		The original version
;**
;**-------------------------------------------------------------------------------------------------------
;** Modified by: 		
;** Modified date:		
;** Version:			
;** Descriptions:
;********************************************************************************************************/

SYSCTRL_BASE_ADDR	        	EQU     0x40004000
BOOT_MAP	                	EQU     (CLK_PM_BASE_ADDR + 0x14)

CLK_PM_BASE_ADDR	    		EQU     0x40004000

PWR_CTRL	               	 	EQU     (CLK_PM_BASE_ADDR + 0x44)
OSC_CTRL	                	EQU     (CLK_PM_BASE_ADDR + 0x4C)
SYSCLK_CTRL	                	EQU     (CLK_PM_BASE_ADDR + 0x50)
PLL397_CTRL	                	EQU     (CLK_PM_BASE_ADDR + 0x48)	
HCLKPLL_CTRL	            	EQU     (CLK_PM_BASE_ADDR + 0x58)
HCLKDIV_CTRL	            	EQU     (CLK_PM_BASE_ADDR + 0x40)
TEST_CLK	                	EQU     (CLK_PM_BASE_ADDR + 0xA4)
AUTOCLK_CTRL	                EQU     (CLK_PM_BASE_ADDR + 0xEC)

START_ER_INT	                EQU     (CLK_PM_BASE_ADDR + 0x20)
START_ER_PIN	                EQU     (CLK_PM_BASE_ADDR + 0x30)
START_RSR_INT	                EQU     (CLK_PM_BASE_ADDR + 0x24)
START_RSR_PIN	                EQU     (CLK_PM_BASE_ADDR + 0x34)
START_SR_INT	                EQU     (CLK_PM_BASE_ADDR + 0x28)
START_SR_PIN	                EQU     (CLK_PM_BASE_ADDR + 0x38)
START_APR_INT	                EQU     (CLK_PM_BASE_ADDR + 0x2C)
START_APR_PIN	                EQU     (CLK_PM_BASE_ADDR + 0x3C)

DMACLK_CTRL	                	EQU     (CLK_PM_BASE_ADDR + 0xE8)
USBDIV_CTRL                 	EQU     (CLK_PM_BASE_ADDR + 0x1C) 
USBCLK_CTRL	               	 	EQU     (CLK_PM_BASE_ADDR + 0x64)
MS_CTRL		                	EQU     (CLK_PM_BASE_ADDR + 0x80)
TIMCLK_CTRL	               	 	EQU     (CLK_PM_BASE_ADDR + 0xBC)
SPI_CTRL	                	EQU     (CLK_PM_BASE_ADDR + 0xC4)
FLASHCLK_CTRL	                EQU     (CLK_PM_BASE_ADDR + 0xC8)
MACCLK_CTRL	                	EQU     (CLK_PM_BASE_ADDR + 0x90)
LCDCLK_CTRL	                	EQU     (CLK_PM_BASE_ADDR + 0x54)
I2S_CTRL	                	EQU     (CLK_PM_BASE_ADDR + 0x7c)
SSP_CTRL	                	EQU     (CLK_PM_BASE_ADDR + 0x78)
;SPI_CTRL	                	EQU     (CLK_PM_BASE_ADDR + 0xC4)
I2CCLK_CTRL	                	EQU     (CLK_PM_BASE_ADDR + 0xAC)
TIMCLK_CTRL1                    EQU     (CLK_PM_BASE_ADDR + 0xC0)
ADCLK_CTRL	                	EQU     (CLK_PM_BASE_ADDR + 0xB4)
ADCLK_CTRL1	                	EQU     (CLK_PM_BASE_ADDR + 0x60)
KEYCLK_CTRL	                	EQU     (CLK_PM_BASE_ADDR + 0xB0)
PWMCLK_CTRL	                	EQU     (CLK_PM_BASE_ADDR + 0xB8)
UARTCLK_CTRL	                EQU     (CLK_PM_BASE_ADDR + 0xE4)
                                                            
MIC_BASE_ADDR	                EQU     0x40008000
MIC_ER		                	EQU     (MIC_BASE_ADDR + 0x00)
MIC_RSR		                	EQU     (MIC_BASE_ADDR + 0x04)
MIC_SR		                	EQU     (MIC_BASE_ADDR + 0x08)
MIC_APR		                	EQU     (MIC_BASE_ADDR + 0x0C)	
MIC_ATR		                	EQU     (MIC_BASE_ADDR + 0x10)
MIC_ITR		                	EQU     (MIC_BASE_ADDR + 0x14)

SIC1_BASE_ADDR	                EQU     0x4000C000
SIC1_ER		                	EQU     (SIC1_BASE_ADDR + 0x00)
SIC1_RSR	                	EQU     (SIC1_BASE_ADDR + 0x04)
SIC1_SR		                	EQU     (SIC1_BASE_ADDR + 0x08)
SIC1_APR	                	EQU     (SIC1_BASE_ADDR + 0x0C)
SIC1_ATR	                	EQU     (SIC1_BASE_ADDR + 0x10)
SIC1_ITR	                	EQU     (SIC1_BASE_ADDR + 0x14)

SIC2_BASE_ADDR	                EQU     0x40010000
SIC2_ER		                	EQU     (SIC2_BASE_ADDR + 0x00)
SIC2_RSR	                	EQU     (SIC2_BASE_ADDR + 0x04)
SIC2_SR		                	EQU     (SIC2_BASE_ADDR + 0x08)
SIC2_APR	                	EQU     (SIC2_BASE_ADDR + 0x0C)
SIC2_ATR	                	EQU     (SIC2_BASE_ADDR + 0x10)
SIC2_ITR	                	EQU     (SIC2_BASE_ADDR + 0x14)

SWI_BASE_ADDR	                EQU     0x40004000
SW_INT		                	EQU     (SWI_BASE_ADDR + 0xA8)

DMA_GENERAL_BASE_ADDR           EQU     0x31000000
DMACIntStat                     EQU     (DMA_GENERAL_BASE_ADDR + 0x00)
DMACIntTCStat                   EQU     (DMA_GENERAL_BASE_ADDR + 0x04)
DMACIntTCClear                  EQU     (DMA_GENERAL_BASE_ADDR + 0x08)
DMACIntErrStat                  EQU     (DMA_GENERAL_BASE_ADDR + 0x0C)
DMACIntErrClr                   EQU     (DMA_GENERAL_BASE_ADDR + 0x10)
DMACRawIntTCStat                EQU     (DMA_GENERAL_BASE_ADDR + 0x14)
DMACRawIntErrStat               EQU     (DMA_GENERAL_BASE_ADDR + 0x18)
DMACEnbldChns                   EQU     (DMA_GENERAL_BASE_ADDR + 0x1C)
DMACSoftBReq                    EQU     (DMA_GENERAL_BASE_ADDR + 0x20)
DMACSoftSReq                    EQU     (DMA_GENERAL_BASE_ADDR + 0x24)
DMACSoftLBReq                   EQU     (DMA_GENERAL_BASE_ADDR + 0x28)
DMACSoftLSReq                   EQU     (DMA_GENERAL_BASE_ADDR + 0x2C)
DMACConfig                      EQU     (DMA_GENERAL_BASE_ADDR + 0x30)

DMA_CHAN0_BASE_ADDR             EQU     0x31000100
DMACC0SrcAddr                   EQU     (DMA_CHAN0_BASE_ADDR + 0x00)
DMACC0DestAddr                  EQU     (DMA_CHAN0_BASE_ADDR + 0x04)
DMACC0LLI                       EQU     (DMA_CHAN0_BASE_ADDR + 0x08)
DMACC0Control                   EQU     (DMA_CHAN0_BASE_ADDR + 0x0C)
DMACC0Config                    EQU     (DMA_CHAN0_BASE_ADDR + 0x10)

DMA_CHAN1_BASE_ADDR             EQU     0x31000120
DMACC1SrcAddr                   EQU     (DMA_CHAN1_BASE_ADDR + 0x00)
DMACC1DestAddr                  EQU     (DMA_CHAN1_BASE_ADDR + 0x04)
DMACC1LLI                       EQU     (DMA_CHAN1_BASE_ADDR + 0x08)
DMACC1Control                   EQU     (DMA_CHAN1_BASE_ADDR + 0x0C)
DMACC1Config                    EQU     (DMA_CHAN1_BASE_ADDR + 0x10)

DMA_CHAN2_BASE_ADDR             EQU     0x31000140
DMACC2SrcAddr                   EQU     (DMA_CHAN2_BASE_ADDR + 0x00)
DMACC2DestAddr                  EQU     (DMA_CHAN2_BASE_ADDR + 0x04)
DMACC2LLI                       EQU     (DMA_CHAN2_BASE_ADDR + 0x08)
DMACC2Control                   EQU     (DMA_CHAN2_BASE_ADDR + 0x0C)
DMACC2Config                    EQU     (DMA_CHAN2_BASE_ADDR + 0x10)

DMA_CHAN3_BASE_ADDR             EQU     0x31000160
DMACC3SrcAddr                   EQU     (DMA_CHAN3_BASE_ADDR + 0x00)
DMACC3DestAddr                  EQU     (DMA_CHAN3_BASE_ADDR + 0x04)
DMACC3LLI                       EQU     (DMA_CHAN3_BASE_ADDR + 0x08)
DMACC3Control                   EQU     (DMA_CHAN3_BASE_ADDR + 0x0C)
DMACC3Config                    EQU     (DMA_CHAN3_BASE_ADDR + 0x10)

DMA_CHAN4_BASE_ADDR             EQU     0x31000180
DMACC4SrcAddr                   EQU     (DMA_CHAN4_BASE_ADDR + 0x00)
DMACC4DestAddr                  EQU     (DMA_CHAN4_BASE_ADDR + 0x04)
DMACC4LLI                       EQU     (DMA_CHAN4_BASE_ADDR + 0x08)
DMACC4Control                   EQU     (DMA_CHAN4_BASE_ADDR + 0x0C)
DMACC4Config                    EQU     (DMA_CHAN4_BASE_ADDR + 0x10)

DMA_CHAN5_BASE_ADDR             EQU     0x310001A0
DMACC5SrcAddr                   EQU     (DMA_CHAN5_BASE_ADDR + 0x00)
DMACC5DestAddr                  EQU     (DMA_CHAN5_BASE_ADDR + 0x04)
DMACC5LLI                       EQU     (DMA_CHAN5_BASE_ADDR + 0x08)
DMACC5Control                   EQU     (DMA_CHAN5_BASE_ADDR + 0x0C)
DMACC5Config                    EQU     (DMA_CHAN5_BASE_ADDR + 0x10)

DMA_CHAN6_BASE_ADDR             EQU     0x310001C0
DMACC6SrcAddr                   EQU     (DMA_CHAN6_BASE_ADDR + 0x00)
DMACC6DestAddr                  EQU     (DMA_CHAN6_BASE_ADDR + 0x04)
DMACC6LLI                       EQU     (DMA_CHAN6_BASE_ADDR + 0x08)
DMACC6Control                   EQU     (DMA_CHAN6_BASE_ADDR + 0x0C)
DMACC6Config                    EQU     (DMA_CHAN6_BASE_ADDR + 0x10)

DMA_CHAN7_BASE_ADDR             EQU     0x310001E0
DMACC7SrcAddr                   EQU     (DMA_CHAN7_BASE_ADDR + 0x00)
DMACC7DestAddr                  EQU     (DMA_CHAN7_BASE_ADDR + 0x04)
DMACC7LLI                       EQU     (DMA_CHAN7_BASE_ADDR + 0x08)
DMACC7Control                   EQU     (DMA_CHAN7_BASE_ADDR + 0x0C)
DMACC7Config                    EQU     (DMA_CHAN7_BASE_ADDR + 0x10)

SDRAM_CLK_BASE_ADDR	        	EQU     0x40004000
SDRAMCLK_CTRL	                EQU     (SDRAM_CLK_BASE_ADDR + 0x68)

SDRAM_BASE_ADDR		        	EQU     0x31080000
EMCControl	                	EQU     (SDRAM_BASE_ADDR + 0x00)
EMCStatus	                	EQU     (SDRAM_BASE_ADDR + 0x04)
EMCConfig	                	EQU     (SDRAM_BASE_ADDR + 0x08)
EMCDynamicControl	        	EQU     (SDRAM_BASE_ADDR + 0x20)
EMCDynamicRefresh	        	EQU     (SDRAM_BASE_ADDR + 0x24)
EMCDynamicReadConfig	        EQU     (SDRAM_BASE_ADDR + 0x28)
EMCDynamicRP	                EQU     (SDRAM_BASE_ADDR + 0x30)
EMCDynamicRAS	                EQU     (SDRAM_BASE_ADDR + 0x34)
EMCDynamicSREX	                EQU     (SDRAM_BASE_ADDR + 0x38)
EMCDynamicWR	                EQU     (SDRAM_BASE_ADDR + 0x44)
EMCDynamicRC	                EQU     (SDRAM_BASE_ADDR + 0x48)
EMCDynamicRFC	                EQU     (SDRAM_BASE_ADDR + 0x4C)
EMCDynamicXSR	                EQU     (SDRAM_BASE_ADDR + 0x50)
EMCDynamicRRD	                EQU     (SDRAM_BASE_ADDR + 0x54)
EMCDynamicMRD	                EQU     (SDRAM_BASE_ADDR + 0x58)
EMCDynamicCDLR	                EQU     (SDRAM_BASE_ADDR + 0x5C)
EMCStaticExtendedWait           EQU     (SDRAM_BASE_ADDR + 0x80)

EMCDynamicConfig0	        	EQU     (SDRAM_BASE_ADDR + 0x100)
EMCDynamicRasCas0	        	EQU     (SDRAM_BASE_ADDR + 0x104)

EMCDynamicConfig1	        	EQU     (SDRAM_BASE_ADDR + 0x120)
EMCDynamicRasCas1	        	EQU     (SDRAM_BASE_ADDR + 0x124)

EMCAHBControl0	                EQU     (SDRAM_BASE_ADDR + 0x400)
EMCAHBStatus0	                EQU     (SDRAM_BASE_ADDR + 0x404)
EMCAHBTimeout0	                EQU     (SDRAM_BASE_ADDR + 0x408)

EMCAHBControl2	                EQU     (SDRAM_BASE_ADDR + 0x440)
EMCAHBStatus2	                EQU     (SDRAM_BASE_ADDR + 0x444)
EMCAHBTimeout2	                EQU     (SDRAM_BASE_ADDR + 0x448)

EMCAHBControl3	                EQU     (SDRAM_BASE_ADDR + 0x460)
EMCAHBStatus3	                EQU     (SDRAM_BASE_ADDR + 0x464)
EMCAHBTimeout3	                EQU     (SDRAM_BASE_ADDR + 0x468)

EMCAHBControl4	                EQU     (SDRAM_BASE_ADDR + 0x480)
EMCAHBStatus4	                EQU     (SDRAM_BASE_ADDR + 0x484)
EMCAHBTimeout4	                EQU     (SDRAM_BASE_ADDR + 0x488)

DDR_BASE_ADDR	                EQU     0x40004000
DDR_LAP_NOM	                	EQU     (DDR_BASE_ADDR + 0x6C)
DDR_LAP_COUNT	                EQU     (DDR_BASE_ADDR + 0x70)
DDR_CAL_DELAY	                EQU     (DDR_BASE_ADDR + 0x74)

EMC_STATIC_BASE_ADDR            EQU     0x31080200
EMCStaticConfig0                EQU     (EMC_STATIC_BASE_ADDR + 0x00)
EMCStaticWaitWen0               EQU     (EMC_STATIC_BASE_ADDR + 0x04)
EMCStaticWaitOen0               EQU     (EMC_STATIC_BASE_ADDR + 0x08)
EMCStaticWaitRd0                EQU     (EMC_STATIC_BASE_ADDR + 0x0C)
EMCStaticwaitPage0              EQU     (EMC_STATIC_BASE_ADDR + 0x10)
EMCStaticWaitWr0                EQU     (EMC_STATIC_BASE_ADDR + 0x14)
EMCStaticWaitTurn0              EQU     (EMC_STATIC_BASE_ADDR + 0x18)

EMCStaticConfig1                EQU     (EMC_STATIC_BASE_ADDR + 0x20)
EMCStaticWaitWen1               EQU     (EMC_STATIC_BASE_ADDR + 0x24)
EMCStaticWaitOen1               EQU     (EMC_STATIC_BASE_ADDR + 0x28)
EMCStaticWaitRd1                EQU     (EMC_STATIC_BASE_ADDR + 0x2C)
EMCStaticwaitPage1              EQU     (EMC_STATIC_BASE_ADDR + 0x30)
EMCStaticWaitWr1                EQU     (EMC_STATIC_BASE_ADDR + 0x34)
EMCStaticWaitTurn1              EQU     (EMC_STATIC_BASE_ADDR + 0x38)

EMCStaticConfig2                EQU     (EMC_STATIC_BASE_ADDR + 0x40)
EMCStaticWaitWen2               EQU     (EMC_STATIC_BASE_ADDR + 0x44)
EMCStaticWaitOen2               EQU     (EMC_STATIC_BASE_ADDR + 0x48)
EMCStaticWaitRd2                EQU     (EMC_STATIC_BASE_ADDR + 0x4C)
EMCStaticwaitPage2              EQU     (EMC_STATIC_BASE_ADDR + 0x50)
EMCStaticWaitWr2                EQU     (EMC_STATIC_BASE_ADDR + 0x54)
EMCStaticWaitTurn2              EQU     (EMC_STATIC_BASE_ADDR + 0x58)

EMCStaticConfig3                EQU     (EMC_STATIC_BASE_ADDR + 0x60)
EMCStaticWaitWen3               EQU     (EMC_STATIC_BASE_ADDR + 0x64)
EMCStaticWaitOen3               EQU     (EMC_STATIC_BASE_ADDR + 0x68)
EMCStaticWaitRd3                EQU     (EMC_STATIC_BASE_ADDR + 0x6C)
EMCStaticwaitPage3              EQU     (EMC_STATIC_BASE_ADDR + 0x70)
EMCStaticWaitWr3                EQU     (EMC_STATIC_BASE_ADDR + 0x74)
EMCStaticWaitTurn3              EQU     (EMC_STATIC_BASE_ADDR + 0x78)

MLC_BASE_ADDR	                EQU     0x200B8000
MLC_CMD		                	EQU     (MLC_BASE_ADDR + 0x00)
MLC_ADDR	                	EQU     (MLC_BASE_ADDR + 0x04)
MLC_ECC_ENC_REG	                EQU     (MLC_BASE_ADDR + 0x08)
MLC_ECC_DEC_REG	                EQU     (MLC_BASE_ADDR + 0x0C)
MLC_ECC_AUTO_ENC_REG	        EQU     (MLC_BASE_ADDR + 0x10)
MLC_ECC_AUTO_DEC_REG	        EQU     (MLC_BASE_ADDR + 0x14)
MLC_RPR		                	EQU     (MLC_BASE_ADDR + 0x18)
MLC_WPR		                	EQU     (MLC_BASE_ADDR + 0x1C)
MLC_RUBP	                	EQU     (MLC_BASE_ADDR + 0x20)
MLC_ROBP	                	EQU     (MLC_BASE_ADDR + 0x24)
MLC_SW_WP_ADD_LOW	        	EQU     (MLC_BASE_ADDR + 0x28)
MLC_SW_WP_ADD_HIG	        	EQU     (MLC_BASE_ADDR + 0x2C)
MLC_ICR		                	EQU     (MLC_BASE_ADDR + 0x30)
MLC_TIME_REG	                EQU     (MLC_BASE_ADDR + 0x34)
MLC_IRQ_MR	                	EQU     (MLC_BASE_ADDR + 0x38)
MLC_IRQ_SR	                	EQU     (MLC_BASE_ADDR + 0x3C)
MLC_LOCK_PR	                	EQU     (MLC_BASE_ADDR + 0x44)
MLC_ISR		                	EQU     (MLC_BASE_ADDR + 0x48)
MLC_CEH		                	EQU     (MLC_BASE_ADDR + 0x4C)

MLC_BUFF                        EQU     (0x200A8000)
MLC_DATA                        EQU     (0x200B0000)

SLC_BASE_ADDR	                EQU     0x20020000
SLC_DATA	                	EQU     (SLC_BASE_ADDR + 0x00)
SLC_ADDR	                	EQU     (SLC_BASE_ADDR + 0x04)
SLC_CMD		                	EQU     (SLC_BASE_ADDR + 0x08)
SLC_STOP	                	EQU     (SLC_BASE_ADDR + 0x0C)
SLC_CTRL	                	EQU     (SLC_BASE_ADDR + 0x10)
SLC_CFG		                	EQU     (SLC_BASE_ADDR + 0x14)
SLC_STAT	                	EQU     (SLC_BASE_ADDR + 0x18)
SLC_INT_STAT	                EQU     (SLC_BASE_ADDR + 0x1C)
SLC_IEN		                	EQU     (SLC_BASE_ADDR + 0x20)
SLC_ISR		                	EQU     (SLC_BASE_ADDR + 0x24)
SLC_ICR		                	EQU     (SLC_BASE_ADDR + 0x28)
SLC_TAC		                	EQU     (SLC_BASE_ADDR + 0x2C)
SLC_TC                      	EQU     (SLC_BASE_ADDR + 0x30)
SLC_ECC	                        EQU     (SLC_BASE_ADDR + 0x34)
SLC_DMA_DATA                    EQU     (SLC_BASE_ADDR + 0x38)

LCD_BASE_ADDR                   EQU     0x31040000
LCD_TIMH                        EQU     (LCD_BASE_ADDR + 0x000)
LCD_TIMV                        EQU     (LCD_BASE_ADDR + 0x004)
LCD_POL                         EQU     (LCD_BASE_ADDR + 0x008)
LCD_LE                          EQU     (LCD_BASE_ADDR + 0x00C)
LCD_UPBASE                      EQU     (LCD_BASE_ADDR + 0x010)
LCD_LPBASE                      EQU     (LCD_BASE_ADDR + 0x014)
LCD_CTRL                        EQU     (LCD_BASE_ADDR + 0x018)
LCD_INTMSK                      EQU     (LCD_BASE_ADDR + 0x01C)
LCD_INTRAW                      EQU     (LCD_BASE_ADDR + 0x020)
LCD_INTSTAT                     EQU     (LCD_BASE_ADDR + 0x024)
LCD_INTCLR                      EQU     (LCD_BASE_ADDR + 0x028)
LCD_UPCURR                      EQU     (LCD_BASE_ADDR + 0x02C)
LCD_LPCURR                      EQU     (LCD_BASE_ADDR + 0x030)

LCD_PAL                         EQU     (LCD_BASE_ADDR + 0x200)

CRSR_IMG                        EQU     (LCD_BASE_ADDR + 0x800)
CRSR_CTRL                       EQU     (LCD_BASE_ADDR + 0xC00) 
CRSR_CFG                        EQU     (LCD_BASE_ADDR + 0xC04)
CRSR_PAL0                       EQU     (LCD_BASE_ADDR + 0xC08)
CRSR_PAL1                       EQU     (LCD_BASE_ADDR + 0xC0C)
CRSR_XY                         EQU     (LCD_BASE_ADDR + 0xC10)
CRSR_CLIP                       EQU     (LCD_BASE_ADDR + 0xC14)
CRSR_INTMSK                     EQU     (LCD_BASE_ADDR + 0xC20)
CRSR_INTCLR                     EQU     (LCD_BASE_ADDR + 0xC24)
CRSR_INTRAW                     EQU     (LCD_BASE_ADDR + 0xC28)
CRSR_INTSTAT                    EQU     (LCD_BASE_ADDR + 0xC2C)

TOUCH_BASE_ADDR                 EQU     0x40048000
ADC_STAT                        EQU     (TOUCH_BASE_ADDR + 0x00) 
ADC_SELECT                      EQU     (TOUCH_BASE_ADDR + 0x04) 
ADC_CTRL                        EQU     (TOUCH_BASE_ADDR + 0x08) 
TSC_SAMPLE_FIFO                 EQU     (TOUCH_BASE_ADDR + 0x0c)         
TSC_DTR                         EQU     (TOUCH_BASE_ADDR + 0x10) 
TSC_RTR                         EQU     (TOUCH_BASE_ADDR + 0x14) 
TSC_UTR                         EQU     (TOUCH_BASE_ADDR + 0x18) 
TSC_TTR                         EQU     (TOUCH_BASE_ADDR + 0x1C) 
TSC_DXP                         EQU     (TOUCH_BASE_ADDR + 0x20) 
TSC_MIN_X                       EQU     (TOUCH_BASE_ADDR + 0x24)   
TSC_MAX_X                       EQU     (TOUCH_BASE_ADDR + 0x28)  
TSC_MIN_Y                       EQU     (TOUCH_BASE_ADDR + 0x2C)  
TSC_MAX_Y                       EQU     (TOUCH_BASE_ADDR + 0x30)   
TSC_AUX_UTR                     EQU     (TOUCH_BASE_ADDR + 0x34)    
TSC_AUX_MIN                     EQU     (TOUCH_BASE_ADDR + 0x38)    
TSC_AUX_MAX                     EQU     (TOUCH_BASE_ADDR + 0x3C)     
TSC_AUX_VALUE                   EQU     (TOUCH_BASE_ADDR + 0x44)      
ADC_VALUE                       EQU     (TOUCH_BASE_ADDR + 0x48)   

KS_BASE_ADDR		        	EQU     0x40050000
KS_DEB		                	EQU     (KS_BASE_ADDR + 0x00)
KS_STATE_COND	                EQU     (KS_BASE_ADDR + 0x04)
KS_IRQ		                	EQU     (KS_BASE_ADDR + 0x08)
KS_SCAN_CTL	                	EQU     (KS_BASE_ADDR + 0x0C)
KS_FAST_TST	                	EQU     (KS_BASE_ADDR + 0x10)
KS_MATRIX_DIM        	        EQU     (KS_BASE_ADDR + 0x14)
KS_DATA0	                	EQU     (KS_BASE_ADDR + 0x40)
KS_DATA1	                	EQU     (KS_BASE_ADDR + 0x44)
KS_DATA2	                	EQU     (KS_BASE_ADDR + 0x48)
KS_DATA3	                	EQU     (KS_BASE_ADDR + 0x4C)
KS_DATA4            	        EQU     (KS_BASE_ADDR + 0x50)
KS_DATA5	                	EQU     (KS_BASE_ADDR + 0x54)
KS_DATA6	                	EQU     (KS_BASE_ADDR + 0x58)
KS_DATA7             	        EQU     (KS_BASE_ADDR + 0x5C)

MAC_BASE_ADDR		        	EQU     0x31060000 
MAC_MAC1                        EQU     (MAC_BASE_ADDR + 0x000) 
MAC_MAC2                        EQU     (MAC_BASE_ADDR + 0x004) 
MAC_IPGT                        EQU     (MAC_BASE_ADDR + 0x008) 
MAC_IPGR                        EQU     (MAC_BASE_ADDR + 0x00C)
MAC_CLRT                        EQU     (MAC_BASE_ADDR + 0x010)
MAC_MAXF                        EQU     (MAC_BASE_ADDR + 0x014)
MAC_SUPP                        EQU     (MAC_BASE_ADDR + 0x018)
MAC_TEST                        EQU     (MAC_BASE_ADDR + 0x01C)
MAC_MCFG                        EQU     (MAC_BASE_ADDR + 0x020)
MAC_MCMD                        EQU     (MAC_BASE_ADDR + 0x024)
MAC_MADR                        EQU     (MAC_BASE_ADDR + 0x028)
MAC_MWTD                        EQU     (MAC_BASE_ADDR + 0x02C)
MAC_MRDD                        EQU     (MAC_BASE_ADDR + 0x030)
MAC_MIND                        EQU     (MAC_BASE_ADDR + 0x034)

MAC_SA0                         EQU     (MAC_BASE_ADDR + 0x040) 
MAC_SA1                         EQU     (MAC_BASE_ADDR + 0x044) 
MAC_SA2                         EQU     (MAC_BASE_ADDR + 0x048) 

MAC_COMMAND                     EQU     (MAC_BASE_ADDR + 0x100) 
MAC_STATUS                      EQU     (MAC_BASE_ADDR + 0x104) 
MAC_RXDESCRIPTOR                EQU     (MAC_BASE_ADDR + 0x108) 
MAC_RXSTATUS                    EQU     (MAC_BASE_ADDR + 0x10C) 
MAC_RXDESCRIPTORNUM             EQU     (MAC_BASE_ADDR + 0x110) 
MAC_RXPRODUCEINDEX              EQU     (MAC_BASE_ADDR + 0x114) 
MAC_RXCONSUMEINDEX              EQU     (MAC_BASE_ADDR + 0x118) 
MAC_TXDESCRIPTOR                EQU     (MAC_BASE_ADDR + 0x11C) 
MAC_TXSTATUS                    EQU     (MAC_BASE_ADDR + 0x120) 
MAC_TXDESCRIPTORNUM             EQU     (MAC_BASE_ADDR + 0x124) 
MAC_TXPRODUCEINDEX              EQU     (MAC_BASE_ADDR + 0x128) 
MAC_TXCONSUMEINDEX              EQU     (MAC_BASE_ADDR + 0x12C) 

MAC_TSV0                        EQU     (MAC_BASE_ADDR + 0x158) 
MAC_TSV1                        EQU     (MAC_BASE_ADDR + 0x15C) 
MAC_RSV                         EQU     (MAC_BASE_ADDR + 0x160) 

MAC_FLOWCONTROLCNT              EQU     (MAC_BASE_ADDR + 0x170) 
MAC_FLOWCONTROLSTS              EQU     (MAC_BASE_ADDR + 0x174) 

MAC_RXFILTERCTRL                EQU     (MAC_BASE_ADDR + 0x200)
MAC_RXFILTERWOLSTS              EQU     (MAC_BASE_ADDR + 0x204)
MAC_RXFILTERWOLCLR              EQU     (MAC_BASE_ADDR + 0x208)

MAC_HASHFILTERL                 EQU     (MAC_BASE_ADDR + 0x210)
MAC_HASHFILTERH                 EQU     (MAC_BASE_ADDR + 0x214)

MAC_INTSTATUS                   EQU     (MAC_BASE_ADDR + 0xFE0)
MAC_INTENABLE                   EQU     (MAC_BASE_ADDR + 0xFE4)
MAC_INTCLEAR                    EQU     (MAC_BASE_ADDR + 0xFE8)
MAC_INTSET                      EQU     (MAC_BASE_ADDR + 0xFEC)

MAC_POWERDOWN                   EQU     (MAC_BASE_ADDR + 0xFF4)
MAC_MODULEID                    EQU     (MAC_BASE_ADDR + 0xFFC)

USB_BASE_ADDR		        	EQU     0x31020200

DEV_INT_STAT                    EQU     (USB_BASE_ADDR + 0x00)
DEV_INT_EN                      EQU     (USB_BASE_ADDR + 0x04)
DEV_INT_CLR                     EQU     (USB_BASE_ADDR + 0x08)
DEV_INT_SET                     EQU     (USB_BASE_ADDR + 0x0C)
DEV_INT_PRIO                    EQU     (USB_BASE_ADDR + 0x2C)
EP_INT_STAT                     EQU     (USB_BASE_ADDR + 0x30)
EP_INT_EN                       EQU     (USB_BASE_ADDR + 0x34)
EP_INT_CLR                      EQU     (USB_BASE_ADDR + 0x38)
EP_INT_SET                      EQU     (USB_BASE_ADDR + 0x3C)
EP_INT_PRIO                     EQU     (USB_BASE_ADDR + 0x40)
REALIZE_EP                      EQU     (USB_BASE_ADDR + 0x44)
EP_INDEX                        EQU     (USB_BASE_ADDR + 0x48)
MAXPACKET_SIZE                  EQU     (USB_BASE_ADDR + 0x4C)
CMD_CODE                        EQU     (USB_BASE_ADDR + 0x10)
CMD_DATA                        EQU     (USB_BASE_ADDR + 0x14)
RX_DATA                         EQU     (USB_BASE_ADDR + 0x18)
TX_DATA                         EQU     (USB_BASE_ADDR + 0x1C)
RX_PLENGTH                      EQU     (USB_BASE_ADDR + 0x20)
TX_PLENGTH                      EQU     (USB_BASE_ADDR + 0x24)
USB_CTRL                        EQU     (USB_BASE_ADDR + 0x28)
DC_REVISION                     EQU     (USB_BASE_ADDR + 0x7C)

DMA_REQ_STAT                    EQU     (USB_BASE_ADDR + 0x50)
DMA_REQ_CLR                     EQU     (USB_BASE_ADDR + 0x54)
DMA_REQ_SET                     EQU     (USB_BASE_ADDR + 0x58)
UDCA_HEAD                       EQU     (USB_BASE_ADDR + 0x80)
EP_DMA_STAT                     EQU     (USB_BASE_ADDR + 0x84)
EP_DMA_EN                       EQU     (USB_BASE_ADDR + 0x88)
EP_DMA_DIS                      EQU     (USB_BASE_ADDR + 0x8C)
DMA_INT_STAT                    EQU     (USB_BASE_ADDR + 0x90)
DMA_INT_EN                      EQU     (USB_BASE_ADDR + 0x94)
EOT_INT_STAT                    EQU     (USB_BASE_ADDR + 0xA0)
EOT_INT_CLR                     EQU     (USB_BASE_ADDR + 0xA4)
EOT_INT_SET                     EQU     (USB_BASE_ADDR + 0xA8)
NDD_REQ_INT_STAT                EQU     (USB_BASE_ADDR + 0xAC)
NDD_REQ_INT_CLR                 EQU     (USB_BASE_ADDR + 0xB0)
NDD_REQ_INT_SET                 EQU     (USB_BASE_ADDR + 0xB4)
SYS_ERR_INT_STAT                EQU     (USB_BASE_ADDR + 0xB8)
SYS_ERR_INT_CLR                 EQU     (USB_BASE_ADDR + 0xBC)
SYS_ERR_INT_SET                 EQU     (USB_BASE_ADDR + 0xC0)
MODULE_ID                       EQU     (USB_BASE_ADDR + 0xFC)

USBH_BASE_ADDR	                EQU     0x31020000
HcRevision	                	EQU     (USBH_BASE_ADDR + 0x00)
HcControl	                	EQU     (USBH_BASE_ADDR + 0x04)
HcCommandStatus	                EQU     (USBH_BASE_ADDR + 0x08)
HcInterruptStatus	        	EQU     (USBH_BASE_ADDR + 0x0C)
HcInterruptEnable	        	EQU     (USBH_BASE_ADDR + 0x10)
HcInterruptDisable	        	EQU     (USBH_BASE_ADDR + 0x14)
HcHCCA		                	EQU     (USBH_BASE_ADDR + 0x18)
HcPeriodCurrentED	        	EQU     (USBH_BASE_ADDR + 0x1C)
HcControlHeadED	                EQU     (USBH_BASE_ADDR + 0x20)
HcControlCurrentED	        	EQU     (USBH_BASE_ADDR + 0x24)
HcBulkHeadED	                EQU     (USBH_BASE_ADDR + 0x28)
HcBulkCurrentED                 EQU     (USBH_BASE_ADDR + 0x2C)
HcDoneHead	                	EQU     (USBH_BASE_ADDR + 0x30)
HcFmInterval	                EQU     (USBH_BASE_ADDR + 0x34)
HcFmRemaining	                EQU     (USBH_BASE_ADDR + 0x38)
HcFmNumber	                	EQU     (USBH_BASE_ADDR + 0x3C)
HcPeriodicStart	                EQU     (USBH_BASE_ADDR + 0x40)
HcLSThreshold	                EQU     (USBH_BASE_ADDR + 0x44)
HcRhDescriptorA	                EQU     (USBH_BASE_ADDR + 0x48)
HcRhDescriptorB	                EQU     (USBH_BASE_ADDR + 0x4C)
HcRhStatus	                	EQU     (USBH_BASE_ADDR + 0x50)
HcRhPortStatus1	                EQU     (USBH_BASE_ADDR + 0x54)
HcRhPortStatus2	                EQU     (USBH_BASE_ADDR + 0x58)
HcModuleID	                	EQU     (USBH_BASE_ADDR + 0x5C)

USB_OTG_BASE_ADDR	        	EQU     0x31020000
OTG_INT_STATUS	                EQU     (USB_OTG_BASE_ADDR + 0x100)
OTG_INT_ENABLE	                EQU     (USB_OTG_BASE_ADDR + 0x104)
OTG_INT_SET                     EQU     (USB_OTG_BASE_ADDR + 0x108)
OTG_INT_CLEAR	                EQU     (USB_OTG_BASE_ADDR + 0x10C)
OTG_STATUS   	                EQU     (USB_OTG_BASE_ADDR + 0x110)
OTG_TIMER	                	EQU     (USB_OTG_BASE_ADDR + 0x114)
OTG_CLOCK_CONTROL	        	EQU     (USB_OTG_BASE_ADDR + 0xFF4)
OTG_CLOCK_STATUS	        	EQU     (USB_OTG_BASE_ADDR + 0xFF8)

OTG_I2C_BASE_ADDR	        	EQU     0x31020300
I2C_RX		                	EQU     (OTG_I2C_BASE_ADDR + 0x00)
I2C_TX		                	EQU     (OTG_I2C_BASE_ADDR + 0x00)
I2C_STS		                	EQU     (OTG_I2C_BASE_ADDR + 0x04)
I2C_CTL  	                	EQU     (OTG_I2C_BASE_ADDR + 0x08)
I2C_CLKHI	                	EQU     (OTG_I2C_BASE_ADDR + 0x0C)
I2C_CLKLO	                	EQU     (OTG_I2C_BASE_ADDR + 0x10)

SD_BASE_ADDR	                EQU     0x20098000
SD_POWER	                	EQU     (SD_BASE_ADDR + 0x00)
SD_CLOCK	                	EQU     (SD_BASE_ADDR + 0x04)
SD_ARGUMENT	               	 	EQU     (SD_BASE_ADDR + 0x08)
SD_COMMAND	                	EQU     (SD_BASE_ADDR + 0x0C)
SD_RESPCMD	                	EQU     (SD_BASE_ADDR + 0x10)
SD_RESPONSE0	                EQU     (SD_BASE_ADDR + 0x14)
SD_RESPONSE1	                EQU     (SD_BASE_ADDR + 0x18)
SD_RESPONSE2        	        EQU     (SD_BASE_ADDR + 0x1C)
SD_RESPONSE3	                EQU     (SD_BASE_ADDR + 0x20)
SD_DATATIMER	                EQU     (SD_BASE_ADDR + 0x24)
SD_DATALENGTH	                EQU     (SD_BASE_ADDR + 0x28)
SD_DATACTRL	                	EQU     (SD_BASE_ADDR + 0x2C)
SD_DATACNT	                	EQU     (SD_BASE_ADDR + 0x30)
SD_STATUS                       EQU     (SD_BASE_ADDR + 0x34)
SD_CLEAR                        EQU     (SD_BASE_ADDR + 0x38)
SD_MASK0	                	EQU     (SD_BASE_ADDR + 0x3C)
SD_MASK1                        EQU     (SD_BASE_ADDR + 0x40)
SD_FIFOCNT	                	EQU     (SD_BASE_ADDR + 0x48)

SD_FIFO		                	EQU     (SD_BASE_ADDR + 0x80)

UART3_BASE_ADDR		        	EQU     0x40080000
U3RBR                           EQU     (UART3_BASE_ADDR + 0x00)
U3THR                           EQU     (UART3_BASE_ADDR + 0x00)
U3DLL                           EQU     (UART3_BASE_ADDR + 0x00)
U3DLM                           EQU     (UART3_BASE_ADDR + 0x04)
U3IER                           EQU     (UART3_BASE_ADDR + 0x04)
U3IIR                           EQU     (UART3_BASE_ADDR + 0x08)
U3FCR                           EQU     (UART3_BASE_ADDR + 0x08)
U3LCR                           EQU     (UART3_BASE_ADDR + 0x0C)
U3LSR                           EQU     (UART3_BASE_ADDR + 0x14)
U3RXLEV                         EQU     (UART3_BASE_ADDR + 0x1C)

UART4_BASE_ADDR		        	EQU     0x40088000
U4RBR                           EQU     (UART4_BASE_ADDR + 0x00)
U4THR                           EQU     (UART4_BASE_ADDR + 0x00)
U4DLL                           EQU     (UART4_BASE_ADDR + 0x00)
U4DLM                           EQU     (UART4_BASE_ADDR + 0x04)
U4IER                           EQU     (UART4_BASE_ADDR + 0x04)
U4IIR                           EQU     (UART4_BASE_ADDR + 0x08)
U4FCR                           EQU     (UART4_BASE_ADDR + 0x08)
U4LCR                           EQU     (UART4_BASE_ADDR + 0x0C)
U4LSR                           EQU     (UART4_BASE_ADDR + 0x14)
U4RXLEV                         EQU     (UART4_BASE_ADDR + 0x1C)

UART5_BASE_ADDR		        	EQU     0x40090000
U5RBR                           EQU     (UART5_BASE_ADDR + 0x00)
U5THR                           EQU     (UART5_BASE_ADDR + 0x00)
U5DLL                           EQU     (UART5_BASE_ADDR + 0x00)
U5DLM                           EQU     (UART5_BASE_ADDR + 0x04)
U5IER                           EQU     (UART5_BASE_ADDR + 0x04)
U5IIR                           EQU     (UART5_BASE_ADDR + 0x08)
U5FCR                           EQU     (UART5_BASE_ADDR + 0x08)
U5LCR                           EQU     (UART5_BASE_ADDR + 0x0C)
U5LSR                           EQU     (UART5_BASE_ADDR + 0x14)
U5RXLEV                         EQU     (UART5_BASE_ADDR + 0x1C)

UART6_BASE_ADDR		        	EQU     0x40098000
U6RBR                           EQU     (UART6_BASE_ADDR + 0x00)
U6THR                           EQU     (UART6_BASE_ADDR + 0x00)
U6DLL                           EQU     (UART6_BASE_ADDR + 0x00)
U6DLM                           EQU     (UART6_BASE_ADDR + 0x04)
U6IER                           EQU     (UART6_BASE_ADDR + 0x04)
U6IIR                           EQU     (UART6_BASE_ADDR + 0x08)
U6FCR                           EQU     (UART6_BASE_ADDR + 0x08)
U6LCR                           EQU     (UART6_BASE_ADDR + 0x0C)
U6LSR                           EQU     (UART6_BASE_ADDR + 0x14)
U6RXLEV                         EQU     (UART6_BASE_ADDR + 0x1C)

UART_CLK_BASE_ADDR	        	EQU     0x40004000
U3CLK                           EQU     (UART_CLK_BASE_ADDR + 0xD0)
U4CLK                           EQU     (UART_CLK_BASE_ADDR + 0xD4)
U5CLK                           EQU     (UART_CLK_BASE_ADDR + 0xD8)
U6CLK                           EQU     (UART_CLK_BASE_ADDR + 0xDC)
IRDACLK                         EQU     (UART_CLK_BASE_ADDR + 0xE0)

UART_CTRL_BASE_ADDR	        	EQU     0x40054000
UART_CTRL                       EQU     (UART_CTRL_BASE_ADDR + 0x00)
UART_CLKMODE                    EQU     (UART_CTRL_BASE_ADDR + 0x04)
UART_LOOP                       EQU     (UART_CTRL_BASE_ADDR + 0x08)

HS_UART1_BASE_ADDR				EQU     0x40014000
HSU1_RX                         EQU     (HS_UART1_BASE_ADDR + 0x00)
HSU1_TX                         EQU     (HS_UART1_BASE_ADDR + 0x00)
HSU1_LEVEL                      EQU     (HS_UART1_BASE_ADDR + 0x04)
HSU1_IIR                        EQU     (HS_UART1_BASE_ADDR + 0x08)
HSU1_CTRL                       EQU     (HS_UART1_BASE_ADDR + 0x0C)
HSU1_RATE                       EQU     (HS_UART1_BASE_ADDR + 0x10)

HS_UART2_BASE_ADDR				EQU     0x40018000
HSU2_RX                         EQU     (HS_UART2_BASE_ADDR + 0x00)
HSU2_TX                         EQU     (HS_UART2_BASE_ADDR + 0x00)
HSU2_LEVEL                      EQU     (HS_UART2_BASE_ADDR + 0x04)
HSU2_IIR                        EQU     (HS_UART2_BASE_ADDR + 0x08)
HSU2_CTRL                       EQU     (HS_UART2_BASE_ADDR + 0x0C)
HSU2_RATE                       EQU     (HS_UART2_BASE_ADDR + 0x10)

HS_UART7_BASE_ADDR				EQU     0x4001C000
HSU7_RX                         EQU     (HS_UART7_BASE_ADDR + 0x00)
HSU7_TX                         EQU     (HS_UART7_BASE_ADDR + 0x00)
HSU7_LEVEL                      EQU     (HS_UART7_BASE_ADDR + 0x04)
HSU7_IIR                        EQU     (HS_UART7_BASE_ADDR + 0x08)
HSU7_CTRL                       EQU     (HS_UART7_BASE_ADDR + 0x0C)
HSU7_RATE                       EQU     (HS_UART7_BASE_ADDR + 0x10)

SPI1_BASE_ADDR	                EQU     0x20088000
SPI1_GLOBAL                     EQU     (SPI1_BASE_ADDR + 0x000)
SPI1_CON	                	EQU     (SPI1_BASE_ADDR + 0x004)
SPI1_FRM	                	EQU     (SPI1_BASE_ADDR + 0x008)
SPI1_IER	                	EQU     (SPI1_BASE_ADDR + 0x00C)
SPI1_STAT	               		EQU     (SPI1_BASE_ADDR + 0x010)
SPI1_DAT	                	EQU     (SPI1_BASE_ADDR + 0x014)
SPI1_TIM_CTRL	                EQU     (SPI1_BASE_ADDR + 0x400)
SPI1_TIM_COUNT	                EQU     (SPI1_BASE_ADDR + 0x404)
SPI1_TIM_STAT	                EQU     (SPI1_BASE_ADDR + 0x408)

SPI2_BASE_ADDR	                EQU     0x20090000
SPI2_GLOBAL	                	EQU     (SPI2_BASE_ADDR + 0x000)
SPI2_CON	                	EQU     (SPI2_BASE_ADDR + 0x004)
SPI2_FRM	                	EQU     (SPI2_BASE_ADDR + 0x008)
SPI2_IER	                	EQU     (SPI2_BASE_ADDR + 0x00C)
SPI2_STAT	                	EQU     (SPI2_BASE_ADDR + 0x010)
SPI2_DAT	                	EQU     (SPI2_BASE_ADDR + 0x014)
SPI2_TIM_CTRL	                EQU     (SPI2_BASE_ADDR + 0x400)
SPI2_TIM_COUNT	                EQU     (SPI2_BASE_ADDR + 0x404)
SPI2_TIM_STAT	                EQU     (SPI2_BASE_ADDR + 0x408)

SSP0_BASE_ADDR	                EQU     0x20084000
SSP0CR0                         EQU     (SSP0_BASE_ADDR + 0x000)
SSP0CR1  	               	 	EQU     (SSP0_BASE_ADDR + 0x004)
SSP0DR   	                	EQU     (SSP0_BASE_ADDR + 0x008)
SSP0SR  	                	EQU     (SSP0_BASE_ADDR + 0x00C)
SSP0CPSR	                	EQU     (SSP0_BASE_ADDR + 0x010)
SSP0IMSC	                	EQU     (SSP0_BASE_ADDR + 0x014)
SSP0RIS             	        EQU     (SSP0_BASE_ADDR + 0x018)
SSP0MIS     	                EQU     (SSP0_BASE_ADDR + 0x01C)
SSP0ICR             	        EQU     (SSP0_BASE_ADDR + 0x020)
SSP0DMACR     	                EQU     (SSP0_BASE_ADDR + 0x024)

SSP1_BASE_ADDR	                EQU     0x2008C000
SSP1CR0                         EQU     (SSP1_BASE_ADDR + 0x000)
SSP1CR1  	                	EQU     (SSP1_BASE_ADDR + 0x004)
SSP1DR   	                	EQU     (SSP1_BASE_ADDR + 0x008)
SSP1SR  	                	EQU     (SSP1_BASE_ADDR + 0x00C)
SSP1CPSR	                	EQU     (SSP1_BASE_ADDR + 0x010)
SSP1IMSC                        EQU     (SSP1_BASE_ADDR + 0x014)
SSP1RIS              	        EQU     (SSP1_BASE_ADDR + 0x018)
SSP1MIS     	                EQU     (SSP1_BASE_ADDR + 0x01C)
SSP1ICR                         EQU     (SSP1_BASE_ADDR + 0x020)
SSP1DMACR     	                EQU     (SSP1_BASE_ADDR + 0x024)

I2C1_BASE_ADDR		        	EQU     0x400A0000
I2C1_RX		                	EQU     (I2C1_BASE_ADDR + 0x00)
I2C1_TX		                	EQU     (I2C1_BASE_ADDR + 0x00)
I2C1_STAT	                	EQU     (I2C1_BASE_ADDR + 0x04)
I2C1_CTRL	                	EQU     (I2C1_BASE_ADDR + 0x08)
I2C1_CLKHI	                	EQU     (I2C1_BASE_ADDR + 0x0C)
I2C1_CLKLO	                	EQU     (I2C1_BASE_ADDR + 0x10)
I2C1_ADR	                	EQU     (I2C1_BASE_ADDR + 0x14)
I2C1_RXFL	                	EQU     (I2C1_BASE_ADDR + 0x18)
I2C1_TXFL	                	EQU     (I2C1_BASE_ADDR + 0x1C)
I2C1_RXB	                	EQU     (I2C1_BASE_ADDR + 0x20)
I2C1_TXB	                	EQU     (I2C1_BASE_ADDR + 0x24)
I2C1_S_TX	                	EQU     (I2C1_BASE_ADDR + 0x28)
I2C1_S_TXFL	                	EQU     (I2C1_BASE_ADDR + 0x2C)

I2C2_BASE_ADDR		        	EQU     0x400A8000
I2C2_RX		                	EQU     (I2C2_BASE_ADDR + 0x00)
I2C2_TX		                	EQU     (I2C2_BASE_ADDR + 0x00)
I2C2_STAT	                	EQU     (I2C2_BASE_ADDR + 0x04)
I2C2_CTRL	                	EQU     (I2C2_BASE_ADDR + 0x08)
I2C2_CLKHI	                	EQU     (I2C2_BASE_ADDR + 0x0C)
I2C2_CLKLO	                	EQU     (I2C2_BASE_ADDR + 0x10)
I2C2_ADR	                	EQU     (I2C2_BASE_ADDR + 0x14)
I2C2_RXFL	                	EQU     (I2C2_BASE_ADDR + 0x18)
I2C2_TXFL	                	EQU     (I2C2_BASE_ADDR + 0x1C)
I2C2_RXB	                	EQU     (I2C2_BASE_ADDR + 0x20)
I2C2_TXB	                	EQU     (I2C2_BASE_ADDR + 0x24)
I2C2_S_TX	                	EQU     (I2C2_BASE_ADDR + 0x28)
I2C2_S_TXFL	               		EQU     (I2C2_BASE_ADDR + 0x2C) 

I2S0_BASE_ADDR                  EQU     0x20094000
I2S0DAO                         EQU     (I2S1_BASE_ADDR + 0x00)
I2S0DAI                         EQU     (I2S1_BASE_ADDR + 0x04)
I2S0TXFIFO                      EQU     (I2S1_BASE_ADDR + 0x08)
I2S0RXFIFO                      EQU     (I2S1_BASE_ADDR + 0x0C)
I2S0STATE                       EQU     (I2S1_BASE_ADDR + 0x10)
I2S0DMA0                        EQU     (I2S1_BASE_ADDR + 0x14)
I2S0DMA1                        EQU     (I2S1_BASE_ADDR + 0x18)
I2S0IRQ                         EQU     (I2S1_BASE_ADDR + 0x1C)
I2S0TXRATE                      EQU     (I2S1_BASE_ADDR + 0x20)
I2S0RXRATE                      EQU     (I2S1_BASE_ADDR + 0x24)

I2S1_BASE_ADDR                  EQU     0x2009C000
I2S1DAO                         EQU     (I2S1_BASE_ADDR + 0x00)
I2S1DAI                         EQU     (I2S1_BASE_ADDR + 0x04)
I2S1TXFIFO                      EQU     (I2S1_BASE_ADDR + 0x08)
I2S1RXFIFO                      EQU     (I2S1_BASE_ADDR + 0x0C)
I2S1STATE                       EQU     (I2S1_BASE_ADDR + 0x10)
I2S1DMA0                        EQU     (I2S1_BASE_ADDR + 0x14)
I2S1DMA1                        EQU     (I2S1_BASE_ADDR + 0x18)
I2S1IRQ                         EQU     (I2S1_BASE_ADDR + 0x1C)
I2S1TXRATE                      EQU     (I2S1_BASE_ADDR + 0x20)
I2S1RXRATE                      EQU     (I2S1_BASE_ADDR + 0x24) 

TIMER0_BASE_ADDR                EQU     0x40044000
T0IR                            EQU     (TIMER0_BASE_ADDR + 0x000)
T0TCR                           EQU     (TIMER0_BASE_ADDR + 0x004)
T0TC                            EQU     (TIMER0_BASE_ADDR + 0x008)
T0PR                            EQU     (TIMER0_BASE_ADDR + 0x00C)
T0PC                            EQU     (TIMER0_BASE_ADDR + 0x010)
T0MCR                           EQU     (TIMER0_BASE_ADDR + 0x014)
T0MR0                           EQU     (TIMER0_BASE_ADDR + 0x018)
T0MR1                           EQU     (TIMER0_BASE_ADDR + 0x01C)
T0MR2                           EQU     (TIMER0_BASE_ADDR + 0x020)
T0MR3                           EQU     (TIMER0_BASE_ADDR + 0x024)
T0CCR                           EQU     (TIMER0_BASE_ADDR + 0x028)
T0CR0                           EQU     (TIMER0_BASE_ADDR + 0x02C)
T0CR1                           EQU     (TIMER0_BASE_ADDR + 0x030)
T0CR2                           EQU     (TIMER0_BASE_ADDR + 0x034)
T0CR3                           EQU     (TIMER0_BASE_ADDR + 0x038)
T0EMR                           EQU     (TIMER0_BASE_ADDR + 0x03C)
T0CTCR                          EQU     (TIMER0_BASE_ADDR + 0x070)


TIMER1_BASE_ADDR                EQU     0x4004C000
T1IR                            EQU     (TIMER1_BASE_ADDR + 0x000)
T1TCR                           EQU     (TIMER1_BASE_ADDR + 0x004)
T1TC                            EQU     (TIMER1_BASE_ADDR + 0x008)
T1PR                            EQU     (TIMER1_BASE_ADDR + 0x00C)
T1PC                            EQU     (TIMER1_BASE_ADDR + 0x010)
T1MCR                           EQU     (TIMER1_BASE_ADDR + 0x014)
T1MR0                           EQU     (TIMER1_BASE_ADDR + 0x018)
T1MR1                           EQU     (TIMER1_BASE_ADDR + 0x01C)
T1MR2                           EQU     (TIMER1_BASE_ADDR + 0x020)
T1MR3                           EQU     (TIMER1_BASE_ADDR + 0x024)
T1CCR                           EQU     (TIMER1_BASE_ADDR + 0x028)
T1CR0                           EQU     (TIMER1_BASE_ADDR + 0x02C)
T1CR1                           EQU     (TIMER1_BASE_ADDR + 0x030)
T1CR2                           EQU     (TIMER1_BASE_ADDR + 0x034)
T1CR3                           EQU     (TIMER1_BASE_ADDR + 0x038)
T1EMR                           EQU     (TIMER1_BASE_ADDR + 0x03C)
T1CTCR                          EQU     (TIMER1_BASE_ADDR + 0x070)

TIMER2_BASE_ADDR                EQU     0x40058000
T2IR                            EQU     (TIMER2_BASE_ADDR + 0x000)
T2TCR                           EQU     (TIMER2_BASE_ADDR + 0x004)
T2TC                            EQU     (TIMER2_BASE_ADDR + 0x008)
T2PR                            EQU     (TIMER2_BASE_ADDR + 0x00C)
T2PC                            EQU     (TIMER2_BASE_ADDR + 0x010)
T2MCR                           EQU     (TIMER2_BASE_ADDR + 0x014)
T2MR0                           EQU     (TIMER2_BASE_ADDR + 0x018)
T2MR1                           EQU     (TIMER2_BASE_ADDR + 0x01C)
T2MR2                           EQU     (TIMER2_BASE_ADDR + 0x020)
T2MR3                           EQU     (TIMER2_BASE_ADDR + 0x024)
T2CCR                           EQU     (TIMER2_BASE_ADDR + 0x028)
T2CR0                           EQU     (TIMER2_BASE_ADDR + 0x02C)
T2CR1                           EQU     (TIMER2_BASE_ADDR + 0x030)
T2CR2                           EQU     (TIMER2_BASE_ADDR + 0x034)
T2CR3                           EQU     (TIMER2_BASE_ADDR + 0x038)
T2EMR                           EQU     (TIMER2_BASE_ADDR + 0x03C)
T2CTCR                          EQU     (TIMER2_BASE_ADDR + 0x070)

TIMER3_BASE_ADDR                EQU     0x40060000
T3IR                            EQU     (TIMER3_BASE_ADDR + 0x000)
T3TCR                           EQU     (TIMER3_BASE_ADDR + 0x004)
T3TC                            EQU     (TIMER3_BASE_ADDR + 0x008)
T3PR                            EQU     (TIMER3_BASE_ADDR + 0x00C)
T3PC                            EQU     (TIMER3_BASE_ADDR + 0x010)
T3MCR                           EQU     (TIMER3_BASE_ADDR + 0x014)
T3MR0                           EQU     (TIMER3_BASE_ADDR + 0x018)
T3MR1                           EQU     (TIMER3_BASE_ADDR + 0x01C)
T3MR2                           EQU     (TIMER3_BASE_ADDR + 0x020)
T3MR3                           EQU     (TIMER3_BASE_ADDR + 0x024)
T3CCR                           EQU     (TIMER3_BASE_ADDR + 0x028)
T3CR0                           EQU     (TIMER3_BASE_ADDR + 0x02C)
T3CR1                           EQU     (TIMER3_BASE_ADDR + 0x030)
T3CR2                           EQU     (TIMER3_BASE_ADDR + 0x034)
T3CR3                           EQU     (TIMER3_BASE_ADDR + 0x038)
T3EMR                           EQU     (TIMER3_BASE_ADDR + 0x03C)
T3CTCR                          EQU     (TIMER3_BASE_ADDR + 0x070)

HSTIM_BASE_ADDR		        	EQU     0x40038000
HSTIM_INT	                	EQU     (HSTIM_BASE_ADDR + 0x00)
HSTIM_CTRL	                	EQU     (HSTIM_BASE_ADDR + 0x04)
HSTIM_COUNTER	                EQU     (HSTIM_BASE_ADDR + 0x08)
HSTIM_PMATCH	                EQU     (HSTIM_BASE_ADDR + 0x0C)
HSTIM_PCOUNT	                EQU     (HSTIM_BASE_ADDR + 0x10)
HSTIM_MCTRL	                	EQU     (HSTIM_BASE_ADDR + 0x14)
HSTIM_MATCH0	                EQU     (HSTIM_BASE_ADDR + 0x18)
HSTIM_MATCH1	                EQU     (HSTIM_BASE_ADDR + 0x1C)
HSTIM_MATCH2	                EQU     (HSTIM_BASE_ADDR + 0x20)
HSTIM_CCR	                	EQU     (HSTIM_BASE_ADDR + 0x28)
HSTIM_CR0	                	EQU     (HSTIM_BASE_ADDR + 0x2C)
HSTIM_CR1	                	EQU     (HSTIM_BASE_ADDR + 0x30)

MSTIM_BASE_ADDR		        	EQU     0x40034000
MSTIM_INT	                	EQU     (MSTIM_BASE_ADDR + 0x00)
MSTIM_CTRL	                	EQU     (MSTIM_BASE_ADDR + 0x04)
MSTIM_COUNTER	                EQU     (MSTIM_BASE_ADDR + 0x08)
MSTIM_MCTRL	                	EQU     (MSTIM_BASE_ADDR + 0x14)
MSTIM_MATCH0	                EQU     (MSTIM_BASE_ADDR + 0x18)
MSTIM_MATCH1	                EQU     (MSTIM_BASE_ADDR + 0x1C)

RTC_BASE_ADDR		        	EQU     0x40024000
RTC_UCOUNT	                	EQU     (RTC_BASE_ADDR + 0x00)
RTC_DCOUNT	               		EQU     (RTC_BASE_ADDR + 0x04)
RTC_MATCH0	                	EQU     (RTC_BASE_ADDR + 0x08)
RTC_MATCH1	                	EQU     (RTC_BASE_ADDR + 0x0C)
RTC_CTRL	                	EQU     (RTC_BASE_ADDR + 0x10)
RTC_INTSTAT                     EQU     (RTC_BASE_ADDR + 0x14)
RTC_KEY		                	EQU     (RTC_BASE_ADDR + 0x18)
RTC_SRAM	                	EQU     (RTC_BASE_ADDR + 0x80)

WDTIM_BASE_ADDR		        	EQU     0x4003C000
WDTIM_INT	                	EQU     (WDTIM_BASE_ADDR + 0x00)
WDTIM_CTRL	                	EQU     (WDTIM_BASE_ADDR + 0x04)
WDTIM_COUNTER                   EQU     (WDTIM_BASE_ADDR + 0x08)
WDTIM_MCTRL	                	EQU     (WDTIM_BASE_ADDR + 0x0C)
WDTIM_MATCH0	                EQU     (WDTIM_BASE_ADDR + 0x10)
WDTIM_EMR	                	EQU     (WDTIM_BASE_ADDR + 0x14)
WDTIM_PULSE	                	EQU     (WDTIM_BASE_ADDR + 0x18)
WDTIM_RES	                	EQU     (WDTIM_BASE_ADDR + 0x1C)

MOTOR_BASE_ADDR                 EQU     0x400E8000
MCCON                           EQU     (MOTOR_BASE_ADDR + 0x00)
MCCON_SET                       EQU     (MOTOR_BASE_ADDR + 0x04)    
MCCON_CLR                       EQU     (MOTOR_BASE_ADDR + 0x08)    
MCCAPCON                        EQU     (MOTOR_BASE_ADDR + 0x0C)   
MCCAPCON_SET                    EQU     (MOTOR_BASE_ADDR + 0x10)       
MCCAPCON_CLR                    EQU     (MOTOR_BASE_ADDR + 0x14)      
MCTC0                           EQU     (MOTOR_BASE_ADDR + 0x18)
MCTC1                           EQU     (MOTOR_BASE_ADDR + 0x1C)
MCTC2                           EQU     (MOTOR_BASE_ADDR + 0x20)
MCLIM0                          EQU     (MOTOR_BASE_ADDR + 0x24) 
MCLIM1                          EQU     (MOTOR_BASE_ADDR + 0x28) 
MCLIM2                          EQU     (MOTOR_BASE_ADDR + 0x2C) 
MCMAT0                          EQU     (MOTOR_BASE_ADDR + 0x30)
MCMAT1                          EQU     (MOTOR_BASE_ADDR + 0x34) 
MCMAT2                          EQU     (MOTOR_BASE_ADDR + 0x38) 
MCDT                            EQU     (MOTOR_BASE_ADDR + 0x3C)
MCCP                            EQU     (MOTOR_BASE_ADDR + 0x40) 
MCCAP0                          EQU     (MOTOR_BASE_ADDR + 0x44)
MCCAP1                          EQU     (MOTOR_BASE_ADDR + 0x48) 
MCCAP2                          EQU     (MOTOR_BASE_ADDR + 0x4C)
MCINTEN                         EQU     (MOTOR_BASE_ADDR + 0x50) 
MCINTEN_SET                     EQU     (MOTOR_BASE_ADDR + 0x54)      
MCINTEN_CLR                     EQU     (MOTOR_BASE_ADDR + 0x58)     
MCINTFLAG                       EQU     (MOTOR_BASE_ADDR + 0x68)    
MCINTFLAG_SET                   EQU     (MOTOR_BASE_ADDR + 0x6C)        
MCINTFLAG_CLR                   EQU     (MOTOR_BASE_ADDR + 0x70)        
MCCAP_CLR                       EQU     (MOTOR_BASE_ADDR + 0x74)   

PWM_BASE_ADDR                   EQU     0x4005C000
PWM1_CTRL                       EQU     (PWM_BASE_ADDR + 0x00)                  
PWM2_CTRL                       EQU     (PWM_BASE_ADDR + 0x04)                 

MUX_BASE_ADDR                   EQU     0x40028000
P_MUX_SET                       EQU     (MUX_BASE_ADDR + 0x100)
P_MUX_CLR                       EQU     (MUX_BASE_ADDR + 0x104)
P_MUX_STATE                     EQU     (MUX_BASE_ADDR + 0x108)
P0_MUX_SET                      EQU     (MUX_BASE_ADDR + 0x120)
P0_MUX_CLR                      EQU     (MUX_BASE_ADDR + 0x124)
P0_MUX_STATE                    EQU     (MUX_BASE_ADDR + 0x128)
P1_MUX_SET                      EQU     (MUX_BASE_ADDR + 0x130)
P1_MUX_CLR                      EQU     (MUX_BASE_ADDR + 0x134)
P1_MUX_STATE                    EQU     (MUX_BASE_ADDR + 0x138)
P2_MUX_SET                      EQU     (MUX_BASE_ADDR + 0x028)
P2_MUX_CLR                      EQU     (MUX_BASE_ADDR + 0x02C)
P2_MUX_STATE                    EQU     (MUX_BASE_ADDR + 0x030)
P3_MUX_SET                      EQU     (MUX_BASE_ADDR + 0x110)
P3_MUX_CLR                      EQU     (MUX_BASE_ADDR + 0x114)
P3_MUX_STATE                    EQU     (MUX_BASE_ADDR + 0x118)

P0_BASE_ADDR                    EQU     0x40028040
P0_INP_STATE                    EQU     (P0_BASE_ADDR + 0x00)
P0_OUTP_SET                     EQU     (P0_BASE_ADDR + 0x04)
P0_OUTP_CLR                     EQU     (P0_BASE_ADDR + 0x08)
P0_OUTP_STATE                   EQU     (P0_BASE_ADDR + 0x0C)
P0_DIR_SET                      EQU     (P0_BASE_ADDR + 0x10)
P0_DIR_CLR                      EQU     (P0_BASE_ADDR + 0x14)
P0_DIR_STATE                    EQU     (P0_BASE_ADDR + 0x18)

P1_BASE_ADDR                    EQU     0x40028060
P1_INP_STATE                    EQU     (P1_BASE_ADDR + 0x00)
P1_OUTP_SET                     EQU     (P1_BASE_ADDR + 0x04)
P1_OUTP_CLR                     EQU     (P1_BASE_ADDR + 0x08)
P1_OUTP_STATE                   EQU     (P1_BASE_ADDR + 0x0C)
P1_DIR_SET                      EQU     (P1_BASE_ADDR + 0x10)
P1_DIR_CLR                      EQU     (P1_BASE_ADDR + 0x14)
P1_DIR_STATE                    EQU     (P1_BASE_ADDR + 0x18)

P2_BASE_ADDR                    EQU     0x40028010
P2_INP_STATE                    EQU     (P2_BASE_ADDR + 0x0C)
P2_OUTP_SET                     EQU     (P2_BASE_ADDR + 0x10)
P2_OUTP_CLR                     EQU     (P2_BASE_ADDR + 0x14)
P2_DIR_SET                      EQU     (P2_BASE_ADDR + 0x00)
P2_DIR_CLR                      EQU     (P2_BASE_ADDR + 0x04)
P2_DIR_STATE                    EQU     (P2_BASE_ADDR + 0x08)

P3_BASE_ADDR                    EQU     0x40028000
P3_INP_STATE                    EQU     (P3_BASE_ADDR + 0x00)
P3_OUTP_SET                     EQU     (P3_BASE_ADDR + 0x04)
P3_OUTP_CLR                     EQU     (P3_BASE_ADDR + 0x08)
P3_OUTP_STATE                   EQU     (P3_BASE_ADDR + 0x0C)

;/********************************************************************************************************
;    Define firmware Functions
;********************************************************************************************************/

rm_init_entry                   EQU     0x7fffff91
rm_undef_handler                EQU     0x7fffffa0
rm_prefetchabort_handler        EQU     0x7fffffb0
rm_dataabort_handler            EQU     0x7fffffc0
rm_irqhandler                   EQU     0x7fffffd0
rm_irqhandler2                  EQU     0x7fffffe0
iap_entry                       EQU     0x7ffffff1

    END
;/********************************************************************************************************
;**                            End Of File
;********************************************************************************************************/