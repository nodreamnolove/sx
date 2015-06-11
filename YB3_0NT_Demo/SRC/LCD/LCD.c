/****************************************Copyright (c)****************************************************
**                               Guangzou ZLG-MCU Development Co.,LTD.
**                                      graduate school
**                                 http://www.zlgmcu.com
**
**--------------File Info---------------------------------------------------------------------------------
** File name:           LCD2478.C
** Last modified Date:  2008-04-07
** Last Version:        1.2
** Descriptions:        the lcd controller's driver
**
**--------------------------------------------------------------------------------------------------------
** Created by:          Houxiaolong
** Created date:        2008-04-07
** Version:             1.0
** Descriptions:        the lcd controller's driver
**
**--------------------------------------------------------------------------------------------------------
** Modified by:         Houxiaolong
** Modified date:       2008-04-23
** Version:             1.1
** Descriptions:        add the lcd_OnOff function 
**-------------------------------------------------------------------------------------------------------
** Modified by:         Houxiaolong
** Modified date:       2008-05-5
** Version:             1.2
** Descriptions:        add the hardware cursor initialize
**                      
*********************************************************************************************************/
#include "config.h"
#include "LCD.h"
#include "cursor.h"

__align(8) volatile unsigned short LCD_BUFFER[U_LCD_YSIZE][U_LCD_XSIZE];/*  display data buffer         */

void delay_Ns(int idly);

/*********************************************************************************************************
** Function name:           lcd_Init
**
** Descriptions:            Initialize the LCD controler
**
** input parameters:        None
** output parameters:       None
** Returned value:          None
**
** Created by:              dengzhi
** Created Date:            2009/03/22
**--------------------------------------------------------------------------------------------------------
** Modified by:
** Modified date:
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void lcd_Init (void)
{
    delay_Ns(100);
    LCD_CTRL   = 0;                                                     /*  power disable               */
    delay_Ns(100);
    
    LCDCLK_CTRL = (0x01 << 6) |
                  (0x01 << 5) |
                  (0x10 << 0)                                           /*  pixel clock 21MHZ            */
                  ;                                                      
                  
    LCD_UPBASE = (unsigned int)(LCD_BUFFER);                            /*  set buffer's base address   */


    LCD_TIMH   = (HBP << 24) |                                          /*  set horizontal timing       */
                 (HFP << 16) |                                            
                 (HSW << 8)  |
                 (PPL << 2)
                 ;
    
    LCD_TIMV   = (VBP << 24) |                                          /*  set vertical timing         */
                 (VFP << 16) |
                 (VSW << 10) |
                 (LPP << 0);
               
    LCD_POL    = (1 << 26)   |                                          /*  bypass pixel color driver   */
                 (CPL << 16) |                                          /*  240 clock per line          */
                 (0 << 14)   |                                          /*  LCDENAB output pin is active
                                                                            HIGH in TFT mode            */
                 (1 << 13)   |                                          /*  Data is driven on the LCD on 
                                                                            the rising edge of LCDDCLK  */
                 (1 << 12)   |                                          /*  HSYNC is active low         */                                           
                 (1 << 11)   |                                          /*  VSYNC is active low         */
                 (0 << 5);                                              /*  select HCLK                 */
    
    LCD_CTRL   = (0x01  << 16)|
                 (LcdTFT << 5) |                                        /*  select TFT LCD type         */
                 (LcdBpp << 1) |                                        /*  select 16bpp                */
                 (LcdEn);                                               /*  LCD enable                  */
    
    LCD_INTMSK = 0;                                                     /*  disable LCD interrupt       */
    delay_Ns(100);
    LCD_CTRL  |= LcdPwr;                                                /*  Power enable                */
}    
  
/*********************************************************************************************************
** Function name:           lcd_ReadPixel
**
** Descriptions:            Get the pixel's color
**
** Input:                   ix       x side coordinate of the pixel
**                          iy       y side coordinate of the pixel 
** Output:                  succeed  return the pixel's color
**                          fail     0
** Created by:              
** Created Date:            
**--------------------------------------------------------------------------------------------------------
** Modified by:
** Modified date:
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
int lcd_ReadPixel (int ix, int iy)
{
    if ((ix < U_LCD_XSIZE) && (iy < U_LCD_YSIZE)) {
        return LCD_BUFFER[SECTION_Y(ix, iy)][SECTION_X(ix, iy)];
    } 
    
    return 0;
}  


/*********************************************************************************************************
** Function name:           lcd_Clear
**
** Descriptions:            Fill the LCD panel with a color
**
** Input:                   uicolor   LCD display color 
** Output:                  none
** Created by:              
** Created Date:            
**--------------------------------------------------------------------------------------------------------
** Modified by:
** Modified date:
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void lcd_Clear (unsigned int uicolor)
{
    int ix, iy;
    

    for (iy = 0; iy <U_LCD_YSIZE; iy++) {
        for (ix = 0; ix < U_LCD_XSIZE; ix++) {
            LCD_BUFFER[SECTION_Y(ix, iy)][SECTION_X(ix, iy)] = (unsigned short)uicolor;
        }
    }
}

/*********************************************************************************************************
** Function name:           lcd_Point
**
** Description :            Fill the LCD panel with a color
** Input:                   uicolor   LCD display color
** Output:                  none
** Created by:              
** Created Date:            
**--------------------------------------------------------------------------------------------------------
** Modified by:
** Modified date:
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void lcd_Point (void)
{
    int ix, iy;
    unsigned short uicolor = 0;

    for (iy = 0; iy <U_LCD_YSIZE; iy++) {
        for (ix = 0; ix < U_LCD_XSIZE; ix++) {
            LCD_BUFFER[iy][ix] = (unsigned short)uicolor ++;
        }
    }
}
/*********************************************************************************************************
** Function name:           lcd_OnOff
**
** Description :            enable or disable the lcd data output
** Input:                   iOnOff  1:enable lcd data output 0: disable lcd data output
** Output:                  none
**                          fail     0
** Created by:              
** Created Date:            
**--------------------------------------------------------------------------------------------------------
** Modified by:
** Modified date:
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void lcd_OnOff(int iOnOff)
{
    if (iOnOff==1) {
        LCD_CTRL |= (1 << 11);                                          /*  Lcd controller enable       */
    } else {
        LCD_CTRL &= ~(1 << 11);                                         /*  Lcd controller disable      */
    }
}

/*********************************************************************************************************
** Function name:           lcd_DrawHLine
**
** Description :            Draw a horizontal line
** Input:                   ix0      x side coordinate of the first pixel
**                          ix1      x side coordinate of the last pixel
**                          iy       y side coordinate of the line
**                          uicolor  the color of the pixel 
** Output:                  none
** Created by:              
** Created Date:            
**--------------------------------------------------------------------------------------------------------
** Modified by:
** Modified date:
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void lcd_DrawHLine (int ix0, int ix1, int iy, unsigned int uicolor)
{
    int i;
    for (i = ix0; i < ix1; i ++) {
        lcd_SetPixel(i, iy, uicolor);
    }
}

/*********************************************************************************************************
** Function name:           lcd_DrawVLine
**
** Description :            Draw a vertical line
** Input:                   ix       x side coordinate of the line
**                          iy0      y side coordinate of the first pixel
**                          iy1      y side coordinate of the last pixel
**                          uicolor  the color of the pixel 
** Output:                  none
** Created by:              
** Created Date:            
**--------------------------------------------------------------------------------------------------------
** Modified by:
** Modified date:
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void lcd_DrawVLine (int ix, int iy0, int iy1, unsigned int uicolor)
{
    int i;
    for (i = iy0; i < iy1; i ++) {
        lcd_SetPixel(ix, i, uicolor);
    }
}


/*********************************************************************************************************
** Function name:           lcd_DrawLine
**
** Description :            Draw a line
** Input:                   ix0      x side coordinate of the first pixel
**                          iy0      y side coordinate of the first pixel
**                          ix1      x side coordinate of the last pixel
**                          iy1      y side coordinate of the last pixel
**                          uicolor  the color of the pixel 
** Output:                  none
** Created by:              
** Created Date:            
**--------------------------------------------------------------------------------------------------------
** Modified by:
** Modified date:
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void lcd_DrawLine (int ix0, int iy0, int ix1, int iy1, unsigned int uicolor)
{
    signed char dx_sym;                                                 /*  increase size of x          */
    signed char dy_sym;                                                 /*  increase size of y          */
    int  dx;                                                            /*  line of x's offset          */
    int  dy;                                                            /*  line of y's offset          */
    int  dx_x2;
    int  dy_x2;
    int  di;
    
    dx = ix1 - ix0;                                                     /*  two pixel's offset          */
    dy = iy1 - iy0;
    
    if (dx > 0) {
        dx_sym = 1;                                                     /*  decide the increase size    */
    } else {
        if (dx < 0) {
            dx_sym = -1;
        } else {
            if (dy >= 0) {
                lcd_DrawVLine(ix0, iy0, iy1, uicolor);                  /*  dx==0,draw a vertical line  */
            } else {
                lcd_DrawVLine(ix0, iy1, iy0, uicolor);  
            }                
            return;
        }
    }
    
    if (dy > 0) {
        dy_sym = 1;
    } else {
        if (dy < 0) {
            dy_sym = -1;
        } else {
            if (dx >= 0) {
                lcd_DrawHLine(ix0, ix1, iy0, uicolor);                  /*  dy==0,Draw a horizontal line*/
            } else {
                lcd_DrawHLine(ix1, ix0, iy0, uicolor);
            }
            return;
        }
    }
    
    dx    = dx_sym * dx;                                                /*  get absolute value          */
    dy    = dy_sym * dy;
    
    dx_x2 = dx * 2;                                                     /*  double dx                   */
    dy_x2 = dy * 2;                                                     /*  double dy                   */
    
    /*
     *  dx >= dy, use x as benchmark
     */
    if (dx >= dy) {                                              
        di = dy_x2 - dx;
        
        while (ix0 != ix1) {
            lcd_SetPixel(ix0, iy0, uicolor);
            ix0 += dx_sym;
            
            if (di < 0) {
                di  += dy_x2;
            } else {
                di  += dy_x2 - dx_x2;
                iy0 += dy_sym;
            }
        }
        
        lcd_SetPixel(ix0, iy0, uicolor);                                  /*  display the last pixel      */
        
    } else {
       /*
        *  dx < dy, use y as benchmark
        */
        di = dx_x2 - dy;
        
        while (iy0 != iy1) {
            lcd_SetPixel(ix0, iy0, uicolor);
            iy0 += dy_sym;
            
            if (di < 0) {
                di  += dx_x2;
            } else {
                di  += dx_x2 - dy_x2;
                ix0 += dx_sym;
            }
        }
        
        lcd_SetPixel(ix0, iy0, uicolor);                                  /*  display the last pixel      */
    }
}    
/*********************************************************************************************************
** Function name:           lcd_DrawRect
**
** Description :            Draw a rectangle
** Input:                   ix0      x side coordinate of the first pixel
**                          iy0      y side coordinate of the first pixel
**                          ix1      x side coordinate of the last pixel
**                          iy1      y side coordinate of the last pixel
**                          uicolor  the color of the pixel 
** Output:                  none
** Created by:              
** Created Date:            
**--------------------------------------------------------------------------------------------------------
** Modified by:
** Modified date:
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void lcd_DrawRect (int ix0, int iy0, int ix1, int iy1, unsigned int uicolor)
{
        lcd_DrawLine(ix0,iy0,ix1,iy0,uicolor);
        lcd_DrawLine(ix1,iy0,ix1,iy1,uicolor);
        lcd_DrawLine(ix1,iy1,ix0,iy1,uicolor);
        lcd_DrawLine(ix0,iy1,ix0,iy0,uicolor);           
}   

/*********************************************************************************************************
** Function name:           lcd_FillRect
**
** Description :            Fill a rectangle
** Input:                   ix0      x side coordinate of the first pixel
**                          iy0      y side coordinate of the first pixel
**                          ix1      x side coordinate of the last pixel
**                          iy1      y side coordinate of the last pixel
**                          uicolor  the color of the pixel
** Output:                  none
** Created by:              
** Created Date:            
**--------------------------------------------------------------------------------------------------------
** Modified by:
** Modified date:
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void lcd_FillRect (int ix0, int iy0, int ix1, int iy1, unsigned int uicolor)
{
    int i;
    
    if (iy1 > iy0) {
        for (i = iy0; i <= iy1; i++) {
            lcd_DrawLine(ix0, i, ix1, i, uicolor);
        }
    } else {
        for (i = iy0; i >= iy1; i--) {
            lcd_DrawLine(ix0, i, ix1, i, uicolor);
        }
    }
            
}  



#if (HWORDMODE == 1)
/*********************************************************************************************************
** Function name:           lcd_DrawBmp
**
** Description :            Draw a bmp
** Input:                   ix0     x side coordinate of the first pixel
**                          iy0     y side coordinate of the first pixel
**                          iwidth  x side width
**                          ilength y side length
**                          pucBmp  the bmp array pointer
** Output:                  none
** Created by:              
** Created Date:            
**--------------------------------------------------------------------------------------------------------
** Modified by:
** Modified date:
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void lcd_DrawBmp (int ix0, 
                  int iy0, 
                  int iwidth, 
                  int ilength, 
                  unsigned short *pucBmp)
{
    int ix, iy;
    unsigned int uicolor;
    
    for (iy = iy0; iy < ilength + iy0; iy++) {
        for (ix = ix0; ix < iwidth + ix0; ix++) {
            uicolor = *pucBmp++;
            lcd_SetPixel(ix, iy, uicolor);
        }
    }          
}


#else
/*********************************************************************************************************
** Function name:           lcd_DrawBmp
**
** Description :            Draw a bmp
** Input:                   ix0     x side coordinate of the first pixel
**                          iy0     y side coordinate of the first pixel
**                          iwidth  x side width
**                          ilength y side length
**                          pucBmp  the bmp array pointer
** Output:                  none
** Created by:              
** Created Date:            
**--------------------------------------------------------------------------------------------------------
** Modified by:
** Modified date:
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void lcd_DrawBmp (int ix0, 
                  int iy0, 
                  int iwidth, 
                  int ilength, 
                  unsigned char *pucBmp)
{
    int ix, iy;
    unsigned int uicolor;
    
    for (iy = iy0; iy < ilength + iy0; iy++) {
        for (ix = ix0; ix < iwidth + ix0; ix++) {
            uicolor = *(pucBmp + 1) | (*(pucBmp) << 8);
            lcd_SetPixel(ix, iy, uicolor);
            pucBmp += 2;
        }
    }
            
}

#endif
/*********************************************************************************************************
** Function name:           delay_Ns
**
** Description :            delay time
** Input:                   idly   the necessary time
** Output:                  no
** Created by:              
** Created Date:            
**--------------------------------------------------------------------------------------------------------
** Modified by:
** Modified date:
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void delay_Ns (int idly)
{
    int i;
    
    for(; idly > 0; idly--) {
        for (i = 0; i < 1000; i++);
    }
}



