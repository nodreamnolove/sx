/****************************************Copyright (c)**************************************************
**                               Guangzou ZLG-MCU Development Co.,LTD.
**                                      graduate school
**                                 http://www.zlgmcu.com
**
**--------------File Info-------------------------------------------------------------------------------
** File name:           LCD2478.h
** Last modified Date:  2008-04-07
** Last Version:        1.0
** Descriptions:        the lcd controller driver's head
**
**------------------------------------------------------------------------------------------------------
** Created by:          Houxiaolong
** Created date:        2008-04-07
** Version:             1.0
** Descriptions:        the lcd controller driver's head
**
**------------------------------------------------------------------------------------------------------
** Modified by:         Houxiaolong
** Modified date:       2008-07-10
** Version:             1.1
** Descriptions:        add the mirror operation macro
**                      
********************************************************************************************************/

#ifndef _LCD_H_
#define _LCD_H_

#define U_LCD_XSIZE     320                                             /*  LCD x size                  */
#define U_LCD_YSIZE     240                                             /*  LCD y size                  */

#define LcdPwr          (1 <<11)                                               
#define LcdTFT          1                                               
#define LcdBpp          6                                               
#define LcdEn           1

#define HBP             3                                               /*  Horizontal back porch       */
#define HFP             7                                               /*  Horizontal front porch      */
#define HSW             3                                               /*  Horizontal pulse width      */

#define VBP             1                                               /*  Vertical back porch         */
#define VFP             4                                               /*  Vertical front porch        */
#define VSW             0                                               /*  Vertical pulse width        */

#define PPL             ((U_LCD_XSIZE / 16) - 1)                        /*  Pixels-per-line             */

#define LPP             (U_LCD_YSIZE - 1)                               /*  clock per line              */

#define CPL             (U_LCD_XSIZE - 1)                               /*  clock per line              */

#define MIRROR_X        0                                               /*  x size mirror               */
#define MIRROR_Y        0                                               /*  y size mirror               */

#define HWORDMODE       1                                               /*  picture data                */

#define LCD_RED         0x001f                                          /*  red color                   */
#define LCD_GREEN       0x07e0                                          /*  green color                 */
#define LCD_BLUE        0xf800                                          /*  blue color                  */
#define LCD_BLACK       0x0000                                          /*  black color                 */
#define LCD_WHITE       0xffff                                          /*  white color                 */

extern void lcd_Point (void);
extern __align(8) volatile unsigned short LCD_BUFFER[U_LCD_YSIZE][U_LCD_XSIZE];

/**********************************************************************************************************
**  mirror operation
**********************************************************************************************************/
#if   (!MIRROR_X && !MIRROR_Y)
    #define    SECTION_X(x, y)    x
    #define    SECTION_Y(x, y)    y
#elif (MIRROR_X && !MIRROR_Y)
    #define    SECTION_X(x, y)    U_LCD_XSIZE - 1 - x
    #define    SECTION_Y(x, y)    y
#elif (!MIRROR_X && MIRROR_Y)
    #define    SECTION_X(x, y)    x
    #define    SECTION_Y(x, y)    U_LCD_YSIZE - 1 - y
#elif (MIRROR_X && MIRROR_Y)
    #define    SECTION_X(x, y)    U_LCD_XSIZE - 1 - x
    #define    SECTION_Y(x, y)    U_LCD_YSIZE - 1 - y
#endif   

/*********************************************************************************************************
**  macro definition of lcd_SetPixel
*********************************************************************************************************/

#define   lcd_SetPixel(x, y, color)                                                                      \
              do {                                                                                       \
                  if((x < U_LCD_XSIZE) &&  (y < U_LCD_YSIZE)) {                                          \
                      LCD_BUFFER[SECTION_Y(x, y)][SECTION_X(x, y)] = (unsigned short)uicolor;            \
                  }                                                                                      \
              } while (0)
                                                  
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
void lcd_Init(void);
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
int lcd_ReadPixel(int ix, int iy);
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
void lcd_Clear(unsigned int uicolor);
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
void lcd_OnOff(int iOnOff);
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
void lcd_DrawHLine(int ix0, int ix1, int iy, unsigned int uicolor);
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
void lcd_DrawVLine (int ix, int iy0, int iy1, unsigned int uicolor);
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
void lcd_DrawLine(int ix0, int iy0, int ix1, int iy1, unsigned int uicolor);
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
void lcd_DrawRect(int ix0, int iy0, int ix1, int iy1, unsigned int uicolor);
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
void lcd_FillRect(int ix0, int iy0, int ix1, int iy1, unsigned int uicolor);

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
void lcd_DrawBmp(int ix0, 
                 int iy0, 
                 int iwidth, 
                 int ilength, 
                 unsigned short *pucBmp);
                 
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
void lcd_DrawBmp(int ix0, 
                 int iy0, 
                 int iwidth, 
                 int ilength, 
                 unsigned char *pucBmp);

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
void delay_Ns(int idly);

#endif
