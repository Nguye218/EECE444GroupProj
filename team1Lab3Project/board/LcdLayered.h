/****************************************************************************************
* LcdLayered.h - A MicroC/OS driver for for a Hitachi-type LCD Display
*
*                This LCD driver implements the concept of layers.
*                This allows asynchronous application tasks to write to
*                a single LCD display without interfering with each
*                other.
*
*                Requires the following be defined in app_cfg.h:
*                   APP_CFG_LCD_TASK_PRIO
*                   APP_CFG_LCD_TASK_STK_SIZE
*
*                It is derived from the work of Matthew Cohn, 2/26/2008
*
*                Cursor code is derived from Keegan Morrow, 02/22/2013
*
* Todd Morton, 02/26/2013, First Revised Release
* 01/22/2015, Added to git repo, general clean up. TDM
* 02/03/2016, More cleanup. TDM
* 01/13/2017 Changed name to LcdLayered (was LayeredLcd), fixed bugs. TDM
* 01/18/2018 Changed to replace includes.h TDM
* 01/20/2019 Changed for MCUXpresso and added LcdDispDecWord(). TDM
* 03/09/2019 Added additional defines and modified LcdDispDecWord. Brad Cowgill
****************************************************************************************/

#ifndef LCD_DEF
#define LCD_DEF
/*************************************************************************
* LCD Layers - Define all layer values here                              *
*              Range from 0 to (LCD_NUM_LAYERS - 1)                      *
*              Arranged from largest number on top, down to 0 on bottom. *
*************************************************************************/
#define LCD_NUM_LAYERS 4

#define LCD_LAYER_MODE       0
#define LCD_LAYER_USER_INPUT 1
#define LCD_LAYER_TSI_VALUE  2
#define LCD_LAYER_WAVE_DATA  3

/*************************************************************************
* LCD Rows and Columns Defines
*
*************************************************************************/
// LCD ROWS
#define LCD_ROW_1 1
#define LCD_ROW_2 2

// LCD COLUMNS
#define LCD_COL_1 1
#define LCD_COL_2 2
#define LCD_COL_3 3
#define LCD_COL_4 4
#define LCD_COL_5 5
#define LCD_COL_6 6
#define LCD_COL_7 7
#define LCD_COL_8 8
#define LCD_COL_9 9
#define LCD_COL_10 10
#define LCD_COL_11 11
#define LCD_COL_12 12
#define LCD_COL_13 13
#define LCD_COL_14 14
#define LCD_COL_15 15
#define LCD_COL_16 16

/*************************************************************************
* Enumerated type for mode parameter in LcdDispDecWord()
*
*************************************************************************/

typedef enum {
    LCD_DEC_MODE_LZ,
    LCD_DEC_MODE_AR,
    LCD_DEC_MODE_AL
} LCD_MODE;

/*************************************************************************
  Public Functions
*************************************************************************/

void LcdInit(void);

void LcdDispChar(INT8U row,INT8U col,INT8U layer,INT8C c);

void LcdDispString(INT8U row,INT8U col,INT8U layer,
                          const INT8C *string);
                          
void LcdDispTime(INT8U row,INT8U col,INT8U layer,
                        INT8U hrs,INT8U mins,INT8U secs);
                        
void LcdDispByte(INT8U row,INT8U col,INT8U layer,INT8U byte);
                        
void LcdDispDecWord(INT8U row, INT8U col, INT8U layer, INT32U binword, INT8U field, LCD_MODE mode);
void LcdDispClear(INT8U layer);

void LcdDispClrLine(INT8U row, INT8U layer);
INT8U LcdCursor(INT8U row, INT8U col, INT8U layer, INT8U on, INT8U blink);
void LcdHideLayer(INT8U layer);
void LcdShowLayer(INT8U layer);
void LcdToggleLayer(INT8U layer);
#endif

