/*
*********************************************************************************************************
*                                              EXAMPLE CODE
*
*                              (c) Copyright 2013; Micrium, Inc.; Weston, FL
*
*               All rights reserved.  Protected by international copyright laws.
*               Knowledge of the source code may NOT be used to develop a similar product.
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                      APPLICATION CONFIGURATION
*
*                                        Freescale Kinetis K60
*                                               on the
*
*                                        Freescale TWR-K60N512
*                                          Evaluation Board
*
* Filename      : app_cfg.h
* Version       : V1.00
* Programmer(s) : DC
*********************************************************************************************************
*/

#ifndef  APP_CFG_MODULE_PRESENT
#define  APP_CFG_MODULE_PRESENT


/*
*********************************************************************************************************
*                                       ADDITIONAL uC/MODULE ENABLES
*********************************************************************************************************
*/
#define  APP_CFG_SERIAL_EN                          DEF_DISABLED //Change to disabled. TDM
/*
*********************************************************************************************************
*                                            TASK PRIORITIES
*********************************************************************************************************
*/
#define APP_CFG_TASK_START_PRIO           2u
#define APP_CFG_KEY_TASK_PRIO             4u
#define APP_CFG_LCD_TASK_PRIO             6u
#define APP_CFG_UI_TASK_PRIO              8u
#define APP_CFG_TSI_START_PRIO            10u
#define APP_CFG_TSI_CNTR_TASK_PRIO        12u
#define APP_CFG_PULSE_WAVE_TASK_PRIO      14u
#define APP_CFG_PROCESSING_TASK_PRIO      16u

/*
*********************************************************************************************************
*                                            TASK STACK SIZES
*********************************************************************************************************
*/
#define APP_CFG_TASK_START_STK_SIZE          128u
#define APP_CFG_UI_TASK_STK_SIZE             128u
#define APP_CFG_LCD_TASK_STK_SIZE            128u
#define APP_CFG_KEY_TASK_STK_SIZE            128u
#define APP_CFG_TSI_TASK_STK_SIZE            128u
#define APP_CFG_TSI_CNTR_TASK_STK_SIZE       128u
#define APP_CFG_DISP_TASK_STK_SIZE	         128u
#define APP_CFG_PULSE_WAVE_TASK_STK_SIZE     128u
#define APP_CFG_PROC_TASK_STK_SIZE           128u

#endif
