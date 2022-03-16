/*
 * SysTickDelay.c
 *
 *  Created on: Nov 12, 2021
 *      Author: ASUS
 */


/*****************************************************************************************
* SysTickDelay.c - A delay module based on the system timer
* v1.1
* 10/30/14 Todd Morton
* v2.1 Modify for k65 at 180MHz
* 11/04/2015 Todd Morton
* 11/05/2017 Todd Morton Modify for new header file structure.
* v3.1 Modify for MCUXpresso and add SysTickmsCount()
* 10/23/2018 Todd Morton
* v4.1 Modified for MCUX11.2
* 10/21/2020 Todd Morton
******************************************************************************************
* Project master header file
*****************************************************************************************/
#include "MCUType.h"
#include "SysTickDelay.h"
#include "K65TWR_GPIO.h"

/*****************************************************************************************
* Handler must not be static so linker can see it.
*****************************************************************************************/
void SysTick_Handler(void);  /* SysTick interrupt service routine*/

/*****************************************************************************************
* Private Resources
*****************************************************************************************/
static volatile INT32U stmsCount;   /* 1ms counter variable */
static INT32U stSliceCount;   /* 1ms counter variable */
static INT8U stInitFlag;
static INT32U stLastEvent;

/*****************************************************************************************
* Module Defines
*****************************************************************************************/
#define CLK_PER_MS 180000U          /* Clock cycles per 1ms, (must be < 16777216)        */

/*****************************************************************************************
* SysTickDelay Function
*    - Public
*    - Delays 'ms' milliseconds
*    - Accuracy +0/-1 ms
*****************************************************************************************/
void SysTickDelay(const INT32U ms){
    INT32U start_cnt;
    start_cnt = stmsCount;
    while((stmsCount - start_cnt) < ms){} /* wait for ms to pass*/
}

/*****************************************************************************************
* Period Delay Function
*    - Public - NOT reentrant...in fact only one instance.
*    - Wait to next event every 'period' milliseconds
*    - Accuracy +0/-1 ms
*****************************************************************************************/
void SysTickWaitEvent(const INT32U period){

    DB0_TURN_ON();
    if(stInitFlag == 1){
        while((stmsCount - stLastEvent) < period){}
    }else{
        stInitFlag = 1;
    }
    stLastEvent = stmsCount;
    stSliceCount++;
    DB0_TURN_OFF();
}

/*****************************************************************************************
* SysTickDlyInit() - Initialization routine for SysTickDelay()
*****************************************************************************************/
void SysTickDlyInit(void){
    stInitFlag = 0;
    stmsCount = 0;
    stSliceCount = 0;
    stLastEvent = 0;
    (void)SysTick_Config(CLK_PER_MS);
}
/*****************************************************************************************
* SysTickGetmsCount() - Get the value of the millisecond counter. Abstract with function
*                       so it is read only.
*****************************************************************************************/
INT32U SysTickGetmsCount(void){
    return stmsCount;
}

/*****************************************************************************************
* SysTickGetSliceCount() - Get the value of the slice counter. Abstract with
*                          function so it is read only.
*****************************************************************************************/
INT32U SysTickGetSliceCount(void){
    return stSliceCount;
}
/*****************************************************************************************
* SysTick_Handler() - System Tick Interrupt Handler.
*    - setup for a 1ms periodic interrupt.
*****************************************************************************************/
void SysTick_Handler(void){
    stmsCount++;                    /* Increment 1ms counter    */
}
/****************************************************************************************/
