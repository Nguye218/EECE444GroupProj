/****************************************************************************************
* SysTickDelay.h - Project Header file for the System Tick - based
*             delay routine.
*
* v1.1
* 10/30/14 Todd Morton
* v2.1 Modify for k65 at 180MHz
* 11/04/2015 Todd Morton
* 11/05/2017 Todd Morton Modify for new header file structure.
* v3.1 Modify for MCUXpresso and add SysTickmsCount()
* 10/23/2018 Todd Morton
*****************************************************************************************
* Public Function Prototypes
****************************************************************************************/
#ifndef SYS_TICK_INC
#define SYS_TICK_INC
/****************************************************************************************
 * SysTickDelay()
 * Blocking delay routine. The parameter is the number of ms to delay.
 ***************************************************************************************/
void SysTickDelay(const INT32U ms);

/****************************************************************************************
 * SysTickDlyInit()
 * SysTickDelay Initialization Routine. This function must be called before
 * any of the other SysTickDelay functions are called.
 ***************************************************************************************/
void SysTickDlyInit(void);

/****************************************************************************************
 * SysTickWaitEvent()
 * SysTickWaitEvent is a periodic blocking routine. It's more like a task - it should
 * only be called one time in timed event or task loop.
 * That is: ONLY ONE INSTANCE is allowed.
 ***************************************************************************************/
void SysTickWaitEvent(const INT32U period);

/*****************************************************************************************
* GetmsCount() - Get the value of the millisecond counter. Abstracted with a function so
*                it is read only.
*****************************************************************************************/
INT32U SysTickGetmsCount(void);

/*****************************************************************************************
* GetSliceCount() - Get the value of the slice counter. Abstracted with a function so it
*                   is read only.
*****************************************************************************************/
INT32U SysTickGetSliceCount(void);

#endif
