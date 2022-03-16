/********************************************************************
* uCOSKey.h - A keypad module that runs under MicroC/OS for a 4x4
* matrix keypad.
* This version allows for multiple column keys to be pressed and
* mapped to a different code by changing ColTable[] in keyScan().
* Multiple rows can not be resolved. The topmost button will be used.
* The keyCodeTable[] is currently set to generate ASCII codes.
*
* Requires the following be defined in app_cfg.h:
*                   APP_CFG_KEY_TASK_PRIO
*                   APP_CFG_KEY_TASK_STK_SIZE
*
* 02/20/2001 TDM Original key.c for 9S12
* 01/14/2013 TDM Modified for K70 custom tower board.
* 02/12/2013 TDM Modified to run under MicroC/OS-III
* 01/18/2018 Changed to replace includes.h TDM
* 01/20/2019 Changed for MCUXpresso TDM
*********************************************************************
* Public Resources
********************************************************************/
#ifndef UC_KEY_DEF
#define UC_KEY_DEF

/*****************************************************************************************
* Defines for the alpha keys: A, B, C, D
* These are ASCII control characters
*****************************************************************************************/
#define DC1 (INT8C)0x11     /*ASCII control code for the A button */
#define DC2 (INT8C)0x12     /*ASCII control code for the B button */
#define DC3 (INT8C)0x13     /*ASCII control code for the C button */
#define DC4 (INT8C)0x14     /*ASCII control code for the D button */

INT8C KeyPend(INT16U tout, OS_ERR *os_err); /* Pend on key press*/
                             /* tout - semaphore timeout           */
                             /* *err - destination of err code     */
                             /* Error codes are identical to a semaphore */

void KeyInit(void);             /* Keypad Initialization    */

#endif
