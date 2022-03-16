/*****************************************************************************************
* EECE 444 Lab 3 Program code
* The program utilizes a user interface with implementation of touch sensors to output a
* sinewave or a pulsewave to DAC0. There are 21 levels that alters the sinewave amplitude or
* the duty cycle of the pulsewave, the right touchpad increases by 1, the left touchpad decreases
* by 1.
*
* Created by: Karen Aguilar, Tyler Roque, Rodrick Muya, Andy Nguyen
*
* 02/22/2022; Rodrick, Added Lcdlayered and uCOSkey and created UITASK
* 03/08/2022; Tyler, Added User input functionality check, cap on user input between 10k and 10Hz
* 03/08/2022; Tyler, built on previously implemented state machine to manage state transition's outputting input
* 03/13/2022: Karen, Rodrick sinewave and pulsewave implementation
* 03/13/2022: Andy, created initialization of EEPROM
* 03/13/2022: Tyler, completed EEPROM Functionality and proper startup
*****************************************************************************************/
#include "os.h"
#include "app_cfg.h"
#include "MCUType.h"
#include "K65TWR_ClkCfg.h"
#include "LCDLayered.h"
#include "uCOSKey.h"
#include "MemoryTools.h"
#include "K65TWR_TSI.h"
#include "MK65F18.h"
#include "SysTickDelay.h"
#include "Pulsetrain.h"
#include "K65TWR_GPIO.h"
#include "Sinewave.h"
#include "EEPROM.h"


typedef enum{SINE,PULSE} OUT_MODES_T;

typedef struct {
	INT32U sineFreqValue;
	INT32U sqrFreqValue;
	INT8U sineAmpValue;
	INT8U sqrCycleValue;
	OUT_MODES_T modeStateValue;
	INT16U checksumValue;
} EEWAVEINPUTS;

typedef union {
	EEWAVEINPUTS wave_inputs;
	INT16U eesettings[(sizeof(EEWAVEINPUTS)+1)/2];
} EEWAVESETTINGS;
static EEWAVESETTINGS EEWaveData;

//checksum to verify non-corrupted data
static INT16U confirm_checksum[sizeof(EEWaveData.eesettings) - 1];

#define INPUT_SIZE 8
#define MAX_INPUT 10000 //10kHz
#define MIN_INPUT 10 //10Hz
#define MAX_AMP 20
#define MIN_AMP 0
#define DEFAULT_FREQ 1000 //1000Hz
#define DEFAULT_AMP 10

/*****************************************************************************************
* Allocate task control blocks
*****************************************************************************************/
static OS_TCB appTaskStartTCB;
static OS_TCB appUITaskTCB;
static OS_TCB TSICounterTaskTCB;
/*****************************************************************************************
* User Input Mutex Key
*****************************************************************************************/
static OS_MUTEX appUserInputKey;
/*****************************************************************************************
* Allocate task stack space.
*****************************************************************************************/
static CPU_STK appTaskStartStk[APP_CFG_TASK_START_STK_SIZE];
static CPU_STK appUITaskStk[APP_CFG_UI_TASK_STK_SIZE];
static CPU_STK TSICounterTaskStk[APP_CFG_TSI_CNTR_TASK_STK_SIZE];
/*****************************************************************************************
* Task Function Prototypes. 
*   - Private if in the same module as startup task. Otherwise public.
*****************************************************************************************/
static void  appStartTask(void *p_arg);
static void  appUITask(void *p_arg);
static void TSICounterTask(void *p_arg);
/*****************************************************************************************
* main()
*****************************************************************************************/

void main(void) {

    OS_ERR  os_err;

    K65TWR_BootClock();
    CPU_IntDis();               /* Disable all interrupts, OS will enable them  */

    OSInit(&os_err);                    /* Initialize uC/OS-III                         */

    OSTaskCreate(&appTaskStartTCB,                  /* Address of TCB assigned to task */
                 "Start Task",                      /* Name you want to give the task */
                 appStartTask,                      /* Address of the task itself */
                 (void *) 0,                        /* p_arg is not used so null ptr */
                 APP_CFG_TASK_START_PRIO,           /* Priority you assign to the task */
                 &appTaskStartStk[0],               /* Base address of task�s stack */
                 (APP_CFG_TASK_START_STK_SIZE/10u), /* Watermark limit for stack growth */
                 APP_CFG_TASK_START_STK_SIZE,       /* Stack size */
                 0,                                 /* Size of task message queue */
                 0,                                 /* Time quanta for round robin */
                 (void *) 0,                        /* Extension pointer is not used */
                 (OS_OPT_TASK_NONE), /* Options */
                 &os_err);                          /* Ptr to error code destination */

    OSStart(&os_err);               /*Start multitasking(i.e. give control to uC/OS)    */
    while(1){                       /* Error Trap - should never get here               */
    }
}

/*****************************************************************************************
* STARTUP TASK
* This should run once and be deleted. Could restart everything by creating.
* Created by: Rodrick Muya, 02/02/2022
* Edited by: Tyler Roque, 03/13/2022 added EEPROM start up
*****************************************************************************************/
static void appStartTask(void *p_arg){
	INT16U read_checksum;
	INT16U calc_checksum;
	INT32U i;
    OS_ERR os_err;

    (void)p_arg;                        /* Avoid compiler warning for unused variable   */

    OS_CPU_SysTickInitFreq(SYSTEM_CLOCK);
    // Initialization
    LcdInit();
	GpioDBugBitsInit();
   	KeyInit();
   	PulseWaveInit();
	SineWaveInit();
	TSIInit();
	SPIInit();

	for(i = 0; i <= (sizeof(EEWaveData.eesettings)); i++) {
		EEWaveData.eesettings[i] = Spi2Read(0x00 + 2*i);
		if(i <= sizeof(EEWaveData.eesettings) - 1){
			confirm_checksum[i] = EEWaveData.eesettings[i];
		} else {
		}
	}
	read_checksum = EEWaveData.wave_inputs.checksumValue;
	calc_checksum = MemChkSum( (INT8U *)&confirm_checksum[0], ((INT8U *)&confirm_checksum[sizeof(EEWaveData.eesettings) - 1]));
	if(calc_checksum != read_checksum){ //Data corrupted, return to default settings
	    //Initial state and display
		EEWaveData.wave_inputs.sineAmpValue = DEFAULT_AMP;
		EEWaveData.wave_inputs.sqrCycleValue = DEFAULT_AMP;
		EEWaveData.wave_inputs.sineFreqValue = DEFAULT_FREQ;
		EEWaveData.wave_inputs.sqrFreqValue = DEFAULT_FREQ;
		EEWaveData.wave_inputs.modeStateValue = SINE;
	}else{
	}
	/* Check if the data read matches our accepted settings */
	if((EEWaveData.wave_inputs.sineAmpValue > MAX_AMP)) {
		EEWaveData.wave_inputs.sineAmpValue = DEFAULT_AMP;
		EEWaveData.wave_inputs.sqrCycleValue = DEFAULT_AMP;
		EEWaveData.wave_inputs.sineFreqValue = DEFAULT_FREQ;
		EEWaveData.wave_inputs.sqrFreqValue = DEFAULT_FREQ;
		EEWaveData.wave_inputs.modeStateValue = SINE;
	} else{
	}

	if((EEWaveData.wave_inputs.sqrCycleValue > MAX_AMP)) {
		EEWaveData.wave_inputs.sineAmpValue = DEFAULT_AMP;
		EEWaveData.wave_inputs.sqrCycleValue = DEFAULT_AMP;
		EEWaveData.wave_inputs.sineFreqValue = DEFAULT_FREQ;
		EEWaveData.wave_inputs.sqrFreqValue = DEFAULT_FREQ;
		EEWaveData.wave_inputs.modeStateValue = SINE;
	} else {
	}

	if((EEWaveData.wave_inputs.sineFreqValue > MAX_INPUT)) {
		EEWaveData.wave_inputs.sineAmpValue = DEFAULT_AMP;
		EEWaveData.wave_inputs.sqrCycleValue = DEFAULT_AMP;
		EEWaveData.wave_inputs.sineFreqValue = DEFAULT_FREQ;
		EEWaveData.wave_inputs.sqrFreqValue = DEFAULT_FREQ;
		EEWaveData.wave_inputs.modeStateValue = SINE;
	} else if((EEWaveData.wave_inputs.sineFreqValue < MIN_INPUT)){
		EEWaveData.wave_inputs.sineAmpValue = DEFAULT_AMP;
		EEWaveData.wave_inputs.sqrCycleValue = DEFAULT_AMP;
		EEWaveData.wave_inputs.sineFreqValue = DEFAULT_FREQ;
		EEWaveData.wave_inputs.sqrFreqValue = DEFAULT_FREQ;
		EEWaveData.wave_inputs.modeStateValue = SINE;
	} else {
	}

	if((EEWaveData.wave_inputs.sqrFreqValue > MAX_INPUT)) {
		EEWaveData.wave_inputs.sineAmpValue = DEFAULT_AMP;
		EEWaveData.wave_inputs.sqrCycleValue = DEFAULT_AMP;
		EEWaveData.wave_inputs.sineFreqValue = DEFAULT_FREQ;
		EEWaveData.wave_inputs.sqrFreqValue = DEFAULT_FREQ;
		EEWaveData.wave_inputs.modeStateValue = SINE;
	} else if((EEWaveData.wave_inputs.sineFreqValue < MIN_INPUT)) {
		EEWaveData.wave_inputs.sineAmpValue = DEFAULT_AMP;
		EEWaveData.wave_inputs.sqrCycleValue = DEFAULT_AMP;
		EEWaveData.wave_inputs.sineFreqValue = DEFAULT_FREQ;
		EEWaveData.wave_inputs.sqrFreqValue = DEFAULT_FREQ;
		EEWaveData.wave_inputs.modeStateValue = SINE;
	}
	switch(EEWaveData.wave_inputs.modeStateValue) {
	case(SINE):
			break;
	case(PULSE):
			break;
	default:
		EEWaveData.wave_inputs.sineAmpValue = DEFAULT_AMP;
		EEWaveData.wave_inputs.sqrCycleValue = DEFAULT_AMP;
		EEWaveData.wave_inputs.sineFreqValue = DEFAULT_FREQ;
		EEWaveData.wave_inputs.sqrFreqValue = DEFAULT_FREQ;
		EEWaveData.wave_inputs.modeStateValue = SINE;
		break;
	}

	//Write the settings to the EEPROM
	for(i = 0; i <= (sizeof(EEWaveData.eesettings) - 1); i++) {
		confirm_checksum[i] = EEWaveData.eesettings[i];
	}
	EEWaveData.wave_inputs.checksumValue = MemChkSum((INT8U *)&confirm_checksum[0],((INT8U *)&confirm_checksum[sizeof(EEWaveData.eesettings) - 1]));
	for(i = 0; i <= (sizeof(EEWaveData.eesettings)); i++) {
		Spi2Write((0x00 + 2 * i),EEWaveData.eesettings[i]);
	    OSTimeDly(6,OS_OPT_TIME_PERIODIC, &os_err); //Delay by 6ms
	}
	//Initialize the wave outputs with the correct settings
	SetPulseFreq(EEWaveData.wave_inputs.sqrFreqValue);
	SetPulseDuty(EEWaveData.wave_inputs.sqrCycleValue);
	SetSinFreq(EEWaveData.wave_inputs.sineFreqValue);
	SetSinAmp(EEWaveData.wave_inputs.sineAmpValue);

	switch(EEWaveData.wave_inputs.modeStateValue){
	case(SINE):
		LcdDispClrLine(LCD_ROW_1,LCD_LAYER_MODE);
		LcdDispString(LCD_ROW_1,LCD_COL_13,LCD_LAYER_MODE,"SINE");
		LcdDispClrLine(LCD_ROW_2,LCD_LAYER_TSI_VALUE);
		LcdDispDecWord(LCD_ROW_2,LCD_COL_15	,LCD_LAYER_TSI_VALUE,EEWaveData.wave_inputs.sineAmpValue, 2, LCD_DEC_MODE_LZ);
		LcdDispClear(LCD_LAYER_WAVE_DATA);
		LcdDispDecWord(LCD_ROW_2,LCD_COL_1,LCD_LAYER_WAVE_DATA,EEWaveData.wave_inputs.sineFreqValue,6,LCD_DEC_MODE_AL);
		LcdDispString(LCD_ROW_2,LCD_COL_7,LCD_LAYER_WAVE_DATA,"Hz");
		break;
	case(PULSE):
		LcdDispClrLine(LCD_ROW_1,LCD_LAYER_MODE);
		LcdDispString(LCD_ROW_1,LCD_COL_12,LCD_LAYER_MODE,"PULSE");
		LcdDispClrLine(LCD_ROW_2,LCD_LAYER_TSI_VALUE);
		if(EEWaveData.wave_inputs.sqrCycleValue == 20) {
			LcdDispDecWord(LCD_ROW_2,LCD_COL_13	,LCD_LAYER_TSI_VALUE,(5 * EEWaveData.wave_inputs.sqrCycleValue), 3, LCD_DEC_MODE_LZ);
		} else {
			LcdDispDecWord(LCD_ROW_2,LCD_COL_14	,LCD_LAYER_TSI_VALUE,(5 * EEWaveData.wave_inputs.sqrCycleValue), 2, LCD_DEC_MODE_LZ);
		}
		LcdDispString(LCD_ROW_2,LCD_COL_16,LCD_LAYER_TSI_VALUE,"%");
		LcdDispClear(LCD_LAYER_WAVE_DATA);
		LcdDispDecWord(LCD_ROW_2,LCD_COL_1,LCD_LAYER_WAVE_DATA,EEWaveData.wave_inputs.sqrFreqValue,6,LCD_DEC_MODE_AL);
		LcdDispString(LCD_ROW_2,LCD_COL_7,LCD_LAYER_WAVE_DATA,"Hz");
		break;
	default:
		break;
	}

    OSTaskCreate(&appUITaskTCB,                  // Create User Interface task
                "app UI Task ",
                appUITask,
                (void *) 0,
                APP_CFG_UI_TASK_PRIO,
                &appUITaskStk[0],
                (APP_CFG_UI_TASK_STK_SIZE / 10u),
                APP_CFG_UI_TASK_STK_SIZE,
                0,
                0,
                (void *) 0,
                (OS_OPT_TASK_NONE),
                &os_err);

    OSTaskCreate(&TSICounterTaskTCB,             // Create Touchpad Counter task
    				"TSICounterTask ",
    				TSICounterTask,
    				(void *) 0,
    				APP_CFG_TSI_CNTR_TASK_PRIO,
    				&TSICounterTaskStk[0],
    				(APP_CFG_TSI_CNTR_TASK_STK_SIZE / 10u),
    				APP_CFG_TSI_CNTR_TASK_STK_SIZE,
    				0,
    				0,
    				(void *) 0,
    				(OS_OPT_TASK_NONE),
    				&os_err);

    OSTaskDel((OS_TCB *)0, &os_err);
   	while(os_err != OS_ERR_NONE){	/* error trap */
   	}

}
/*****************************************************************************************
* appUITask()-Private
* User input is taken in on keypad, saved, and pushed out for wave generation
* D Key resets settings
* A and B swap modes
* C Key backspaces input
* # Key confirms user input
* 03/13/2022: Tyler, added state machine, and settings management
*****************************************************************************************/
static void appUITask(void *p_arg){
    INT8C kchar;
    INT8C user_input[INPUT_SIZE] = {'\0','\0','\0','\0','\0','\0','\0','\0'};
    INT8U input_index = 0;
    INT32U dec_user_input = 0;
    INT32U i;
    OS_ERR os_err;
    (void)p_arg;
    OSMutexCreate(&appUserInputKey, "User Input Mutex", &os_err);

    while(1) {
    	DB0_TURN_OFF();            // Enable debug bit 0 while waiting
    	kchar=KeyPend(0,&os_err);
    	DB0_TURN_ON();             // Disable debug bit 0 while ready/running
		if (kchar == DC1){         //A Key has been pressed
			OSMutexPend(&appUserInputKey, 0, OS_OPT_PEND_BLOCKING,
									   (CPU_TS *)0, &os_err);
			EEWaveData.wave_inputs.modeStateValue = SINE;
			OSMutexPost(&appUserInputKey, OS_OPT_POST_NONE, &os_err);
			LcdDispClear(LCD_LAYER_MODE);
			LcdDispString(LCD_ROW_1,LCD_COL_13,LCD_LAYER_MODE,"SINE");
		} else if (kchar == DC2){  //B Key has been pressed
			OSMutexPend(&appUserInputKey, 0, OS_OPT_PEND_BLOCKING,
									   (CPU_TS *)0, &os_err);
			EEWaveData.wave_inputs.modeStateValue = PULSE;
			OSMutexPost(&appUserInputKey, OS_OPT_POST_NONE, &os_err);
			LcdDispClear(LCD_LAYER_MODE);
			LcdDispString(LCD_ROW_1,LCD_COL_12,LCD_LAYER_MODE,"PULSE");
		} else if (kchar == DC3){
			if((input_index - 1 )>= 0) {
				user_input[input_index - 1] = '\0'; //backspace the input
				input_index--;
			} else {
			}
			LcdDispClear(LCD_LAYER_USER_INPUT);
			LcdDispString(LCD_ROW_1,LCD_COL_1,LCD_LAYER_USER_INPUT,user_input);
			if(input_index > 0 ) {
				LcdDispString(LCD_ROW_1,LCD_COL_7,LCD_LAYER_USER_INPUT,"Hz");
			} else {
			}
		} else if(kchar == '#'){
			dec_user_input = 0;
			for(i = 0; i < INPUT_SIZE; i++){
				if(user_input[i] != '\0'){ //Checks if the value is not a empty slot
					dec_user_input = (dec_user_input * 10) + (user_input[i] - 0x30); //Convert to decimal
					user_input[i] = '\0'; //Clear user input, saved as decimal
				} else {
				}
			}
			if(dec_user_input >= MAX_INPUT){
				dec_user_input = MAX_INPUT;
			} else if(dec_user_input <= MIN_INPUT){
				dec_user_input = MIN_INPUT;
			} else {
			}
			LcdDispClear(LCD_LAYER_USER_INPUT);
			input_index = 0;
		} else if(kchar == DC4) {
			OSMutexPend(&appUserInputKey, 0, OS_OPT_PEND_BLOCKING,
									   (CPU_TS *)0, &os_err);
			EEWaveData.wave_inputs.sineAmpValue = DEFAULT_AMP;
			EEWaveData.wave_inputs.sqrCycleValue = DEFAULT_AMP;
			EEWaveData.wave_inputs.sineFreqValue = DEFAULT_FREQ;
			EEWaveData.wave_inputs.sqrFreqValue = DEFAULT_FREQ;
			EEWaveData.wave_inputs.modeStateValue = SINE;
			OSMutexPost(&appUserInputKey, OS_OPT_POST_NONE, &os_err);
			dec_user_input = 0;
			LcdDispClear(LCD_LAYER_MODE);
			LcdDispString(LCD_ROW_1,LCD_COL_13,LCD_LAYER_MODE,"SINE");
		} else {
			if(input_index < INPUT_SIZE) {
				user_input[input_index] = kchar;
				input_index++;
			} else {
			}
			LcdDispString(LCD_ROW_1,LCD_COL_1,LCD_LAYER_USER_INPUT,user_input);
			LcdDispString(LCD_ROW_1,LCD_COL_7,LCD_LAYER_USER_INPUT,"Hz");
		}
		switch(EEWaveData.wave_inputs.modeStateValue){
		case SINE:
			OSMutexPend(&appUserInputKey, 0, OS_OPT_PEND_BLOCKING,
								   (CPU_TS *)0, &os_err);
			if(dec_user_input > 0){
				EEWaveData.wave_inputs.sineFreqValue = dec_user_input;
				dec_user_input = 0;
			} else{
			}
			for(i = 0; i <= (sizeof(EEWaveData.eesettings) - 1); i++) {
				confirm_checksum[i] = EEWaveData.eesettings[i];
			}
			EEWaveData.wave_inputs.checksumValue = MemChkSum((INT8U *)&confirm_checksum[0],((INT8U *)&confirm_checksum[sizeof(EEWaveData.eesettings) - 1]));
			for(i = 0; i <= (sizeof(EEWaveData.eesettings)); i++) {
				Spi2Write((0x00 + 2 * i),EEWaveData.eesettings[i]);
				OSTimeDly(6,OS_OPT_TIME_PERIODIC, &os_err); //Delay by 6ms
			}
			LcdDispClear(LCD_LAYER_WAVE_DATA);
			LcdDispDecWord(LCD_ROW_2,LCD_COL_1,LCD_LAYER_WAVE_DATA,EEWaveData.wave_inputs.sineFreqValue,6,LCD_DEC_MODE_AL);
			LcdDispString(LCD_ROW_2,LCD_COL_7,LCD_LAYER_WAVE_DATA,"Hz");
			LcdDispClrLine(LCD_ROW_2,LCD_LAYER_TSI_VALUE);
			LcdDispDecWord(LCD_ROW_2,LCD_COL_15	,LCD_LAYER_TSI_VALUE,EEWaveData.wave_inputs.sineAmpValue, 2, LCD_DEC_MODE_LZ);
			SetSinFreq(EEWaveData.wave_inputs.sineFreqValue);
			SetSinAmp(EEWaveData.wave_inputs.sineAmpValue);
			SetPulseDuty(EEWaveData.wave_inputs.sqrCycleValue);
			SetPulseFreq(EEWaveData.wave_inputs.sqrFreqValue);
			OSMutexPost(&appUserInputKey, OS_OPT_POST_NONE, &os_err);
			break;
			case PULSE:
				OSMutexPend(&appUserInputKey, 0, OS_OPT_PEND_BLOCKING,
								   (CPU_TS *)0, &os_err);
				if(dec_user_input > 0){
					EEWaveData.wave_inputs.sqrFreqValue = dec_user_input;
					dec_user_input = 0;
				} else{
				}
				for(i = 0; i <= (sizeof(EEWaveData.eesettings) - 1); i++) {
					confirm_checksum[i] = EEWaveData.eesettings[i];
				}
				EEWaveData.wave_inputs.checksumValue = MemChkSum((INT8U *)&confirm_checksum[0],((INT8U *)&confirm_checksum[sizeof(EEWaveData.eesettings) - 1]));
				for(i = 0; i <= (sizeof(EEWaveData.eesettings)); i++) {
					Spi2Write((0x00 + 2 * i),EEWaveData.eesettings[i]);
					OSTimeDly(6,OS_OPT_TIME_PERIODIC, &os_err); //Delay by 6ms
				}
				LcdDispClear(LCD_LAYER_WAVE_DATA);
				LcdDispDecWord(LCD_ROW_2,LCD_COL_1,LCD_LAYER_WAVE_DATA,EEWaveData.wave_inputs.sqrFreqValue,6,LCD_DEC_MODE_AL);
				LcdDispString(LCD_ROW_2,LCD_COL_7,LCD_LAYER_WAVE_DATA,"Hz");
				LcdDispClrLine(LCD_ROW_2,LCD_LAYER_TSI_VALUE);
				if(EEWaveData.wave_inputs.sqrCycleValue == 20) {
					LcdDispDecWord(LCD_ROW_2,LCD_COL_13	,LCD_LAYER_TSI_VALUE,(5 * EEWaveData.wave_inputs.sqrCycleValue), 3, LCD_DEC_MODE_LZ);
				} else {
					LcdDispDecWord(LCD_ROW_2,LCD_COL_14	,LCD_LAYER_TSI_VALUE,(5 * EEWaveData.wave_inputs.sqrCycleValue), 2, LCD_DEC_MODE_LZ);
				}
				LcdDispString(LCD_ROW_2,LCD_COL_16,LCD_LAYER_TSI_VALUE,"%");
				SetSinFreq(EEWaveData.wave_inputs.sineFreqValue);
				SetSinAmp(EEWaveData.wave_inputs.sineAmpValue);
				SetPulseDuty(EEWaveData.wave_inputs.sqrCycleValue);
				SetPulseFreq(EEWaveData.wave_inputs.sqrFreqValue);
				OSMutexPost(&appUserInputKey, OS_OPT_POST_NONE, &os_err);
				break;
			default:
				break;
		}
	}
}

/**************************************************************************************
 * TSICounterTask()-Private
 * Touchpad count increases by 1 if touchpad 1 is triggered, count decreases by 1 if
 * touchpad 2 is triggered. There are 21 levels(0-20) in which the count can increase or
 * decrease.
 *
 * Created by: Andy Nguyen 03/02/2022
 * 03/13/2022: Tyler, added state machine to differentiate between the different data incrementations
 **************************************************************************************/
static void TSICounterTask(void *p_arg){
    INT32U i;
    OUT_MODES_T tsi_state = SINE;
    OS_FLAGS cur_sense_flags;

    OS_ERR os_err;

    (void)p_arg;

    while(1) {
        DB1_TURN_OFF();                    // Disable debug bit 1 while waiting
        cur_sense_flags = TSIPend(100,&os_err);
        DB1_TURN_ON();                     // Enable debug bit 1 while ready/running
        if(os_err == OS_ERR_TIMEOUT){
            cur_sense_flags = 0;
        } else{
        }
        if((cur_sense_flags & (1<<BRD_PAD1_CH)) != 0){
    		OSMutexPend(&appUserInputKey, 0, OS_OPT_PEND_BLOCKING,
	            		               (CPU_TS *)0, &os_err);
	        tsi_state = EEWaveData.wave_inputs.modeStateValue;

	        switch(tsi_state){
	        case(SINE):
	        	if(EEWaveData.wave_inputs.sineAmpValue < 20){
	        		EEWaveData.wave_inputs.sineAmpValue++;
	        		SetSinAmp(EEWaveData.wave_inputs.sineAmpValue);
	        	    LcdDispClrLine(LCD_ROW_2,LCD_LAYER_TSI_VALUE);
	        		LcdDispDecWord(LCD_ROW_2,LCD_COL_15	,LCD_LAYER_TSI_VALUE,EEWaveData.wave_inputs.sineAmpValue, 2, LCD_DEC_MODE_LZ);
	        	} else {
	        	}
				for(i = 0; i <= (sizeof(EEWaveData.eesettings) - 1); i++) {
					confirm_checksum[i] = EEWaveData.eesettings[i];
				}
				EEWaveData.wave_inputs.checksumValue = MemChkSum((INT8U *)&confirm_checksum[0],((INT8U *)&confirm_checksum[sizeof(EEWaveData.eesettings) - 1]));

				for(i = 0; i <= (sizeof(EEWaveData.eesettings)); i++) {
					Spi2Write((0x00 + 2 * i),EEWaveData.eesettings[i]);
					OSTimeDly(6,OS_OPT_TIME_PERIODIC, &os_err); //Delay by 6ms
				}
	        	break;
	        case(PULSE):
				if(EEWaveData.wave_inputs.sqrCycleValue < 20){
					EEWaveData.wave_inputs.sqrCycleValue++;
					SetPulseDuty(EEWaveData.wave_inputs.sqrCycleValue);
				    LcdDispClrLine(LCD_ROW_2,LCD_LAYER_TSI_VALUE);
					if(EEWaveData.wave_inputs.sqrCycleValue == 20) {
						LcdDispDecWord(LCD_ROW_2,LCD_COL_13	,LCD_LAYER_TSI_VALUE,(5 * EEWaveData.wave_inputs.sqrCycleValue), 3, LCD_DEC_MODE_LZ);
					}else{
						LcdDispDecWord(LCD_ROW_2,LCD_COL_14	,LCD_LAYER_TSI_VALUE,(5 * EEWaveData.wave_inputs.sqrCycleValue), 2, LCD_DEC_MODE_LZ);
					}
					LcdDispString(LCD_ROW_2,LCD_COL_16,LCD_LAYER_TSI_VALUE,"%");
				}else{
				}

				for(i = 0; i <= (sizeof(EEWaveData.eesettings) - 1); i++) {
					confirm_checksum[i] = EEWaveData.eesettings[i];
				}
				EEWaveData.wave_inputs.checksumValue = MemChkSum((INT8U *)&confirm_checksum[0],((INT8U *)&confirm_checksum[sizeof(EEWaveData.eesettings) - 1]));
				for(i = 0; i <= (sizeof(EEWaveData.eesettings)); i++) {
					Spi2Write((0x00 + 2 * i),EEWaveData.eesettings[i]);
					OSTimeDly(6,OS_OPT_TIME_PERIODIC, &os_err); //Delay by 6ms
				}
	        	break;
	        default:
	        	break;
			}
			OSMutexPost(&appUserInputKey, OS_OPT_POST_NONE, &os_err);
		}else{}
        if((cur_sense_flags & (1<<BRD_PAD2_CH)) != 0){
	        OSMutexPend(&appUserInputKey, 0, OS_OPT_PEND_BLOCKING,
	            		               (CPU_TS *)0, &os_err);
	        tsi_state = EEWaveData.wave_inputs.modeStateValue;

	        switch(tsi_state){
	        case(SINE):
	        	if(EEWaveData.wave_inputs.sineAmpValue > 0){
	        		EEWaveData.wave_inputs.sineAmpValue--;
	        		SetSinAmp(EEWaveData.wave_inputs.sineAmpValue);
	        	    LcdDispClrLine(LCD_ROW_2,LCD_LAYER_TSI_VALUE);
	        		LcdDispDecWord(LCD_ROW_2,LCD_COL_15	,LCD_LAYER_TSI_VALUE,EEWaveData.wave_inputs.sineAmpValue, 2, LCD_DEC_MODE_LZ);
	        	} else {
	        	}
				for(i = 0; i <= (sizeof(EEWaveData.eesettings) - 1); i++){
					confirm_checksum[i] = EEWaveData.eesettings[i];
				}
				EEWaveData.wave_inputs.checksumValue = MemChkSum((INT8U *)&confirm_checksum[0],((INT8U *)&confirm_checksum[sizeof(EEWaveData.eesettings) - 1]));
				for(i = 0; i <= (sizeof(EEWaveData.eesettings)); i++){
					Spi2Write((0x00 + 2 * i),EEWaveData.eesettings[i]);
					OSTimeDly(6,OS_OPT_TIME_PERIODIC, &os_err); //Delay by 6ms
				}
	        	break;
	        case(PULSE):
				if(EEWaveData.wave_inputs.sqrCycleValue > 0){
					EEWaveData.wave_inputs.sqrCycleValue--;
					SetPulseDuty(EEWaveData.wave_inputs.sqrCycleValue);
				    LcdDispClrLine(LCD_ROW_2,LCD_LAYER_TSI_VALUE);
					if(EEWaveData.wave_inputs.sqrCycleValue == 20) {
						LcdDispDecWord(LCD_ROW_2,LCD_COL_13	,LCD_LAYER_TSI_VALUE,(5 * EEWaveData.wave_inputs.sqrCycleValue), 3, LCD_DEC_MODE_LZ);
					}else{
						LcdDispDecWord(LCD_ROW_2,LCD_COL_14	,LCD_LAYER_TSI_VALUE,(5 * EEWaveData.wave_inputs.sqrCycleValue), 2, LCD_DEC_MODE_LZ);
					}
					LcdDispString(LCD_ROW_2,LCD_COL_16,LCD_LAYER_TSI_VALUE,"%");
				}else{
				}
				for(i = 0; i <= (sizeof(EEWaveData.eesettings) - 1); i++){
					confirm_checksum[i] = EEWaveData.eesettings[i];
				}
				EEWaveData.wave_inputs.checksumValue = MemChkSum((INT8U *)&confirm_checksum[0],((INT8U *)&confirm_checksum[sizeof(EEWaveData.eesettings) - 1]));
				for(i = 0; i <= (sizeof(EEWaveData.eesettings)); i++){
					Spi2Write((0x00 + 2 * i),EEWaveData.eesettings[i]);
					OSTimeDly(6,OS_OPT_TIME_PERIODIC, &os_err); //Delay by 6ms
				}
	        	break;
	        default:
	        	break;
			}
			OSMutexPost(&appUserInputKey, OS_OPT_POST_NONE, &os_err);
        }
        else{}
    }
}

