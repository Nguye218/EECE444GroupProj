

/* TSI low level module based on K65 Tower board low-power demo.
 * Todd Morton, 11/18/2014
 * Todd Morton, 11/19/2018 MCUXpresso version
 * Todd Morton, 11/17/2020 MCUX11.2 version
 */


#include "os.h"
#include "app_cfg.h"
#include "MCUType.h"
#include "K65TWR_GPIO.h"
#include "K65TWR_TSI.h"
#include "K65TWR_ClkCfg.h"
#include "LCDLayered.h"
#include "MK65F18.h"
#include "SysTickDelay.h"

typedef enum {PROC1START2, PROC2START1} TSI_TASK_STATE_T;
typedef struct{
    INT16U baseline;
    INT16U offset;
    INT16U threshold;
    INT16U state;
}TOUCH_LEVEL_T;


#define MAX_NUM_ELECTRODES 16U

#define E1_TOUCH_OFFSET  0x0400U    // Touch offset from baseline
#define E2_TOUCH_OFFSET  0x0400U    // Determined experimentally
#define TSI0_ENABLE()    TSI0->GENCS |= TSI_GENCS_TSIEN_MASK
#define TSI0_DISABLE()   TSI0->GENCS &= ~TSI_GENCS_TSIEN_MASK


static TOUCH_LEVEL_T tsiSensorLevels[MAX_NUM_ELECTRODES];
static void tsiStartScan(INT8U channel);
static void tsiProcScan(INT8U channel);
// void TSITask(void *p_arg);

static CPU_STK TSITaskStk[APP_CFG_TSI_TASK_STK_SIZE];
static OS_TCB TSITaskTCB;



static INT16U tsiSensorFlags = 0;

typedef enum {T_ON, T_OFF}T_STATE;

#define BRD_PAD1_CH  12U
#define BRD_PAD2_CH  11U

static OS_FLAG_GRP tsiFlags;


/********************************************************************************
 * K65TWR_TSI0Init: Initializes TSI0 module
 * Notes:
 *    -
 ********************************************************************************/
void TSIInit(void){

	OS_ERR os_err;


    SIM->SCGC5 |= SIM_SCGC5_TSI(1);         //Turn on clock to TSI module
    SIM->SCGC5 |= SIM_SCGC5_PORTB(1);

    PORTB->PCR[18]=PORT_PCR_MUX(0);         //Set electrode pins to ALT0
    PORTB->PCR[19]=PORT_PCR_MUX(0);
    tsiSensorLevels[BRD_PAD1_CH].offset = E1_TOUCH_OFFSET;
    tsiSensorLevels[BRD_PAD2_CH].offset = E2_TOUCH_OFFSET;


    //16 consecutive scans, Prescale divide by 32, software trigger
    //16uA ext. charge current, 16uA Ref. charge current, .592V dV
    TSI0->GENCS = ((TSI_GENCS_EXTCHRG(5))|
                   (TSI_GENCS_REFCHRG(5))|
                   (TSI_GENCS_DVOLT(1))|
                   (TSI_GENCS_PS(5))|
                   (TSI_GENCS_NSCN(15)));

    TSI0_ENABLE();
    TSIChCalibration(BRD_PAD1_CH);
    TSIChCalibration(BRD_PAD2_CH);

    OSFlagCreate(&tsiFlags,"TSI FLags",0,&os_err);

    OSTaskCreate(&TSITaskTCB,
                  "TSITask",
                  TSITask,
                  (void *) 0,
                  APP_CFG_TSI_START_PRIO,
                  &TSITaskStk[0],
                  (APP_CFG_TSI_TASK_STK_SIZE/10u),
                  APP_CFG_TSI_TASK_STK_SIZE,
                  0,
                  0,
                  (void *) 0,
                  (OS_OPT_TASK_NONE), /* Options */
                  &os_err);

}

/********************************************************************************
 *   TSICalibration: Calibration to find non-touch baseline for a channel
 *                   channel - the channel to calibrate, range 0-15
 *                   Note - the sensor must not be pressed when this is executed.
 ********************************************************************************/
void TSIChCalibration(INT8U channel){
        tsiStartScan(channel);
        while((TSI0->GENCS & TSI_GENCS_EOSF_MASK) == 0){} //wait for scan to finish
        TSI0->GENCS |= TSI_GENCS_EOSF(1);    //Clear flag
        tsiSensorLevels[channel].baseline = (INT16U)(TSI0->DATA & TSI_DATA_TSICNT_MASK);
        tsiSensorLevels[channel].threshold = tsiSensorLevels[channel].baseline +
                                             tsiSensorLevels[channel].offset;
}

/********************************************************************************
 *   TSITask: Cooperative task for timeslice scheduler
 *            Processes and starts alternate sensors each time through to avoid
 *            blocking.
 *            In order to not block the task period should be > 5ms.
 *            To not miss a press, the task period should be < ~25ms.
  ********************************************************************************/
void TSITask(void *p_arg){

	    OS_ERR os_err;
	    (void)p_arg;


	    while(1){
	        tsiStartScan(BRD_PAD1_CH);
	        OSTimeDly(6,OS_OPT_TIME_PERIODIC,&os_err);
	        tsiProcScan(BRD_PAD1_CH);
	        tsiStartScan(BRD_PAD2_CH);
	        OSTimeDly(6,OS_OPT_TIME_PERIODIC,&os_err);
	        tsiProcScan(BRD_PAD2_CH);
	    }
	}
/********************************************************************************
 *   TSIGetStartScan: Starts a scan of a TSI sensor.
 *                    channel - the TSI channel to be started. Range 0-15
 ********************************************************************************/
static void tsiStartScan(INT8U channel){
    TSI0->DATA = TSI_DATA_TSICH(channel);       //set channel
    TSI0->DATA |= TSI_DATA_SWTS(1);             //start a scan sequence
}

/********************************************************************************
 *   TSIProcScan: Waits for the scan to complete, then sets the appropriate
 *                flags is a touch was detected.
 *                Note the scan must be started before this is called.
 *                channel - the channel to be processed
 ********************************************************************************/
static void tsiProcScan(INT8U channel){

		OS_ERR os_err;
	    INT16U tsi_touch_count;
	    T_STATE cur_state;

	    while((TSI0->GENCS & TSI_GENCS_EOSF_MASK) == 0){}
	    TSI0->GENCS |= TSI_GENCS_EOSF(1);    //Clear flag

	    tsi_touch_count = (TSI0->DATA & TSI_DATA_TSICNT_MASK);
	    if((tsi_touch_count > tsiSensorLevels[channel].threshold)){
	        cur_state = T_ON;
	    }else{
	        cur_state = T_OFF;
	    }
	    if((cur_state == T_ON) && (tsiSensorLevels[channel].state == T_OFF)){
	        (void)OSFlagPost(&tsiFlags,
	                        (OS_FLAGS)(1<<channel),OS_OPT_POST_FLAG_SET, &os_err);
	        tsiSensorLevels[channel].state = T_ON;
	    }else if(cur_state == T_OFF){
	        tsiSensorLevels[channel].state = T_OFF;
	    }else{

	    }
	}

/********************************************************************************
 *   TSIGetSensorFlags: Returns value of sensor flag variable and clears it
 *                      to receive sensor press only one time.
 ********************************************************************************/
OS_FLAGS TSIPend(OS_TICK t_out,OS_ERR *os_err_ptr){
    OS_FLAGS sflags;

    sflags = OSFlagPend (&tsiFlags, 0xFFFF,t_out,
                         OS_OPT_PEND_FLAG_SET_ANY + OS_OPT_PEND_FLAG_CONSUME,
                         (void *)0,os_err_ptr);
    return sflags;
}


INT16U TSIGetSensorFlags(void){
    INT16U sflags;
    sflags = tsiSensorFlags;
    tsiSensorFlags = 0;
    return sflags;
}
