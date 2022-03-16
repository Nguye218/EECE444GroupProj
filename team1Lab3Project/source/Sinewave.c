/****************************************************************************
 * Sinewave.c
 * Program uses fixed point math to generate a sinewave. DMA sends samples
 * to the DAC and outputs to DAC0.
 * Created by: Karen Aguilar,Rodrick Muya 03/06/2022
 ****************************************************************************/
/*****************************************************************************
 * Include header files
 ****************************************************************************/
#include "os.h"
#include "app_cfg.h"
#include "MCUType.h"
#include "K65TWR_GPIO.h"
#include "SineWave.h"
#include "arm_common_tables.h"
#include "arm_math.h"

typedef struct{
    INT8U index;
    OS_SEM flag;
}DMA_BLOCK_RDY;
typedef struct {
    INT16U  sinefreq;   //frequency of sinewave
    INT8U   sineamp;    //amplitude of sinewave
} WAVE_VALUE;
static WAVE_VALUE SineData;

#define SIZE_CODE_16BIT 001
#define SAMPLES_PER_BLOCK 128
#define WAVE_DMA_OUT_CH 0
#define NUM_BLOCKS                  2
#define WAVE_BYTES_PER_SAMPLE       2
#define BYTES_PER_BLOCK             (SAMPLES_PER_BLOCK * WAVE_BYTES_PER_SAMPLE)
#define BYTES_PER_BUFFER            (NUM_BLOCKS * BYTES_PER_BLOCK)

/*****************************************************************************************
*  Mutex Key
*****************************************************************************************/
static OS_MUTEX SineKey;
/*****************************************************************************************
* Allocate task control blocks
*****************************************************************************************/
static OS_TCB SineWaveGenTCB;
/*****************************************************************************************
* Allocate task stack space.
*****************************************************************************************/
static CPU_STK sinewaveProcTaskSTK[APP_CFG_PROC_TASK_STK_SIZE];
/*******************************************************************************************
* Functions Declarations
*******************************************************************************************/
static void sinewaveProcTask(void *p_arg);
static INT8U DMAPingPend(OS_TICK tout, OS_ERR *os_err_ptr); // may need to be static
static INT16U GetSinFreq(void);
static INT8U GetSinAmp(void);
/*******************************************************************************************
* Variable Declarations
*******************************************************************************************/
static DMA_BLOCK_RDY dmaInBlockRdy;
static INT16U DMABuffer[NUM_BLOCKS][SAMPLES_PER_BLOCK]; // full double-buffering (ping-pong)
/******************************************************************************
* SineWaveInit() - Initializes the WaveGen module including PIT DMA and DAC
* for sinewave.
* Created by: Karen Aguilar, Rodrick Muya 03/09/2022
******************************************************************************/
void SineWaveInit(void){
    OS_ERR os_err;

    dmaInBlockRdy.index = 1;
    OSMutexCreate(&SineKey, "Sine Key", &os_err);

    OSTaskCreate(&SineWaveGenTCB,                  /* Create Sine wave Processing Task              */
                "ProcessingTask",
                sinewaveProcTask,
                (void *) 0,
                APP_CFG_PROCESSING_TASK_PRIO,
                &sinewaveProcTaskSTK[0],
                APP_CFG_PROC_TASK_STK_SIZE/10,
                APP_CFG_PROC_TASK_STK_SIZE,
                0,
                0,
                (void *) 0,
                (OS_OPT_TASK_NONE),
                &os_err);

    SIM->SCGC6 |= SIM_SCGC6_PIT(1);     // Turn on PIT Clock
    SIM->SCGC2 |= SIM_SCGC2_DAC0(1);    // Turn on DAC Clock
    SIM->SCGC6 |= SIM_SCGC6_DMAMUX_MASK;
    SIM->SCGC7 |= SIM_SCGC7_DMA_MASK;   //Turn on DMA Clock

    // DAC initialization
    DAC0->C0 |= DAC_C0_DACEN(1);        //Enable DAC throwing a hard fault
    DAC0->C1 |= DAC_C1_DMAEN(1);        // DMA Request
    DAC0->C0 |= DAC_C0_DACRFS(1);       //sets Vreference to DACREF_1 =1.65V
    //PIT Initialization
    PIT->MCR = PIT_MCR_MDIS(0);         //Enable PIT clock
    PIT->CHANNEL[0].LDVAL = 1249 ;      //set to 48khz freq (Buss clock = 60MHz, 1/19200sps)
    PIT->CHANNEL[0].TCTRL = (PIT_TCTRL_TEN(1)); //Enable PIT
    //DMA Initialization
    DMAMUX->CHCFG[WAVE_DMA_OUT_CH] &= ~(DMAMUX_CHCFG_ENBL_MASK|DMAMUX_CHCFG_TRIG_MASK);

    DMA0->TCD[WAVE_DMA_OUT_CH].SADDR = DMA_SADDR_SADDR(&DMABuffer[0][0]);

    DMA0->TCD[WAVE_DMA_OUT_CH].ATTR = (DMA_ATTR_SMOD(0)|DMA_ATTR_SSIZE(SIZE_CODE_16BIT)|DMA_ATTR_DMOD(0)|DMA_ATTR_DSIZE(SIZE_CODE_16BIT)); //001 this may be different
    DMA0->TCD[WAVE_DMA_OUT_CH].SOFF = DMA_SOFF_SOFF(WAVE_BYTES_PER_SAMPLE);
    DMA0->TCD[WAVE_DMA_OUT_CH].NBYTES_MLNO = DMA_NBYTES_MLNO_NBYTES(WAVE_BYTES_PER_SAMPLE);
    DMA0->TCD[WAVE_DMA_OUT_CH].CITER_ELINKNO = DMA_CITER_ELINKNO_ELINK(0)|DMA_CITER_ELINKNO_CITER(NUM_BLOCKS * SAMPLES_PER_BLOCK); //how long each block is in memory buffer
    DMA0->TCD[WAVE_DMA_OUT_CH].BITER_ELINKNO = DMA_BITER_ELINKNO_ELINK(0)|DMA_BITER_ELINKNO_BITER(NUM_BLOCKS * SAMPLES_PER_BLOCK); //samples per block needs to be buffer size
    DMA0->TCD[WAVE_DMA_OUT_CH].SLAST = DMA_SLAST_SLAST(-(BYTES_PER_BUFFER));
    DMA0->TCD[WAVE_DMA_OUT_CH].DADDR = DMA_DADDR_DADDR(&DAC0->DAT[0].DATL);
    DMA0->TCD[WAVE_DMA_OUT_CH].DOFF = DMA_DOFF_DOFF(0);
    DMA0->TCD[WAVE_DMA_OUT_CH].DLAST_SGA = DMA_DLAST_SGA_DLASTSGA(0);

    DMA0->TCD[WAVE_DMA_OUT_CH].CSR = DMA_CSR_ESG(0) | DMA_CSR_MAJORELINK(0) | DMA_CSR_BWC(3) |DMA_CSR_INTHALF(1)|
                                     DMA_CSR_INTMAJOR(1) ; //this may be wrong
    DMAMUX->CHCFG[WAVE_DMA_OUT_CH] = DMAMUX_CHCFG_ENBL(1) | DMAMUX_CHCFG_TRIG(1) | DMAMUX_CHCFG_SOURCE(60); // Enables Source to DMA this may be wrong

    NVIC_EnableIRQ(DMA0_DMA16_IRQn); //Enables Interrupts
    //NVIC_EnableIRQ(0); //
    DMA0->SERQ = DMA_SERQ_SERQ(0);  // enables dma channel
}
/*******************************************************************************
 * sinewaveProcTask()- Public
 * Convert raw samples to q31 positive numbers and processing block.
 * Writes to a single Ping-Pong buffer register, and the DMA will
 * read from the other Ping-Pong buffer register.
 *
 * Created by: Karen Aguilar,Rodrick Muya 03/09/2022
 * *****************************************************************************/
void sinewaveProcTask(void *p_arg){

    INT8U buffer_index;
    INT32U i;
    INT16U dac_data;
    INT8U sin_amp;
    INT16U sin_freq;
    q31_t sin_step;
    q31_t sin_arg = 0;
    q31_t sin_value;
    q31_t sin_amp_value;      //intermediate sin term

	OS_ERR os_err;

	(void)p_arg;

	while(1){
		DB3_TURN_OFF();                             // Disable debug bit 3 while waiting
		buffer_index = DMAPingPend(0, &os_err);     //wait for flag from dma
		DB3_TURN_ON();                              // Enable debug bit 3 while ready/running
		sin_amp = GetSinAmp();;
		sin_freq = GetSinFreq();
		sin_step = (q31_t)(sin_freq * 44739);  //promote to q31_t and get to match sps
		for(i=0; i<SAMPLES_PER_BLOCK;i++){
			sin_arg = ((sin_arg + sin_step)& 0x7FFFFFFF); // force it to be positive
			sin_value = arm_sin_q31(sin_arg);
			sin_amp_value =  sin_amp * (sin_value/22); // 22 to prevent overflow
			dac_data = ((sin_amp_value>>20)+0x7FF);
			DMABuffer[buffer_index][i] = dac_data;
        }
	}
}
 /*****************************************************************************************
 * GetSinFreq()-Private
 * Gets the sine frequency from the mutex.
 * Created by: Karen Aguilar,Rodrick Muya 03/09/2022
 *****************************************************************************************/
static INT16U GetSinFreq(void){
	INT16U returnfreq;
	OS_ERR os_err;
	OSMutexPend(&SineKey, 0, OS_OPT_PEND_BLOCKING, (CPU_TS*)0, &os_err );
	returnfreq = SineData.sinefreq;
	OSMutexPost(&SineKey,OS_OPT_NONE,&os_err);
	return returnfreq;
}
/*****************************************************************************************
* SetSinFreq()- Public
* Sets the sine frequency in mutex.
* Created by: Karen Aguilar,Rodrick Muya 03/09/2022
*****************************************************************************************/
void SetSinFreq(INT16U changefreq){
	OS_ERR os_err;
	OSMutexPend(&SineKey, 0, OS_OPT_PEND_BLOCKING, (CPU_TS*)0, &os_err );
	SineData.sinefreq = changefreq;
	OSMutexPost(&SineKey,OS_OPT_NONE,&os_err);
}
/*****************************************************************************************
* GetsinAmp()-Public
* Gets the sine amplitude from mutex.
* Created by: Karen Aguilar,Rodrick Muya 03/09/2022
*****************************************************************************************/
static INT8U GetSinAmp(void){
	INT16U returnamp;
	OS_ERR os_err;
	OSMutexPend(&SineKey, 0, OS_OPT_PEND_BLOCKING, (CPU_TS*)0, &os_err );
	returnamp = SineData.sineamp;
	OSMutexPost(&SineKey,OS_OPT_NONE,&os_err);
	return returnamp;
}
/*****************************************************************************************
* SetSinAmp()-Public
* Sets the sine amplitude in mutex.
* Created by: Karen Aguilar,Rodrick Muya 03/09/2022
*****************************************************************************************/
void SetSinAmp(INT8U changeamp){
	OS_ERR os_err;
	OSMutexPend(&SineKey, 0, OS_OPT_PEND_BLOCKING, (CPU_TS*)0, &os_err );
	SineData.sineamp = changeamp;
	OSMutexPost(&SineKey,OS_OPT_NONE,&os_err);
}
/***************************************************************************************
 * DMA0_DMA16_IRQHandler()-Public
 * Parameters: none
 * Return: none
 * Interrupt service routine for DAC0 channel 0
 * Created by: Karen Aguilar,Rodrick Muya 03/09/2022
 ***************************************************************************************/
void DMA0_DMA16_IRQHandler(void){
	OS_ERR os_err;
	OSIntEnter();
	DB4_TURN_ON();                     // Enable debug bit 4
	DMA0->CINT = DMA_CINT_CINT(0);     // clears  flag
	if((DMA0->TCD[0].CSR & DMA_CSR_DONE_MASK) != 0){
		dmaInBlockRdy.index = 1;       //set buffer index to opposite of DMA
	}else{
		dmaInBlockRdy.index = 0;
	}
	OSSemPost(&(dmaInBlockRdy.flag),OS_OPT_POST_1,&os_err); // maybe void before
	DB4_TURN_OFF();                    // Disable debug bit 4
	OSIntExit();
}
/****************************************************************************************
 * DMAPingPend()- DMA Interrupt Handler for pending on the flag
 * Private.
 * Created by: Karen Aguilar,Rodrick Muya 03/09/2022
 ***************************************************************************************/
static INT8U DMAPingPend(OS_TICK tout, OS_ERR *os_err_ptr){
	OSSemPend(&(dmaInBlockRdy.flag), tout, OS_OPT_PEND_BLOCKING,(void *)0, os_err_ptr);
	return dmaInBlockRdy.index;
}
