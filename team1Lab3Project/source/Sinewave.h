/****************************************************
 * Sinewave.h
 * Header file for Sinewave.c
 * Created by: Karen Aguilar,Rodrick Muya 03/09/2022
 ****************************************************/
#ifndef SINEWAVE_H_
#define SINEWAVE_H_

void SineWaveInit(void);
void SetSinFreq(INT16U changefreq);
void SetSinAmp(INT8U changeamp);
void DMA0_DMA16_IRQHandler(void);

#endif /* SINEWAVE_H_ */
