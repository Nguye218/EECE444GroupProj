/**********************************************************
* EEPROM.h
* Header file for EEPROM.c
* Created on: Mar 12, 2022
* Author: Andy Nguyen
***********************************************************/
#ifndef EEPROM_H_
#define EEPROM_H_

INT16U Spi2fr16(INT32U pushr);
INT16U Spi2Read(INT8U addr);
void Spi2Write(INT8U addr, INT16U wr_data);
void SpiCmd(INT16U cmd);
void SPIInit(void);

#endif /* EEPROM_H_ */
