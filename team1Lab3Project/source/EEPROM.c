/**********************************************************************
* EEPROM.c
* Program handles SPI2 initializing, transfering, reading and writing data.
* Provided by: Todd Morton
* Modified by: Andy Nguyen 03/12/2022
***********************************************************************/
/**********************************************************************
* Include header files
**********************************************************************/
#include "os.h"
#include "app_cfg.h"
#include "MCUType.h"
#include "K65TWR_ClkCfg.h"
#include "LCDLayered.h"
#include "uCOSKey.h"
#include "K65TWR_TSI.h"
#include "MK65F18.h"
#include "SysTickDelay.h"
#include "Pulsetrain.h"
#include "K65TWR_GPIO.h"
#include "EEPROM.h"

#define EWEN 0x980 	//enable writing to EEPROM
#define EWDS 0x800	//disable writing to EEPROM
/****************************************************************************
 * SPIInit()- Initialize SPI2 on the K65-TWR board, fbus = 60MHz.
 * Parameters:none
 * Returns:none
 * Created by: Andy Nguyen, 03/12/2022
 ****************************************************************************/
void SPIInit(void){
     SIM->SCGC3 |= SIM_SCGC3_SPI2(1);     // Enable clock gate for SPI2
     SIM->SCGC5 |= SIM_SCGC5_PORTD(1);    // Enable clock gate for PORTD
     PORTD->PCR[11] = PORT_PCR_MUX(2);    //Connecting SPI2 to PORTD pins
     PORTD->PCR[12] = PORT_PCR_MUX(2);
     PORTD->PCR[13] = PORT_PCR_MUX(2);
     PORTD->PCR[14] = PORT_PCR_MUX(2);

     /* Two CTARs req'd for 93LC56B, one for write and one for read both
        set to 1.5Mbps and 16-bit frames, MSB first */

     // Write CTAR CPHA:CPOL= 00,
     SPI2->CTAR[0] = SPI_CTAR_BR(3)|SPI_CTAR_PBR(2)|SPI_CTAR_FMSZ(15)|
                     SPI_CTAR_CPHA(0)|SPI_CTAR_CPOL(0);

     // Read CTAR CPHA:CPOL= 10,
     SPI2->CTAR[1] = SPI_CTAR_BR(3)|SPI_CTAR_PBR(2)|SPI_CTAR_FMSZ(15)|
                     SPI_CTAR_CPHA(1)|SPI_CTAR_CPOL(0);

     // Set as controller with transfer FIFOs disabled
     SPI2->MCR = SPI_MCR_MSTR(1)|SPI_MCR_DIS_TXF(1)|SPI_MCR_DIS_RXF(1);
}
/***************************************************************************
 * Spifr16() - Performs a 16-bit transfer function.
 * Parameters: 32 bit unsigned integer
 *
 * Created by: Andy Nguyen, 03/12/2022
 ***************************************************************************/
 INT16U Spi2fr16(INT32U pushr){

	// Wait for previous transmit to complete and clear flag
	while((SPI2->SR & SPI_SR_TFFF_MASK) == 0){}
	SPI2->SR |= SPI_SR_TFFF_MASK;

	// Initiate a transfer using selected CTAR, PUSHR commands and data
	SPI2->PUSHR = pushr;

	// Wait for received data and clear flag
	while((SPI2->SR & SPI_SR_RFDF_MASK) == 0){}
	SPI2->SR |= SPI_SR_RFDF_MASK;

	// Return received data
	return SPI2->POPR;
}
/***************************************************************************
 * SpiCmd()-Public
 * Send a command to the EEPROM.
 * Parameters: 16-bit unsigned integer
 * Returns:none
 *
 * Created by: Andy Nguyen, 03/12/2022
 **************************************************************************/
void SpiCmd(INT16U cmd){
	//send command and dummy address, normal PCS0, CTAR0
	(void)Spi2fr16(SPI_PUSHR_PCS(1)|SPI_PUSHR_CONT(0)|SPI_PUSHR_CTAS(0)|
                  SPI_PUSHR_TXDATA(cmd));
}
/***************************************************************************
 * Spi2Write()-Public
 * Write and program the EEPROM.
 * Returns:none
 * Created by: Andy Nguyen, 03/12/2022
 **************************************************************************/
void Spi2Write(INT8U addr, INT16U wr_data){

	// Enables writing to the EEPROM
    SpiCmd(EWEN);

	//send command and address, continuous PCS0, use CTAR0
    (void)Spi2fr16(SPI_PUSHR_PCS(1)|SPI_PUSHR_CONT(1)|SPI_PUSHR_CTAS(0)|
                   SPI_PUSHR_TXDATA((0x5<<8)|(addr & 0x7F)));

    //send data, normal PCS0, CTAR0
    (void)Spi2fr16(SPI_PUSHR_PCS(1)|SPI_PUSHR_CONT(0)|SPI_PUSHR_CTAS(0)|
                   SPI_PUSHR_TXDATA(wr_data));
	// Disables writing to the EEPROM
    SpiCmd(EWDS);

}
/**************************************************************************
 * Spi2Read()-
 * Read a single 16-bit word from the EEPROM.
 *
 * Created by: Andy Nguyen 03/12/2022
 ***************************************************************************/
INT16U Spi2Read(INT8U addr){
    INT16U rd_value;
    //send command and address, continuous PCS0, use CTAR0
    (void)Spi2fr16(SPI_PUSHR_PCS(1)|SPI_PUSHR_CONT(1)|SPI_PUSHR_CTAS(0)|
                   SPI_PUSHR_TXDATA((0x6<<8)|addr));
    //send dummy data to shift in real data, normal PCS0, use CTAR1
    rd_value = Spi2fr16(SPI_PUSHR_PCS(1)|SPI_PUSHR_CONT(0)|SPI_PUSHR_CTAS(1)|
                        SPI_PUSHR_TXDATA(0x0000));
    return rd_value;
}
