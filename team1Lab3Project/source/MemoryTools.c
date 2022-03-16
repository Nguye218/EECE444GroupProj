/*******************************************************************************
* EECE344 Memory Tools
* Module containing memory based functions that manipulate data
* stored in RAM
* Contains a memory fill function to fill contents given between two addresses
* with given data
* Contains a checksum function to add byte contents between two addresses given
*
* Todd Morton, 10/06/2021
* Created by: Tyler Roque, 10/18/2021
* 10/19/2021: Added checksum function and memory fill function
*******************************************************************************/
#include "MCUType.h"               /* Include header files                    */
#include "MemoryTools.h"


void MemFill(INT8U fillc, INT8U *startaddr, INT8U *endaddr){
    INT8U *byteptr;

    byteptr = startaddr;
    while(byteptr < endaddr) {
        *byteptr = fillc;
        byteptr++;                 //Increment through address range
    }
    *byteptr = fillc;
}
/*******************************************************************************
* MemChkSum() - calculates correct checksum for any block in the 32 bit
* memory map.
*
* Created by: Tyler Roque, 10/18/2021
*********************************************************************************/
INT16U MemChkSum(INT8U *startaddr, INT8U *endaddr){
    INT8U *byteptr;
    INT16U checksum_total = 0;                        //16-bit checksum

    if(startaddr <= endaddr) {
        byteptr = startaddr;
        while(byteptr < endaddr){
            checksum_total = checksum_total + (INT16U)*byteptr;
            byteptr++;                               //increment through the address range
        }
        checksum_total = checksum_total + *byteptr;  //Terminal count inclusive
    }else{
    }
    return checksum_total;
}
