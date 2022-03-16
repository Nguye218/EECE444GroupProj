/*******************************************************************************
 * MemoryTools.h - Module containing memory based functions that manipulate data
 * stored in RAM
 * Contains a memory fill function to fill contents given between two addresses
 * with given data
 * Contains a checksum function to add byte contents between two addresses given
 * Todd Morton 10/06/2021
 *Created by: Tyler Roque, Oct 19, 2021
 ******************************************************************************/

#ifndef MEMORYTOOLS_H_
#define MEMORYTOOLS_H_

void MemFill(INT8U fillc, INT8U *startaddr, INT8U *endaddr);
INT16U MemChkSum(INT8U *startaddr, INT8U *endaddr);

#endif /* MEMORYTOOLS_H_ */
