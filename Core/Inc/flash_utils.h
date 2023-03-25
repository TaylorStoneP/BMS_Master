/*
 * flash_utils.h
 *
 *  Created on: Dec 8, 2022
 *      Author: Taylor Phillips
 */

#ifndef INC_FLASH_UTILS_H_
#define INC_FLASH_UTILS_H_

#define SECTOR0 0x08000000
#define SECTOR1 0x08004000
#define SECTOR2 0x08008000
#define SECTOR3 0x0800C000
#define SECTOR4 0x08010000
#define SECTOR5 0x08020000
#define SECTOR6 0x08040000
#define SECTOR7 0x08060000
#define MAINMEMEND 0x0807FFFF

uint32_t Flash_Write_Data(uint32_t StartSectorAddress, uint32_t * data, uint16_t numberofwords);
void Flash_Read_Data (uint32_t StartPageAddress, uint32_t *RxBuf, uint16_t numberofwords);
uint32_t GetSector(uint32_t);



#endif /* SRC_FLASH_UTILS_H_ */
