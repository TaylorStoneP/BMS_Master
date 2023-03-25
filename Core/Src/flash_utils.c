/*
 * flash_utils.c
 *
 *  Created on: Dec 8, 2022
 *      Author: Taylor Phillips
 */
#include "main.h"
#include "flash_utils.h"

uint32_t Flash_Write_Data (uint32_t StartSectorAddress, uint32_t *data, uint16_t numberofwords)
{
	/* Unlock the Flash to enable the flash control register access *************/
	HAL_FLASH_Unlock();

	static FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t SECTORError;
	int ctr = 0;

	uint32_t StartSector		  = GetSector(StartSectorAddress);
	uint32_t EndSectorAddress	  = StartSectorAddress + numberofwords*4;
	uint32_t EndSector 			  = GetSector(EndSectorAddress);
	uint32_t NumberOfSectors 	  = (EndSector - StartSector) + 1;
	//Fill EraseInit structure
	EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
	EraseInitStruct.Sector        = StartSector;
	EraseInitStruct.NbSectors     = NumberOfSectors;

	if (HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError) != HAL_OK){
		return HAL_FLASH_GetError ();
	}

	/* Program the user Flash area word by word*/
	while (ctr < numberofwords){
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, StartSectorAddress, data[ctr]) == HAL_OK){
			StartSectorAddress += 4;
			ctr++;
		}
		else{
			return HAL_FLASH_GetError ();
		}
	}

   /* Lock the Flash to disable the flash control register access (recommended
	  to protect the FLASH memory against possible unwanted operation) *********/
   HAL_FLASH_Lock();

   return 0;
}

void Flash_Read_Data (uint32_t StartSectorAddress, uint32_t *RxBuf, uint16_t numberofwords)
{
	while (1)
	{

		*RxBuf = *(__IO uint32_t *)StartSectorAddress;
		StartSectorAddress += 4;
		RxBuf++;
		if (!(--numberofwords)) break;
	}
}

uint32_t GetSector(uint32_t address)
{
	if(address > MAINMEMEND)
		return -1;
	else if(address >= SECTOR7)
		return 7U;
	else if(address >= SECTOR6)
		return 6U;
	else if(address >= SECTOR5)
		return 5U;
	else if(address >= SECTOR4)
		return 4U;
	else if(address >= SECTOR3)
		return 3U;
	else if(address >= SECTOR2)
		return 2U;
	else if(address >= SECTOR1)
		return 1U;
	else if(address >= SECTOR0)
		return 0U;
	else
		return -1;
}
