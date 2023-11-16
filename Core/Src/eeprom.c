/*
 * eeprom.c
 *
 *  Created on: Aug 13, 2023
 *      Author: TheDa
 */


#include "main.h"
#include "eeprom.h"
#include "stm32_utils.h"

uint8_t eeprom_tx_buffer[1];
uint8_t eeprom_rx_buffer[1];

uint8_t eeprom_word_tx_buffer[4];
uint8_t eeprom_word_rx_buffer[4];

uint8_t eeprom_backup_state;

//Write byte to eeprom. No timing protection. Must allow 5ms between reading from memory.
void EEPROM_WriteByte(uint8_t page, uint8_t address, uint8_t data){
	  HAL_GPIO_WritePin(FLASH_WP_GPIO_Port, FLASH_WP_Pin, GPIO_PIN_RESET);

	if(page>7){return;}	//Outwith number of pages available.
	eeprom_tx_buffer[0] = data;
	HAL_I2C_Mem_Write(&hi2c2, 0xA0 | page, address, 1, eeprom_tx_buffer, 1, 1000);

	  HAL_GPIO_WritePin(FLASH_WP_GPIO_Port, FLASH_WP_Pin, GPIO_PIN_SET);

}

uint8_t EEPROM_ReadByte(uint8_t page, uint8_t address){
	if(page>7){return 0;}	//Outwith number of pages available.
	HAL_I2C_Mem_Read(&hi2c2, 0xA0 | page, address, 1, eeprom_rx_buffer, 1, 1000);
	return eeprom_rx_buffer[0];
}

void EEPROM_WriteUInt32(uint8_t page, uint8_t address, uint32_t data){
	  HAL_GPIO_WritePin(FLASH_WP_GPIO_Port, FLASH_WP_Pin, GPIO_PIN_RESET);

	if(page>7){return 0;}	//Outwith number of pages available.
	if(address>252){return 0;}	//Outwith number of bytes available with a length of 4.

	eeprom_word_tx_buffer[0] = BYTE0(data);
	eeprom_word_tx_buffer[1] = BYTE1(data);
	eeprom_word_tx_buffer[2] = BYTE2(data);
	eeprom_word_tx_buffer[3] = BYTE3(data);

	HAL_I2C_Mem_Write(&hi2c2, 0xA0 | page, address, 1, eeprom_word_tx_buffer, 4, 1000);

	HAL_Delay(5);

	  HAL_GPIO_WritePin(FLASH_WP_GPIO_Port, FLASH_WP_Pin, GPIO_PIN_SET);

}

uint32_t EEPROM_ReadUInt32(uint8_t page, uint8_t address){
	if(page>7){return 0;}	//Outwith number of pages available.
	if(address>252){return 0;}	//Outwith number of bytes available with a length of 4.

	HAL_I2C_Mem_Read(&hi2c2, 0xA0 | page, address, 1, eeprom_word_rx_buffer, 4, 1000);
	return ((uint32_t)eeprom_word_rx_buffer[0]) | ((uint32_t)eeprom_word_rx_buffer[1]<<8) | ((uint32_t)eeprom_word_rx_buffer[2]<<16) | ((uint32_t)eeprom_word_rx_buffer[3]<<24);
}

//Uses 9 bytes for redundancy.
void EEPROM_WriteUInt32_BackedUp(uint8_t page, uint8_t address, uint32_t data){
	if(page>7){return 0;}	//Outwith number of pages available.
	if(address>247){return 0;}	//Outwith number of bytes available with a length of 9.

	EEPROM_WriteByte(page, address+8, EEPROM_Main);
	HAL_Delay(5);
	EEPROM_WriteUInt32(page, address, data);
	HAL_Delay(5);
	EEPROM_WriteByte(page, address+8, EEPROM_Backup);
	HAL_Delay(5);
	EEPROM_WriteUInt32(page, address+4, data);
	HAL_Delay(5);
	EEPROM_WriteByte(page, address+8, EEPROM_None);

}
uint32_t EEPROM_ReadUInt32_BackedUp(uint8_t page, uint8_t address){
	eeprom_backup_state = EEPROM_ReadByte(page, address+8);
	uint32_t main = EEPROM_ReadUInt32(page, address);
	uint32_t backup = EEPROM_ReadUInt32(page, address+4);

	switch(eeprom_backup_state){
	case EEPROM_None:
		return main;
	case EEPROM_Main:
		return backup;
	case EEPROM_Backup:
		return main;
	}
}
