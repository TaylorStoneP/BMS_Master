/*
 * eeprom.h
 *
 *  Created on: Aug 13, 2023
 *      Author: TheDa
 *
 *     Designed specifically for the 24AA16 eeprom chip with
 *     limited functionality for the limited use case.
 */

#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

#define EEPROM_I2C_HANDLE hi2c2
extern I2C_HandleTypeDef EEPROM_I2C_HANDLE;

void EEPROM_WriteByte(uint8_t page, uint8_t address, uint8_t data);
uint8_t EEPROM_ReadByte(uint8_t page, uint8_t address);

void EEPROM_WriteUInt32(uint8_t page, uint8_t address, uint32_t data);
uint32_t EEPROM_ReadUInt32(uint8_t page, uint8_t address);

enum EEPROM_BACKUP{
	EEPROM_None,
	EEPROM_Main,
	EEPROM_Backup
};
void EEPROM_WriteUInt32_BackedUp(uint8_t page, uint8_t address, uint32_t data);
uint32_t EEPROM_ReadUInt32_BackedUp(uint8_t page, uint8_t address);



#endif /* INC_EEPROM_H_ */
