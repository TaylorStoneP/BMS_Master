/*
 * ascii_hex.h
 *
 *  Created on: Dec 6, 2022
 *      Author: Taylor Phillips
 */

#ifndef INC_ASCII_HEX_H_
#define INC_ASCII_HEX_H_
#include "main.h"
uint8_t char_to_ascii_hex(char num);
void number_to_ascii_hex(uint8_t* buffer, uint32_t num, int bytes);
#endif /* INC_ASCII_HEX_H_ */
