/*
 * ascii_hex.c
 *
 *  Created on: Dec 6, 2022
 *      Author: Taylor Phillips
 */

#include "ascii_hex.h"

uint8_t char_to_ascii_hex(char num)
{
    switch(num){
    case 0:
        return '0';
    case 1:
        return '1';
    case 2:
        return '2';
    case 3:
        return '3';
    case 4:
        return '4';
    case 5:
        return '5';
    case 6:
        return '6';
    case 7:
        return '7';
    case 8:
        return '8';
    case 9:
        return '9';
    case 10:
        return 'A';
    case 11:
        return 'B';
    case 12:
        return 'C';
    case 13:
        return 'D';
    case 14:
        return 'E';
    case 15:
        return 'F';
    default:
        return 'X';
    }
}

void number_to_ascii_hex(uint8_t* buffer, uint32_t num, int bytes)
{
    for(int i = bytes*2 - 1;i>=0;i--){
        buffer[bytes*2 - 1-i]=char_to_ascii_hex((num>>i*4)&0xF);
    }
}

