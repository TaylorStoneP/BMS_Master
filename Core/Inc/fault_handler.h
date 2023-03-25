/*
 * fault_handler.h
 *
 *  Created on: Dec 9, 2022
 *      Author: TheDa
 */

#ifndef INC_FAULT_HANDLER_H_
#define INC_FAULT_HANDLER_H_

#include "main.h"
#include "stm32_utils.h"

typedef struct FaultCode{
	uint16_t faults;
	uint8_t info[16][4];
}FaultCode;

enum FAULT_CODE_TYPE{
	FAULT_A = BIT17,
	FAULT_B = BIT18,
	FAULT_C = BIT19,
	FAULT_D = BIT20
};

typedef enum FAULT_CODESA{
	FAULT_A1_PEC_ERR 			= FAULT_A | BIT1,
	FAULT_A2_MUX_SEL_MISMATCH 	= FAULT_A | BIT2,
	FAULT_A3				    = FAULT_A | BIT3,
	FAULT_A4					= FAULT_A | BIT4,
	FAULT_A5 					= FAULT_A | BIT5,
	FAULT_A6 					= FAULT_A | BIT6,
	FAULT_A7 					= FAULT_A | BIT7,
	FAULT_A8 					= FAULT_A | BIT8,
	FAULT_A9 					= FAULT_A | BIT9,
	FAULT_A10 					= FAULT_A | BIT10,
	FAULT_A11 					= FAULT_A | BIT11,
	FAULT_A12 					= FAULT_A | BIT12,
	FAULT_A13 					= FAULT_A | BIT13,
	FAULT_A14 					= FAULT_A | BIT14,
	FAULT_A15 					= FAULT_A | BIT15,
	FAULT_A16 					= FAULT_A | BIT16
}FAULT_CODESA;

typedef enum FAULT_CODESB{
	FAULT_B1_OPEN_WIRING 		= FAULT_B | BIT1,
	FAULT_B2 					= FAULT_B | BIT2,
	FAULT_B3 					= FAULT_B | BIT3,
	FAULT_B4 					= FAULT_B | BIT4,
	FAULT_B5 					= FAULT_B | BIT5,
	FAULT_B6 					= FAULT_B | BIT6,
	FAULT_B7 					= FAULT_B | BIT7,
	FAULT_B8 					= FAULT_B | BIT8,
	FAULT_B9 					= FAULT_B | BIT9,
	FAULT_B10 					= FAULT_B | BIT10,
	FAULT_B11 					= FAULT_B | BIT11,
	FAULT_B12 					= FAULT_B | BIT12,
	FAULT_B13 					= FAULT_B | BIT13,
	FAULT_B14 					= FAULT_B | BIT14,
	FAULT_B15 					= FAULT_B | BIT15,
	FAULT_B16 					= FAULT_B | BIT16
}FAULT_CODESB;

typedef enum FAULT_CODESC{
	FAULT_C1 					= FAULT_C | BIT1,
	FAULT_C2 					= FAULT_C | BIT2,
	FAULT_C3 					= FAULT_C | BIT3,
	FAULT_C4 					= FAULT_C | BIT4,
	FAULT_C5 					= FAULT_C | BIT5,
	FAULT_C6 					= FAULT_C | BIT6,
	FAULT_C7 					= FAULT_C | BIT7,
	FAULT_C8 					= FAULT_C | BIT8,
	FAULT_C9 					= FAULT_C | BIT9,
	FAULT_C10 					= FAULT_C | BIT10,
	FAULT_C11 					= FAULT_C | BIT11,
	FAULT_C12 					= FAULT_C | BIT12,
	FAULT_C13 					= FAULT_C | BIT13,
	FAULT_C14 					= FAULT_C | BIT14,
	FAULT_C15 					= FAULT_C | BIT15,
	FAULT_C16 					= FAULT_C | BIT16
}FAULT_CODESC;

typedef enum FAULT_CODESD{
	FAULT_D1 					= FAULT_D | BIT1,
	FAULT_D2 					= FAULT_D | BIT2,
	FAULT_D3 					= FAULT_D | BIT3,
	FAULT_D4 					= FAULT_D | BIT4,
	FAULT_D5 					= FAULT_D | BIT5,
	FAULT_D6 					= FAULT_D | BIT6,
	FAULT_D7 					= FAULT_D | BIT7,
	FAULT_D8 					= FAULT_D | BIT8,
	FAULT_D9 					= FAULT_D | BIT9,
	FAULT_D10 					= FAULT_D | BIT10,
	FAULT_D11 					= FAULT_D | BIT11,
	FAULT_D12 					= FAULT_D | BIT12,
	FAULT_D13 					= FAULT_D | BIT13,
	FAULT_D14 					= FAULT_D | BIT14,
	FAULT_D15 					= FAULT_D | BIT15,
	FAULT_D16 					= FAULT_D | BIT16
}FAULT_CODESD;

typedef struct FaultHandler{
	FaultCode faultsA;
	FaultCode faultsB;
	FaultCode faultsC;
	FaultCode faultsD;
}FaultHandler;
extern FaultHandler hfault;
extern uint8_t FaultInfoBuff[4];

//Can only raise single fault.
void FaultRaise(uint32_t FAULT_CODE, uint8_t info_in[4]);
void FaultLower(uint32_t FAULT_CODE);

uint8_t FaultCheck(uint32_t FAULT_CODE);
uint8_t* FaultData(uint32_t FAULT_CODE);
#endif /* INC_FAULT_HANDLER_H_ */
