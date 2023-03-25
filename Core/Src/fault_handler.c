/*
 * fault_handler.c
 *
 *  Created on: Dec 9, 2022
 *      Author: TheDa
 */

#include "fault_handler.h"

FaultHandler hfault;
uint8_t FaultInfoBuff[4];

void FaultRaise(uint32_t FAULT_CODE, uint8_t info_in[4]){
	uint16_t fault = FAULT_CODE&0xFFFF;
	uint8_t index = BitToIndex(fault);

	switch(FAULT_CODE & 0xFFFF0000){
	case FAULT_A:
		BITSET(hfault.faultsA.faults,fault);
		for(int i = 0;i<4;i++){
			hfault.faultsA.info[index][i] = info_in[i];
		}
		break;
	case FAULT_B:
		BITSET(hfault.faultsB.faults,fault);
		for(int i = 0;i<4;i++){
			hfault.faultsB.info[index][i] = info_in[i];
		}
		break;
	case FAULT_C:
		BITSET(hfault.faultsC.faults,fault);
		for(int i = 0;i<4;i++){
			hfault.faultsC.info[index][i] = info_in[i];
		}
		break;
	case FAULT_D:
		BITSET(hfault.faultsD.faults,fault);
		for(int i = 0;i<4;i++){
			hfault.faultsD.info[index][i] = info_in[i];
		}
		break;
	}
}
void FaultLower(uint32_t FAULT_CODE){
	uint16_t fault = FAULT_CODE&0xFFFF;
	uint8_t index = BitToIndex(fault);

	switch(FAULT_CODE & 0xFFFF0000){
	case FAULT_A:
		BITRESET(hfault.faultsA.faults,fault);
		for(int i = 0;i<4;i++){
			hfault.faultsA.info[index][i] = 0;
		}
		break;
	case FAULT_B:
		BITRESET(hfault.faultsB.faults,fault);
		for(int i = 0;i<4;i++){
			hfault.faultsB.info[index][i] = 0;
		}
		break;
	case FAULT_C:
		BITRESET(hfault.faultsC.faults,fault);
		for(int i = 0;i<4;i++){
			hfault.faultsC.info[index][i] = 0;
		}
		break;
	case FAULT_D:
		BITRESET(hfault.faultsD.faults,fault);
		for(int i = 0;i<4;i++){
			hfault.faultsD.info[index][i] = 0;
		}
		break;
	}
}

uint8_t FaultCheck(uint32_t FAULT_CODE){
	uint16_t fault = FAULT_CODE&0xFFFF;
	uint8_t index = BitToIndex(fault);

	switch(FAULT_CODE & 0xFFFF0000){
	case FAULT_A:
		return BITCHECK(hfault.faultsA.faults,fault);
	case FAULT_B:
		return BITCHECK(hfault.faultsB.faults,fault);
	case FAULT_C:
		return BITCHECK(hfault.faultsC.faults,fault);
	case FAULT_D:
		return BITCHECK(hfault.faultsD.faults,fault);
	}
}

uint8_t* FaultData(uint32_t FAULT_CODE){
	uint16_t fault = FAULT_CODE&0xFFFF;
	uint8_t index = BitToIndex(fault);

	switch(FAULT_CODE & 0xFFFF0000){
	case FAULT_A:
		return hfault.faultsA.info;
	case FAULT_B:
		return hfault.faultsA.info;
	case FAULT_C:
		return hfault.faultsA.info;
	case FAULT_D:
		return hfault.faultsA.info;
	}
}
