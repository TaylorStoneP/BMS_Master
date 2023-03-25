/*
 * state_of_charge.h
 *
 *  Created on: Mar 3, 2023
 *      Author: TheDa
 */

#ifndef INC_STATE_OF_CHARGE_H_
#define INC_STATE_OF_CHARGE_H_

#include "stm32_utils.h"
#include "control.h"
#include "mathops.h"

#define N_OV_SAMPLES 200

extern uint32_t SOC_OV_LUT[SEGMENT_N][N_OV_SAMPLES];
extern uint32_t segment_initial_charge_lost[SEGMENT_N];

void CurrentUpdate(int32_t current);

void SetupInitialCharges(uint32_t segment_voltages[SEGMENT_N]);

#endif /* INC_STATE_OF_CHARGE_H_ */
