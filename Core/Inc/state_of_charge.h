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

#define N_OV_SAMPLES 11

extern uint32_t SOC_OV_curve[N_OV_SAMPLES];	//Battery basis SOC OCV look up table
extern int32_t battery_initial_charge_lost;
extern DiscreteTimeIntegratorHandle current_integrator;
extern DiscreteDerivativeHandle voltage_roc;
extern int32_t battery_max_charge;

void SOC_Prediction();

void SOC_Correction();

#endif /* INC_STATE_OF_CHARGE_H_ */
