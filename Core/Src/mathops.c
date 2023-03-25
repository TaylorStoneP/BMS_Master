/*
 * mathops.c
 *
 *  Created on: 1 Mar 2023
 *      Author: TheDa
 */

#include "mathops.h"
#include "stm32_utils.h"

void Integrate(DiscreteIntegratorHandle integrator, uint32_t new_value){
	integrator.value = integrator.value + ((CurrentTime.time_ms - integrator.previous_time) * new_value)/1000;
	integrator.previous_time = CurrentTime.time_ms;
	integrator.count++;
}
