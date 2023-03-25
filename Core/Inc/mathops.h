/*
 * mathops.h
 *
 *  Created on: 1 Mar 2023
 *      Author: TheDa
 */

#ifndef INC_MATHOPS_H_
#define INC_MATHOPS_H_

#include "main.h"

typedef struct DiscreteIntegratorHandle{
	uint32_t count;
	uint32_t value;
	uint32_t previous_time;
}DiscreteIntegratorHandle;

void Integrate(DiscreteIntegratorHandle integrator, uint32_t new_value);

#endif /* INC_MATHOPS_H_ */
