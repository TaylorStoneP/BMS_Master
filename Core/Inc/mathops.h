/*
 * mathops.h
 *
 *  Created on: 1 Mar 2023
 *      Author: TheDa
 */

#ifndef INC_MATHOPS_H_
#define INC_MATHOPS_H_

#include "main.h"

typedef struct DiscreteTimeIntegratorHandle{
	uint32_t count;	//unused.
	int32_t value;	//current integral of error.

}DiscreteTimeIntegratorHandle;

void IntegrateTime(DiscreteTimeIntegratorHandle* integrator, int32_t interval, int32_t new_value);

typedef struct DiscreteIntegratorHandle{
	uint32_t count;	//unused.
	int32_t value;	//current integral of error.

}DiscreteIntegratorHandle;

typedef struct DiscreteDerivativeHandle{
	uint32_t count;	//unused.
	int32_t value;	//current derivative of error.
	//previous error values.
	int32_t kminus1;
	int32_t kminus2;
	int32_t kminus3;
}DiscreteDerivativeHandle;

void Integrate(DiscreteIntegratorHandle* integrator, int32_t new_value);
void Derivative(DiscreteDerivativeHandle* derivitive, int32_t new_value);

typedef struct PID_Handle{
	int32_t gain_p;
	int32_t gain_i;
	int32_t gain_d;

	DiscreteIntegratorHandle integral;
	DiscreteDerivativeHandle derivitive;

	int32_t set_point;
}PID_Handle;

void PID_Reset(PID_Handle* hpid);
void PID_SetGain(PID_Handle* hpid, int32_t p, int32_t i, int32_t d);
void PID_UpdateSetpoint(PID_Handle* hpid, int32_t set_point);
int32_t PID_Compute(PID_Handle* hpid, int32_t current_point);

typedef struct RUNTIME_AVERAGE{
	uint32_t n_samples;
	int32_t average;
}RUNTIME_AVERAGE;

void RUNTIME_AVERAGE_Update(RUNTIME_AVERAGE* hrave, int32_t value);
void RUNTIME_AVERAGE_Reset(RUNTIME_AVERAGE* hrave);

typedef struct MOVING_AVERAGE{
	uint32_t n_samples;
	int32_t average;
}MOVING_AVERAGE;

void MOVING_AVERAGE_Update(MOVING_AVERAGE* hmave, int32_t value);
void MOVING_AVERAGE_Reset(MOVING_AVERAGE* hmave, uint8_t samples);

int32_t lerp(int32_t lower, int32_t upper, int32_t normalised_in, uint32_t scale);
uint32_t normalise(int32_t lower, int32_t upper, int32_t value, uint32_t scale);

int32_t easeOutCubic(int32_t value, uint32_t scale);

uint32_t clamp(uint32_t value, uint32_t min, uint32_t max);


#endif /* INC_MATHOPS_H_ */
