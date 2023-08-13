/*
 * mathops.c
 *
 *  Created on: 1 Mar 2023
 *      Author: TheDa
 */

#include "mathops.h"
#include "stm32_utils.h"

void IntegrateTime(DiscreteTimeIntegratorHandle* integrator, int32_t interval, int32_t new_value){
	integrator->value = integrator->value + (new_value/((int32_t)1000/interval));
	integrator->count++;
}

void Integrate(DiscreteIntegratorHandle* integrator, int32_t new_value){
	integrator->value = integrator->value + new_value;
	integrator->count++;
}

void Derivative(DiscreteDerivativeHandle* derivitive, int32_t new_value){
	derivitive->value = ((new_value-derivitive->kminus1)*8 + (derivitive->kminus1-derivitive->kminus2)*5 + (derivitive->kminus2-derivitive->kminus3)*2)/15;
	derivitive->kminus3 = derivitive->kminus2;
	derivitive->kminus2 = derivitive->kminus1;
	derivitive->kminus1 = new_value;
	derivitive->count++;
}

void PID_SetGain(PID_Handle* hpid, int32_t p, int32_t i, int32_t d){
	hpid->gain_p = p;
	hpid->gain_i = i;
	hpid->gain_d = d;
}

void PID_Reset(PID_Handle* hpid){
	hpid->integral.value = 0;
	hpid->derivitive.value = 0;
}
void PID_UpdateSetpoint(PID_Handle* hpid, int32_t set_point){
	hpid->set_point = set_point;
}
int32_t PID_Compute(PID_Handle* hpid, int32_t current_point){
	int32_t error = hpid->set_point - current_point;
	Integrate(&hpid->integral, error);
	Derivative(&hpid->derivitive, error);
	return (hpid->gain_p * error + hpid->gain_i * hpid->integral.value + hpid->gain_d * hpid->derivitive.value)/1000;
}

void RUNTIME_AVERAGE_Update(RUNTIME_AVERAGE* hrave, int32_t value){
	hrave->n_samples++;
	hrave->average =  ((hrave->average*10000) - (hrave->average*10000)/(int32_t)hrave->n_samples + (value*10000)/(int32_t)hrave->n_samples)/10000;
}
void RUNTIME_AVERAGE_Reset(RUNTIME_AVERAGE* hrave){
	hrave->n_samples=1;
}

void MOVING_AVERAGE_Update(MOVING_AVERAGE* hmave, int32_t value){
	if(hmave->n_samples==0){
		return;
	}
	hmave->average = ((hmave->average*10000) - (hmave->average*10000)/(int32_t)hmave->n_samples + (value*10000)/(int32_t)hmave->n_samples)/10000;
}
void MOVING_AVERAGE_Reset(MOVING_AVERAGE* hmave, uint8_t samples){
	hmave->n_samples = samples;
	hmave->average = 0;
}

int32_t lerp(int32_t lower, int32_t upper, int32_t normalised_in, uint32_t scale){
	return lower+(((upper-lower)*normalised_in)/scale);
}
uint32_t normalise(int32_t lower, int32_t upper, int32_t value, uint32_t scale){
	return clamp(((value-lower)*scale)/(upper-lower),0,scale);
}

int32_t easeOutCubic(int32_t normalised_in, uint32_t scale){
	return scale - (((scale - normalised_in)*(scale - normalised_in)*(scale - normalised_in)))/(scale*scale);
}

uint32_t clamp(uint32_t value, uint32_t min, uint32_t max){
	if(value<=min){
		return min;
	}
	if(value>=max){
		return max;
	}
	return value;
}
