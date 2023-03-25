/*
 * state_or_charge.c
 *
 *  Created on: Mar 3, 2023
 *      Author: TheDa
 */


#include "state_of_charge.h"

extern uint32_t SOC_OV_LUT[SEGMENT_N][N_OV_SAMPLES];
DiscreteIntegratorHandle current_integrator;
uint32_t segment_max_charge[SEGMENT_N];
uint32_t segment_initial_charge_lost[SEGMENT_N];

void CurrentUpdate(int32_t current){
	Integrate(current_integrator, current);


}

void SetupInitialCharges(uint32_t segment_voltages[SEGMENT_N]){
	for(int seg = 0;seg<SEGMENT_N;seg++){
		for(int i = 0;i<N_OV_SAMPLES;i++){
			if(segment_voltages[seg]<SOC_OV_LUT[seg][i]){
				// ADD CHECK FOR SOC_OV_LUT OUT OF RANGE***
				// ADD CHECK FOR SOC_OV_LUT NOT FOUND***
				//(v-a)/(b-a) = normalised value between soc states.
				/*
				 * SOC ranges from 0 -> 100 in units of 0.01% but lut increments in 0.5% units. therefore, * by 100 we get from 0 -> 10000
				 * with units of 1 representing 0.01%. There are 50 divisions of 0.01% in 0.5% so multiplying our normalised ov by 50 will
				 * give us our SoC offset with the correct precision and units.
				 *
				 * Multiplying 50 by (i-1) gives us our soc position in loop.
				 */

				uint32_t normalised_ov = (50*(segment_voltages[seg]-SOC_OV_LUT[seg][i-1]))/(SOC_OV_LUT[seg][i]-SOC_OV_LUT[seg][i-1]);
				uint32_t initial_soc = 50*(i-1) + normalised_ov;
				segment_initial_charge_lost[seg] = (10000-initial_soc)*segment_max_charge[seg]/10000;
			}
		}
	}
}
