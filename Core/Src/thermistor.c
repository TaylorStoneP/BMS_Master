/*
 * thermistor.c
 *
 *  Created on: Dec 6, 2022
 *      Author: TheDa
 */

#include "thermistor.h"

uint16_t Thermistor_R(uint16_t thermistor_output)
{
	// Calculation for thermistor resistance 10k/(Vo/Vin)-10k-MUXr.
	if(thermistor_output==0){
		return 0;
	}
	return 10000.0f/((float)thermistor_output/50000)-10030;
}

uint16_t Thermistor_R_Aux(uint16_t thermistor_output)
{
	// Calculation for thermistor resistance 10k/(Vo/Vin)-10k-MUXr.
	if(thermistor_output==0){
		return 0;
	}
	return 10000.0f/((50000.0f/(float)thermistor_output)-1);
}

uint16_t Thermistor_T(uint16_t thermistor_R)
{
	//Calculation for thermistor temperature using B value 3428, R0 10k, T0 298.15.
	return 1.0f/(1.0f/298.15f+1.0f/3428*logf((float)thermistor_R/10000.0f))*100;
}
