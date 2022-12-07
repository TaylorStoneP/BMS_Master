/*
 * thermistor.h
 *
 *  Created on: Dec 6, 2022
 *      Author: TheDa
 */

#ifndef SRC_THERMISTOR_H_
#define SRC_THERMISTOR_H_

#include "main.h"
#include "math.h"

//Calculates the resistance of the thermistor given an adc reading.
uint16_t Thermistor_R(uint16_t);

//Calculates the temperature of the thermistor given a resistance for the thermistor.
uint16_t Thermistor_T(uint16_t);

#endif /* SRC_THERMISTOR_H_ */
