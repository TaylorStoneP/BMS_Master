/*
 * control.c
 *
 *  Created on: Nov 1, 2022
 *      Author: Taylor Phillips
 */

#include "control.h"
#include "stm32_utils.h"
#include "main.h"

struct LTC68041_Data LTCData;

uint8_t N_SEGMENTS = SEGMENT_N, N_CELLS_PER_SEG=14, N_TEMP_ROWS=3, N_TEMP_COLS=7, N_AUX_TEMPS=1;

LTC_MUX_SEL LTC_MUX_LUT[8] = {	LTC_MUX_N0,
								LTC_MUX_N1,
								LTC_MUX_N2,
								LTC_MUX_N3,
								LTC_MUX_N4,
								LTC_MUX_N5,
								LTC_MUX_N6,
								LTC_MUX_N7};

int LTC_NTC_LUT_1[8] = {1, 0, 3, -1, 6, 4, 5, 2};
int LTC_NTC_LUT_2[8] = {8, 7, 9, -1, 13, 10, 11, 12};
int LTC_NTC_LUT_3[8] = {16,14, 15, -1, 20, 18, 19, 17};

void LTC68041_ConfigGPIO(LTC_Index chips, LTC_GPIO pins)
{
	for(uint8_t chip_index = 0; chip_index < IC_N; chip_index++){
		if(BITSHIFTCHECK(chips, chip_index)){
			LTC68041_UpdateCFGRx(chip_index, 0, pins, CFGR0_ADCOPT | CFGR0_REFON | CFGR0_SWTRD);
		}
	}
}

void LTC68041_UpdateCFGRx(LTC_Index chip, uint8_t CFGRx, uint8_t byte, uint8_t ignore_mask){
	LTCData.chip_config[chip][CFGRx] = byte | (LTCData.chip_config[chip][CFGRx]&ignore_mask);
}
void LTC68041_ConfigDCC(LTC_Index chips, LTC_CELLS cells){

	for(uint8_t chip_index = 0; chip_index < IC_N; chip_index++){
		if(BITSHIFTCHECK(chips, chip_index)){
			LTC68041_UpdateCFGRx(chip_index, 4, cells&0xFF, 0);
			LTC68041_UpdateCFGRx(chip_index, 5, cells>>8, CFGR5_DCTOALL);
		}
	}
}

void LTC68041_ConfigDCTO(LTC_Index chips, LTC_DCTO dcto){
	for(uint8_t chip_index = 0; chip_index < IC_N; chip_index++){
		if(BITSHIFTCHECK(chips, chip_index)){
			LTC68041_UpdateCFGRx(chip_index, 5, dcto<<4, (~CFGR5_DCTOALL)&0xFF);
		}
	}
}
void LTC68041_ConfigSend(uint8_t ic_n)
{
	LTC6804_wrcfg(ic_n, LTCData.chip_config);
}
uint16_t MinimumCellVoltage(){
	uint16_t minimum = 0xFFFF;
	for(int segment = 0; segment < N_SEGMENTS;segment++){
		for(int cell = 0;cell<N_CELLS_PER_SEG;cell++){
			if(LTCData.cell_voltage[segment][cell]<minimum){
				minimum = LTCData.cell_voltage[segment][cell];
			}
		}
	}
	return minimum;
}
void GrabMinMaxSegmentTemperature(){
	uint16_t maximum = 0;
	uint16_t minimum = 0xFFFF;
	for(int segment = 0; segment < N_SEGMENTS;segment++){
		for(int temp = 0;temp<22;temp++){
			if(LTCData.segment_temperatures[segment][temp]>maximum){
				maximum = LTCData.segment_temperatures[segment][temp];
			}
			if(LTCData.segment_temperatures[segment][temp]<minimum){
				minimum = LTCData.segment_temperatures[segment][temp];
			}
		}
	}
	LTCData.maximum_temperature = maximum;
	LTCData.minimum_temperature = minimum;
	return;
}

