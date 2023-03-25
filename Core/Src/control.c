/*
 * control.c
 *
 *  Created on: Nov 1, 2022
 *      Author: Taylor Phillips
 */

#include "control.h"
#include "stm32_utils.h"
#include "main.h"

BMS_Data_Struct BMS_Data;

uint32_t DeviceConnections;

uint8_t N_SEGMENTS = SEGMENT_N, N_CELLS_PER_SEG=14, N_TEMP_ROWS=3, N_TEMP_COLS=7, N_AUX_TEMPS=1;

LTC_MUX_SEL LTC_MUX_LUT[8] = {	LTC_MUX_N0,
								LTC_MUX_N1,
								LTC_MUX_N2,
								LTC_MUX_N3,
								LTC_MUX_N4,
								LTC_MUX_N5,
								LTC_MUX_N6,
								LTC_MUX_N7};

int LTC_NTC_LUT_1[8] = {NTC2, NTC1, NTC4, -1, NTC7, NTC5, NTC6, NTC3};
int LTC_NTC_LUT_2[8] = {NTC9, NTC8, NTC10, -1, NTC14, NTC11, NTC12, NTC13};
int LTC_NTC_LUT_3[8] = {NTC17, NTC15, NTC16, -1, NTC21, NTC19, NTC20, NTC18};
uint16_t DCC_LUT[7] = {DCC_CELL1, DCC_CELL2, DCC_CELL3, DCC_CELL4, DCC_CELL5, DCC_CELL6, DCC_CELL7};

void LTC68041_ConfigGPIO(LTC_Index chips, LTC_GPIO pins)
{
	for(uint8_t chip_index = 0; chip_index < IC_N; chip_index++){
		if(BITSHIFTCHECK(chips, chip_index)){
			LTC68041_UpdateCFGRx(chip_index, 0, pins, CFGR0_ADCOPT | CFGR0_REFON | CFGR0_SWTRD);
		}
	}
}

void LTC68041_UpdateCFGRx(LTC_Index chip, uint8_t CFGRx, uint8_t byte, uint8_t ignore_mask){
	BMS_Data.chip_config[chip][CFGRx] = byte | (BMS_Data.chip_config[chip][CFGRx]&ignore_mask);
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
	LTC6804_wrcfg(ic_n, BMS_Data.chip_config);
}

void GrabMinMaxCellVoltage(){
	uint16_t minimum = 0xFFFF;
	uint16_t maximum = 0;
	uint16_t minimum_index = 0;
	uint16_t maximum_index = 0;
	for(int segment = 0; segment < N_SEGMENTS;segment++){
		for(int cell = 0;cell<N_CELLS_PER_SEG;cell++){
			if(BMS_Data.cell_voltage[segment][cell]<minimum){
				minimum = BMS_Data.cell_voltage[segment][cell];
				minimum_index = segment*N_CELLS_PER_SEG+cell;
			}
			if(BMS_Data.cell_voltage[segment][cell]>maximum){
				maximum = BMS_Data.cell_voltage[segment][cell];
				maximum_index = segment*N_CELLS_PER_SEG+cell;
			}
		}
	}
	BMS_Data.minimum_cell_id = minimum_index;
	BMS_Data.maximum_cell_id = maximum_index;
	BMS_Data.minimum_cell_voltage = minimum;
	BMS_Data.maximum_cell_voltage = maximum;
	return;
}

void GrabMinMaxSegmentTemperature(){
	uint16_t maximum = 0;
	uint16_t minimum = 0xFFFF;
	uint16_t seg_min = 0xFFFF;
	uint16_t seg_max = 0;
	for(int segment = 0; segment < SEGMENT_N;segment++){
		seg_min = 0xFFFF;
		seg_max = 0;
		for(int temp = 0;temp<22;temp++){
			if(BMS_Data.segment_temperatures[segment][temp]>maximum){
				maximum = BMS_Data.segment_temperatures[segment][temp];
			}
			if(BMS_Data.segment_temperatures[segment][temp]<minimum){
				minimum = BMS_Data.segment_temperatures[segment][temp];
			}
			if(BMS_Data.segment_temperatures[segment][temp]>seg_max){
				seg_max = BMS_Data.segment_temperatures[segment][temp];
			}
			if(BMS_Data.segment_temperatures[segment][temp]<seg_min){
				seg_min = BMS_Data.segment_temperatures[segment][temp];
			}
		}
		BMS_Data.segment_minimum_temperatures[segment] = seg_min;
		BMS_Data.segment_maximum_temperatures[segment] = seg_max;
	}
	BMS_Data.maximum_temperature = maximum;
	BMS_Data.minimum_temperature = minimum;
	return;
}

