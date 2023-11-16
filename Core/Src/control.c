/*
 * control.c
 *
 *  Created on: Nov 1, 2022
 *      Author: Taylor Phillips
 */

#include "control.h"
#include "stm32_utils.h"
#include "main.h"
#include "fault_handler.h"
#include "config_loader.h"
#include "state_of_charge.h"

BMS_Data_Struct BMS_Data;
CHARGER_STATE hchgr;

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

	if(minimum<CFG_Main[CFGID_MinimumCellVoltage]){
		FaultRaise(FAULT_B2_LOW_CELL_VOLTAGE, FaultInfoBuff);
	}else{
		FaultLower(FAULT_B2_LOW_CELL_VOLTAGE);
	}

	if(maximum>CFG_Main[CFGID_MaximumCellVoltage]){
		FaultRaise(FAULT_B3_HIGH_CELL_VOLTAGE, FaultInfoBuff);
	}else{
		FaultLower(FAULT_B3_HIGH_CELL_VOLTAGE);
	}

	if(maximum>CFG_Main[CFGID_MaximumCellVoltageCharging]){
		FaultRaise(FAULT_B12_HIGH_VOLTAGE_CHARGING, FaultInfoBuff);
	}else{
		FaultLower(FAULT_B12_HIGH_VOLTAGE_CHARGING);
	}


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
		for(int temp = 0;temp<21;temp++){
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

void GrabMaxBalanceTemperature(){
	uint16_t maximum = 0;
	for(int segment = 0; segment < SEGMENT_N;segment++){
		for(int temp = 21;temp<22;temp++){
			if(BMS_Data.segment_temperatures[segment][temp]>maximum){
				maximum = BMS_Data.segment_temperatures[segment][temp];
			}
		}
	}
	BMS_Data.maximum_balance_temperature = maximum;
	return;
}

uint8_t Status_Check(uint8_t status){
	if(BMS_Data.status == status){
		return True;
	}else{
		return False;
	}
}
uint8_t Status_Set(uint8_t status){
	if(Status_Check(STATUS_SHUTDOWN)){
		return 1;
	}
	switch(status){
	case STATUS_STARTUP:
		if(BMS_Data.status!=STATUS_STARTUP){
			FaultRaise(FAULT_A3_INVALID_STATE_CHANGE, FaultInfoBuff);
			return 1;
		}
		break;
	case STATUS_CHARGE:
		if(BMS_Data.status!=STATUS_CHARGE_RDY){
			FaultRaise(FAULT_A3_INVALID_STATE_CHANGE, FaultInfoBuff);
			return 1;
		}
		break;
	case STATUS_DISCHARGE:
		if(BMS_Data.status!=STATUS_DISCHARGE_RDY){
			FaultRaise(FAULT_A3_INVALID_STATE_CHANGE, FaultInfoBuff);
			return 1;
		}
		break;
	}

	BMS_Data.status = status;
	return 0;
}


uint16_t I_SENSE_ADC_BUFFER[3] = {0};
uint8_t I_SENSE_RUNNING = False;
uint8_t I_SENSE_ADC_INDEX = 0;
uint8_t overcurrent_check = False;

void Start_CurrentADC(){
	if(I_SENSE_RUNNING){
		return;
	}
	I_SENSE_ADC_INDEX=0;
	I_SENSE_RUNNING = True;
	HAL_ADC_Start_DMA(&I_SENSE_ADC_TYPE,I_SENSE_ADC_BUFFER,3);
}

void Finish_CurrentADC(){
	float CH1_read_V = I_SENSE_ADC_BUFFER[0]*3.3/4095;
	float CH2_read_V = I_SENSE_ADC_BUFFER[1]*3.3/4095;
	float VREF_read_V = I_SENSE_ADC_BUFFER[2]*3.3/4095;

	float CH1 = ((CH1_read_V/VREF_read_V)*5.0f-2.5f)*1000/10*1.006f;
	float CH2 = ((CH2_read_V/VREF_read_V)*5.0f-2.5f)*1000/40;

	if(current_calibrated){
		if(CH2<0.0f){
			CH2*=1.006f;
		}else{
			CH2*=1.05f;
		}
	}

	int32_t current_ch1 = (int32_t)(CH1*100);
	int32_t current_ch2 = (int32_t)(CH2*100);

	if(!current_calibrated){
		MOVING_AVERAGE_Update(&current_offset_ch1, current_ch1);
		MOVING_AVERAGE_Update(&current_offset_ch2, current_ch2);
	}else{
		current_ch1 -= current_offset_ch1.average;
		current_ch2 -= current_offset_ch2.average;
	}

	MOVING_AVERAGE_Update(&current_ave_ch1, current_ch1);
	MOVING_AVERAGE_Update(&current_ave_ch2, current_ch2);

	BMS_Data.current_ch1 = current_ave_ch1.average;
	BMS_Data.current_ch2 = current_ave_ch2.average;

	if(CFG_Main[CFGID_CurrentSensorReversed]){
		BMS_Data.current_ch1 = -BMS_Data.current_ch1;
		BMS_Data.current_ch2 = -BMS_Data.current_ch2;
	}

	if((BMS_Data.current_ch2>(50*100)) || (BMS_Data.current_ch2<(-50*100))){
		BMS_Data.pack_current = BMS_Data.current_ch1;
	}else{
		BMS_Data.pack_current = BMS_Data.current_ch2;
	}

//	if(BITCHECK(DeviceConnections,DEVCON_Charger)){
//		if(BMS_Data.pack_current<(-((int32_t)CFG_Main[CFGID_MaximumChargeCurrent])*100)){
//			if(overcurrent_check==False){
//				ScheduleTask(SCH_CheckOvercurrent, 500, False, 0);
//				overcurrent_check=True;
//			}
//		}else{
//			FaultLower(FAULT_B13_HIGH_CURRENT_CHARGING);
//		}
//	}else{
//		if(BMS_Data.pack_current>(((int32_t)CFG_Main[CFGID_MaximumDischargeCurrent])*100)){
//			if(overcurrent_check==False){
//				ScheduleTask(SCH_CheckOvercurrent, 500, False, 0);
//				overcurrent_check=True;
//			}
//		}else{
//			FaultLower(FAULT_B14_HIGH_CURRENT_DISCHARGING);
//		}
//	}

	if(BITCHECK(DeviceConnections,DEVCON_Charger)){
		if(BMS_Data.pack_current<(-((int32_t)CFG_Main[CFGID_MaximumChargeCurrent])*100)){
			if(overcurrent_check==False){
				ScheduleTask(SCH_CheckOvercurrent, 250, False, 0);
				overcurrent_check=True;
			}
		}else{
			FaultLower(FAULT_B13_HIGH_CURRENT_CHARGING);
		}

		if(BMS_Data.pack_current>(((int32_t)CFG_Main[CFGID_MaximumChargeCurrent])*100)){
			if(overcurrent_check==False){
				ScheduleTask(SCH_CheckOvercurrent, 250, False, 0);
				overcurrent_check=True;
			}
		}else{
			FaultLower(FAULT_B15_REVERSE_CURRENT_CHARGING);
		}
	}else{

		if(BMS_Data.pack_current>(((int32_t)CFG_Main[CFGID_MaximumDischargeCurrent])*100)){
			if(overcurrent_check==False){
				ScheduleTask(SCH_CheckOvercurrent, 250, False, 0);
				overcurrent_check=True;
			}
		}else{
			FaultLower(FAULT_B14_HIGH_CURRENT_DISCHARGING);
		}
		if(BMS_Data.pack_current<(-((int32_t)CFG_Main[CFGID_MaximumDischargeCurrent])*100)){
			if(overcurrent_check==False){
				ScheduleTask(SCH_CheckOvercurrent, 250, False, 0);
				overcurrent_check=True;
			}
		}else{
			FaultLower(FAULT_B16_REVERSE_CURRENT_DISCHARGING);
		}
	}

	I_SENSE_RUNNING = False;

	if(BMS_Data.ts_active){
		SOC_Prediction();
	}
}

void BMS_Okay(uint8_t en){
	if(en){
		if(BITCHECK(DeviceConnections,DEVCON_Charger)){
			Status_Set(STATUS_CHARGE_RDY);
		}else{
			Status_Set(STATUS_DISCHARGE_RDY);
		}
	}
	HAL_GPIO_WritePin(AMS_OKAY_GPIO_Port, AMS_OKAY_Pin, en);

}

void SoftShutdown(){
	BMS_Okay(False);
	Status_Set(STATUS_FAULT);
}
void HardShutdown(){
	BMS_Okay(False);
	Status_Set(STATUS_SHUTDOWN);
}

MOVING_AVERAGE current_offset_ch1;
MOVING_AVERAGE current_offset_ch2;
MOVING_AVERAGE current_ave_ch1;
MOVING_AVERAGE current_ave_ch2;
uint8_t current_calibrated = False;
void CalibrateCurrentSensor(){
	current_calibrated = True;
}

