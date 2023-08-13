/*
 * state_or_charge.c
 *
 *  Created on: Mar 3, 2023
 *      Author: TheDa
 */


#include "state_of_charge.h"
#include "fault_handler.h"

uint32_t SOC_OV_curve[N_OV_SAMPLES] = {25000, 29755, 31796, 32849, 33836, 34758, 35614, 36338, 37194, 37948, 42000};
int32_t battery_initial_charge_lost;

DiscreteTimeIntegratorHandle current_integrator;
int32_t battery_max_charge = (2930*6)/10*3600;
DiscreteDerivativeHandle voltage_roc;

void SOC_Prediction(){
	IntegrateTime(&current_integrator, 50, BMS_Data.pack_current);
	FaultRaise(FAULT_A16_VOLTAGE_FLUCTUATION, FaultInfoBuff);
	if((battery_initial_charge_lost+current_integrator.value) > battery_max_charge){
		BMS_Data.SOC = 0;
		battery_initial_charge_lost = battery_max_charge;
		current_integrator.value = 0;
	}else if((battery_initial_charge_lost+current_integrator.value) < 0){
		BMS_Data.SOC = 100*2;
		battery_initial_charge_lost = 0;
		current_integrator.value = 0;
	}else{
		BMS_Data.SOC = (battery_max_charge - battery_initial_charge_lost - current_integrator.value)*200/battery_max_charge;
	}
}

void SOC_Correction(){
	FaultLower(FAULT_A16_VOLTAGE_FLUCTUATION);
	current_integrator.value = 0;
	uint8_t lower_index = 0;
	for(uint8_t index = 0;index<N_OV_SAMPLES-1;index++){
		if((BMS_Data.minimum_cell_voltage>SOC_OV_curve[index])&&(BMS_Data.minimum_cell_voltage<SOC_OV_curve[index+1])){
			lower_index = index;
			break;
		}
		if(index == (N_OV_SAMPLES-2)){
			BMS_Data.SOC = 100*2;
			battery_initial_charge_lost = 0;
			return;
		}
	}
	//SOC ranging between 0 - 10000.
	int32_t state_of_charge = lerp(lower_index*1000, (lower_index+1)*1000,normalise(SOC_OV_curve[lower_index], SOC_OV_curve[lower_index+1], BMS_Data.minimum_cell_voltage, 1000),1000);
	//SOC between 0 - 1
	float state_of_charge_float = (float)state_of_charge/10000.0f;
	//Initial charge of the battery in units of 10mAh
	uint32_t battery_initial_charge = state_of_charge_float*battery_max_charge;
	battery_initial_charge_lost = battery_max_charge - battery_initial_charge;

	BMS_Data.SOC = (battery_max_charge - battery_initial_charge_lost)*200/battery_max_charge;

}
