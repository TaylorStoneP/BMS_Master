/*
 * control.h
 *
 *  Created on: Nov 1, 2022
 *      Author: TheDa
 */

#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_
#include "main.h"
#include "stm32_utils.h"
#include "mathops.h"

//Number of ICs in system.
#define IC_N 16
#define SEGMENT_N ((1+IC_N)/2)
#define CELLS_PER_SEG (14)
#define CELLS_N (SEGMENT_N*CELLS_PER_SEG)

#define DELAY_TEMP_SCH 10

#define I_SENSE_TIMER_TYPE htim6
#define I_SENSE_ADC_TYPE hadc2
extern TIM_HandleTypeDef I_SENSE_TIMER_TYPE;
extern ADC_HandleTypeDef I_SENSE_ADC_TYPE;

#define SPI_EN 1
#if SPI_EN == 0
#define SPI_(x)
#else
#define SPI_(x) x
#endif

//===CONNECTIONSTATUSES===
extern uint32_t DeviceConnections;
enum DEVICE_CONNECTIONS{
	DEVCON_Seg1		= BIT1,
	DEVCON_Seg2		= BIT2,
	DEVCON_Seg3		= BIT3,
	DEVCON_Seg4		= BIT4,
	DEVCON_Seg5		= BIT5,
	DEVCON_Seg6		= BIT6,
	DEVCON_Seg7		= BIT7,
	DEVCON_Seg8		= BIT8,
	DEVCON_Client 	= BIT9,
	DEVCON_RCB 		= BIT10,
	DEVCON_Charger  = BIT11
};
//========================

enum CFGR0{
	CFGR0_GPIO1 = 0x08,
	CFGR0_GPIO2 = 0x10,
	CFGR0_GPIO3 = 0x20,
	CFGR0_GPIO4 = 0x40,
	CFGR0_GPIO5 = 0x80,
	CFRG0_GPIOALL = 0xF8,
	CFGR0_REFON = 0x04,
	CFGR0_SWTRD = 0x02,
	CFGR0_ADCOPT = 0x01
};
enum CFGR4{
	CFGR4_DCC1 = 0x01,
	CFGR4_DCC2 = 0x02,
	CFGR4_DCC3 = 0x04,
	CFGR4_DCC4 = 0x08,
	CFGR4_DCC5 = 0x10,
	CFGR4_DCC6 = 0x20,
	CFGR4_DCC7 = 0x40,
	CFGR4_DCC8 = 0x80
};
enum CFGR5{
	CFGR5_DCC9 = 0x01,
	CFGR5_DCC10 = 0x02,
	CFGR5_DCC11 = 0x04,
	CFGR5_DCC12 = 0x08,
	CFGR5_DCTO0 = 0x10,
	CFGR5_DCTO1 = 0x20,
	CFGR5_DCTO2 = 0x40,
	CFGR5_DCTO3 = 0x80,
	CFGR5_DCTOALL = CFGR5_DCTO0 | CFGR5_DCTO1 | CFGR5_DCTO2 | CFGR5_DCTO3
};
typedef enum LTC_DCTO{
	DCTOoff,
	DCTO0_5,
	DCTO1,
	DCTO2,
	DCTO3,
	DCTO4,
	DCTO5,
	DCTO10,
	DCTO15,
	DCTO20,
	DCTO30,
	DCTO40,
	DCTO60,
	DCTO75,
	DCTO90,
	DCTO120

}LTC_DCTO;
typedef enum LTC_CELLS{
	LTC_CELL1 = 0x01,
	LTC_CELL2 = 0x02,
	LTC_CELL3 = 0x04,
	LTC_CELL4 = 0x08,
	LTC_CELL5 = 0x10,
	LTC_CELL6 = 0x20,
	LTC_CELL7 = 0x40,
	LTC_CELL8 = 0x80,
	LTC_CELL9 = 0x100,
	LTC_CELL10 = 0x200,
	LTC_CELL11 = 0x400,
	LTC_CELL12 = 0x800
}LTC_CELLS;

typedef enum DCC_CELLS{
	DCC_CELL1 = LTC_CELL1,
	DCC_CELL2 = LTC_CELL2,
	DCC_CELL3 = LTC_CELL3,
	DCC_CELL4 = LTC_CELL4,
	DCC_CELL5 = LTC_CELL7,
	DCC_CELL6 = LTC_CELL8,
	DCC_CELL7 = LTC_CELL9,
}DCC_CELLS;

enum STATUS{
	STATUS_STARTUP,
	STATUS_DISCHARGE_RDY,
	STATUS_CHARGE_RDY,
	STATUS_DISCHARGE,
	STATUS_CHARGE,
	STATUS_FAULT,
	STATUS_SHUTDOWN
};

typedef struct BMS_Data_Struct
{
	uint8_t chip_config[IC_N][6];
	uint16_t cell_voltage[SEGMENT_N][14];
	uint8_t DCC[SEGMENT_N][14];
	uint16_t segment_temperatures[SEGMENT_N][22];
	uint16_t segment_minimum_temperatures[SEGMENT_N];
	uint16_t segment_maximum_temperatures[SEGMENT_N];
	uint16_t minimum_cell_voltage;
	uint16_t minimum_cell_id;
	uint16_t maximum_cell_voltage;
	uint16_t maximum_cell_id;
	uint16_t minimum_temperature;
	uint16_t maximum_balance_temperature;
	uint16_t maximum_temperature;

	uint8_t status;

	uint8_t current_mux;
	uint8_t spi_free;
	uint16_t auto_balance_wait_period;

	int32_t current_ch1;
	int32_t current_ch2;

	uint8_t fan_pwm;

	uint8_t SOC;
	uint32_t pack_voltage;
	int16_t pack_current;

	MOVING_AVERAGE average_pack_current;

	int16_t maximum_current_charge;
	int16_t maximum_current_discharge;

	uint8_t latches;

	uint8_t ts_active;

	uint8_t n_ic_connected;

}BMS_Data_Struct;
extern BMS_Data_Struct BMS_Data;

enum LATCHES{
	IND_AMS_LATCH = BIT1,
	IND_AMS_OKAY = BIT2,
	IND_IMD_LATCH = BIT3,
	IND_IMD_OKAY = BIT4
};

enum CHARGER_STATES{
	CHGR_HARDWARE_FAULT = BIT1,
	CHGR_TEMPERATURE_FAULT = BIT2,
	CHGR_INPUT_FAULT = BIT3,
	CHGR_BATTERY_FAULT = BIT4,
	CHGR_COMMS_FAULT = BIT5
};

typedef struct CHARGER_STATE{
	uint8_t state;
	uint32_t voltage;
	uint32_t current;
	int8_t temperature;
}CHARGER_STATE;
extern CHARGER_STATE hchgr;

typedef enum LTC_GPIO{
	LTC_GPIO1 = CFGR0_GPIO1,
	LTC_GPIO2 = CFGR0_GPIO2,
	LTC_GPIO3 = CFGR0_GPIO3,
	LTC_GPIO4 = CFGR0_GPIO4,
	LTC_GPIO5 = CFGR0_GPIO5
}LTC_GPIO;

typedef enum LTC_GPIO_ADC{
	LTC_ADC0 = LTC_GPIO1,
	LTC_ADC1 = LTC_GPIO3,
	LTC_ADC_ALL = LTC_ADC0 | LTC_ADC1
}LTC_GPIO_ADC;

typedef enum LTC_MUX_SEL{
	LTC_MUX_SEL0 = LTC_GPIO2,
	LTC_MUX_SEL1 = LTC_GPIO4,
	LTC_MUX_SEL2 = LTC_GPIO5,

	LTC_MUX_N0 = 0,
	LTC_MUX_N1 = LTC_MUX_SEL0,
	LTC_MUX_N2 = LTC_MUX_SEL1,
	LTC_MUX_N3 = LTC_MUX_SEL0 | LTC_MUX_SEL1,
	LTC_MUX_N4 = LTC_MUX_SEL2,
	LTC_MUX_N5 = LTC_MUX_SEL0 | LTC_MUX_SEL2,
	LTC_MUX_N6 = LTC_MUX_SEL1 | LTC_MUX_SEL2,
	LTC_MUX_N7 = LTC_MUX_SEL0 | LTC_MUX_SEL1 | LTC_MUX_SEL2
}LTC_MUX_SEL;

extern LTC_MUX_SEL LTC_MUX_LUT[8];

extern int LTC_NTC_LUT_1[8];
extern int LTC_NTC_LUT_2[8];
extern int LTC_NTC_LUT_3[8];
extern uint16_t DCC_LUT[7];

typedef enum LTC_Index{
	LTC_ALL = 0,
	LTC_1 = 0x1,
	LTC_2 = 0x2,
	LTC_3 = 0x4,
	LTC_4 = 0x8,
	LTC_5 = 0x10,
	LTC_6 = 0x20,
	LTC_7 = 0x40,
	LTC_8 = 0x80,
	LTC_9 = 0x100,
	LTC_10 = 0x200,
	LTC_11 = 0x400,
	LTC_12 = 0x800,
	LTC_13 = 0x1000,
	LTC_14 = 0x2000,
	LTC_15 = 0x4000,
	LTC_16 = 0x8000
}LTC_Index;

enum NTC_TO_TEMP{
	NTC1 = 0,
	NTC2 = 7,
	NTC3 = 14,
	NTC4 = 1,
	NTC5 = 8,
	NTC6 = 15,
	NTC7 = 2,
	NTC8 = 9,
	NTC9 = 16,
	NTC10 = 3,
	NTC11 = 10,
	NTC12 = 17,
	NTC13 = 4,
	NTC14 = 11,
	NTC15 = 18,
	NTC16 = 5,
	NTC17 = 12,
	NTC18 = 19,
	NTC19 = 6,
	NTC20 = 13,
	NTC21 = 20,
};

void LTC68041_UpdateCFGRx(LTC_Index chip, uint8_t CFGRx, uint8_t byte, uint8_t ignore_mask);
void LTC68041_ConfigGPIO(LTC_Index chips, LTC_GPIO pins);
void LTC68041_ConfigDCC(LTC_Index chips, LTC_CELLS cell);
void LTC68041_ConfigDCTO(LTC_Index chips, LTC_DCTO dcto);
void LTC68041_ConfigSend(uint8_t ic_n);

void GrabMinMaxCellVoltage();
void GrabMinMaxSegmentTemperature();
void GrabMaxBalanceTemperature();

uint8_t Status_Check(uint8_t status);
uint8_t Status_Set(uint8_t status);

//Current values stored in BMS_Data structure for convenient access.
extern uint8_t overcurrent_check;

void Start_CurrentADC();
void Finish_CurrentADC();

void BMS_Okay(uint8_t en);

void SoftShutdown();
void HardShutdown();

extern MOVING_AVERAGE current_offset_ch1;
extern MOVING_AVERAGE current_offset_ch2;
extern MOVING_AVERAGE current_ave_ch1;
extern MOVING_AVERAGE current_ave_ch2;
extern uint8_t current_calibrated;
void CalibrateCurrentSensor();
#endif /* INC_CONTROL_H_ */
