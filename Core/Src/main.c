/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LTC68041.h"
#include "control.h"
#include "stm32_utils.h"
#include "can_utils.h"
#include "ascii_hex.h"
#include "thermistor.h"
#include "flash_utils.h"
#include "config_loader.h"
#include "warninfo.h"
#include "fault_handler.h"
#include "fan_control.h"
#include "state_of_charge.h"
//#include "mathops.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc2;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */

uint16_t gpio_read_buff[IC_N][6];
uint16_t cell_read_buff[IC_N][12];

uint16_t CAN_Cell_Index = 0;
uint16_t CAN_Temp_Index = 0;
uint16_t CAN_Stopclk_Index = 0;
uint16_t CAN_Temp_Segment_Index = 0;
uint8_t CAN_Fan_Index = 0;

uint8_t DCC_IC_Index = 0;

uint8_t temp_read_fault_flag = 0;
uint8_t cell_read_fault_flag = 0;

uint32_t internal_temperature = 0;

uint16_t fan_test_pwm;

uint32_t flash_data[2];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM4_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM6_Init(void);
static void MX_CAN2_Init(void);
static void MX_ADC2_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void TaskScheduleHandler(){
	SCHEDULE_HANDLE(SCH_FAN_TEST)
		if(fan_test_pwm>500){
			fan_test_pwm = 0;
		}

		//Fan_PWMSet(fan_test_pwm);

		fan_test_pwm+=100;

	}
	SCHEDULE_HANDLE(SCH_FAN_Update)
		uint8_t slow_tach_fault = False;
		for(int fan = 0;fan<FAN_N;fan++){
			//hfans[fan].speed = Fan_GetSpeed(&(hfans[fan]));
			//hfans[fan].speed = FAN_SpeedFromTime(hfans[fan].tach_time_end-hfans[fan].tach_time_start, TACH_COUNT);
			MOVING_AVERAGE_Update(&(hfans[fan].ave_speed),FAN_SpeedFromTime(hfans[fan].tach_time_end-hfans[fan].tach_time_start, TACH_COUNT));
			hfans[fan].speed = hfans[fan].ave_speed.average;
			if(hfans[fan].tach_finished==False){
				slow_tach_fault = True;
			}
			FAN_StartTach(&hfans[fan]);
		}
		if(slow_tach_fault){
			FaultRaise(FAULT_A4_SLOW_TACH, FaultInfoBuff);
			//Fan_PWMSet(0);
		}else{
			FaultLower(FAULT_A4_SLOW_TACH);

		}
		if(CFG_Main[CFGID_FanOverrideEnable]){
			FAN_SetDC(CFG_Main[CFGID_FanOverrideDC]);
		}else{
			FAN_SetDC(FAN_LerpTempToDC(BMS_Data.maximum_temperature));
		}
		fan_tach_n = 0;
		TACH_TIM->CNT = 0;
		HAL_TIM_Base_Start(&TACH_TIMER_TYPE);

		HAL_NVIC_EnableIRQ(EXTI4_IRQn);
		HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
	}
	SCHEDULE_HANDLE(SCH_LED_NUCLEO)
		//OUTPUT_TOGGLE(OUT_LED_NUCLEO);
	}

	SCHEDULE_HANDLE(SCH_LED_FAULT)
		if(temp_read_fault_flag || cell_read_fault_flag){
			OUTPUT_TOGGLE(OUT_LED_FAULT);
		}else{
			OUTPUT_RESET(OUT_LED_FAULT);
		}
	}

	SCHEDULE_HANDLE(SCH_LED_INDICATOR)
		OUTPUT_TOGGLE(OUT_LED_INDICATOR);
		//OUTPUT_TOGGLE(OUT_LED_NUCLEO);
	}

//	SCHEDULE_HANDLE(SCH_TEMP_INTERNAL_ADC)
//		HAL_ADC_Start_IT(&hadc1);
//	}

	SCHEDULE_HANDLE(SCH_LED_CAN)
		OUTPUT_TOGGLE(OUT_LED_CAN);
	}

	SCHEDULE_HANDLE(SCH_LED_WARN)
		OUTPUT_TOGGLE(OUT_LED_WARN);
	}
	SCHEDULE_HANDLE(SCH_BALANCE_Init)
		if(BMS_Data.spi_free){	//If SPI free, take control of SPI & start cell read operation.

			if(CFG_Main[CFGID_BalancingEnable]){
				if(Status_Check(STATUS_CHARGE_RDY) || Status_Check(STATUS_CHARGE)){
					for(int seg = 0;seg<SEGMENT_N;seg++){
						for(int cell = 0;cell<CELLS_PER_SEG;cell++){
							if(BMS_Data.maximum_balance_temperature>CFG_Main[CFGID_BalancingMaximumTemperature]){
								BMS_Data.DCC[seg][cell] = False;
							}else if(BMS_Data.cell_voltage[seg][cell]>CFG_Main[CFGID_MaximumCellVoltageBalancing]){
								BMS_Data.DCC[seg][cell] = False;
							}else if(BMS_Data.cell_voltage[seg][cell]<CFG_Main[CFGID_MinimumCellVoltageBalancing]){
								BMS_Data.DCC[seg][cell] = False;
							}else if(BMS_Data.maximum_cell_voltage<CFG_Main[CFGID_StartBalancingVoltage]){
								BMS_Data.DCC[seg][cell] = False;
							}else if((BMS_Data.cell_voltage[seg][cell]-BMS_Data.minimum_cell_voltage)>CFG_Main[CFGID_CellBalanceWindow]){
								BMS_Data.DCC[seg][cell] = True;
							}
							else{
								BMS_Data.DCC[seg][cell] = False;
							}
						}
					}
				}
			}

			uint16_t dcc_selection = 0;
			for(int ic = 0;ic<IC_N;ic++){
				dcc_selection = 0;
				for(int cell = 0;cell<7;cell++){
					if(BMS_Data.DCC[ic/2][cell+((ic%2)*7)]==True){
						dcc_selection |= DCC_LUT[cell];
					}
				}
				LTC68041_ConfigDCC(1<<ic , dcc_selection);
			}

			LTC68041_ConfigSend(IC_N);

			if(CFG_Main[CFGID_BalancingEnable]){
				if(CFG_Main[CFGID_BalancingAutoTimingEnable]){
					uint16_t threshold = CFG_Main[CFGID_CellBalanceWindow];
					uint16_t delta_v = BMS_Data.maximum_cell_voltage - BMS_Data.minimum_cell_voltage;
					uint16_t time = 0;

					if(delta_v > (threshold * 10)){
						time = 500;
					}else if(delta_v > (threshold * 5)){
						time = 800;
					}else if(delta_v > (threshold * 4)){
						time = 1500;
					}else if(delta_v > (threshold * 3)){
						time = 3000;
					}else if(delta_v > (threshold * 2)){
						time = 10000;
					}else if(delta_v > (threshold)){
						time = 30000;
					}
					BMS_Data.auto_balance_wait_period = time;
					ScheduleTask(SCH_BALANCE_Wait, 3000, False,0);
				}else{
					ScheduleTask(SCH_BALANCE_Wait, CFG_Main[CFGID_BalancingPeriod], False,0);
				}
			}

		}else{	//If SPI in-use, check again on next schedule cycle.
			ScheduleQueue[SCH_BALANCE_Init].flag = True;

		}

	}
	SCHEDULE_HANDLE(SCH_BALANCE_Wait)
		if(BMS_Data.spi_free){	//If SPI free, take control of SPI & start cell read operation.

			for(int ic = 0;ic<IC_N;ic++){
				for(int cell = 0;cell<7;cell++){
					BMS_Data.DCC[ic/2][cell+((ic%2)*7)]=False;
				}
				LTC68041_ConfigDCC(1<<ic , 0);
			}

			LTC68041_ConfigSend(IC_N);

			if(CFG_Main[CFGID_BalancingAutoTimingEnable]){
				ScheduleTask(SCH_BALANCE_Init, BMS_Data.auto_balance_wait_period, False,0);
			}else{
				ScheduleTask(SCH_BALANCE_Init, CFG_Main[CFGID_BalancingWaitPeriod], False,0);
			}



		}else{	//If SPI in-use, check again on next schedule cycle.
			ScheduleQueue[SCH_BALANCE_Wait].flag = True;

		}
	}
	SCHEDULE_HANDLE(SCH_CELL_Init)
	//Handle for Initialising a cell conv. and read operation.
		//Check if SPI is free.
		if(BMS_Data.spi_free==True){	//If SPI free, take control of SPI & start cell read operation.
			BMS_Data.spi_free = False;
			ScheduleTask(SCH_CELL_ADC, 5, False,0);
			StopclkEnd(STOPCLK_CELL_PERIOD);
			StopclkStart(STOPCLK_CELL_PERIOD);
			StopclkStart(STOPCLK_CELL_CYCLE);
		}else{	//If SPI in-use, check again on next schedule cycle.
			ScheduleQueue[SCH_CELL_Init].flag = True;
		}
	}
	SCHEDULE_HANDLE(SCH_CELL_ADC)
	//Handle for starting cell ADC conversion.
		wakeup_sleep();	//Wake-up device in-case asleep.
		LTC6804_adcv();	//Send ADC conversion command to all LTC6804-1 devices.
		ScheduleTask(SCH_CELL_Read, 40, False,0);	//Schedule ADC read operation after given delay to ensure conversion completed.
	}
	SCHEDULE_HANDLE(SCH_CELL_Read)
	//Handle for reading cell voltage ADC reading.
		if(LTC6804_rdcv(0,IC_N, cell_read_buff)!=0){	//Send read command to all LTC6804-1 devices.
			cell_read_fault_flag = 1;
			FaultRaise(FAULT_A1_PEC_ERR, FaultInfoBuff);
		}
		else{
			cell_read_fault_flag = 0;
			FaultLower(FAULT_A1_PEC_ERR);
		}

		/* Organise ADC readings into more intuitive arrangement.
		 * Array of size [N_SEGMENTS][N_CELLS_PER_SEG]
		 * Each IC is connected to 7 cells so offset of ic*7 is applied to LTCData.cell_voltage column.
		 * Each segment has 2 ICs so offset of segment*2 is applied to cell_read_buff row.
		 *
		 * Data in buffer is organised by rows of ICs and columns of cell index, however some cell indexes are not used.
		 * We use 7 of 12 cells available per IC, those indexes being [0,1,2,3,6,7,8], giving us our 7 cell readings per IC
		 * and 14 cell readings per segment when organised.
		 */
		BMS_Data.n_ic_connected = 0;
		BMS_Data.pack_voltage = 0;
		for(uint8_t segment = 0; segment < SEGMENT_N; segment++){	//Loop through all segments...
			for(uint8_t ic = 0; ic < 2; ic++){	//Loop through each IC on the segment.
				BMS_Data.cell_voltage[segment][ic*7] = 	cell_read_buff[segment*2+ic][0];
				if(BMS_Data.cell_voltage[segment][ic*7]!=65535){
					BMS_Data.n_ic_connected+=1;
				}
				BMS_Data.cell_voltage[segment][ic*7+1] = cell_read_buff[segment*2+ic][1];
				BMS_Data.cell_voltage[segment][ic*7+2] = cell_read_buff[segment*2+ic][2];
				BMS_Data.cell_voltage[segment][ic*7+3] = cell_read_buff[segment*2+ic][3];
				BMS_Data.cell_voltage[segment][ic*7+4] = cell_read_buff[segment*2+ic][6];
				BMS_Data.cell_voltage[segment][ic*7+5] = cell_read_buff[segment*2+ic][7];
				BMS_Data.cell_voltage[segment][ic*7+6] = cell_read_buff[segment*2+ic][8];
				if(IC_N==1){break;}	//If only 1 IC, exit loop. This should only happen for testing individual IC setup.
			}

			/* Calculates the pack voltage
			 * Already looping so segments so must only loop for cells.
			 */
			for(uint8_t cell = 0; cell < 14; cell++){
				BMS_Data.pack_voltage += BMS_Data.cell_voltage[segment][cell];
			}
		}
		if(BMS_Data.n_ic_connected!=IC_N){
			FaultRaise(FAULT_C6_MISSING_SEGMENT_COMMS, FaultInfoBuff);
		}else{
			FaultLower(FAULT_C6_MISSING_SEGMENT_COMMS);
		}
		GrabMinMaxCellVoltage();

		if(BMS_Data.pack_voltage<CFG_Main[CFGID_MinimumPackVoltage]){
			FaultRaise(FAULT_B9_LOW_PACK_VOLTAGE, FaultInfoBuff);
		}else{
			FaultLower(FAULT_B9_LOW_PACK_VOLTAGE);
		}


		CAN_UTIL_SetByte(CAN_TX_Voltage_Bounds,0,BYTE1(BMS_Data.maximum_cell_id) | (BMS_Data.DCC[BMS_Data.maximum_cell_id/CELLS_PER_SEG][BMS_Data.maximum_cell_id%CELLS_PER_SEG]<<7));
		CAN_UTIL_SetByte(CAN_TX_Voltage_Bounds,1,BYTE0(BMS_Data.maximum_cell_id));
		CAN_UTIL_SetByte(CAN_TX_Voltage_Bounds,2,BYTE1(BMS_Data.maximum_cell_voltage));
		CAN_UTIL_SetByte(CAN_TX_Voltage_Bounds,3,BYTE0(BMS_Data.maximum_cell_voltage));
		CAN_UTIL_SetByte(CAN_TX_Voltage_Bounds,4,BYTE1(BMS_Data.minimum_cell_id) | (BMS_Data.DCC[BMS_Data.minimum_cell_id/CELLS_PER_SEG][BMS_Data.minimum_cell_id%CELLS_PER_SEG]<<7));
		CAN_UTIL_SetByte(CAN_TX_Voltage_Bounds,5,BYTE0(BMS_Data.minimum_cell_id));
		CAN_UTIL_SetByte(CAN_TX_Voltage_Bounds,6,BYTE1(BMS_Data.minimum_cell_voltage));
		CAN_UTIL_SetByte(CAN_TX_Voltage_Bounds,7,BYTE0(BMS_Data.minimum_cell_voltage));

		CAN_UTIL_Transmit(&hcan1, CAN_TX_Voltage_Bounds);
		//CAN_UTIL_TransmitUSB(CAN_TX_Voltage_Bounds);

		StopclkEnd(STOPCLK_CELL_CYCLE);
		BMS_Data.spi_free = True;	//Free up SPI.
	}
	SCHEDULE_HANDLE(SCH_TEMP_Init)
	//Handle for Initialising temperature config, conv. and reading operation.
		//Check if SPI is free.
		if(BMS_Data.spi_free==True){	//If SPI free, take control of SPI & start temperature config operation.
			BMS_Data.spi_free = False;
			ScheduleTask(SCH_TEMP_Config, 0, False,0);

			StopclkEnd(STOPCLK_TEMP_PERIOD);
			StopclkStart(STOPCLK_TEMP_PERIOD);
			StopclkStart(STOPCLK_TEMP_CYCLE);
		}else{
			ScheduleQueue[SCH_TEMP_Init].flag = True; //If SPI in-use, check again on next schedule cycle.
		}
	}
	SCHEDULE_HANDLE(SCH_TEMP_Config)
	//Handle for sending temperature config to ICs
		/*
		 * Configure all LTC6804-1 settings settings to select MUX & enable ADCs.
		 * Then send config to all LTC6804-1 ICs.
		 * Schedule temperature ADC conversion after given time to let GPIO change.
		 */
		for(int ic = 0; ic<IC_N;ic++){
			LTC68041_ConfigGPIO(1<<ic, LTC_MUX_LUT[BMS_Data.current_mux] | LTC_ADC_ALL);
		}
		wakeup_sleep();
		LTC68041_ConfigSend(IC_N);
		ScheduleTask(SCH_TEMP_ADC, DELAY_TEMP_SCH*IC_N/4, False,0);
	}
	SCHEDULE_HANDLE(SCH_TEMP_ADC)
	//Handle for starting temperature ADC conv.
		/*
		 * Set ADC settings to all cell channels & all auxiliary channels.
		 * Send command to begin conversion on GPIO pins.
		 * Schedule temperature read operation after given time to allow conv. to happen.
		 */
		set_adc(MD_NORMAL,DCP_DISABLED,CELL_CH_ALL,AUX_CH_ALL);
		wakeup_sleep();
		LTC6804_adax();
		ScheduleTask(SCH_TEMP_Read, DELAY_TEMP_SCH, False,0);	//<6ms causes double register readings before conversion finishes.
	}

	SCHEDULE_HANDLE(SCH_TEMP_Read)
		wakeup_sleep();
		if(LTC6804_rdaux(0,IC_N, gpio_read_buff)!=0){	//Send read command to all LTC6804-1 devices.
			temp_read_fault_flag = 1;
			FaultRaise(FAULT_A1_PEC_ERR, FaultInfoBuff);
		}
		else{
			temp_read_fault_flag = 0;
			FaultLower(FAULT_A1_PEC_ERR);
		}
		if(BMS_Data.current_mux!=3){		//MUX 3 is +V reference so not part of temperatures.
			for(int seg = 0; seg < SEGMENT_N; seg++){
				BMS_Data.segment_temperatures[seg][LTC_NTC_LUT_2[BMS_Data.current_mux]] 	= clamp(Thermistor_T(Thermistor_R(gpio_read_buff[seg*2][0])),17315,77315);
				BMS_Data.segment_temperatures[seg][LTC_NTC_LUT_1[BMS_Data.current_mux]] 	= clamp(Thermistor_T(Thermistor_R(gpio_read_buff[seg*2][2])),17315,77315);;
				if(IC_N==1){break;}
				BMS_Data.segment_temperatures[seg][LTC_NTC_LUT_3[BMS_Data.current_mux]] 	= clamp(Thermistor_T(Thermistor_R(gpio_read_buff[seg*2+1][0])),17315,77315);;
				BMS_Data.segment_temperatures[seg][21] 									= clamp(Thermistor_T(Thermistor_R_Aux(gpio_read_buff[seg*2+1][2])),17315,77315);;
			}
		}

		/*
		 * Verifies if current GPIO selection is triggered correctly by checking the voltage on the MUX Select pins.
		 */
		uint8_t mux_mismatch = False;
		for(int ic = 0; ic < IC_N; ic++){
			if(((gpio_read_buff[ic][1]/30000) | ((gpio_read_buff[ic][3]/30000)<<1) | ((gpio_read_buff[ic][4]/30000)<<2))!=BMS_Data.current_mux){
				mux_mismatch = True;
				break;
			}
		}
		if(mux_mismatch){
			FaultRaise(FAULT_A2_MUX_SEL_MISMATCH, FaultInfoBuff);
		}else{
			FaultLower(FAULT_A2_MUX_SEL_MISMATCH);
		}

		//CHECK TEMP AGAINST MAX

		INCLOOP(BMS_Data.current_mux,8);

		if(BMS_Data.current_mux==0){
			BMS_Data.spi_free = True;
			GrabMinMaxSegmentTemperature();
			GrabMaxBalanceTemperature();
			if(BITCHECK(DeviceConnections,DEVCON_Charger)){
				if(BMS_Data.maximum_temperature>=CFG_Main[CFGID_ThermalShutdownUpperCharge]){
					FaultRaise(FAULT_B6_HIGH_TEMP_CHARGING,FaultInfoBuff);
				}else{
					FaultLower(FAULT_B6_HIGH_TEMP_CHARGING);
				}

				if(BMS_Data.maximum_temperature<=CFG_Main[CFGID_ThermalShutdownLowerCharge]){
					FaultRaise(FAULT_B7_LOW_TEMP_CHARGING,FaultInfoBuff);
				}else{
					FaultLower(FAULT_B7_LOW_TEMP_CHARGING);
				}
			}else{
				if(BMS_Data.maximum_temperature>=CFG_Main[CFGID_ThermalShutdownUpperDischarge]){
					FaultRaise(FAULT_B4_HIGH_TEMP,FaultInfoBuff);
				}else{
					FaultLower(FAULT_B4_HIGH_TEMP);
				}

				if(BMS_Data.maximum_temperature<=CFG_Main[CFGID_ThermalShutdownLowerDischarge]){
					FaultRaise(FAULT_B5_LOW_TEMP,FaultInfoBuff);
				}else{
					FaultLower(FAULT_B5_LOW_TEMP);
				}
			}

			StopclkEnd(STOPCLK_TEMP_CYCLE);
		}else{
			ScheduleTask(SCH_TEMP_Config, 0, False,0);
		}



	}
	SCHEDULE_HANDLE(SCH_CAN_CELL)
		CAN_UTIL_SetByte(CAN_TX_Cells,0,BYTE1(CAN_Cell_Index) | (BMS_Data.DCC[CAN_Cell_Index/CELLS_PER_SEG][CAN_Cell_Index%CELLS_PER_SEG]<<7));
		CAN_UTIL_SetByte(CAN_TX_Cells,1,BYTE0(CAN_Cell_Index));
		CAN_UTIL_SetByte(CAN_TX_Cells,2,BYTE1(BMS_Data.cell_voltage[CAN_Cell_Index/CELLS_PER_SEG][CAN_Cell_Index%CELLS_PER_SEG]));
		CAN_UTIL_SetByte(CAN_TX_Cells,3,BYTE0(BMS_Data.cell_voltage[CAN_Cell_Index/CELLS_PER_SEG][CAN_Cell_Index%CELLS_PER_SEG]));
		CAN_UTIL_SetByte(CAN_TX_Cells,4,0);
		CAN_UTIL_SetByte(CAN_TX_Cells,5,0);
		CAN_UTIL_SetByte(CAN_TX_Cells,6,0);
		CAN_UTIL_SetByte(CAN_TX_Cells,7,0);

		CAN_UTIL_Transmit(&hcan1, CAN_TX_Cells);
		//CAN_UTIL_TransmitUSB(CAN_TX_Cells);

		INCLOOP(CAN_Cell_Index,CELLS_PER_SEG*SEGMENT_N);
	}
	SCHEDULE_HANDLE(SCH_CAN_TEMP_MAIN)

		CAN_UTIL_SetByte(CAN_TX_Temperature_Main,0,BYTE1(CAN_Temp_Index));
		CAN_UTIL_SetByte(CAN_TX_Temperature_Main,1,BYTE0(CAN_Temp_Index));
		CAN_UTIL_SetByte(CAN_TX_Temperature_Main,2,BYTE1(BMS_Data.segment_temperatures[CAN_Temp_Index/22][CAN_Temp_Index%22]));
		CAN_UTIL_SetByte(CAN_TX_Temperature_Main,3,BYTE0(BMS_Data.segment_temperatures[CAN_Temp_Index/22][CAN_Temp_Index%22]));
		CAN_UTIL_SetByte(CAN_TX_Temperature_Main,4,BYTE1(BMS_Data.maximum_temperature));
		CAN_UTIL_SetByte(CAN_TX_Temperature_Main,5,BYTE0(BMS_Data.maximum_temperature));
		CAN_UTIL_SetByte(CAN_TX_Temperature_Main,6,BYTE1(BMS_Data.minimum_temperature));
		CAN_UTIL_SetByte(CAN_TX_Temperature_Main,7,BYTE0(BMS_Data.minimum_temperature));

		CAN_UTIL_Transmit(&hcan1, CAN_TX_Temperature_Main);
		//CAN_UTIL_TransmitUSB(CAN_TX_Temperature_Main);

		INCLOOP(CAN_Temp_Index,22*SEGMENT_N);
	}
	SCHEDULE_HANDLE(SCH_CAN_TEMP_INTERNAL)

		CAN_UTIL_SetByte(CAN_TX_Internal_Temp,0,0);
		CAN_UTIL_SetByte(CAN_TX_Internal_Temp,1,BYTE1(internal_temperature));
		CAN_UTIL_SetByte(CAN_TX_Internal_Temp,2,BYTE0(internal_temperature));
		CAN_UTIL_SetByte(CAN_TX_Internal_Temp,3,0);
		CAN_UTIL_SetByte(CAN_TX_Internal_Temp,4,0);
		CAN_UTIL_SetByte(CAN_TX_Internal_Temp,5,0);
		CAN_UTIL_SetByte(CAN_TX_Internal_Temp,6,0);
		CAN_UTIL_SetByte(CAN_TX_Internal_Temp,7,0);

		CAN_UTIL_Transmit(&hcan1, CAN_TX_Internal_Temp);
		//CAN_UTIL_TransmitUSB(CAN_TX_Internal_Temp);
	}
	SCHEDULE_HANDLE(SCH_CAN_STOPCLK)

		CAN_UTIL_SetByte(CAN_TX_Stopclk,0,CAN_Stopclk_Index);
		CAN_UTIL_SetByte(CAN_TX_Stopclk,1,BYTE3(StopClocker[CAN_Stopclk_Index].duration_us));
		CAN_UTIL_SetByte(CAN_TX_Stopclk,2,BYTE2(StopClocker[CAN_Stopclk_Index].duration_us));
		CAN_UTIL_SetByte(CAN_TX_Stopclk,3,BYTE1(StopClocker[CAN_Stopclk_Index].duration_us));
		CAN_UTIL_SetByte(CAN_TX_Stopclk,4,BYTE0(StopClocker[CAN_Stopclk_Index].duration_us));
		CAN_UTIL_SetByte(CAN_TX_Stopclk,5,StopClocker[CAN_Stopclk_Index].count);
		CAN_UTIL_SetByte(CAN_TX_Stopclk,6,0);
		CAN_UTIL_SetByte(CAN_TX_Stopclk,7,0);

		CAN_UTIL_Transmit(&hcan1, CAN_TX_Stopclk);
		//CAN_UTIL_TransmitUSB(CAN_TX_Stopclk);
		INCLOOP(CAN_Stopclk_Index,STOPCLK_N);
	}
	SCHEDULE_HANDLE(SCH_CAN_TEMP_SEG)

		CAN_UTIL_SetByte(CAN_TX_Temperature_Segment,0,CAN_Temp_Segment_Index);
		CAN_UTIL_SetByte(CAN_TX_Temperature_Segment,1,BYTE1(BMS_Data.segment_maximum_temperatures[CAN_Temp_Segment_Index]));
		CAN_UTIL_SetByte(CAN_TX_Temperature_Segment,2,BYTE0(BMS_Data.segment_maximum_temperatures[CAN_Temp_Segment_Index]));
		CAN_UTIL_SetByte(CAN_TX_Temperature_Segment,3,BYTE1(BMS_Data.segment_minimum_temperatures[CAN_Temp_Segment_Index]));
		CAN_UTIL_SetByte(CAN_TX_Temperature_Segment,4,BYTE0(BMS_Data.segment_minimum_temperatures[CAN_Temp_Segment_Index]));
		CAN_UTIL_SetByte(CAN_TX_Temperature_Segment,5,0);
		CAN_UTIL_SetByte(CAN_TX_Temperature_Segment,6,0);
		CAN_UTIL_SetByte(CAN_TX_Temperature_Segment,7,0);

		CAN_UTIL_Transmit(&hcan1, CAN_TX_Temperature_Segment);
		//CAN_UTIL_TransmitUSB(CAN_TX_Temperature_Segment);
		INCLOOP(CAN_Temp_Segment_Index,SEGMENT_N);
	}
	SCHEDULE_HANDLE(SCH_CAN_KeepAlive)
		CAN_UTIL_SetData(CAN_TX_KeepAlive,0,0,0,0,0,0,0,0);
		CAN_UTIL_Transmit(&hcan1, CAN_TX_KeepAlive);
		CAN_UTIL_Transmit(&hcan2, CAN_TX_KeepAlive);

	}
	SCHEDULE_HANDLE(SCH_CAN_FAN)
		CAN_UTIL_SetData(CAN_TX_Fan,CAN_Fan_Index,BYTE1(hfans[CAN_Fan_Index].speed),BYTE0(hfans[CAN_Fan_Index].speed),BMS_Data.fan_pwm*2,0,0,0,0);
		CAN_UTIL_Transmit(&hcan1, CAN_TX_Fan);
		INCLOOP(CAN_Fan_Index,FAN_N);
	}
	SCHEDULE_HANDLE(SCH_CAN_Battery)
		CAN_UTIL_SetData(CAN_TX_Battery,BMS_Data.SOC,BYTE1(BMS_Data.pack_voltage/1000),BYTE0(BMS_Data.pack_voltage/1000),BYTE1(BMS_Data.pack_current),BYTE0(BMS_Data.pack_current),BMS_Data.latches,BMS_Data.n_ic_connected/2,BMS_Data.status);
		CAN_UTIL_Transmit(&hcan1, CAN_TX_Battery);
	}
	SCHEDULE_HANDLE(SCH_CAN_Charger)
		if(BITCHECK(DeviceConnections,DEVCON_Charger)){
			uint16_t max_voltage = (CELLS_N * CFG_Main[CFGID_MaximumCellVoltageCharging])/1000;
			uint16_t max_current = BMS_Data.maximum_current_charge/10;
			uint8_t charging_on = False;

			if(!BITCHECK(DeviceConnections,DEVCON_RCB) || (!Status_Check(STATUS_CHARGE))){
				max_voltage = 0;
				max_current = 0;
			}

			if((max_current>0) && (!FaultCheck(FAULT_B3_HIGH_CELL_VOLTAGE))){
				charging_on = True;
			}
			CAN_UTIL_SetData(CAN_TX_Charger,BYTE1(max_voltage),BYTE0(max_voltage),BYTE1(max_current),BYTE0(max_current),charging_on?0:1,0,0,0);
			CAN_UTIL_Transmit(&hcan1, CAN_TX_Charger);
			CAN_UTIL_Transmit(&hcan2, CAN_TX_Charger);
		}
	}
	SCHEDULE_HANDLE(SCH_CAN_Current)
		if(BITCHECK(DeviceConnections,DEVCON_Charger)){
			CAN_UTIL_SetData(CAN_TX_Current,BYTE1(BMS_Data.maximum_current_charge/10),BYTE0(BMS_Data.maximum_current_charge/10),0,0,0,0,0,0);
		}else{
			CAN_UTIL_SetData(CAN_TX_Current,BYTE1(BMS_Data.maximum_current_discharge/10),BYTE0(BMS_Data.maximum_current_discharge/10),0,0,0,0,0,0);
		}
		CAN_UTIL_Transmit(&hcan1, CAN_TX_Current);
	}
	SCHEDULE_HANDLE(SCH_CAN_FaultsA)
		CAN_UTIL_SetByte(CAN_TX_FaultsA,0,BYTE1(hfault.faultsA.faults));
		CAN_UTIL_SetByte(CAN_TX_FaultsA,1,BYTE0(hfault.faultsA.faults));
		CAN_UTIL_SetByte(CAN_TX_FaultsA,2,BYTE1(CFG_Main[CFGID_FaultcodeFatalA]));
		CAN_UTIL_SetByte(CAN_TX_FaultsA,3,BYTE0(CFG_Main[CFGID_FaultcodeFatalA]));
		CAN_UTIL_SetByte(CAN_TX_FaultsA,4,BYTE1(CFG_Main[CFGID_FaultcodeResettableA]));
		CAN_UTIL_SetByte(CAN_TX_FaultsA,5,BYTE0(CFG_Main[CFGID_FaultcodeResettableA]));
		CAN_UTIL_SetByte(CAN_TX_FaultsA,6,0);
		CAN_UTIL_SetByte(CAN_TX_FaultsA,7,0);
		CAN_UTIL_Transmit(&hcan1, CAN_TX_FaultsA);
	}
	SCHEDULE_HANDLE(SCH_CAN_FaultsB)
		CAN_UTIL_SetByte(CAN_TX_FaultsB,0,BYTE1(hfault.faultsB.faults));
		CAN_UTIL_SetByte(CAN_TX_FaultsB,1,BYTE0(hfault.faultsB.faults));
		CAN_UTIL_SetByte(CAN_TX_FaultsB,2,BYTE1(CFG_Main[CFGID_FaultcodeFatalB]));
		CAN_UTIL_SetByte(CAN_TX_FaultsB,3,BYTE0(CFG_Main[CFGID_FaultcodeFatalB]));
		CAN_UTIL_SetByte(CAN_TX_FaultsB,4,BYTE1(CFG_Main[CFGID_FaultcodeResettableB]));
		CAN_UTIL_SetByte(CAN_TX_FaultsB,5,BYTE0(CFG_Main[CFGID_FaultcodeResettableB]));
		CAN_UTIL_SetByte(CAN_TX_FaultsB,6,0);
		CAN_UTIL_SetByte(CAN_TX_FaultsB,7,0);
		CAN_UTIL_Transmit(&hcan1, CAN_TX_FaultsB);
	}
	SCHEDULE_HANDLE(SCH_CAN_FaultsC)
		CAN_UTIL_SetByte(CAN_TX_FaultsC,0,BYTE1(hfault.faultsC.faults));
		CAN_UTIL_SetByte(CAN_TX_FaultsC,1,BYTE0(hfault.faultsC.faults));
		CAN_UTIL_SetByte(CAN_TX_FaultsC,2,BYTE1(CFG_Main[CFGID_FaultcodeFatalC]));
		CAN_UTIL_SetByte(CAN_TX_FaultsC,3,BYTE0(CFG_Main[CFGID_FaultcodeFatalC]));
		CAN_UTIL_SetByte(CAN_TX_FaultsC,4,BYTE1(CFG_Main[CFGID_FaultcodeResettableC]));
		CAN_UTIL_SetByte(CAN_TX_FaultsC,5,BYTE0(CFG_Main[CFGID_FaultcodeResettableC]));
		CAN_UTIL_SetByte(CAN_TX_FaultsC,6,0);
		CAN_UTIL_SetByte(CAN_TX_FaultsC,7,0);
		CAN_UTIL_Transmit(&hcan1, CAN_TX_FaultsC);
	}
	SCHEDULE_HANDLE(SCH_CAN_FaultsD)
		CAN_UTIL_SetByte(CAN_TX_FaultsD,0,BYTE1(hfault.faultsD.faults));
		CAN_UTIL_SetByte(CAN_TX_FaultsD,1,BYTE0(hfault.faultsD.faults));
		CAN_UTIL_SetByte(CAN_TX_FaultsD,2,BYTE1(CFG_Main[CFGID_FaultcodeFatalD]));
		CAN_UTIL_SetByte(CAN_TX_FaultsD,3,BYTE0(CFG_Main[CFGID_FaultcodeFatalD]));
		CAN_UTIL_SetByte(CAN_TX_FaultsD,4,BYTE1(CFG_Main[CFGID_FaultcodeResettableD]));
		CAN_UTIL_SetByte(CAN_TX_FaultsD,5,BYTE0(CFG_Main[CFGID_FaultcodeResettableD]));
		CAN_UTIL_SetByte(CAN_TX_FaultsD,6,0);
		CAN_UTIL_SetByte(CAN_TX_FaultsD,7,0);
		CAN_UTIL_Transmit(&hcan1, CAN_TX_FaultsD);
	}
	SCHEDULE_HANDLE(SCH_TIMEOUT_Client)
		BITRESET(DeviceConnections, DEVCON_Client);
		OUTPUT_RESET(OUT_LED_WARN);
		ScheduleTaskStop(SCH_TIMEOUT_Client);
	}
	SCHEDULE_HANDLE(SCH_TIMEOUT_Charger)
		BITRESET(DeviceConnections, DEVCON_Charger);
		FaultRaise(FAULT_D2_DEVCON_CHARGER, FaultInfoBuff);
		ScheduleTaskStop(SCH_TIMEOUT_Charger);
	}
	SCHEDULE_HANDLE(SCH_TIMEOUT_RCB)
		BITRESET(DeviceConnections, DEVCON_RCB);
		FaultRaise(FAULT_D1_DEVCON_RCB, FaultInfoBuff);
		if(BMS_Data.ts_active){
			BMS_Data.ts_active = False;
			if(BITCHECK(DeviceConnections, DEVCON_Charger)){
				Status_Set(STATUS_CHARGE_RDY);
				ScheduleTask(SCH_FaultLowerRCBLostCharging, 1000, False, 0);
				FaultRaise(FAULT_C7_RCB_LOST_WHILE_CHARGING, FaultInfoBuff);
			}else{
				Status_Set(STATUS_DISCHARGE_RDY);
				ScheduleTask(SCH_FaultLowerRCBLostDischarging, 1000, False, 0);
				FaultRaise(FAULT_C5_RCB_LOST_WHILE_DISCHARGING, FaultInfoBuff);
			}
		}
		ScheduleTaskStop(SCH_TIMEOUT_RCB);
	}
	SCHEDULE_HANDLE(SCH_FaultLowerRCBLostCharging)
		FaultLower(FAULT_C7_RCB_LOST_WHILE_CHARGING);
	}
	SCHEDULE_HANDLE(SCH_FaultLowerRCBLostDischarging)
		FaultLower(FAULT_C5_RCB_LOST_WHILE_DISCHARGING);
	}
	SCHEDULE_HANDLE(SCH_WarnInfoHandle)
		WarnInfoHandler();
	}
	SCHEDULE_HANDLE(SCH_CheckOvercurrent)
		overcurrent_check=False;
		if(BITCHECK(DeviceConnections,DEVCON_Charger)){
			if(BMS_Data.pack_current<(-((int32_t)CFG_Main[CFGID_MaximumChargeCurrent])*100)){
				FaultRaise(FAULT_B13_HIGH_CURRENT_CHARGING, FaultInfoBuff);
			}
		}else{
			if(BMS_Data.pack_current>(CFG_Main[CFGID_MaximumDischargeCurrent])*100){
				FaultRaise(FAULT_B14_HIGH_CURRENT_DISCHARGING, FaultInfoBuff);
			}
		}
	}
	SCHEDULE_HANDLE(SCH_LogDOD)
		//Check if SPI is free.
		if(BMS_Data.spi_free==True){	//If SPI free, take control of SPI & start temperature config operation.
			flash_data[0] = battery_initial_charge_lost + current_integrator.value;
			flash_data[1] = (flash_data[1])+1;;
			DisableIRQ();
			Flash_Write_Data(SECTOR4, flash_data, 2);
			//HAL_Delay(20);
			EnableIRQ();
		}else{
			ScheduleQueue[SCH_LogDOD].flag = True; //If SPI in-use, check again on next schedule cycle.
		}
	}
	SCHEDULE_HANDLE(SCH_CalibrateCurrentSensor)
		CalibrateCurrentSensor();
	}
	SCHEDULE_HANDLE(SCH_VoltageROC)
		Derivative(&voltage_roc, BMS_Data.pack_voltage);
		if(!BMS_Data.ts_active){
			if(voltage_roc.count>3){
				if((voltage_roc.value>-5)&&(voltage_roc.value<5)){
					SOC_Correction();
				}else{
					FaultRaise(FAULT_A16_VOLTAGE_FLUCTUATION, FaultInfoBuff);
				}
			}
		}
	}
	SCHEDULE_HANDLE(SCH_FAULT_MONITOR)
		if(!Status_Check(STATUS_SHUTDOWN)){
			uint8_t fatal_fault = False;
			for(int i = 0;i<16;i++){
				if(!BITSHIFTCHECK(CFG_Main[CFGID_FaultcodeFatalA],i)){
					if(FaultCheck(FAULT_A | (1<<i))){
						fatal_fault = True;
						break;
					}
				}
				if(!BITSHIFTCHECK(CFG_Main[CFGID_FaultcodeFatalB],i)){
					if(FaultCheck(FAULT_B | (1<<i))){
						fatal_fault = True;
						break;
					}
				}
				if(!BITSHIFTCHECK(CFG_Main[CFGID_FaultcodeFatalC],i)){
					if(FaultCheck(FAULT_C | (1<<i))){
						fatal_fault = True;
						break;
					}
				}
				if(!BITSHIFTCHECK(CFG_Main[CFGID_FaultcodeFatalD],i)){
					if(FaultCheck(FAULT_D | (1<<i))){
						fatal_fault = True;
						break;
					}
				}
			}
			if(!fatal_fault){
				if((!Status_Check(STATUS_DISCHARGE))&&(!Status_Check(STATUS_CHARGE))){
					BMS_Okay(True);
				}
			}
		}
	}
	SCHEDULE_HANDLE(SCH_Current_Decision)
		if(BITCHECK(DeviceConnections,DEVCON_Charger)){
			BMS_Data.maximum_current_discharge = 0;

			uint16_t min_charge_current = CFG_Main[CFGID_MaximumChargeCurrent]*100;
			uint16_t charge_current = 0;


			if(!Status_Check(STATUS_CHARGE)){
				charge_current = 0;
				if(charge_current<min_charge_current){
					min_charge_current = charge_current;
				}
			}else{
				charge_current = CFG_Main[CFGID_MaximumChargeCurrent]*100;
				if(charge_current<min_charge_current){
					min_charge_current = charge_current;
				}

				if(BMS_Data.maximum_temperature>CFG_Main[CFGID_ThermalDerateUpperStartCharge]){
					charge_current = lerp(0, CFG_Main[CFGID_MaximumChargeCurrent]*100, 100-normalise(CFG_Main[CFGID_ThermalDerateUpperStartCharge], CFG_Main[CFGID_ThermalDerateUpperEndCharge], BMS_Data.maximum_temperature, 100), 100);
					if(charge_current<min_charge_current){
						min_charge_current = charge_current;
					}
				}else if(BMS_Data.maximum_temperature<CFG_Main[CFGID_ThermalDerateLowerEndCharge]){
					charge_current = lerp(0, CFG_Main[CFGID_MaximumChargeCurrent]*100, normalise(CFG_Main[CFGID_ThermalDerateLowerStartCharge], CFG_Main[CFGID_ThermalDerateLowerEndCharge], BMS_Data.maximum_temperature, 100), 100);
					if(charge_current<min_charge_current){
						min_charge_current = charge_current;
					}
				}

				if(BMS_Data.maximum_cell_voltage>(CFG_Main[CFGID_MinimumCellVoltage]+(CFG_Main[CFGID_MaximumCellVoltageCharging]-CFG_Main[CFGID_MinimumCellVoltage])*CFG_Main[CFGID_VoltageDerateStartCharging]/100)){
					charge_current = lerp(CFG_Main[CFGID_MinCurrentCharging]*100, CFG_Main[CFGID_MaximumChargeCurrent]*100, 1000 - normalise((CFG_Main[CFGID_MinimumCellVoltage]+(CFG_Main[CFGID_MaximumCellVoltageCharging]-CFG_Main[CFGID_MinimumCellVoltage])*CFG_Main[CFGID_VoltageDerateStartCharging]/100), CFG_Main[CFGID_MaximumCellVoltageCharging], BMS_Data.maximum_cell_voltage, 1000), 1000);
					if(charge_current<min_charge_current){
						min_charge_current = charge_current;
					}
				}

				BMS_Data.maximum_current_charge = min_charge_current;

			}

		}else{
			uint16_t min_discharge_current = CFG_Main[CFGID_MaximumDischargeCurrent]*100;
			uint16_t discharge_current = 0;
			BMS_Data.maximum_current_charge = -10000;
			if(!Status_Check(STATUS_DISCHARGE)){
				discharge_current = 0;
				if(discharge_current<min_discharge_current){
					min_discharge_current = discharge_current;
				}
			}else{
				discharge_current = CFG_Main[CFGID_MaximumDischargeCurrent]*100;
				if(discharge_current<min_discharge_current){
					min_discharge_current = discharge_current;
				}

				if(BMS_Data.maximum_temperature>CFG_Main[CFGID_ThermalDerateUpperStartDischarge]){
					discharge_current = lerp(0, CFG_Main[CFGID_MaximumDischargeCurrent]*100, 100-normalise(CFG_Main[CFGID_ThermalDerateUpperStartDischarge], CFG_Main[CFGID_ThermalDerateUpperEndDischarge], BMS_Data.maximum_temperature, 100), 100);
					if(discharge_current<min_discharge_current){
						min_discharge_current = discharge_current;
					}
				}else if(BMS_Data.maximum_temperature<CFG_Main[CFGID_ThermalDerateLowerEndDischarge]){
					discharge_current = lerp(0, CFG_Main[CFGID_MaximumDischargeCurrent]*100, normalise(CFG_Main[CFGID_ThermalDerateLowerStartDischarge], CFG_Main[CFGID_ThermalDerateLowerEndDischarge], BMS_Data.maximum_temperature, 100), 100);
					if(discharge_current<min_discharge_current){
						min_discharge_current = discharge_current;
					}
				}

				if(BMS_Data.pack_voltage>CFG_Main[CFGID_MinimumVoltageDischarge]){
					discharge_current = clamp((BMS_Data.pack_voltage-CFG_Main[CFGID_MinimumVoltageDischarge])*100/CFG_Main[CFGID_BatteryImpedance], 0, CFG_Main[CFGID_MaximumDischargeCurrent]*100);
					if(discharge_current<min_discharge_current){
						min_discharge_current = discharge_current;
					}
				}else{
					min_discharge_current = 0;
				}


				BMS_Data.maximum_current_discharge = min_discharge_current;

			}
		}
	}
	SCHEDULE_HANDLE(SCH_CheckLatches)
		if(HAL_GPIO_ReadPin(IMD_LATCH_GPIO_Port, IMD_LATCH_Pin)){
			BITSET(BMS_Data.latches,IND_IMD_LATCH);
		}else{
			BITRESET(BMS_Data.latches,IND_IMD_LATCH);
		}

		if(HAL_GPIO_ReadPin(IMD_OKAY_GPIO_Port, IMD_OKAY_Pin)){
			BITSET(BMS_Data.latches,IND_IMD_OKAY);
		}else{
			BITRESET(BMS_Data.latches,IND_IMD_OKAY);
		}

		if(HAL_GPIO_ReadPin(AMS_LATCH_GPIO_Port, AMS_LATCH_Pin)){
			BITSET(BMS_Data.latches,IND_AMS_LATCH);
		}else{
			BITRESET(BMS_Data.latches,IND_AMS_LATCH);
		}

		if(HAL_GPIO_ReadPin(AMS_OKAY_GPIO_Port, AMS_OKAY_Pin)){
			BITSET(BMS_Data.latches,IND_AMS_OKAY);
		}else{
			BITRESET(BMS_Data.latches,IND_AMS_OKAY);
		}

		if(BITCHECK(BMS_Data.latches,IND_IMD_LATCH)&&(!BITCHECK(BMS_Data.latches,IND_IMD_OKAY))){
			FaultRaise(FAULT_C3_IMD_LATCH_CLOSED, FaultInfoBuff);
		}else{
			FaultLower(FAULT_C3_IMD_LATCH_CLOSED);
		}

		if(BITCHECK(BMS_Data.latches,IND_AMS_LATCH)&&(!BITCHECK(BMS_Data.latches,IND_AMS_OKAY))){
			FaultRaise(FAULT_C4_AMS_LATCH_CLOSED, FaultInfoBuff);
		}else{
			FaultLower(FAULT_C4_AMS_LATCH_CLOSED);
		}
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_SPI3_Init();
  MX_TIM4_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_ADC2_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */




  CAN_UTIL_Setup(CAN_TX_Cells, 0x01B36, 8);
  CAN_UTIL_Setup(CAN_TX_Temperature_Main, 0x01B72, 8);
  CAN_UTIL_Setup(CAN_TX_Temperature_Segment, 0x01B73, 8);
  CAN_UTIL_Setup(CAN_TX_Internal_Temp, 0x04A72, 8);
  CAN_UTIL_Setup(CAN_TX_Stopclk, 0x04A26, 8);
  CAN_UTIL_Setup(CAN_TX_Voltage_Bounds, 0x01B37, 8);
  CAN_UTIL_Setup(CAN_TX_Configuration, 0x0A12, 8);
  CAN_UTIL_Setup(CAN_TX_ConfigurationTX, 0x0A13, 8);
  CAN_UTIL_Setup(CAN_TX_KeepAlive, 0x1B00, 8);
  CAN_UTIL_Setup(CAN_TX_WarnInfo, 0x1B10, 8);
  CAN_UTIL_Setup(CAN_TX_BalanceResponse, 0x1B38, 8);
  CAN_UTIL_Setup(CAN_TX_FaultsA, 0x1B16, 8);
  CAN_UTIL_Setup(CAN_TX_FaultsB, 0x1B17, 8);
  CAN_UTIL_Setup(CAN_TX_FaultsC, 0x1B18, 8);
  CAN_UTIL_Setup(CAN_TX_FaultsD, 0x1B19, 8);
  CAN_UTIL_Setup(CAN_TX_Fan, 0x1B80, 8);
  CAN_UTIL_Setup(CAN_TX_Battery, 0x1B08, 8);
  CAN_UTIL_Setup(CAN_TX_Charger, 0x1806E5F4, 8);
  CAN_UTIL_Setup(CAN_TX_Current, 0x1B09, 8);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

  OUTPUT_SET(OUT_LED_FAULT);
  delayu(500000);
  OUTPUT_SET(OUT_LED_INDICATOR);
  delayu(500000);
  OUTPUT_SET(OUT_LED_WARN);
  delayu(500000);
  OUTPUT_SET(OUT_LED_CAN);
  delayu(500000);
  OUTPUT_RESET(OUT_LED_INDICATOR);
  OUTPUT_RESET(OUT_LED_FAULT);
  OUTPUT_RESET(OUT_LED_WARN);
  OUTPUT_RESET(OUT_LED_CAN);

  OUTPUT_RESET(OUT_SPI_SLOW);
  OUTPUT_SET(OUT_SPI_MASTER);
  OUTPUT_SET(OUT_SPI_POL);
  OUTPUT_SET(OUT_SPI_PHA);

  AddWarnInfo(WARNINFO_FirstConnectOnStart,NullData);

  //===LOADING CONFIG ON STARTUP===
  Flash_Read_Data(SECTOR7, CFG_Main, N_CONFIG_WORDS);
  Flash_Read_Data(SECTOR6, CFG_Backup, N_CONFIG_WORDS);
  Flash_Read_Data(SECTOR5, CFG_Info, 1);
  {
  Flash_Read_Data(SECTOR4, flash_data, 2);
  battery_initial_charge_lost = flash_data[0];
  BMS_Data.SOC = (battery_max_charge - battery_initial_charge_lost)*200/battery_max_charge;



  uint8_t configs_mismatch=0;
  switch(CFG_Info[0]){
  case CFGFS_Config:
	  //No issues to fix, however let client know power was lost during config at first connect.
	  AddWarnInfo(WARNINFO_Flash_Config_PwrLost,NullData);
	  break;
  case CFGFS_FlashMain:
	  //Overwrite main config with backup as power was lost during flash.
	  AddWarnInfo(WARNINFO_Flash_Main_PwrLost,NullData);
	  Flash_Write_Data(SECTOR7, CFG_Backup, N_CONFIG_WORDS);
	  for(int i = 0;i<N_CONFIG_WORDS;i++){
		  CFG_Main[i] = CFG_Backup[i];
	  }
	  break;
  case CFGFS_FlashBackup:
	  //Overwrite backup config with main as power was lost during flash.
	  AddWarnInfo(WARNINFO_Flash_Backup_PwrLost,NullData);
	  Flash_Write_Data(SECTOR6, CFG_Main, N_CONFIG_WORDS);
	  break;
  case CFGFS_None:
	  for(int i = 0;i<N_CONFIG_WORDS;i++){
		  if(CFG_Main[i]!=CFG_Backup[i]){
			  configs_mismatch=1;
			  break;
		  }
	  }
	  if(configs_mismatch){
		  //Error, configs mismatch on startup when everything appeared okay.
		  AddWarnInfo(WARNINFO_Flash_Config_Mismatch,NullData);
	  }
	  break;
  }
  }

  CFG_Info[0]=0;
  Flash_Write_Data(SECTOR5, CFG_Info, 1);

  //=======

  Status_Set(STATUS_STARTUP);

  FaultRaise(FAULT_D2_DEVCON_CHARGER, FaultInfoBuff);
  FaultRaise(FAULT_D1_DEVCON_RCB, FaultInfoBuff);

  // Start CAN
  HAL_CAN_Start(&hcan1);
  HAL_CAN_Start(&hcan2);
  // Activate interrupts for the TX Mailbox Empty and for an incoming message
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING);

  HAL_TIM_Base_Start_IT(&SOFTCLK_TIMER_TYPE);
  HAL_TIM_Base_Start_IT(&I_SENSE_TIMER_TYPE);
  HAL_TIM_Base_Start(&STOPCLK_TIMER_TYPE);

  LTC6804_initialize();

  BMS_Data.current_mux = 0;


  ScheduleTask(SCH_LED_FAULT, 100, True,0);
  ScheduleTask(SCH_LED_INDICATOR, 200, True,0);
  ScheduleTask(SCH_CELL_Init, 500, True,3);
  ScheduleTask(SCH_TEMP_Init, 1000, True,7);
  ScheduleTask(SCH_BALANCE_Init, 1000, CFG_Main[CFGID_BalancingEnable]?False:True,20);

  //ScheduleTask(SCH_TEMP_INTERNAL_ADC, 1000, True,0);

  ScheduleTask(SCH_CAN_CELL, 8, True,1);
  ScheduleTask(SCH_CAN_TEMP_INTERNAL, 250, True,2);
  ScheduleTask(SCH_CAN_TEMP_MAIN, 5, True,3);
  ScheduleTask(SCH_CAN_TEMP_SEG, 100, True,4);
  ScheduleTask(SCH_CAN_STOPCLK, 30, True,0);
  ScheduleTask(SCH_CAN_FAN, TACH_PERIOD/FAN_N, True,1);

  ScheduleTask(SCH_CAN_KeepAlive, 1000, True,5);
  ScheduleTask(SCH_CAN_FaultsA, 250, True,10);
  ScheduleTask(SCH_CAN_FaultsB, 250, True,20);
  ScheduleTask(SCH_CAN_FaultsC, 250, True,30);
  ScheduleTask(SCH_CAN_FaultsD, 250, True,40);
  ScheduleTask(SCH_CAN_Battery, 250, True,25);

  ScheduleTask(SCH_WarnInfoHandle, 250, True,6);

  ScheduleTask(SCH_FAULT_MONITOR, 5000, True, 0);

  ScheduleTask(SCH_CAN_Charger, 800, True, 18);
  ScheduleTask(SCH_CAN_Current, 500, True, 21);
  ScheduleTask(SCH_CheckLatches, 250, True, 0);
  ScheduleTask(SCH_Current_Decision, 250, True, 0);

  ScheduleTask(SCH_FAN_Update, TACH_PERIOD, True, 0);

  ScheduleTask(SCH_LogDOD, 10000, True, 0);

  ScheduleTask(SCH_VoltageROC, 30000, True, 0);

  ScheduleTask(SCH_CalibrateCurrentSensor, 5000, False, 0);

  for(int fan = 0;fan<FAN_N;fan++){
	  MOVING_AVERAGE_Reset(&(hfans[fan].ave_speed),16);
  }
  PID_SetGain(&hpid_fans,150, 10, 100);
  PID_UpdateSetpoint(&hpid_fans, 4500);

  BMS_Data.spi_free = True;

  MOVING_AVERAGE_Reset(&current_ave_ch1, 5);
  MOVING_AVERAGE_Reset(&current_ave_ch2, 5);

  MOVING_AVERAGE_Reset(&current_offset_ch1, 5);
  MOVING_AVERAGE_Reset(&current_offset_ch2, 5);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(CFG_Reset){
			HAL_Delay(20);
			HAL_NVIC_SystemReset();
		}
	  CAN_RX_Handler();
	  TaskScheduleSoftClock();
	  TaskScheduleHandler();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ENABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 3;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 4;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = ENABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  CAN_FilterTypeDef CAN_Filter;
  CAN_Filter.FilterActivation = CAN_FILTER_ENABLE;
  CAN_Filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  CAN_Filter.FilterBank = 0;
  CAN_Filter.FilterScale = CAN_FILTERSCALE_32BIT;
  CAN_Filter.FilterMode = CAN_FILTERMODE_IDMASK;

  uint32_t filter_id = 	 0x1FFFFFFF;
  uint32_t filter_mask = 0x0;

  CAN_Filter.FilterIdHigh = ((filter_id<<5)|(filter_id>>(32-5)))&0xFFFF;
  CAN_Filter.FilterIdLow = (filter_id>>(11-3))&0xFFF8;
  CAN_Filter.FilterMaskIdHigh = ((filter_mask<<5)|(filter_mask>>(32-5)))&0xFFFF;
  CAN_Filter.FilterMaskIdLow = (filter_mask>>(11-3))&0xFFF8;

  HAL_CAN_ConfigFilter(&hcan1, &CAN_Filter);
  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 4;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = ENABLE;
  hcan2.Init.AutoWakeUp = ENABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */
  CAN_FilterTypeDef CAN_Filter;
  CAN_Filter.FilterActivation = CAN_FILTER_ENABLE;
  CAN_Filter.FilterFIFOAssignment = CAN_FILTER_FIFO1;
  CAN_Filter.FilterBank = 14;
  CAN_Filter.FilterScale = CAN_FILTERSCALE_32BIT;
  CAN_Filter.FilterMode = CAN_FILTERMODE_IDMASK;

  uint32_t filter_id = 	 0x1FFFFFFF;
  uint32_t filter_mask = 0x0;

  CAN_Filter.FilterIdHigh = ((filter_id<<5)|(filter_id>>(32-5)))&0xFFFF;
  CAN_Filter.FilterIdLow = (filter_id>>(11-3))&0xFFF8;
  CAN_Filter.FilterMaskIdHigh = ((filter_mask<<5)|(filter_mask>>(32-5)))&0xFFFF;
  CAN_Filter.FilterMaskIdLow = (filter_mask>>(11-3))&0xFFF8;

  HAL_CAN_ConfigFilter(&hcan2, &CAN_Filter);
  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = APB2-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = APB1-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = APB1-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = APB1-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = APB1-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = APB1-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 50000;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_CAN_Pin|LED_WARN_Pin|LED_IND_Pin|LED_FAULT_Pin
                          |SPI_MSTR_Pin|SPI_SLOW_Pin|SPI_PHA_Pin|SPI_POL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPI_LED_EN_Pin|SPI_NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(AMS_OKAY_GPIO_Port, AMS_OKAY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_CAN_Pin LED_WARN_Pin LED_IND_Pin LED_FAULT_Pin
                           SPI_MSTR_Pin SPI_SLOW_Pin SPI_PHA_Pin SPI_POL_Pin */
  GPIO_InitStruct.Pin = LED_CAN_Pin|LED_WARN_Pin|LED_IND_Pin|LED_FAULT_Pin
                          |SPI_MSTR_Pin|SPI_SLOW_Pin|SPI_PHA_Pin|SPI_POL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : IMD_OKAY_Pin IMD_LATCH_Pin AMS_LATCH_Pin */
  GPIO_InitStruct.Pin = IMD_OKAY_Pin|IMD_LATCH_Pin|AMS_LATCH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : FAN_TACH4_Pin FAN_TACH3_Pin */
  GPIO_InitStruct.Pin = FAN_TACH4_Pin|FAN_TACH3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : FAN_TACH2_Pin FAN_TACH1_Pin */
  GPIO_InitStruct.Pin = FAN_TACH2_Pin|FAN_TACH1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI_LED_EN_Pin SPI_NSS_Pin */
  GPIO_InitStruct.Pin = SPI_LED_EN_Pin|SPI_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : AMS_OKAY_Pin */
  GPIO_InitStruct.Pin = AMS_OKAY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(AMS_OKAY_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef * htim) {
	if (htim == &SOFTCLK_TIMER_TYPE) {
		TaskScheduleSoftClock_FlagSet();
	}else if(htim == &I_SENSE_TIMER_TYPE){
		Start_CurrentADC();
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
//	if(hadc == &hadc1){
//		//Internal Thermistor Reading
//		uint32_t reading = HAL_ADC_GetValue(&hadc1);
//		internal_temperature = (int32_t)(((110.0f - 30.0f) / ((float)(*((uint16_t*)((uint32_t)0x1FFF7A2E))) - (float)(*((uint16_t*)((uint32_t)0x1FFF7A2C)))) * ((float)reading - (float)(*((uint16_t*)((uint32_t)0x1FFF7A2C)))) + 30.0f))*100+27315;
//		if((internal_temperature-27315)>10000){
//			OUTPUT_TOGGLE(OUT_LED_WARN);
//		}
//	}
	if(hadc==&hadc2){
		Finish_CurrentADC();
	}
}
void HAL_GPIO_EXTI_Callback( uint16_t GPIO_Pin)
{
	switch(GPIO_Pin){
	case FAN_TACH1_Pin:
		if(hfans[FAN_IN_1].tach_finished){return;}
		if(hfans[FAN_IN_1].tach_started){
			FAN_EndTach(&hfans[FAN_IN_1]);
		}else{
			FAN_TachActive(&hfans[FAN_IN_1]);
		}
		//Fan_TachUpdate(&hfans[FAN_IN_1]);
		break;
 	case FAN_TACH2_Pin:
		if(hfans[FAN_IN_2].tach_finished){return;}
		if(hfans[FAN_IN_2].tach_started){
			FAN_EndTach(&hfans[FAN_IN_2]);
		}else{
			FAN_TachActive(&hfans[FAN_IN_2]);
		}
		//Fan_TachUpdate(&(hfans[FAN_IN_2]));
		break;
	case FAN_TACH3_Pin:
		if(hfans[FAN_IN_3].tach_finished){return;}
		if(hfans[FAN_IN_3].tach_started){
			FAN_EndTach(&hfans[FAN_IN_3]);
		}else{
			FAN_TachActive(&hfans[FAN_IN_3]);
		}
		//Fan_TachUpdate(&hfans[FAN_IN_3]);
		break;
	case FAN_TACH4_Pin:
		if(hfans[FAN_IN_4].tach_finished){return;}
		if(hfans[FAN_IN_4].tach_started){
			FAN_EndTach(&hfans[FAN_IN_4]);
		}else{
			FAN_TachActive(&hfans[FAN_IN_4]);
		}
		//Fan_TachUpdate(&hfans[FAN_IN_4]);
		break;
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
