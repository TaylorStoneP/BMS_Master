/*
 * stm32_utils.h
 *
 *  Created on: Nov 6, 2022
 *      Author: Taylor Phillips
 */

#ifndef INC_STM32_UTILS_H_
#define INC_STM32_UTILS_H_

#include "main.h"

#define True 1
#define False 0

#define GPIO_PORT_ADDR 0x40020000

#define SPI_HANDLE hspi3

#define SOFTCLK_TIMER_TYPE htim4

#define DELAY_TIMER_TYPE htim1
#define DELAY_TIM TIM1

#define STOPCLK_TIMER_TYPE htim2
#define STOPCLK_TIM TIM2

extern TIM_HandleTypeDef DELAY_TIMER_TYPE;
extern TIM_HandleTypeDef SOFTCLK_TIMER_TYPE;
extern TIM_HandleTypeDef STOPCLK_TIMER_TYPE;

#define PINSETUP(pin, port)  (pin + (((uint32_t)port&0xFFFF)<<16))

#define IWDG_EN False

#if IWDG_EN == True
extern IWDG_HandleTypeDef hiwdg;
#define IWDG_RESET() HAL_IWDG_Refresh(&hiwdg);
#else
#define IWDG_RESET()
#endif

typedef enum PIN_UTIL{
	//OUT_x = x_pin | (x_port<<8),
	OUT_LED_FAULT 		= LED_FAULT_Pin + (((uint32_t)LED_FAULT_GPIO_Port&0xFFFF)<<16),
	OUT_LED_INDICATOR 	= LED_IND_Pin + (((uint32_t)LED_IND_GPIO_Port&0xFFFF)<<16),
	OUT_LED_CAN 		= LED_CAN_Pin + (((uint32_t)LED_CAN_GPIO_Port&0xFFFF)<<16),
	OUT_LED_WARN 		= LED_WARN_Pin + (((uint32_t)LED_WARN_GPIO_Port&0xFFFF)<<16),
	OUT_LED_NUCLEO 		= LED_NUCLEO_Pin + (((uint32_t)LED_NUCLEO_GPIO_Port&0xFFFF)<<16),
	OUT_SPI_SLOW 		= SPI_SLOW_Pin + (((uint32_t)SPI_SLOW_GPIO_Port&0xFFFF)<<16),
	OUT_SPI_MASTER 		= SPI_MSTR_Pin + (((uint32_t)SPI_MSTR_GPIO_Port&0xFFFF)<<16),
	OUT_SPI_POL 		= SPI_POL_Pin + (((uint32_t)SPI_POL_GPIO_Port&0xFFFF)<<16),
	OUT_SPI_PHA 		= SPI_PHA_Pin + (((uint32_t)SPI_PHA_GPIO_Port&0xFFFF)<<16),
	OUT_NSS 			= SPI_NSS_Pin + (((uint32_t)SPI_NSS_GPIO_Port&0xFFFF)<<16)
}PIN_UTIL;

#define OUTPUT_RESET(PIN_x) 	HAL_GPIO_WritePin((GPIO_TypeDef *)((PIN_x>>16)|GPIO_PORT_ADDR), PIN_x&0xFFFF, 0)
#define OUTPUT_SET(PIN_x) 		HAL_GPIO_WritePin((GPIO_TypeDef *)((PIN_x>>16)|GPIO_PORT_ADDR), PIN_x&0xFFFF, 1)
#define OUTPUT_TOGGLE(PIN_x) 	HAL_GPIO_TogglePin((GPIO_TypeDef *)((PIN_x>>16)|GPIO_PORT_ADDR), PIN_x&0xFFFF)
#define INPUT_READ(PIN_x) 		HAL_GPIO_ReadPin((GPIO_TypeDef *)((PIN_x>>16)|GPIO_PORT_ADDR), PIN_x&0xFFFF)

enum{
	BIT1 = 0x1,
	BIT2 = 0x2,
	BIT3 = 0x4,
	BIT4 = 0x8,
	BIT5 = 0x10,
	BIT6 = 0x20,
	BIT7 = 0x40,
	BIT8 = 0x80,
	BIT9 = 0x100,
	BIT10 = 0x200,
	BIT11 = 0x400,
	BIT12 = 0x800,
	BIT13 = 0x1000,
	BIT14 = 0x2000,
	BIT15 = 0x4000,
	BIT16 = 0x8000,
	BIT17 = 0x10000,
	BIT18 = 0x20000,
	BIT19 = 0x40000,
	BIT20 = 0x80000,
	BIT21 = 0x100000,
	BIT22 = 0x200000,
	BIT23 = 0x400000,
	BIT24 = 0x800000,
	BIT25 = 0x1000000,
	BIT26 = 0x2000000,
	BIT27 = 0x4000000,
	BIT28 = 0x8000000,
	BIT29 = 0x10000000,
	BIT30 = 0x20000000,
	BIT31 = 0x40000000,
	BIT32 = 0x80000000
	
};

#define BITCHECK(reg,bit) 	((reg&(bit)) == (bit))		//Checks if bit is set.
#define BITSHIFTCHECK(reg,shift) ((reg>>(shift))&0x1)	//Checks bit at shift level is set. Shift 0->BIT0, 1->BIT1, 2->BIT2 etc.
#define BITSET(reg,bit)		(reg |= (bit))				//Sets bit in register.
#define BITRESET(reg,bit) 	(reg &= ~(bit))				//Clears bit in register.
#define BITTOGGLE(reg,bit) 	(reg ^= (bit))				//Toggles bit in register.

uint8_t BitToIndex(uint32_t bit);
//Increments a counter that resets on n_loops -> Never reaches n_loops.
#define INCLOOP(counter, n_loops) counter=(counter+1)%(n_loops);

#define BYTE3(reg)	((reg>>24)&0xFF)
#define BYTE2(reg)	((reg>>16)&0xFF)
#define BYTE1(reg)	((reg>>8)&0xFF)
#define BYTE0(reg) 	(reg&0xFF)

typedef struct{
	uint32_t time_ms;	
	uint8_t flag;
	uint8_t auto_schedule;
	uint32_t period;
	uint32_t count;
} Schedule;
extern Schedule CurrentTime;


typedef enum SCHEDULES{
	SCH_LED_NUCLEO,
	SCH_LED_FAULT,
	SCH_LED_INDICATOR,
	SCH_LED_CAN,
	SCH_LED_WARN,
	SCH_TEMP_Init,
	SCH_TEMP_Config,
	SCH_TEMP_ADC,
	SCH_TEMP_Read,
	SCH_CELL_Init,
	SCH_CELL_ADC,
	SCH_CELL_Read,
	SCH_CAN_CELL,
	SCH_CAN_TEMP_MAIN,
	SCH_CAN_TEMP_SEG,
	SCH_CAN_FAN,
	SCH_CAN_FaultsAB,
	SCH_CAN_FaultsCD,
	SCH_BALANCE_Init,
	SCH_BALANCE_Wait,
	SCH_TEMP_INTERNAL_ADC,
	SCH_CAN_TEMP_INTERNAL,
	SCH_CAN_STOPCLK,
	SCH_TIMEOUT_Client,
	SCH_CAN_KeepAlive,
	SCH_WarnInfoHandle,
	SCHEDULE_N
}SCHEDULES;
extern Schedule ScheduleQueue[SCHEDULE_N];

#define SCHEDULE_HANDLE(task) 	if(ScheduleQueue[task].flag){ \
		ScheduleQueue[task].flag=0;


void ScheduleTask(SCHEDULES schedule, uint32_t period, uint8_t auto_re, uint32_t offset);
void ScheduleTaskStop(SCHEDULES schedule);
void TaskScheduleSoftClock();
void TaskScheduleHandler();
void TaskScheduleSoftClock_FlagSet();

typedef struct{
	uint32_t start_time_us;
	uint32_t duration_us;
	uint8_t active;
	uint8_t count;
}StopClk;

typedef enum STOPCLOCKS{
	STOPCLK_TEMP_PERIOD,
	STOPCLK_CELL_PERIOD,
	STOPCLK_TEMP_CYCLE,
	STOPCLK_CELL_CYCLE,
	STOPCLK_FAULT,
	STOPCLK_INDICATOR,
	STOPCLK_N
}STOPCLOCKS;


extern StopClk StopClocker[STOPCLK_N];

void StopclkStart(STOPCLOCKS stopclock);
uint32_t StopclkEnd(STOPCLOCKS stopclock);

//General Use Delay
void delayu(unsigned int micro);
//Specific Use (<1000us)
int __delayu(unsigned int micro);

#endif /* INC_STM32_UTILS_H_ */


