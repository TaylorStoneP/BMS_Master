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

typedef enum PIN_UTIL{
	//OUT_x = x_pin | (x_port<<8),
	OUT_LED_FAULT 		= LED_FAULT_Pin + (((uint32_t)LED_FAULT_GPIO_Port&0xFFFF)<<16),
	OUT_LED_INDICATOR 	= LED_IND_Pin + (((uint32_t)LED_IND_GPIO_Port&0xFFFF)<<16),
	OUT_LED_CAN 		= LED_CAN_Pin + (((uint32_t)LED_CAN_GPIO_Port&0xFFFF)<<16),
	OUT_LED_WARN 		= LED_WARN_Pin + (((uint32_t)LED_WARN_GPIO_Port&0xFFFF)<<16),
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
	BIT8 = 0x80
	
};

#define BITCHECK(reg,bit) 	((reg&(bit)) == (bit))
#define BITSHIFTCHECK(reg,shift) ((reg>>(shift))&0X1)
#define BITSET(reg,bit)		(reg |= (bit))
#define BITRESET(reg,bit) 	(reg &= ~(bit))
#define BITTOGGLE(reg,bit) 	(reg ^= (bit))

typedef struct{
	uint32_t time_ms;	
	uint8_t flag;
	uint8_t auto_schedule;
	uint32_t period;
} Schedule;
extern Schedule CurrentTime;

typedef enum SCHEDULES{
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
	SCH_BALANCE_Init,
	SCHEDULE_N
}SCHEDULES;
extern Schedule Scheduler[SCHEDULE_N];

#define SCHEDULE_HANDLE(task) 	if(Scheduler[task].flag){ \
									Scheduler[task].flag=0;


void ScheduleTask(SCHEDULES schedule, uint32_t period, uint8_t auto_re);
void TaskScheduleSoftClock();
void TaskScheduleHandler();

typedef struct{
	uint32_t start_time_us;
	uint32_t duration_us;
}StopClk;

typedef enum STOPCLOCKS{
	STOPCLK_FAULT,
	STOPCLK_INDICATOR,
	STOPCLOCKS_N
}STOPCLOCKS;


extern StopClk StopClocker[STOPCLOCKS_N];

void StopClkStart(STOPCLOCKS stopclock);
uint32_t StopClkEnd(STOPCLOCKS stopclock);

//General Use Delay
void delayu(unsigned int micro);
//Specific Use (<1000us)
int __delayu(unsigned int micro);

#endif /* INC_STM32_UTILS_H_ */


