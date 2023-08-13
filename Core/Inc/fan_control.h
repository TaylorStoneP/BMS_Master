/*
 * fan_control.h
 *
 *  Created on: Jun 14, 2023
 *      Author: TheDa
 */

#ifndef INC_FAN_CONTROL_H_
#define INC_FAN_CONTROL_H_

#include "main.h"
#include "control.h"
#include "mathops.h"

#define TACH_PERIOD 250
#define TACH_TIMER_TYPE htim5
#define TACH_TIM TIM5
#define TACH_COUNT 2

extern TIM_HandleTypeDef TACH_TIMER_TYPE;

enum FAN_N{
	FAN_IN_1,
	FAN_IN_2,
	FAN_IN_3,
	FAN_IN_4,
	FAN_N
};
typedef struct FAN_HANDLE{
	uint16_t tach_count;
	uint16_t speed;
	uint8_t tach_started;
	uint8_t tach_finished;
	uint32_t tach_time_start;
	uint32_t tach_time_end;
	MOVING_AVERAGE ave_speed;
}FAN_HANDLE;

extern FAN_HANDLE hfans[FAN_N];
extern PID_Handle hpid_fans;
extern uint8_t fan_tach_n;

void FAN_SetPWM(uint16_t value);
void Fan_TachUpdate(FAN_HANDLE* fan);
uint16_t Fan_GetSpeed(FAN_HANDLE* fan);

uint16_t FAN_SpeedFromTime(uint16_t us, uint16_t count);
void FAN_StartTach(FAN_HANDLE* fan);
void FAN_TachActive(FAN_HANDLE* fan);
void FAN_EndTach(FAN_HANDLE* fan);

uint8_t FAN_TempToDC(uint32_t temperature);
uint8_t FAN_LerpTempToDC(uint32_t temperature);
void FAN_SetDC(uint8_t dc);

#endif /* INC_FAN_CONTROL_H_ */
