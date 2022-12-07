/*
 * stm32_utils.c
 *
 *  Created on: Nov 1, 2022
 *      Author: Taylor Phillips
 */
#include "stm32_utils.h"

Schedule CurrentTime;
Schedule Scheduler[SCHEDULE_N];
StopClk StopClocker[STOPCLOCKS_N];

void ScheduleTask(SCHEDULES schedule, uint32_t period, uint8_t auto_re){
	if(period==0){
		Scheduler[schedule].time_ms = 0xFFFFFFFF;
		Scheduler[schedule].flag = True;
	}else if(period==0xFFFFFFFF){
		Scheduler[schedule].time_ms = 0xFFFFFFFF;
	}
	else{
		Scheduler[schedule].time_ms = CurrentTime.time_ms+period;
	}
    Scheduler[schedule].auto_schedule = auto_re;
    Scheduler[schedule].period = period;

}

void TaskScheduleSoftClock(){
    if(++CurrentTime.time_ms==0xFFFFFFFF){
    	CurrentTime.time_ms=0;
    }

    for(int task = 0; task<SCHEDULE_N; task++){
        if((Scheduler[task].time_ms == CurrentTime.time_ms)){
            Scheduler[task].flag = True;
            if(Scheduler[task].auto_schedule){
                ScheduleTask(task, Scheduler[task].period, True);
            }else{
                Scheduler[task].time_ms = 0xFFFFFFFF;
            }
        }
    }

}

void StopClkStart(STOPCLOCKS stopclock){
	StopClocker[stopclock].start_time_us = STOPCLK_TIM->CNT;
}

uint32_t StopClkEnd(STOPCLOCKS stopclock){
	if(StopClocker[stopclock].start_time_us > (STOPCLK_TIM->CNT)){
		StopClocker[stopclock].duration_us = (0xFFFFFFFF)-StopClocker[stopclock].start_time_us+(STOPCLK_TIM->CNT);
	}else{
		StopClocker[stopclock].duration_us = (STOPCLK_TIM->CNT)-StopClocker[stopclock].start_time_us;
	}
	return StopClocker[stopclock].duration_us;
}

extern TIM_HandleTypeDef DELAY_TIMER_TYPE;

void delayu(unsigned int micro)
{
	if(micro<1000)
	{
		__delayu(micro);
	}
	else if(micro%1000==0)
	{
		HAL_Delay(micro/1000);
	}
	else
	{
		HAL_Delay(micro/1000);
		__delayu(micro%1000);
	}
}
int __delayu(unsigned int micro)
{
	if(micro>999)
		return;

	DELAY_TIM->CNT=0;
	HAL_TIM_Base_Start(&DELAY_TIMER_TYPE);
	while(DELAY_TIM->CNT < micro-1)
	{}
	return 1;
}
