/*
 * stm32_utils.c
 *
 *  Created on: Nov 1, 2022
 *      Author: Taylor Phillips
 */
#include "stm32_utils.h"

Schedule CurrentTime;
Schedule ScheduleQueue[SCHEDULE_N];
StopClk StopClocker[STOPCLK_N];

uint8_t TaskSchedulerSoftClock_Flag;

uint8_t BitToIndex(uint32_t bit){
	switch(bit){
	case BIT1:
		return 0;
	case BIT2:
		return 1;
	case BIT3:
		return 2;
	case BIT4:
		return 3;
	case BIT5:
		return 4;
	case BIT6:
		return 5;
	case BIT7:
		return 6;
	case BIT8:
		return 7;
	case BIT9:
		return 8;
	case BIT10:
		return 9;
	case BIT11:
		return 10;
	case BIT12:
		return 11;
	case BIT13:
		return 12;
	case BIT14:
		return 13;
	case BIT15:
		return 14;
	case BIT16:
		return 15;
	}
	return -1;
}

void ScheduleTask(SCHEDULES schedule, uint32_t period, uint8_t auto_re, uint32_t offset){
	if(period==0){
		ScheduleQueue[schedule].time_ms = 0xFFFFFFFF;
		ScheduleQueue[schedule].flag = True;
	}else if(period==0xFFFFFFFF){
		ScheduleQueue[schedule].time_ms = 0xFFFFFFFF;
	}
	else{
		ScheduleQueue[schedule].time_ms = (CurrentTime.time_ms+period+offset)%0xFFFFFFFF;
	}
    ScheduleQueue[schedule].auto_schedule = auto_re;
    ScheduleQueue[schedule].period = period;

}

void ScheduleTaskStop(SCHEDULES schedule){
	ScheduleTask(schedule, 0xFFFFFFFF, False, 0);
}

void TaskScheduleSoftClock(){
	if(!TaskSchedulerSoftClock_Flag){
		return;
	}
	TaskSchedulerSoftClock_Flag = 0;

    if(++CurrentTime.time_ms==0xFFFFFFFF){
    	CurrentTime.time_ms=0;
    }

    for(int task = 0; task<SCHEDULE_N; task++){
        if((ScheduleQueue[task].time_ms == CurrentTime.time_ms)){
            ScheduleQueue[task].flag = True;
            ScheduleQueue[task].count++;
            if(ScheduleQueue[task].auto_schedule){
                ScheduleTask(task, ScheduleQueue[task].period, True,0);
            }else{
                ScheduleQueue[task].time_ms = 0xFFFFFFFF;
            }
        }
    }
}

void TaskScheduleSoftClock_FlagSet(){
	TaskSchedulerSoftClock_Flag = 1;
}

void StopclkStart(STOPCLOCKS stopclock){
	StopClocker[stopclock].start_time_us = STOPCLK_TIM->CNT;
	StopClocker[stopclock].count++;
	StopClocker[stopclock].active = True;
}

uint32_t StopclkEnd(STOPCLOCKS stopclock){
	if(StopClocker[stopclock].active == True){
		StopClocker[stopclock].active = False;
		if(StopClocker[stopclock].start_time_us > (STOPCLK_TIM->CNT)){
			StopClocker[stopclock].duration_us = (0xFFFFFFFF)-StopClocker[stopclock].start_time_us+(STOPCLK_TIM->CNT);
		}else{
			StopClocker[stopclock].duration_us = (STOPCLK_TIM->CNT)-StopClocker[stopclock].start_time_us;
		}
		return StopClocker[stopclock].duration_us;
	}else{
		return 0;
	}
}


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

uint8_t prim;
void DisableIRQ(){
	prim = __get_PRIMASK();
	__disable_irq();
}
void EnableIRQ(){
	if(!prim){
		__enable_irq();
	}
}
