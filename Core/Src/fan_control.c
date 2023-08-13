/*
 * fan_control.c
 *
 *  Created on: Jun 14, 2023
 *      Author: TheDa
 */
#include "fan_control.h"
#include "config_loader.h"

FAN_HANDLE hfans[FAN_N];
PID_Handle hpid_fans;

void FAN_SetPWM(uint16_t value){
	if(value>=100){
		value=99;
		BMS_Data.fan_pwm = 100;
	}else{
		BMS_Data.fan_pwm = value;
	}
	TIM3->CCR3 = 99-value;
}

void Fan_TachUpdate(FAN_HANDLE* fan){
	fan->tach_count += 1;
}

uint16_t Fan_GetSpeed(FAN_HANDLE* fan){
	uint16_t out = fan->tach_count/2 * (1000/TACH_PERIOD) * 60;
	fan->tach_count = 0;
	return out;
}

uint16_t FAN_SpeedFromTime(uint16_t us, uint16_t count){
	if(us==0){
		return 0;
	}
	float period = (float)us/count;
	float pulses_per_second = 1000000.0f/period;
	float revs_per_second = pulses_per_second/2.0f;
	return (uint16_t)(revs_per_second*60);
	//return (60*6400000)/((us/count)*2);
}

uint8_t fan_tach_n;
void FAN_StartTach(FAN_HANDLE* fan){
	fan->tach_finished = False;
	fan->tach_started = False;
	fan->tach_time_start = 0;
	fan->tach_time_end = 0;
	fan->tach_count = 0;
}
void FAN_TachActive(FAN_HANDLE* fan){
	fan->tach_finished = False;
	fan->tach_started = True;
	fan->tach_time_start = TACH_TIM->CNT;
}
void FAN_EndTach(FAN_HANDLE* fan){
	fan->tach_count++;
	if(fan->tach_count>=TACH_COUNT){
		fan->tach_finished = True;
		fan->tach_started = False;
		fan->tach_time_end = TACH_TIM->CNT;
		fan_tach_n++;
		if(fan_tach_n==FAN_N){
			HAL_TIM_Base_Stop(&TACH_TIMER_TYPE);
			HAL_NVIC_DisableIRQ(EXTI4_IRQn);
			HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
		}
	}
}

uint8_t FAN_TempToDC(uint32_t temperature){
if(temperature<CFG_Main[CFGID_FanTemperatureDC10]){
		return 0;
	}else if(temperature<CFG_Main[CFGID_FanTemperatureDC20]){
		return 10;
	}else if(temperature<CFG_Main[CFGID_FanTemperatureDC30]){
		return 20;
	}else if(temperature<CFG_Main[CFGID_FanTemperatureDC40]){
		return 30;
	}else if(temperature<CFG_Main[CFGID_FanTemperatureDC50]){
		return 40;
	}else if(temperature<CFG_Main[CFGID_FanTemperatureDC60]){
		return 50;
	}else if(temperature<CFG_Main[CFGID_FanTemperatureDC70]){
		return 60;
	}else if(temperature<CFG_Main[CFGID_FanTemperatureDC80]){
		return 70;
	}else if(temperature<CFG_Main[CFGID_FanTemperatureDC90]){
		return 80;
	}else if(temperature<CFG_Main[CFGID_FanTemperatureDC100]){
		return 90;
	}else{
		return 100;
	}
}

uint8_t FAN_LerpTempToDC(uint32_t temperature){
	uint32_t low_temp;
	uint32_t high_temp;
	uint8_t low_dc;
	uint8_t high_dc;
	if(temperature<CFG_Main[CFGID_FanTemperatureDC0]){
		return 0;
	}else if(temperature<CFG_Main[CFGID_FanTemperatureDC10]){
		low_temp = CFG_Main[CFGID_FanTemperatureDC0];
		high_temp = CFG_Main[CFGID_FanTemperatureDC10];
		low_dc = 0;
		high_dc = 10;
	}else if(temperature<CFG_Main[CFGID_FanTemperatureDC20]){
		low_temp = CFG_Main[CFGID_FanTemperatureDC10];
		high_temp = CFG_Main[CFGID_FanTemperatureDC20];
		low_dc = 10;
		high_dc = 20;
	}else if(temperature<CFG_Main[CFGID_FanTemperatureDC30]){
		low_temp = CFG_Main[CFGID_FanTemperatureDC20];
		high_temp = CFG_Main[CFGID_FanTemperatureDC30];
		low_dc = 20;
		high_dc = 30;
	}else if(temperature<CFG_Main[CFGID_FanTemperatureDC40]){
		low_temp = CFG_Main[CFGID_FanTemperatureDC30];
		high_temp = CFG_Main[CFGID_FanTemperatureDC40];
		low_dc = 30;
		high_dc = 40;
	}else if(temperature<CFG_Main[CFGID_FanTemperatureDC50]){
		low_temp = CFG_Main[CFGID_FanTemperatureDC40];
		high_temp = CFG_Main[CFGID_FanTemperatureDC50];
		low_dc = 40;
		high_dc = 50;
	}else if(temperature<CFG_Main[CFGID_FanTemperatureDC60]){
		low_temp = CFG_Main[CFGID_FanTemperatureDC50];
		high_temp = CFG_Main[CFGID_FanTemperatureDC60];
		low_dc = 50;
		high_dc = 60;
	}else if(temperature<CFG_Main[CFGID_FanTemperatureDC70]){
		low_temp = CFG_Main[CFGID_FanTemperatureDC60];
		high_temp = CFG_Main[CFGID_FanTemperatureDC70];
		low_dc = 60;
		high_dc = 70;
	}else if(temperature<CFG_Main[CFGID_FanTemperatureDC80]){
		low_temp = CFG_Main[CFGID_FanTemperatureDC70];
		high_temp = CFG_Main[CFGID_FanTemperatureDC80];
		low_dc = 70;
		high_dc = 80;
	}else if(temperature<CFG_Main[CFGID_FanTemperatureDC90]){
		low_temp = CFG_Main[CFGID_FanTemperatureDC80];
		high_temp = CFG_Main[CFGID_FanTemperatureDC90];
		low_dc = 80;
		high_dc = 90;
	}else if(temperature<CFG_Main[CFGID_FanTemperatureDC100]){
		low_temp = CFG_Main[CFGID_FanTemperatureDC90];
		high_temp = CFG_Main[CFGID_FanTemperatureDC100];
		low_dc = 90;
		high_dc = 100;
	}else{
		return 100;
	}
	return clamp(lerp(low_dc, high_dc, normalise(low_temp, high_temp, temperature, 100), 100),0,100);
}
void FAN_SetDC(uint8_t dc){
	//Converts duty cycle to pwm timer ccrx register count.
	FAN_SetPWM(dc);
}


