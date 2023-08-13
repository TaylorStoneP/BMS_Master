/*
 * warninfo.c
 *
 *  Created on: Jan 31, 2023
 *      Author: TheDa
 */

#include "warninfo.h"
#include "control.h"
#include "stm32_utils.h"
#include "can_utils.h"

WarnInfo WarnInfoQueue[32];
uint8_t WarnInfoQueue_Count;
uint8_t NullData[8];

static void AppendWarnInfoToQueue(uint8_t warntype){
	WarnInfoQueue[WarnInfoQueue_Count].warning = warntype;
	for(int i = 0;i<7;i++){
		WarnInfoQueue[WarnInfoQueue_Count].data[i] = 0;
	}
	WarnInfoQueue_Count++;
}

void AddWarnInfo(uint8_t warntype, uint8_t* data){
	for(int i = 0;i<7;i++){
		WarnInfoQueue[WarnInfoQueue_Count].data[i] = data[i];
	}
	AppendWarnInfoToQueue(warntype);
}

void WarnInfoHandler(){
	if(!BITCHECK(DeviceConnections, DEVCON_Client)){return;}
	if(WarnInfoQueue_Count==0){return;}

	WarnInfoQueue_Count--;
	CAN_UTIL_SetByte(CAN_TX_WarnInfo,0,WarnInfoQueue[WarnInfoQueue_Count].warning);
	CAN_UTIL_SetByte(CAN_TX_WarnInfo,1,WarnInfoQueue[WarnInfoQueue_Count].data[0]);
	CAN_UTIL_SetByte(CAN_TX_WarnInfo,2,WarnInfoQueue[WarnInfoQueue_Count].data[1]);
	CAN_UTIL_SetByte(CAN_TX_WarnInfo,3,WarnInfoQueue[WarnInfoQueue_Count].data[2]);
	CAN_UTIL_SetByte(CAN_TX_WarnInfo,4,WarnInfoQueue[WarnInfoQueue_Count].data[3]);
	CAN_UTIL_SetByte(CAN_TX_WarnInfo,5,WarnInfoQueue[WarnInfoQueue_Count].data[4]);
	CAN_UTIL_SetByte(CAN_TX_WarnInfo,6,WarnInfoQueue[WarnInfoQueue_Count].data[5]);
	CAN_UTIL_SetByte(CAN_TX_WarnInfo,7,WarnInfoQueue[WarnInfoQueue_Count].data[6]);

	CAN_UTIL_Transmit(&hcan1, CAN_TX_WarnInfo);
}
