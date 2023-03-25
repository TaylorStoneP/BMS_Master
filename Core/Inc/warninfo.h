/*
 * warninfo.h
 *
 *  Created on: Jan 31, 2023
 *      Author: TheDa
 */

#ifndef INC_WARNINFO_H_
#define INC_WARNINFO_H_

#include "main.h"

enum WARNINFOTYPES{
	WARNINFO_Flash_Config_PwrLost,
	WARNINFO_Flash_Main_PwrLost,
	WARNINFO_Flash_Backup_PwrLost,
	WARNINFO_Flash_Config_Mismatch,
	WARNINFO_FirstConnectOnStart
};

typedef struct WarnInfo{
	uint8_t warning;
	uint8_t data[7];
}WarnInfo;

extern WarnInfo WarnInfoQueue[32];
extern uint8_t WarnInfoQueue_Count;
extern uint8_t NullData[8];

void AddWarnInfo(uint8_t warntype, uint8_t* data);

void WarnInfoHandler();



#endif /* INC_WARNINFO_H_ */
