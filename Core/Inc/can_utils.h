#ifndef CAN_UTILS_H
#define CAN_UTILS_H

#include "main.h"
#include "stm32_utils.h"

///===CONFIG===
#define CAN_UTIL_LED_EN False
#define CAN_UTIL_LED_Port LED_CAN_GPIO_Port
#define CAN_UTIL_LED_Pin LED_CAN_Pin

#define USB_EN True

typedef struct{
	uint8_t data[8];
	uint8_t length;
	uint32_t id;
}CAN_Message;

typedef enum CAN_TX{
	CAN_TX_Cells,
	CAN_TX_Temperature_Main,
	CAN_TX_Temperature_Segment,
	CAN_TX_Fan,
	CAN_TX_N
}CAN_TX;

extern CAN_Message CAN_Messages[CAN_TX_N];

#if CAN_UTIL_LED_EN == True
	#define CAN_UTIL_LED_Code(code) code
#else
	#define CAN_UTIL_LED_Code(code)
#endif

#if USB_EN == True
void CAN_UTIL_TransmitUSB(CAN_TX message);
#endif
void CAN_UTIL_Transmit(CAN_HandleTypeDef *hcan, CAN_TX message);
void CAN_UTIL_SetID(CAN_TX message, uint32_t id);
void CAN_UTIL_SetLength(CAN_TX message, uint8_t length);
void CAN_UTIL_Setup(CAN_TX message, uint32_t id, uint8_t length);
void CAN_UTIL_SetByte(CAN_TX message, uint8_t index, uint8_t data);
uint8_t CAN_UTIL_GetByte(CAN_TX message, uint8_t index);
uint16_t CAN_UTIL_ToString(CAN_TX message, uint8_t* buffer);
#endif

//void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef * hcan);
